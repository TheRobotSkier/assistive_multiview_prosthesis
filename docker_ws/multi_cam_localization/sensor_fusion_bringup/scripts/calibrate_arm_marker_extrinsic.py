#!/usr/bin/env python3

"""Calibrate the fixed transform from the arm D435i camera to an arm marker.

This is an offline calibration path for dynamic arm-mounted markers. It never
publishes marker observations to OpenVINS and never treats the dynamic marker as
a fixed marker_map landmark.
"""

from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Iterable, Optional

import cv2
import numpy as np
import yaml

from aruco_marker_pose_node import (
    R_to_rotvec,
    T_inv,
    is_finite_array,
    load_kalibr_imucam,
    marker_object_points,
    marker_quality_metrics,
    rotvec_to_R,
    stamp_to_sec,
)


RESIDUAL_ORDER = ["x", "y", "z", "roll", "pitch", "yaw"]


@dataclass
class QualityConfig:
    min_marker_area_px2: float = 800.0
    max_marker_distance_m: float = 2.0
    max_reprojection_error_px: float = 3.0
    border_margin_px: float = 8.0
    min_corner_angle_deg: float = 25.0
    max_corner_angle_deg: float = 155.0
    min_border_geometry_score: float = 0.35

    @classmethod
    def from_marker_config(cls, marker_config: dict[str, Any]) -> "QualityConfig":
        cfg = marker_config.get("quality", {})
        return cls(
            min_marker_area_px2=float(cfg.get("min_marker_area_px2", 800.0)),
            max_marker_distance_m=float(cfg.get("max_marker_distance_m", 2.0)),
            max_reprojection_error_px=float(cfg.get("max_reprojection_error_px", 3.0)),
            border_margin_px=float(cfg.get("border_margin_px", 8.0)),
            min_corner_angle_deg=float(cfg.get("min_corner_angle_deg", 25.0)),
            max_corner_angle_deg=float(cfg.get("max_corner_angle_deg", 155.0)),
            min_border_geometry_score=float(cfg.get("min_border_geometry_score", 0.35)),
        )


@dataclass
class CameraConfig:
    image_topic: str
    camera_info_topic: str
    camera_frame: str
    imu_frame: str
    K: np.ndarray
    D: np.ndarray
    T_cam_imu: np.ndarray
    marker_config: dict[str, Any]
    quality: QualityConfig


@dataclass
class MarkerDetection:
    marker_id: int
    stamp: float
    T_cam_marker: np.ndarray
    reprojection_error_px: float
    distance_m: float
    view_angle_deg: float
    area_px2: float
    side_mean_px: float
    geometry_score: float


@dataclass
class CameraMapSample:
    stamp: float
    T_map_cam: np.ndarray
    detection: MarkerDetection


@dataclass
class CalibrationSample:
    stamp: float
    T_armcam_marker: np.ndarray
    head_id0: MarkerDetection
    head_dynamic: MarkerDetection
    arm_before: CameraMapSample
    arm_after: CameraMapSample
    arm_alpha: float
    arm_before_dt: float
    arm_after_dt: float


@dataclass
class RobustEstimate:
    T_estimate: np.ndarray
    medoid_index: int
    inlier_indices: list[int]
    outlier_indices: list[int]
    residuals: np.ndarray
    residual_covariance: np.ndarray
    robust_diag_covariance: np.ndarray
    estimate_covariance: np.ndarray
    translation_norms: np.ndarray
    rotation_angles_deg: np.ndarray
    translation_gate_m: float
    rotation_gate_deg: float


def import_rosbag_dependencies():
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 bag dependencies are not available. Run this script inside the "
            "Jazzy realsense_camera Docker container after sourcing the overlay."
        ) from exc
    return deserialize_message, ConverterOptions, SequentialReader, StorageOptions, get_message


def read_bag_messages(bag_dir: Path, topics: set[str]):
    deserialize_message, ConverterOptions, SequentialReader, StorageOptions, get_message = import_rosbag_dependencies()
    storage_id = read_bag_storage_id(bag_dir)
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    missing = sorted(topics - set(topic_types))
    if missing:
        raise RuntimeError(f"Bag is missing required topics: {missing}")
    message_types = {topic: get_message(topic_types[topic]) for topic in topics}
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in topics:
            continue
        yield topic, deserialize_message(serialized, message_types[topic]), int(timestamp_ns)


def read_bag_storage_id(bag_dir: Path) -> str:
    metadata_path = bag_dir / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    return str(metadata["rosbag2_bagfile_information"].get("storage_identifier", "mcap"))


def camera_info_topic_for_image(image_topic: str) -> str:
    if image_topic.endswith("/image_raw"):
        return image_topic[: -len("/image_raw")] + "/camera_info"
    return str(Path(image_topic).parent / "camera_info")


def resolve_path(path_text: str, config_path: Optional[Path] = None) -> Path:
    path = Path(path_text)
    if path.exists():
        return path
    if not path.is_absolute() and config_path is not None:
        candidate = (config_path.parent / path).resolve()
        if candidate.exists():
            return candidate
    miahand_prefix = "/miahand_ws/src/"
    if path_text.startswith(miahand_prefix):
        repo_candidate = Path.cwd() / path_text[len(miahand_prefix) :]
        if repo_candidate.exists():
            return repo_candidate
    return path


def load_marker_config(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def load_camera_config(config_path: Path) -> CameraConfig:
    marker_config = load_marker_config(config_path)
    topics = marker_config.get("topics", {})
    frames = marker_config.get("frames", {})
    image_topic = str(topics.get("image"))
    if not image_topic:
        raise RuntimeError(f"{config_path} does not define topics.image")
    calib_path = resolve_path(str(marker_config["calibration"]["kalibr_imucam_chain"]), config_path)
    K, D, T_cam_imu, _timeshift = load_kalibr_imucam(str(calib_path))
    return CameraConfig(
        image_topic=image_topic,
        camera_info_topic=camera_info_topic_for_image(image_topic),
        camera_frame=str(frames.get("detected_camera_frame", frames.get("camera_frame", ""))),
        imu_frame=str(frames.get("imu_frame", "")),
        K=K,
        D=D,
        T_cam_imu=T_cam_imu,
        marker_config=marker_config,
        quality=QualityConfig.from_marker_config(marker_config),
    )


def image_msg_to_gray(msg: Any) -> Optional[np.ndarray]:
    enc = msg.encoding.lower()
    h = msg.height
    w = msg.width
    step = msg.step
    data = np.frombuffer(msg.data, dtype=np.uint8)
    rows = data.reshape((h, step))

    if enc in ("rgb8", "bgr8"):
        img = rows[:, : w * 3].reshape((h, w, 3))
        img = np.ascontiguousarray(img)
        if enc == "rgb8":
            return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if enc in ("mono8", "8uc1"):
        return np.ascontiguousarray(rows[:, :w])

    return None


def compute_reprojection_error(
    obj_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(obj_points, rvec, tvec, K, D)
    projected = projected.reshape(4, 2)
    residuals = projected - image_points
    return float(math.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))


def check_corner_geometry(
    image_points: np.ndarray,
    image_width: int,
    image_height: int,
    quality: QualityConfig,
) -> tuple[bool, float, str]:
    if not cv2.isContourConvex(image_points.reshape(-1, 1, 2)):
        return False, 0.0, "marker_corners_not_convex"

    xs = image_points[:, 0]
    ys = image_points[:, 1]
    near_border = (
        np.min(xs) < quality.border_margin_px
        or np.max(xs) > image_width - quality.border_margin_px
        or np.min(ys) < quality.border_margin_px
        or np.max(ys) > image_height - quality.border_margin_px
    )

    angles = []
    side_lengths = []
    for i in range(4):
        p_prev = image_points[(i - 1) % 4]
        p = image_points[i]
        p_next = image_points[(i + 1) % 4]
        v1 = p_prev - p
        v2 = p_next - p
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 <= 1e-6 or n2 <= 1e-6:
            return False, 0.0, "marker_degenerate_corner"
        cos_angle = float(np.dot(v1, v2) / (n1 * n2))
        cos_angle = max(-1.0, min(1.0, cos_angle))
        angles.append(math.degrees(math.acos(cos_angle)))
        side_lengths.append(float(np.linalg.norm(p_next - p)))

    min_angle = min(angles)
    max_angle = max(angles)
    if min_angle < quality.min_corner_angle_deg or max_angle > quality.max_corner_angle_deg:
        return False, 0.0, "marker_corner_angles_bad"

    min_side = min(side_lengths)
    max_side = max(side_lengths)
    side_ratio = min_side / max(max_side, 1e-6)
    angle_spread = max_angle - min_angle
    geometry_score = max(0.0, min(1.0, 0.5 * side_ratio + 0.5 * (1.0 - angle_spread / 180.0)))
    if near_border and geometry_score < quality.min_border_geometry_score:
        return False, geometry_score, "marker_near_border_poor_geometry"
    return True, geometry_score, ""


def detect_marker_pose(
    marker_id: int,
    image_points: np.ndarray,
    marker_size_m: float,
    stamp: float,
    K: np.ndarray,
    D: np.ndarray,
    quality: QualityConfig,
    image_width: int,
    image_height: int,
) -> Optional[MarkerDetection]:
    if not is_finite_array(image_points):
        return None
    area = abs(float(cv2.contourArea(image_points)))
    if area < quality.min_marker_area_px2:
        return None
    geometry_ok, geometry_score, _reason = check_corner_geometry(image_points, image_width, image_height, quality)
    if not geometry_ok:
        return None

    obj_points = marker_object_points(marker_size_m)
    ok, rvec, tvec = cv2.solvePnP(obj_points, image_points, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None

    R_cam_marker, _ = cv2.Rodrigues(rvec)
    T_cam_marker = np.eye(4)
    T_cam_marker[:3, :3] = R_cam_marker
    T_cam_marker[:3, 3] = tvec.reshape(3)
    if not is_finite_array(T_cam_marker):
        return None

    distance_m = float(np.linalg.norm(T_cam_marker[:3, 3]))
    if distance_m > quality.max_marker_distance_m:
        return None

    reprojection_error_px = compute_reprojection_error(obj_points, image_points, rvec, tvec, K, D)
    if reprojection_error_px > quality.max_reprojection_error_px:
        return None

    metrics = marker_quality_metrics(image_points, T_cam_marker, reprojection_error_px, area_px2=area)
    return MarkerDetection(
        marker_id=int(marker_id),
        stamp=float(stamp),
        T_cam_marker=T_cam_marker,
        reprojection_error_px=metrics.reprojection_error_px,
        distance_m=metrics.distance_m,
        view_angle_deg=metrics.view_angle_deg,
        area_px2=metrics.area_px2,
        side_mean_px=metrics.side_mean_px,
        geometry_score=geometry_score,
    )


def detect_required_markers(
    msg: Any,
    camera: CameraConfig,
    aruco_dict: Any,
    marker_sizes: dict[int, float],
) -> dict[int, MarkerDetection]:
    gray = image_msg_to_gray(msg)
    if gray is None:
        return {}
    corners, ids, _rejected = cv2.aruco.detectMarkers(gray, aruco_dict)
    if ids is None:
        return {}
    id_counts = {int(marker_id_arr[0]): 0 for marker_id_arr in ids}
    for marker_id_arr in ids:
        id_counts[int(marker_id_arr[0])] += 1

    out: dict[int, MarkerDetection] = {}
    stamp = stamp_to_sec(msg.header.stamp)
    for idx, marker_id_arr in enumerate(ids):
        marker_id = int(marker_id_arr[0])
        if marker_id not in marker_sizes or id_counts[marker_id] != 1:
            continue
        detection = detect_marker_pose(
            marker_id=marker_id,
            image_points=corners[idx].reshape(4, 2).astype(np.float32),
            marker_size_m=marker_sizes[marker_id],
            stamp=stamp,
            K=camera.K,
            D=camera.D,
            quality=camera.quality,
            image_width=gray.shape[1],
            image_height=gray.shape[0],
        )
        if detection is not None:
            out[marker_id] = detection
    return out


def fixed_marker_size(config: dict[str, Any], marker_id: int) -> float:
    markers = config.get("markers", {})
    marker_cfg = markers.get(marker_id, markers.get(str(marker_id)))
    if marker_cfg is None:
        raise RuntimeError(f"Fixed marker ID {marker_id} is missing from marker config")
    return float(marker_cfg.get("size_m", config.get("aruco", {}).get("default_marker_size_m", 0.100)))


def dynamic_marker_config(config: dict[str, Any], marker_id: int) -> dict[str, Any]:
    markers = config.get("dynamic_markers", {})
    marker_cfg = markers.get(marker_id, markers.get(str(marker_id)))
    if marker_cfg is None:
        raise RuntimeError(f"Dynamic marker ID {marker_id} is missing from marker config")
    return marker_cfg


def fixed_T_map_marker(config: dict[str, Any], marker_id: int) -> np.ndarray:
    markers = config.get("markers", {})
    marker_cfg = markers.get(marker_id, markers.get(str(marker_id)))
    if marker_cfg is None:
        raise RuntimeError(f"Fixed marker ID {marker_id} is missing from marker config")
    return np.array(marker_cfg["T_map_marker"], dtype=float)


def compute_camera_map_sample(detection: MarkerDetection, T_map_marker: np.ndarray) -> CameraMapSample:
    T_map_cam = T_map_marker @ T_inv(detection.T_cam_marker)
    return CameraMapSample(stamp=detection.stamp, T_map_cam=T_map_cam, detection=detection)


def compute_armcam_marker_sample(
    T_map_headcam: np.ndarray,
    T_headcam_marker: np.ndarray,
    T_map_armcam: np.ndarray,
) -> np.ndarray:
    return T_inv(T_map_armcam) @ T_map_headcam @ T_headcam_marker


def se3_exp(delta: Iterable[float]) -> np.ndarray:
    delta = np.asarray(list(delta), dtype=float).reshape(6)
    T = np.eye(4)
    T[:3, 3] = delta[:3]
    T[:3, :3] = rotvec_to_R(delta[3:])
    return T


def se3_residual(T_ref: np.ndarray, T_sample: np.ndarray) -> np.ndarray:
    delta = T_inv(T_ref) @ T_sample
    return np.concatenate([delta[:3, 3], R_to_rotvec(delta[:3, :3])])


def interpolate_transform(T_a: np.ndarray, T_b: np.ndarray, alpha: float) -> np.ndarray:
    alpha = max(0.0, min(1.0, float(alpha)))
    T = np.eye(4)
    T[:3, 3] = (1.0 - alpha) * T_a[:3, 3] + alpha * T_b[:3, 3]
    R_delta = T_a[:3, :3].T @ T_b[:3, :3]
    T[:3, :3] = T_a[:3, :3] @ rotvec_to_R(alpha * R_to_rotvec(R_delta))
    return T


def interpolate_arm_sample(
    arm_samples: list[CameraMapSample],
    stamp: float,
    tolerance_s: float,
) -> Optional[tuple[np.ndarray, CameraMapSample, CameraMapSample, float, float, float]]:
    stamps = [sample.stamp for sample in arm_samples]
    idx = bisect.bisect_left(stamps, stamp)
    if idx < len(arm_samples) and abs(arm_samples[idx].stamp - stamp) <= 1e-9:
        sample = arm_samples[idx]
        return sample.T_map_cam, sample, sample, 0.0, 0.0, 0.0
    if idx == 0 or idx >= len(arm_samples):
        return None
    before = arm_samples[idx - 1]
    after = arm_samples[idx]
    before_dt = stamp - before.stamp
    after_dt = after.stamp - stamp
    if before_dt < 0.0 or after_dt < 0.0 or before_dt > tolerance_s or after_dt > tolerance_s:
        return None
    span = max(after.stamp - before.stamp, 1e-12)
    alpha = before_dt / span
    return interpolate_transform(before.T_map_cam, after.T_map_cam, alpha), before, after, alpha, before_dt, after_dt


def median_or_default(values: Iterable[float], default: float = 0.0) -> float:
    arr = np.asarray([float(v) for v in values if math.isfinite(float(v))], dtype=float)
    if arr.size == 0:
        return default
    return float(np.median(arr))


def percentile_or_default(values: Iterable[float], q: float, default: float = 0.0) -> float:
    arr = np.asarray([float(v) for v in values if math.isfinite(float(v))], dtype=float)
    if arr.size == 0:
        return default
    return float(np.percentile(arr, q))


def rms_or_default(values: Iterable[float], default: float = 0.0) -> float:
    arr = np.asarray([float(v) for v in values if math.isfinite(float(v))], dtype=float)
    if arr.size == 0:
        return default
    return float(math.sqrt(np.mean(arr * arr)))


def mad_std(values: Iterable[float], default: float = 0.0) -> float:
    arr = np.asarray([float(v) for v in values if math.isfinite(float(v))], dtype=float)
    if arr.size == 0:
        return default
    med = float(np.median(arr))
    return float(1.4826 * np.median(np.abs(arr - med)))


def choose_se3_medoid(transforms: list[np.ndarray]) -> int:
    if not transforms:
        raise RuntimeError("Cannot choose medoid from zero transforms")
    best_idx = 0
    best_score = float("inf")
    for i, T_i in enumerate(transforms):
        distances = []
        for T_j in transforms:
            residual = se3_residual(T_i, T_j)
            trans = float(np.linalg.norm(residual[:3])) / 0.03
            rot = math.degrees(float(np.linalg.norm(residual[3:]))) / 5.0
            distances.append(math.sqrt(trans * trans + rot * rot))
        score = median_or_default(distances, default=float("inf"))
        if score < best_score:
            best_idx = i
            best_score = score
    return best_idx


def iterative_se3_mean(transforms: list[np.ndarray], initial: np.ndarray, max_iters: int = 20) -> np.ndarray:
    T_est = initial.copy()
    for _iter in range(max_iters):
        residuals = np.array([se3_residual(T_est, T_sample) for T_sample in transforms], dtype=float)
        step = np.mean(residuals, axis=0)
        if np.linalg.norm(step[:3]) < 1e-8 and np.linalg.norm(step[3:]) < 1e-8:
            break
        T_est = T_est @ se3_exp(step)
    return T_est


def robust_se3_estimate(
    transforms: list[np.ndarray],
    translation_floor_m: float = 0.03,
    rotation_floor_deg: float = 5.0,
) -> RobustEstimate:
    if not transforms:
        raise RuntimeError("No transform samples are available for robust estimation")
    medoid_idx = choose_se3_medoid(transforms)
    medoid = transforms[medoid_idx]
    medoid_residuals = np.array([se3_residual(medoid, T_sample) for T_sample in transforms], dtype=float)
    medoid_trans = np.linalg.norm(medoid_residuals[:, :3], axis=1)
    medoid_rot_deg = np.degrees(np.linalg.norm(medoid_residuals[:, 3:], axis=1))
    translation_gate_m = max(float(translation_floor_m), 3.0 * mad_std(medoid_trans, default=0.0))
    rotation_gate_deg = max(float(rotation_floor_deg), 3.0 * mad_std(medoid_rot_deg, default=0.0))
    inliers = [
        idx
        for idx, (trans_norm, rot_deg) in enumerate(zip(medoid_trans, medoid_rot_deg))
        if trans_norm <= translation_gate_m and rot_deg <= rotation_gate_deg
    ]
    if not inliers:
        inliers = [medoid_idx]
    outliers = [idx for idx in range(len(transforms)) if idx not in set(inliers)]
    inlier_transforms = [transforms[idx] for idx in inliers]
    T_est = iterative_se3_mean(inlier_transforms, medoid)
    residuals = np.array([se3_residual(T_est, T_sample) for T_sample in inlier_transforms], dtype=float)
    if len(inlier_transforms) > 1:
        residual_cov = np.cov(residuals, rowvar=False)
    else:
        residual_cov = np.zeros((6, 6), dtype=float)
    robust_diag = np.diag([mad_std(residuals[:, axis]) ** 2 for axis in range(6)])
    estimate_cov = residual_cov / max(len(inlier_transforms), 1)
    translation_norms = np.linalg.norm(residuals[:, :3], axis=1)
    rotation_angles_deg = np.degrees(np.linalg.norm(residuals[:, 3:], axis=1))
    return RobustEstimate(
        T_estimate=T_est,
        medoid_index=medoid_idx,
        inlier_indices=inliers,
        outlier_indices=outliers,
        residuals=residuals,
        residual_covariance=residual_cov,
        robust_diag_covariance=robust_diag,
        estimate_covariance=estimate_cov,
        translation_norms=translation_norms,
        rotation_angles_deg=rotation_angles_deg,
        translation_gate_m=translation_gate_m,
        rotation_gate_deg=rotation_gate_deg,
    )


def build_calibration_samples(
    head_samples: list[tuple[MarkerDetection, MarkerDetection, np.ndarray]],
    arm_samples: list[CameraMapSample],
    sync_tolerance_s: float,
) -> list[CalibrationSample]:
    out: list[CalibrationSample] = []
    for head_id0, head_dynamic, T_map_headcam in head_samples:
        interp = interpolate_arm_sample(arm_samples, head_id0.stamp, sync_tolerance_s)
        if interp is None:
            continue
        T_map_armcam, arm_before, arm_after, alpha, before_dt, after_dt = interp
        out.append(
            CalibrationSample(
                stamp=head_id0.stamp,
                T_armcam_marker=compute_armcam_marker_sample(
                    T_map_headcam,
                    head_dynamic.T_cam_marker,
                    T_map_armcam,
                ),
                head_id0=head_id0,
                head_dynamic=head_dynamic,
                arm_before=arm_before,
                arm_after=arm_after,
                arm_alpha=alpha,
                arm_before_dt=before_dt,
                arm_after_dt=after_dt,
            )
        )
    return out


def matrix_to_yaml_rows(T: np.ndarray) -> list[list[float]]:
    return [[round(float(value), 10) for value in row] for row in np.asarray(T, dtype=float)]


def to_builtin(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return to_builtin(value.tolist())
    if isinstance(value, (list, tuple)):
        return [to_builtin(item) for item in value]
    if isinstance(value, dict):
        return {key: to_builtin(item) for key, item in value.items()}
    if isinstance(value, (np.floating, float)):
        out = float(value)
        return out if math.isfinite(out) else None
    if isinstance(value, (np.integer, int)):
        return int(value)
    return value


def summarize_estimate(
    estimate: RobustEstimate,
    samples: list[CalibrationSample],
    source_bag: Path,
    marker_id: int,
    marker_frame: str,
    marker_size_m: float,
    parent_camera_frame: str,
    parent_imu_frame: str,
    T_armcam_imu: np.ndarray,
) -> dict[str, Any]:
    T_armimu_marker = T_inv(T_armcam_imu) @ estimate.T_estimate
    sync_before = [sample.arm_before_dt for sample in samples]
    sync_after = [sample.arm_after_dt for sample in samples]
    sync_max = [max(sample.arm_before_dt, sample.arm_after_dt) for sample in samples]
    return {
        "arm_mounted_markers": {
            int(marker_id): {
                "marker_frame": marker_frame,
                "marker_size_m": float(marker_size_m),
                "parent_camera_frame": parent_camera_frame,
                "parent_imu_frame": parent_imu_frame,
                "T_armcam_marker": matrix_to_yaml_rows(estimate.T_estimate),
                "T_armimu_marker": matrix_to_yaml_rows(T_armimu_marker),
                "source_bag": str(source_bag),
                "estimate": {
                    "method": "robust_se3_mean_after_medoid_outlier_rejection",
                    "sample_count": len(samples),
                    "inlier_count": len(estimate.inlier_indices),
                    "outlier_count": len(estimate.outlier_indices),
                    "medoid_sample_index": int(estimate.medoid_index),
                    "translation_gate_m": float(estimate.translation_gate_m),
                    "rotation_gate_deg": float(estimate.rotation_gate_deg),
                    "residual_covariance_se3": matrix_to_yaml_rows(estimate.residual_covariance),
                    "robust_diag_covariance_se3": matrix_to_yaml_rows(estimate.robust_diag_covariance),
                    "estimate_covariance_se3": matrix_to_yaml_rows(estimate.estimate_covariance),
                    "residual_order": RESIDUAL_ORDER,
                    "median_translation_residual_m": median_or_default(estimate.translation_norms),
                    "rms_translation_residual_m": rms_or_default(estimate.translation_norms),
                    "p95_translation_residual_m": percentile_or_default(estimate.translation_norms, 95.0),
                    "median_rotation_residual_deg": median_or_default(estimate.rotation_angles_deg),
                    "rms_rotation_residual_deg": rms_or_default(estimate.rotation_angles_deg),
                    "p95_rotation_residual_deg": percentile_or_default(estimate.rotation_angles_deg, 95.0),
                    "sync": {
                        "max_bracket_dt_median_s": median_or_default(sync_max),
                        "max_bracket_dt_p95_s": percentile_or_default(sync_max, 95.0),
                        "before_dt_median_s": median_or_default(sync_before),
                        "after_dt_median_s": median_or_default(sync_after),
                    },
                },
            }
        }
    }


def compare_camera_info(camera_name: str, camera: CameraConfig, camera_info_msg: Any, tolerance_px: float) -> list[str]:
    if camera_info_msg is None:
        return [f"{camera_name}: no CameraInfo message was found in the bag"]
    warnings = []
    K_info = np.asarray(camera_info_msg.k, dtype=float).reshape(3, 3)
    max_k_diff = float(np.max(np.abs(K_info - camera.K)))
    if max_k_diff > tolerance_px:
        warnings.append(
            f"{camera_name}: Kalibr intrinsics differ from bag CameraInfo by {max_k_diff:.3f} px"
        )
    D_info = np.asarray(camera_info_msg.d, dtype=float).reshape(-1)
    D_kalibr = camera.D.reshape(-1)
    if D_info.size == D_kalibr.size:
        max_d_diff = float(np.max(np.abs(D_info - D_kalibr)))
        if max_d_diff > 1e-3:
            warnings.append(f"{camera_name}: Kalibr distortion differs from bag CameraInfo by {max_d_diff:.6f}")
    return warnings


def write_residual_csv(path: Path, estimate: RobustEstimate, samples: list[CalibrationSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    inlier_samples = [samples[idx] for idx in estimate.inlier_indices]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "sample_index",
                "stamp",
                "residual_x_m",
                "residual_y_m",
                "residual_z_m",
                "residual_roll_rad",
                "residual_pitch_rad",
                "residual_yaw_rad",
                "translation_residual_m",
                "rotation_residual_deg",
                "arm_before_dt_s",
                "arm_after_dt_s",
                "arm_alpha",
                "head_id2_reprojection_error_px",
                "head_id2_distance_m",
                "head_id2_view_angle_deg",
            ],
        )
        writer.writeheader()
        for residual_idx, (sample, residual) in enumerate(zip(inlier_samples, estimate.residuals)):
            writer.writerow(
                {
                    "sample_index": estimate.inlier_indices[residual_idx],
                    "stamp": f"{sample.stamp:.9f}",
                    "residual_x_m": f"{residual[0]:.9g}",
                    "residual_y_m": f"{residual[1]:.9g}",
                    "residual_z_m": f"{residual[2]:.9g}",
                    "residual_roll_rad": f"{residual[3]:.9g}",
                    "residual_pitch_rad": f"{residual[4]:.9g}",
                    "residual_yaw_rad": f"{residual[5]:.9g}",
                    "translation_residual_m": f"{estimate.translation_norms[residual_idx]:.9g}",
                    "rotation_residual_deg": f"{estimate.rotation_angles_deg[residual_idx]:.9g}",
                    "arm_before_dt_s": f"{sample.arm_before_dt:.9g}",
                    "arm_after_dt_s": f"{sample.arm_after_dt:.9g}",
                    "arm_alpha": f"{sample.arm_alpha:.9g}",
                    "head_id2_reprojection_error_px": f"{sample.head_dynamic.reprojection_error_px:.9g}",
                    "head_id2_distance_m": f"{sample.head_dynamic.distance_m:.9g}",
                    "head_id2_view_angle_deg": f"{sample.head_dynamic.view_angle_deg:.9g}",
                }
            )


def collect_samples(args: argparse.Namespace):
    head_config_path = resolve_path(args.head_config)
    arm_config_path = resolve_path(args.arm_config)
    head_camera = load_camera_config(head_config_path)
    arm_camera = load_camera_config(arm_config_path)

    dictionary_name = str(head_camera.marker_config.get("aruco", {}).get("dictionary", "DICT_6X6_1000"))
    if dictionary_name != str(arm_camera.marker_config.get("aruco", {}).get("dictionary", dictionary_name)):
        raise RuntimeError("Head and arm marker configs use different ArUco dictionaries")
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))

    fixed_marker_id = int(args.fixed_marker_id)
    dynamic_marker_id = int(args.dynamic_marker_id)
    head_dynamic_cfg = dynamic_marker_config(head_camera.marker_config, dynamic_marker_id)
    marker_frame = str(head_dynamic_cfg.get("frame_id", f"arm_marker_{dynamic_marker_id}"))
    dynamic_marker_size_m = float(
        head_dynamic_cfg.get(
            "size_m",
            head_camera.marker_config.get("aruco", {}).get("default_marker_size_m", 0.100),
        )
    )
    head_marker_sizes = {
        fixed_marker_id: fixed_marker_size(head_camera.marker_config, fixed_marker_id),
        dynamic_marker_id: dynamic_marker_size_m,
    }
    arm_marker_sizes = {fixed_marker_id: fixed_marker_size(arm_camera.marker_config, fixed_marker_id)}
    T_map_marker0_head = fixed_T_map_marker(head_camera.marker_config, fixed_marker_id)
    T_map_marker0_arm = fixed_T_map_marker(arm_camera.marker_config, fixed_marker_id)

    topics = {
        head_camera.image_topic,
        arm_camera.image_topic,
        head_camera.camera_info_topic,
        arm_camera.camera_info_topic,
    }

    head_both_samples: list[tuple[MarkerDetection, MarkerDetection, np.ndarray]] = []
    arm_samples: list[CameraMapSample] = []
    camera_infos: dict[str, Any] = {}
    raw_counts = {"head_images": 0, "arm_images": 0}
    detection_counts = {
        "head_id0": 0,
        "head_dynamic": 0,
        "head_id0_and_dynamic": 0,
        "arm_id0": 0,
    }

    for topic, msg, _bag_stamp_ns in read_bag_messages(resolve_path(args.bag), topics):
        if topic == head_camera.camera_info_topic and topic not in camera_infos:
            camera_infos[topic] = msg
            continue
        if topic == arm_camera.camera_info_topic and topic not in camera_infos:
            camera_infos[topic] = msg
            continue
        if topic == head_camera.image_topic:
            raw_counts["head_images"] += 1
            detections = detect_required_markers(msg, head_camera, aruco_dict, head_marker_sizes)
            if fixed_marker_id in detections:
                detection_counts["head_id0"] += 1
            if dynamic_marker_id in detections:
                detection_counts["head_dynamic"] += 1
            if fixed_marker_id in detections and dynamic_marker_id in detections:
                detection_counts["head_id0_and_dynamic"] += 1
                head_id0 = detections[fixed_marker_id]
                T_map_headcam = compute_camera_map_sample(head_id0, T_map_marker0_head).T_map_cam
                head_both_samples.append((head_id0, detections[dynamic_marker_id], T_map_headcam))
        elif topic == arm_camera.image_topic:
            raw_counts["arm_images"] += 1
            detections = detect_required_markers(msg, arm_camera, aruco_dict, arm_marker_sizes)
            if fixed_marker_id in detections:
                detection_counts["arm_id0"] += 1
                arm_samples.append(compute_camera_map_sample(detections[fixed_marker_id], T_map_marker0_arm))

    head_both_samples.sort(key=lambda sample: sample[0].stamp)
    arm_samples.sort(key=lambda sample: sample.stamp)
    calibration_samples = build_calibration_samples(head_both_samples, arm_samples, args.sync_tolerance_s)
    warnings = []
    warnings.extend(
        compare_camera_info(
            "head",
            head_camera,
            camera_infos.get(head_camera.camera_info_topic),
            args.camera_info_tolerance_px,
        )
    )
    warnings.extend(
        compare_camera_info(
            "arm",
            arm_camera,
            camera_infos.get(arm_camera.camera_info_topic),
            args.camera_info_tolerance_px,
        )
    )
    return (
        head_camera,
        arm_camera,
        marker_frame,
        dynamic_marker_size_m,
        raw_counts,
        detection_counts,
        head_both_samples,
        arm_samples,
        calibration_samples,
        warnings,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="ROS 2 bag directory")
    parser.add_argument("--head-config", required=True, help="Head marker YAML config")
    parser.add_argument("--arm-config", required=True, help="Arm marker YAML config")
    parser.add_argument("--dynamic-marker-id", type=int, default=2)
    parser.add_argument("--fixed-marker-id", type=int, default=0)
    parser.add_argument("--output", required=True, help="Output YAML path")
    parser.add_argument("--report-dir", default="", help="Optional report directory for summary/residual CSV")
    parser.add_argument("--sync-tolerance-s", type=float, default=0.05)
    parser.add_argument("--min-inliers", type=int, default=100)
    parser.add_argument("--camera-info-tolerance-px", type=float, default=2.0)
    parser.add_argument("--strict-camera-info", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    (
        head_camera,
        arm_camera,
        marker_frame,
        dynamic_marker_size_m,
        raw_counts,
        detection_counts,
        head_both_samples,
        arm_samples,
        calibration_samples,
        warnings,
    ) = collect_samples(args)

    for warning in warnings:
        print(f"WARNING: {warning}")
    if args.strict_camera_info and warnings:
        raise RuntimeError("CameraInfo mismatch warnings were found and --strict-camera-info was set")

    print(f"Raw image counts: {raw_counts}")
    print(f"Detection counts: {detection_counts}")
    print(f"Head ID0+ID{args.dynamic_marker_id} candidate frames: {len(head_both_samples)}")
    print(f"Arm ID0 map samples: {len(arm_samples)}")
    print(f"Synchronized calibration samples: {len(calibration_samples)}")

    if not calibration_samples:
        raise RuntimeError("No synchronized calibration samples were produced")

    transforms = [sample.T_armcam_marker for sample in calibration_samples]
    estimate = robust_se3_estimate(transforms)
    if len(estimate.inlier_indices) < int(args.min_inliers):
        raise RuntimeError(
            f"Calibration has {len(estimate.inlier_indices)} inliers, below required {args.min_inliers}"
        )

    output = summarize_estimate(
        estimate=estimate,
        samples=calibration_samples,
        source_bag=resolve_path(args.bag),
        marker_id=int(args.dynamic_marker_id),
        marker_frame=marker_frame,
        marker_size_m=dynamic_marker_size_m,
        parent_camera_frame=arm_camera.camera_frame,
        parent_imu_frame=arm_camera.imu_frame,
        T_armcam_imu=arm_camera.T_cam_imu,
    )
    output_path = resolve_path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(yaml.safe_dump(to_builtin(output), sort_keys=False), encoding="utf-8")
    print(f"Wrote calibration: {output_path}")
    print(
        "Residuals: "
        f"inliers={len(estimate.inlier_indices)} outliers={len(estimate.outlier_indices)} "
        f"median_trans={median_or_default(estimate.translation_norms):.4f} m "
        f"p95_trans={percentile_or_default(estimate.translation_norms, 95.0):.4f} m "
        f"median_rot={median_or_default(estimate.rotation_angles_deg):.3f} deg "
        f"p95_rot={percentile_or_default(estimate.rotation_angles_deg, 95.0):.3f} deg"
    )

    if args.report_dir:
        report_dir = resolve_path(args.report_dir)
        report_dir.mkdir(parents=True, exist_ok=True)
        summary_path = report_dir / "arm_marker_extrinsic_summary.yaml"
        summary_path.write_text(yaml.safe_dump(to_builtin(output), sort_keys=False), encoding="utf-8")
        write_residual_csv(report_dir / "arm_marker_extrinsic_residuals.csv", estimate, calibration_samples)
        print(f"Wrote report: {report_dir}")


if __name__ == "__main__":
    main()
