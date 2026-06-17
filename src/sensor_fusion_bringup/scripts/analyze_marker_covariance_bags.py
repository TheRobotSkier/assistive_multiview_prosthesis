#!/usr/bin/env python3

"""Analyze Phase 1 marker covariance validation bags.

The script is intended to run inside the ROS 2 Jazzy environment so it can use
rosbag2_py for MCAP deserialization. The numerical analysis itself only uses
the public topics already recorded by the Phase 1 external marker pipeline.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import re
from typing import Any, Iterable, Optional

import numpy as np
import yaml


ALL_MARKER_QUALITY_TOPIC = "/head/marker_pose/all_marker_quality"
MARKER_QUALITY_TOPIC = "/head/marker_pose/marker_quality"
MARKER_VALID_TOPIC = "/head/marker_pose/marker_valid"
ACTIVE_MARKER_ID_TOPIC = "/head/marker_pose/active_marker_id"
REANCHOR_EVENT_TOPIC = "/head/marker_pose/reanchor_event"
VIO_VALID_TOPIC = "/head/marker_pose/vio_valid"
CORRECTED_ODOM_TOPIC = "/head/marker_pose/ov_corrected_odom"
RAW_ODOM_TOPIC = "/ov_msckf/odomimu"
IMU_POSE_TOPIC = "/head/marker_pose/imu_pose"
CAMERA_INFO_TOPIC = "/head/d435i_head/color/camera_info"

ANALYSIS_TOPICS = {
    ALL_MARKER_QUALITY_TOPIC,
    MARKER_QUALITY_TOPIC,
    MARKER_VALID_TOPIC,
    ACTIVE_MARKER_ID_TOPIC,
    REANCHOR_EVENT_TOPIC,
    VIO_VALID_TOPIC,
    CORRECTED_ODOM_TOPIC,
    RAW_ODOM_TOPIC,
    IMU_POSE_TOPIC,
    CAMERA_INFO_TOPIC,
}

ESTIMATION_SETUP_ORDER = ("near_mixed", "medium_mixed", "far_mixed")
AXES = ("x", "y", "z", "roll", "pitch", "yaw")


@dataclass
class BagInfo:
    name: str
    path: Path
    phase: str
    setup: str
    timestamp: str
    split: str = ""
    duration_s: float = 0.0
    total_messages: int = 0
    topic_counts: dict[str, int] = field(default_factory=dict)


@dataclass
class BagData:
    info: BagInfo
    all_marker_rows: list[dict[str, Any]] = field(default_factory=list)
    marker_quality_rows: list[dict[str, Any]] = field(default_factory=list)
    reanchor_events: list[dict[str, Any]] = field(default_factory=list)
    marker_valid: list[dict[str, Any]] = field(default_factory=list)
    vio_valid: list[dict[str, Any]] = field(default_factory=list)
    active_marker_id: list[dict[str, Any]] = field(default_factory=list)
    corrected_odom: list[dict[str, Any]] = field(default_factory=list)
    raw_odom: list[dict[str, Any]] = field(default_factory=list)
    imu_pose: list[dict[str, Any]] = field(default_factory=list)
    camera_f_avg_px: Optional[float] = None


@dataclass
class CovarianceConfig:
    corner_noise_floor_px: float = 0.35
    max_view_angle_deg: float = 75.0
    max_view_penalty: float = 4.0
    min_marker_xy_std_m: float = 0.02
    min_marker_z_std_m: float = 0.025
    max_marker_xy_std_m: float = 0.75
    max_marker_z_std_m: float = 0.75
    min_marker_roll_pitch_std_deg: float = 2.0
    min_marker_yaw_std_deg: float = 2.0
    max_marker_roll_pitch_std_deg: float = 45.0
    max_marker_yaw_std_deg: float = 45.0
    marker_xy_std_px_gain: float = 8.0
    marker_z_std_px_gain: float = 2.0
    marker_roll_pitch_std_px_gain: float = 5.0
    marker_yaw_std_px_gain: float = 3.0

    @classmethod
    def from_marker_config(cls, config: dict[str, Any]) -> "CovarianceConfig":
        cfg = config.get("covariance", {})
        legacy_min_position = float(cfg.get("min_marker_position_std_m", 0.02))
        legacy_max_position = float(cfg.get("max_marker_position_std_m", 0.75))
        legacy_min_rotation = float(cfg.get("min_marker_rotation_std_deg", 2.0))
        legacy_max_rotation = float(cfg.get("max_marker_rotation_std_deg", 45.0))
        return cls(
            corner_noise_floor_px=float(cfg.get("corner_noise_floor_px", 0.35)),
            max_view_angle_deg=float(cfg.get("max_view_angle_deg", 75.0)),
            max_view_penalty=float(cfg.get("max_view_penalty", 4.0)),
            min_marker_xy_std_m=float(cfg.get("min_marker_xy_std_m", legacy_min_position)),
            min_marker_z_std_m=float(cfg.get("min_marker_z_std_m", legacy_min_position)),
            max_marker_xy_std_m=float(cfg.get("max_marker_xy_std_m", legacy_max_position)),
            max_marker_z_std_m=float(cfg.get("max_marker_z_std_m", legacy_max_position)),
            min_marker_roll_pitch_std_deg=float(
                cfg.get("min_marker_roll_pitch_std_deg", legacy_min_rotation)
            ),
            min_marker_yaw_std_deg=float(cfg.get("min_marker_yaw_std_deg", legacy_min_rotation)),
            max_marker_roll_pitch_std_deg=float(
                cfg.get("max_marker_roll_pitch_std_deg", legacy_max_rotation)
            ),
            max_marker_yaw_std_deg=float(cfg.get("max_marker_yaw_std_deg", legacy_max_rotation)),
            marker_xy_std_px_gain=float(cfg.get("marker_xy_std_px_gain", 8.0)),
            marker_z_std_px_gain=float(cfg.get("marker_z_std_px_gain", 2.0)),
            marker_roll_pitch_std_px_gain=float(cfg.get("marker_roll_pitch_std_px_gain", 5.0)),
            marker_yaw_std_px_gain=float(cfg.get("marker_yaw_std_px_gain", 3.0)),
        )


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def finite_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def percentile(values: Iterable[float], q: float, default: float = 0.0) -> float:
    arr = np.asarray([v for v in values if math.isfinite(float(v))], dtype=float)
    if arr.size == 0:
        return default
    return float(np.percentile(arr, q))


def mean_or_default(values: Iterable[float], default: float = 0.0) -> float:
    arr = np.asarray([v for v in values if math.isfinite(float(v))], dtype=float)
    if arr.size == 0:
        return default
    return float(np.mean(arr))


def std_or_default(values: Iterable[float], default: float = 0.0) -> float:
    arr = np.asarray([v for v in values if math.isfinite(float(v))], dtype=float)
    if arr.size < 2:
        return default
    return float(np.std(arr, ddof=1))


def mad_std(values: Iterable[float], default: float = 0.0) -> float:
    arr = np.asarray([v for v in values if math.isfinite(float(v))], dtype=float)
    if arr.size == 0:
        return default
    med = float(np.median(arr))
    return float(1.4826 * np.median(np.abs(arr - med)))


def csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.9g}"
    if isinstance(value, (list, tuple, dict)):
        return json.dumps(value, separators=(",", ":"))
    return value


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: Optional[list[str]] = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        seen: list[str] = []
        for row in rows:
            for key in row.keys():
                if key not in seen:
                    seen.append(key)
        fieldnames = seen
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: csv_value(row.get(key)) for key in fieldnames})


def read_metadata(bag_dir: Path) -> tuple[float, int, dict[str, int], str]:
    metadata_path = bag_dir / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    info = metadata["rosbag2_bagfile_information"]
    duration_s = float(info["duration"]["nanoseconds"]) * 1e-9
    topic_counts = {
        row["topic_metadata"]["name"]: int(row["message_count"])
        for row in info.get("topics_with_message_count", [])
    }
    return duration_s, int(info["message_count"]), topic_counts, str(info.get("storage_identifier", "mcap"))


def parse_bag_name(bag_dir: Path) -> BagInfo:
    name = bag_dir.name
    match = re.match(r"head_marker_cov_(estimation|validation)_100mm_(.+)_([0-9]{8}_[0-9]{6})$", name)
    if not match:
        phase = "unknown"
        setup = "unknown"
        timestamp = ""
    else:
        phase, setup, timestamp = match.groups()
    duration_s, total_messages, topic_counts, _storage = read_metadata(bag_dir)
    return BagInfo(
        name=name,
        path=bag_dir,
        phase=phase,
        setup=setup,
        timestamp=timestamp,
        duration_s=duration_s,
        total_messages=total_messages,
        topic_counts=topic_counts,
    )


def assign_calibration_splits(infos: list[BagInfo], calibration_replicates: int) -> None:
    by_setup: dict[str, list[BagInfo]] = defaultdict(list)
    for info in infos:
        if info.phase == "estimation":
            by_setup[info.setup].append(info)
        else:
            info.split = "validation"

    for setup, rows in by_setup.items():
        rows.sort(key=lambda item: item.timestamp)
        for idx, info in enumerate(rows):
            info.split = "calibration" if idx < calibration_replicates else "holdout"


def discover_bags(bag_root: Path, calibration_replicates: int) -> list[BagInfo]:
    bag_dirs = sorted(path for path in bag_root.iterdir() if (path / "metadata.yaml").exists())
    infos = [parse_bag_name(path) for path in bag_dirs]
    assign_calibration_splits(infos, calibration_replicates)
    return infos


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
    _duration_s, _total, _topic_counts, storage_id = read_metadata(bag_dir)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(topic_type)
        for topic, topic_type in topic_types.items()
        if topic in topics
    }

    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in topics:
            continue
        msg = deserialize_message(serialized, message_types[topic])
        yield topic, msg, int(timestamp_ns)


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def json_from_string_msg(msg: Any) -> Optional[dict[str, Any]]:
    try:
        payload = json.loads(msg.data)
    except (TypeError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def rotvec_to_matrix(rotvec: Iterable[float]) -> np.ndarray:
    v = np.asarray(list(rotvec), dtype=float).reshape(3)
    theta = float(np.linalg.norm(v))
    if theta < 1e-12:
        return np.eye(3)
    axis = v / theta
    kx, ky, kz = axis
    K = np.array(
        [
            [0.0, -kz, ky],
            [kz, 0.0, -kx],
            [-ky, kx, 0.0],
        ],
        dtype=float,
    )
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def matrix_to_rotvec(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    cos_angle = clamp((float(np.trace(R)) - 1.0) * 0.5, -1.0, 1.0)
    angle = math.acos(cos_angle)
    skew_vec = np.array(
        [
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ],
        dtype=float,
    )
    if angle < 1e-9:
        return 0.5 * skew_vec
    sin_angle = math.sin(angle)
    if abs(sin_angle) < 1e-9:
        return 0.5 * angle * skew_vec
    return angle / (2.0 * sin_angle) * skew_vec


def quat_xyzw_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n <= 0.0 or not math.isfinite(n):
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=float,
    )


def mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    if not rotations:
        return np.eye(3)
    M = np.mean(np.stack(rotations, axis=0), axis=0)
    U, _S, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0.0:
        U[:, -1] *= -1.0
        R = U @ Vt
    return R


def pose_dict_from_pose_msg(stamp_sec: float, pose: Any, covariance: Optional[list[float]] = None) -> dict[str, Any]:
    p = pose.position
    q = pose.orientation
    R = quat_xyzw_to_matrix(float(q.x), float(q.y), float(q.z), float(q.w))
    rotvec = matrix_to_rotvec(R)
    row = {
        "stamp": stamp_sec,
        "x_m": float(p.x),
        "y_m": float(p.y),
        "z_m": float(p.z),
        "roll_rad": float(rotvec[0]),
        "pitch_rad": float(rotvec[1]),
        "yaw_rad": float(rotvec[2]),
        "R": R,
    }
    if covariance is not None:
        diag = np.diag(np.asarray(covariance, dtype=float).reshape(6, 6))
        row.update(
            {
                "std_x_m": math.sqrt(max(float(diag[0]), 0.0)),
                "std_y_m": math.sqrt(max(float(diag[1]), 0.0)),
                "std_z_m": math.sqrt(max(float(diag[2]), 0.0)),
                "std_roll_deg": math.degrees(math.sqrt(max(float(diag[3]), 0.0))),
                "std_pitch_deg": math.degrees(math.sqrt(max(float(diag[4]), 0.0))),
                "std_yaw_deg": math.degrees(math.sqrt(max(float(diag[5]), 0.0))),
            }
        )
    return row


def odom_row_from_msg(msg: Any) -> dict[str, Any]:
    stamp = stamp_to_sec(msg.header.stamp)
    row = pose_dict_from_pose_msg(stamp, msg.pose.pose, list(msg.pose.covariance))
    row["frame_id"] = msg.header.frame_id
    row["child_frame_id"] = msg.child_frame_id
    linear = msg.twist.twist.linear
    angular = msg.twist.twist.angular
    row.update(
        {
            "linear_speed_mps": math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z),
            "angular_speed_radps": math.sqrt(angular.x * angular.x + angular.y * angular.y + angular.z * angular.z),
        }
    )
    return row


def extract_bag_data(info: BagInfo) -> BagData:
    data = BagData(info=info)

    for topic, msg, timestamp_ns in read_bag_messages(info.path, ANALYSIS_TOPICS):
        bag_stamp = float(timestamp_ns) * 1e-9
        if topic == CAMERA_INFO_TOPIC and data.camera_f_avg_px is None:
            K = list(msg.k)
            data.camera_f_avg_px = 0.5 * (float(K[0]) + float(K[4]))

        elif topic == ALL_MARKER_QUALITY_TOPIC:
            payload = json_from_string_msg(msg)
            if payload is None:
                continue
            stamp = finite_float(payload.get("stamp"), bag_stamp)
            for marker in payload.get("markers", []):
                tvec = marker.get("tvec_cam_marker_m", [None, None, None])
                rvec = marker.get("rvec_cam_marker_rad", [None, None, None])
                row = {
                    "bag": info.name,
                    "phase": info.phase,
                    "setup": info.setup,
                    "split": info.split,
                    "stamp": stamp,
                    "frame_id": payload.get("frame_id", ""),
                    "marker_count": int(payload.get("marker_count", len(payload.get("markers", [])))),
                    "marker_id": int(marker.get("marker_id", -1)),
                    "marker_size_m": finite_float(marker.get("marker_size_m"), 0.100),
                    "area_px2": finite_float(marker.get("area_px2")),
                    "sqrt_area_px": finite_float(marker.get("sqrt_area_px")),
                    "side_mean_px": finite_float(marker.get("side_mean_px")),
                    "side_min_px": finite_float(marker.get("side_min_px")),
                    "distance_m": finite_float(marker.get("distance_m")),
                    "view_angle_deg": finite_float(marker.get("view_angle_deg")),
                    "reprojection_error_px": finite_float(marker.get("reprojection_error_px")),
                    "tvec_x_m": finite_float(tvec[0] if len(tvec) > 0 else None),
                    "tvec_y_m": finite_float(tvec[1] if len(tvec) > 1 else None),
                    "tvec_z_m": finite_float(tvec[2] if len(tvec) > 2 else None),
                    "rvec_x_rad": finite_float(rvec[0] if len(rvec) > 0 else None),
                    "rvec_y_rad": finite_float(rvec[1] if len(rvec) > 1 else None),
                    "rvec_z_rad": finite_float(rvec[2] if len(rvec) > 2 else None),
                    "camera_f_avg_px": data.camera_f_avg_px,
                }
                data.all_marker_rows.append(row)

        elif topic == MARKER_QUALITY_TOPIC:
            payload = json_from_string_msg(msg)
            if payload is None:
                continue
            payload = dict(payload)
            payload.update(
                {
                    "bag": info.name,
                    "phase": info.phase,
                    "setup": info.setup,
                    "split": info.split,
                    "stamp": finite_float(payload.get("stamp"), bag_stamp),
                }
            )
            data.marker_quality_rows.append(payload)

        elif topic == REANCHOR_EVENT_TOPIC:
            payload = json_from_string_msg(msg)
            if payload is None:
                continue
            payload = dict(payload)
            payload.update(
                {
                    "bag": info.name,
                    "phase": info.phase,
                    "setup": info.setup,
                    "split": info.split,
                    "stamp": finite_float(payload.get("stamp"), bag_stamp),
                }
            )
            data.reanchor_events.append(payload)

        elif topic == MARKER_VALID_TOPIC:
            data.marker_valid.append({"stamp": bag_stamp, "value": bool(msg.data)})

        elif topic == VIO_VALID_TOPIC:
            data.vio_valid.append({"stamp": bag_stamp, "value": bool(msg.data)})

        elif topic == ACTIVE_MARKER_ID_TOPIC:
            data.active_marker_id.append({"stamp": bag_stamp, "value": int(msg.data)})

        elif topic == CORRECTED_ODOM_TOPIC:
            data.corrected_odom.append(odom_row_from_msg(msg))

        elif topic == RAW_ODOM_TOPIC:
            data.raw_odom.append(odom_row_from_msg(msg))

        elif topic == IMU_POSE_TOPIC:
            stamp = stamp_to_sec(msg.header.stamp)
            row = pose_dict_from_pose_msg(stamp, msg.pose.pose, list(msg.pose.covariance))
            row["frame_id"] = msg.header.frame_id
            data.imu_pose.append(row)

    return data


def covariance_view_penalty(view_angle_deg: float, cfg: CovarianceConfig) -> float:
    cos_view = math.cos(math.radians(clamp(float(view_angle_deg), 0.0, 89.9)))
    cos_limit = math.cos(math.radians(clamp(float(cfg.max_view_angle_deg), 0.0, 89.9)))
    penalty = 1.0 / max(cos_view, cos_limit, 1e-6)
    return clamp(penalty, 1.0, max(1.0, float(cfg.max_view_penalty)))


def offline_camera_std(row: dict[str, Any], cfg: CovarianceConfig, fallback_f_avg_px: float) -> dict[str, float]:
    sigma_px = max(cfg.corner_noise_floor_px, finite_float(row.get("reprojection_error_px")))
    side_mean_px = max(finite_float(row.get("side_mean_px"), 1.0), 1.0)
    f_avg = max(finite_float(row.get("camera_f_avg_px"), fallback_f_avg_px), 1.0)
    distance_m = max(finite_float(row.get("distance_m")), 0.0)
    view_penalty = covariance_view_penalty(finite_float(row.get("view_angle_deg")), cfg)

    sigma_xy = cfg.min_marker_xy_std_m + cfg.marker_xy_std_px_gain * distance_m * sigma_px / f_avg
    sigma_z = cfg.min_marker_z_std_m + cfg.marker_z_std_px_gain * distance_m * sigma_px / side_mean_px * view_penalty
    sigma_roll_pitch = math.radians(cfg.min_marker_roll_pitch_std_deg) + (
        cfg.marker_roll_pitch_std_px_gain * sigma_px / side_mean_px * view_penalty
    )
    sigma_yaw = math.radians(cfg.min_marker_yaw_std_deg) + cfg.marker_yaw_std_px_gain * sigma_px / side_mean_px

    return {
        "pred_sigma_px": sigma_px,
        "pred_view_penalty": view_penalty,
        "pred_std_x_m": clamp(sigma_xy, cfg.min_marker_xy_std_m, cfg.max_marker_xy_std_m),
        "pred_std_y_m": clamp(sigma_xy, cfg.min_marker_xy_std_m, cfg.max_marker_xy_std_m),
        "pred_std_z_m": clamp(sigma_z, cfg.min_marker_z_std_m, cfg.max_marker_z_std_m),
        "pred_std_roll_deg": math.degrees(
            clamp(
                sigma_roll_pitch,
                math.radians(cfg.min_marker_roll_pitch_std_deg),
                math.radians(cfg.max_marker_roll_pitch_std_deg),
            )
        ),
        "pred_std_pitch_deg": math.degrees(
            clamp(
                sigma_roll_pitch,
                math.radians(cfg.min_marker_roll_pitch_std_deg),
                math.radians(cfg.max_marker_roll_pitch_std_deg),
            )
        ),
        "pred_std_yaw_deg": math.degrees(
            clamp(
                sigma_yaw,
                math.radians(cfg.min_marker_yaw_std_deg),
                math.radians(cfg.max_marker_yaw_std_deg),
            )
        ),
    }


def add_prediction_fields(rows: list[dict[str, Any]], cfg: CovarianceConfig, fallback_f_avg_px: float) -> None:
    for row in rows:
        row.update(offline_camera_std(row, cfg, fallback_f_avg_px))


def add_stationary_residuals(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, int], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[(str(row["bag"]), int(row["marker_id"]))].append(row)

    for (_bag, _marker_id), group in grouped.items():
        translations = np.array(
            [[row["tvec_x_m"], row["tvec_y_m"], row["tvec_z_m"]] for row in group],
            dtype=float,
        )
        rotations = [
            rotvec_to_matrix([row["rvec_x_rad"], row["rvec_y_rad"], row["rvec_z_rad"]])
            for row in group
        ]
        mean_t = np.mean(translations, axis=0)
        mean_R = mean_rotation(rotations)

        for row, t, R in zip(group, translations, rotations):
            trans_res = t - mean_t
            rot_res = matrix_to_rotvec(R @ mean_R.T)
            row.update(
                {
                    "bag_marker_sample_count": len(group),
                    "mean_tvec_x_m": mean_t[0],
                    "mean_tvec_y_m": mean_t[1],
                    "mean_tvec_z_m": mean_t[2],
                    "residual_x_m": trans_res[0],
                    "residual_y_m": trans_res[1],
                    "residual_z_m": trans_res[2],
                    "residual_roll_deg": math.degrees(float(rot_res[0])),
                    "residual_pitch_deg": math.degrees(float(rot_res[1])),
                    "residual_yaw_deg": math.degrees(float(rot_res[2])),
                    "translation_error_norm_m": float(np.linalg.norm(trans_res)),
                    "rotation_error_angle_deg": math.degrees(float(np.linalg.norm(rot_res))),
                }
            )
    return rows


def summarize_residual_rows(rows: list[dict[str, Any]], label_fields: dict[str, Any]) -> dict[str, Any]:
    out = dict(label_fields)
    n = len(rows)
    out["sample_count"] = n
    if n == 0:
        return out

    for key in ("distance_m", "side_mean_px", "side_min_px", "view_angle_deg", "reprojection_error_px"):
        out[f"median_{key}"] = percentile((finite_float(row.get(key)) for row in rows), 50.0)

    axis_specs = [
        ("x", "m", "residual_x_m", "pred_std_x_m"),
        ("y", "m", "residual_y_m", "pred_std_y_m"),
        ("z", "m", "residual_z_m", "pred_std_z_m"),
        ("roll", "deg", "residual_roll_deg", "pred_std_roll_deg"),
        ("pitch", "deg", "residual_pitch_deg", "pred_std_pitch_deg"),
        ("yaw", "deg", "residual_yaw_deg", "pred_std_yaw_deg"),
    ]
    robust_scales = []
    for axis, unit, residual_key, pred_key in axis_specs:
        values = [finite_float(row.get(residual_key)) for row in rows]
        preds = [finite_float(row.get(pred_key), default=float("nan")) for row in rows]
        robust = mad_std(values)
        robust_scales.append((residual_key, robust if robust > 1e-12 else std_or_default(values)))
        out[f"std_{axis}_{unit}"] = std_or_default(values)
        out[f"mad_std_{axis}_{unit}"] = robust
        out[f"median_pred_std_{axis}_{unit}"] = percentile(preds, 50.0)
        pred_med = out[f"median_pred_std_{axis}_{unit}"]
        out[f"empirical_over_pred_{axis}"] = (
            out[f"mad_std_{axis}_{unit}"] / pred_med if pred_med > 1e-12 else ""
        )
        out[f"coverage_1sigma_{axis}"] = coverage_fraction(values, preds, 1.0)
        out[f"coverage_2sigma_{axis}"] = coverage_fraction(values, preds, 2.0)

    out["p68_translation_error_norm_m"] = percentile((row["translation_error_norm_m"] for row in rows), 68.0)
    out["p95_translation_error_norm_m"] = percentile((row["translation_error_norm_m"] for row in rows), 95.0)
    out["p68_rotation_error_angle_deg"] = percentile((row["rotation_error_angle_deg"] for row in rows), 68.0)
    out["p95_rotation_error_angle_deg"] = percentile((row["rotation_error_angle_deg"] for row in rows), 95.0)

    outlier_count = 0
    for row in rows:
        outlier = False
        for residual_key, scale in robust_scales:
            if scale > 1e-12 and abs(finite_float(row.get(residual_key))) > 3.0 * scale:
                outlier = True
                break
        if outlier:
            outlier_count += 1
    out["outlier_rate_3mad"] = outlier_count / max(n, 1)
    return out


def coverage_fraction(values: Iterable[float], stds: Iterable[float], multiplier: float) -> float:
    count = 0
    inside = 0
    for value, std in zip(values, stds):
        value = finite_float(value, default=float("nan"))
        std = finite_float(std, default=float("nan"))
        if not math.isfinite(value) or not math.isfinite(std) or std <= 0.0:
            continue
        count += 1
        if abs(value) <= multiplier * std:
            inside += 1
    return inside / count if count else 0.0


def summarize_stationary_groups(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, int], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[(str(row["bag"]), int(row["marker_id"]))].append(row)

    summaries = []
    for (bag, marker_id), group in sorted(grouped.items()):
        first = group[0]
        summary = summarize_residual_rows(
            group,
            {
                "bag": bag,
                "phase": first["phase"],
                "setup": first["setup"],
                "split": first["split"],
                "marker_id": marker_id,
                "mean_tvec_x_m": mean_or_default(row["tvec_x_m"] for row in group),
                "mean_tvec_y_m": mean_or_default(row["tvec_y_m"] for row in group),
                "mean_tvec_z_m": mean_or_default(row["tvec_z_m"] for row in group),
                "camera_f_avg_px": percentile(
                    (finite_float(row.get("camera_f_avg_px"), default=float("nan")) for row in group),
                    50.0,
                    default=0.0,
                ),
            },
        )
        summaries.append(summary)
    return summaries


def bin_label(metric: str, lower: float, upper: float) -> str:
    if math.isinf(upper):
        return f"{metric}>={lower:g}"
    return f"{lower:g}<={metric}<{upper:g}"


def initial_bin_index(value: float, edges: list[float]) -> int:
    for idx in range(len(edges) - 1):
        if edges[idx] <= value < edges[idx + 1]:
            return idx
    return len(edges) - 2


def merged_metric_bins(
    rows: list[dict[str, Any]],
    metric: str,
    edges: list[float],
    min_samples: int,
) -> list[tuple[str, list[dict[str, Any]]]]:
    initial: list[tuple[float, float, list[dict[str, Any]]]] = []
    for idx in range(len(edges) - 1):
        bucket = [
            row
            for row in rows
            if initial_bin_index(finite_float(row.get(metric)), edges) == idx
        ]
        if bucket:
            initial.append((edges[idx], edges[idx + 1], bucket))

    merged: list[tuple[float, float, list[dict[str, Any]]]] = []
    current_low: Optional[float] = None
    current_high: Optional[float] = None
    current_rows: list[dict[str, Any]] = []

    for low, high, bucket in initial:
        if current_low is None:
            current_low = low
        current_high = high
        current_rows.extend(bucket)
        if len(current_rows) >= min_samples:
            merged.append((current_low, current_high, current_rows))
            current_low = None
            current_high = None
            current_rows = []

    if current_rows:
        if merged:
            low, _high, old_rows = merged[-1]
            merged[-1] = (low, current_high if current_high is not None else float("inf"), old_rows + current_rows)
        else:
            merged.append((current_low if current_low is not None else edges[0], current_high or edges[-1], current_rows))

    return [(bin_label(metric, low, high), bucket) for low, high, bucket in merged]


def summarize_bins(rows: list[dict[str, Any]], min_samples: int) -> list[dict[str, Any]]:
    bin_specs = {
        "distance_m": [0.0, 0.6, 1.1, 1.8, float("inf")],
        "side_mean_px": [0.0, 60.0, 90.0, 130.0, float("inf")],
        "view_angle_deg": [0.0, 15.0, 30.0, 45.0, 60.0, 90.0],
        "reprojection_error_px": [0.0, 0.25, 0.5, 1.0, 2.0, float("inf")],
    }
    summaries: list[dict[str, Any]] = []
    for metric, edges in bin_specs.items():
        for label, bucket in merged_metric_bins(rows, metric, edges, min_samples):
            summaries.append(summarize_residual_rows(bucket, {"bin_dimension": metric, "bin": label}))

    for marker_id in sorted({int(row["marker_id"]) for row in rows}):
        bucket = [row for row in rows if int(row["marker_id"]) == marker_id]
        summaries.append(summarize_residual_rows(bucket, {"bin_dimension": "marker_id", "bin": str(marker_id)}))

    return summaries


def coverage_summary(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    splits = [("all_estimation", rows)]
    for split in ("calibration", "holdout"):
        splits.append((split, [row for row in rows if row.get("split") == split]))
    for setup in ESTIMATION_SETUP_ORDER:
        splits.append((setup, [row for row in rows if row.get("setup") == setup]))

    axis_specs = [
        ("x", "m", "residual_x_m", "pred_std_x_m"),
        ("y", "m", "residual_y_m", "pred_std_y_m"),
        ("z", "m", "residual_z_m", "pred_std_z_m"),
        ("roll", "deg", "residual_roll_deg", "pred_std_roll_deg"),
        ("pitch", "deg", "residual_pitch_deg", "pred_std_pitch_deg"),
        ("yaw", "deg", "residual_yaw_deg", "pred_std_yaw_deg"),
    ]
    out: list[dict[str, Any]] = []
    for split_name, split_rows in splits:
        for axis, unit, residual_key, pred_key in axis_specs:
            values = [finite_float(row.get(residual_key)) for row in split_rows]
            preds = [finite_float(row.get(pred_key), default=float("nan")) for row in split_rows]
            pred_median = percentile(preds, 50.0)
            empirical = mad_std(values)
            out.append(
                {
                    "source": "offline_all_marker_current_model",
                    "split": split_name,
                    "axis": axis,
                    "unit": unit,
                    "sample_count": len(split_rows),
                    "empirical_mad_std": empirical,
                    "empirical_sample_std": std_or_default(values),
                    "predicted_median_std": pred_median,
                    "empirical_over_pred": empirical / pred_median if pred_median > 1e-12 else "",
                    "coverage_1sigma": coverage_fraction(values, preds, 1.0),
                    "coverage_2sigma": coverage_fraction(values, preds, 2.0),
                }
            )
    return out


def summarize_published_marker_quality(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[str(row.get("bag", ""))].append(row)

    out: list[dict[str, Any]] = []
    for bag, group in sorted(grouped.items()):
        if not group:
            continue
        first = group[0]
        accepted = [row for row in group if row.get("hard_gate_passed", True)]
        summary = {
            "bag": bag,
            "phase": first.get("phase", ""),
            "setup": first.get("setup", ""),
            "split": first.get("split", ""),
            "sample_count": len(group),
            "accepted_quality_count": len(accepted),
            "accepted_quality_fraction": len(accepted) / max(len(group), 1),
        }
        fields = [
            ("marker_distance_m", "m"),
            ("marker_side_mean_px", "px"),
            ("marker_reprojection_error_px", "px"),
            ("marker_view_angle_deg", "deg"),
            ("marker_covariance_camera_std_x_m", "m"),
            ("marker_covariance_camera_std_y_m", "m"),
            ("marker_covariance_camera_std_z_m", "m"),
            ("marker_covariance_camera_std_roll_deg", "deg"),
            ("marker_covariance_camera_std_pitch_deg", "deg"),
            ("marker_covariance_camera_std_yaw_deg", "deg"),
        ]
        for field_name, _unit in fields:
            values = [finite_float(row.get(field_name), default=float("nan")) for row in group]
            summary[f"median_{field_name}"] = percentile(values, 50.0)
            summary[f"p95_{field_name}"] = percentile(values, 95.0)
        out.append(summary)
    return out


def bool_fraction(rows: list[dict[str, Any]]) -> float:
    if not rows:
        return 0.0
    return sum(1 for row in rows if row["value"]) / len(rows)


def count_false_spans(rows: list[dict[str, Any]]) -> tuple[int, list[tuple[float, float]]]:
    spans: list[tuple[float, float]] = []
    in_false = False
    start = 0.0
    previous_stamp = 0.0
    for row in sorted(rows, key=lambda item: item["stamp"]):
        stamp = float(row["stamp"])
        if not row["value"] and not in_false:
            in_false = True
            start = stamp
        if row["value"] and in_false:
            spans.append((start, stamp))
            in_false = False
        previous_stamp = stamp
    if in_false:
        spans.append((start, previous_stamp))
    return len(spans), spans


def pose_series_motion_stats(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {
            "pose_sample_count": 0,
            "position_std_x_m": "",
            "position_std_y_m": "",
            "position_std_z_m": "",
            "p95_step_translation_m": "",
            "max_step_translation_m": "",
            "p95_step_rotation_deg": "",
            "max_step_rotation_deg": "",
        }

    rows = sorted(rows, key=lambda item: item["stamp"])
    positions = np.array([[row["x_m"], row["y_m"], row["z_m"]] for row in rows], dtype=float)
    step_trans: list[float] = []
    step_rot: list[float] = []
    for prev, cur in zip(rows[:-1], rows[1:]):
        step_trans.append(float(np.linalg.norm(positions[len(step_trans) + 1] - positions[len(step_trans)])))
        R_delta = cur["R"] @ prev["R"].T
        step_rot.append(math.degrees(float(np.linalg.norm(matrix_to_rotvec(R_delta)))))

    return {
        "pose_sample_count": len(rows),
        "position_std_x_m": std_or_default(positions[:, 0]),
        "position_std_y_m": std_or_default(positions[:, 1]),
        "position_std_z_m": std_or_default(positions[:, 2]),
        "p95_step_translation_m": percentile(step_trans, 95.0),
        "max_step_translation_m": max(step_trans) if step_trans else 0.0,
        "p95_step_rotation_deg": percentile(step_rot, 95.0),
        "max_step_rotation_deg": max(step_rot) if step_rot else 0.0,
    }


def imu_pose_covariance_coverage(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if len(rows) < 2:
        return {}
    positions = np.array([[row["x_m"], row["y_m"], row["z_m"]] for row in rows], dtype=float)
    rotations = [row["R"] for row in rows]
    mean_t = np.mean(positions, axis=0)
    mean_R = mean_rotation(rotations)

    residuals = {axis: [] for axis in AXES}
    predicted = {axis: [] for axis in AXES}
    for row, t, R in zip(rows, positions, rotations):
        trans_res = t - mean_t
        rot_res = matrix_to_rotvec(R @ mean_R.T)
        residuals["x"].append(float(trans_res[0]))
        residuals["y"].append(float(trans_res[1]))
        residuals["z"].append(float(trans_res[2]))
        residuals["roll"].append(math.degrees(float(rot_res[0])))
        residuals["pitch"].append(math.degrees(float(rot_res[1])))
        residuals["yaw"].append(math.degrees(float(rot_res[2])))
        predicted["x"].append(finite_float(row.get("std_x_m"), default=float("nan")))
        predicted["y"].append(finite_float(row.get("std_y_m"), default=float("nan")))
        predicted["z"].append(finite_float(row.get("std_z_m"), default=float("nan")))
        predicted["roll"].append(finite_float(row.get("std_roll_deg"), default=float("nan")))
        predicted["pitch"].append(finite_float(row.get("std_pitch_deg"), default=float("nan")))
        predicted["yaw"].append(finite_float(row.get("std_yaw_deg"), default=float("nan")))

    out: dict[str, Any] = {}
    for axis in AXES:
        out[f"imu_pose_coverage_1sigma_{axis}"] = coverage_fraction(residuals[axis], predicted[axis], 1.0)
        out[f"imu_pose_coverage_2sigma_{axis}"] = coverage_fraction(residuals[axis], predicted[axis], 2.0)
        out[f"imu_pose_empirical_mad_std_{axis}"] = mad_std(residuals[axis])
        out[f"imu_pose_median_pred_std_{axis}"] = percentile(predicted[axis], 50.0)
    return out


def summarize_validation(data: BagData) -> dict[str, Any]:
    info = data.info
    marker_false_count, marker_false_spans = count_false_spans(data.marker_valid)
    vio_false_count, vio_false_spans = count_false_spans(data.vio_valid)
    event_counter = Counter(
        (
            bool(event.get("correction_accepted", event.get("accepted", False))),
            str(event.get("correction_mode", "")),
            str(event.get("reason", "")),
        )
        for event in data.reanchor_events
    )
    accepted_modes = Counter(
        str(event.get("correction_mode", ""))
        for event in data.reanchor_events
        if bool(event.get("correction_accepted", event.get("accepted", False)))
    )
    hard_modes = {"manual_hard_reanchor", "vio_invalid_hard_reanchor"}
    unexpected_hard = [
        event
        for event in data.reanchor_events
        if bool(event.get("correction_accepted", event.get("accepted", False)))
        and str(event.get("correction_mode", "")) in hard_modes
    ]
    if info.setup != "final_stationary":
        unexpected_hard = [
            event
            for event in unexpected_hard
            if not (
                info.setup == "final_lost_regained_vio_drift"
                and str(event.get("correction_mode", "")) == "vio_invalid_hard_reanchor"
            )
        ]

    summary = {
        "bag": info.name,
        "setup": info.setup,
        "duration_s": info.duration_s,
        "all_marker_quality_count": info.topic_counts.get(ALL_MARKER_QUALITY_TOPIC, 0),
        "marker_quality_count": info.topic_counts.get(MARKER_QUALITY_TOPIC, 0),
        "imu_pose_count": info.topic_counts.get(IMU_POSE_TOPIC, 0),
        "corrected_odom_count": info.topic_counts.get(CORRECTED_ODOM_TOPIC, 0),
        "raw_odom_count": info.topic_counts.get(RAW_ODOM_TOPIC, 0),
        "marker_valid_fraction": bool_fraction(data.marker_valid),
        "marker_lost_span_count": marker_false_count,
        "marker_lost_total_s": sum(max(0.0, end - start) for start, end in marker_false_spans),
        "vio_valid_fraction": bool_fraction(data.vio_valid),
        "vio_invalid_span_count": vio_false_count,
        "vio_invalid_total_s": sum(max(0.0, end - start) for start, end in vio_false_spans),
        "reanchor_event_count": len(data.reanchor_events),
        "accepted_reanchor_event_count": sum(accepted_modes.values()),
        "accepted_modes": dict(accepted_modes),
        "event_mode_reason_counts": {
            f"{accepted}:{mode}:{reason}": count
            for (accepted, mode, reason), count in sorted(event_counter.items())
        },
        "unexpected_hard_reanchor_count": len(unexpected_hard),
        "active_marker_ids": dict(Counter(row["value"] for row in data.active_marker_id)),
    }
    summary.update({f"corrected_{k}": v for k, v in pose_series_motion_stats(data.corrected_odom).items()})
    summary.update({f"raw_{k}": v for k, v in pose_series_motion_stats(data.raw_odom).items()})
    summary.update({f"imu_pose_{k}": v for k, v in pose_series_motion_stats(data.imu_pose).items()})
    summary.update(imu_pose_covariance_coverage(data.imu_pose))
    return summary


def event_timeline_rows(data_sets: list[BagData]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for data in data_sets:
        if not data.reanchor_events:
            continue
        t0 = min(finite_float(event.get("stamp")) for event in data.reanchor_events)
        for event in sorted(data.reanchor_events, key=lambda item: finite_float(item.get("stamp"))):
            stamp = finite_float(event.get("stamp"))
            rows.append(
                {
                    "bag": data.info.name,
                    "setup": data.info.setup,
                    "stamp": stamp,
                    "stamp_rel_s": stamp - t0,
                    "marker_id": event.get("marker_id", ""),
                    "marker_valid": event.get("marker_valid", ""),
                    "vio_valid": event.get("vio_valid", ""),
                    "accepted": event.get("correction_accepted", event.get("accepted", "")),
                    "correction_mode": event.get("correction_mode", ""),
                    "reason": event.get("reason", ""),
                    "correction_translation_norm_m": event.get("correction_translation_norm_m", ""),
                    "correction_rotation_deg": event.get("correction_rotation_deg", ""),
                    "chi2": event.get("chi2", ""),
                    "marker_distance_m": event.get("marker_distance_m", ""),
                    "marker_reprojection_error_px": event.get("marker_reprojection_error_px", ""),
                }
            )
    return rows


def metadata_rows(infos: list[BagInfo]) -> list[dict[str, Any]]:
    keys = [
        ALL_MARKER_QUALITY_TOPIC,
        MARKER_QUALITY_TOPIC,
        MARKER_VALID_TOPIC,
        ACTIVE_MARKER_ID_TOPIC,
        REANCHOR_EVENT_TOPIC,
        VIO_VALID_TOPIC,
        CORRECTED_ODOM_TOPIC,
        RAW_ODOM_TOPIC,
        IMU_POSE_TOPIC,
        CAMERA_INFO_TOPIC,
    ]
    rows = []
    for info in infos:
        row = {
            "bag": info.name,
            "phase": info.phase,
            "setup": info.setup,
            "split": info.split,
            "duration_s": info.duration_s,
            "total_messages": info.total_messages,
        }
        for key in keys:
            row[key.strip("/").replace("/", "_")] = info.topic_counts.get(key, 0)
        rows.append(row)
    return rows


def config_gain_needed(target: float, floor: float, feature: float) -> Optional[float]:
    if feature <= 1e-12 or target <= floor:
        return None
    return max(0.0, (target - floor) / feature)


def conservative_recommendations(
    group_summaries: list[dict[str, Any]],
    cfg: CovarianceConfig,
) -> tuple[dict[str, Any], list[str]]:
    calibration = [row for row in group_summaries if row.get("split") == "calibration"]
    notes: list[str] = []
    candidate = {
        "min_marker_xy_std_m": cfg.min_marker_xy_std_m,
        "min_marker_z_std_m": cfg.min_marker_z_std_m,
        "min_marker_roll_pitch_std_deg": cfg.min_marker_roll_pitch_std_deg,
        "min_marker_yaw_std_deg": cfg.min_marker_yaw_std_deg,
        "marker_xy_std_px_gain": cfg.marker_xy_std_px_gain,
        "marker_z_std_px_gain": cfg.marker_z_std_px_gain,
        "marker_roll_pitch_std_px_gain": cfg.marker_roll_pitch_std_px_gain,
        "marker_yaw_std_px_gain": cfg.marker_yaw_std_px_gain,
    }
    if not calibration:
        notes.append("No calibration split rows were available, so no tuning recommendation was generated.")
        return candidate, notes

    xy_needed: list[float] = []
    z_needed: list[float] = []
    rp_needed: list[float] = []
    yaw_needed: list[float] = []
    for row in calibration:
        distance = finite_float(row.get("median_distance_m"))
        sigma_px = max(cfg.corner_noise_floor_px, finite_float(row.get("median_reprojection_error_px")))
        side = max(finite_float(row.get("median_side_mean_px"), 1.0), 1.0)
        f_avg = max(finite_float(row.get("camera_f_avg_px"), 600.0), 1.0)
        view_penalty = covariance_view_penalty(finite_float(row.get("median_view_angle_deg")), cfg)

        xy_target = max(finite_float(row.get("mad_std_x_m")), finite_float(row.get("mad_std_y_m")))
        z_target = finite_float(row.get("mad_std_z_m"))
        rp_target_rad = math.radians(
            max(finite_float(row.get("mad_std_roll_deg")), finite_float(row.get("mad_std_pitch_deg")))
        )
        yaw_target_rad = math.radians(finite_float(row.get("mad_std_yaw_deg")))

        xy_gain = config_gain_needed(xy_target, cfg.min_marker_xy_std_m, distance * sigma_px / f_avg)
        z_gain = config_gain_needed(z_target, cfg.min_marker_z_std_m, distance * sigma_px / side * view_penalty)
        rp_gain = config_gain_needed(
            rp_target_rad,
            math.radians(cfg.min_marker_roll_pitch_std_deg),
            sigma_px / side * view_penalty,
        )
        yaw_gain = config_gain_needed(
            yaw_target_rad,
            math.radians(cfg.min_marker_yaw_std_deg),
            sigma_px / side,
        )
        if xy_gain is not None:
            xy_needed.append(xy_gain)
        if z_gain is not None:
            z_needed.append(z_gain)
        if rp_gain is not None:
            rp_needed.append(rp_gain)
        if yaw_gain is not None:
            yaw_needed.append(yaw_gain)

    needed = {
        "marker_xy_std_px_gain": xy_needed,
        "marker_z_std_px_gain": z_needed,
        "marker_roll_pitch_std_px_gain": rp_needed,
        "marker_yaw_std_px_gain": yaw_needed,
    }
    current = {
        "marker_xy_std_px_gain": cfg.marker_xy_std_px_gain,
        "marker_z_std_px_gain": cfg.marker_z_std_px_gain,
        "marker_roll_pitch_std_px_gain": cfg.marker_roll_pitch_std_px_gain,
        "marker_yaw_std_px_gain": cfg.marker_yaw_std_px_gain,
    }
    for key, values in needed.items():
        if not values:
            notes.append(f"{key}: current floor/gain already covers calibration MAD targets; keep current value.")
            continue
        required = percentile(values, 90.0)
        if required > current[key] * 1.10:
            candidate[key] = required
            notes.append(f"{key}: increase from {current[key]:.4g} to {required:.4g} to cover calibration targets.")
        else:
            notes.append(f"{key}: keep {current[key]:.4g}; calibration target requires at most {required:.4g}.")

    notes.append("Floors are not lowered by this tool. If the report shows severe overconfidence, lower floors manually only after holdout review.")
    return candidate, notes


def markdown_table(rows: list[dict[str, Any]], columns: list[str], max_rows: int = 20) -> str:
    if not rows:
        return "_No rows._"
    selected = rows[:max_rows]
    header = "| " + " | ".join(columns) + " |"
    sep = "| " + " | ".join("---" for _ in columns) + " |"
    body = []
    for row in selected:
        body.append("| " + " | ".join(str(csv_value(row.get(col, ""))) for col in columns) + " |")
    if len(rows) > max_rows:
        more_cells = ["...", f"{len(rows) - max_rows} more rows"] + [""] * max(0, len(columns) - 2)
        body.append("| " + " | ".join(more_cells) + " |")
    return "\n".join([header, sep] + body)


def write_report(
    output_dir: Path,
    infos: list[BagInfo],
    stationary: list[dict[str, Any]],
    coverage: list[dict[str, Any]],
    validation: list[dict[str, Any]],
    recommendation: dict[str, Any],
    recommendation_notes: list[str],
) -> None:
    metadata = metadata_rows(infos)
    readiness_notes = []
    final_stationary = next(
        (row for row in metadata if row["setup"] == "final_stationary"),
        None,
    )
    if final_stationary and final_stationary.get("head_marker_pose_all_marker_quality", 0) == 0:
        readiness_notes.append(
            "`final_stationary` has 0 `/head/marker_pose/all_marker_quality` messages; use `/marker_quality` and `/imu_pose` for that bag."
        )

    lines = [
        "# Phase 1 Marker Covariance Bag Analysis",
        "",
        "Generated by `analyze_marker_covariance_bags.py`.",
        "",
        "## Metadata Readiness",
        "",
        *[f"- {note}" for note in readiness_notes],
        "",
        markdown_table(
            metadata,
            [
                "bag",
                "setup",
                "split",
                "duration_s",
                "head_marker_pose_all_marker_quality",
                "head_marker_pose_marker_quality",
                "head_marker_pose_ov_corrected_odom",
                "ov_msckf_odomimu",
            ],
            max_rows=20,
        ),
        "",
        "## Stationary Repeatability",
        "",
        markdown_table(
            stationary,
            [
                "bag",
                "marker_id",
                "split",
                "sample_count",
                "median_distance_m",
                "median_side_mean_px",
                "mad_std_x_m",
                "mad_std_y_m",
                "mad_std_z_m",
                "mad_std_roll_deg",
                "mad_std_pitch_deg",
                "mad_std_yaw_deg",
                "p95_translation_error_norm_m",
                "p95_rotation_error_angle_deg",
            ],
            max_rows=18,
        ),
        "",
        "## Predicted Vs Observed Coverage",
        "",
        markdown_table(
            coverage,
            [
                "split",
                "axis",
                "sample_count",
                "empirical_mad_std",
                "predicted_median_std",
                "empirical_over_pred",
                "coverage_1sigma",
                "coverage_2sigma",
            ],
            max_rows=36,
        ),
        "",
        "## Validation Bags",
        "",
        markdown_table(
            validation,
            [
                "bag",
                "setup",
                "marker_valid_fraction",
                "marker_lost_span_count",
                "vio_valid_fraction",
                "accepted_reanchor_event_count",
                "unexpected_hard_reanchor_count",
                "corrected_p95_step_translation_m",
                "corrected_max_step_translation_m",
                "corrected_p95_step_rotation_deg",
            ],
            max_rows=10,
        ),
        "",
        "## Conservative Config Recommendation",
        "",
        "```yaml",
        "covariance:",
    ]
    for key, value in recommendation.items():
        lines.append(f"  {key}: {value:.6g}" if isinstance(value, float) else f"  {key}: {value}")
    lines.extend(
        [
            "```",
            "",
            *[f"- {note}" for note in recommendation_notes],
            "",
            "Detailed CSV outputs are in this directory.",
            "",
        ]
    )
    (output_dir / "analysis_report.md").write_text("\n".join(lines), encoding="utf-8")


def default_bag_root() -> Path:
    candidates = [
        Path("/miahand_ws/src/bags/openvins_tests/head_marker_covariance"),
        Path.cwd() / "bags" / "openvins_tests" / "head_marker_covariance",
        Path.cwd() / "docker_ws" / "bags" / "openvins_tests" / "head_marker_covariance",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def default_config_path() -> Path:
    script_dir = Path(__file__).resolve().parent
    candidates = [
        Path("/miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml"),
        script_dir.parent / "config" / "markers" / "head_aruco_map.yaml",
        Path.cwd()
        / "docker_ws"
        / "multi_cam_localization"
        / "sensor_fusion_bringup"
        / "config"
        / "markers"
        / "head_aruco_map.yaml",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-root", type=Path, default=default_bag_root())
    parser.add_argument("--config", type=Path, default=default_config_path())
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--fallback-f-avg-px", type=float, default=600.0)
    parser.add_argument("--min-bin-samples", type=int, default=80)
    parser.add_argument("--calibration-replicates", type=int, default=2)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bag_root = args.bag_root.resolve()
    output_dir = args.output_dir or (bag_root / "analysis_phase1_marker_covariance")
    output_dir.mkdir(parents=True, exist_ok=True)

    marker_config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    covariance_cfg = CovarianceConfig.from_marker_config(marker_config)

    infos = discover_bags(bag_root, args.calibration_replicates)
    write_csv(output_dir / "metadata_summary.csv", metadata_rows(infos))

    data_sets: list[BagData] = []
    for info in infos:
        print(f"Reading {info.name}...")
        data_sets.append(extract_bag_data(info))

    all_marker_rows = [row for data in data_sets for row in data.all_marker_rows if data.info.phase == "estimation"]
    marker_quality_rows = [row for data in data_sets for row in data.marker_quality_rows]

    add_prediction_fields(all_marker_rows, covariance_cfg, args.fallback_f_avg_px)
    add_stationary_residuals(all_marker_rows)

    stationary = summarize_stationary_groups(all_marker_rows)
    bins = summarize_bins(all_marker_rows, args.min_bin_samples)
    coverage = coverage_summary(all_marker_rows)
    quality_summary = summarize_published_marker_quality(marker_quality_rows)
    validation = [summarize_validation(data) for data in data_sets if data.info.phase == "validation"]
    timeline = event_timeline_rows([data for data in data_sets if data.info.phase == "validation"])
    recommendation, recommendation_notes = conservative_recommendations(stationary, covariance_cfg)

    write_csv(output_dir / "all_marker_detections.csv", all_marker_rows)
    write_csv(output_dir / "stationary_repeatability.csv", stationary)
    write_csv(output_dir / "binned_empirical_covariance.csv", bins)
    write_csv(output_dir / "prediction_coverage.csv", coverage)
    write_csv(output_dir / "published_marker_quality_summary.csv", quality_summary)
    write_csv(output_dir / "validation_summary.csv", validation)
    write_csv(output_dir / "validation_event_timeline.csv", timeline)
    write_csv(output_dir / "recommended_covariance_config.csv", [recommendation])
    write_report(output_dir, infos, stationary, coverage, validation, recommendation, recommendation_notes)

    print(f"Wrote analysis outputs to {output_dir}")


if __name__ == "__main__":
    main()
