#!/usr/bin/env python3

"""Debug-only head-derived preview of the arm D435i camera pose.

This node intentionally does not publish MarkerPoseObservation and does not
feed OpenVINS. It only visualizes a candidate arm camera pose when the head
camera observes the arm-mounted dynamic marker.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Optional

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_fusion_msgs.msg import DynamicMarkerObservation
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from aruco_marker_pose_node import (
    R_to_rotvec,
    T_cam_body_display,
    T_inv,
    fill_pose,
    load_kalibr_imucam,
    quat_to_R_xyzw,
    rot_to_quat_xyzw,
    rotvec_to_R,
    stamp_to_sec,
)


RESIDUAL_ORDER = ["x", "y", "z", "roll", "pitch", "yaw"]


@dataclass
class ArmMarkerExtrinsic:
    marker_id: int
    marker_frame: str
    parent_camera_frame: str
    parent_imu_frame: str
    T_armcam_marker: np.ndarray
    covariance: np.ndarray
    covariance_source: str


@dataclass
class HeadPoseSample:
    stamp: Any
    stamp_sec: float
    T_map_headimu: np.ndarray
    covariance: np.ndarray


@dataclass
class MatchedHeadPose:
    stamp: Any
    stamp_sec: float
    T_map_headimu: np.ndarray
    covariance: np.ndarray
    dt_s: float
    mode: str


@dataclass
class CandidatePose:
    T_map_headcam: np.ndarray
    T_map_marker: np.ndarray
    T_map_armcam: np.ndarray
    covariance: np.ndarray


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
        candidate = Path.cwd() / path_text[len(miahand_prefix) :]
        if candidate.exists():
            return candidate
    return path


def se3_exp(delta: np.ndarray) -> np.ndarray:
    values = np.asarray(delta, dtype=float).reshape(6)
    T = np.eye(4)
    T[:3, 3] = values[:3]
    T[:3, :3] = rotvec_to_R(values[3:])
    return T


def se3_left_residual(T_ref: np.ndarray, T_sample: np.ndarray) -> np.ndarray:
    delta = np.asarray(T_sample, dtype=float).reshape(4, 4) @ T_inv(np.asarray(T_ref, dtype=float).reshape(4, 4))
    return np.concatenate([delta[:3, 3], R_to_rotvec(delta[:3, :3])])


def pose_to_T(pose_msg) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_to_R_xyzw(pose_msg.orientation)
    T[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
    return T


def quaternion_norm(pose_msg) -> float:
    q = pose_msg.orientation
    return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)


def is_finite_transform(T: np.ndarray) -> bool:
    return bool(np.asarray(T).shape == (4, 4) and np.all(np.isfinite(T)))


def covariance_matrix_from_ros(
    covariance_values: Any,
    fallback_diag: np.ndarray,
) -> tuple[np.ndarray, bool]:
    P = np.asarray(covariance_values, dtype=float).reshape(6, 6)
    if not np.all(np.isfinite(P)) or np.any(np.diag(P) < 0.0):
        return np.diag(np.asarray(fallback_diag, dtype=float).reshape(6)), True
    P = 0.5 * (P + P.T)
    if float(np.trace(P)) <= 0.0:
        return np.diag(np.asarray(fallback_diag, dtype=float).reshape(6)), True
    return P, False


def sanitize_covariance(P: np.ndarray, min_diag: np.ndarray) -> np.ndarray:
    P = np.asarray(P, dtype=float).reshape(6, 6)
    if not np.all(np.isfinite(P)):
        return np.diag(np.asarray(min_diag, dtype=float).reshape(6))
    P = 0.5 * (P + P.T)
    diag = np.maximum(np.diag(P), np.asarray(min_diag, dtype=float).reshape(6))
    P[np.diag_indices(6)] = diag
    return P


def matrix_from_yaml(value: Any, name: str, shape: tuple[int, int]) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.shape != shape or not np.all(np.isfinite(arr)):
        raise RuntimeError(f"{name} must be a finite {shape[0]}x{shape[1]} matrix")
    return arr


def load_head_camera_transform(head_config_path: Path) -> tuple[str, str, np.ndarray]:
    config = yaml.safe_load(head_config_path.read_text(encoding="utf-8"))
    frames = config.get("frames", {})
    camera_frame = str(frames.get("detected_camera_frame", frames.get("camera_frame", "")))
    map_frame = str(frames.get("map_frame", "marker_map"))
    calib_path = resolve_path(str(config["calibration"]["kalibr_imucam_chain"]), head_config_path)
    _K, _D, T_headcam_headimu, _timeshift = load_kalibr_imucam(str(calib_path))
    return map_frame, camera_frame, T_headcam_headimu


def load_arm_marker_extrinsic(
    extrinsics_path: Path,
    marker_id: int,
    covariance_source: str,
    fallback_covariance_diag: np.ndarray,
) -> ArmMarkerExtrinsic:
    config = yaml.safe_load(extrinsics_path.read_text(encoding="utf-8"))
    markers = config.get("arm_mounted_markers", {})
    marker_cfg = markers.get(marker_id, markers.get(str(marker_id)))
    if marker_cfg is None:
        raise RuntimeError(f"Marker ID {marker_id} is missing from {extrinsics_path}")

    T_armcam_marker = matrix_from_yaml(marker_cfg["T_armcam_marker"], "T_armcam_marker", (4, 4))
    estimate = marker_cfg.get("estimate", {})
    residual_order = estimate.get("residual_order", RESIDUAL_ORDER)
    if list(residual_order) != RESIDUAL_ORDER:
        raise RuntimeError(f"Unsupported extrinsic residual order: {residual_order}")

    source = covariance_source
    cov_value = estimate.get(source)
    if cov_value is None and source != "robust_diag_covariance_se3":
        source = "robust_diag_covariance_se3"
        cov_value = estimate.get(source)
    if cov_value is None:
        source = "residual_covariance_se3"
        cov_value = estimate.get(source)
    if cov_value is None:
        source = "fallback_diag"
        covariance = np.diag(np.asarray(fallback_covariance_diag, dtype=float).reshape(6))
    else:
        covariance = matrix_from_yaml(cov_value, source, (6, 6))

    covariance = sanitize_covariance(covariance, fallback_covariance_diag)
    return ArmMarkerExtrinsic(
        marker_id=int(marker_id),
        marker_frame=str(marker_cfg.get("marker_frame", f"arm_marker_{marker_id}")),
        parent_camera_frame=str(marker_cfg.get("parent_camera_frame", "")),
        parent_imu_frame=str(marker_cfg.get("parent_imu_frame", "")),
        T_armcam_marker=T_armcam_marker,
        covariance=covariance,
        covariance_source=source,
    )


def interpolate_transform(T_a: np.ndarray, T_b: np.ndarray, alpha: float) -> np.ndarray:
    a = max(0.0, min(1.0, float(alpha)))
    T = np.eye(4)
    T[:3, 3] = (1.0 - a) * T_a[:3, 3] + a * T_b[:3, 3]
    R_delta = T_a[:3, :3].T @ T_b[:3, :3]
    T[:3, :3] = T_a[:3, :3] @ rotvec_to_R(a * R_to_rotvec(R_delta))
    return T


def match_head_pose(
    samples: list[HeadPoseSample],
    stamp_sec: float,
    max_dt_s: float,
) -> Optional[MatchedHeadPose]:
    if not samples:
        return None

    before = None
    after = None
    for sample in samples:
        if sample.stamp_sec <= stamp_sec:
            before = sample
        elif sample.stamp_sec > stamp_sec:
            after = sample
            break

    if before is not None and after is not None:
        before_dt = stamp_sec - before.stamp_sec
        after_dt = after.stamp_sec - stamp_sec
        if before_dt >= 0.0 and after_dt >= 0.0 and max(before_dt, after_dt) <= max_dt_s:
            span = max(after.stamp_sec - before.stamp_sec, 1e-12)
            alpha = before_dt / span
            return MatchedHeadPose(
                stamp=before.stamp,
                stamp_sec=stamp_sec,
                T_map_headimu=interpolate_transform(before.T_map_headimu, after.T_map_headimu, alpha),
                covariance=(1.0 - alpha) * before.covariance + alpha * after.covariance,
                dt_s=max(before_dt, after_dt),
                mode="interpolated",
            )

    nearest = min(samples, key=lambda sample: abs(sample.stamp_sec - stamp_sec))
    dt = abs(nearest.stamp_sec - stamp_sec)
    if dt > max_dt_s:
        return None
    return MatchedHeadPose(
        stamp=nearest.stamp,
        stamp_sec=nearest.stamp_sec,
        T_map_headimu=nearest.T_map_headimu,
        covariance=nearest.covariance,
        dt_s=dt,
        mode="nearest",
    )


def compose_head_derived_arm_pose(
    T_map_headimu: np.ndarray,
    T_headcam_headimu: np.ndarray,
    T_headcam_marker: np.ndarray,
    T_armcam_marker: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    T_map_headcam = T_map_headimu @ T_inv(T_headcam_headimu)
    T_map_marker = T_map_headcam @ T_headcam_marker
    T_map_armcam = T_map_marker @ T_inv(T_armcam_marker)
    return T_map_headcam, T_map_marker, T_map_armcam


def propagate_candidate_covariance(
    T_map_headimu: np.ndarray,
    P_head: np.ndarray,
    T_headcam_headimu: np.ndarray,
    T_headcam_marker: np.ndarray,
    P_dynamic: np.ndarray,
    T_armcam_marker: np.ndarray,
    P_extrinsic: np.ndarray,
    min_diag: np.ndarray,
    epsilon: float = 1e-4,
) -> np.ndarray:
    _T_map_headcam, _T_map_marker, T_nominal = compose_head_derived_arm_pose(
        T_map_headimu,
        T_headcam_headimu,
        T_headcam_marker,
        T_armcam_marker,
    )

    def jacobian(which: str) -> np.ndarray:
        J = np.zeros((6, 6), dtype=float)
        for axis in range(6):
            delta = np.zeros(6, dtype=float)
            delta[axis] = float(epsilon)
            dT = se3_exp(delta)
            head = T_map_headimu
            dynamic = T_headcam_marker
            extrinsic = T_armcam_marker
            if which == "head":
                head = dT @ head
            elif which == "dynamic":
                dynamic = dT @ dynamic
            elif which == "extrinsic":
                extrinsic = dT @ extrinsic
            else:
                raise RuntimeError(f"Unknown covariance source {which}")
            _T_headcam, _T_marker, T_perturbed = compose_head_derived_arm_pose(
                head,
                T_headcam_headimu,
                dynamic,
                extrinsic,
            )
            J[:, axis] = se3_left_residual(T_nominal, T_perturbed) / float(epsilon)
        return J

    J_head = jacobian("head")
    J_dynamic = jacobian("dynamic")
    J_extrinsic = jacobian("extrinsic")
    P = (
        J_head @ P_head @ J_head.T
        + J_dynamic @ P_dynamic @ J_dynamic.T
        + J_extrinsic @ P_extrinsic @ J_extrinsic.T
    )
    return sanitize_covariance(P, min_diag)


def covariance_diag_std(P: np.ndarray) -> np.ndarray:
    return np.sqrt(np.maximum(np.diag(np.asarray(P, dtype=float).reshape(6, 6)), 0.0))


def rotation_angle_deg(R: np.ndarray) -> float:
    cos_angle = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) * 0.5))
    return math.degrees(math.acos(cos_angle))


class HeadDerivedArmPosePreviewNode(Node):
    def __init__(self):
        super().__init__("head_derived_arm_pose_preview_node")

        self.declare_parameter("head_marker_config", "")
        self.declare_parameter("arm_marker_extrinsics", "")
        self.declare_parameter("dynamic_observation_topic", "/head/marker_pose/dynamic_observation")
        self.declare_parameter("head_pose_topic", "/ov_msckf/poseimu")
        self.declare_parameter("output_prefix", "/arm/marker_pose/head_derived")
        self.declare_parameter("marker_id", 2)
        self.declare_parameter("map_frame", "marker_map")
        self.declare_parameter("max_head_pose_dt_s", 0.05)
        self.declare_parameter("head_pose_buffer_seconds", 5.0)
        self.declare_parameter("require_stable_dynamic_marker", True)
        self.declare_parameter("max_pose_covariance_trace", 10.0)
        self.declare_parameter("max_reprojection_error_px", 3.0)
        self.declare_parameter("max_marker_distance_m", 2.0)
        self.declare_parameter("max_view_angle_deg", 75.0)
        self.declare_parameter("min_marker_area_px2", 800.0)
        self.declare_parameter("min_geometry_score", 0.35)
        self.declare_parameter("max_preview_translation_jump_m", 0.50)
        self.declare_parameter("max_preview_rotation_jump_deg", 45.0)
        self.declare_parameter("jump_reset_gap_s", 1.0)
        self.declare_parameter("path_length", 300)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("extrinsic_covariance_source", "robust_diag_covariance_se3")
        self.declare_parameter("default_head_position_std_m", 0.20)
        self.declare_parameter("default_head_rotation_std_deg", 20.0)
        self.declare_parameter("default_dynamic_position_std_m", 0.05)
        self.declare_parameter("default_dynamic_rotation_std_deg", 8.0)
        self.declare_parameter("default_extrinsic_position_std_m", 0.03)
        self.declare_parameter("default_extrinsic_rotation_std_deg", 5.0)
        self.declare_parameter("covariance_epsilon", 1e-4)

        head_config = str(self.get_parameter("head_marker_config").value)
        extrinsics_path = str(self.get_parameter("arm_marker_extrinsics").value)
        if not head_config:
            raise RuntimeError("Parameter head_marker_config is required")
        if not extrinsics_path:
            raise RuntimeError("Parameter arm_marker_extrinsics is required")

        self.marker_id = int(self.get_parameter("marker_id").value)
        self.output_prefix = str(self.get_parameter("output_prefix").value).rstrip("/")
        self.dynamic_topic = str(self.get_parameter("dynamic_observation_topic").value)
        self.head_pose_topic = str(self.get_parameter("head_pose_topic").value)
        self.max_head_pose_dt_s = float(self.get_parameter("max_head_pose_dt_s").value)
        self.head_pose_buffer_seconds = float(self.get_parameter("head_pose_buffer_seconds").value)
        self.require_stable_dynamic_marker = bool(self.get_parameter("require_stable_dynamic_marker").value)
        self.max_pose_covariance_trace = float(self.get_parameter("max_pose_covariance_trace").value)
        self.max_reprojection_error_px = float(self.get_parameter("max_reprojection_error_px").value)
        self.max_marker_distance_m = float(self.get_parameter("max_marker_distance_m").value)
        self.max_view_angle_deg = float(self.get_parameter("max_view_angle_deg").value)
        self.min_marker_area_px2 = float(self.get_parameter("min_marker_area_px2").value)
        self.min_geometry_score = float(self.get_parameter("min_geometry_score").value)
        self.max_preview_translation_jump_m = float(self.get_parameter("max_preview_translation_jump_m").value)
        self.max_preview_rotation_jump_deg = float(self.get_parameter("max_preview_rotation_jump_deg").value)
        self.jump_reset_gap_s = float(self.get_parameter("jump_reset_gap_s").value)
        self.path_length = max(1, int(self.get_parameter("path_length").value))
        self.publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self.covariance_epsilon = float(self.get_parameter("covariance_epsilon").value)

        self.head_fallback_diag = self.default_covariance_diag(
            float(self.get_parameter("default_head_position_std_m").value),
            float(self.get_parameter("default_head_rotation_std_deg").value),
        )
        self.dynamic_fallback_diag = self.default_covariance_diag(
            float(self.get_parameter("default_dynamic_position_std_m").value),
            float(self.get_parameter("default_dynamic_rotation_std_deg").value),
        )
        self.extrinsic_fallback_diag = self.default_covariance_diag(
            float(self.get_parameter("default_extrinsic_position_std_m").value),
            float(self.get_parameter("default_extrinsic_rotation_std_deg").value),
        )
        self.output_min_diag = np.minimum.reduce(
            [self.head_fallback_diag, self.dynamic_fallback_diag, self.extrinsic_fallback_diag]
        ) * 1e-3

        head_config_path = resolve_path(head_config)
        self.map_frame, self.head_camera_frame, self.T_headcam_headimu = load_head_camera_transform(head_config_path)
        map_frame_override = str(self.get_parameter("map_frame").value)
        if map_frame_override:
            self.map_frame = map_frame_override

        self.extrinsic = load_arm_marker_extrinsic(
            resolve_path(extrinsics_path),
            self.marker_id,
            str(self.get_parameter("extrinsic_covariance_source").value),
            self.extrinsic_fallback_diag,
        )

        self.arm_preview_frame = f"{self.extrinsic.parent_camera_frame}_head_preview"
        self.arm_body_preview_frame = f"{self.extrinsic.parent_camera_frame}_head_preview_body_display"
        self.marker_preview_frame = f"{self.extrinsic.marker_frame}_head_preview"

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=20)
        self.dynamic_sub = self.create_subscription(
            DynamicMarkerObservation,
            self.dynamic_topic,
            self.dynamic_cb,
            qos,
        )
        self.head_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.head_pose_topic,
            self.head_pose_cb,
            qos,
        )

        self.arm_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            f"{self.output_prefix}/arm_camera_pose",
            10,
        )
        self.arm_body_pose_pub = self.create_publisher(PoseStamped, f"{self.output_prefix}/arm_camera_body_pose", 10)
        self.marker_pose_pub = self.create_publisher(PoseStamped, f"{self.output_prefix}/arm_marker_pose", 10)
        self.path_pub = self.create_publisher(PathMsg, f"{self.output_prefix}/path", 10)
        self.status_pub = self.create_publisher(String, f"{self.output_prefix}/status", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.head_buffer: deque[HeadPoseSample] = deque()
        self.path_msg = PathMsg()
        self.path_msg.header.frame_id = self.map_frame
        self.last_candidate_stamp_sec: Optional[float] = None
        self.last_T_map_armcam: Optional[np.ndarray] = None

        self.get_logger().info(f"Dynamic observation topic: {self.dynamic_topic}")
        self.get_logger().info(f"Head pose topic: {self.head_pose_topic}")
        self.get_logger().info(f"Output prefix: {self.output_prefix}")
        self.get_logger().info(f"Marker ID: {self.marker_id}")
        self.get_logger().info(f"Head camera frame: {self.head_camera_frame}")
        self.get_logger().info(f"Arm preview frame: {self.arm_preview_frame}")
        self.get_logger().info(f"Extrinsic covariance source: {self.extrinsic.covariance_source}")

    @staticmethod
    def default_covariance_diag(position_std_m: float, rotation_std_deg: float) -> np.ndarray:
        return np.array(
            [
                float(position_std_m) ** 2,
                float(position_std_m) ** 2,
                float(position_std_m) ** 2,
                math.radians(float(rotation_std_deg)) ** 2,
                math.radians(float(rotation_std_deg)) ** 2,
                math.radians(float(rotation_std_deg)) ** 2,
            ],
            dtype=float,
        )

    def head_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        if msg.header.frame_id != self.map_frame:
            self.publish_status(False, "head_pose_wrong_frame", stamp_to_sec(msg.header.stamp), {})
            return
        q_norm = quaternion_norm(msg.pose.pose)
        if not math.isfinite(q_norm) or q_norm < 1e-9:
            self.publish_status(False, "head_pose_bad_quaternion", stamp_to_sec(msg.header.stamp), {})
            return
        T_map_headimu = pose_to_T(msg.pose.pose)
        if not is_finite_transform(T_map_headimu):
            self.publish_status(False, "head_pose_non_finite", stamp_to_sec(msg.header.stamp), {})
            return
        P_head, used_fallback = covariance_matrix_from_ros(msg.pose.covariance, self.head_fallback_diag)
        if float(np.trace(P_head)) > self.max_pose_covariance_trace:
            self.publish_status(
                False,
                "head_pose_covariance_too_large",
                stamp_to_sec(msg.header.stamp),
                {"head_pose_covariance_trace": round(float(np.trace(P_head)), 6)},
            )
            return

        stamp_sec = stamp_to_sec(msg.header.stamp)
        self.head_buffer.append(
            HeadPoseSample(
                stamp=msg.header.stamp,
                stamp_sec=stamp_sec,
                T_map_headimu=T_map_headimu,
                covariance=P_head,
            )
        )
        self.prune_head_buffer(stamp_sec)
        if used_fallback:
            self.publish_status(False, "head_pose_covariance_fallback", stamp_sec, {})

    def prune_head_buffer(self, latest_stamp_sec: float) -> None:
        cutoff = latest_stamp_sec - self.head_pose_buffer_seconds
        while self.head_buffer and self.head_buffer[0].stamp_sec < cutoff:
            self.head_buffer.popleft()

    def dynamic_cb(self, msg: DynamicMarkerObservation) -> None:
        stamp_sec = stamp_to_sec(msg.header.stamp)
        reason = self.dynamic_gate_reason(msg)
        if reason:
            self.publish_status(False, reason, stamp_sec, self.dynamic_status_fields(msg))
            return

        matched = match_head_pose(list(self.head_buffer), stamp_sec, self.max_head_pose_dt_s)
        if matched is None:
            self.publish_status(False, "no_head_pose_match", stamp_sec, self.dynamic_status_fields(msg))
            return

        T_headcam_marker = pose_to_T(msg.pose.pose)
        if not is_finite_transform(T_headcam_marker):
            self.publish_status(False, "dynamic_pose_non_finite", stamp_sec, self.dynamic_status_fields(msg))
            return
        P_dynamic, dynamic_cov_fallback = covariance_matrix_from_ros(msg.pose.covariance, self.dynamic_fallback_diag)
        _T_map_headcam, _T_map_marker, T_map_armcam = compose_head_derived_arm_pose(
            matched.T_map_headimu,
            self.T_headcam_headimu,
            T_headcam_marker,
            self.extrinsic.T_armcam_marker,
        )
        jump_reason = self.preview_jump_reason(stamp_sec, T_map_armcam)
        if jump_reason:
            fields = self.dynamic_status_fields(msg)
            fields.update({"head_pose_match_dt_s": round(float(matched.dt_s), 6), "head_pose_match_mode": matched.mode})
            self.publish_status(False, jump_reason, stamp_sec, fields)
            return

        covariance = propagate_candidate_covariance(
            matched.T_map_headimu,
            matched.covariance,
            self.T_headcam_headimu,
            T_headcam_marker,
            P_dynamic,
            self.extrinsic.T_armcam_marker,
            self.extrinsic.covariance,
            self.output_min_diag,
            epsilon=self.covariance_epsilon,
        )
        T_map_headcam, T_map_marker, T_map_armcam = compose_head_derived_arm_pose(
            matched.T_map_headimu,
            self.T_headcam_headimu,
            T_headcam_marker,
            self.extrinsic.T_armcam_marker,
        )
        candidate = CandidatePose(
            T_map_headcam=T_map_headcam,
            T_map_marker=T_map_marker,
            T_map_armcam=T_map_armcam,
            covariance=covariance,
        )
        self.publish_candidate(msg, candidate, matched, dynamic_cov_fallback)

    def dynamic_gate_reason(self, msg: DynamicMarkerObservation) -> str:
        if int(msg.marker_id) != self.marker_id:
            return "wrong_marker_id"
        if msg.header.frame_id != self.head_camera_frame or msg.camera_frame != self.head_camera_frame:
            return "wrong_dynamic_camera_frame"
        if msg.marker_frame != self.extrinsic.marker_frame:
            return "wrong_dynamic_marker_frame"
        if not bool(msg.hard_gate_passed):
            return "dynamic_hard_gate_failed"
        if self.require_stable_dynamic_marker and not bool(msg.stable):
            return "dynamic_marker_not_stable"
        if float(msg.reprojection_error_px) > self.max_reprojection_error_px:
            return "dynamic_reprojection_error_too_high"
        if float(msg.distance_m) > self.max_marker_distance_m:
            return "dynamic_marker_too_far"
        if float(msg.view_angle_deg) > self.max_view_angle_deg:
            return "dynamic_view_angle_too_high"
        if float(msg.area_px2) < self.min_marker_area_px2:
            return "dynamic_marker_area_too_small"
        if float(msg.geometry_score) < self.min_geometry_score:
            return "dynamic_geometry_score_too_low"
        return ""

    def preview_jump_reason(self, stamp_sec: float, T_map_armcam: np.ndarray) -> str:
        if self.last_T_map_armcam is None or self.last_candidate_stamp_sec is None:
            return ""
        dt = max(0.0, float(stamp_sec) - float(self.last_candidate_stamp_sec))
        if dt > self.jump_reset_gap_s:
            return ""
        translation_jump = float(np.linalg.norm(T_map_armcam[:3, 3] - self.last_T_map_armcam[:3, 3]))
        rotation_jump = rotation_angle_deg(T_map_armcam[:3, :3] @ self.last_T_map_armcam[:3, :3].T)
        if translation_jump > self.max_preview_translation_jump_m:
            return "preview_translation_jump"
        if rotation_jump > self.max_preview_rotation_jump_deg:
            return "preview_rotation_jump"
        return ""

    def publish_candidate(
        self,
        msg: DynamicMarkerObservation,
        candidate: CandidatePose,
        matched: MatchedHeadPose,
        dynamic_cov_fallback: bool,
    ) -> None:
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = self.map_frame
        fill_pose(pose_msg.pose.pose, candidate.T_map_armcam)
        pose_msg.pose.covariance = list(np.asarray(candidate.covariance, dtype=float).reshape(36))
        self.arm_pose_pub.publish(pose_msg)

        T_map_body = candidate.T_map_armcam @ T_cam_body_display()
        body_msg = PoseStamped()
        body_msg.header = pose_msg.header
        fill_pose(body_msg.pose, T_map_body)
        self.arm_body_pose_pub.publish(body_msg)

        marker_msg = PoseStamped()
        marker_msg.header = pose_msg.header
        fill_pose(marker_msg.pose, candidate.T_map_marker)
        self.marker_pose_pub.publish(marker_msg)

        path_pose = PoseStamped()
        path_pose.header = pose_msg.header
        fill_pose(path_pose.pose, candidate.T_map_armcam)
        self.path_msg.header = pose_msg.header
        self.path_msg.poses.append(path_pose)
        if len(self.path_msg.poses) > self.path_length:
            self.path_msg.poses = self.path_msg.poses[-self.path_length :]
        self.path_pub.publish(self.path_msg)

        if self.publish_tf_enabled:
            self.publish_tf(msg.header.stamp, self.map_frame, self.arm_preview_frame, candidate.T_map_armcam)
            self.publish_tf(msg.header.stamp, self.map_frame, self.arm_body_preview_frame, T_map_body)
            self.publish_tf(msg.header.stamp, self.map_frame, self.marker_preview_frame, candidate.T_map_marker)

        self.last_candidate_stamp_sec = stamp_to_sec(msg.header.stamp)
        self.last_T_map_armcam = candidate.T_map_armcam.copy()

        std = covariance_diag_std(candidate.covariance)
        fields = self.dynamic_status_fields(msg)
        fields.update(
            {
                "head_pose_match_dt_s": round(float(matched.dt_s), 6),
                "head_pose_match_mode": matched.mode,
                "dynamic_covariance_fallback": bool(dynamic_cov_fallback),
                "extrinsic_covariance_source": self.extrinsic.covariance_source,
                "preview_std_x_m": round(float(std[0]), 6),
                "preview_std_y_m": round(float(std[1]), 6),
                "preview_std_z_m": round(float(std[2]), 6),
                "preview_std_roll_deg": round(math.degrees(float(std[3])), 6),
                "preview_std_pitch_deg": round(math.degrees(float(std[4])), 6),
                "preview_std_yaw_deg": round(math.degrees(float(std[5])), 6),
            }
        )
        self.publish_status(True, "accepted", stamp_to_sec(msg.header.stamp), fields)

    def dynamic_status_fields(self, msg: DynamicMarkerObservation) -> dict[str, Any]:
        return {
            "marker_id": int(msg.marker_id),
            "marker_frame": str(msg.marker_frame),
            "camera_frame": str(msg.camera_frame),
            "stable": bool(msg.stable),
            "stable_frames": int(msg.stable_frames),
            "reprojection_error_px": round(float(msg.reprojection_error_px), 4),
            "distance_m": round(float(msg.distance_m), 4),
            "view_angle_deg": round(float(msg.view_angle_deg), 4),
            "area_px2": round(float(msg.area_px2), 3),
            "geometry_score": round(float(msg.geometry_score), 4),
        }

    def publish_status(self, accepted: bool, reason: str, stamp_sec: float, fields: dict[str, Any]) -> None:
        payload = {
            "event_type": "head_derived_arm_pose_preview",
            "stamp": round(float(stamp_sec), 9),
            "accepted": bool(accepted),
            "reason": str(reason),
        }
        payload.update(fields)
        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(out)

    def publish_tf(self, stamp, parent: str, child: str, T: np.ndarray) -> None:
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x = float(T[0, 3])
        tf.transform.translation.y = float(T[1, 3])
        tf.transform.translation.z = float(T[2, 3])
        qx, qy, qz, qw = rot_to_quat_xyzw(T[:3, :3])
        tf.transform.rotation.x = float(qx)
        tf.transform.rotation.y = float(qy)
        tf.transform.rotation.z = float(qz)
        tf.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = HeadDerivedArmPosePreviewNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
