#!/usr/bin/env python3

"""Head-derived dynamic arm pose measurement producer.

This node publishes an explicit OpenVINS-facing measurement for the arm IMU
pose when the head camera observes the arm-mounted dynamic marker. It is kept
separate from the RViz preview outputs: preview TF/path state is not used to
decide whether an estimator measurement exists.
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
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_fusion_msgs.msg import DynamicArmPoseObservation, DynamicMarkerObservation
from std_msgs.msg import String

_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Reliable QoS for the DynamicArmPoseObservation output that feeds
# jetson_relay → run_subscribe_msckf_marker (compiled C++ EKF).
_RELIABLE_OBS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

from aruco_marker_pose_node import (
    R_to_rotvec,
    T_inv,
    fill_pose,
    load_kalibr_imucam,
    quat_to_R_xyzw,
    rotvec_to_R,
    stamp_to_sec,
)
from head_derived_arm_pose_preview_node import (
    RESIDUAL_ORDER,
    covariance_diag_std,
    covariance_matrix_from_ros,
    load_arm_marker_extrinsic,
    load_head_camera_transform,
    resolve_path,
    sanitize_covariance,
    se3_exp,
    se3_left_residual,
)


@dataclass
class DynamicArmPoseCandidate:
    T_map_headcam: np.ndarray
    T_map_marker: np.ndarray
    T_map_armcam: np.ndarray
    T_map_armimu: np.ndarray
    covariance: np.ndarray


@dataclass
class HeadPoseMeasurementSample:
    stamp: Any
    stamp_sec: float
    T_map_headimu: np.ndarray
    covariance: np.ndarray
    covariance_fallback: bool


@dataclass
class MatchedHeadPoseMeasurement:
    stamp: Any
    stamp_sec: float
    T_map_headimu: np.ndarray
    covariance: np.ndarray
    covariance_fallback: bool
    dt_s: float
    time_offset_s: float
    mode: str


def interpolate_transform(T_a: np.ndarray, T_b: np.ndarray, alpha: float) -> np.ndarray:
    a = max(0.0, min(1.0, float(alpha)))
    T = np.eye(4)
    T[:3, 3] = (1.0 - a) * T_a[:3, 3] + a * T_b[:3, 3]
    R_delta = T_a[:3, :3].T @ T_b[:3, :3]
    T[:3, :3] = T_a[:3, :3] @ rotvec_to_R(a * R_to_rotvec(R_delta))
    return T


def match_head_pose_measurement(
    samples: list[HeadPoseMeasurementSample],
    stamp_sec: float,
    max_dt_s: float,
) -> Optional[MatchedHeadPoseMeasurement]:
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
            return MatchedHeadPoseMeasurement(
                stamp=before.stamp,
                stamp_sec=stamp_sec,
                T_map_headimu=interpolate_transform(before.T_map_headimu, after.T_map_headimu, alpha),
                covariance=(1.0 - alpha) * before.covariance + alpha * after.covariance,
                covariance_fallback=bool(before.covariance_fallback or after.covariance_fallback),
                dt_s=max(before_dt, after_dt),
                time_offset_s=0.0,
                mode="interpolated",
            )

    nearest = min(samples, key=lambda sample: abs(sample.stamp_sec - stamp_sec))
    dt = abs(nearest.stamp_sec - stamp_sec)
    if dt > max_dt_s:
        return None
    return MatchedHeadPoseMeasurement(
        stamp=nearest.stamp,
        stamp_sec=nearest.stamp_sec,
        T_map_headimu=nearest.T_map_headimu,
        covariance=nearest.covariance,
        covariance_fallback=bool(nearest.covariance_fallback),
        dt_s=dt,
        time_offset_s=float(stamp_sec - nearest.stamp_sec),
        mode="nearest",
    )


def load_arm_camera_transform(arm_config_path: Path) -> tuple[str, str, np.ndarray]:
    config = yaml.safe_load(arm_config_path.read_text(encoding="utf-8"))
    frames = config.get("frames", {})
    camera_frame = str(frames.get("detected_camera_frame", frames.get("camera_frame", "")))
    imu_frame = str(frames.get("imu_frame", "arm_imu"))
    calib_path = resolve_path(str(config["calibration"]["kalibr_imucam_chain"]), arm_config_path)
    _K, _D, T_armcam_armimu, _timeshift = load_kalibr_imucam(str(calib_path))
    return camera_frame, imu_frame, T_armcam_armimu


def compose_dynamic_arm_imu_pose(
    T_map_headimu: np.ndarray,
    T_headcam_headimu: np.ndarray,
    T_headcam_marker: np.ndarray,
    T_armcam_marker: np.ndarray,
    T_armcam_armimu: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    T_map_headcam = T_map_headimu @ T_inv(T_headcam_headimu)
    T_map_marker = T_map_headcam @ T_headcam_marker
    T_map_armcam = T_map_marker @ T_inv(T_armcam_marker)
    T_map_armimu = T_map_armcam @ T_armcam_armimu
    return T_map_headcam, T_map_marker, T_map_armcam, T_map_armimu


def propagate_dynamic_arm_imu_covariance(
    T_map_headimu: np.ndarray,
    P_head: np.ndarray,
    T_headcam_headimu: np.ndarray,
    T_headcam_marker: np.ndarray,
    P_dynamic: np.ndarray,
    T_armcam_marker: np.ndarray,
    P_extrinsic: np.ndarray,
    T_armcam_armimu: np.ndarray,
    P_armcam_armimu: np.ndarray,
    min_diag: np.ndarray,
    epsilon: float = 1e-4,
) -> np.ndarray:
    _T_headcam, _T_marker, _T_armcam, T_nominal = compose_dynamic_arm_imu_pose(
        T_map_headimu,
        T_headcam_headimu,
        T_headcam_marker,
        T_armcam_marker,
        T_armcam_armimu,
    )

    def jacobian(which: str) -> np.ndarray:
        J = np.zeros((6, 6), dtype=float)
        for axis in range(6):
            delta = np.zeros(6, dtype=float)
            delta[axis] = float(epsilon)
            dT = se3_exp(delta)
            head = T_map_headimu
            dynamic = T_headcam_marker
            marker_ext = T_armcam_marker
            arm_cam_imu = T_armcam_armimu
            if which == "head":
                head = dT @ head
            elif which == "dynamic":
                dynamic = dT @ dynamic
            elif which == "marker_ext":
                marker_ext = dT @ marker_ext
            elif which == "arm_cam_imu":
                arm_cam_imu = dT @ arm_cam_imu
            else:
                raise RuntimeError(f"Unknown covariance source {which}")
            _T_hc, _T_m, _T_ac, T_perturbed = compose_dynamic_arm_imu_pose(
                head,
                T_headcam_headimu,
                dynamic,
                marker_ext,
                arm_cam_imu,
            )
            J[:, axis] = se3_left_residual(T_nominal, T_perturbed) / float(epsilon)
        return J

    J_head = jacobian("head")
    J_dynamic = jacobian("dynamic")
    J_marker = jacobian("marker_ext")
    J_armcamimu = jacobian("arm_cam_imu")
    P = (
        J_head @ P_head @ J_head.T
        + J_dynamic @ P_dynamic @ J_dynamic.T
        + J_marker @ P_extrinsic @ J_marker.T
        + J_armcamimu @ P_armcam_armimu @ J_armcamimu.T
    )
    return sanitize_covariance(P, min_diag)


def pose_to_T(pose_msg) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_to_R_xyzw(pose_msg.orientation)
    T[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
    return T


def is_finite_transform(T: np.ndarray) -> bool:
    return bool(np.asarray(T).shape == (4, 4) and np.all(np.isfinite(T)))


def quaternion_norm(pose_msg) -> float:
    q = pose_msg.orientation
    return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)


class DynamicArmPoseMeasurementNode(Node):
    def __init__(self):
        super().__init__("dynamic_arm_pose_measurement_node")

        self.declare_parameter("head_marker_config", "")
        self.declare_parameter("arm_marker_config", "")
        self.declare_parameter("arm_marker_extrinsics", "")
        self.declare_parameter("dynamic_observation_topic", "/head/marker_pose/dynamic_observation")
        self.declare_parameter("head_pose_topic", "/ov_msckf/odomimu")
        self.declare_parameter("head_pose_message_type", "odometry")
        self.declare_parameter("dynamic_arm_pose_observation_topic", "/arm/marker_pose/dynamic_arm_pose_observation")
        self.declare_parameter("dynamic_arm_measurement_status_topic", "/arm/marker_pose/dynamic_arm_measurement/status")
        self.declare_parameter("publish_dynamic_arm_pose_observation", False)
        self.declare_parameter("marker_id", 2)
        self.declare_parameter("target_frame", "arm_imu")
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
        self.declare_parameter("extrinsic_covariance_source", "robust_diag_covariance_se3")
        self.declare_parameter("default_head_position_std_m", 0.20)
        self.declare_parameter("default_head_rotation_std_deg", 20.0)
        self.declare_parameter("default_dynamic_position_std_m", 0.05)
        self.declare_parameter("default_dynamic_rotation_std_deg", 8.0)
        self.declare_parameter("default_extrinsic_position_std_m", 0.03)
        self.declare_parameter("default_extrinsic_rotation_std_deg", 5.0)
        self.declare_parameter("default_arm_cam_imu_position_std_m", 0.005)
        self.declare_parameter("default_arm_cam_imu_rotation_std_deg", 1.0)
        self.declare_parameter("covariance_epsilon", 1e-4)

        head_config = str(self.get_parameter("head_marker_config").value)
        arm_config = str(self.get_parameter("arm_marker_config").value)
        extrinsics_path = str(self.get_parameter("arm_marker_extrinsics").value)
        if not head_config:
            raise RuntimeError("Parameter head_marker_config is required")
        if not arm_config:
            raise RuntimeError("Parameter arm_marker_config is required")
        if not extrinsics_path:
            raise RuntimeError("Parameter arm_marker_extrinsics is required")

        self.marker_id = int(self.get_parameter("marker_id").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.dynamic_topic = str(self.get_parameter("dynamic_observation_topic").value)
        self.head_pose_topic = str(self.get_parameter("head_pose_topic").value)
        self.head_pose_message_type = str(self.get_parameter("head_pose_message_type").value).strip().lower()
        if self.head_pose_message_type not in {"odometry", "pose_with_covariance_stamped"}:
            raise RuntimeError(
                "Parameter head_pose_message_type must be 'odometry' or 'pose_with_covariance_stamped'"
            )
        self.output_topic = str(self.get_parameter("dynamic_arm_pose_observation_topic").value)
        self.status_topic = str(self.get_parameter("dynamic_arm_measurement_status_topic").value)
        self.publish_measurement_enabled = bool(self.get_parameter("publish_dynamic_arm_pose_observation").value)
        self.max_head_pose_dt_s = float(self.get_parameter("max_head_pose_dt_s").value)
        self.head_pose_buffer_seconds = float(self.get_parameter("head_pose_buffer_seconds").value)
        self.require_stable_dynamic_marker = bool(self.get_parameter("require_stable_dynamic_marker").value)
        self.max_pose_covariance_trace = float(self.get_parameter("max_pose_covariance_trace").value)
        self.max_reprojection_error_px = float(self.get_parameter("max_reprojection_error_px").value)
        self.max_marker_distance_m = float(self.get_parameter("max_marker_distance_m").value)
        self.max_view_angle_deg = float(self.get_parameter("max_view_angle_deg").value)
        self.min_marker_area_px2 = float(self.get_parameter("min_marker_area_px2").value)
        self.min_geometry_score = float(self.get_parameter("min_geometry_score").value)
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
        self.arm_cam_imu_diag = self.default_covariance_diag(
            float(self.get_parameter("default_arm_cam_imu_position_std_m").value),
            float(self.get_parameter("default_arm_cam_imu_rotation_std_deg").value),
        )
        self.output_min_diag = np.minimum.reduce(
            [self.head_fallback_diag, self.dynamic_fallback_diag, self.extrinsic_fallback_diag, self.arm_cam_imu_diag]
        ) * 1e-3

        head_config_path = resolve_path(head_config)
        arm_config_path = resolve_path(arm_config)
        self.map_frame, self.head_camera_frame, self.T_headcam_headimu = load_head_camera_transform(head_config_path)
        head_config_data = yaml.safe_load(head_config_path.read_text(encoding="utf-8"))
        self.head_imu_frame = str(head_config_data.get("frames", {}).get("imu_frame", "head_imu"))
        self.arm_camera_frame, self.arm_imu_frame, self.T_armcam_armimu = load_arm_camera_transform(arm_config_path)
        map_frame_override = str(self.get_parameter("map_frame").value)
        if map_frame_override:
            self.map_frame = map_frame_override
        if self.target_frame != self.arm_imu_frame:
            self.get_logger().warning(
                f"target_frame '{self.target_frame}' differs from arm config imu_frame '{self.arm_imu_frame}'"
            )

        self.extrinsic = load_arm_marker_extrinsic(
            resolve_path(extrinsics_path),
            self.marker_id,
            str(self.get_parameter("extrinsic_covariance_source").value),
            self.extrinsic_fallback_diag,
        )
        self.check_extrinsic_consistency()

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=20)
        self.dynamic_sub = self.create_subscription(DynamicMarkerObservation, self.dynamic_topic, self.dynamic_cb, qos)
        if self.head_pose_message_type == "odometry":
            self.head_pose_sub = self.create_subscription(Odometry, self.head_pose_topic, self.head_odom_cb, qos)
        else:
            self.head_pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.head_pose_topic,
                self.head_pose_cb,
                qos,
            )
        self.measurement_pub = self.create_publisher(DynamicArmPoseObservation, self.output_topic, _RELIABLE_OBS_QOS)
        self.status_pub = self.create_publisher(String, self.status_topic, _RELIABLE_OBS_QOS)

        self.head_buffer: deque[HeadPoseMeasurementSample] = deque()
        self.get_logger().info(f"Dynamic observation topic: {self.dynamic_topic}")
        self.get_logger().info(f"Head pose topic: {self.head_pose_topic} type={self.head_pose_message_type}")
        self.get_logger().info(f"Measurement topic: {self.output_topic} enabled={self.publish_measurement_enabled}")
        self.get_logger().info(f"Status topic: {self.status_topic}")
        self.get_logger().info(f"Target frame: {self.target_frame}")
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

    def check_extrinsic_consistency(self) -> None:
        if self.extrinsic.parent_camera_frame and self.extrinsic.parent_camera_frame != self.arm_camera_frame:
            self.get_logger().warning(
                f"Arm marker extrinsic parent_camera_frame '{self.extrinsic.parent_camera_frame}' "
                f"differs from arm config camera_frame '{self.arm_camera_frame}'"
            )
        if self.extrinsic.parent_imu_frame and self.extrinsic.parent_imu_frame != self.arm_imu_frame:
            self.get_logger().warning(
                f"Arm marker extrinsic parent_imu_frame '{self.extrinsic.parent_imu_frame}' "
                f"differs from arm config imu_frame '{self.arm_imu_frame}'"
            )

    def head_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        stamp_sec = stamp_to_sec(msg.header.stamp)
        if msg.header.frame_id != self.map_frame:
            self.publish_status(False, "head_pose_wrong_frame", stamp_sec, {})
            return
        self.store_head_pose_sample(
            stamp=msg.header.stamp,
            stamp_sec=stamp_sec,
            pose=msg.pose.pose,
            covariance=msg.pose.covariance,
        )

    def head_odom_cb(self, msg: Odometry) -> None:
        stamp_sec = stamp_to_sec(msg.header.stamp)
        if msg.header.frame_id != self.map_frame:
            self.publish_status(False, "head_pose_wrong_frame", stamp_sec, {"head_pose_child_frame": str(msg.child_frame_id)})
            return
        if msg.child_frame_id != self.head_imu_frame:
            self.publish_status(
                False,
                "head_pose_wrong_child_frame",
                stamp_sec,
                {"head_pose_child_frame": str(msg.child_frame_id), "expected_head_pose_child_frame": self.head_imu_frame},
            )
            return
        self.store_head_pose_sample(
            stamp=msg.header.stamp,
            stamp_sec=stamp_sec,
            pose=msg.pose.pose,
            covariance=msg.pose.covariance,
        )

    def store_head_pose_sample(self, stamp, stamp_sec: float, pose, covariance) -> None:
        q_norm = quaternion_norm(pose)
        if not math.isfinite(q_norm) or q_norm < 1e-9:
            self.publish_status(False, "head_pose_bad_quaternion", stamp_sec, {})
            return
        T_map_headimu = pose_to_T(pose)
        if not is_finite_transform(T_map_headimu):
            self.publish_status(False, "head_pose_non_finite", stamp_sec, {})
            return
        P_head, used_fallback = covariance_matrix_from_ros(covariance, self.head_fallback_diag)
        if float(np.trace(P_head)) > self.max_pose_covariance_trace:
            self.publish_status(
                False,
                "head_pose_covariance_too_large",
                stamp_sec,
                {"head_pose_covariance_trace": round(float(np.trace(P_head)), 6)},
            )
            return
        self.head_buffer.append(
            HeadPoseMeasurementSample(
                stamp=stamp,
                stamp_sec=stamp_sec,
                T_map_headimu=T_map_headimu,
                covariance=P_head,
                covariance_fallback=bool(used_fallback),
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
        fields = self.dynamic_status_fields(msg)
        reason = self.dynamic_gate_reason(msg)
        if reason:
            self.publish_status(False, reason, stamp_sec, fields)
            return

        matched = match_head_pose_measurement(list(self.head_buffer), stamp_sec, self.max_head_pose_dt_s)
        if matched is None:
            fields.update({"head_pose_buffer_size": len(self.head_buffer)})
            self.publish_status(False, "no_head_pose_match", stamp_sec, fields)
            return

        T_headcam_marker = pose_to_T(msg.pose.pose)
        if not is_finite_transform(T_headcam_marker):
            fields.update({"head_pose_match_dt_s": round(float(matched.dt_s), 6), "head_pose_match_mode": matched.mode})
            self.publish_status(False, "dynamic_pose_non_finite", stamp_sec, fields)
            return

        P_dynamic, dynamic_cov_fallback = covariance_matrix_from_ros(msg.pose.covariance, self.dynamic_fallback_diag)
        covariance = propagate_dynamic_arm_imu_covariance(
            matched.T_map_headimu,
            matched.covariance,
            self.T_headcam_headimu,
            T_headcam_marker,
            P_dynamic,
            self.extrinsic.T_armcam_marker,
            self.extrinsic.covariance,
            self.T_armcam_armimu,
            np.diag(self.arm_cam_imu_diag),
            self.output_min_diag,
            epsilon=self.covariance_epsilon,
        )
        T_map_headcam, T_map_marker, T_map_armcam, T_map_armimu = compose_dynamic_arm_imu_pose(
            matched.T_map_headimu,
            self.T_headcam_headimu,
            T_headcam_marker,
            self.extrinsic.T_armcam_marker,
            self.T_armcam_armimu,
        )
        candidate = DynamicArmPoseCandidate(
            T_map_headcam=T_map_headcam,
            T_map_marker=T_map_marker,
            T_map_armcam=T_map_armcam,
            T_map_armimu=T_map_armimu,
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

    def publish_candidate(
        self,
        msg: DynamicMarkerObservation,
        candidate: DynamicArmPoseCandidate,
        matched,
        dynamic_cov_fallback: bool,
    ) -> None:
        if self.publish_measurement_enabled:
            out = DynamicArmPoseObservation()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.map_frame
            out.marker_id = int(msg.marker_id)
            out.source_camera_frame = msg.camera_frame
            out.marker_frame = msg.marker_frame
            out.target_frame = self.target_frame
            fill_pose(out.pose.pose, candidate.T_map_armimu)
            out.pose.covariance = list(np.asarray(candidate.covariance, dtype=float).reshape(36))
            out.hard_gate_passed = bool(msg.hard_gate_passed)
            out.hard_gate_status = str(msg.hard_gate_status)
            out.stable = bool(msg.stable)
            out.stable_frames = int(msg.stable_frames)
            out.stability_factor = float(msg.stability_factor)
            out.reprojection_error_px = float(msg.reprojection_error_px)
            out.distance_m = float(msg.distance_m)
            out.view_angle_deg = float(msg.view_angle_deg)
            out.area_px2 = float(msg.area_px2)
            out.side_mean_px = float(msg.side_mean_px)
            out.side_min_px = float(msg.side_min_px)
            out.geometry_score = float(msg.geometry_score)
            out.covariance_sigma_px = float(msg.covariance_sigma_px)
            out.head_pose_match_dt_s = float(matched.dt_s)
            out.head_pose_match_mode = str(matched.mode)
            out.head_pose_source_topic = self.head_pose_topic
            out.head_pose_source_type = self.head_pose_message_type
            out.head_pose_time_offset_s = float(matched.time_offset_s)
            out.dynamic_covariance_fallback = bool(dynamic_cov_fallback)
            out.head_covariance_fallback = bool(matched.covariance_fallback)
            out.extrinsic_covariance_source = self.extrinsic.covariance_source
            self.measurement_pub.publish(out)

        std = covariance_diag_std(candidate.covariance)
        fields = self.dynamic_status_fields(msg)
        fields.update(
            {
                "measurement_published": bool(self.publish_measurement_enabled),
                "head_pose_match_dt_s": round(float(matched.dt_s), 6),
                "head_pose_match_mode": matched.mode,
                "head_pose_source_topic": self.head_pose_topic,
                "head_pose_source_type": self.head_pose_message_type,
                "head_pose_time_offset_s": round(float(matched.time_offset_s), 6),
                "dynamic_covariance_fallback": bool(dynamic_cov_fallback),
                "head_covariance_fallback": bool(matched.covariance_fallback),
                "extrinsic_covariance_source": self.extrinsic.covariance_source,
                "measurement_std_x_m": round(float(std[0]), 6),
                "measurement_std_y_m": round(float(std[1]), 6),
                "measurement_std_z_m": round(float(std[2]), 6),
                "measurement_std_roll_deg": round(math.degrees(float(std[3])), 6),
                "measurement_std_pitch_deg": round(math.degrees(float(std[4])), 6),
                "measurement_std_yaw_deg": round(math.degrees(float(std[5])), 6),
            }
        )
        reason = "accepted_published" if self.publish_measurement_enabled else "accepted_measurement_only"
        self.publish_status(True, reason, stamp_to_sec(msg.header.stamp), fields)

    def dynamic_status_fields(self, msg: DynamicMarkerObservation) -> dict[str, Any]:
        return {
            "marker_id": int(msg.marker_id),
            "marker_frame": str(msg.marker_frame),
            "source_camera_frame": str(msg.camera_frame),
            "target_frame": self.target_frame,
            "head_pose_source_topic": self.head_pose_topic,
            "head_pose_source_type": self.head_pose_message_type,
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
            "event_type": "dynamic_arm_pose_measurement",
            "stamp": round(float(stamp_sec), 9),
            "accepted": bool(accepted),
            "reason": str(reason),
        }
        payload.update(fields)
        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(out)


def main():
    rclpy.init()
    node = DynamicArmPoseMeasurementNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
