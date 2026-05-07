#!/usr/bin/env python3

"""External ArUco-based correction layer for OpenVINS odometry.

Transform convention used throughout this file:
  T_A_B maps a point expressed in frame B into frame A.

Important transforms:
  T_map_global  = OpenVINS global frame -> marker_map frame
  T_map_marker  = marker_<id> frame -> marker_map frame
  T_cam_marker  = marker frame -> camera optical frame from OpenCV solvePnP
  T_marker_cam  = inverse(T_cam_marker)
  T_cam_imu     = IMU frame -> camera optical frame from Kalibr/OpenVINS
  T_map_imu     = T_map_marker * T_marker_cam * T_cam_imu

The raw OpenCV marker frame is only an intermediate measurement frame. The
marker_map frame is configured in YAML and should be aligned to the desired
navigation/world convention.

camera_pose_raw is the true ROS/OpenCV camera optical frame:
  +X right, +Y down, +Z forward through the lens.
RViz pose arrows point along +X, so the red arrow is not the camera viewing
direction for this topic. The blue +Z axis pointing toward the marker is
expected when the camera is looking at the marker.

imu_pose is the true calibrated IMU pose in marker_map. Its axes are whatever
the calibrated IMU frame is, so they should not be modified for RViz comfort.
Use camera_body_pose for an intuitive visualization-only body frame.
"""

from __future__ import annotations

import ast
from collections import Counter, deque
from dataclasses import dataclass
import json
import math
import re
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def T_inv(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Tout = np.eye(4)
    Tout[:3, :3] = R.T
    Tout[:3, 3] = -R.T @ t
    return Tout


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def is_finite_array(values: np.ndarray) -> bool:
    return bool(np.all(np.isfinite(values)))


def quat_to_R_xyzw(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n <= 0.0 or not math.isfinite(n):
        return np.eye(3)
    s = 2.0 / n
    xx = x * x * s
    yy = y * y * s
    zz = z * z * s
    xy = x * y * s
    xz = x * z * s
    yz = y * z * s
    wx = w * x * s
    wy = w * y * s
    wz = w * z * s
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=float,
    )


def rot_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    tr = np.trace(R)
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw], dtype=float)
    norm = np.linalg.norm(q)
    if norm <= 0.0 or not math.isfinite(norm):
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


def rotvec_to_R(rotvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.asarray(rotvec, dtype=float).reshape(3, 1))
    return R


def R_to_rotvec(R: np.ndarray) -> np.ndarray:
    rotvec, _ = cv2.Rodrigues(np.asarray(R, dtype=float))
    return rotvec.reshape(3)


def rotation_angle_deg(R: np.ndarray) -> float:
    trace_value = float(np.trace(R))
    cos_angle = max(-1.0, min(1.0, (trace_value - 1.0) * 0.5))
    return math.degrees(math.acos(cos_angle))


def odom_to_T(msg: Odometry) -> np.ndarray:
    p = msg.pose.pose.position
    T = np.eye(4)
    T[:3, :3] = quat_to_R_xyzw(msg.pose.pose.orientation)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def fill_pose(pose_msg, T: np.ndarray) -> None:
    qx, qy, qz, qw = rot_to_quat_xyzw(T[:3, :3])
    pose_msg.position.x = float(T[0, 3])
    pose_msg.position.y = float(T[1, 3])
    pose_msg.position.z = float(T[2, 3])
    pose_msg.orientation.x = float(qx)
    pose_msg.orientation.y = float(qy)
    pose_msg.orientation.z = float(qz)
    pose_msg.orientation.w = float(qw)


def covariance_from_diag(diag6: np.ndarray) -> list[float]:
    cov = [0.0] * 36
    values = np.asarray(diag6, dtype=float).reshape(6)
    for i in range(6):
        cov[6 * i + i] = float(max(values[i], 0.0))
    return cov


def covariance_diag_to_std(diag6: np.ndarray) -> np.ndarray:
    values = np.asarray(diag6, dtype=float).reshape(6)
    return np.sqrt(np.maximum(values, 0.0))


def pose_covariance_diag(msg: Odometry, fallback_diag: np.ndarray) -> np.ndarray:
    cov = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
    diag = np.array([cov[0, 0], cov[1, 1], cov[2, 2], cov[3, 3], cov[4, 4], cov[5, 5]], dtype=float)
    if not is_finite_array(diag) or np.any(diag < 0.0):
        return fallback_diag.copy()
    return diag


def twist_covariance_diag(msg: Odometry, fallback_diag: np.ndarray) -> np.ndarray:
    cov = np.asarray(msg.twist.covariance, dtype=float).reshape(6, 6)
    diag = np.array([cov[0, 0], cov[1, 1], cov[2, 2], cov[3, 3], cov[4, 4], cov[5, 5]], dtype=float)
    if not is_finite_array(diag) or np.any(diag < 0.0):
        return fallback_diag.copy()
    return diag


def parse_list_from_line(text: str, key: str) -> list[float]:
    m = re.search(rf"{key}:\s*(\[[^\n]+\])", text)
    if not m:
        raise RuntimeError(f"Could not find '{key}' in calibration file")
    return ast.literal_eval(m.group(1))


def parse_optional_list_from_line(text: str, key: str) -> Optional[list[float]]:
    m = re.search(rf"{key}:\s*([-+0-9.eE]+|\[[^\n]+\])", text)
    if not m:
        return None
    value = ast.literal_eval(m.group(1))
    if isinstance(value, list):
        return value
    return [float(value)]


def parse_T_block_optional(text: str, key: str) -> Optional[np.ndarray]:
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if line.strip().startswith(f"{key}:"):
            rows = []
            for j in range(i + 1, min(i + 8, len(lines))):
                s = lines[j].strip()
                if s.startswith("- [") or s.startswith("["):
                    s = s[1:].strip() if s.startswith("-") else s
                    rows.append(ast.literal_eval(s))
                    if len(rows) == 4:
                        return np.array(rows, dtype=float)
    return None


def load_kalibr_imucam(path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    text = Path(path).read_text()
    fx, fy, cx, cy = parse_list_from_line(text, "intrinsics")
    dist = parse_list_from_line(text, "distortion_coeffs")

    T_cam_imu = parse_T_block_optional(text, "T_cam_imu")
    if T_cam_imu is None:
        T_imu_cam = parse_T_block_optional(text, "T_imu_cam")
        if T_imu_cam is None:
            raise RuntimeError("Could not find T_cam_imu or T_imu_cam in calibration file")
        # The correction formula uses T_cam_imu = camera optical frame <- IMU frame.
        T_cam_imu = T_inv(T_imu_cam)

    timeshift = parse_optional_list_from_line(text, "timeshift_cam_imu")
    timeshift_cam_imu = float(timeshift[0]) if timeshift is not None else 0.0

    K = np.array(
        [
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    D = np.array(dist, dtype=float).reshape(-1, 1)
    return K, D, T_cam_imu, timeshift_cam_imu


def marker_object_points(size_m: float) -> np.ndarray:
    s = float(size_m)
    h = s / 2.0
    # OpenCV ArUco corner order: top-left, top-right, bottom-right, bottom-left.
    # Marker frame is centered on the marker, x right, y down in the printed plane.
    return np.array(
        [
            [-h, -h, 0.0],
            [h, -h, 0.0],
            [h, h, 0.0],
            [-h, h, 0.0],
        ],
        dtype=np.float32,
    )


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def marker_side_lengths_px(image_points: np.ndarray) -> np.ndarray:
    points = np.asarray(image_points, dtype=float).reshape(4, 2)
    return np.array(
        [np.linalg.norm(points[(i + 1) % 4] - points[i]) for i in range(4)],
        dtype=float,
    )


def marker_view_angle_deg(T_cam_marker: np.ndarray) -> float:
    R_cam_marker = np.asarray(T_cam_marker, dtype=float).reshape(4, 4)[:3, :3]
    marker_normal_cam = R_cam_marker @ np.array([0.0, 0.0, 1.0], dtype=float)
    norm = float(np.linalg.norm(marker_normal_cam))
    if norm <= 1e-9 or not math.isfinite(norm):
        return 90.0
    cos_angle = abs(float(marker_normal_cam[2] / norm))
    cos_angle = clamp(cos_angle, -1.0, 1.0)
    return math.degrees(math.acos(cos_angle))


@dataclass
class MarkerQualityMetrics:
    area_px2: float
    sqrt_area_px: float
    side_mean_px: float
    side_min_px: float
    distance_m: float
    reprojection_error_px: float
    view_angle_deg: float


@dataclass
class MarkerTemporalStats:
    stable_frames: int
    stable: bool
    stability_factor: float
    detection_translation_delta_m: Optional[float]
    detection_rotation_delta_deg: Optional[float]
    correction_translation_delta_m: Optional[float]
    correction_rotation_delta_deg: Optional[float]
    odom_match_dt: Optional[float]


@dataclass
class MarkerCovarianceModelConfig:
    corner_noise_floor_px: float
    max_view_angle_deg: float
    max_view_penalty: float
    min_xy_std_m: float
    min_z_std_m: float
    max_xy_std_m: float
    max_z_std_m: float
    min_roll_pitch_std_deg: float
    min_yaw_std_deg: float
    max_roll_pitch_std_deg: float
    max_yaw_std_deg: float
    xy_std_px_gain: float
    z_std_px_gain: float
    roll_pitch_std_px_gain: float
    yaw_std_px_gain: float
    temporal_missing_position_std_m: float
    temporal_missing_rotation_std_deg: float
    temporal_correction_position_gain: float
    temporal_correction_rotation_gain: float
    geometry_position_std_m: float
    geometry_rotation_std_deg: float

    @classmethod
    def from_mapping(
        cls,
        cfg: dict[str, Any],
        legacy_min_position_std_m: float = 0.02,
        legacy_max_position_std_m: float = 0.75,
        legacy_min_rotation_std_deg: float = 2.0,
        legacy_max_rotation_std_deg: float = 45.0,
    ) -> "MarkerCovarianceModelConfig":
        return cls(
            corner_noise_floor_px=float(cfg.get("corner_noise_floor_px", 0.35)),
            max_view_angle_deg=float(cfg.get("max_view_angle_deg", 75.0)),
            max_view_penalty=float(cfg.get("max_view_penalty", 4.0)),
            min_xy_std_m=float(cfg.get("min_marker_xy_std_m", legacy_min_position_std_m)),
            min_z_std_m=float(cfg.get("min_marker_z_std_m", legacy_min_position_std_m)),
            max_xy_std_m=float(cfg.get("max_marker_xy_std_m", legacy_max_position_std_m)),
            max_z_std_m=float(cfg.get("max_marker_z_std_m", legacy_max_position_std_m)),
            min_roll_pitch_std_deg=float(
                cfg.get("min_marker_roll_pitch_std_deg", legacy_min_rotation_std_deg)
            ),
            min_yaw_std_deg=float(cfg.get("min_marker_yaw_std_deg", legacy_min_rotation_std_deg)),
            max_roll_pitch_std_deg=float(
                cfg.get("max_marker_roll_pitch_std_deg", legacy_max_rotation_std_deg)
            ),
            max_yaw_std_deg=float(cfg.get("max_marker_yaw_std_deg", legacy_max_rotation_std_deg)),
            xy_std_px_gain=float(cfg.get("marker_xy_std_px_gain", 8.0)),
            z_std_px_gain=float(cfg.get("marker_z_std_px_gain", 2.0)),
            roll_pitch_std_px_gain=float(cfg.get("marker_roll_pitch_std_px_gain", 5.0)),
            yaw_std_px_gain=float(cfg.get("marker_yaw_std_px_gain", 3.0)),
            temporal_missing_position_std_m=float(cfg.get("temporal_missing_position_std_m", 0.05)),
            temporal_missing_rotation_std_deg=float(cfg.get("temporal_missing_rotation_std_deg", 8.0)),
            temporal_correction_position_gain=float(cfg.get("temporal_correction_position_gain", 0.50)),
            temporal_correction_rotation_gain=float(cfg.get("temporal_correction_rotation_gain", 0.50)),
            geometry_position_std_m=float(cfg.get("geometry_position_std_m", 0.03)),
            geometry_rotation_std_deg=float(cfg.get("geometry_rotation_std_deg", 5.0)),
        )


@dataclass
class MarkerCovarianceEstimate:
    diag: np.ndarray
    std_diag: np.ndarray
    camera_std_diag: np.ndarray
    sigma_px: float
    view_penalty: float


@dataclass
class MarkerPoseHistoryEntry:
    T_map_imu: np.ndarray
    T_meas_map_global: Optional[np.ndarray]


def marker_quality_metrics(
    image_points: np.ndarray,
    T_cam_marker: np.ndarray,
    reprojection_error_px: float,
    area_px2: Optional[float] = None,
) -> MarkerQualityMetrics:
    area = abs(float(cv2.contourArea(np.asarray(image_points, dtype=np.float32).reshape(4, 2))))
    if area_px2 is not None:
        area = float(area_px2)
    side_lengths = marker_side_lengths_px(image_points)
    distance_m = float(np.linalg.norm(np.asarray(T_cam_marker, dtype=float).reshape(4, 4)[:3, 3]))
    return MarkerQualityMetrics(
        area_px2=area,
        sqrt_area_px=math.sqrt(max(area, 0.0)),
        side_mean_px=float(np.mean(side_lengths)),
        side_min_px=float(np.min(side_lengths)),
        distance_m=distance_m,
        reprojection_error_px=float(reprojection_error_px),
        view_angle_deg=marker_view_angle_deg(T_cam_marker),
    )


def covariance_view_penalty(view_angle_deg: float, cfg: MarkerCovarianceModelConfig) -> float:
    cos_view = math.cos(math.radians(clamp(float(view_angle_deg), 0.0, 89.9)))
    cos_limit = math.cos(math.radians(clamp(float(cfg.max_view_angle_deg), 0.0, 89.9)))
    penalty = 1.0 / max(cos_view, cos_limit, 1e-6)
    return clamp(penalty, 1.0, max(1.0, float(cfg.max_view_penalty)))


def rotate_covariance_diag(diag: np.ndarray, R_target_source: np.ndarray) -> np.ndarray:
    values = np.asarray(diag, dtype=float).reshape(3)
    R = np.asarray(R_target_source, dtype=float).reshape(3, 3)
    cov = R @ np.diag(np.maximum(values, 0.0)) @ R.T
    return np.maximum(np.diag(cov), 0.0)


def estimate_marker_covariance_v2(
    metrics: MarkerQualityMetrics,
    T_map_cam: np.ndarray,
    f_avg_px: float,
    temporal_stats: MarkerTemporalStats,
    geometry_score: float,
    cfg: MarkerCovarianceModelConfig,
) -> MarkerCovarianceEstimate:
    sigma_px = max(float(cfg.corner_noise_floor_px), float(metrics.reprojection_error_px))
    side_mean_px = max(float(metrics.side_mean_px), 1.0)
    f_avg = max(float(f_avg_px), 1.0)
    view_penalty = covariance_view_penalty(metrics.view_angle_deg, cfg)
    distance_m = max(float(metrics.distance_m), 0.0)
    geometry_penalty = clamp(1.0 - float(geometry_score), 0.0, 1.0)
    stability_factor = clamp(float(temporal_stats.stability_factor), 0.0, 1.0)

    sigma_xy_cam = cfg.min_xy_std_m + cfg.xy_std_px_gain * distance_m * sigma_px / f_avg
    sigma_z_cam = cfg.min_z_std_m + cfg.z_std_px_gain * distance_m * sigma_px / side_mean_px * view_penalty
    sigma_roll_pitch = math.radians(cfg.min_roll_pitch_std_deg) + (
        cfg.roll_pitch_std_px_gain * sigma_px / side_mean_px * view_penalty
    )
    sigma_yaw = math.radians(cfg.min_yaw_std_deg) + cfg.yaw_std_px_gain * sigma_px / side_mean_px

    sigma_xy_cam += cfg.temporal_missing_position_std_m * stability_factor
    sigma_z_cam += cfg.temporal_missing_position_std_m * stability_factor
    temporal_rot = math.radians(cfg.temporal_missing_rotation_std_deg) * stability_factor
    sigma_roll_pitch += temporal_rot
    sigma_yaw += temporal_rot

    if temporal_stats.correction_translation_delta_m is not None:
        temporal_pos = cfg.temporal_correction_position_gain * max(
            0.0, float(temporal_stats.correction_translation_delta_m)
        )
        sigma_xy_cam += temporal_pos
        sigma_z_cam += temporal_pos

    if temporal_stats.correction_rotation_delta_deg is not None:
        temporal_rot = math.radians(
            cfg.temporal_correction_rotation_gain * max(0.0, float(temporal_stats.correction_rotation_delta_deg))
        )
        sigma_roll_pitch += temporal_rot
        sigma_yaw += temporal_rot

    sigma_xy_cam += cfg.geometry_position_std_m * geometry_penalty
    sigma_z_cam += cfg.geometry_position_std_m * geometry_penalty
    geometry_rot = math.radians(cfg.geometry_rotation_std_deg) * geometry_penalty
    sigma_roll_pitch += geometry_rot
    sigma_yaw += geometry_rot

    sigma_xy_cam = clamp(sigma_xy_cam, cfg.min_xy_std_m, cfg.max_xy_std_m)
    sigma_z_cam = clamp(sigma_z_cam, cfg.min_z_std_m, cfg.max_z_std_m)
    sigma_roll_pitch = clamp(
        sigma_roll_pitch,
        math.radians(cfg.min_roll_pitch_std_deg),
        math.radians(cfg.max_roll_pitch_std_deg),
    )
    sigma_yaw = clamp(sigma_yaw, math.radians(cfg.min_yaw_std_deg), math.radians(cfg.max_yaw_std_deg))

    R_map_cam = np.asarray(T_map_cam, dtype=float).reshape(4, 4)[:3, :3]
    pos_diag_map = rotate_covariance_diag(
        np.array([sigma_xy_cam**2, sigma_xy_cam**2, sigma_z_cam**2], dtype=float),
        R_map_cam,
    )
    rot_diag_map = rotate_covariance_diag(
        np.array([sigma_roll_pitch**2, sigma_roll_pitch**2, sigma_yaw**2], dtype=float),
        R_map_cam,
    )

    max_position_var = max(cfg.max_xy_std_m, cfg.max_z_std_m) ** 2
    max_rotation_var = math.radians(max(cfg.max_roll_pitch_std_deg, cfg.max_yaw_std_deg)) ** 2
    diag = np.concatenate(
        [
            np.clip(pos_diag_map, min(cfg.min_xy_std_m, cfg.min_z_std_m) ** 2, max_position_var),
            np.clip(
                rot_diag_map,
                math.radians(min(cfg.min_roll_pitch_std_deg, cfg.min_yaw_std_deg)) ** 2,
                max_rotation_var,
            ),
        ]
    )
    std_diag = covariance_diag_to_std(diag)
    camera_std_diag = np.array([sigma_xy_cam, sigma_xy_cam, sigma_z_cam, sigma_roll_pitch, sigma_roll_pitch, sigma_yaw])
    return MarkerCovarianceEstimate(
        diag=diag,
        std_diag=std_diag,
        camera_std_diag=camera_std_diag,
        sigma_px=sigma_px,
        view_penalty=view_penalty,
    )


def compute_marker_map_poses(T_map_marker: np.ndarray, T_cam_marker: np.ndarray, T_cam_imu: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return T_map_cam and T_map_imu from the solvePnP marker pose.

    OpenCV solvePnP estimates T_cam_marker, i.e. marker frame -> camera optical
    frame. The camera pose in the marker frame is therefore inverse(T_cam_marker).
    Using T_cam_marker directly here flips the physical motion direction in RViz.
    """
    T_marker_cam = T_inv(T_cam_marker)
    T_map_cam = T_map_marker @ T_marker_cam
    T_map_imu = T_map_cam @ T_cam_imu
    return T_map_cam, T_map_imu


def T_cam_body_display() -> np.ndarray:
    """Return camera optical frame <- visualization body frame.

    The display body frame is only for RViz/user interpretation:
      +X forward through the lens, +Y left, +Z up.

    The camera optical frame remains the ROS/OpenCV convention:
      +X right, +Y down, +Z forward.
    """
    T = np.eye(4)
    T[:3, :3] = np.array(
        [
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ],
        dtype=float,
    )
    return T


@dataclass
class MarkerMeasurement:
    stamp: Any
    stamp_sec: float
    marker_id: int
    marker_frame: str
    T_cam_marker: np.ndarray
    T_marker_cam: np.ndarray
    T_map_cam: np.ndarray
    T_map_imu: np.ndarray
    T_map_marker: np.ndarray
    area_px2: float
    sqrt_area_px: float
    side_mean_px: float
    side_min_px: float
    distance_m: float
    reprojection_error_px: float
    view_angle_deg: float
    view_penalty: float
    covariance_diag: np.ndarray
    covariance_std_diag: np.ndarray
    covariance_camera_std_diag: np.ndarray
    covariance_sigma_px: float
    stable_frames: int
    stable: bool
    stability_factor: float
    temporal_detection_translation_m: Optional[float]
    temporal_detection_rotation_deg: Optional[float]
    temporal_correction_translation_m: Optional[float]
    temporal_correction_rotation_deg: Optional[float]
    temporal_odom_match_dt: Optional[float]
    geometry_score: float
    image_width: int
    image_height: int


class ArucoMarkerPoseNode(Node):
    def __init__(self):
        super().__init__("aruco_marker_pose_node")

        self.declare_parameter("config_file", "")
        config_file = self.get_parameter("config_file").value
        if not config_file:
            raise RuntimeError("Parameter config_file is required")

        self.config = yaml.safe_load(Path(config_file).read_text())

        aruco_cfg = self.config["aruco"]
        self.dictionary_name = aruco_cfg.get("dictionary", "DICT_6X6_1000")
        self.default_marker_size_m = float(aruco_cfg.get("default_marker_size_m", 0.154))

        self.map_frame = self.config["frames"].get("map_frame", "marker_map")
        self.camera_frame = self.config["frames"].get("camera_frame", "camera_color_optical_frame")
        self.detected_camera_frame = self.config["frames"].get("detected_camera_frame", self.camera_frame)
        self.imu_frame = self.config["frames"].get("imu_frame", "imu")

        calib_path = self.config["calibration"]["kalibr_imucam_chain"]
        self.K, self.D, self.T_cam_imu, self.timeshift_cam_imu = load_kalibr_imucam(calib_path)

        topics_cfg = self.config.get("topics", {})
        self.image_topic = topics_cfg.get("image", "/head/d435i_head/color/image_raw")
        self.odom_topic = topics_cfg.get("openvins_odom", "/ov_msckf/odomimu")
        self.output_prefix = topics_cfg.get("output_prefix", "/head/marker_pose").rstrip("/")

        timing_cfg = self.config.get("timing", {})
        self.max_odom_match_dt = float(timing_cfg.get("max_odom_match_dt", 0.05))
        self.odom_buffer_seconds = float(timing_cfg.get("odom_buffer_seconds", 5.0))
        self.publish_uncorrected_before_lock = bool(timing_cfg.get("publish_uncorrected_before_lock", False))

        quality_cfg = self.config.get("quality", {})
        self.min_marker_area_px2 = float(quality_cfg.get("min_marker_area_px2", 800.0))
        self.max_marker_distance_m = float(quality_cfg.get("max_marker_distance_m", 2.0))
        self.stable_frames_required = int(quality_cfg.get("stable_frames_required", 8))
        self.max_marker_translation_jump_m = float(quality_cfg.get("max_marker_translation_jump_m", 0.20))
        self.max_marker_rotation_jump_deg = float(quality_cfg.get("max_marker_rotation_jump_deg", 15.0))
        self.max_reprojection_error_px = float(quality_cfg.get("max_reprojection_error_px", 3.0))
        self.border_margin_px = float(quality_cfg.get("border_margin_px", 8.0))
        self.min_corner_angle_deg = float(quality_cfg.get("min_corner_angle_deg", 25.0))
        self.max_corner_angle_deg = float(quality_cfg.get("max_corner_angle_deg", 155.0))
        self.min_border_geometry_score = float(quality_cfg.get("min_border_geometry_score", 0.35))

        correction_cfg = self.config.get("correction", {})
        self.reanchor_enabled = bool(correction_cfg.get("enabled", True))
        self.enable_periodic_marker_correction = bool(correction_cfg.get("enable_periodic_marker_correction", True))
        self.periodic_correction_interval_s = float(correction_cfg.get("periodic_correction_interval_s", 1.0))
        self.reanchor_cooldown_s = float(correction_cfg.get("reanchor_cooldown_s", 3.0))
        self.max_periodic_translation_correction_m = float(correction_cfg.get("max_periodic_translation_correction_m", 0.10))
        self.max_periodic_rotation_correction_deg = float(correction_cfg.get("max_periodic_rotation_correction_deg", 5.0))
        self.correction_chi2_gate = float(correction_cfg.get("correction_chi2_gate", 16.8))
        self.max_position_correction_step_m = float(correction_cfg.get("max_position_correction_step_m", 0.05))
        self.max_rotation_correction_step_deg = float(correction_cfg.get("max_rotation_correction_step_deg", 2.0))

        covariance_cfg = self.config.get("covariance", {})
        self.initial_correction_std_m = float(covariance_cfg.get("initial_correction_std_m", 0.20))
        self.initial_correction_std_deg = float(covariance_cfg.get("initial_correction_std_deg", 20.0))
        self.min_marker_position_std_m = float(covariance_cfg.get("min_marker_position_std_m", 0.02))
        self.max_marker_position_std_m = float(covariance_cfg.get("max_marker_position_std_m", 0.75))
        self.min_marker_rotation_std_deg = float(covariance_cfg.get("min_marker_rotation_std_deg", 2.0))
        self.max_marker_rotation_std_deg = float(covariance_cfg.get("max_marker_rotation_std_deg", 45.0))
        self.marker_covariance_area_ref_px2 = float(covariance_cfg.get("marker_covariance_area_ref_px2", 10000.0))
        self.marker_covariance_model = MarkerCovarianceModelConfig.from_mapping(
            covariance_cfg,
            legacy_min_position_std_m=self.min_marker_position_std_m,
            legacy_max_position_std_m=self.max_marker_position_std_m,
            legacy_min_rotation_std_deg=self.min_marker_rotation_std_deg,
            legacy_max_rotation_std_deg=self.max_marker_rotation_std_deg,
        )
        self.correction_position_growth_std_mps = float(covariance_cfg.get("correction_position_growth_std_mps", 0.03))
        self.correction_rotation_growth_std_degps = float(covariance_cfg.get("correction_rotation_growth_std_degps", 2.0))
        self.high_position_std_m = float(covariance_cfg.get("high_position_std_m", 2.0))
        self.high_rotation_std_deg = float(covariance_cfg.get("high_rotation_std_deg", 90.0))
        self.camera_f_avg_px = float(0.5 * (self.K[0, 0] + self.K[1, 1]))

        twist_cfg = self.config.get("twist", {})
        self.zero_twist_on_large_reanchor = bool(twist_cfg.get("zero_twist_on_large_reanchor", True))
        self.large_reanchor_translation_m = float(twist_cfg.get("large_reanchor_translation_m", 0.5))
        self.large_reanchor_rotation_deg = float(twist_cfg.get("large_reanchor_rotation_deg", 20.0))
        self.marker_velocity_enabled = bool(twist_cfg.get("marker_velocity_enabled", False))
        self.post_reanchor_settle_s = float(twist_cfg.get("post_reanchor_settle_s", 0.5))
        self.high_twist_std = float(twist_cfg.get("high_twist_std", 10.0))

        vio_cfg = self.config.get("vio_health", {})
        self.max_position_norm_m = float(vio_cfg.get("max_position_norm_m", 5.0))
        self.max_linear_velocity_mps = float(vio_cfg.get("max_linear_velocity_mps", 2.0))
        self.max_pose_covariance_trace = float(vio_cfg.get("max_pose_covariance_trace", 10.0))

        self.default_pose_cov_diag = np.array(
            [
                self.initial_correction_std_m**2,
                self.initial_correction_std_m**2,
                self.initial_correction_std_m**2,
                math.radians(self.initial_correction_std_deg) ** 2,
                math.radians(self.initial_correction_std_deg) ** 2,
                math.radians(self.initial_correction_std_deg) ** 2,
            ],
            dtype=float,
        )
        self.high_pose_cov_diag = np.array(
            [
                self.high_position_std_m**2,
                self.high_position_std_m**2,
                self.high_position_std_m**2,
                math.radians(self.high_rotation_std_deg) ** 2,
                math.radians(self.high_rotation_std_deg) ** 2,
                math.radians(self.high_rotation_std_deg) ** 2,
            ],
            dtype=float,
        )
        self.high_twist_cov_diag = np.array([self.high_twist_std**2] * 6, dtype=float)

        self.markers: dict[int, dict[str, Any]] = {}
        for marker_id_str, marker_cfg in self.config["markers"].items():
            marker_id = int(marker_id_str)
            T_map_marker = np.array(marker_cfg["T_map_marker"], dtype=float)
            size_m = float(marker_cfg.get("size_m", self.default_marker_size_m))
            frame_id = marker_cfg.get("frame_id", f"marker_{marker_id}")
            self.markers[marker_id] = {
                "frame_id": frame_id,
                "size_m": size_m,
                "T_map_marker": T_map_marker,
            }

        if not hasattr(cv2, "aruco"):
            raise RuntimeError("cv2.aruco is not available. Install OpenCV with aruco support.")

        dict_id = getattr(cv2.aruco, self.dictionary_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        # Keep the detector object disabled. Passing DetectorParameters to
        # cv2.aruco.detectMarkers segfaulted on Jetson OpenCV 4.6.0 Python.
        self.aruco_detector = None

        self.tf_broadcaster = TransformBroadcaster(self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_cb, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, odom_qos)

        self.camera_pose_raw_pub = self.create_publisher(PoseStamped, f"{self.output_prefix}/camera_pose_raw", 10)
        self.camera_body_pose_pub = self.create_publisher(PoseStamped, f"{self.output_prefix}/camera_body_pose", 10)
        self.imu_pose_pub = self.create_publisher(PoseWithCovarianceStamped, f"{self.output_prefix}/imu_pose", 10)
        self.corrected_odom_pub = self.create_publisher(Odometry, f"{self.output_prefix}/ov_corrected_odom", 20)
        self.marker_quality_pub = self.create_publisher(String, f"{self.output_prefix}/marker_quality", 10)
        self.active_marker_pub = self.create_publisher(Int32, f"{self.output_prefix}/active_marker_id", 10)
        self.marker_valid_pub = self.create_publisher(Bool, f"{self.output_prefix}/marker_valid", 10)
        self.vio_valid_pub = self.create_publisher(Bool, f"{self.output_prefix}/vio_valid", 10)
        self.reanchor_event_pub = self.create_publisher(String, f"{self.output_prefix}/reanchor_event", 10)
        self.reanchor_srv = self.create_service(Trigger, f"{self.output_prefix}/request_reanchor", self.request_reanchor_cb)

        self.odom_buffer: deque[Odometry] = deque()
        self.marker_histories: dict[int, deque[MarkerPoseHistoryEntry]] = {}
        self.last_marker_measurement: Optional[MarkerMeasurement] = None
        self.last_marker_valid = False
        self.last_vio_valid = False
        self.vio_valid = False
        self.vio_forced_invalid_until_sec: Optional[float] = None
        self.vio_forced_invalid_reason = ""
        self.last_odom_msg: Optional[Odometry] = None
        self.last_odom_wall_time_sec: Optional[float] = None

        self.T_map_global: Optional[np.ndarray] = None
        self.P_correction_diag = self.default_pose_cov_diag.copy()
        self.last_correction_time_sec: Optional[float] = None
        self.last_hard_reanchor_time_sec: Optional[float] = None
        self.last_large_reanchor_time_sec: Optional[float] = None
        self.last_correction_jump_translation_m = 0.0
        self.last_correction_jump_rotation_deg = 0.0
        self.last_event_key: Optional[tuple[Any, ...]] = None
        self.last_event_wall_time_sec = 0.0

        self.watchdog_timer = self.create_timer(2.0, self.qos_watchdog_cb)

        self.get_logger().info(f"Using marker config: {config_file}")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"OpenVINS odom topic: {self.odom_topic}")
        self.get_logger().info(f"Output prefix: {self.output_prefix}")
        self.get_logger().info(f"Dictionary: {self.dictionary_name}")
        self.get_logger().info(f"Known marker IDs: {sorted(self.markers.keys())}")
        self.get_logger().info(f"Kalibr timeshift_cam_imu: {self.timeshift_cam_imu:.6f} s")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def detect_markers(self, gray: np.ndarray):
        # Do not pass DetectorParameters here. On Jetson OpenCV 4.6.0 Python,
        # that call path segfaulted on live ROS image buffers.
        corners, ids, _rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict)
        return corners, ids

    def image_msg_to_gray(self, msg: Image) -> Optional[np.ndarray]:
        enc = msg.encoding.lower()
        h = msg.height
        w = msg.width
        step = msg.step
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ("rgb8", "bgr8"):
            channels = 3
            rows = data.reshape((h, step))
            img = rows[:, : w * channels].reshape((h, w, channels))
            img = np.ascontiguousarray(img)
            if enc == "rgb8":
                return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if enc in ("mono8", "8uc1"):
            rows = data.reshape((h, step))
            gray = rows[:, :w]
            return np.ascontiguousarray(gray)

        self.get_logger().warning(f"Unsupported image encoding for marker detector: {msg.encoding}")
        return None

    def image_cb(self, msg: Image) -> None:
        gray = self.image_msg_to_gray(msg)
        if gray is None:
            self.publish_marker_state(False, -1)
            return

        try:
            corners, ids = self.detect_markers(gray)
        except Exception as exc:
            self.get_logger().warning(f"ArUco detection failed: {exc}")
            self.publish_marker_state(False, -1)
            return

        if ids is None or len(ids) == 0:
            self.publish_marker_state(False, -1)
            return

        marker_ids = [int(marker_id_arr[0]) for marker_id_arr in ids]
        id_counts = Counter(marker_ids)
        candidates = []
        reject_reasons = []

        for idx, marker_id in enumerate(marker_ids):
            if marker_id not in self.markers:
                reject_reasons.append(f"unknown_marker_id_{marker_id}")
                continue
            if id_counts[marker_id] > 1:
                reject_reasons.append(f"duplicate_marker_id_{marker_id}")
                continue

            measurement, reason = self.build_marker_measurement(
                msg,
                gray.shape[1],
                gray.shape[0],
                marker_id,
                corners[idx].reshape(4, 2).astype(np.float32),
            )
            if measurement is None:
                reject_reasons.append(reason)
                continue
            candidates.append(measurement)

        if not candidates:
            self.publish_marker_state(False, -1)
            if reject_reasons:
                self.publish_reanchor_event(
                    stamp_sec=stamp_to_sec(msg.header.stamp),
                    marker_id=-1,
                    marker_valid=False,
                    correction_accepted=False,
                    correction_mode="rejected",
                    reason=";".join(sorted(set(reject_reasons))),
                )
            return

        candidates.sort(key=lambda m: (not m.stable, m.reprojection_error_px, -m.area_px2))
        measurement = candidates[0]
        self.last_marker_measurement = measurement

        self.publish_marker_state(True, measurement.marker_id)
        self.publish_marker_poses(measurement)
        self.publish_marker_quality(measurement, hard_gate_status="accepted")
        self.try_apply_marker_correction(measurement, force_manual=False)

    def build_marker_measurement(
        self,
        msg: Image,
        image_width: int,
        image_height: int,
        marker_id: int,
        image_points: np.ndarray,
    ) -> tuple[Optional[MarkerMeasurement], str]:
        if not is_finite_array(image_points):
            return None, "non_finite_corners"

        area = abs(float(cv2.contourArea(image_points)))
        if area < self.min_marker_area_px2:
            return None, "marker_area_too_small"

        geometry_ok, geometry_score, geometry_reason = self.check_corner_geometry(image_points, image_width, image_height)
        if not geometry_ok:
            return None, geometry_reason

        marker_cfg = self.markers[marker_id]
        size_m = marker_cfg["size_m"]
        obj_points = marker_object_points(size_m)

        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            image_points,
            self.K,
            self.D,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None, "solvepnp_failed"

        R_cam_marker, _ = cv2.Rodrigues(rvec)
        T_cam_marker = np.eye(4)
        T_cam_marker[:3, :3] = R_cam_marker
        T_cam_marker[:3, 3] = tvec.reshape(3)

        if not is_finite_array(T_cam_marker):
            return None, "non_finite_pose"

        distance_m = float(np.linalg.norm(T_cam_marker[:3, 3]))
        if distance_m > self.max_marker_distance_m:
            return None, "marker_too_far"

        reprojection_error_px = self.compute_reprojection_error(obj_points, image_points, rvec, tvec)
        if reprojection_error_px > self.max_reprojection_error_px:
            return None, "reprojection_error_too_high"

        metrics = marker_quality_metrics(
            image_points,
            T_cam_marker,
            reprojection_error_px,
            area_px2=area,
        )

        T_map_marker = marker_cfg["T_map_marker"]
        T_map_cam, T_map_imu = compute_marker_map_poses(T_map_marker, T_cam_marker, self.T_cam_imu)
        T_marker_cam = T_inv(T_cam_marker)

        jump_ok, jump_reason = self.check_marker_pose_jump(marker_id, T_map_imu)
        if not jump_ok:
            return None, jump_reason

        T_meas_map_global: Optional[np.ndarray] = None
        odom_match_dt: Optional[float] = None
        matched_odom, odom_match_dt = self.find_nearest_odom(stamp_to_sec(msg.header.stamp))
        if self.vio_valid and matched_odom is not None:
            T_meas_map_global = T_map_imu @ T_inv(odom_to_T(matched_odom))

        temporal_stats = self.record_marker_history(marker_id, T_map_imu, T_meas_map_global, odom_match_dt)
        covariance_estimate = self.estimate_marker_covariance(
            metrics,
            T_map_cam,
            geometry_score,
            temporal_stats,
        )

        return (
            MarkerMeasurement(
                stamp=msg.header.stamp,
                stamp_sec=stamp_to_sec(msg.header.stamp),
                marker_id=marker_id,
                marker_frame=marker_cfg["frame_id"],
                T_cam_marker=T_cam_marker,
                T_marker_cam=T_marker_cam,
                T_map_cam=T_map_cam,
                T_map_imu=T_map_imu,
                T_map_marker=T_map_marker,
                area_px2=metrics.area_px2,
                sqrt_area_px=metrics.sqrt_area_px,
                side_mean_px=metrics.side_mean_px,
                side_min_px=metrics.side_min_px,
                distance_m=metrics.distance_m,
                reprojection_error_px=metrics.reprojection_error_px,
                view_angle_deg=metrics.view_angle_deg,
                view_penalty=covariance_estimate.view_penalty,
                covariance_diag=covariance_estimate.diag,
                covariance_std_diag=covariance_estimate.std_diag,
                covariance_camera_std_diag=covariance_estimate.camera_std_diag,
                covariance_sigma_px=covariance_estimate.sigma_px,
                stable_frames=temporal_stats.stable_frames,
                stable=temporal_stats.stable,
                stability_factor=temporal_stats.stability_factor,
                temporal_detection_translation_m=temporal_stats.detection_translation_delta_m,
                temporal_detection_rotation_deg=temporal_stats.detection_rotation_delta_deg,
                temporal_correction_translation_m=temporal_stats.correction_translation_delta_m,
                temporal_correction_rotation_deg=temporal_stats.correction_rotation_delta_deg,
                temporal_odom_match_dt=temporal_stats.odom_match_dt,
                geometry_score=geometry_score,
                image_width=image_width,
                image_height=image_height,
            ),
            "",
        )

    def check_corner_geometry(self, image_points: np.ndarray, image_width: int, image_height: int) -> tuple[bool, float, str]:
        if not cv2.isContourConvex(image_points.reshape(-1, 1, 2)):
            return False, 0.0, "marker_corners_not_convex"

        xs = image_points[:, 0]
        ys = image_points[:, 1]
        near_border = (
            np.min(xs) < self.border_margin_px
            or np.max(xs) > image_width - self.border_margin_px
            or np.min(ys) < self.border_margin_px
            or np.max(ys) > image_height - self.border_margin_px
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
        if min_angle < self.min_corner_angle_deg or max_angle > self.max_corner_angle_deg:
            return False, 0.0, "marker_corner_angles_bad"

        min_side = min(side_lengths)
        max_side = max(side_lengths)
        side_ratio = min_side / max(max_side, 1e-6)
        angle_spread = max_angle - min_angle
        geometry_score = max(0.0, min(1.0, 0.5 * side_ratio + 0.5 * (1.0 - angle_spread / 180.0)))

        if near_border and geometry_score < self.min_border_geometry_score:
            return False, geometry_score, "marker_near_border_poor_geometry"

        return True, geometry_score, ""

    def compute_reprojection_error(self, obj_points: np.ndarray, image_points: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> float:
        projected, _ = cv2.projectPoints(obj_points, rvec, tvec, self.K, self.D)
        projected = projected.reshape(4, 2)
        residuals = projected - image_points
        return float(math.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))

    def check_marker_pose_jump(self, marker_id: int, T_map_imu: np.ndarray) -> tuple[bool, str]:
        history = self.marker_histories.get(marker_id)
        if not history:
            return True, ""
        last_T_map_imu = history[-1].T_map_imu
        trans_jump = float(np.linalg.norm(T_map_imu[:3, 3] - last_T_map_imu[:3, 3]))
        rot_jump = rotation_angle_deg(T_map_imu[:3, :3] @ last_T_map_imu[:3, :3].T)
        if trans_jump > self.max_marker_translation_jump_m:
            return False, "marker_translation_jump"
        if rot_jump > self.max_marker_rotation_jump_deg:
            return False, "marker_rotation_jump"
        return True, ""

    def record_marker_history(
        self,
        marker_id: int,
        T_map_imu: np.ndarray,
        T_meas_map_global: Optional[np.ndarray],
        odom_match_dt: Optional[float],
    ) -> MarkerTemporalStats:
        if marker_id not in self.marker_histories:
            self.marker_histories[marker_id] = deque(maxlen=max(self.stable_frames_required, 1))

        history = self.marker_histories[marker_id]
        previous = history[-1] if history else None
        detection_translation_delta_m = None
        detection_rotation_delta_deg = None
        correction_translation_delta_m = None
        correction_rotation_delta_deg = None

        if previous is not None:
            detection_translation_delta_m = float(np.linalg.norm(T_map_imu[:3, 3] - previous.T_map_imu[:3, 3]))
            detection_rotation_delta_deg = rotation_angle_deg(T_map_imu[:3, :3] @ previous.T_map_imu[:3, :3].T)
            if T_meas_map_global is not None and previous.T_meas_map_global is not None:
                correction_translation_delta_m = float(
                    np.linalg.norm(T_meas_map_global[:3, 3] - previous.T_meas_map_global[:3, 3])
                )
                correction_rotation_delta_deg = rotation_angle_deg(
                    T_meas_map_global[:3, :3] @ previous.T_meas_map_global[:3, :3].T
                )

        history.append(
            MarkerPoseHistoryEntry(
                T_map_imu=T_map_imu.copy(),
                T_meas_map_global=None if T_meas_map_global is None else T_meas_map_global.copy(),
            )
        )
        stable_frames = len(history)
        stable = stable_frames >= self.stable_frames_required
        stability_missing = max(0, self.stable_frames_required - stable_frames)
        stability_factor = stability_missing / max(1, self.stable_frames_required)
        return MarkerTemporalStats(
            stable_frames=stable_frames,
            stable=stable,
            stability_factor=stability_factor,
            detection_translation_delta_m=detection_translation_delta_m,
            detection_rotation_delta_deg=detection_rotation_delta_deg,
            correction_translation_delta_m=correction_translation_delta_m,
            correction_rotation_delta_deg=correction_rotation_delta_deg,
            odom_match_dt=odom_match_dt,
        )

    def estimate_marker_covariance(
        self,
        metrics: MarkerQualityMetrics,
        T_map_cam: np.ndarray,
        geometry_score: float,
        temporal_stats: MarkerTemporalStats,
    ) -> MarkerCovarianceEstimate:
        return estimate_marker_covariance_v2(
            metrics=metrics,
            T_map_cam=T_map_cam,
            f_avg_px=self.camera_f_avg_px,
            temporal_stats=temporal_stats,
            geometry_score=geometry_score,
            cfg=self.marker_covariance_model,
        )

    def publish_marker_state(self, marker_valid: bool, marker_id: int) -> None:
        valid_msg = Bool()
        valid_msg.data = bool(marker_valid)
        self.marker_valid_pub.publish(valid_msg)

        id_msg = Int32()
        id_msg.data = int(marker_id)
        self.active_marker_pub.publish(id_msg)
        self.last_marker_valid = bool(marker_valid)

    def publish_marker_poses(self, measurement: MarkerMeasurement) -> None:
        # Raw pose: camera optical frame in marker_<id>. In ROS/OpenCV optical
        # frames +Z (blue in RViz axes) points forward through the lens.
        # The RViz arrow/red axis is +X and is not the camera viewing direction.
        raw_msg = PoseStamped()
        raw_msg.header.stamp = measurement.stamp
        raw_msg.header.frame_id = measurement.marker_frame
        fill_pose(raw_msg.pose, measurement.T_marker_cam)
        self.camera_pose_raw_pub.publish(raw_msg)

        # Visualization-only body pose: +X forward, +Y left, +Z up.
        # This is not used for correction, covariance, or OpenVINS alignment.
        T_map_body = measurement.T_map_cam @ T_cam_body_display()
        body_msg = PoseStamped()
        body_msg.header.stamp = measurement.stamp
        body_msg.header.frame_id = self.map_frame
        fill_pose(body_msg.pose, T_map_body)
        self.camera_body_pose_pub.publish(body_msg)

        # True calibrated IMU pose in marker_map. Its axes are the calibrated
        # IMU frame and may not look like an intuitive body frame in RViz.
        imu_msg = PoseWithCovarianceStamped()
        imu_msg.header.stamp = measurement.stamp
        imu_msg.header.frame_id = self.map_frame
        fill_pose(imu_msg.pose.pose, measurement.T_map_imu)
        imu_msg.pose.covariance = covariance_from_diag(measurement.covariance_diag)
        self.imu_pose_pub.publish(imu_msg)

        self.publish_tf(measurement.stamp, self.map_frame, measurement.marker_frame, measurement.T_map_marker)
        self.publish_tf(measurement.stamp, measurement.marker_frame, f"{self.detected_camera_frame}_raw", measurement.T_marker_cam)
        self.publish_tf(measurement.stamp, self.map_frame, f"{self.detected_camera_frame}_from_marker", measurement.T_map_cam)
        self.publish_tf(measurement.stamp, self.map_frame, f"{self.detected_camera_frame}_body_display", T_map_body)
        self.publish_tf(measurement.stamp, self.map_frame, f"{self.imu_frame}_from_marker", measurement.T_map_imu)

    def marker_quality_payload(self, measurement: MarkerMeasurement, hard_gate_status: str) -> dict[str, Any]:
        std = measurement.covariance_std_diag
        cam_std = measurement.covariance_camera_std_diag
        return {
            "marker_id": int(measurement.marker_id),
            "hard_gate_status": hard_gate_status,
            "hard_gate_passed": hard_gate_status == "accepted",
            "marker_area_px2": round(float(measurement.area_px2), 3),
            "marker_sqrt_area_px": round(float(measurement.sqrt_area_px), 3),
            "marker_side_mean_px": round(float(measurement.side_mean_px), 3),
            "marker_side_min_px": round(float(measurement.side_min_px), 3),
            "marker_reprojection_error_px": round(float(measurement.reprojection_error_px), 4),
            "marker_distance_m": round(float(measurement.distance_m), 4),
            "marker_view_angle_deg": round(float(measurement.view_angle_deg), 4),
            "marker_view_penalty": round(float(measurement.view_penalty), 4),
            "marker_geometry_score": round(float(measurement.geometry_score), 4),
            "marker_stable_frames": int(measurement.stable_frames),
            "marker_stable": bool(measurement.stable),
            "marker_stability_factor": round(float(measurement.stability_factor), 4),
            "marker_temporal_detection_translation_m": None
            if measurement.temporal_detection_translation_m is None
            else round(float(measurement.temporal_detection_translation_m), 6),
            "marker_temporal_detection_rotation_deg": None
            if measurement.temporal_detection_rotation_deg is None
            else round(float(measurement.temporal_detection_rotation_deg), 6),
            "marker_temporal_correction_translation_m": None
            if measurement.temporal_correction_translation_m is None
            else round(float(measurement.temporal_correction_translation_m), 6),
            "marker_temporal_correction_rotation_deg": None
            if measurement.temporal_correction_rotation_deg is None
            else round(float(measurement.temporal_correction_rotation_deg), 6),
            "marker_temporal_odom_match_dt": None
            if measurement.temporal_odom_match_dt is None
            else round(float(measurement.temporal_odom_match_dt), 6),
            "marker_covariance_sigma_px": round(float(measurement.covariance_sigma_px), 4),
            "marker_covariance_std_x_m": round(float(std[0]), 6),
            "marker_covariance_std_y_m": round(float(std[1]), 6),
            "marker_covariance_std_z_m": round(float(std[2]), 6),
            "marker_covariance_std_roll_deg": round(math.degrees(float(std[3])), 6),
            "marker_covariance_std_pitch_deg": round(math.degrees(float(std[4])), 6),
            "marker_covariance_std_yaw_deg": round(math.degrees(float(std[5])), 6),
            "marker_covariance_camera_std_x_m": round(float(cam_std[0]), 6),
            "marker_covariance_camera_std_y_m": round(float(cam_std[1]), 6),
            "marker_covariance_camera_std_z_m": round(float(cam_std[2]), 6),
            "marker_covariance_camera_std_roll_deg": round(math.degrees(float(cam_std[3])), 6),
            "marker_covariance_camera_std_pitch_deg": round(math.degrees(float(cam_std[4])), 6),
            "marker_covariance_camera_std_yaw_deg": round(math.degrees(float(cam_std[5])), 6),
        }

    def publish_marker_quality(self, measurement: MarkerMeasurement, hard_gate_status: str) -> None:
        event = {
            "event_type": "marker_quality",
            "stamp": round(float(measurement.stamp_sec), 9),
            "frame_id": self.map_frame,
        }
        event.update(self.marker_quality_payload(measurement, hard_gate_status))
        msg = String()
        msg.data = json.dumps(event, separators=(",", ":"))
        self.marker_quality_pub.publish(msg)

    def odom_cb(self, msg: Odometry) -> None:
        self.last_odom_msg = msg
        self.last_odom_wall_time_sec = self.now_sec()
        self.odom_buffer.append(msg)
        self.prune_odom_buffer(stamp_to_sec(msg.header.stamp))

        self.update_vio_valid(msg)

        if self.T_map_global is None:
            if self.publish_uncorrected_before_lock:
                self.publish_uncorrected_odom(msg)
            else:
                self.publish_reanchor_event(
                    stamp_sec=stamp_to_sec(msg.header.stamp),
                    marker_id=-1,
                    marker_valid=False,
                    correction_accepted=False,
                    correction_mode="not_initialized",
                    reason="correction_not_initialized",
                )
            return

        self.publish_corrected_odom(msg)

    def prune_odom_buffer(self, newest_stamp_sec: float) -> None:
        while self.odom_buffer and newest_stamp_sec - stamp_to_sec(self.odom_buffer[0].header.stamp) > self.odom_buffer_seconds:
            self.odom_buffer.popleft()

    def find_nearest_odom(self, image_stamp_sec: float) -> tuple[Optional[Odometry], Optional[float]]:
        if not self.odom_buffer:
            return None, None
        best_msg = min(self.odom_buffer, key=lambda msg: abs(stamp_to_sec(msg.header.stamp) - image_stamp_sec))
        dt = abs(stamp_to_sec(best_msg.header.stamp) - image_stamp_sec)
        if dt > self.max_odom_match_dt:
            return None, dt
        return best_msg, dt

    def update_vio_valid(self, msg: Odometry, force_invalid: bool = False) -> None:
        valid = not force_invalid
        reason = ""
        stamp_sec = stamp_to_sec(msg.header.stamp)

        if (
            self.vio_forced_invalid_until_sec is not None
            and stamp_sec <= self.vio_forced_invalid_until_sec
        ):
            valid = False
            reason = self.vio_forced_invalid_reason

        T_global_imu = odom_to_T(msg)
        if not is_finite_array(T_global_imu):
            valid = False
            reason = "non_finite_odom_pose"

        position = T_global_imu[:3, 3]
        if valid and float(np.linalg.norm(position)) > self.max_position_norm_m:
            valid = False
            reason = "position_outside_workspace"

        linear = msg.twist.twist.linear
        speed = math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)
        if valid and (not math.isfinite(speed) or speed > self.max_linear_velocity_mps):
            valid = False
            reason = "linear_velocity_implausible"

        pose_diag = pose_covariance_diag(msg, self.default_pose_cov_diag)
        pose_trace = float(np.sum(pose_diag))
        if valid and (not math.isfinite(pose_trace) or pose_trace > self.max_pose_covariance_trace):
            valid = False
            reason = "pose_covariance_too_large"

        self.vio_valid = valid
        out = Bool()
        out.data = bool(valid)
        self.vio_valid_pub.publish(out)

        if valid != self.last_vio_valid:
            if valid:
                self.get_logger().info("VIO health is valid")
            else:
                self.get_logger().warning(f"VIO health is invalid: {reason}")
            self.last_vio_valid = valid

    def hold_vio_invalid(self, stamp_sec: float, reason: str) -> None:
        hold_s = max(1.0, self.periodic_correction_interval_s)
        self.vio_forced_invalid_until_sec = stamp_sec + hold_s
        self.vio_forced_invalid_reason = reason

    def try_apply_marker_correction(self, measurement: MarkerMeasurement, force_manual: bool) -> tuple[bool, str]:
        if not self.reanchor_enabled:
            self.publish_reanchor_event_from_measurement(
                measurement,
                correction_accepted=False,
                correction_mode="rejected",
                reason="correction_disabled",
            )
            return False, "correction_disabled"

        matched_odom, odom_dt = self.find_nearest_odom(measurement.stamp_sec)
        if matched_odom is None:
            self.publish_reanchor_event_from_measurement(
                measurement,
                correction_accepted=False,
                correction_mode="marker_only_no_odom_match",
                reason="odom_match_timeout",
                odom_match_dt=odom_dt,
            )
            return False, "odom_match_timeout"

        if not measurement.stable:
            mode = "not_initialized" if self.T_map_global is None else "rejected"
            self.publish_reanchor_event_from_measurement(
                measurement,
                correction_accepted=False,
                correction_mode=mode,
                reason="marker_not_stable",
                odom_match_dt=odom_dt,
            )
            return False, "marker_not_stable"

        T_global_imu = odom_to_T(matched_odom)
        T_meas_map_global = measurement.T_map_imu @ T_inv(T_global_imu)
        P_ov_diag = pose_covariance_diag(matched_odom, self.default_pose_cov_diag)

        if self.T_map_global is None:
            self.hard_set_correction(T_meas_map_global, measurement.covariance_diag, P_ov_diag, measurement.stamp_sec)
            self.publish_reanchor_event_from_measurement(
                measurement,
                correction_accepted=True,
                correction_mode="initial_lock",
                reason="accepted",
                odom_match_dt=odom_dt,
            )
            return True, "accepted"

        innovation = self.correction_innovation(T_meas_map_global)
        trans_norm = float(np.linalg.norm(innovation[:3]))
        rot_deg = math.degrees(float(np.linalg.norm(innovation[3:])))

        if force_manual:
            self.hard_set_correction(T_meas_map_global, measurement.covariance_diag, P_ov_diag, measurement.stamp_sec)
            self.mark_large_reanchor_if_needed(trans_norm, rot_deg, measurement.stamp_sec)
            self.publish_reanchor_event_from_measurement(
                measurement,
                correction_accepted=True,
                correction_mode="manual_hard_reanchor",
                reason="accepted",
                odom_match_dt=odom_dt,
                correction_translation_norm_m=trans_norm,
                correction_rotation_deg=rot_deg,
            )
            return True, "accepted"

        if not self.vio_valid:
            if self.last_hard_reanchor_time_sec is not None and measurement.stamp_sec - self.last_hard_reanchor_time_sec < self.reanchor_cooldown_s:
                self.publish_reanchor_event_from_measurement(
                    measurement,
                    correction_accepted=False,
                    correction_mode="rejected",
                    reason="reanchor_cooldown_active",
                    odom_match_dt=odom_dt,
                    correction_translation_norm_m=trans_norm,
                    correction_rotation_deg=rot_deg,
                )
                return False, "reanchor_cooldown_active"
            self.hard_set_correction(T_meas_map_global, measurement.covariance_diag, P_ov_diag, measurement.stamp_sec)
            self.mark_large_reanchor_if_needed(trans_norm, rot_deg, measurement.stamp_sec, force=True)
            self.publish_reanchor_event_from_measurement(
                measurement,
                correction_accepted=True,
                correction_mode="vio_invalid_hard_reanchor",
                reason="accepted",
                odom_match_dt=odom_dt,
                correction_translation_norm_m=trans_norm,
                correction_rotation_deg=rot_deg,
            )
            return True, "accepted"

        if not self.enable_periodic_marker_correction:
            self.publish_reanchor_event_from_measurement(
                measurement,
                correction_accepted=False,
                correction_mode="rejected",
                reason="periodic_correction_disabled",
                odom_match_dt=odom_dt,
                correction_translation_norm_m=trans_norm,
                correction_rotation_deg=rot_deg,
            )
            return False, "periodic_correction_disabled"

        if self.last_correction_time_sec is not None and measurement.stamp_sec - self.last_correction_time_sec < self.periodic_correction_interval_s:
            self.publish_reanchor_event_from_measurement(
                measurement,
                correction_accepted=False,
                correction_mode="rejected",
                reason="periodic_interval_wait",
                odom_match_dt=odom_dt,
                correction_translation_norm_m=trans_norm,
                correction_rotation_deg=rot_deg,
            )
            return False, "periodic_interval_wait"

        if trans_norm > self.max_periodic_translation_correction_m or rot_deg > self.max_periodic_rotation_correction_deg:
            self.hold_vio_invalid(measurement.stamp_sec, "marker_innovation_jump")
            self.update_vio_valid(matched_odom, force_invalid=True)
            self.publish_reanchor_event_from_measurement(
                measurement,
                correction_accepted=False,
                correction_mode="rejected",
                reason="valid_vio_correction_jump_too_large",
                odom_match_dt=odom_dt,
                correction_translation_norm_m=trans_norm,
                correction_rotation_deg=rot_deg,
            )
            return False, "valid_vio_correction_jump_too_large"

        chi2 = self.innovation_chi2(innovation, measurement.covariance_diag, P_ov_diag)
        if chi2 > self.correction_chi2_gate:
            self.hold_vio_invalid(measurement.stamp_sec, "marker_innovation_chi2")
            self.update_vio_valid(matched_odom, force_invalid=True)
            self.publish_reanchor_event_from_measurement(
                measurement,
                correction_accepted=False,
                correction_mode="rejected",
                reason="innovation_chi2_rejected",
                odom_match_dt=odom_dt,
                correction_translation_norm_m=trans_norm,
                correction_rotation_deg=rot_deg,
                chi2=chi2,
            )
            return False, "innovation_chi2_rejected"

        self.soft_update_correction(T_meas_map_global, measurement.covariance_diag, P_ov_diag, measurement.stamp_sec)
        self.publish_reanchor_event_from_measurement(
            measurement,
            correction_accepted=True,
            correction_mode="periodic_soft_update",
            reason="accepted",
            odom_match_dt=odom_dt,
            correction_translation_norm_m=trans_norm,
            correction_rotation_deg=rot_deg,
            chi2=chi2,
        )
        return True, "accepted"

    def correction_innovation(self, T_meas_map_global: np.ndarray) -> np.ndarray:
        if self.T_map_global is None:
            return np.zeros(6, dtype=float)
        trans_error = T_meas_map_global[:3, 3] - self.T_map_global[:3, 3]
        R_error = T_meas_map_global[:3, :3] @ self.T_map_global[:3, :3].T
        rot_error = R_to_rotvec(R_error)
        return np.concatenate([trans_error, rot_error])

    def innovation_chi2(self, innovation: np.ndarray, R_marker_diag: np.ndarray, P_ov_diag: np.ndarray) -> float:
        S_diag = np.maximum(self.P_correction_diag + R_marker_diag + P_ov_diag, 1e-9)
        return float(np.sum((innovation * innovation) / S_diag))

    def hard_set_correction(
        self,
        T_meas_map_global: np.ndarray,
        R_marker_diag: np.ndarray,
        P_ov_diag: np.ndarray,
        stamp_sec: float,
    ) -> None:
        self.T_map_global = T_meas_map_global.copy()
        self.P_correction_diag = np.clip(R_marker_diag + P_ov_diag, 1e-8, self.high_pose_cov_diag)
        self.last_correction_time_sec = stamp_sec
        self.last_hard_reanchor_time_sec = stamp_sec

    def soft_update_correction(
        self,
        T_meas_map_global: np.ndarray,
        R_marker_diag: np.ndarray,
        P_ov_diag: np.ndarray,
        stamp_sec: float,
    ) -> None:
        if self.T_map_global is None:
            self.hard_set_correction(T_meas_map_global, R_marker_diag, P_ov_diag, stamp_sec)
            return

        innovation = self.correction_innovation(T_meas_map_global)
        S_diag = np.maximum(self.P_correction_diag + R_marker_diag + P_ov_diag, 1e-9)
        K = np.clip(self.P_correction_diag / S_diag, 0.0, 1.0)
        step = K * innovation

        trans_step_norm = float(np.linalg.norm(step[:3]))
        if trans_step_norm > self.max_position_correction_step_m:
            step[:3] *= self.max_position_correction_step_m / max(trans_step_norm, 1e-9)

        rot_step_norm = float(np.linalg.norm(step[3:]))
        max_rot_step = math.radians(self.max_rotation_correction_step_deg)
        if rot_step_norm > max_rot_step:
            step[3:] *= max_rot_step / max(rot_step_norm, 1e-9)

        T_new = self.T_map_global.copy()
        T_new[:3, 3] = self.T_map_global[:3, 3] + step[:3]
        T_new[:3, :3] = rotvec_to_R(step[3:]) @ self.T_map_global[:3, :3]
        self.T_map_global = T_new
        self.P_correction_diag = np.clip((1.0 - K) * self.P_correction_diag, 1e-8, self.high_pose_cov_diag)
        self.last_correction_time_sec = stamp_sec
        self.last_correction_jump_translation_m = float(np.linalg.norm(innovation[:3]))
        self.last_correction_jump_rotation_deg = math.degrees(float(np.linalg.norm(innovation[3:])))

    def mark_large_reanchor_if_needed(self, trans_norm: float, rot_deg: float, stamp_sec: float, force: bool = False) -> None:
        large = force or trans_norm > self.large_reanchor_translation_m or rot_deg > self.large_reanchor_rotation_deg
        if self.zero_twist_on_large_reanchor and large:
            self.last_large_reanchor_time_sec = stamp_sec
        self.last_correction_jump_translation_m = trans_norm
        self.last_correction_jump_rotation_deg = rot_deg

    def publish_uncorrected_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = f"{msg.child_frame_id}_uncorrected"
        out.pose = msg.pose
        out.twist = msg.twist
        self.corrected_odom_pub.publish(out)

    def publish_corrected_odom(self, msg: Odometry) -> None:
        if self.T_map_global is None:
            return

        T_global_imu = odom_to_T(msg)
        T_map_imu_corrected = self.T_map_global @ T_global_imu

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = f"{self.imu_frame}_openvins_corrected"
        fill_pose(out.pose.pose, T_map_imu_corrected)
        out.pose.covariance = covariance_from_diag(self.corrected_pose_covariance_diag(msg))

        self.fill_corrected_twist(out, msg, T_global_imu, stamp_to_sec(msg.header.stamp))
        self.corrected_odom_pub.publish(out)
        self.publish_tf(msg.header.stamp, self.map_frame, out.child_frame_id, T_map_imu_corrected)

    def corrected_pose_covariance_diag(self, msg: Odometry) -> np.ndarray:
        P_ov_diag = pose_covariance_diag(msg, self.default_pose_cov_diag)
        P_corr = self.grown_correction_covariance(stamp_to_sec(msg.header.stamp))
        diag = np.clip(P_ov_diag + P_corr, 1e-8, self.high_pose_cov_diag)
        if not self.vio_valid and not self.marker_recent(stamp_to_sec(msg.header.stamp)):
            diag = np.maximum(diag, self.high_pose_cov_diag)
        return diag

    def grown_correction_covariance(self, stamp_sec: float) -> np.ndarray:
        P = self.P_correction_diag.copy()
        if self.last_correction_time_sec is None:
            return np.maximum(P, self.default_pose_cov_diag)
        dt = max(0.0, stamp_sec - self.last_correction_time_sec)
        pos_growth = (self.correction_position_growth_std_mps * dt) ** 2
        rot_growth = (math.radians(self.correction_rotation_growth_std_degps) * dt) ** 2
        P[:3] += pos_growth
        P[3:] += rot_growth
        return np.clip(P, 1e-8, self.high_pose_cov_diag)

    def marker_recent(self, stamp_sec: float) -> bool:
        if self.last_marker_measurement is None:
            return False
        return abs(stamp_sec - self.last_marker_measurement.stamp_sec) <= 0.5

    def fill_corrected_twist(self, out: Odometry, source: Odometry, T_global_imu: np.ndarray, stamp_sec: float) -> None:
        settle = (
            self.last_large_reanchor_time_sec is not None
            and stamp_sec - self.last_large_reanchor_time_sec <= self.post_reanchor_settle_s
        )
        if settle or not self.vio_valid:
            out.twist.twist.linear.x = 0.0
            out.twist.twist.linear.y = 0.0
            out.twist.twist.linear.z = 0.0
            out.twist.twist.angular.x = 0.0
            out.twist.twist.angular.y = 0.0
            out.twist.twist.angular.z = 0.0
            out.twist.covariance = covariance_from_diag(self.high_twist_cov_diag)
            return

        v_imu = np.array(
            [
                source.twist.twist.linear.x,
                source.twist.twist.linear.y,
                source.twist.twist.linear.z,
            ],
            dtype=float,
        )
        w_imu = np.array(
            [
                source.twist.twist.angular.x,
                source.twist.twist.angular.y,
                source.twist.twist.angular.z,
            ],
            dtype=float,
        )

        # OpenVINS publishes velocity in the local IMU frame. For the corrected
        # odom topic we express twist in marker_map so downstream point-cloud
        # consumers see the same world convention as the corrected pose.
        R_map_global = self.T_map_global[:3, :3] if self.T_map_global is not None else np.eye(3)
        R_global_imu = T_global_imu[:3, :3]
        v_map = R_map_global @ R_global_imu @ v_imu
        w_map = R_map_global @ R_global_imu @ w_imu

        out.twist.twist.linear.x = float(v_map[0])
        out.twist.twist.linear.y = float(v_map[1])
        out.twist.twist.linear.z = float(v_map[2])
        out.twist.twist.angular.x = float(w_map[0])
        out.twist.twist.angular.y = float(w_map[1])
        out.twist.twist.angular.z = float(w_map[2])

        twist_diag = twist_covariance_diag(source, self.high_twist_cov_diag)
        out.twist.covariance = covariance_from_diag(np.clip(twist_diag, 1e-8, self.high_twist_cov_diag))

    def request_reanchor_cb(self, _request, response):
        if self.last_marker_measurement is None:
            response.success = False
            response.message = "No valid marker measurement has been received yet"
            return response

        accepted, reason = self.try_apply_marker_correction(self.last_marker_measurement, force_manual=True)
        response.success = bool(accepted)
        response.message = reason
        return response

    def publish_reanchor_event_from_measurement(
        self,
        measurement: MarkerMeasurement,
        correction_accepted: bool,
        correction_mode: str,
        reason: str,
        odom_match_dt: Optional[float] = None,
        correction_translation_norm_m: Optional[float] = None,
        correction_rotation_deg: Optional[float] = None,
        chi2: Optional[float] = None,
    ) -> None:
        self.publish_reanchor_event(
            stamp_sec=measurement.stamp_sec,
            marker_id=measurement.marker_id,
            marker_valid=True,
            correction_accepted=correction_accepted,
            correction_mode=correction_mode,
            reason=reason,
            correction_translation_norm_m=correction_translation_norm_m,
            correction_rotation_deg=correction_rotation_deg,
            marker_reprojection_error_px=measurement.reprojection_error_px,
            marker_distance_m=measurement.distance_m,
            odom_match_dt=odom_match_dt,
            chi2=chi2,
            marker_quality=self.marker_quality_payload(measurement, hard_gate_status="accepted"),
        )

    def publish_reanchor_event(
        self,
        stamp_sec: float,
        marker_id: int,
        marker_valid: bool,
        correction_accepted: bool,
        correction_mode: str,
        reason: str,
        correction_translation_norm_m: Optional[float] = None,
        correction_rotation_deg: Optional[float] = None,
        marker_reprojection_error_px: Optional[float] = None,
        marker_distance_m: Optional[float] = None,
        odom_match_dt: Optional[float] = None,
        chi2: Optional[float] = None,
        marker_quality: Optional[dict[str, Any]] = None,
    ) -> None:
        event_key = (correction_mode, reason, marker_id, correction_accepted)
        wall_now = self.now_sec()
        if not correction_accepted and self.last_event_key == event_key and wall_now - self.last_event_wall_time_sec < 1.0:
            return
        self.last_event_key = event_key
        self.last_event_wall_time_sec = wall_now

        event = {
            "event_type": "marker_correction",
            "stamp": round(float(stamp_sec), 9),
            "marker_id": int(marker_id),
            "marker_valid": bool(marker_valid),
            "correction_accepted": bool(correction_accepted),
            "accepted": bool(correction_accepted),
            "correction_mode": correction_mode,
            "reason": reason,
            "vio_valid": bool(self.vio_valid),
            "correction_translation_norm_m": None
            if correction_translation_norm_m is None
            else round(float(correction_translation_norm_m), 6),
            "correction_rotation_deg": None if correction_rotation_deg is None else round(float(correction_rotation_deg), 6),
            "marker_reprojection_error_px": None
            if marker_reprojection_error_px is None
            else round(float(marker_reprojection_error_px), 4),
            "marker_distance_m": None if marker_distance_m is None else round(float(marker_distance_m), 4),
            "odom_match_dt": None if odom_match_dt is None else round(float(odom_match_dt), 6),
            "chi2": None if chi2 is None else round(float(chi2), 6),
        }
        if marker_quality is not None:
            event.update(marker_quality)
        msg = String()
        msg.data = json.dumps(event, separators=(",", ":"))
        self.reanchor_event_pub.publish(msg)

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

    def qos_watchdog_cb(self) -> None:
        if self.last_odom_wall_time_sec is None:
            self.get_logger().warning(
                f"No OpenVINS odom received yet on {self.odom_topic}. Check topic name and QoS compatibility."
            )
            return
        age = self.now_sec() - self.last_odom_wall_time_sec
        if age > 2.0:
            self.get_logger().warning(
                f"OpenVINS odom on {self.odom_topic} is stale by {age:.2f} s. Correction updates are disabled until odom returns."
            )


def main():
    rclpy.init()
    node = ArucoMarkerPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
