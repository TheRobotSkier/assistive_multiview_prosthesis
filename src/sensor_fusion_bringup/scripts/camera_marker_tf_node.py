#!/usr/bin/env python3

"""Publish camera poses from a known ArUco marker map.

Transform convention:
  T_A_B maps a point expressed in frame B into frame A.

OpenCV solvePnP returns T_camera_marker for each detected marker. The configured
marker map provides T_world_marker, so the camera pose is:

  T_world_camera = T_world_marker * inverse(T_camera_marker)

The node publishes world -> camera_link when the RealSense static transform from
color optical frame to link is available. That keeps the RealSense depth/color
TF tree intact, so RViz can transform point clouds into the world frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import partial
import math
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int32, String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def T_inv(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4, dtype=float)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def rvec_tvec_to_T(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
    return T


def quat_to_R_xyzw(q) -> np.ndarray:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
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


def quat_array_to_R_xyzw(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
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


def R_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    tr = float(np.trace(R))
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
    norm = float(np.linalg.norm(q))
    if norm <= 0.0 or not math.isfinite(norm):
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


def slerp_xyzw(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = np.asarray(q0, dtype=float).reshape(4)
    q1 = np.asarray(q1, dtype=float).reshape(4)
    if float(np.dot(q0, q1)) < 0.0:
        q1 = -q1

    dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
    if dot > 0.9995:
        q = q0 + alpha * (q1 - q0)
        return q / np.linalg.norm(q)

    theta_0 = math.acos(dot)
    theta = theta_0 * alpha
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    q = s0 * q0 + s1 * q1
    return q / np.linalg.norm(q)


def tf_msg_to_T(msg: TransformStamped) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = quat_to_R_xyzw(msg.transform.rotation)
    t = msg.transform.translation
    T[:3, 3] = [float(t.x), float(t.y), float(t.z)]
    return T


def T_to_tf_msg(T: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(T[0, 3])
    msg.transform.translation.y = float(T[1, 3])
    msg.transform.translation.z = float(T[2, 3])
    qx, qy, qz, qw = R_to_quat_xyzw(T[:3, :3])
    msg.transform.rotation.x = float(qx)
    msg.transform.rotation.y = float(qy)
    msg.transform.rotation.z = float(qz)
    msg.transform.rotation.w = float(qw)
    return msg


def fill_pose_msg(msg: PoseStamped, T: np.ndarray) -> None:
    msg.pose.position.x = float(T[0, 3])
    msg.pose.position.y = float(T[1, 3])
    msg.pose.position.z = float(T[2, 3])
    qx, qy, qz, qw = R_to_quat_xyzw(T[:3, :3])
    msg.pose.orientation.x = float(qx)
    msg.pose.orientation.y = float(qy)
    msg.pose.orientation.z = float(qz)
    msg.pose.orientation.w = float(qw)


def marker_object_points(size_m: float) -> np.ndarray:
    h = float(size_m) / 2.0
    # OpenCV ArUco corner order: top-left, top-right, bottom-right, bottom-left.
    # Marker frame is centered on the print, x right and y down in the printed plane.
    return np.array(
        [
            [-h, -h, 0.0],
            [h, -h, 0.0],
            [h, h, 0.0],
            [-h, h, 0.0],
        ],
        dtype=np.float32,
    )


def contour_area(corners: np.ndarray) -> float:
    return abs(float(cv2.contourArea(np.asarray(corners, dtype=np.float32).reshape(4, 2))))


def reprojection_error_px(
    obj_points: np.ndarray,
    img_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(obj_points, rvec, tvec, K, D)
    projected = projected.reshape(-1, 2)
    observed = np.asarray(img_points, dtype=float).reshape(-1, 2)
    return float(np.mean(np.linalg.norm(projected - observed, axis=1)))


def average_transforms(observations: list["MarkerObservation"]) -> np.ndarray:
    weights = np.array([max(obs.weight, 1e-6) for obs in observations], dtype=float)
    weights = weights / float(np.sum(weights))

    T = np.eye(4, dtype=float)
    T[:3, 3] = np.sum([w * obs.T_world_camera[:3, 3] for w, obs in zip(weights, observations)], axis=0)

    accum = np.zeros((4, 4), dtype=float)
    for w, obs in zip(weights, observations):
        q = R_to_quat_xyzw(obs.T_world_camera[:3, :3])
        accum += w * np.outer(q, q)
    eigvals, eigvecs = np.linalg.eigh(accum)
    q_avg = eigvecs[:, int(np.argmax(eigvals))]
    if q_avg[3] < 0.0:
        q_avg = -q_avg
    T[:3, :3] = quat_array_to_R_xyzw(q_avg)
    return T


def smooth_transform(previous: Optional[np.ndarray], current: np.ndarray, alpha: float) -> np.ndarray:
    if previous is None or alpha >= 1.0:
        return current
    if alpha <= 0.0:
        return previous

    out = np.eye(4, dtype=float)
    out[:3, 3] = (1.0 - alpha) * previous[:3, 3] + alpha * current[:3, 3]
    q_prev = R_to_quat_xyzw(previous[:3, :3])
    q_cur = R_to_quat_xyzw(current[:3, :3])
    q_out = slerp_xyzw(q_prev, q_cur, alpha)
    out[:3, :3] = quat_array_to_R_xyzw(q_out)
    return out


def get_aruco_dictionary(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is not available. Install OpenCV with contrib/aruco support.")
    if not hasattr(cv2.aruco, name):
        raise RuntimeError(f"Unknown OpenCV ArUco dictionary: {name}")
    dict_id = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


def image_msg_to_gray(msg: Image) -> Optional[np.ndarray]:
    enc = msg.encoding.lower()
    h = int(msg.height)
    w = int(msg.width)
    step = int(msg.step)
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
        return np.ascontiguousarray(rows[:, :w])

    return None


@dataclass
class MarkerConfig:
    marker_id: int
    frame_id: str
    size_m: float
    T_world_marker: np.ndarray


@dataclass
class MarkerObservation:
    marker_id: int
    T_world_camera: np.ndarray
    T_world_marker: np.ndarray
    area_px2: float
    distance_m: float
    reprojection_error_px: float
    weight: float


class CameraState:
    def __init__(self, name: str, cfg: dict[str, Any], default_rate_hz: float):
        self.name = name
        self.image_topic = str(cfg["image_topic"])
        self.camera_info_topic = str(cfg["camera_info_topic"])
        self.detected_camera_frame = str(cfg.get("detected_camera_frame", ""))
        self.link_frame = str(cfg.get("link_frame", ""))
        self.pose_topic = str(cfg.get("pose_topic", f"/{name}/camera_marker_tf/pose"))
        self.valid_topic = str(cfg.get("valid_topic", f"/{name}/camera_marker_tf/valid"))
        self.active_marker_topic = str(cfg.get("active_marker_topic", f"/{name}/camera_marker_tf/active_marker_id"))
        self.quality_topic = str(cfg.get("quality_topic", f"/{name}/camera_marker_tf/quality"))
        self.detection_rate_hz = float(cfg.get("detection_rate_hz", default_rate_hz))

        self.K: Optional[np.ndarray] = None
        self.D: Optional[np.ndarray] = None
        self.last_processed_stamp_sec: Optional[float] = None
        self.last_T_world_camera: Optional[np.ndarray] = None
        self.cached_T_camera_link: Optional[np.ndarray] = None
        self.tf_warned = False
        self.info_warned = False
        self.image_encoding_warned = False
        self.frame_mismatch_warned = False

        self.pose_pub = None
        self.valid_pub = None
        self.active_marker_pub = None
        self.quality_pub = None


class CameraMarkerTfNode(Node):
    def __init__(self):
        super().__init__("camera_marker_tf_node")

        self.declare_parameter("config_file", "")
        self.declare_parameter("marker_detection_rate_hz", 15.0)
        self.declare_parameter("smoothing_alpha", -1.0)
        self.declare_parameter("publish_marker_frames", True)
        self.declare_parameter("publish_debug_camera_frames", True)
        self.declare_parameter("publish_pose_topics", True)
        self.declare_parameter("publish_link_tf", True)
        self.declare_parameter("publish_optical_tf_if_no_link", False)
        self.declare_parameter("tf_lookup_timeout_sec", 0.2)

        config_file = str(self.get_parameter("config_file").value)
        if not config_file:
            raise RuntimeError("Parameter 'config_file' is required")
        self.config_path = Path(config_file)
        self.config = self._load_config(self.config_path)

        aruco_cfg = self.config.get("aruco", {})
        self.dictionary_name = str(aruco_cfg.get("dictionary", "DICT_6X6_1000"))
        self.default_marker_size_m = float(aruco_cfg.get("default_marker_size_m", 0.1))
        self.aruco_dict = get_aruco_dictionary(self.dictionary_name)

        frames_cfg = self.config.get("frames", {})
        self.world_frame = str(frames_cfg.get("world_frame", "world"))

        quality_cfg = self.config.get("quality", {})
        self.min_marker_area_px2 = float(quality_cfg.get("min_marker_area_px2", 500.0))
        self.max_marker_distance_m = float(quality_cfg.get("max_marker_distance_m", 2.5))
        self.max_reprojection_error_px = float(quality_cfg.get("max_reprojection_error_px", 4.0))

        filter_cfg = self.config.get("filter", {})
        cfg_smoothing_alpha = float(filter_cfg.get("smoothing_alpha", 0.45))
        param_smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)
        self.smoothing_alpha = cfg_smoothing_alpha if param_smoothing_alpha < 0.0 else param_smoothing_alpha

        self.publish_marker_frames = bool(self.get_parameter("publish_marker_frames").value)
        self.publish_debug_camera_frames = bool(self.get_parameter("publish_debug_camera_frames").value)
        self.publish_pose_topics = bool(self.get_parameter("publish_pose_topics").value)
        self.publish_link_tf = bool(self.get_parameter("publish_link_tf").value)
        self.publish_optical_tf_if_no_link = bool(self.get_parameter("publish_optical_tf_if_no_link").value)
        self.tf_lookup_timeout_sec = float(self.get_parameter("tf_lookup_timeout_sec").value)

        self.markers = self._load_markers(self.config)
        if not self.markers:
            raise RuntimeError("Marker TF config has no known markers")

        default_rate_hz = float(self.get_parameter("marker_detection_rate_hz").value)
        self.cameras = {
            name: CameraState(name, cfg, default_rate_hz)
            for name, cfg in self.config.get("cameras", {}).items()
        }
        if not self.cameras:
            raise RuntimeError("Marker TF config has no cameras")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for name, camera in self.cameras.items():
            self.create_subscription(
                CameraInfo,
                camera.camera_info_topic,
                partial(self.camera_info_cb, name),
                sensor_qos,
            )
            self.create_subscription(
                Image,
                camera.image_topic,
                partial(self.image_cb, name),
                sensor_qos,
            )
            if self.publish_pose_topics:
                camera.pose_pub = self.create_publisher(PoseStamped, camera.pose_topic, 10)
                camera.valid_pub = self.create_publisher(Bool, camera.valid_topic, 10)
                camera.active_marker_pub = self.create_publisher(Int32, camera.active_marker_topic, 10)
                camera.quality_pub = self.create_publisher(String, camera.quality_topic, 10)

        self.get_logger().info(f"Loaded marker TF config: {self.config_path}")
        self.get_logger().info(f"World frame: {self.world_frame}")
        self.get_logger().info(f"Dictionary: {self.dictionary_name}")
        self.get_logger().info(f"Known markers: {sorted(self.markers.keys())}")
        self.get_logger().info(f"Cameras: {sorted(self.cameras.keys())}")

    def _load_config(self, path: Path) -> dict[str, Any]:
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            raise RuntimeError(f"Invalid marker TF config: {path}")
        return data

    def _load_markers(self, config: dict[str, Any]) -> dict[int, MarkerConfig]:
        markers: dict[int, MarkerConfig] = {}
        for marker_id_raw, marker_cfg in config.get("markers", {}).items():
            marker_id = int(marker_id_raw)
            size_m = float(marker_cfg.get("size_m", self.default_marker_size_m))
            frame_id = str(marker_cfg.get("frame_id", f"marker_{marker_id}"))
            raw_T = marker_cfg.get("T_world_marker", marker_cfg.get("T_map_marker"))
            if raw_T is None:
                raise RuntimeError(f"Marker {marker_id} is missing T_world_marker")
            T_world_marker = np.asarray(raw_T, dtype=float).reshape(4, 4)
            markers[marker_id] = MarkerConfig(
                marker_id=marker_id,
                frame_id=frame_id,
                size_m=size_m,
                T_world_marker=T_world_marker,
            )
        return markers

    def camera_info_cb(self, name: str, msg: CameraInfo) -> None:
        camera = self.cameras[name]
        K = np.asarray(msg.k, dtype=float).reshape(3, 3)
        if not np.all(np.isfinite(K)) or abs(float(K[0, 0])) < 1e-9 or abs(float(K[1, 1])) < 1e-9:
            if not camera.info_warned:
                self.get_logger().warning(f"{name}: ignoring invalid CameraInfo K matrix")
                camera.info_warned = True
            return

        camera.K = K
        camera.D = np.asarray(msg.d, dtype=float).reshape(-1, 1)
        if msg.header.frame_id:
            if (
                camera.detected_camera_frame
                and camera.detected_camera_frame != msg.header.frame_id
                and not camera.frame_mismatch_warned
            ):
                self.get_logger().warning(
                    f"{name}: config camera frame '{camera.detected_camera_frame}' differs from "
                    f"CameraInfo frame '{msg.header.frame_id}', using CameraInfo frame"
                )
                camera.frame_mismatch_warned = True
            camera.detected_camera_frame = msg.header.frame_id

    def image_cb(self, name: str, msg: Image) -> None:
        camera = self.cameras[name]
        stamp_sec = stamp_to_sec(msg.header.stamp)
        if not self.should_process(camera, stamp_sec):
            return
        camera.last_processed_stamp_sec = stamp_sec

        if camera.K is None or camera.D is None:
            self.publish_state(camera, False, -1, "waiting_for_camera_info")
            return

        if not camera.detected_camera_frame:
            camera.detected_camera_frame = msg.header.frame_id
        if not camera.detected_camera_frame:
            self.publish_state(camera, False, -1, "missing_camera_frame")
            return

        gray = image_msg_to_gray(msg)
        if gray is None:
            if not camera.image_encoding_warned:
                self.get_logger().warning(f"{name}: unsupported image encoding '{msg.encoding}'")
                camera.image_encoding_warned = True
            self.publish_state(camera, False, -1, "unsupported_image_encoding")
            return

        try:
            corners, ids, _rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict)
        except Exception as exc:
            self.get_logger().warning(f"{name}: ArUco detection failed: {exc}")
            self.publish_state(camera, False, -1, "detection_exception")
            return

        if ids is None or len(ids) == 0:
            self.publish_state(camera, False, -1, "no_markers")
            return

        observations: list[MarkerObservation] = []
        reject_reasons: list[str] = []
        for idx, marker_id_arr in enumerate(ids):
            marker_id = int(marker_id_arr[0])
            marker = self.markers.get(marker_id)
            if marker is None:
                reject_reasons.append(f"unknown_marker_{marker_id}")
                continue
            observation, reason = self.build_observation(camera, marker, corners[idx].reshape(4, 2))
            if observation is None:
                reject_reasons.append(f"{marker_id}:{reason}")
                continue
            observations.append(observation)

        if not observations:
            reason = ";".join(reject_reasons) if reject_reasons else "no_valid_markers"
            self.publish_state(camera, False, -1, reason)
            return

        T_world_camera_raw = average_transforms(observations)
        T_world_camera = smooth_transform(camera.last_T_world_camera, T_world_camera_raw, self.smoothing_alpha)
        camera.last_T_world_camera = T_world_camera

        best = max(observations, key=lambda obs: obs.weight)
        self.publish_camera_outputs(camera, msg, observations, best, T_world_camera)

    def should_process(self, camera: CameraState, stamp_sec: float) -> bool:
        rate_hz = camera.detection_rate_hz
        if rate_hz <= 0.0 or camera.last_processed_stamp_sec is None:
            return True
        if stamp_sec <= camera.last_processed_stamp_sec:
            return True
        return stamp_sec - camera.last_processed_stamp_sec + 1e-9 >= 1.0 / rate_hz

    def build_observation(
        self,
        camera: CameraState,
        marker: MarkerConfig,
        image_points: np.ndarray,
    ) -> tuple[Optional[MarkerObservation], str]:
        image_points = np.asarray(image_points, dtype=np.float32).reshape(4, 2)
        if not np.all(np.isfinite(image_points)):
            return None, "non_finite_corners"

        area = contour_area(image_points)
        if area < self.min_marker_area_px2:
            return None, "area_too_small"

        obj_points = marker_object_points(marker.size_m)
        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            image_points,
            camera.K,
            camera.D,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None, "solvepnp_failed"

        T_camera_marker = rvec_tvec_to_T(rvec, tvec)
        if not np.all(np.isfinite(T_camera_marker)):
            return None, "non_finite_pose"

        distance_m = float(np.linalg.norm(T_camera_marker[:3, 3]))
        if distance_m > self.max_marker_distance_m:
            return None, "too_far"

        error_px = reprojection_error_px(obj_points, image_points, rvec, tvec, camera.K, camera.D)
        if error_px > self.max_reprojection_error_px:
            return None, "reprojection_error_too_high"

        T_world_camera = marker.T_world_marker @ T_inv(T_camera_marker)
        weight = area / max(distance_m * distance_m, 0.05 * 0.05) / max(1.0, error_px * error_px)

        return (
            MarkerObservation(
                marker_id=marker.marker_id,
                T_world_camera=T_world_camera,
                T_world_marker=marker.T_world_marker,
                area_px2=area,
                distance_m=distance_m,
                reprojection_error_px=error_px,
                weight=float(weight),
            ),
            "",
        )

    def publish_camera_outputs(
        self,
        camera: CameraState,
        msg: Image,
        observations: list[MarkerObservation],
        best: MarkerObservation,
        T_world_camera: np.ndarray,
    ) -> None:
        stamp = msg.header.stamp
        transforms: list[TransformStamped] = []

        if self.publish_marker_frames:
            for obs in observations:
                marker_frame = self.markers[obs.marker_id].frame_id
                transforms.append(T_to_tf_msg(obs.T_world_marker, self.world_frame, marker_frame, stamp))

        if self.publish_debug_camera_frames:
            debug_frame = f"{camera.detected_camera_frame}_from_markers"
            transforms.append(T_to_tf_msg(T_world_camera, self.world_frame, debug_frame, stamp))

        published_link_tf = False
        if self.publish_link_tf and camera.link_frame:
            T_camera_link = self.get_camera_to_link(camera)
            if T_camera_link is not None:
                transforms.append(T_to_tf_msg(T_world_camera @ T_camera_link, self.world_frame, camera.link_frame, stamp))
                published_link_tf = True

        if not published_link_tf and self.publish_optical_tf_if_no_link:
            transforms.append(T_to_tf_msg(T_world_camera, self.world_frame, camera.detected_camera_frame, stamp))

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

        if self.publish_pose_topics and camera.pose_pub is not None:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.world_frame
            fill_pose_msg(pose, T_world_camera)
            camera.pose_pub.publish(pose)
            self.publish_state(
                camera,
                True,
                best.marker_id,
                (
                    f"markers={len(observations)} best={best.marker_id} "
                    f"distance_m={best.distance_m:.3f} area_px2={best.area_px2:.1f} "
                    f"reprojection_px={best.reprojection_error_px:.2f}"
                ),
            )

    def get_camera_to_link(self, camera: CameraState) -> Optional[np.ndarray]:
        if camera.cached_T_camera_link is not None:
            return camera.cached_T_camera_link
        if not camera.detected_camera_frame or not camera.link_frame:
            return None
        try:
            msg = self.tf_buffer.lookup_transform(
                camera.detected_camera_frame,
                camera.link_frame,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
            camera.cached_T_camera_link = tf_msg_to_T(msg)
            return camera.cached_T_camera_link
        except Exception as exc:
            if not camera.tf_warned:
                self.get_logger().warning(
                    f"{camera.name}: cannot look up {camera.detected_camera_frame} <- {camera.link_frame}; "
                    f"world->{camera.link_frame} TF will wait. Error: {exc}"
                )
                camera.tf_warned = True
            return None

    def publish_state(self, camera: CameraState, valid: bool, active_marker_id: int, quality: str) -> None:
        if not self.publish_pose_topics or camera.valid_pub is None:
            return
        valid_msg = Bool()
        valid_msg.data = bool(valid)
        camera.valid_pub.publish(valid_msg)

        marker_msg = Int32()
        marker_msg.data = int(active_marker_id)
        camera.active_marker_pub.publish(marker_msg)

        quality_msg = String()
        quality_msg.data = quality
        camera.quality_pub.publish(quality_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraMarkerTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
