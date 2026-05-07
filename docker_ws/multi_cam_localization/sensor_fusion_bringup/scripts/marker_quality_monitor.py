#!/usr/bin/env python3

"""Monitor all visible ArUco markers for covariance data collection.

This node is intentionally separate from the Phase 1 correction node. The
correction node treats configured markers as fixed map landmarks and selects one
active marker. This monitor detects every visible marker independently from the
image stream, publishes per-marker JSON, and prints compact live rows for
stationary covariance-estimation bags.
"""

from __future__ import annotations

import json
import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from aruco_marker_pose_node import (
    is_finite_array,
    marker_object_points,
    marker_quality_metrics,
    stamp_to_sec,
)


def parse_marker_id_filter(value: str) -> Optional[set[int]]:
    text = str(value).strip()
    if not text or text.lower() in {"all", "*"}:
        return None
    marker_ids = set()
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        marker_ids.add(int(part))
    return marker_ids


def image_msg_to_gray(msg: Image) -> Optional[np.ndarray]:
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


def marker_summary_line(marker: dict) -> str:
    return (
        f"id={marker['marker_id']} "
        f"dist={marker['distance_m']:.3f}m "
        f"angle={marker['view_angle_deg']:.1f}deg "
        f"side={marker['side_mean_px']:.1f}px "
        f"reproj={marker['reprojection_error_px']:.3f}px"
    )


class MarkerQualityMonitor(Node):
    def __init__(self):
        super().__init__("marker_quality_monitor")

        self.declare_parameter("image_topic", "/head/d435i_head/color/image_raw")
        self.declare_parameter("camera_info_topic", "/head/d435i_head/color/camera_info")
        self.declare_parameter("output_topic", "/head/marker_pose/all_marker_quality")
        self.declare_parameter("dictionary", "DICT_6X6_1000")
        self.declare_parameter("marker_size_m", 0.100)
        self.declare_parameter("marker_ids", "0,1,2")
        self.declare_parameter("print_hz", 2.0)
        self.declare_parameter("publish_empty", False)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.dictionary_name = str(self.get_parameter("dictionary").value)
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)
        self.marker_id_filter = parse_marker_id_filter(str(self.get_parameter("marker_ids").value))
        self.print_period_s = 1.0 / max(1e-6, float(self.get_parameter("print_hz").value))
        self.publish_empty = bool(self.get_parameter("publish_empty").value)

        if not hasattr(cv2, "aruco"):
            raise RuntimeError("cv2.aruco is not available. Install OpenCV with aruco support.")

        dict_id = getattr(cv2.aruco, self.dictionary_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.obj_points = marker_object_points(self.marker_size_m)

        self.K: Optional[np.ndarray] = None
        self.D: Optional[np.ndarray] = None
        self.last_print_time_sec = 0.0
        self.last_camera_info_warn_sec = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, sensor_qos)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_cb, sensor_qos)
        self.pub = self.create_publisher(String, self.output_topic, 10)

        ids_text = "all" if self.marker_id_filter is None else ",".join(str(i) for i in sorted(self.marker_id_filter))
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Camera info topic: {self.camera_info_topic}")
        self.get_logger().info(f"Output topic: {self.output_topic}")
        self.get_logger().info(f"Dictionary: {self.dictionary_name}")
        self.get_logger().info(f"Marker size: {self.marker_size_m:.4f} m")
        self.get_logger().info(f"Marker IDs: {ids_text}")

    def camera_info_cb(self, msg: CameraInfo) -> None:
        self.K = np.asarray(msg.k, dtype=float).reshape(3, 3)
        self.D = np.asarray(msg.d, dtype=float).reshape(-1, 1)

    def image_cb(self, msg: Image) -> None:
        if self.K is None or self.D is None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_camera_info_warn_sec > 2.0:
                self.get_logger().warning(f"Waiting for camera info on {self.camera_info_topic}")
                self.last_camera_info_warn_sec = now
            return

        gray = image_msg_to_gray(msg)
        if gray is None:
            self.get_logger().warning(f"Unsupported image encoding: {msg.encoding}")
            return

        corners, ids, _rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict)
        markers = []
        if ids is not None:
            for idx, marker_id_arr in enumerate(ids):
                marker_id = int(marker_id_arr[0])
                if self.marker_id_filter is not None and marker_id not in self.marker_id_filter:
                    continue

                marker = self.measure_marker(marker_id, corners[idx].reshape(4, 2).astype(np.float32))
                if marker is not None:
                    markers.append(marker)

        if markers or self.publish_empty:
            self.publish_measurement(msg, markers)

    def measure_marker(self, marker_id: int, image_points: np.ndarray) -> Optional[dict]:
        if not is_finite_array(image_points):
            return None

        ok, rvec, tvec = cv2.solvePnP(
            self.obj_points,
            image_points,
            self.K,
            self.D,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None

        R_cam_marker, _ = cv2.Rodrigues(rvec)
        T_cam_marker = np.eye(4)
        T_cam_marker[:3, :3] = R_cam_marker
        T_cam_marker[:3, 3] = tvec.reshape(3)
        if not is_finite_array(T_cam_marker):
            return None

        reprojection_error_px = compute_reprojection_error(
            self.obj_points,
            image_points,
            rvec,
            tvec,
            self.K,
            self.D,
        )
        metrics = marker_quality_metrics(image_points, T_cam_marker, reprojection_error_px)
        return {
            "marker_id": int(marker_id),
            "marker_size_m": round(float(self.marker_size_m), 6),
            "area_px2": round(float(metrics.area_px2), 3),
            "sqrt_area_px": round(float(metrics.sqrt_area_px), 3),
            "side_mean_px": round(float(metrics.side_mean_px), 3),
            "side_min_px": round(float(metrics.side_min_px), 3),
            "distance_m": round(float(metrics.distance_m), 6),
            "view_angle_deg": round(float(metrics.view_angle_deg), 6),
            "reprojection_error_px": round(float(metrics.reprojection_error_px), 6),
            "tvec_cam_marker_m": [round(float(v), 6) for v in tvec.reshape(3)],
            "rvec_cam_marker_rad": [round(float(v), 6) for v in rvec.reshape(3)],
        }

    def publish_measurement(self, msg: Image, markers: list[dict]) -> None:
        stamp_sec = stamp_to_sec(msg.header.stamp)
        event = {
            "event_type": "all_marker_quality",
            "stamp": round(float(stamp_sec), 9),
            "frame_id": msg.header.frame_id,
            "marker_count": len(markers),
            "markers": markers,
        }
        out = String()
        out.data = json.dumps(event, separators=(",", ":"))
        self.pub.publish(out)

        now = self.get_clock().now().nanoseconds * 1e-9
        if markers and now - self.last_print_time_sec >= self.print_period_s:
            self.get_logger().info(" | ".join(marker_summary_line(marker) for marker in markers))
            self.last_print_time_sec = now


def main():
    rclpy.init()
    node = MarkerQualityMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
