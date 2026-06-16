#!/usr/bin/env python3

"""Jetson -> Laptop sensor-data relay with per-stream throttling, compression,
and decimation.  Subscribes to raw topics from the dual-D435i + dual-OpenVINS
pipeline and republishes under ``/jetson/`` at lower, laptop-friendly rates.

Supported streams (each independently togglable via ROS2 parameters):
  - PointCloud2     — stride decimation (keep every Nth point) + throttle
  - Image (colour)  — JPEG compression (cv2.imencode) + throttle
  - CameraInfo      — throttle only
  - Odometry        — passthrough (no throttling — they are tiny)
  - marker_map_locked — latched passthrough (transient_local QoS)

IMU is deliberately NOT relayed — no laptop node subscribes to it, and the
laptop reconstructs its TF tree from the odometry messages alone.

Usage (inside the Docker container)::

    python3 jetson_relay.py
    python3 jetson_relay.py --ros-args \\
        -p pointcloud.decimation.enabled:=false \\
        -p image.compression.quality:=90
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

try:
    import cv2
    _HAS_CV2 = True
except ImportError:  # pragma: no cover
    _HAS_CV2 = False


# ═══════════════════════════════════════════════════════════════════════════
# QoS profiles
# ═══════════════════════════════════════════════════════════════════════════

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_ODOM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

_CAMINFO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

_LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_HEALTH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# ═══════════════════════════════════════════════════════════════════════════
# Source -> destination topic map  (head + arm camera pair)
# ═══════════════════════════════════════════════════════════════════════════

_SRC = {
    "head_pc":   "/head/d435i_head/depth/color/points",
    "arm_pc":    "/arm/d435i_arm/depth/color/points",
    "head_img":  "/head/d435i_head/color/image_raw",
    "arm_img":   "/arm/d435i_arm/color/image_raw",
    "head_ci":   "/head/d435i_head/color/camera_info",
    "arm_ci":    "/arm/d435i_arm/color/camera_info",
    "head_odom": "/ov_msckf/odomimu",
    "arm_odom":  "/ov_msckf_arm/odomimu",
    "head_mml":  "/ov_msckf/marker_map_locked",
    "arm_mml":   "/ov_msckf_arm/marker_map_locked",
}

_DST = {
    "head_pc":   "/jetson/head/points",
    "arm_pc":    "/jetson/arm/points",
    "head_img":  "/jetson/head/image/compressed",
    "arm_img":   "/jetson/arm/image/compressed",
    "head_ci":   "/jetson/head/camera_info",
    "arm_ci":    "/jetson/arm/camera_info",
    "head_odom": "/jetson/head/odom",
    "arm_odom":  "/jetson/arm/odom",
    "head_mml":  "/jetson/head/marker_map_locked",
    "arm_mml":   "/jetson/arm/marker_map_locked",
}

CAMERAS = ("head", "arm")


# ═══════════════════════════════════════════════════════════════════════════
# RateGate — wall-clock throttle
# ═══════════════════════════════════════════════════════════════════════════

class RateGate:
    """Simple time-based rate limiter.

    Unlike stamp-based gating this is immune to clock skew between the
    Jetson and laptop, and to messages that arrive with zero timestamps.
    """

    def __init__(self, hz: float) -> None:
        self._interval: float = 1.0 / max(float(hz), 0.001)
        self._last: float = 0.0

    def should_publish(self) -> bool:
        now = time.time()
        if now - self._last >= self._interval:
            self._last = now
            return True
        return False


# ═══════════════════════════════════════════════════════════════════════════
# PointCloud2 stride-decimation helper
# ═══════════════════════════════════════════════════════════════════════════

def decimate_pointcloud(msg: PointCloud2, step: int) -> PointCloud2:
    """Return a new PointCloud2 with every *step*-th point kept.

    Handles both organised clouds (height > 1) and unorganised ones.
    Output is always ``height=1`` (flattened).
    """
    if step <= 1:
        return msg

    point_step = msg.point_step
    n_points = msg.width * msg.height
    if n_points <= step:
        return msg

    # Handle organised clouds that may have per-row padding bytes.
    if msg.height > 1 and msg.row_step != point_step * msg.width:
        rows = []
        for r in range(msg.height):
            row_start = r * msg.row_step
            row_data = msg.data[row_start : row_start + point_step * msg.width]
            rows.append(
                np.frombuffer(row_data, dtype=np.uint8).reshape(msg.width, point_step)
            )
        arr = np.concatenate(rows, axis=0)
    else:
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, point_step)

    decimated = arr[::step, :]

    out = PointCloud2()
    out.header = msg.header
    out.height = 1
    out.width = decimated.shape[0]
    out.fields = msg.fields
    out.point_step = point_step
    out.row_step = point_step * out.width
    out.data = decimated.tobytes()
    out.is_bigendian = msg.is_bigendian
    out.is_dense = msg.is_dense
    return out


# ═══════════════════════════════════════════════════════════════════════════
# JetsonRelay node
# ═══════════════════════════════════════════════════════════════════════════

class JetsonRelay(Node):
    """Relay node: raw -> /jetson/ with compression + throttling."""

    def __init__(self) -> None:
        super().__init__("jetson_relay")

        # ── declare parameters ──────────────────────────────────────────
        self.declare_parameter("pointcloud.enabled", True)
        self.declare_parameter("pointcloud.hz", 5.0)
        self.declare_parameter("pointcloud.decimation.enabled", True)
        self.declare_parameter("pointcloud.decimation.step", 4)

        self.declare_parameter("image.enabled", True)
        self.declare_parameter("image.hz", 5.0)
        self.declare_parameter("image.compression.enabled", True)
        self.declare_parameter("image.compression.quality", 80)

        self.declare_parameter("camera_info.enabled", True)
        self.declare_parameter("camera_info.hz", 1.0)

        self.declare_parameter("odometry.enabled", True)
        self.declare_parameter("marker_map_locked.enabled", True)

        # ── read parameters ─────────────────────────────────────────────
        self._pc_enabled = self.get_parameter("pointcloud.enabled").value
        self._pc_hz = self.get_parameter("pointcloud.hz").value
        self._pc_dec_enabled = self.get_parameter("pointcloud.decimation.enabled").value
        self._pc_dec_step = self.get_parameter("pointcloud.decimation.step").value

        self._img_enabled = self.get_parameter("image.enabled").value
        self._img_hz = self.get_parameter("image.hz").value
        self._img_comp_enabled = self.get_parameter("image.compression.enabled").value
        self._img_comp_quality = self.get_parameter("image.compression.quality").value

        self._ci_enabled = self.get_parameter("camera_info.enabled").value
        self._ci_hz = self.get_parameter("camera_info.hz").value

        self._odom_enabled = self.get_parameter("odometry.enabled").value
        self._mml_enabled = self.get_parameter("marker_map_locked.enabled").value

        # ── rate gates (one per stream per camera) ──────────────────────
        self._gates: dict[str, RateGate] = {}
        for cam in CAMERAS:
            self._gates[f"pc_{cam}"] = RateGate(self._pc_hz)
            self._gates[f"img_{cam}"] = RateGate(self._img_hz)
            self._gates[f"ci_{cam}"] = RateGate(self._ci_hz)

        # ── cv_bridge ───────────────────────────────────────────────────
        self._bridge = CvBridge() if _HAS_CV2 else None
        if self._img_comp_enabled and not _HAS_CV2:
            self.get_logger().warn(
                "Image compression enabled but OpenCV not available. "
                "Images will be relayed raw."
            )

        # ── setup pubs/subs ─────────────────────────────────────────────
        self._setup_pointclouds()
        self._setup_images()
        self._setup_camera_info()
        self._setup_odometry()
        self._setup_marker_map_locked()

        # ── health ──────────────────────────────────────────────────────
        self._health_pub = self.create_publisher(
            Bool, "/jetson/relay/health", _HEALTH_QOS
        )
        self._health_timer = self.create_timer(1.0, self._publish_health)

        self._log_config()

    # ── setup helpers ────────────────────────────────────────────────────

    def _setup_pointclouds(self) -> None:
        if not self._pc_enabled:
            self.get_logger().info("Pointcloud relay:  DISABLED")
            return
        for cam in CAMERAS:
            key = f"{cam}_pc"
            self.create_subscription(
                PointCloud2, _SRC[key],
                lambda m, c=cam: self._on_pc(m, c), _SENSOR_QOS,
            )
        self._pc_pub = {
            cam: self.create_publisher(PointCloud2, _DST[f"{cam}_pc"], _SENSOR_QOS)
            for cam in CAMERAS
        }

    def _setup_images(self) -> None:
        if not self._img_enabled:
            self.get_logger().info("Image relay:       DISABLED")
            return
        for cam in CAMERAS:
            key = f"{cam}_img"
            self.create_subscription(
                Image, _SRC[key],
                lambda m, c=cam: self._on_img(m, c), _SENSOR_QOS,
            )
        self._img_pub = {
            cam: self.create_publisher(Image, _DST[f"{cam}_img"], _SENSOR_QOS)
            for cam in CAMERAS
        }

    def _setup_camera_info(self) -> None:
        if not self._ci_enabled:
            self.get_logger().info("CameraInfo relay:  DISABLED")
            return
        for cam in CAMERAS:
            key = f"{cam}_ci"
            self.create_subscription(
                CameraInfo, _SRC[key],
                lambda m, c=cam: self._on_ci(m, c), _CAMINFO_QOS,
            )
        self._ci_pub = {
            cam: self.create_publisher(CameraInfo, _DST[f"{cam}_ci"], _CAMINFO_QOS)
            for cam in CAMERAS
        }

    def _setup_odometry(self) -> None:
        if not self._odom_enabled:
            self.get_logger().info("Odometry relay:    DISABLED")
            return
        for cam in CAMERAS:
            key = f"{cam}_odom"
            self.create_subscription(
                Odometry, _SRC[key],
                lambda m, c=cam: self._on_odom(m, c), _ODOM_QOS,
            )
        self._odom_pub = {
            cam: self.create_publisher(Odometry, _DST[f"{cam}_odom"], _ODOM_QOS)
            for cam in CAMERAS
        }

    def _setup_marker_map_locked(self) -> None:
        if not self._mml_enabled:
            self.get_logger().info("marker_map_locked: DISABLED")
            return
        for cam in CAMERAS:
            key = f"{cam}_mml"
            self.create_subscription(
                Bool, _SRC[key],
                lambda m, c=cam: self._on_mml(m, c), _LATCHED_QOS,
            )
        self._mml_pub = {
            cam: self.create_publisher(Bool, _DST[f"{cam}_mml"], _LATCHED_QOS)
            for cam in CAMERAS
        }

    # ── callbacks ─────────────────────────────────────────────────────────

    def _on_pc(self, msg: PointCloud2, camera: str) -> None:
        if not self._gates[f"pc_{camera}"].should_publish():
            return
        if self._pc_dec_enabled and self._pc_dec_step > 1:
            msg = decimate_pointcloud(msg, self._pc_dec_step)
        self._pc_pub[camera].publish(msg)

    def _on_img(self, msg: Image, camera: str) -> None:
        if not self._gates[f"img_{camera}"].should_publish():
            return
        out = msg
        if self._img_comp_enabled and self._bridge is not None:
            try:
                out = self._compress_image(msg, self._img_comp_quality)
            except Exception as exc:
                self.get_logger().warn(
                    f"Image compression failed ({camera}): {exc}"
                )
                return
        self._img_pub[camera].publish(out)

    def _on_ci(self, msg: CameraInfo, camera: str) -> None:
        if not self._gates[f"ci_{camera}"].should_publish():
            return
        self._ci_pub[camera].publish(msg)

    def _on_odom(self, msg: Odometry, camera: str) -> None:
        self._odom_pub[camera].publish(msg)

    def _on_mml(self, msg: Bool, camera: str) -> None:
        self._mml_pub[camera].publish(msg)

    # ── image compression ────────────────────────────────────────────────

    def _compress_image(self, msg: Image, quality: int) -> Image:
        cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        _, jpeg_bytes = cv2.imencode(
            ".jpg", cv_img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)]
        )
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = "jpeg"
        out.step = len(jpeg_bytes)
        out.data = jpeg_bytes.tobytes()
        return out

    # ── health ───────────────────────────────────────────────────────────

    def _publish_health(self) -> None:
        self._health_pub.publish(Bool(data=True))

    # ── startup log ──────────────────────────────────────────────────────

    def _log_config(self) -> None:
        info = self.get_logger().info
        info("── JetsonRelay configuration ────────────────────────────")
        info(
            "  pointcloud:  enabled=%s  hz=%.1f  dec=%s (step=%d)",
            self._pc_enabled, self._pc_hz,
            self._pc_dec_enabled, self._pc_dec_step,
        )
        info(
            "  image:       enabled=%s  hz=%.1f  jpeg=%s (q=%d)",
            self._img_enabled, self._img_hz,
            self._img_comp_enabled, self._img_comp_quality,
        )
        info(
            "  camera_info: enabled=%s  hz=%.1f",
            self._ci_enabled, self._ci_hz,
        )
        info(
            "  odometry:    enabled=%s  (passthrough at source rate)",
            self._odom_enabled,
        )
        info(
            "  marker_map_locked: enabled=%s  (latched)",
            self._mml_enabled,
        )
        info("  health:      /jetson/relay/health @ 1 Hz")
        info("──────────────────────────────────────────────────────")


# ═══════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None) -> None:
    rclpy.init(args=args)
    node = JetsonRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
