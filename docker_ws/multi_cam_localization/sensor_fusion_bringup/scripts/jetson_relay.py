#!/usr/bin/env python3

"""Jetson -> Laptop sensor-data relay with per-stream throttling,
pointcloud decimation, and trackhist downsampling.

Subscribes to raw topics from the dual-D435i + dual-OpenVINS pipeline and
republishes under ``/jetson/`` at lower, laptop-friendly rates.

Supported streams (each independently togglable via ROS2 parameters):
  - PointCloud2     — stride decimation (keep every Nth point) + throttle
  - Image (colour)  — spatial downscale + JPEG compression (default Q=75) + throttle
  - Depth image     — NEAREST spatial downscale (2×) + PNG compression (lossless)
                        + throttle (for host-side depth-to-cloud backprojection
                        via depth_image_proc)
  - ArUco poses     — passthrough (fixed marker observations + dynamic arm pose)
  - trackhist       — resolution downsample + throttle (visualization only)
  - CameraInfo      — throttle + K/P intrinsic scaling (to match 2× downsample)
  - Odometry        — throttled (default 50 Hz, to avoid NACK storms)
  - marker_map_locked — latched passthrough (transient_local QoS)

IMU is deliberately NOT relayed — no laptop node subscribes to it, and the
laptop reconstructs its TF tree from the odometry messages alone.

Usage (inside the Docker container)::

    python3 jetson_relay.py
    python3 jetson_relay.py --ros-args \\
        -p pointcloud.decimation.enabled:=false \\
        -p depth.enabled:=true \\
        -p aruco.enabled:=true
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

try:
    from sensor_fusion_msgs.msg import (
        DynamicArmPoseObservation,
        DynamicMarkerObservation,
        MarkerPoseObservation,
    )
    _HAS_ARUCO_MSGS = True
except ImportError:
    _HAS_ARUCO_MSGS = False

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

# Reliable QoS for ArUco marker observation relay topics.
# Must match the Reliable publishers in aruco_marker_pose_node.py
# and dynamic_arm_pose_measurement_node.py so the compiled C++
# run_subscribe_msckf_marker EKF node can receive observations.
_ARUCO_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ═══════════════════════════════════════════════════════════════════════════
# Source -> destination topic map  (head + arm camera pair)
# ═══════════════════════════════════════════════════════════════════════════

_SRC = {
    "head_pc":         "/head/d435i_head/depth/color/points",
    "arm_pc":          "/arm/d435i_arm/depth/color/points",
    "head_depth":      "/head/d435i_head/aligned_depth_to_color/image_raw",
    "arm_depth":       "/arm/d435i_arm/aligned_depth_to_color/image_raw",
    "head_img":        "/head/d435i_head/color/image_raw",
    "arm_img":         "/arm/d435i_arm/color/image_raw",
    "head_trackhist":  "/ov_msckf/trackhist",
    "arm_trackhist":   "/ov_msckf_arm/trackhist",
    "head_ci":         "/head/d435i_head/color/camera_info",
    "arm_ci":          "/arm/d435i_arm/color/camera_info",
    "head_odom":       "/ov_msckf/odomimu",
    "arm_odom":        "/ov_msckf_arm/odomimu",
    "head_mml":        "/ov_msckf/marker_map_locked",
    "arm_mml":         "/ov_msckf_arm/marker_map_locked",
}

_DST = {
    "head_pc":         "/jetson/head/points",
    "arm_pc":          "/jetson/arm/points",
    "head_depth":      "/jetson/head/depth/compressed",
    "arm_depth":       "/jetson/arm/depth/compressed",
    "head_img":        "/jetson/head/image/compressed",
    "arm_img":         "/jetson/arm/image/compressed",
    "head_trackhist":  "/jetson/head/trackhist",
    "arm_trackhist":   "/jetson/arm/trackhist",
    "head_ci":         "/jetson/head/camera_info",
    "arm_ci":          "/jetson/arm/camera_info",
    "head_odom":       "/jetson/head/odom",
    "arm_odom":        "/jetson/arm/odom",
    "head_mml":        "/jetson/head/marker_map_locked",
    "arm_mml":         "/jetson/arm/marker_map_locked",
    "head_aruco_obs":  "/jetson/head/aruco_observation",
    "arm_aruco_obs":   "/jetson/arm/aruco_observation",
    "arm_aruco_dyn":   "/jetson/arm/aruco_dynamic_observation",
    "arm_aruco_arm":   "/jetson/arm/aruco_arm_pose_observation",
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
# Image downsample helper (fast bilinear — for trackhist visualization only)
# ═══════════════════════════════════════════════════════════════════════════

def downsample_image(msg: Image, factor: int) -> Image:
    """Downsample an Image message by an integer factor using fast
    bilinear interpolation (``cv2.INTER_LINEAR``).

    Preserves the original encoding.  If OpenCV is unavailable or the
    encoding is not a simple 1/3/4-channel pixel format, the original
    message is returned unchanged.
    """
    if factor <= 1 or not _HAS_CV2:
        return msg

    enc = msg.encoding
    channels = {
        "mono8": 1, "mono16": 1,
        "rgb8": 3, "bgr8": 3,
        "rgba8": 4, "bgra8": 4,
    }.get(enc)
    if channels is None:
        return msg  # unknown encoding — don't touch it

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    try:
        arr = raw.reshape(msg.height, msg.width, channels)
    except ValueError:
        return msg

    new_w = msg.width // factor
    new_h = msg.height // factor
    resized = cv2.resize(arr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    out = Image()
    out.header = msg.header
    out.height = new_h
    out.width = new_w
    out.encoding = enc
    out.step = new_w * channels
    out.data = resized.tobytes()
    return out


# ═══════════════════════════════════════════════════════════════════════════
# Depth downsampler (NEAREST — no interpolation across depth discontinuities)
# ═══════════════════════════════════════════════════════════════════════════

def downsample_depth(msg: Image, factor: int) -> Image:
    """Downsample a Z16 depth image by an integer factor using
    Nearest-Neighbor interpolation (row/col stride — no OpenCV).

    Bilinear/bicubic averaging is strictly prohibited for depth data:
    it fabricates phantom depth values along structural edges where
    foreground and background pixels are averaged together.

    Returns the original message unchanged if factor <= 1 or the
    encoding is not a recognised 16-bit unsigned format.
    """
    if factor <= 1:
        return msg
    if msg.encoding not in ("16UC1", "mono16"):
        return msg  # unknown encoding — don't touch it

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint16)
    try:
        arr = raw.reshape(msg.height, msg.width)
    except ValueError:
        return msg

    # Nearest-neighbour via row/col striding — no interpolation
    arr_ds = arr[::factor, ::factor]

    out = Image()
    out.header = msg.header
    out.height = arr_ds.shape[0]
    out.width = arr_ds.shape[1]
    out.encoding = msg.encoding
    out.step = arr_ds.shape[1] * 2  # 2 bytes per uint16
    out.data = arr_ds.tobytes()
    return out


# ═══════════════════════════════════════════════════════════════════════════
# CameraInfo intrinsic scaling — match resolution after spatial downsampling
# ═══════════════════════════════════════════════════════════════════════════

def scale_camera_info(msg: CameraInfo, factor: int) -> CameraInfo:
    """Divide image dimensions and K/P matrix elements by *factor*.

    Focal lengths (fx, fy) and principal points (cx, cy) are in pixel
    units — halving the spatial resolution requires halving these values.
    Distortion coefficients (d) and rectification matrix (R) operate in
    normalised coordinates and are left unchanged.
    """
    if factor <= 1:
        return msg

    out = CameraInfo()
    out.header = msg.header
    out.height = msg.height // factor
    out.width = msg.width // factor
    out.distortion_model = msg.distortion_model
    out.d = msg.d  # normalised coords — unchanged

    # Intrinsic matrix K (3×3 row-major): [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    k = list(msg.k)
    k[0] /= factor  # fx
    k[2] /= factor  # cx
    k[4] /= factor  # fy
    k[5] /= factor  # cy
    out.k = k

    # Projection matrix P (3×4 row-major)
    p = list(msg.p)
    p[0] /= factor  # fx'
    p[2] /= factor  # cx'
    p[5] /= factor  # fy'
    p[6] /= factor  # cy'
    out.p = p

    # Rectification matrix R (3×3) — normalised coords, unchanged
    out.r = msg.r

    return out



# ═══════════════════════════════════════════════════════════════════════════
# Compression helpers — cv2 imencode (zero extra dependencies)
# ═══════════════════════════════════════════════════════════════════════════

def compress_depth_png(msg: Image) -> CompressedImage:
    """Compress a Z16 depth image to lossless PNG.

    PNG compression level 1 (fastest).  Returns a CompressedImage ready
    for publishing on a /compressed transport topic.
    """
    if not _HAS_CV2:
        return msg  # type: ignore[return-value]
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint16)
    try:
        arr = raw.reshape(msg.height, msg.width)
    except ValueError:
        return msg  # type: ignore[return-value]
    _, buf = cv2.imencode('.png', arr, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    out = CompressedImage()
    out.header = msg.header
    out.format = "png"
    out.data = buf.tobytes()
    return out


def compress_image_jpeg(msg: Image, quality: int = 75) -> CompressedImage:
    """Compress an RGB8/BGR8 image to lossy JPEG.

    JPEG quality 75 is a good balance between visual fidelity (~40 KB
    for 320×240) and wire size.
    """
    if not _HAS_CV2:
        return msg  # type: ignore[return-value]
    # Determine channel count from encoding
    enc = msg.encoding
    if enc in ("rgb8", "bgr8", "8UC3"):
        channels = 3
    elif enc in ("mono8", "8UC1"):
        channels = 1
    else:
        return msg  # type: ignore[return-value]
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    try:
        arr = raw.reshape(msg.height, msg.width, channels)
    except ValueError:
        return msg  # type: ignore[return-value]
    # JPEG expects BGR
    if enc == "rgb8":
        arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    _, buf = cv2.imencode('.jpg', arr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    out = CompressedImage()
    out.header = msg.header
    out.format = "jpeg"
    out.data = buf.tobytes()
    return out


# ═══════════════════════════════════════════════════════════════════════════
# JetsonRelay node
# ═══════════════════════════════════════════════════════════════════════════

class JetsonRelay(Node):
    """Relay node: raw -> /jetson/ with throttling + decimation."""

    def __init__(self) -> None:
        super().__init__("jetson_relay")

        # ── declare parameters ──────────────────────────────────────────
        self.declare_parameter("pointcloud.enabled", True)
        self.declare_parameter("pointcloud.hz", 5.0)
        self.declare_parameter("pointcloud.decimation.enabled", True)
        self.declare_parameter("pointcloud.decimation.step", 4)

        self.declare_parameter("image.enabled", True)
        self.declare_parameter("image.hz", 5.0)
        self.declare_parameter("image.downsample_factor", 1)

        self.declare_parameter("depth.enabled", False)
        self.declare_parameter("depth.hz", 5.0)
        self.declare_parameter("depth.downsample_factor", 2)
        self.declare_parameter("depth.compress", True)
        self.declare_parameter("depth.compress_format", "png")

        self.declare_parameter("image.compress", True)
        self.declare_parameter("image.compress_quality", 75)

        self.declare_parameter("aruco.enabled", False)

        self.declare_parameter("trackhist.enabled", True)
        self.declare_parameter("trackhist.hz", 5.0)
        self.declare_parameter("trackhist.downsample_factor", 2)

        self.declare_parameter("camera_info.enabled", True)
        self.declare_parameter("camera_info.hz", 1.0)

        self.declare_parameter("odometry.enabled", True)
        self.declare_parameter("odometry.hz", 50.0)
        self.declare_parameter("marker_map_locked.enabled", True)

        # ── read parameters ─────────────────────────────────────────────
        self._pc_enabled = self.get_parameter("pointcloud.enabled").value
        self._pc_hz = self.get_parameter("pointcloud.hz").value
        self._pc_dec_enabled = self.get_parameter("pointcloud.decimation.enabled").value
        self._pc_dec_step = self.get_parameter("pointcloud.decimation.step").value

        self._img_enabled = self.get_parameter("image.enabled").value
        self._img_hz = self.get_parameter("image.hz").value
        self._img_ds = self.get_parameter("image.downsample_factor").value

        self._depth_enabled = self.get_parameter("depth.enabled").value
        self._depth_hz = self.get_parameter("depth.hz").value
        self._depth_ds = self.get_parameter("depth.downsample_factor").value
        self._depth_compress = self.get_parameter("depth.compress").value
        self._depth_compress_fmt = self.get_parameter("depth.compress_format").value

        self._img_compress = self.get_parameter("image.compress").value
        self._img_compress_qty = self.get_parameter("image.compress_quality").value

        self._aruco_enabled = self.get_parameter("aruco.enabled").value and _HAS_ARUCO_MSGS

        self._trackhist_enabled = self.get_parameter("trackhist.enabled").value
        self._trackhist_hz = self.get_parameter("trackhist.hz").value
        self._trackhist_ds = self.get_parameter("trackhist.downsample_factor").value

        self._ci_enabled = self.get_parameter("camera_info.enabled").value
        self._ci_hz = self.get_parameter("camera_info.hz").value

        self._odom_enabled = self.get_parameter("odometry.enabled").value
        self._odom_hz = self.get_parameter("odometry.hz").value
        self._mml_enabled = self.get_parameter("marker_map_locked.enabled").value

        # ── rate gates (one per stream per camera) ──────────────────────
        self._gates: dict[str, RateGate] = {}
        for cam in CAMERAS:
            self._gates[f"pc_{cam}"] = RateGate(self._pc_hz)
            self._gates[f"img_{cam}"] = RateGate(self._img_hz)
            self._gates[f"depth_{cam}"] = RateGate(self._depth_hz)
            self._gates[f"trackhist_{cam}"] = RateGate(self._trackhist_hz)
            self._gates[f"ci_{cam}"] = RateGate(self._ci_hz)
            self._gates[f"odom_{cam}"] = RateGate(self._odom_hz)

        # ── setup pubs/subs ─────────────────────────────────────────────
        if self._pc_enabled:
            self._setup_pointclouds()
        if self._img_enabled:
            self._setup_images()
        if self._depth_enabled:
            self._setup_depth()
        if self._aruco_enabled:
            self._setup_aruco()
        if self._trackhist_enabled:
            self._setup_trackhist()
        if self._ci_enabled:
            self._setup_camera_info()
        if self._odom_enabled:
            self._setup_odometry()
        if self._mml_enabled:
            self._setup_marker_map_locked()

        # ── health ──────────────────────────────────────────────────────
        self._health_pub = self.create_publisher(
            Bool, "/jetson/relay/health", _HEALTH_QOS
        )
        self._health_timer = self.create_timer(1.0, self._publish_health)

        self._log_config()

    # ── setup helpers ────────────────────────────────────────────────────

    def _setup_pointclouds(self) -> None:
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
        for cam in CAMERAS:
            key = f"{cam}_img"
            self.create_subscription(
                Image, _SRC[key],
                lambda m, c=cam: self._on_img(m, c), _SENSOR_QOS,
            )
        self._img_pub = {
            cam: self.create_publisher(
                CompressedImage if self._img_compress else Image,
                _DST[f"{cam}_img"],
                _SENSOR_QOS,
            )
            for cam in CAMERAS
        }

    def _setup_depth(self) -> None:
        for cam in CAMERAS:
            key = f"{cam}_depth"
            self.create_subscription(
                Image, _SRC[key],
                lambda m, c=cam: self._on_depth(m, c), _SENSOR_QOS,
            )
        self._depth_pub = {
            cam: self.create_publisher(
                CompressedImage if self._depth_compress else Image,
                _DST[f"{cam}_depth"],
                _SENSOR_QOS,
            )
            for cam in CAMERAS
        }

    def _setup_aruco(self) -> None:
        """Subscribe to Jetson ArUco marker observation topics.

        These topic names match what the OpenVINS phase2 launch files
        remap the marker_pose_node outputs to, NOT the raw OpenVINS
        subscription names.
        """
        for cam in CAMERAS:
            # Fixed marker observation for each camera
            self.create_subscription(
                MarkerPoseObservation,
                f"/{cam}/marker_pose/observation",
                lambda m, c=cam: self._on_aruco_obs(m, c),
                _ARUCO_RELIABLE_QOS,
            )
        # Dynamic observation: head camera sees marker ID 2 on arm
        self.create_subscription(
            DynamicMarkerObservation,
            "/head/marker_pose/dynamic_observation",
            self._on_aruco_dynamic_obs,
            _ARUCO_RELIABLE_QOS,
        )
        # Arm-side dynamic arm pose (converted by dynamic_arm_updater)
        self.create_subscription(
            DynamicArmPoseObservation,
            "/arm/marker_pose/dynamic_arm_pose_observation",
            self._on_aruco_dynamic_arm_pose,
            _ARUCO_RELIABLE_QOS,
        )
        self._aruco_obs_pub = {
            cam: self.create_publisher(
                MarkerPoseObservation, _DST[f"{cam}_aruco_obs"], _ARUCO_RELIABLE_QOS
            )
            for cam in CAMERAS
        }
        self._aruco_dyn_pub = self.create_publisher(
            DynamicMarkerObservation, _DST["arm_aruco_dyn"], _ARUCO_RELIABLE_QOS
        )
        # Separate publisher for DynamicArmPoseObservation — this is a
        # distinct message type from DynamicMarkerObservation.  Publishing
        # through the wrong-type publisher silently drops the message.
        self._aruco_arm_pose_pub = self.create_publisher(
            DynamicArmPoseObservation, _DST["arm_aruco_arm"], _ARUCO_RELIABLE_QOS
        )

    def _setup_trackhist(self) -> None:
        for cam in CAMERAS:
            key = f"{cam}_trackhist"
            self.create_subscription(
                Image, _SRC[key],
                lambda m, c=cam: self._on_trackhist(m, c), _SENSOR_QOS,
            )
        self._trackhist_pub = {
            cam: self.create_publisher(Image, _DST[f"{cam}_trackhist"], _SENSOR_QOS)
            for cam in CAMERAS
        }

    def _setup_camera_info(self) -> None:
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
        # Preserve original RealSense hardware timestamp — the ASIC clock is
        # the same domain for both depth and colour frames from a single
        # D435i, so the temporal bond needed by depth_image_proc's
        # approximate-time synchronizer is preserved.
        self._pc_pub[camera].publish(msg)

    def _on_img(self, msg: Image, camera: str) -> None:
        if not self._gates[f"img_{camera}"].should_publish():
            return
        if self._img_ds > 1:
            msg = downsample_image(msg, self._img_ds)
        # Preserve original hardware timestamp (see _on_pc comment).
        if self._img_compress:
            msg = compress_image_jpeg(msg, self._img_compress_qty)
        self._img_pub[camera].publish(msg)

    def _on_depth(self, msg: Image, camera: str) -> None:
        """Throttled + downsampled depth relay (NEAREST), then compressed."""
        if not self._gates[f"depth_{camera}"].should_publish():
            return
        if self._depth_ds > 1:
            msg = downsample_depth(msg, self._depth_ds)
        # Preserve original hardware timestamp — depth and colour frames
        # from the same D435i share the same ASIC clock domain, so the
        # temporal bond needed by depth_image_proc's approximate-time
        # synchronizer is preserved.
        if self._depth_compress:
            msg = compress_depth_png(msg)
        self._depth_pub[camera].publish(msg)

    def _on_trackhist(self, msg: Image, camera: str) -> None:
        if not self._gates[f"trackhist_{camera}"].should_publish():
            return
        if self._trackhist_ds > 1:
            msg = downsample_image(msg, self._trackhist_ds)
        self._trackhist_pub[camera].publish(msg)

    def _on_ci(self, msg: CameraInfo, camera: str) -> None:
        if not self._gates[f"ci_{camera}"].should_publish():
            return
        # Scale intrinsics to match the downsampled image/depth resolution.
        # The same factor is used for both colour and depth channels since
        # aligned depth is registered to the colour frame.
        if self._depth_ds > 1:
            msg = scale_camera_info(msg, self._depth_ds)
        self._ci_pub[camera].publish(msg)

    def _on_odom(self, msg: Odometry, camera: str) -> None:
        if not self._gates[f"odom_{camera}"].should_publish():
            return
        self._odom_pub[camera].publish(msg)

    def _on_mml(self, msg: Bool, camera: str) -> None:
        self._mml_pub[camera].publish(msg)

    def _on_aruco_obs(self, msg, camera: str) -> None:
        """Fixed marker observation — passthrough relay."""
        self._aruco_obs_pub[camera].publish(msg)

    def _on_aruco_dynamic_obs(self, msg) -> None:
        """Dynamic marker observation (head sees marker ID 2 on arm)."""
        self._aruco_dyn_pub.publish(msg)

    def _on_aruco_dynamic_arm_pose(self, msg) -> None:
        """Converted dynamic arm pose (arm-side).

        This is a DynamicArmPoseObservation (arm pose in marker_map from
        head-observed ID2).  Must be published on a matching-type publisher
        — publishing through the DynamicMarkerObservation publisher silently
        drops the message.
        """
        self._aruco_arm_pose_pub.publish(msg)

    # ── health ───────────────────────────────────────────────────────────

    def _publish_health(self) -> None:
        self._health_pub.publish(Bool(data=True))

    # ── startup log ──────────────────────────────────────────────────────

    def _log_config(self) -> None:
        info = self.get_logger().info
        info("── JetsonRelay configuration ──────────────────────")
        info(f"  pointcloud:  enabled={self._pc_enabled}  hz={self._pc_hz:.1f}  "
             f"dec={self._pc_dec_enabled} (step={self._pc_dec_step})")
        info(f"  image:       enabled={self._img_enabled}  hz={self._img_hz:.1f}  "
             f"ds={self._img_ds}x")
        info(f"  depth:       enabled={self._depth_enabled}  hz={self._depth_hz:.1f}  "
             f"(raw passthrough)")
        info(f"  aruco:       enabled={self._aruco_enabled}  "
             f"(msgs_available={_HAS_ARUCO_MSGS})")
        info(f"  trackhist:   enabled={self._trackhist_enabled}  hz={self._trackhist_hz:.1f}  "
             f"ds={self._trackhist_ds}x")
        info(f"  camera_info: enabled={self._ci_enabled}  hz={self._ci_hz:.1f}")
        info(f"  odometry:    enabled={self._odom_enabled}  hz={self._odom_hz:.1f}")
        info(f"  marker_map_locked: enabled={self._mml_enabled}  (latched)")
        info("  health:      /jetson/relay/health @ 1 Hz")
        info("──────────────────────────────────────────────────")


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
