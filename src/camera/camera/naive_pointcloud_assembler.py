#!/usr/bin/env python3
"""Naive pointcloud assembler — replaces depth_image_proc::PointCloudXyzrgbNode.

This node constructs a coloured PointCloud2 from depth + RGB + CameraInfo
**without any time synchronization**.  It caches the latest of each input
independently and assembles a cloud whenever a fresh depth frame is
available.  Frames that arrive during processing are simply dropped
(overwrite the cache), providing natural backpressure relief.

Design rationale
----------------
The native ``depth_image_proc::PointCloudXyzrgbNode`` uses a 4-way
``ApproximateTimeSynchronizer`` (depth, rgb, depth camera_info, rgb
camera_info).  When the RealSense ASIC clock (used for image/depth
timestamps) differs from the system clock (used for camera_info), the
synchronizer can never match a tuple, producing a 100% drop rate.

This assembler sidesteps the problem entirely:
  - CameraInfo intrinsics are static, so its timestamp is irrelevant.
  - Depth and RGB are already paired on the Jetson via the token gate.
  - The depth image is ``aligned_depth_to_color``, so depth and RGB
    pixels map 1:1 without an additional extrinsic transform.

The output cloud's header stamp is copied from the depth image, which is
restamped to Jetson system clock in the relay.  This ensures the
downstream fusion node's TF lookup (``lookup_transform_full``) finds the
temporally-correct transform.

Topics (per side)
-----------------
Subscribe (BEST_EFFORT, matching decompress bridges):
  depth_topic       sensor_msgs/Image        — aligned_depth_to_color (16UC1)
  rgb_topic         sensor_msgs/Image        — colour image (rgb8)
  camera_info_topic sensor_msgs/CameraInfo   — intrinsics (K matrix)

Publish (BEST_EFFORT):
  output_topic      sensor_msgs/PointCloud2  — XYZRGB cloud
"""

from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


# BEST_EFFORT QoS — the decompress bridges publish RELIABLE, but BEST_EFFORT
# subscribers are compatible and avoid head-of-line blocking if a frame is
# dropped.  The assembler is designed to be resilient to drops.
_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# RELIABLE for output — the fusion node's subscriber uses RELIABLE by default.
_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def _rgb_to_float_packed_flat(rgb_flat: np.ndarray) -> np.ndarray:
    """Pack an (N, 3) uint8 array into (N,) float32 (0x00RRGGBB).

    PointCloud2 stores colour as a float32 whose bit pattern is
    0x00RRGGBB.  We build the uint32 then view-cast to float32 so the
    bytes land correctly in the message buffer.
    """
    r = rgb_flat[:, 0].astype(np.uint32)
    g = rgb_flat[:, 1].astype(np.uint32)
    b = rgb_flat[:, 2].astype(np.uint32)
    packed = (r << 16) | (g << 8) | b
    return packed.view(np.float32)


def _build_cloud2(
    xyz: np.ndarray,
    rgb_packed: np.ndarray,
    header,
) -> PointCloud2:
    """Build an XYZRGB PointCloud2 from flat numpy arrays.

    Parameters
    ----------
    xyz        : (N, 3) float32
    rgb_packed : (N,)   float32  — 0x00RRGGBB viewed as float32
    header     : std_msgs/Header — frame_id + stamp copied verbatim
    """
    n = xyz.shape[0]
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16  # 4 fields * 4 bytes
    msg.row_step = 16 * n
    msg.is_dense = True

    # Interleave xyz and rgb into a single contiguous byte buffer.
    buf = np.empty((n, 4), dtype=np.float32)
    buf[:, 0:3] = xyz
    buf[:, 3] = rgb_packed
    msg.data = buf.tobytes()
    return msg


class NaivePointcloudAssembler(Node):
    """Assemble coloured PointCloud2 from cached depth/RGB/CameraInfo.

    No message_filters, no ApproximateTimeSynchronizer, no slop.  Just
    cache-and-go.
    """

    def __init__(self) -> None:
        super().__init__("naive_pointcloud_assembler")

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter("side", "head")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("rgb_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("max_rate_hz", 5.0)
        self.declare_parameter("depth_scale", 1000.0)  # mm -> m
        self.declare_parameter("telemetry_interval_s", 5.0)
        self.declare_parameter("max_depth_m", 10.0)  # filter far points

        self._side = str(self.get_parameter("side").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        rgb_topic = str(self.get_parameter("rgb_topic").value)
        ci_topic = str(self.get_parameter("camera_info_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._max_rate = float(self.get_parameter("max_rate_hz").value)
        self._depth_scale = float(self.get_parameter("depth_scale").value)
        self._telemetry_interval = float(
            self.get_parameter("telemetry_interval_s").value)
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)

        if not depth_topic or not output_topic:
            self.get_logger().error(
                f"depth_topic and output_topic must be set "
                f"(got depth={depth_topic!r}, output={output_topic!r})")
            raise SystemExit(1)

        # ── Cached inputs (latest-wins, protected by a lock) ─────────
        self._lock = threading.Lock()
        self._latest_depth: Image | None = None
        self._latest_rgb: Image | None = None
        self._latest_ci: CameraInfo | None = None
        self._depth_seq: int = 0  # increments on each new depth frame
        self._last_processed_seq: int = -1

        # ── Processing guard ─────────────────────────────────────────
        self._processing = False

        # ── Publisher ────────────────────────────────────────────────
        self._pub = self.create_publisher(PointCloud2, output_topic, _RELIABLE_QOS)

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(
            Image, depth_topic, self._on_depth, _BEST_EFFORT_QOS)
        if rgb_topic:
            self.create_subscription(
                Image, rgb_topic, self._on_rgb, _BEST_EFFORT_QOS)
        if ci_topic:
            self.create_subscription(
                CameraInfo, ci_topic, self._on_ci, _BEST_EFFORT_QOS)

        # ── Telemetry counters ───────────────────────────────────────
        self._depth_in_count = 0
        self._rgb_in_count = 0
        self._ci_in_count = 0
        self._out_count = 0
        self._skip_no_ci = 0
        self._skip_processing = 0
        self._skip_no_new = 0
        self._last_points = 0
        self._last_proc_ms = 0.0
        self._last_telemetry_host = time.time()
        self._first_depth_host = 0.0
        self._last_depth_host = 0.0
        self._ci_warned = False

        # ── Timer ────────────────────────────────────────────────────
        interval = 1.0 / max(self._max_rate, 0.1)
        self.create_timer(interval, self._tick)

        self.get_logger().info(
            f"NaivePointcloudAssembler[{self._side}]: "
            f"depth={{{depth_topic}}} rgb={{{rgb_topic}}} "
            f"ci={{{ci_topic}}} -> {{{output_topic}}} "
            f"(max_rate={self._max_rate:.1f}Hz)")

        if self._telemetry_interval > 0:
            self.create_timer(self._telemetry_interval, self._telemetry)

    # ── Callbacks ─────────────────────────────────────────────────────

    def _on_depth(self, msg: Image) -> None:
        """Cache latest depth; the timer picks it up."""
        now = time.time()
        if self._first_depth_host == 0.0:
            self._first_depth_host = now
        self._last_depth_host = now
        self._depth_in_count += 1
        with self._lock:
            self._latest_depth = msg
            self._depth_seq += 1

    def _on_rgb(self, msg: Image) -> None:
        self._rgb_in_count += 1
        with self._lock:
            self._latest_rgb = msg

    def _on_ci(self, msg: CameraInfo) -> None:
        self._ci_in_count += 1
        with self._lock:
            self._latest_ci = msg

    # ── Timer-driven assembly ─────────────────────────────────────────

    def _tick(self) -> None:
        """Assemble and publish a cloud if a fresh depth frame is waiting."""
        # Snapshot under lock (fast, non-blocking for callbacks)
        with self._lock:
            if self._processing:
                # Previous frame still processing — drop this tick.
                self._skip_processing += 1
                return
            depth_msg = self._latest_depth
            seq = self._depth_seq
            rgb_msg = self._latest_rgb
            ci_msg = self._latest_ci

        if depth_msg is None:
            return
        if seq == self._last_processed_seq:
            # No new depth since last processing.
            self._skip_no_new += 1
            return
        if ci_msg is None:
            self._skip_no_ci += 1
            if not self._ci_warned:
                self._ci_warned = True
                self.get_logger().warn(
                    f"[{self._side}] Waiting for CameraInfo on "
                    f"{self.get_parameter('camera_info_topic').value} — "
                    f"skipping pointcloud assembly until intrinsics arrive.",
                    throttle_duration_sec=10.0)
            return

        # Claim the processing slot so new depth frames just overwrite cache.
        with self._lock:
            self._processing = True

        try:
            t0 = time.perf_counter()
            cloud = self._assemble(depth_msg, rgb_msg, ci_msg)
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._last_proc_ms = dt_ms
            self._last_points = cloud.width

            self._pub.publish(cloud)
            self._out_count += 1
            self._last_processed_seq = seq
        except Exception as exc:
            self.get_logger().error(
                f"[{self._side}] Assembly failed: {exc}",
                throttle_duration_sec=5.0)
        finally:
            with self._lock:
                self._processing = False

    # ── Core assembly ─────────────────────────────────────────────────

    def _assemble(
        self,
        depth_msg: Image,
        rgb_msg: Image | None,
        ci_msg: CameraInfo,
    ) -> PointCloud2:
        """Deproject depth to XYZ, attach RGB, build PointCloud2."""
        # ── Parse depth image ────────────────────────────────────────
        depth = self._decode_depth(depth_msg)
        h, w = depth.shape

        # ── Extract intrinsics from CameraInfo K matrix ──────────────
        # K = [fx  0  cx]
        #     [ 0 fy  cy]
        #     [ 0  0   1]
        fx = ci_msg.k[0]
        fy = ci_msg.k[4]
        cx = ci_msg.k[2]
        cy = ci_msg.k[5]

        # Defensive: if camera_info resolution doesn't match depth, the
        # intrinsics are wrong.  Log once and proceed (the cloud will be
        # distorted but not crash).
        if ci_msg.width != w or ci_msg.height != h:
            self.get_logger().warn(
                f"[{self._side}] CameraInfo resolution "
                f"({ci_msg.width}x{ci_msg.height}) != depth ({w}x{h}) — "
                f"intrinsics may be incorrect.",
                throttle_duration_sec=30.0)

        # ── Vectorized pinhole deprojection ──────────────────────────
        # Create pixel coordinate grids.
        u = np.arange(w, dtype=np.float32)  # (W,)
        v = np.arange(h, dtype=np.float32)  # (H,)
        u_grid, v_grid = np.meshgrid(u, v)  # (H, W) each

        # Depth in metres.
        z = depth.astype(np.float32) / self._depth_scale  # (H, W)

        # Valid mask: non-zero depth, finite, within range.
        valid = (z > 0.0) & np.isfinite(z) & (z <= self._max_depth_m)

        # Deproject only valid pixels.
        z_v = z[valid]
        x_v = (u_grid[valid] - cx) * z_v / fx
        y_v = (v_grid[valid] - cy) * z_v / fy

        xyz = np.column_stack([x_v, y_v, z_v]).astype(np.float32)

        # ── RGB colour assignment ────────────────────────────────────
        n = xyz.shape[0]
        rgb_packed = np.zeros(n, dtype=np.float32)

        if rgb_msg is not None:
            rgb = self._decode_rgb(rgb_msg, h, w)
            if rgb is not None:
                # Depth is aligned_depth_to_color → 1:1 pixel mapping.
                rgb_flat = rgb[valid]  # (n, 3) uint8
                rgb_packed = _rgb_to_float_packed_flat(rgb_flat)

        # ── Build PointCloud2 ────────────────────────────────────────
        # Use the depth image's header (frame_id + stamp).  The stamp is
        # already in Jetson system-clock domain (restamped in the relay),
        # so downstream TF lookups are temporally accurate.
        header = depth_msg.header
        return _build_cloud2(xyz, rgb_packed, header)

    # ── Decoding helpers ──────────────────────────────────────────────

    @staticmethod
    def _decode_depth(msg: Image) -> np.ndarray:
        """Decode a depth Image into a 2D uint16/float32 array (metres-free)."""
        if msg.encoding in ("16UC1", "mono16"):
            return np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width)
        if msg.encoding in ("32FC1",):
            return np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width)
        # Fallback: try uint16 (most common for RealSense depth).
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width)

    @staticmethod
    def _decode_rgb(msg: Image, h: int, w: int) -> np.ndarray | None:
        """Decode an RGB Image into an (H, W, 3) uint8 array, or None."""
        try:
            if msg.encoding in ("rgb8", "bgr8"):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                if msg.encoding == "bgr8":
                    arr = arr[:, :, ::-1]  # BGR -> RGB
                return arr
            if msg.encoding in ("rgba8", "bgra8"):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 4)
                arr = arr[:, :, :3]
                if msg.encoding == "bgra8":
                    arr = arr[:, :, ::-1]
                return arr
            if msg.encoding == "mono8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width)
                return np.stack([arr, arr, arr], axis=-1)
        except Exception:
            return None
        return None

    # ── Telemetry ─────────────────────────────────────────────────────

    def _telemetry(self) -> None:
        """Log a machine-parsable throughput summary.

        Format matches the [DIAG-PC] style so analyze_log.py / the
        pipeline_diagnostics_node can ingest it.
        """
        host_now = time.time()
        dt = host_now - self._last_telemetry_host if self._last_telemetry_host else self._telemetry_interval
        if dt <= 0:
            dt = self._telemetry_interval
        depth_hz = (self._depth_in_count / dt) if dt else 0.0
        rgb_hz = (self._rgb_in_count / dt) if dt else 0.0
        out_hz = (self._out_count / dt) if dt else 0.0

        self.get_logger().info(
            f"[DIAG-ASM] side={self._side} "
            f"depth_in={depth_hz:.1f}Hz rgb_in={rgb_hz:.1f}Hz "
            f"ci_received={'yes' if self._latest_ci is not None else 'no'} "
            f"out={out_hz:.1f}Hz "
            f"points={self._last_points} "
            f"proc_ms={self._last_proc_ms:.1f} "
            f"skips(proc/no_ci/no_new)="
            f"{self._skip_processing}/{self._skip_no_ci}/{self._skip_no_new}"
        )
        # Reset windowed counters
        self._depth_in_count = 0
        self._rgb_in_count = 0
        self._out_count = 0
        self._skip_processing = 0
        self._skip_no_ci = 0
        self._skip_no_new = 0
        self._last_telemetry_host = host_now


def main() -> None:
    rclpy.init()
    try:
        node = NaivePointcloudAssembler()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
