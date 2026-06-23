#!/usr/bin/env python3
"""Decompress bridge — subscribes to CompressedImage topics from the Jetson
self._depth_pub[camera] = self.create_publisher(
self._image_pub[camera] = self.create_publisher(
                Image, f"/local/{camera}/image_raw", _BEST_EFFORT_QOS
            )
while the Jetson transmits compressed data over the Cat5e link.

Usage (as an ExecuteProcess in a launch file)::

    python3 decompress_bridge.py \
        --sub /jetson/head/depth/compressed \
        --pub /jetson/head/depth

Or with ROS2 args::

    python3 decompress_bridge.py --ros-args \\
        -p sub_topic:=/jetson/head/depth/compressed \\
        -p pub_topic:=/jetson/head/depth
"""

from __future__ import annotations

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image
import cv2


from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
# Reliable — the decompressed output must match PointCloudXyzrgbNode's
# rclcpp::SystemDefaultsQoS() subscriber, which hardcodes RELIABLE.
# Running inside local RAM, this carries zero network overhead.
_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class DecompressBridge(Node):
    """Subscribe to a CompressedImage topic, publish raw Image."""

    def __init__(self) -> None:
        super().__init__("decompress_bridge")

        self.declare_parameter("sub_topic", "")
        self.declare_parameter("pub_topic", "")
        self.declare_parameter("frame_id", "")
        # Telemetry interval (seconds); 0 disables periodic logging.
        self.declare_parameter("telemetry_interval_s", 5.0)

        sub_topic = self.get_parameter("sub_topic").value
        pub_topic = self.get_parameter("pub_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._telemetry_interval = float(
            self.get_parameter("telemetry_interval_s").value)

        if not sub_topic or not pub_topic:
            self.get_logger().error(
                "sub_topic and pub_topic must be set (got "
                f"sub={sub_topic!r}, pub={pub_topic!r})"
            )
            raise SystemExit(1)

        self._pub = self.create_publisher(Image, pub_topic, _RELIABLE_QOS)
        self._sub = self.create_subscription(
            CompressedImage,
            sub_topic,
            self._on_compressed,
            _BEST_EFFORT_QOS,
        )

        # ── Telemetry counters ────────────────────────────────────────
        # Track input/output rates and decode failures so the host-side
        # pipeline_diagnostics_node can correlate decompressor health with
        # pointcloud output via the [DIAG-PC] block.
        self._in_count = 0
        self._out_count = 0
        self._fail_count = 0
        self._fail_unknown_format = 0
        self._fail_decode = 0
        self._bytes_in = 0
        self._bytes_out = 0
        self._first_in_ts: float | None = None
        self._last_in_ts: float | None = None
        self._last_summary_in = 0
        self._last_summary_out = 0
        self._last_summary_host = 0.0

        self.get_logger().info(
            f"DecompressBridge: {{{sub_topic}}} -> {{{pub_topic}}}"
        )
        if self._telemetry_interval > 0:
            self.create_timer(self._telemetry_interval, self._telemetry)

    def _telemetry(self) -> None:
        """Log a one-line throughput + failure summary.

        Format is machine-parsable so analyze_log.py can ingest it as a
        DIAG-rate-style line:
            [DIAG-DECOMPRESS] sub=/jetson/head/depth/compressed
                in=4.2Hz out=4.2Hz fails=0/0 bytes_in=36.2KB/s
        """
        host_now = time.time()
        dt = (host_now - self._last_summary_host) if self._last_summary_host else self._telemetry_interval
        if dt <= 0:
            dt = self._telemetry_interval
        d_in = self._in_count - self._last_summary_in
        d_out = self._out_count - self._last_summary_out
        in_hz = d_in / dt
        out_hz = d_out / dt
        bytes_in_rate = (self._bytes_in / 1024.0) / dt if dt else 0.0
        # Snapshot cumulative counters for the summary line
        self.get_logger().info(
            f"[DIAG-DECOMPRESS] sub={self.get_parameter('sub_topic').value} "
            f"in={in_hz:.1f}Hz out={out_hz:.1f}Hz "
            f"fails={self._fail_decode}/{self._fail_unknown_format} "
            f"bytes_in={bytes_in_rate:.1f}KB/s "
            f"(total in={self._in_count} out={self._out_count})"
        )
        self._last_summary_in = self._in_count
        self._last_summary_out = self._out_count
        self._last_summary_host = host_now
        # Reset byte counter so the rate reflects the last window
        self._bytes_in = 0

    def _on_compressed(self, msg: CompressedImage) -> None:
        """Decode the compressed image and republish as raw Image."""
        host_now = time.time()
        self._in_count += 1
        self._bytes_in += len(msg.data)
        if self._first_in_ts is None:
            self._first_in_ts = host_now
        self._last_in_ts = host_now
        try:
            buf = np.frombuffer(msg.data, np.uint8)
            if msg.format in ("png",):
                arr = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
                encoding = self._infer_encoding(arr)
            elif msg.format in ("jpeg", "jpg"):
                arr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                # cv2.imdecode returns BGR; convert to RGB for ROS convention
                if arr.ndim == 3 and arr.shape[2] == 3:
                    arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
                encoding = "rgb8" if arr.ndim == 3 else "mono8"
            else:
                self._fail_count += 1
                self._fail_unknown_format += 1
                self.get_logger().warning(
                    f"Unknown compression format: {msg.format}", throttle_duration_sec=10
                )
                return
        except Exception as exc:
            self._fail_count += 1
            self._fail_decode += 1
            self.get_logger().warning(
                f"Failed to decompress image: {exc}", throttle_duration_sec=10
            )
            return

        out = Image()
        out.header = msg.header
        if self._frame_id:
            out.header.frame_id = self._frame_id
        out.height = arr.shape[0]
        out.width = arr.shape[1]
        out.encoding = encoding
        out.step = arr.shape[1] * (1 if arr.ndim == 2 else arr.shape[2])
        out.data = arr.tobytes()
        self._bytes_out += len(out.data)
        self._pub.publish(out)
        self._out_count += 1

    @staticmethod
    def _infer_encoding(arr: np.ndarray) -> str:
        """Guess the ROS image encoding from a numpy array."""
        if arr.ndim == 2:
            # Single-channel — check dtype
            if arr.dtype == np.uint16:
                return "16UC1"
            return "mono8"
        if arr.ndim == 3 and arr.shape[2] == 3:
            return "rgb8"
        return "8UC1"


def main() -> None:
    rclpy.init()
    try:
        node = DecompressBridge()
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
