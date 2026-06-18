#!/usr/bin/env python3
"""Rate-limit TF broadcasts to prevent CycloneDDS retransmission storms.

When OpenVINS odometry arrives at ~200-300 Hz and multiple nodes rebroadcast
every message as a TF transform (Reliable QoS), the combined /tf rate can
exceed 1,400 Hz.  Under CPU pressure (~91 %), even tiny packet drops trigger
a DDS NACK/retransmission spiral that overwhelms the 1 Gbps link, starving
the PointCloud2 streams (0.5 MB each) of usable bandwidth.

ARCHITECTURE
------------
This node is the bridge between a local (Jetson-internal) high-rate TF
topic and the global /tf topic that crosses the physical link:

  aruco nodes --200 Hz--> /tf_raw  (Jetson-local, BestEffort)
  OpenVINS     --200 Hz--> /tf_raw  (suppressed by publish_*_tf:=False)
                              |
                     tf_throttle_node
                     (subscribe_topic=/tf_raw, BestEffort)
                              |
                         rate-limit to 50 Hz per child frame
                              |
                   publish to /tf (global, BestEffort, crosses the cable)

This way CycloneDDS only broadcasts the 50 Hz /tf stream over the Cat5e
link.  The 200 Hz /tf_raw traffic stays entirely within the Jetson's local
shared-memory transport (/dev/shm).

The output /tf publisher uses BestEffort QoS by default.  Dynamic TF frames
are ephemeral — a dropped frame is replaced by the next update ~20 ms later.
This eliminates the last remaining NACK source for TF traffic on the wire.

USAGE
-----
Deploy on the Jetson by adding to the launch file:

  ros2 run camera tf_throttle_node --ros-args \
      -p max_hz:=50.0 \
      -p subscribe_topic:=/tf_raw \
      -p child_frames:='["head_imu_openvins_corrected", "arm_imu_openvins_corrected"]'

Parameters
----------
max_hz : float (default 50.0)
    Maximum publish rate per child frame.
subscribe_topic : str (default /tf_raw)
    Topic to subscribe to for raw TF messages.  The throttle publishes
    its output to the global /tf topic.
child_frames : list[str]
    Child frames to throttle.  If empty, throttle ALL frames.
subscribe_qos : str
    Reliability for the subscription ("best_effort" or "reliable").
    Default "best_effort" avoids contributing to the Reliable
    retransmission pressure.
publish_qos : str
    Reliability for the output /tf publisher ("best_effort" or "reliable").
    Default "best_effort" — dynamic TF frames are replaceable and a
    dropped frame is harmless.
"""

from __future__ import annotations

import hashlib
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster


class TFThrottleNode(Node):
    """Subscribe to raw TF stream, rate-limit, republish on global /tf."""

    def __init__(self):
        super().__init__("tf_throttle")

        # -- Parameters --------------------------------------------------
        self.declare_parameter("max_hz", 50.0)
        self.declare_parameter("subscribe_topic", "/tf_raw")
        self.declare_parameter("child_frames", [""])
        self.declare_parameter("subscribe_qos", "best_effort")
        self.declare_parameter("publish_qos", "best_effort")

        self._max_hz = float(self.get_parameter("max_hz").value)
        self._interval_ns = int(1e9 / max(self._max_hz, 0.1))
        self._subscribe_topic = self.get_parameter("subscribe_topic").value
        self._publish_qos = self.get_parameter("publish_qos").value.lower()
        raw_frames = self.get_parameter("child_frames").value
        qos_str = self.get_parameter("subscribe_qos").value.lower()

        # Normalise: empty list or [""] means "all frames"
        self._child_frames: set[str] = set()
        if raw_frames and isinstance(raw_frames, list):
            for f in raw_frames:
                f = str(f).strip()
                if f:
                    self._child_frames.add(f)

        # -- QoS for the subscription ------------------------------------
        if qos_str == "reliable":
            sub_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
        else:
            sub_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )

        # -- Publishers / Subscribers ------------------------------------
        # Output: global /tf via raw publisher (crosses the wire).
        # Dynamic TF frames are ephemeral — a dropped frame is replaced by
        # the next one ~20 ms later.  BestEffort eliminates the last NACK
        # source for TF traffic on the 1 Gbps link.
        if self._publish_qos == "best_effort":
            pub_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
        else:
            pub_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
        self._tf_pub = self.create_publisher(TFMessage, "/tf", pub_qos)
        # Static transforms: keep Reliable on /tf_static — they are
        # low-frequency calibration frames that must not be dropped.
        self._tf_static_broadcaster = StaticTransformBroadcaster(self)
        # Input: local raw stream (stays within Jetson shared memory).
        self.create_subscription(TFMessage, self._subscribe_topic,
                                 self._on_tf, sub_qos)
        self.create_subscription(
            TFMessage, "/tf_static", self._on_tf_static, sub_qos)

        # -- Per-frame rate-limit state ----------------------------------
        self._last_publish_ns: dict[str, int] = {}
        # Content dedup: hash -> (child_frame, timestamp_ns)
        self._sent_hashes: dict[str, tuple[str, int]] = {}
        self._dedup_last_purge = time.monotonic()
        self._dedup_purge_interval_s = 10.0

        # -- Counters ----------------------------------------------------
        self._rx_count = 0
        self._tx_count = 0
        self._skipped_count = 0
        self._dedup_count = 0
        self.create_timer(10.0, self._log_stats)

        frames_str = (
            ", ".join(sorted(self._child_frames)) if self._child_frames
            else "ALL"
        )
        self.get_logger().info(
            f"TF throttle active: subscribing to {self._subscribe_topic}, "
            f"max_hz={self._max_hz:.0f} Hz, "
            f"frames=[{frames_str}], sub_qos={qos_str}, "
            f"pub_qos={self._publish_qos}"
        )

    # -- Callbacks -------------------------------------------------------

    def _on_tf(self, msg: TFMessage):
        """Process incoming raw TF messages with rate limiting."""
        self._maybe_purge_dedup()
        now_ns = self.get_clock().now().nanoseconds

        for transform in msg.transforms:
            self._rx_count += 1
            child = transform.child_frame_id

            # Filter by child frame whitelist (if configured).
            if self._child_frames and child not in self._child_frames:
                continue

            # Content dedup: compute hash of the transform data to
            # prevent echoing our own republished messages.
            content_hash = self._hash_transform(transform)
            dedup_key = f"{child}:{content_hash}"
            if dedup_key in self._sent_hashes:
                self._dedup_count += 1
                continue

            # Rate limit per child frame.
            last_ns = self._last_publish_ns.get(child, 0)
            if now_ns - last_ns < self._interval_ns:
                self._skipped_count += 1
                continue

            # Republish as BestEffort TFMessage on global /tf.
            transform.header.stamp = self.get_clock().now().to_msg()
            self._tf_pub.publish(TFMessage(transforms=[transform]))
            self._last_publish_ns[child] = now_ns
            self._sent_hashes[dedup_key] = (child, now_ns)
            self._tx_count += 1

    def _on_tf_static(self, msg: TFMessage):
        """Forward static transforms at a low rate (once per 2 s per frame).
        Uses StaticTransformBroadcaster so /tf_static stays Reliable."""
        now_ns = self.get_clock().now().nanoseconds
        static_interval_ns = 2_000_000_000  # 2 s

        for transform in msg.transforms:
            child = transform.child_frame_id
            if self._child_frames and child not in self._child_frames:
                continue

            content_hash = self._hash_transform(transform)
            dedup_key = f"static:{child}:{content_hash}"
            if dedup_key in self._sent_hashes:
                self._dedup_count += 1
                continue

            last_ns = self._last_publish_ns.get(f"static:{child}", 0)
            if now_ns - last_ns < static_interval_ns:
                continue

            transform.header.stamp = self.get_clock().now().to_msg()
            self._tf_static_broadcaster.sendTransform(transform)
            self._last_publish_ns[f"static:{child}"] = now_ns
            self._sent_hashes[dedup_key] = (child, now_ns)

    # -- Helpers ---------------------------------------------------------

    @staticmethod
    def _hash_transform(transform) -> str:
        """Compute a stable short hash of a TransformStamped payload."""
        t = transform.transform.translation
        r = transform.transform.rotation
        raw = (
            f"{t.x:.6f}|{t.y:.6f}|{t.z:.6f}|"
            f"{r.x:.6f}|{r.y:.6f}|{r.z:.6f}|{r.w:.6f}"
        )
        return hashlib.md5(raw.encode()).hexdigest()[:12]

    def _maybe_purge_dedup(self):
        """Periodically purge the dedup cache to bound memory."""
        now = time.monotonic()
        if now - self._dedup_last_purge < self._dedup_purge_interval_s:
            return
        # Keep only entries from the last 30 seconds.
        cutoff_ns = self.get_clock().now().nanoseconds - 30_000_000_000
        stale = [
            k for k, (_, ts) in self._sent_hashes.items() if ts < cutoff_ns
        ]
        for k in stale:
            del self._sent_hashes[k]
        self._dedup_last_purge = now

    def _log_stats(self):
        self.get_logger().info(
            f"TF throttle stats -- rx={self._rx_count} tx={self._tx_count} "
            f"skipped={self._skipped_count} dedup={self._dedup_count} "
            f"(max_hz={self._max_hz:.0f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TFThrottleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
