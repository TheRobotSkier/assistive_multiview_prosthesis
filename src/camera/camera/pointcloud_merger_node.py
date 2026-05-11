#!/usr/bin/env python3
"""Pointcloud merger — transforms cam2 cloud into cam1 frame and fuses them.

Strategy:
  1. Subscribe to both camera pointclouds
  2. Try to transform cam2's cloud into cam1's frame via TF
  3. If TF available: properly merge both clouds by raw byte concatenation
  4. If TF NOT available: fall back to cam1-only passthrough (stable, no glitch)

Publishes:
  /fused_pointcloud (sensor_msgs/PointCloud2)
"""

import rclpy
import numpy as np
import tf2_ros
import tf2_sensor_msgs  # noqa: F401 — registers do_transform_cloud
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointcloudMerger(Node):
    """Transforms cam2 cloud into cam1 frame and merges both into one cloud."""

    def __init__(self):
        super().__init__("pointcloud_merger")

        self.declare_parameter("cam1_topic", "/cam1/d435_1/depth/color/points")
        self.declare_parameter("cam2_topic", "/cam2/d435_2/depth/color/points")
        self.declare_parameter("output_topic", "/fused_pointcloud")
        self.declare_parameter("target_frame", "cam1camera_depth_optical_frame")

        cam1_topic = self.get_parameter("cam1_topic").value
        cam2_topic = self.get_parameter("cam2_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._target_frame = self.get_parameter("target_frame").value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(PointCloud2, output_topic, 5)

        self._cam1_cloud: PointCloud2 | None = None
        self._cam2_cloud: PointCloud2 | None = None

        self.create_subscription(PointCloud2, cam1_topic, self._cb_cam1, 10)
        self.create_subscription(PointCloud2, cam2_topic, self._cb_cam2, 10)
        self.create_timer(1.0 / 15.0, self._merge_and_publish)

        self._stats = {"cam1": 0, "cam2": 0, "merged": 0, "fallback": 0}
        self.create_timer(10.0, self._log_stats)
        self._tf_ok = False

        self.get_logger().info(
            f"Merger: {cam1_topic} + {cam2_topic} -> {output_topic} "
            f"(target_frame={self._target_frame})"
        )

    def _cb_cam1(self, msg: PointCloud2):
        self._cam1_cloud = msg
        self._stats["cam1"] += 1

    def _cb_cam2(self, msg: PointCloud2):
        self._cam2_cloud = msg
        self._stats["cam2"] += 1

    def _merge_and_publish(self):
        cloud1 = self._cam1_cloud
        cloud2 = self._cam2_cloud

        if cloud1 is None and cloud2 is None:
            return

        # ── Try proper merge (TF-transform cam2 into cam1 frame) ─────────
        all_clouds: list[PointCloud2] = []
        if cloud1 is not None:
            all_clouds.append(cloud1)

        if cloud2 is not None:
            try:
                # Use can_transform + small wait for robustness
                if self._tf_buffer.can_transform(
                    self._target_frame, cloud2.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                ):
                    transform = self._tf_buffer.lookup_transform(
                        self._target_frame,
                        cloud2.header.frame_id,
                        rclpy.time.Time(),
                    )
                    transformed = tf2_sensor_msgs.do_transform_cloud(cloud2, transform)
                    all_clouds.append(transformed)
            except Exception:
                pass  # TF unavailable — fallback

        if len(all_clouds) >= 2:
            merged = self._concat_clouds(all_clouds)
            if merged is not None:
                merged.header.frame_id = self._target_frame
                self._pub.publish(merged)
                self._stats["merged"] += 1
                if not self._tf_ok:
                    self._tf_ok = True
                    self.get_logger().info("TF chain connected — proper merge active")
                return

        # ── Fallback: cam1-only passthrough ──────────────────────────────
        self._stats["fallback"] += 1
        if self._tf_ok:
            self._tf_ok = False
            self.get_logger().warn(
                "TF chain lost — falling back to cam1-only passthrough",
                throttle_duration_sec=5.0,
            )
        if cloud1 is not None:
            self._pub.publish(cloud1)

    def _concat_clouds(self, clouds: list[PointCloud2]) -> PointCloud2 | None:
        """Concatenate raw byte data from multiple PointCloud2 messages."""
        if len(clouds) == 1:
            return clouds[0]

        # Verify field compatibility
        ref = [(f.name, f.datatype, f.count) for f in clouds[0].fields]
        for i, c in enumerate(clouds[1:], 1):
            chk = [(f.name, f.datatype, f.count) for f in c.fields]
            if chk != ref:
                self.get_logger().warn(
                    f"Cloud {i} fields mismatch — using first cloud only",
                    throttle_duration_sec=10.0,
                )
                return clouds[0]

        total_points = sum(c.width * c.height for c in clouds)
        raw = bytearray()
        for c in clouds:
            raw.extend(c.data)

        out = PointCloud2()
        out.header = clouds[0].header
        out.header.frame_id = self._target_frame
        out.height = 1
        out.width = total_points
        out.fields = clouds[0].fields
        out.is_bigendian = clouds[0].is_bigendian
        out.point_step = clouds[0].point_step
        out.data = bytes(raw)
        out.row_step = len(out.data)
        return out

    def _log_stats(self):
        self.get_logger().info(
            f"cam1: {self._stats['cam1']} | cam2: {self._stats['cam2']} | "
            f"merged: {self._stats['merged']} | fallback: {self._stats['fallback']}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudMerger()
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
