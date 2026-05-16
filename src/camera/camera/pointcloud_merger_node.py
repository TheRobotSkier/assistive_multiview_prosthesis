#!/usr/bin/env python3
"""Pointcloud merger — transforms camera clouds into one target frame and fuses them.

Strategy:
  1. Subscribe to both camera pointclouds
  2. Transform each cloud into target_frame via TF
  3. If TF available: merge transformed clouds by raw byte concatenation
  4. If TF NOT available: fall back to cam1-only passthrough

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
    """Transforms input clouds into target_frame and merges them."""

    def __init__(self):
        super().__init__("pointcloud_merger")

        self.declare_parameter("cam1_topic", "/cam1/d435_1/depth/color/points")
        self.declare_parameter("cam2_topic", "/cam2/d435_2/depth/color/points")
        self.declare_parameter("output_topic", "/fused_pointcloud")
        self.declare_parameter("target_frame", "cam1camera_depth_optical_frame")
        self.declare_parameter("use_cloud_timestamps", True)
        self.declare_parameter("tf_timeout_s", 0.1)

        cam1_topic = self.get_parameter("cam1_topic").value
        cam2_topic = self.get_parameter("cam2_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._target_frame = self.get_parameter("target_frame").value
        self._use_cloud_timestamps = bool(self.get_parameter("use_cloud_timestamps").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_s").value)

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

        all_clouds: list[PointCloud2] = []
        if cloud1 is not None:
            transformed = self._transform_to_target(cloud1)
            if transformed is not None:
                all_clouds.append(transformed)

        if cloud2 is not None:
            transformed = self._transform_to_target(cloud2)
            if transformed is not None:
                all_clouds.append(transformed)

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

    def _transform_to_target(self, cloud: PointCloud2) -> PointCloud2 | None:
        if cloud.header.frame_id == self._target_frame:
            return cloud
        try:
            stamp = self._lookup_time(cloud)
            timeout = rclpy.duration.Duration(seconds=self._tf_timeout)
            if not self._tf_buffer.can_transform(
                self._target_frame, cloud.header.frame_id, stamp, timeout=timeout
            ):
                return None
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                cloud.header.frame_id,
                stamp,
            )
            transformed = tf2_sensor_msgs.do_transform_cloud(cloud, transform)
            transformed.header.frame_id = self._target_frame
            return transformed
        except Exception:
            return None

    def _lookup_time(self, cloud: PointCloud2) -> rclpy.time.Time:
        if not self._use_cloud_timestamps:
            return rclpy.time.Time()
        stamp = cloud.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return rclpy.time.Time()
        return rclpy.time.Time.from_msg(stamp)

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
