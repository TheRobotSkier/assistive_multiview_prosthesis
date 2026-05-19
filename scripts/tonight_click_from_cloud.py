#!/usr/bin/env python3
"""Publish one segmentation click from a live PointCloud2 sample.

This is intentionally small and operational: it lets Makefile validation
targets test the fused-cloud -> segmentation path without RViz interaction.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class ClickFromCloud(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("tonight_click_from_cloud")
        self._args = args
        self._cloud: PointCloud2 | None = None
        self._object_cloud: PointCloud2 | None = None
        self._click_pub = self.create_publisher(PointStamped, args.click_topic, 10)
        self.create_subscription(PointCloud2, args.cloud_topic, self._cloud_cb, 1)
        self.create_subscription(PointCloud2, args.object_topic, self._object_cb, 1)

    def _cloud_cb(self, msg: PointCloud2):
        self._cloud = msg

    def _object_cb(self, msg: PointCloud2):
        self._object_cloud = msg

    def wait_for_cloud(self) -> PointCloud2:
        deadline = time.monotonic() + self._args.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._cloud is not None:
                return self._cloud
        raise TimeoutError(f"timed out waiting for {self._args.cloud_topic}")

    def pick_point(self, cloud: PointCloud2) -> tuple[float, float, float]:
        start_index = max(self._args.sample_index, 0)
        for index, point in enumerate(
            point_cloud2.read_points(
                cloud,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        ):
            if index < start_index:
                continue
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                return x, y, z
        raise RuntimeError(f"no finite XYZ point found in {self._args.cloud_topic}")

    def publish_click(self, cloud: PointCloud2, point: tuple[float, float, float]):
        msg = PointStamped()
        msg.header = cloud.header
        msg.point.x, msg.point.y, msg.point.z = point

        # Publish a few times so late-matching subscribers see the click.
        for _ in range(self._args.repeat):
            self._click_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(
            "published click to %s: frame=%s xyz=(%.3f, %.3f, %.3f)"
            % (
                self._args.click_topic,
                msg.header.frame_id,
                msg.point.x,
                msg.point.y,
                msg.point.z,
            )
        )

    def wait_for_object_cloud(self) -> PointCloud2:
        deadline = time.monotonic() + self._args.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            cloud = self._object_cloud
            if cloud is not None and cloud.width * cloud.height > 0:
                return cloud
        raise TimeoutError(f"timed out waiting for {self._args.object_topic}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cloud-topic", default="/fused_pointcloud")
    parser.add_argument("--click-topic", default="/segmentation/click_positive")
    parser.add_argument("--object-topic", default="/segmentation/object_cloud")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--sample-index", type=int, default=2000)
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--no-wait-object", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = ClickFromCloud(args)
    try:
        cloud = node.wait_for_cloud()
        point = node.pick_point(cloud)
        node.publish_click(cloud, point)
        if not args.no_wait_object:
            object_cloud = node.wait_for_object_cloud()
            node.get_logger().info(
                "received %s: frame=%s width=%d height=%d"
                % (
                    args.object_topic,
                    object_cloud.header.frame_id,
                    object_cloud.width,
                    object_cloud.height,
                )
            )
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
