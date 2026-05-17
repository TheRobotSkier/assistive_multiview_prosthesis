#!/usr/bin/env python3
"""Call the grasp planner whenever segmentation publishes a non-empty object cloud."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger


class SegmentedCloudGraspTrigger(Node):
    def __init__(self) -> None:
        super().__init__("segmented_cloud_grasp_trigger")

        self.declare_parameter("cloud_topic", "/segmentation/object_cloud")
        self.declare_parameter("compute_service", "/grasp_preshaping/compute_grasp")
        self.declare_parameter("min_points", 1)
        self.declare_parameter("debounce_s", 1.0)
        self.declare_parameter("service_wait_s", 0.2)

        self._cloud_topic = self.get_parameter("cloud_topic").value
        self._compute_service = self.get_parameter("compute_service").value
        self._min_points = int(self.get_parameter("min_points").value)
        self._debounce_s = float(self.get_parameter("debounce_s").value)
        self._service_wait_s = float(self.get_parameter("service_wait_s").value)
        self._last_trigger_time = None
        self._pending = False

        self._client = self.create_client(Trigger, self._compute_service)
        self.create_subscription(PointCloud2, self._cloud_topic, self._on_cloud, 10)

        self.get_logger().info(
            f"Segmented cloud grasp trigger: {self._cloud_topic} -> {self._compute_service}, "
            f"min_points={self._min_points}, debounce_s={self._debounce_s:.1f}"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        points = int(msg.width) * int(msg.height)
        if points < self._min_points:
            return
        if self._pending:
            return
        now = self.get_clock().now()
        if self._last_trigger_time is not None:
            elapsed_s = (now - self._last_trigger_time).nanoseconds / 1e9
            if elapsed_s < self._debounce_s:
                return

        if not self._client.wait_for_service(timeout_sec=self._service_wait_s):
            self.get_logger().warn(
                f"Grasp service {self._compute_service} unavailable; segmented cloud not processed",
                throttle_duration_sec=2.0,
            )
            return

        self._pending = True
        self._last_trigger_time = now
        future = self._client.call_async(Trigger.Request())
        future.add_done_callback(lambda fut: self._on_response(fut, points))
        self.get_logger().info(f"Triggered grasp preshaping from segmented cloud ({points} points)")

    def _on_response(self, future, points: int) -> None:
        self._pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Grasp service call failed after {points} point cloud: {exc}")
            return

        level = self.get_logger().info if response.success else self.get_logger().warn
        level(f"Grasp service completed: success={response.success}, message='{response.message}'")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SegmentedCloudGraspTrigger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
