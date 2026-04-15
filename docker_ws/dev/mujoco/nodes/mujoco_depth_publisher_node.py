#!/usr/bin/env python3
"""Republish shared MuJoCo depth outputs from internal topics to public topics."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class MujocoDepthPublisher(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_depth_publisher")

        self.declare_parameter("internal_depth_image_topic", "/mujoco/internal/depth/image")
        self.declare_parameter("internal_depth_camera_info_topic", "/mujoco/internal/depth/camera_info")
        self.declare_parameter("internal_depth_camera_pointcloud_topic", "/mujoco/internal/depth/points_camera")
        self.declare_parameter("internal_segmented_pointcloud_topic", "/mujoco/internal/segmented_object_cloud")
        self.declare_parameter("depth_image_topic", "/mujoco/depth/image")
        self.declare_parameter("depth_camera_info_topic", "/mujoco/depth/camera_info")
        self.declare_parameter("depth_camera_pointcloud_topic", "/mujoco/depth/points_camera")
        self.declare_parameter("segmented_pointcloud_topic", "/segmented_object_cloud")

        internal_depth_image_topic = str(self.get_parameter("internal_depth_image_topic").value)
        internal_depth_camera_info_topic = str(self.get_parameter("internal_depth_camera_info_topic").value)
        internal_depth_camera_pointcloud_topic = str(self.get_parameter("internal_depth_camera_pointcloud_topic").value)
        internal_segmented_pointcloud_topic = str(self.get_parameter("internal_segmented_pointcloud_topic").value)

        self.depth_pub = self.create_publisher(
            Image,
            str(self.get_parameter("depth_image_topic").value),
            10,
        )
        self.info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("depth_camera_info_topic").value),
            10,
        )
        self.camera_cloud_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("depth_camera_pointcloud_topic").value),
            10,
        )
        self.world_cloud_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("segmented_pointcloud_topic").value),
            10,
        )

        self.relay_subscriptions = [
            self.create_subscription(Image, internal_depth_image_topic, self.depth_pub.publish, 10),
            self.create_subscription(CameraInfo, internal_depth_camera_info_topic, self.info_pub.publish, 10),
            self.create_subscription(PointCloud2, internal_depth_camera_pointcloud_topic, self.camera_cloud_pub.publish, 10),
            self.create_subscription(PointCloud2, internal_segmented_pointcloud_topic, self.world_cloud_pub.publish, 10),
        ]



def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = MujocoDepthPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
