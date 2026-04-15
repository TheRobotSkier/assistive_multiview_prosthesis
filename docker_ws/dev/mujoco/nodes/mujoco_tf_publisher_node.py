#!/usr/bin/env python3
"""Broadcast TF from shared MuJoCo scene-state messages."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


class MujocoTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_tf_publisher")

        self.declare_parameter("scene_tf_topic", "/mujoco/internal/scene_transforms")
        topic = str(self.get_parameter("scene_tf_topic").value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.scene_tf_sub = self.create_subscription(
            TFMessage,
            topic,
            self.on_scene_tf,
            10,
        )

    def on_scene_tf(self, msg: TFMessage) -> None:
        if msg.transforms:
            self.tf_broadcaster.sendTransform(list(msg.transforms))


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = MujocoTfPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
