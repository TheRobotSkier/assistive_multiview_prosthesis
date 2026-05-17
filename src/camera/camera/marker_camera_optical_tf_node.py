#!/usr/bin/env python3
"""Publish camera optical-frame TFs from marker camera_pose_raw topics."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster


class MarkerCameraOpticalTfNode(Node):
    def __init__(self):
        super().__init__("marker_camera_optical_tf")
        self.declare_parameter("head_pose_topic", "/head/marker_pose/camera_pose_raw")
        self.declare_parameter("arm_pose_topic", "/arm/marker_pose/camera_pose_raw")
        self.declare_parameter("head_color_frame", "d435i_head_color_optical_frame")
        self.declare_parameter("head_depth_frame", "d435i_head_depth_optical_frame")
        self.declare_parameter("arm_color_frame", "d435i_arm_color_optical_frame")
        self.declare_parameter("arm_depth_frame", "d435i_arm_depth_optical_frame")
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("max_cached_tf_age_s", 2.0)

        self._tf_pub = TransformBroadcaster(self)
        self._head_color = self.get_parameter("head_color_frame").value
        self._head_depth = self.get_parameter("head_depth_frame").value
        self._arm_color = self.get_parameter("arm_color_frame").value
        self._arm_depth = self.get_parameter("arm_depth_frame").value
        self._max_cached_tf_age_s = float(self.get_parameter("max_cached_tf_age_s").value)
        self._latest: dict[tuple[str, str], tuple[PoseStamped, Time]] = {}
        self._expired_logged: set[tuple[str, str]] = set()

        self.create_subscription(
            PoseStamped,
            self.get_parameter("head_pose_topic").value,
            lambda msg: self._store(msg, self._head_color, self._head_depth),
            20,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("arm_pose_topic").value,
            lambda msg: self._store(msg, self._arm_color, self._arm_depth),
            20,
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self._publish_latest)
        self.get_logger().info(
            "Marker camera optical TF bridge active: "
            f"{self._head_color}/{self._head_depth}, {self._arm_color}/{self._arm_depth}; "
            f"max_cached_tf_age_s={self._max_cached_tf_age_s:.1f}"
        )

    def _store(self, msg: PoseStamped, color_frame: str, depth_frame: str) -> None:
        if not msg.header.frame_id:
            return
        frames = (color_frame, depth_frame)
        self._latest[frames] = (msg, self.get_clock().now())
        self._expired_logged.discard(frames)

    def _publish_latest(self) -> None:
        if not self._latest:
            return
        transforms = []
        now = self.get_clock().now().to_msg()
        for frames, (msg, received_time) in self._latest.items():
            age_s = (self.get_clock().now() - received_time).nanoseconds / 1e9
            if age_s > self._max_cached_tf_age_s:
                if frames not in self._expired_logged:
                    self.get_logger().warn(
                        f"Marker camera TF cache expired for {frames}; "
                        f"last update age={age_s:.2f}s"
                    )
                    self._expired_logged.add(frames)
                continue
            for child in frames:
                tf = TransformStamped()
                tf.header.frame_id = msg.header.frame_id
                tf.header.stamp = now
                tf.child_frame_id = child
                tf.transform.translation.x = msg.pose.position.x
                tf.transform.translation.y = msg.pose.position.y
                tf.transform.translation.z = msg.pose.position.z
                tf.transform.rotation = msg.pose.orientation
                transforms.append(tf)
        self._tf_pub.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerCameraOpticalTfNode()
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
