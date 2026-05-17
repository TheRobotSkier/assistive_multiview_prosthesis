#!/usr/bin/env python3
"""Publish fallback world->camera_link TFs while marker tracking is unavailable."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class CameraTfFallbackNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_tf_fallback")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("head_frame", "d435i_head_link")
        self.declare_parameter("arm_frame", "d435i_arm_link")
        self.declare_parameter("head_marker_valid_topic", "/head/marker_pose/marker_valid")
        self.declare_parameter("arm_marker_valid_topic", "/arm/marker_pose/marker_valid")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("marker_dropout_grace_s", 2.0)
        self.declare_parameter("head_xyz_rpy", [0.0, -0.20, 0.65, 0.0, 0.0, 0.0])
        self.declare_parameter("arm_xyz_rpy", [0.35, 0.20, 0.35, 0.0, 0.0, 0.0])

        self._world = self.get_parameter("world_frame").value
        self._head_frame = self.get_parameter("head_frame").value
        self._arm_frame = self.get_parameter("arm_frame").value
        self._head_pose = [float(v) for v in self.get_parameter("head_xyz_rpy").value]
        self._arm_pose = [float(v) for v in self.get_parameter("arm_xyz_rpy").value]

        self._marker_valid = {
            self._head_frame: False,
            self._arm_frame: False,
        }
        self._invalid_since = {
            self._head_frame: None,
            self._arm_frame: None,
        }
        self._marker_dropout_grace_s = float(self.get_parameter("marker_dropout_grace_s").value)

        self._tf_pub = TransformBroadcaster(self)
        self.create_subscription(
            Bool,
            self.get_parameter("head_marker_valid_topic").value,
            lambda msg: self._set_marker_valid(self._head_frame, msg),
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("arm_marker_valid_topic").value,
            lambda msg: self._set_marker_valid(self._arm_frame, msg),
            10,
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            "Camera TF fallback active until marker tracking is valid: "
            f"{self._world}->{self._head_frame}, {self._world}->{self._arm_frame}"
        )

    def _set_marker_valid(self, frame: str, msg: Bool) -> None:
        was_valid = self._marker_valid[frame]
        is_valid = bool(msg.data)
        self._marker_valid[frame] = is_valid
        self._invalid_since[frame] = None if is_valid else self.get_clock().now()
        if is_valid != was_valid:
            state = "disabled" if is_valid else "enabled"
            if is_valid:
                self.get_logger().info(f"Fallback TF {state} for {frame}")
            else:
                self.get_logger().info(
                    f"Fallback TF pending for {frame}; grace={self._marker_dropout_grace_s:.1f}s"
                )

    def _make_tf(self, child: str, xyz_rpy: list[float]) -> TransformStamped:
        x, y, z, roll, pitch, yaw = xyz_rpy
        qx, qy, qz, qw = _quat_from_rpy(roll, pitch, yaw)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world
        msg.child_frame_id = child
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = z
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg

    def _publish(self) -> None:
        transforms = []
        if self._should_publish_fallback(self._head_frame):
            transforms.append(self._make_tf(self._head_frame, self._head_pose))
        if self._should_publish_fallback(self._arm_frame):
            transforms.append(self._make_tf(self._arm_frame, self._arm_pose))
        if transforms:
            self._tf_pub.sendTransform(transforms)

    def _should_publish_fallback(self, frame: str) -> bool:
        if self._marker_valid[frame]:
            return False
        invalid_since = self._invalid_since[frame]
        if invalid_since is None:
            return True
        age_s = (self.get_clock().now() - invalid_since).nanoseconds / 1e9
        return age_s > self._marker_dropout_grace_s


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraTfFallbackNode()
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
