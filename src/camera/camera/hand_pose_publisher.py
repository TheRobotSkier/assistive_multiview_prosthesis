#!/usr/bin/env python3
"""Hand Pose TF Publisher — reads wrist_link TF and publishes /hand_pose.

Listens to the TF tree and continuously publishes the transform from
'world' to 'wrist_link' as a geometry_msgs/PoseStamped on /hand_pose.

Publishes:
    /hand_pose  (geometry_msgs/PoseStamped) — wrist_link pose in world frame
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_ros


class HandPosePublisher(Node):
    """Publishes the wrist_link TF transform as a pose at 50 Hz."""

    def __init__(self):
        super().__init__("hand_pose_publisher")

        # ── TF setup ─────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Publisher (latched so late subscribers always get the pose) ──
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(PoseStamped, "/hand_pose", latched)

        # ── Timer: 50 Hz continuous TF lookup ────────────────────────────
        self.create_timer(1.0 / 50.0, self._lookup_and_publish)

        self.get_logger().info("Hand pose publisher started — 50 Hz TF → /hand_pose")

    def _lookup_and_publish(self) -> None:
        """Lookup world→wrist_link transform and publish as PoseStamped."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "world",         # target frame
                "wrist_link",    # source frame
                rclpy.time.Time(),  # latest available
            )
        except tf2_ros.LookupException as e:
            self.get_logger().warn(
                f"world → wrist_link TF not available: {e}. "
                f"Ensure robot_state_publisher is running."
            )
            return

        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation

        self._publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = HandPosePublisher()
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
