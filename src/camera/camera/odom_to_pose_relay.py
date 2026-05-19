#!/usr/bin/env python3
"""Odometry to PoseStamped relay.

Subscribes to a nav_msgs/Odometry topic and republishes the pose portion
as geometry_msgs/PoseStamped.  Used to feed twist propagation's /hand_pose
input from an OpenVINS odometry stream.

The odometry twist and covariance are also forwarded to /hand_twist and
/hand_odom respectively so that twist propagation can use them for
covariance estimation (via its odom_topic parameter).

Parameters:
    odom_topic  (str)  – input Odometry topic (default: /ov_msckf_arm/odomimu)
    pose_topic  (str)  – output PoseStamped topic (default: /hand_pose)
    twist_topic (str)  – output TwistStamped topic (default: /hand_twist)
    odom_out     (str)  – forwarded Odometry topic (default: /hand_odom)

Publishes:
    /hand_pose   (PoseStamped)  – pose from odometry
    /hand_twist  (TwistStamped) – twist from odometry
    /hand_odom   (Odometry)     – full odometry (for covariance)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry


class OdomToPoseRelay(Node):

    def __init__(self):
        super().__init__("odom_to_pose_relay")

        self.declare_parameter("odom_topic", "/ov_msckf_arm/odomimu")
        self.declare_parameter("pose_topic", "/hand_pose")
        self.declare_parameter("twist_topic", "/hand_twist")
        self.declare_parameter("odom_out", "/hand_odom")

        odom_topic = self.get_parameter("odom_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        twist_topic = self.get_parameter("twist_topic").value
        odom_out = self.get_parameter("odom_out").value

        # BEST_EFFORT — OpenVINS publishes odom with BEST_EFFORT QoS
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self._twist_pub = self.create_publisher(TwistStamped, twist_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, odom_out, 10)

        self.create_subscription(Odometry, odom_topic, self._on_odom, best_effort)

        self._count = 0
        self.get_logger().info(
            f"Relaying {odom_topic} -> {pose_topic} + {twist_topic} + {odom_out}"
        )

    def _on_odom(self, msg: Odometry):
        # PoseStamped from odometry pose
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self._pose_pub.publish(ps)

        # TwistStamped from odometry twist
        ts = TwistStamped()
        ts.header = msg.header
        ts.twist = msg.twist.twist
        self._twist_pub.publish(ts)

        # Forward full odometry (for covariance)
        self._odom_pub.publish(msg)


        self._count += 1
        if self._count % 100 == 1:
            p = msg.pose.pose.position
            self.get_logger().info(
                f"Relay #{self._count}: pose=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPoseRelay()
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
