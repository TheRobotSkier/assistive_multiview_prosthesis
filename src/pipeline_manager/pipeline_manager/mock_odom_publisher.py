#!/usr/bin/env python3
"""Mock odometry publisher for testing the GTSAM tracker without OpenVINS.

Publishes synthetic ``nav_msgs/Odometry`` messages on the head and arm odom
topics with a slowly drifting trajectory.  This lets the ``gtsam_tracker``
node be exercised in mock mode.

Publishes:
    /ov_msckf/odomimu       (nav_msgs/Odometry) — head odometry
    /ov_msckf_arm/odomimu   (nav_msgs/Odometry) — arm odometry

Optionally publishes fake GTSAM poses so the keyframe_buffer and
cross_camera_features nodes can be tested without the tracker:
    /gtsam/head_pose        (geometry_msgs/PoseWithCovarianceStamped)
    /gtsam/arm_pose         (geometry_msgs/PoseWithCovarianceStamped)
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header


class MockOdomPublisher(Node):
    def __init__(self):
        super().__init__("mock_odom_publisher")

        self.declare_parameter("publish_hz", 15.0)
        self.declare_parameter("head_odom_topic", "/ov_msckf/odomimu")
        self.declare_parameter("arm_odom_topic", "/ov_msckf_arm/odomimu")
        self.declare_parameter("head_pose_topic", "/gtsam/head_pose")
        self.declare_parameter("arm_pose_topic", "/gtsam/arm_pose")
        self.declare_parameter("publish_gtsam_poses", True)
        self.declare_parameter("world_frame", "marker_map")

        publish_hz = float(self.get_parameter("publish_hz").value)
        self._head_topic = str(self.get_parameter("head_odom_topic").value)
        self._arm_topic = str(self.get_parameter("arm_odom_topic").value)
        self._head_pose_topic = str(self.get_parameter("head_pose_topic").value)
        self._arm_pose_topic = str(self.get_parameter("arm_pose_topic").value)
        self._publish_gtsam = bool(
            self.get_parameter("publish_gtsam_poses").value)
        self._world_frame = str(self.get_parameter("world_frame").value)

        self._head_pub = self.create_publisher(Odometry, self._head_topic, 10)
        self._arm_pub = self.create_publisher(Odometry, self._arm_topic, 10)

        if self._publish_gtsam:
            self._head_pose_pub = self.create_publisher(
                PoseWithCovarianceStamped, self._head_pose_topic, 10)
            self._arm_pose_pub = self.create_publisher(
                PoseWithCovarianceStamped, self._arm_pose_topic, 10)

        self._t = 0.0
        self._dt = 1.0 / publish_hz if publish_hz > 0 else 0.1

        self.timer = self.create_timer(self._dt, self._publish)
        self.get_logger().info(
            f"Mock odom publisher started (hz={publish_hz}, "
            f"head={self._head_topic}, arm={self._arm_topic})")

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        t = self._t

        # Head: slowly oscillating around a fixed point (simulates head
        # looking around).  Position drifts in a small circle.
        head_x = 0.1 * math.sin(t * 0.3)
        head_y = 0.0
        head_z = 0.5 + 0.02 * math.sin(t * 0.5)
        head_yaw = 0.1 * math.sin(t * 0.3)

        # Arm: moves in a larger arc (simulates reaching).
        arm_x = 0.3 + 0.1 * math.sin(t * 0.2)
        arm_y = 0.1 * math.cos(t * 0.2)
        arm_z = 0.3 + 0.05 * math.sin(t * 0.4)
        arm_yaw = 0.3 * math.sin(t * 0.2)

        head_odom = self._build_odom(
            head_x, head_y, head_z, head_yaw, stamp, "marker_map",
            "head_imu")
        arm_odom = self._build_odom(
            arm_x, arm_y, arm_z, arm_yaw, stamp, "marker_map",
            "arm_imu")

        self._head_pub.publish(head_odom)
        self._arm_pub.publish(arm_odom)

        if self._publish_gtsam:
            head_pose = self._build_pose(
                head_x, head_y, head_z, head_yaw, stamp)
            arm_pose = self._build_pose(
                arm_x, arm_y, arm_z, arm_yaw, stamp)
            self._head_pose_pub.publish(head_pose)
            self._arm_pose_pub.publish(arm_pose)

        self._t += self._dt

    def _build_odom(self, x, y, z, yaw, stamp, frame_id, child_frame_id):
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        # Quaternion from yaw
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(sy)
        msg.pose.pose.orientation.w = float(cy)
        # Identity-ish covariance (diagonal)
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = 0.001  # position
        cov[21] = cov[28] = cov[35] = 0.001  # orientation
        msg.pose.covariance = cov
        msg.twist.covariance = [0.0] * 36
        return msg

    def _build_pose(self, x, y, z, yaw, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = self._world_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(sy)
        msg.pose.pose.orientation.w = float(cy)
        msg.pose.covariance = [0.0] * 36
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MockOdomPublisher()
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
