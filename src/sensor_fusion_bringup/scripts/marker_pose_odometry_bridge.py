#!/usr/bin/env python3
"""Bridge ArUco marker pose into OpenVINS-compatible odometry and TF.

This is a runtime fallback for bench integration when the OpenVINS marker
estimator binary is not present in the Jetson image. It keeps downstream
interfaces identical: /ov_msckf_*/odomimu plus marker_map->camera_link TF.
"""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster


class MarkerPoseOdometryBridge(Node):
    def __init__(self):
        super().__init__("marker_pose_odometry_bridge")
        self.declare_parameter("input_pose_topic", "/arm/marker_pose/imu_pose")
        self.declare_parameter("output_odom_topic", "/ov_msckf_arm/odomimu")
        self.declare_parameter("odom_child_frame", "arm_imu")
        self.declare_parameter("tf_child_frame", "arm_d435i_arm_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("max_cached_tf_age_s", 2.0)

        self._odom_child = self.get_parameter("odom_child_frame").value
        self._tf_child = self.get_parameter("tf_child_frame").value
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        input_topic = self.get_parameter("input_pose_topic").value
        output_topic = self.get_parameter("output_odom_topic").value

        self._pub = self.create_publisher(Odometry, output_topic, 20)
        self._tf_pub = TransformBroadcaster(self)
        self._latest_tf: TransformStamped | None = None
        self._latest_tf_time: Time | None = None
        self._max_cached_tf_age_s = float(self.get_parameter("max_cached_tf_age_s").value)
        self._cache_expired_logged = False
        self.create_subscription(PoseWithCovarianceStamped, input_topic, self._cb, 20)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / publish_rate, self._publish_latest_tf)
        self.get_logger().info(
            f"Marker odom bridge: {input_topic} -> {output_topic}, TF child={self._tf_child}, "
            f"max_cached_tf_age_s={self._max_cached_tf_age_s:.1f}"
        )

    def _cb(self, msg: PoseWithCovarianceStamped):
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = self._odom_child
        odom.pose = msg.pose
        odom.twist.covariance[0] = 10.0
        odom.twist.covariance[7] = 10.0
        odom.twist.covariance[14] = 10.0
        odom.twist.covariance[21] = 10.0
        odom.twist.covariance[28] = 10.0
        odom.twist.covariance[35] = 10.0
        self._pub.publish(odom)

        if not self._publish_tf:
            return
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = self._tf_child
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self._latest_tf = tf
        self._latest_tf_time = self.get_clock().now()
        self._cache_expired_logged = False

    def _publish_latest_tf(self):
        if not self._publish_tf or self._latest_tf is None:
            return
        if self._latest_tf_time is not None:
            age_s = (self.get_clock().now() - self._latest_tf_time).nanoseconds / 1e9
            if age_s > self._max_cached_tf_age_s:
                if not self._cache_expired_logged:
                    self.get_logger().warn(
                        f"Marker odom TF cache expired for {self._tf_child}; "
                        f"last update age={age_s:.2f}s"
                    )
                    self._cache_expired_logged = True
                return
        tf = TransformStamped()
        tf.header = self._latest_tf.header
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.child_frame_id = self._latest_tf.child_frame_id
        tf.transform = self._latest_tf.transform
        self._tf_pub.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPoseOdometryBridge()
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
