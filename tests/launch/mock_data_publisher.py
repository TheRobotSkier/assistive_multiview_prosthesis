#!/usr/bin/env python3
"""Mock data publisher for smoke-testing the trajectory grasp pipeline.

Publishes:
  /hand_pose                 geometry_msgs/PoseStamped
  /camera/depth/color/points sensor_msgs/PointCloud2
  /segmentation/object_cloud sensor_msgs/PointCloud2
  /emg/gesture_label         std_msgs/Int32
  /emg/confidence            std_msgs/Float32

This allows the pipeline_manager and twist_propagation nodes to run through
their state machines without real hardware.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, Int32
import numpy as np


class MockDataPublisher(Node):
    def __init__(self):
        super().__init__('mock_data_publisher')

        self._hand_pose_pub = self.create_publisher(PoseStamped, '/hand_pose', 10)
        self._cloud_pub = self.create_publisher(PointCloud2, '/camera/depth/color/points', 10)
        self._seg_cloud_pub = self.create_publisher(PointCloud2, '/segmentation/object_cloud', 10)
        self._emg_gesture_pub = self.create_publisher(Int32, '/emg/gesture_label', 10)
        self._emg_confidence_pub = self.create_publisher(Float32, '/emg/confidence', 10)

        self._timer = self.create_timer(0.1, self._publish_all)
        self._step = 0
        self.get_logger().info('Mock data publisher started')

    def _build_cloud(self, frame_id: str, n_points: int = 100) -> PointCloud2:
        xyz = np.random.randn(n_points, 3).astype(np.float32) * 0.1
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = n_points
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * n_points
        msg.is_dense = True
        msg.data = np.ascontiguousarray(xyz).tobytes()
        return msg

    def _publish_all(self):
        now = self.get_clock().now().to_msg()

        # Publish moving hand pose
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'world'
        pose.pose.position.x = 0.1 * np.sin(self._step * 0.05)
        pose.pose.position.y = 0.1 * np.cos(self._step * 0.05)
        pose.pose.position.z = 0.2
        pose.pose.orientation.w = 1.0
        self._hand_pose_pub.publish(pose)

        # Publish scene cloud
        cloud = self._build_cloud('world', n_points=200)
        self._cloud_pub.publish(cloud)

        # Publish EMG gesture trigger every ~5 s
        if self._step % 50 == 0:
            self._emg_gesture_pub.publish(Int32(data=1))  # GESTURE_POWER
            self._emg_confidence_pub.publish(Float32(data=0.8))

        # Publish segmented object cloud shortly after gesture
        if self._step % 50 == 5:
            seg_cloud = self._build_cloud('world', n_points=50)
            self._seg_cloud_pub.publish(seg_cloud)

        self._step += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
