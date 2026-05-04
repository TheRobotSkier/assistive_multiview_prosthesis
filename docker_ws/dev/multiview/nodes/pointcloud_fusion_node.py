#!/usr/bin/env python3
"""Fuse two RealSense D435 pointclouds into /fused_pointcloud.

Subscribes (approximate time sync, tolerance 100 ms):
  /cam1/d435_1/depth/color/points   sensor_msgs/PointCloud2
  /cam2/d435_2/depth/color/points   sensor_msgs/PointCloud2

Publishes:
  /fused_pointcloud                  sensor_msgs/PointCloud2  (in cam1 frame)

TF: requires a transform from cam2's frame to cam1's frame.
    Publish a static TF (see multiview_full_launch.py) with calibrated values.
    An identity transform is used until calibration is available.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import message_filters
from tf2_ros import Buffer, TransformListener
import tf2_sensor_msgs
from sensor_msgs_py import point_cloud2 as pc2


class PointcloudFusionNode(Node):
    def __init__(self):
        super().__init__('pointcloud_fusion')

        self._tf_buf = Buffer()
        TransformListener(self._tf_buf, self)

        self._pub = self.create_publisher(PointCloud2, '/fused_pointcloud', 1)

        cam1_sub = message_filters.Subscriber(
            self, PointCloud2, '/cam1/d435_1/depth/color/points')
        cam2_sub = message_filters.Subscriber(
            self, PointCloud2, '/cam2/d435_2/depth/color/points')

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [cam1_sub, cam2_sub], queue_size=5, slop=0.1)
        self._sync.registerCallback(self._on_clouds)

        self.get_logger().info('Pointcloud fusion node ready.')

    def _on_clouds(self, cloud1: PointCloud2, cloud2: PointCloud2):
        target_frame = cloud1.header.frame_id
        try:
            t = self._tf_buf.lookup_transform(
                target_frame,
                cloud2.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            cloud2_transformed = tf2_sensor_msgs.do_transform_cloud(cloud2, t)
        except Exception as e:
            self.get_logger().warn(
                f'TF lookup failed ({target_frame} ← {cloud2.header.frame_id}): {e}',
                throttle_duration_sec=5.0,
            )
            return

        pts1 = list(pc2.read_points(cloud1, skip_nans=True))
        pts2 = list(pc2.read_points(cloud2_transformed, skip_nans=True))
        merged = pts1 + pts2

        out = pc2.create_cloud(cloud1.header, cloud1.fields, merged)
        self._pub.publish(out)


def main():
    rclpy.init()
    node = PointcloudFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
