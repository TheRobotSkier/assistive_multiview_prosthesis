#!/usr/bin/env python3
"""Publish `/fused_pointcloud` from one or two RealSense D435 pointcloud streams.

Primary input:
  /cam1/d435_1/depth/color/points   sensor_msgs/PointCloud2

Optional secondary input:
  /cam2/d435_2/depth/color/points   sensor_msgs/PointCloud2

Behavior:
  * If only the primary camera is active, the node republishes cam1 directly to
    `/fused_pointcloud`.
  * If a recent cam2 cloud is available, the node transforms it into the cam1
    frame and publishes the merged cloud.

TF: requires a transform from cam2's frame to cam1's frame when the secondary
camera is active. The launch file publishes an identity transform until a real
calibration is available.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
import tf2_sensor_msgs
from sensor_msgs_py import point_cloud2 as pc2


_SYNC_SLOP_NS = int(0.1 * 1e9)


def _stamp_to_ns(msg: PointCloud2) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class PointcloudFusionNode(Node):
    def __init__(self):
        super().__init__('pointcloud_fusion')

        self._tf_buf = Buffer()
        TransformListener(self._tf_buf, self)

        self._pub = self.create_publisher(PointCloud2, '/fused_pointcloud', 1)
        self._latest_cam2 = None

        self.create_subscription(
            PointCloud2,
            '/cam1/d435_1/depth/color/points',
            self._on_cam1_cloud,
            1,
        )
        self.create_subscription(
            PointCloud2,
            '/cam2/d435_2/depth/color/points',
            self._on_cam2_cloud,
            1,
        )

        self.get_logger().info(
            'Pointcloud fusion node ready. Publishing cam1 directly unless a recent cam2 cloud is available.'
        )

    def _on_cam2_cloud(self, cloud: PointCloud2):
        self._latest_cam2 = cloud

    def _on_cam1_cloud(self, cloud1: PointCloud2):
        cloud2 = self._latest_cam2
        if cloud2 is None:
            self._pub.publish(cloud1)
            return

        if abs(_stamp_to_ns(cloud1) - _stamp_to_ns(cloud2)) > _SYNC_SLOP_NS:
            self._pub.publish(cloud1)
            return

        target_frame = cloud1.header.frame_id
        try:
            transform = self._tf_buf.lookup_transform(
                target_frame,
                cloud2.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            cloud2_transformed = tf2_sensor_msgs.do_transform_cloud(cloud2, transform)
        except Exception as exc:
            self.get_logger().warn(
                f'TF lookup failed ({target_frame} ← {cloud2.header.frame_id}): {exc}. Republishing cam1 only.',
                throttle_duration_sec=5.0,
            )
            self._pub.publish(cloud1)
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

