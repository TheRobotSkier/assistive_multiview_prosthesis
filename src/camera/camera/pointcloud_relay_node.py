"""Pointcloud relay node.

Subscribes to /fused_pointcloud and republishes on /segmentation/input_cloud.
This bridges the real multiview camera output to the segmentation node.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointcloudRelayNode(Node):
    def __init__(self):
        super().__init__('pointcloud_relay')
        self.pub = self.create_publisher(PointCloud2, '/segmentation/input_cloud', 10)
        self.create_subscription(PointCloud2, '/fused_pointcloud', self._cb, 10)
        self.get_logger().info('Relaying /fused_pointcloud -> /segmentation/input_cloud')

    def _cb(self, msg: PointCloud2):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudRelayNode()
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
