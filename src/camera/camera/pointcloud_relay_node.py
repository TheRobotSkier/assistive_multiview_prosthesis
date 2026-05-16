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
        self.declare_parameter('input_topic', '/fused_pointcloud')
        self.declare_parameter('output_topic', '/segmentation/input_cloud')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.create_subscription(PointCloud2, input_topic, self._cb, 10)
        self.get_logger().info(f'Relaying {input_topic} -> {output_topic}')

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
