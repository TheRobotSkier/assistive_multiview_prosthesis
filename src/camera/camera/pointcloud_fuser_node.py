"""Pointcloud fuser — subscribes to dual camera pointclouds, republishes to /fused_pointcloud.

Subscribes to individual camera pointcloud topics and republishes each
incoming cloud on /fused_pointcloud. This is the bridge between dual D435
cameras and the digital twin pipeline (which expects a single fused cloud).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointcloudFuser(Node):
    def __init__(self):
        super().__init__("pointcloud_fuser")

        self.declare_parameter("cam1_topic", "/cam1/d435_1/depth/color/points")
        self.declare_parameter("cam2_topic", "/cam2/d435_2/depth/color/points")
        self.declare_parameter("output_topic", "/fused_pointcloud")

        cam1_topic = self.get_parameter("cam1_topic").value
        cam2_topic = self.get_parameter("cam2_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.create_subscription(PointCloud2, cam1_topic, self._cb_cam1, 10)
        self.create_subscription(PointCloud2, cam2_topic, self._cb_cam2, 10)

        self._stats = {"cam1": 0, "cam2": 0}
        self.create_timer(10.0, self._log_stats)
        self.get_logger().info(
            f"Pointcloud fuser started: {cam1_topic} + {cam2_topic} -> {output_topic}"
        )

    def _cb_cam1(self, msg: PointCloud2):
        self._stats["cam1"] += 1
        self._pub.publish(msg)

    def _cb_cam2(self, msg: PointCloud2):
        self._stats["cam2"] += 1
        self._pub.publish(msg)

    def _log_stats(self):
        self.get_logger().info(
            f"Stats — cam1: {self._stats['cam1']} msgs, cam2: {self._stats['cam2']} msgs"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudFuser()
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
