"""Click relay — forwards RViz PublishPoint clicks to segmentation seed topics.

Subscribes to /clicked_point (PointStamped from RViz PublishPoint tool)
and publishes to /segmentation/click_positive as PointStamped.

To place a negative seed, publish a PointStamped to /segmentation/click_negative
directly (e.g. via ros2 topic pub).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class ClickRelayNode(Node):
    def __init__(self):
        super().__init__("click_relay")
        self._pub = self.create_publisher(PointStamped, "/segmentation/click_positive", 10)
        self.create_subscription(PointStamped, "/clicked_point", self._cb, 10)
        self.get_logger().info("Click relay started — /clicked_point -> /segmentation/click_positive")

    def _cb(self, msg: PointStamped):
        self._pub.publish(msg)
        self.get_logger().info(f"Relayed click at ({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})")


def main(args=None):
    rclpy.init(args=args)
    node = ClickRelayNode()
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
