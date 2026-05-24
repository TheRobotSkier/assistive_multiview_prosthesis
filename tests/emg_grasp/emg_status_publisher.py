#!/usr/bin/env python3
"""Minimal system status publisher for the EMG grasp test runtime.

Publishes a consolidated system status on /emg/system_status for external
monitoring (e.g., the Makefile rule or a dashboard).

Replace or extend this node once the full status infrastructure is ready.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EmgStatusPublisher(Node):
    def __init__(self):
        super().__init__("emg_status_publisher")
        self._pub = self.create_publisher(String, "/emg/system_status", 10)
        self._timer = self.create_timer(2.0, self._publish_status)
        self.get_logger().info("EMG status publisher started (2 Hz)")

    def _publish_status(self):
        msg = String()
        msg.data = "EMG_GRASP_RUNNING"
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmgStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
