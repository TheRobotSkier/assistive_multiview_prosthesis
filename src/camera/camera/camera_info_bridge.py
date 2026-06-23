#!/usr/bin/env python3
"""Camera-info QoS bridge.

Subscribes to /jetson/{side}/camera_info with BEST_EFFORT QoS (matching
the Jetson relay) and republishes locally with RELIABLE QoS so the host-
side naive_pointcloud_assembler (which subscribes with BEST_EFFORT but
benefits from the RELIABLE bridge for robustness) receives the intrinsics.

This mirrors the decompress_bridge pattern: BEST_EFFORT over the wire,
RELIABLE inside local RAM (zero network overhead).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo

_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class CameraInfoBridge(Node):
    """Relay CameraInfo from BEST_EFFORT to RELIABLE QoS."""

    def __init__(self) -> None:
        super().__init__("camera_info_bridge")

        self.declare_parameter("sub_topic", "")
        self.declare_parameter("pub_topic", "")

        sub_topic = self.get_parameter("sub_topic").value
        pub_topic = self.get_parameter("pub_topic").value

        if not sub_topic or not pub_topic:
            self.get_logger().error(
                f"sub_topic and pub_topic must be set (got "
                f"sub={sub_topic!r}, pub={pub_topic!r})"
            )
            raise SystemExit(1)

        self._pub = self.create_publisher(CameraInfo, pub_topic, _RELIABLE_QOS)
        self._sub = self.create_subscription(
            CameraInfo,
            sub_topic,
            self._on_info,
            _BEST_EFFORT_QOS,
        )
        self.get_logger().info(f"CameraInfoBridge: {{{sub_topic}}} -> {{{pub_topic}}}")

    def _on_info(self, msg: CameraInfo) -> None:
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    try:
        node = CameraInfoBridge()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
