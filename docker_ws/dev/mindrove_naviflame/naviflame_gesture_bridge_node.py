#!/usr/bin/env python3
"""TCP socket server that bridges NaviFlame gesture predictions onto the ROS 2 network.

NaviFlame connects to localhost:<port> as a TCP client and sends one gesture ID per
prediction as a 4-byte little-endian integer (same protocol used for the Unity visualiser).
This node listens on that port and republishes each received integer as
std_msgs/Int32 on /naviflame/gesture.

Start this node BEFORE the naviflame container so NaviFlame's first connection
attempt succeeds. NaviFlame retries every 40 s on connection refusal.
"""

from __future__ import annotations

import socket
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class NaviflameGestureBridge(Node):
    def __init__(self) -> None:
        super().__init__("naviflame_gesture_bridge")

        self.declare_parameter("port", 8052)
        self.declare_parameter("gesture_topic", "/naviflame/gesture")

        port: int = int(self.get_parameter("port").value)
        topic: str = str(self.get_parameter("gesture_topic").value)

        self._pub = self.create_publisher(Int32, topic, 10)

        # Non-blocking server socket — polled in timer callback so rclpy stays alive.
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("", port))
        self._server.listen(1)
        self._server.setblocking(False)
        self.get_logger().info(f"Listening for NaviFlame on port {port}, publishing to {topic}")

        self._client: socket.socket | None = None
        self._buf = b""

        # Poll ~50 Hz — fast enough to keep latency low, cheap enough to not burn CPU.
        self.create_timer(0.02, self._poll)

    def _poll(self) -> None:
        # Accept a new client if none connected.
        if self._client is None:
            try:
                self._client, addr = self._server.accept()
                self._client.setblocking(False)
                self._buf = b""
                self.get_logger().info(f"NaviFlame connected from {addr}")
            except BlockingIOError:
                return

        # Read available bytes from connected client.
        try:
            chunk = self._client.recv(256)
            if not chunk:
                self.get_logger().info("NaviFlame disconnected.")
                self._client.close()
                self._client = None
                return
            self._buf += chunk
        except BlockingIOError:
            pass
        except ConnectionResetError:
            self.get_logger().info("NaviFlame connection reset.")
            self._client.close()
            self._client = None
            return

        # Consume all complete 4-byte messages in the buffer.
        while len(self._buf) >= 4:
            (gesture_id,) = struct.unpack_from("<i", self._buf, 0)
            self._buf = self._buf[4:]
            msg = Int32()
            msg.data = gesture_id
            self._pub.publish(msg)
            self.get_logger().info(f"Gesture: {gesture_id}")

    def destroy_node(self) -> None:
        if self._client is not None:
            self._client.close()
        self._server.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = NaviflameGestureBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
