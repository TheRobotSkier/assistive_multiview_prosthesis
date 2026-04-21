#!/usr/bin/env python3
"""
Click relay demo node — no GUI dependency.

Subscribes to:
  /clicked_point  (geometry_msgs/PointStamped)  — from RViz2 "Publish Point" tool

Forwards each click to:
  /segmentation/click_positive  or  /segmentation/click_negative
depending on the current mode (default: positive).

Reset is sent to:
  /segmentation/reset  (std_msgs/Empty)

Mode is controlled interactively via stdin in a background thread:
  p  → switch to POSITIVE mode
  n  → switch to NEGATIVE mode
  r  → reset all clicks
  q  → quit
  (just press Enter to see current mode)

If stdin is not a tty (e.g. piped), the node just relays in positive mode silently.
"""
import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty


class ClickRelayNode(Node):
    def __init__(self):
        super().__init__('demo_click_relay')
        self._mode = 'positive'  # 'positive' or 'negative'
        self._lock = threading.Lock()

        self._pub_pos = self.create_publisher(PointStamped, '/segmentation/click_positive', 10)
        self._pub_neg = self.create_publisher(PointStamped, '/segmentation/click_negative', 10)
        self._pub_reset = self.create_publisher(Empty, '/segmentation/reset', 10)

        self.create_subscription(PointStamped, '/clicked_point', self._on_click, 10)
        self.get_logger().info(
            'Click relay ready. Clicks from RViz2 /clicked_point → /segmentation/click_positive\n'
            '  p = positive mode | n = negative mode | r = reset | q = quit'
        )

    def _on_click(self, msg: PointStamped):
        with self._lock:
            mode = self._mode
        if mode == 'positive':
            self._pub_pos.publish(msg)
            self.get_logger().info(
                f'[+] positive click at ({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})'
            )
        else:
            self._pub_neg.publish(msg)
            self.get_logger().info(
                f'[-] negative click at ({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})'
            )

    def set_mode(self, mode: str):
        with self._lock:
            self._mode = mode
        self.get_logger().info(f'Mode → {mode.upper()}')

    def reset(self):
        self._pub_reset.publish(Empty())
        self.get_logger().info('Reset sent.')


def stdin_input_loop(node: ClickRelayNode):
    """Read single-character commands from stdin (runs in a daemon thread)."""
    if not sys.stdin.isatty():
        return  # non-interactive — skip
    print('\n--- Click relay controls ---')
    print('  p  positive mode   n  negative mode   r  reset   q  quit')
    print(f'  Current mode: {node._mode.upper()}')
    print('----------------------------\n', flush=True)
    while rclpy.ok():
        try:
            line = input('> ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if line in ('p', 'positive'):
            node.set_mode('positive')
        elif line in ('n', 'negative'):
            node.set_mode('negative')
        elif line in ('r', 'reset'):
            node.reset()
        elif line in ('q', 'quit'):
            rclpy.shutdown()
            break
        elif line == '':
            with node._lock:
                print(f'  Current mode: {node._mode.upper()}', flush=True)
        else:
            print('  Unknown command. p/n/r/q', flush=True)


def main():
    rclpy.init()
    node = ClickRelayNode()

    t = threading.Thread(target=stdin_input_loop, args=(node,), daemon=True)
    t.start()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
