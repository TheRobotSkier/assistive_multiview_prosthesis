#!/usr/bin/env python3
"""Mock EMG publisher for volitional controller testing.

Cycles through gestures in a loop for demo purposes.
Usage:
  python3 mock_emg_publisher.py --cycle-interval 3.0
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32


GESTURE_CYCLE = [0, 1, 3, 0]  # REST -> POWER -> OPEN -> REST


class MockEmgVolitionalPublisher(Node):
    def __init__(self, cycle_interval: float):
        super().__init__("mock_emg_volitional_publisher")
        self._cycle_interval = cycle_interval
        self._start_time = time.time()
        self._idx = 0

        self._gesture_pub = self.create_publisher(Int32, "/emg/gesture_label", 10)
        self._conf_pub = self.create_publisher(Float32, "/emg/confidence", 10)
        self._timer = self.create_timer(0.1, self._publish)

    def _publish(self):
        elapsed = time.time() - self._start_time
        self._idx = int(elapsed / self._cycle_interval) % len(GESTURE_CYCLE)
        gesture = GESTURE_CYCLE[self._idx]

        self._gesture_pub.publish(Int32(data=gesture))
        self._conf_pub.publish(Float32(data=0.95))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycle-interval", type=float, default=3.0,
                        help="Seconds per gesture in cycle")
    args = parser.parse_args()

    rclpy.init()
    node = MockEmgVolitionalPublisher(args.cycle_interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
