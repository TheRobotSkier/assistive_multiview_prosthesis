#!/usr/bin/env python3
"""Mock EMG publisher for testing EMG grasp without hardware.

Publishes gesture labels on /emg/gesture_label and confidence on /emg/confidence.
Usage:
  python3 mock_emg_publisher.py --gesture 1 --duration 3.0
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32


class MockEmgPublisher(Node):
    def __init__(self, gesture: int, confidence: float, duration: float):
        super().__init__("mock_emg_publisher")
        self._gesture = gesture
        self._confidence = confidence
        self._duration = duration
        self._start_time = time.time()

        self._gesture_pub = self.create_publisher(Int32, "/emg/gesture_label", 10)
        self._conf_pub = self.create_publisher(Float32, "/emg/confidence", 10)
        self._timer = self.create_timer(0.1, self._publish)

    def _publish(self):
        elapsed = time.time() - self._start_time
        if elapsed > self._duration:
            self.get_logger().info("Duration exceeded — publishing REST")
            gesture = 0
        else:
            gesture = self._gesture

        self._gesture_pub.publish(Int32(data=gesture))
        self._conf_pub.publish(Float32(data=self._confidence))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gesture", type=int, default=1, help="Gesture ID to publish")
    parser.add_argument("--confidence", type=float, default=0.95, help="Confidence value")
    parser.add_argument("--duration", type=float, default=5.0, help="How long to publish gesture")
    args = parser.parse_args()

    rclpy.init()
    node = MockEmgPublisher(args.gesture, args.confidence, args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
