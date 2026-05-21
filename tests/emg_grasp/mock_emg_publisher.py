#!/usr/bin/env python3
"""Mock EMG publisher for testing EMG grasp without hardware.

Publishes gesture labels on /emg/gesture_label and confidence on /emg/confidence.

Usage (single gesture):
  python3 mock_emg_publisher.py --gesture 1 --duration 3.0

Usage (gesture sequence):
  python3 mock_emg_publisher.py --sequence "1:2.0,2:1.0,0:5.0"
  # Format: gesture_id:duration_s,gesture_id:duration_s,...
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32


class MockEmgPublisher(Node):
    def __init__(self, gesture: int = None, confidence: float = 0.95,
                 duration: float = 5.0, sequence: list = None):
        super().__init__("mock_emg_publisher")
        self._confidence = confidence
        self._start_time = time.time()

        if sequence:
            self._sequence = sequence
            self._current_idx = 0
            self._segment_start = self._start_time
            self._mode = "sequence"
        else:
            self._gesture = gesture if gesture is not None else 1
            self._duration = duration
            self._mode = "single"

        self._gesture_pub = self.create_publisher(Int32, "/emg/gesture_label", 10)
        self._conf_pub = self.create_publisher(Float32, "/emg/confidence", 10)
        self._timer = self.create_timer(0.1, self._publish)

    def _publish(self):
        if self._mode == "sequence":
            now = time.time()
            elapsed = now - self._segment_start

            # Advance through sequence segments
            while self._current_idx < len(self._sequence):
                gid, dur = self._sequence[self._current_idx]
                if elapsed >= dur:
                    self._segment_start += dur
                    elapsed -= dur
                    self._current_idx += 1
                else:
                    break

            if self._current_idx >= len(self._sequence):
                gesture = 0  # REST after sequence ends
            else:
                gesture = self._sequence[self._current_idx][0]
        else:
            elapsed = time.time() - self._start_time
            if elapsed > self._duration:
                self.get_logger().info("Duration exceeded — publishing REST")
                gesture = 0
            else:
                gesture = self._gesture

        self._gesture_pub.publish(Int32(data=gesture))
        self._conf_pub.publish(Float32(data=self._confidence))


def main():
    parser = argparse.ArgumentParser(description="Mock EMG publisher")
    parser.add_argument("--gesture", type=int, default=None,
                        help="Gesture ID to publish")
    parser.add_argument("--confidence", type=float, default=0.95,
                        help="Confidence value")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="How long to publish gesture (single mode)")
    parser.add_argument("--sequence", type=str, default=None,
                        help="Gesture sequence: gesture_id:duration_s,gesture_id:duration_s,...")
    args = parser.parse_args()

    if args.sequence:
        sequence = []
        for item in args.sequence.split(','):
            gid, dur = item.split(':')
            sequence.append((int(gid), float(dur)))
        rclpy.init()
        node = MockEmgPublisher(confidence=args.confidence, sequence=sequence)
    elif args.gesture is not None:
        rclpy.init()
        node = MockEmgPublisher(
            gesture=args.gesture, confidence=args.confidence, duration=args.duration)
    else:
        # Default: publish POWER for 5 seconds
        rclpy.init()
        node = MockEmgPublisher(gesture=1, confidence=args.confidence, duration=args.duration)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
