#!/usr/bin/env python3
"""Mock EMG gesture publisher for triggering the grasp pipeline without hardware.

Publishes /emg/gesture_label and /emg/confidence on a configurable cycle
so the pipeline_manager state machine can transition through its states
without a real MindRove EMG band.

Default cycle (configurable):
  1. POWER gesture → high confidence → pipeline starts SEGMENTING
  2. Wait grasp_duration seconds
  3. OPEN gesture → high confidence → pipeline releases
  4. Wait release_duration seconds
  5. Repeat

Publishes:
    /emg/gesture_label  (std_msgs/Int32)   — gesture class label
    /emg/confidence      (std_msgs/Float32) — gesture confidence [0,1]
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

# Must match pipeline_manager_node's GESTURE_* constants
GESTURE_REST = 0
GESTURE_POWER = 1
GESTURE_PINCH = 2
GESTURE_OPEN = 3
GESTURE_POINT = 4


class MockEmgPublisher(Node):
    def __init__(self) -> None:
        super().__init__("mock_emg_publisher")

        self.declare_parameter("grasp_gesture", GESTURE_POWER)
        self.declare_parameter("release_gesture", GESTURE_OPEN)
        self.declare_parameter("grasp_duration", 8.0)
        self.declare_parameter("release_duration", 3.0)
        self.declare_parameter("confidence", 0.9)

        self._grasp_gesture = int(self.get_parameter("grasp_gesture").value)
        self._release_gesture = int(self.get_parameter("release_gesture").value)
        self._grasp_duration = float(self.get_parameter("grasp_duration").value)
        self._release_duration = float(self.get_parameter("release_duration").value)
        self._confidence = float(self.get_parameter("confidence").value)

        self._label_pub = self.create_publisher(Int32, "/emg/gesture_label", 10)
        self._conf_pub = self.create_publisher(Float32, "/emg/confidence", 10)

        # Start with the release gesture so the pipeline begins idle
        self._phase = "release"
        self._elapsed = 0.0

        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"Mock EMG publisher started — "
            f"grasp={self._grasp_gesture} ({self._grasp_duration}s), "
            f"release={self._release_gesture} ({self._release_duration}s)"
        )

    def _tick(self) -> None:
        self._elapsed += 0.1

        if self._phase == "release":
            if self._elapsed >= self._release_duration:
                self._phase = "grasp"
                self._elapsed = 0.0
                self._publish_gesture(self._grasp_gesture)
                self.get_logger().debug(
                    f"Triggering grasp (gesture={self._grasp_gesture})"
                )
        elif self._phase == "grasp":
            if self._elapsed >= self._grasp_duration:
                self._phase = "release"
                self._elapsed = 0.0
                self._publish_gesture(self._release_gesture)
                self.get_logger().debug(
                    f"Triggering release (gesture={self._release_gesture})"
                )

    def _publish_gesture(self, label: int) -> None:
        stamp = self.get_clock().now().to_msg()
        label_msg = Int32(data=label)
        conf_msg = Float32(data=self._confidence)
        self._label_pub.publish(label_msg)
        self._conf_pub.publish(conf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockEmgPublisher()
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
