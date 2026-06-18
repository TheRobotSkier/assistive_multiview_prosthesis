#!/usr/bin/env python3
"""MVP-HJ3: emg_input_node.

Wraps the existing emg_bridge board reader and classifier, predicts gesture at
~50 Hz, and publishes::

    /emg/gesture      (std_msgs/String)   human-readable gesture name
    /emg/gesture_label (std_msgs/Int32)   numeric class label
    /emg/confidence   (std_msgs/Float32)  classifier confidence
    /emg/proportional (std_msgs/Float32)  continuous 0-1 proportional value
"""

from __future__ import annotations

import os
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String

_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and __package__ is None and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.constants import GESTURES, REST_GESTURE_LABEL


class EmgInputNode(Node):
    """MindRove EMG input + gesture prediction node."""

    def __init__(self) -> None:
        super().__init__("emg_input_node")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("use_simulated_gestures", False)
        self.declare_parameter("simulated_gesture", "REST")
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)

        self._gesture_pub = self.create_publisher(String, "/emg/gesture", 10)
        self._label_pub = self.create_publisher(Int32, "/emg/gesture_label", 10)
        self._conf_pub = self.create_publisher(Float32, "/emg/confidence", 10)
        self._prop_pub = self.create_publisher(Float32, "/emg/proportional", 10)

        self._use_simulated = bool(self.get_parameter("use_simulated_gestures").value)
        self._simulated_gesture = str(self.get_parameter("simulated_gesture").value)

        self._reader = None
        self._classifier = None
        if not self._use_simulated:
            try:
                from emg_bridge.board_reader import BoardReader
                from emg_bridge.classifier import Classifier
                self._reader = BoardReader()
                self._classifier = Classifier()
                self.get_logger().info("MindRove board and classifier initialised")
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to initialise MindRove hardware ({exc}); falling back to simulated REST"
                )
                self._use_simulated = True

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="emg_loop")
        self._thread.start()

    def _predict(self) -> tuple[str, int, float, float]:
        """Return (gesture_name, label, confidence, proportional)."""
        if self._use_simulated:
            name = self._simulated_gesture
            label = GESTURES.get(name, REST_GESTURE_LABEL)
            return name, label, 1.0, 0.0

        try:
            sample = self._reader.read() if self._reader else None
            if sample is None:
                return "REST", REST_GESTURE_LABEL, 0.0, 0.0
            prediction = self._classifier.predict(sample)
            name = prediction.get("gesture", "REST")
            label = int(prediction.get("label", GESTURES.get(name, REST_GESTURE_LABEL)))
            confidence = float(prediction.get("confidence", 0.0))
            proportional = float(prediction.get("proportional", 0.0))
            return name, label, confidence, proportional
        except Exception as exc:
            self.get_logger().warn(f"Prediction failed: {exc}")
            return "REST", REST_GESTURE_LABEL, 0.0, 0.0

    def _loop(self) -> None:
        while rclpy.ok() and self._running:
            t0 = time.monotonic()
            name, label, confidence, proportional = self._predict()

            self._gesture_pub.publish(String(data=name))
            self._label_pub.publish(Int32(data=label))
            self._conf_pub.publish(Float32(data=confidence))
            self._prop_pub.publish(Float32(data=proportional))

            elapsed = time.monotonic() - t0
            remaining = self._dt - elapsed
            if remaining > 0.0:
                time.sleep(remaining)

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = EmgInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
