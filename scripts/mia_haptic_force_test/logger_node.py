#!/usr/bin/env python3
"""MVP-6GI: logger_node.

Asynchronously logs a configurable set of topics to CSV.  Subscriptions are
kept lightweight; rows are written in a background thread so publishers are
never blocked.
"""

from __future__ import annotations

import csv
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64, Float64MultiArray, Int32, String

_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and __package__ is None and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.constants import (
    FINGER_LABELS,
    MOTOR_COUNT,
    TOPIC_EMG_GESTURE,
)


class LoggerNode(Node):
    """CSV logger for the multi-node haptic force test."""

    def __init__(self) -> None:
        super().__init__("logger_node")
        self.declare_parameter("output_dir", os.path.join(_REPO_ROOT, "data", "mia_haptic_force_test"))
        self.declare_parameter("samples_csv", "samples.csv")
        self.declare_parameter("events_csv", "events.csv")
        self.declare_parameter("log_rate_hz", 50.0)

        output_dir = Path(str(self.get_parameter("output_dir").value))
        run_name = f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self._run_dir = output_dir / run_name
        self._run_dir.mkdir(parents=True, exist_ok=True)

        self._fields = self._build_fields()
        self._samples_path = self._run_dir / str(self.get_parameter("samples_csv").value)
        self._events_path = self._run_dir / str(self.get_parameter("events_csv").value)
        self._samples_file = self._samples_path.open("w", newline="", encoding="utf-8")
        self._events_file = self._events_path.open("w", newline="", encoding="utf-8")
        self._samples_writer = csv.DictWriter(self._samples_file, fieldnames=self._fields)
        self._events_writer = csv.DictWriter(
            self._events_file, fieldnames=["time", "stage", "event", "detail"]
        )
        self._samples_writer.writeheader()
        self._events_writer.writeheader()

        self._lock = threading.Lock()
        self._state: dict[str, Any] = {}
        self._queue: deque[dict[str, Any]] = deque(maxlen=10000)
        self._event_queue: deque[dict[str, Any]] = deque()

        # Subscriptions
        self.create_subscription(Float32MultiArray, "/hand/forces", self._on_forces, 10)
        self.create_subscription(String, "/hand/force_source", self._on("force_source"), 10)
        self.create_subscription(JointState, "/hand/joint_states", self._on_joint_states, 10)
        self.create_subscription(String, TOPIC_EMG_GESTURE, self._on("gesture"), 10)
        self.create_subscription(Int32, "/emg/gesture_label", self._on_int("gesture_label"), 10)
        self.create_subscription(Float32, "/emg/confidence", self._on_float("confidence"), 10)
        self.create_subscription(Float32, "/emg/proportional", self._on_float("proportional"), 10)
        self.create_subscription(Float64MultiArray, "/control/target_force", self._on_target_force, 10)
        self.create_subscription(Float64, "/control/target_wrist", self._on_float("target_wrist"), 10)
        self.create_subscription(String, "/control/mode", self._on("mode"), 10)
        self.create_subscription(String, "/control/hold_mode", self._on("hold_mode"), 10)
        self.create_subscription(Bool, "/control/enable", self._on_bool("enable"), 10)
        self.create_subscription(String, "/test/stage", self._on("stage"), 10)
        self.create_subscription(String, "/test/event", self._on_event, 10)
        self.create_subscription(Float64MultiArray, "/controller/force_error", self._on_force_error, 10)
        self.create_subscription(Bool, "/controller/active", self._on_bool("controller_active"), 10)
        self.create_subscription(String, "/controller/loop_timing", self._on_timing, 10)
        self.create_subscription(Float32MultiArray, "/haptic_band/motors", self._on_haptics, 10)

        rate_hz = float(self.get_parameter("log_rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)

        self._running = True
        self._thread = threading.Thread(target=self._write_loop, daemon=True, name="logger_loop")
        self._thread.start()

        self.get_logger().info(f"Logging to {self._run_dir}")

    def _build_fields(self) -> list[str]:
        fields = ["time"]
        for label in FINGER_LABELS:
            fields.extend([
                f"force_{label}",
                f"joint_position_{label}",
                f"target_force_{label}",
                f"force_error_{label}",
            ])
        fields.extend([
            "gesture", "gesture_label", "confidence", "proportional",
            "target_wrist", "mode", "hold_mode", "enable", "stage",
            "controller_active", "loop_timing_json",
        ])
        for i in range(MOTOR_COUNT):
            fields.append(f"haptic_motor_{i}")
        return fields

    def _on(self, key: str):
        def cb(msg: String) -> None:
            with self._lock:
                self._state[key] = msg.data
        return cb

    def _on_int(self, key: str):
        def cb(msg: Int32) -> None:
            with self._lock:
                self._state[key] = msg.data
        return cb

    def _on_float(self, key: str):
        def cb(msg: Float32 | Float64) -> None:
            with self._lock:
                self._state[key] = msg.data
        return cb

    def _on_bool(self, key: str):
        def cb(msg: Bool) -> None:
            with self._lock:
                self._state[key] = msg.data
        return cb

    def _on_forces(self, msg: Float32MultiArray) -> None:
        with self._lock:
            for i, label in enumerate(FINGER_LABELS):
                self._state[f"force_{label}"] = msg.data[i] if i < len(msg.data) else 0.0

    def _on_joint_states(self, msg: JointState) -> None:
        with self._lock:
            for i, label in enumerate(FINGER_LABELS):
                self._state[f"joint_position_{label}"] = msg.position[i] if i < len(msg.position) else 0.0

    def _on_target_force(self, msg: Float64MultiArray) -> None:
        with self._lock:
            for i, label in enumerate(FINGER_LABELS):
                self._state[f"target_force_{label}"] = msg.data[i] if i < len(msg.data) else 0.0

    def _on_force_error(self, msg: Float64MultiArray) -> None:
        with self._lock:
            for i, label in enumerate(FINGER_LABELS):
                self._state[f"force_error_{label}"] = msg.data[i] if i < len(msg.data) else 0.0

    def _on_timing(self, msg: String) -> None:
        with self._lock:
            self._state["loop_timing_json"] = msg.data

    def _on_haptics(self, msg: Float32MultiArray) -> None:
        with self._lock:
            for i in range(MOTOR_COUNT):
                self._state[f"haptic_motor_{i}"] = msg.data[i] if i < len(msg.data) else 0.0

    def _on_event(self, msg: String) -> None:
        """Parse event JSON and queue it."""
        import json
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"event": msg.data, "detail": ""}
        self._event_queue.append({
            "time": time.time(),
            "stage": self._state.get("stage", ""),
            "event": payload.get("event", ""),
            "detail": payload.get("detail", ""),
        })

    def _write_loop(self) -> None:
        while rclpy.ok() and self._running:
            t0 = time.monotonic()
            row: dict[str, Any] = {"time": time.time()}
            with self._lock:
                for f in self._fields:
                    if f != "time":
                        row[f] = self._state.get(f, "")
            self._samples_writer.writerow(row)
            self._samples_file.flush()

            while self._event_queue:
                self._events_writer.writerow(self._event_queue.popleft())
            self._events_file.flush()

            elapsed = time.monotonic() - t0
            remaining = self._dt - elapsed
            if remaining > 0.0:
                time.sleep(remaining)

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._samples_file.close()
        self._events_file.close()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LoggerNode()
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
