#!/usr/bin/env python3
"""MVP-DCA: terminal_ui_node.

Lightweight status printer at ~2 Hz.  Avoids heavy TUI libraries so it cannot
block the control loop; it simply prints a compact summary to stdout/stderr.
"""

from __future__ import annotations

import os
import sys
import threading
import time
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64MultiArray, String

_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and __package__ is None and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.constants import FINGER_LABELS, MOTOR_COUNT, TOPIC_EMG_GESTURE


class TerminalUINode(Node):
    """Minimal terminal status UI."""

    def __init__(self) -> None:
        super().__init__("terminal_ui_node")
        self.declare_parameter("refresh_rate_hz", 2.0)
        rate_hz = float(self.get_parameter("refresh_rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 0.1)

        self._lock = threading.Lock()
        self._stage = "initialising"
        self._gesture = "REST"
        self._forces: list[float] = [0.0] * len(FINGER_LABELS)
        self._target_force: list[float] = [0.0] * len(FINGER_LABELS)
        self._target_wrist: float = 0.0
        self._mode = "velocity"
        self._hold_mode = "force"
        self._active = False
        self._timing_json = "{}"
        self._haptics: list[float] = [0.0] * MOTOR_COUNT
        self._joint_positions: list[float] = [0.0] * len(FINGER_LABELS)

        self.create_subscription(String, "/test/stage", self._on_string("stage"), 10)
        self.create_subscription(String, TOPIC_EMG_GESTURE, self._on_string("gesture"), 10)
        self.create_subscription(Float32MultiArray, "/hand/forces", self._on_forces, 10)
        self.create_subscription(Float64MultiArray, "/control/target_force", self._on_target_force, 10)
        self.create_subscription(String, "/control/mode", self._on_string("mode"), 10)
        self.create_subscription(String, "/control/hold_mode", self._on_string("hold_mode"), 10)
        self.create_subscription(Bool, "/controller/active", self._on_bool("active"), 10)
        self.create_subscription(String, "/controller/loop_timing", self._on_string("timing_json"), 10)
        self.create_subscription(Float32MultiArray, "/haptic_band/motors", self._on_haptics, 10)
        self.create_subscription(JointState, "/hand/joint_states", self._on_joint_states, 10)
        # target_wrist is Float64 in contract; also accept Float64MultiArray for compatibility
        self.create_subscription(Float64MultiArray, "/control/target_wrist", self._on_target_wrist, 10)

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="ui_loop")
        self._thread.start()

    def _on_string(self, key: str):
        def cb(msg: String) -> None:
            with self._lock:
                setattr(self, f"_{key}", msg.data)
        return cb

    def _on_bool(self, key: str):
        def cb(msg: Bool) -> None:
            with self._lock:
                setattr(self, f"_{key}", msg.data)
        return cb

    def _on_forces(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._forces = list(msg.data[: len(FINGER_LABELS)])

    def _on_target_force(self, msg: Float64MultiArray) -> None:
        with self._lock:
            self._target_force = list(msg.data[: len(FINGER_LABELS)])

    def _on_haptics(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._haptics = list(msg.data[:MOTOR_COUNT])

    def _on_joint_states(self, msg: JointState) -> None:
        with self._lock:
            self._joint_positions = list(msg.position[: len(FINGER_LABELS)])

    def _on_target_wrist(self, msg: Float64MultiArray) -> None:
        with self._lock:
            if msg.data:
                self._target_wrist = float(msg.data[0])

    def _loop(self) -> None:
        while rclpy.ok() and self._running:
            t0 = time.monotonic()
            self._render()
            elapsed = time.monotonic() - t0
            remaining = self._dt - elapsed
            if remaining > 0.0:
                time.sleep(remaining)

    def _render(self) -> None:
        with self._lock:
            stage = self._stage
            gesture = self._gesture
            forces = self._forces
            targets = self._target_force
            mode = self._mode
            hold = self._hold_mode
            active = self._active
            timing = self._timing_json
            haptics = self._haptics
            positions = self._joint_positions
            wrist = self._target_wrist

        lines = [
            "=" * 70,
            f"  Stage: {stage:24s}  Gesture: {gesture}",
            f"  Mode: {mode}   Hold: {hold}   Controller active: {active}",
            "-" * 70,
        ]
        fstr = "  Force: " + "  ".join(
            f"{l}: {f:6.1f} / {t:6.1f}" for l, f, t in zip(FINGER_LABELS, forces, targets)
        )
        lines.append(fstr)
        jstr = "  Joints:" + "  ".join(f"{l}: {math.degrees(p):5.1f}°" for l, p in zip(FINGER_LABELS, positions))
        lines.append(jstr)
        lines.append(f"  Wrist target: {wrist:6.1f}°   Loop timing: {timing}")
        hstr = "  Haptics: " + " ".join(f"{h:4.1f}" for h in haptics)
        lines.append(hstr)
        lines.append("=" * 70)
        print("\n".join(lines), flush=True)

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TerminalUINode()
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
