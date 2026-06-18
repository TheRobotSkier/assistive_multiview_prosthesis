#!/usr/bin/env python3
"""MVP-8U7: haptic_node.

Maps test stage, wrist angle, and grasp-force information to haptic-band motor
intensities.  Publishes ``/haptic_band/motors`` (std_msgs/Float32MultiArray) at
~10 Hz.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray, String

_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and __package__ is None and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.constants import MOTOR_COUNT
from scripts.mia_haptic_force_test.common.conversions import force_haptics, wrist_haptics


def _scale_motors(values: list[float], scale: float) -> list[float]:
    return [clamp(v * scale, 0.0, 100.0) for v in values]


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class HapticNode(Node):
    """Haptic feedback generator."""

    def __init__(self) -> None:
        super().__init__("haptic_node")
        self.declare_parameter("publish_rate_hz", 10.0)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)

        self._pub = self.create_publisher(Float32MultiArray, "/haptic_band/motors", 10)

        self.create_subscription(String, "/test/stage", self._on_stage, 10)
        self.create_subscription(String, "/emg/gesture", self._on_gesture, 10)
        self.create_subscription(Float32, "/emg/proportional", self._on_proportional, 10)
        self.create_subscription(Float64MultiArray, "/wrist/state", self._on_wrist_state, 10)
        self.create_subscription(Float32MultiArray, "/hand/forces", self._on_forces, 10)

        self._lock = threading.Lock()
        self._stage = "initialising"
        self._gesture = "REST"
        self._proportional = 0.0
        self._wrist_angle = 0.0
        self._forces: list[float] = [0.0] * 3
        self._event_deadline: float = 0.0
        self._event_intensities: list[float] = [0.0] * MOTOR_COUNT

        self._cfg: dict = self._load_config()

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="haptic_loop")
        self._thread.start()

    def _load_config(self) -> dict:
        import yaml

        path = os.path.join(_REPO_ROOT, "config", "mia_haptic_force_test.yaml")
        try:
            with open(path) as f:
                raw = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(f"Config load failed ({path}): {exc}")
            raw = {}
        return dict(raw.get("haptics", {}))

    def _on_stage(self, msg: String) -> None:
        with self._lock:
            new_stage = msg.data
            if new_stage != self._stage:
                self._stage = new_stage
                self._trigger_event_for_stage(new_stage)

    def _on_gesture(self, msg: String) -> None:
        with self._lock:
            self._gesture = msg.data

    def _on_proportional(self, msg: Float32) -> None:
        with self._lock:
            self._proportional = clamp(msg.data, 0.0, 1.0)

    def _on_wrist_state(self, msg: Float64MultiArray) -> None:
        with self._lock:
            if msg.data:
                self._wrist_angle = float(msg.data[0])

    def _on_forces(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._forces = list(msg.data) if msg.data else [0.0] * 3

    def _trigger_event_for_stage(self, stage: str) -> None:
        """Emit short buzz on stage transitions."""
        duration = 0.0
        intensity = 0.0
        if stage == "rotating_to_vertical":
            duration = float(self._cfg.get("activation_buzz_duration_s", 0.12))
            intensity = float(self._cfg.get("activation_buzz_intensity_pct", 12.0))
        elif stage == "force_closing":
            duration = float(self._cfg.get("closure_start_buzz_duration_s", 0.15))
            intensity = float(self._cfg.get("closure_start_buzz_intensity_pct", 22.0))
        elif stage == "force_hold":
            duration = float(self._cfg.get("contact_buzz_duration_s", 0.20))
            intensity = float(self._cfg.get("contact_buzz_intensity_pct", 35.0))
        if duration > 0.0 and intensity > 0.0:
            indices = list(self._cfg.get("event_motor_indices", range(MOTOR_COUNT)))
            self._event_intensities = [
                intensity if i in indices else 0.0 for i in range(MOTOR_COUNT)
            ]
            self._event_deadline = time.monotonic() + duration

    def _compute_motors(self) -> list[float]:
        with self._lock:
            stage = self._stage
            gesture = self._gesture
            proportional = self._proportional
            wrist_angle = self._wrist_angle
            cfg = self._cfg

        # Event buzz overrides everything.
        if time.monotonic() < self._event_deadline:
            return list(self._event_intensities)

        if stage in ("initialising", "waiting_for_activation", "complete", "fault"):
            return [0.0] * MOTOR_COUNT

        motors = [0.0] * MOTOR_COUNT

        # Wrist haptics during vertical rotation / wrist control.
        if stage in ("rotating_to_vertical", "vertical_delay", "return_wrist"):
            motors = wrist_haptics(wrist_angle, cfg)

        # Force haptics during hold/closing.
        if stage in ("force_closing", "force_hold"):
            f_min = float(cfg.get("force_min_grasp_force", 50.0))
            f_max = float(cfg.get("force_max_grasp_force", 500.0))
            avg_force = sum(self._forces) / max(len(self._forces), 1)
            pct = clamp((avg_force - f_min) / max(f_max - f_min, 1.0) * 100.0, 0.0, 100.0)
            force_motors, _ = force_haptics(pct, cfg)
            # Blend: max of wrist and force patterns.
            motors = [max(m, f) for m, f in zip(motors, force_motors)]

        # Scale by proportional when gesture is active.
        if gesture != "REST":
            motors = _scale_motors(motors, 0.5 + 0.5 * proportional)

        return motors

    def _loop(self) -> None:
        while rclpy.ok() and self._running:
            t0 = time.monotonic()
            motors = self._compute_motors()
            msg = Float32MultiArray()
            msg.data = motors
            self._pub.publish(msg)
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
    node = HapticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        # Zero motors on exit.
        node._pub.publish(Float32MultiArray(data=[0.0] * MOTOR_COUNT))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
