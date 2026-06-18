#!/usr/bin/env python3
"""MVP-N6M: supervisor_node.

Test state machine.  Decides target force, wrist angle, controller mode, and
hold modality based on EMG gestures and contact-force feedback.  Publishes
setpoints for the controller and orchestration state for the rest of the stack.
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Float64, Float64MultiArray, Int32, String

_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and __package__ is None and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.constants import (
    EMG_ACTIVATION_LABEL,
    EMG_DECREASE_FORCE_LABEL,
    EMG_INCREASE_FORCE_LABEL,
    EMG_OPEN_LABEL,
    EMG_REST_LABEL,
    EMG_WRIST_NEGATIVE_LABEL,
    EMG_WRIST_POSITIVE_LABEL,
    EMG_WRIST_TOGGLE_LABEL,
    FINGER_COUNT,
    FINGER_LABELS,
    Stage,
    TOPIC_CONTROL_ENABLE,
    TOPIC_CONTROL_HOLD_MODE,
    TOPIC_CONTROL_MODE,
    TOPIC_CONTROL_TARGET_FORCE,
    TOPIC_CONTROL_TARGET_WRIST,
    TOPIC_EMG_GESTURE,
    TOPIC_EMG_GESTURE_LABEL,
    TOPIC_HAND_FORCES,
    TOPIC_TEST_EVENT,
    TOPIC_TEST_STAGE,
)
from scripts.mia_haptic_force_test.common.conversions import as_bool, clamp, dict_get, finger_values


def _now() -> float:
    return time.monotonic()


class SupervisorNode(Node):
    """High-level test supervisor / state machine."""

    def __init__(self) -> None:
        super().__init__("supervisor_node")
        self._cfg = self._load_config()
        self._dt = 1.0 / float(self._cfg.get("control_rate_hz", 100.0))

        # Publishers
        self._target_force_pub = self.create_publisher(Float64MultiArray, TOPIC_CONTROL_TARGET_FORCE, 10)
        self._target_wrist_pub = self.create_publisher(Float64MultiArray, TOPIC_CONTROL_TARGET_WRIST, 10)
        self._mode_pub = self.create_publisher(String, TOPIC_CONTROL_MODE, 10)
        self._enable_pub = self.create_publisher(Bool, TOPIC_CONTROL_ENABLE, 10)
        self._hold_mode_pub = self.create_publisher(String, TOPIC_CONTROL_HOLD_MODE, 10)
        self._stage_pub = self.create_publisher(String, TOPIC_TEST_STAGE, 10)
        self._event_pub = self.create_publisher(String, TOPIC_TEST_EVENT, 10)

        # Subscriptions
        self.create_subscription(Int32, TOPIC_EMG_GESTURE_LABEL, self._on_gesture_label, 10)
        self.create_subscription(String, TOPIC_EMG_GESTURE, self._on_gesture_name, 10)
        self.create_subscription(Float32MultiArray, TOPIC_HAND_FORCES, self._on_forces, 10)

        # State
        self._lock = threading.Lock()
        self._stage = Stage.INITIALISING
        self._stage_start = _now()
        self._gesture_label = EMG_REST_LABEL
        self._gesture_name = "REST"
        self._forces: list[float] = [0.0] * FINGER_COUNT
        self._target_force: list[float] = list(self._cfg["initial_hold_targets"])
        self._target_wrist: float = float(self._cfg["wrist"]["horizontal_deg"])
        self._mode: str = "velocity"
        self._hold_mode: str = "force"
        self._enable: bool = True
        self._wrist_control: bool = False
        self._closing_velocity: float = float(self._cfg["closing_velocity_start_rad_s"])
        self._force_percent: float = 0.0

        # Transition debounce
        self._last_gesture_label: int = EMG_REST_LABEL
        self._last_activation_time: float = 0.0
        self._last_toggle_time: float = 0.0
        self._last_adjustment_time: float = 0.0
        self._activation_debounce_s: float = 0.5
        self._toggle_debounce_s: float = 0.5
        self._adjustment_interval_s: float = 0.2

        self.get_logger().info("Supervisor initialised")
        self._publish_all()

        # State machine loop
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="supervisor_loop")
        self._thread.start()

    def _load_config(self) -> dict[str, Any]:
        import yaml

        path = os.path.join(_REPO_ROOT, "config", "mia_haptic_force_test.yaml")
        try:
            with open(path) as f:
                raw = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(f"Config load failed ({path}): {exc}; using defaults")
            raw = {}
        return {
            "control_rate_hz": float(dict_get(raw, "runtime.control_rate_hz", 100.0)),
            "open_positions": finger_values(raw, "hand.open_positions", 0.0),
            "max_closure_positions": finger_values(raw, "hand.max_closure_positions", 1.5),
            "closing_velocity_start_rad_s": float(dict_get(raw, "hand.closing_velocity_start_rad_s", 0.3)),
            "closing_velocity_end_rad_s": float(dict_get(raw, "hand.closing_velocity_end_rad_s", 0.05)),
            "closing_decay_steps": int(dict_get(raw, "hand.closing_decay_steps", 20)),
            "contact_thresholds": finger_values(raw, "force.contact_thresholds", 50.0),
            "initial_hold_targets": finger_values(raw, "force.initial_hold_targets", 150.0),
            "target_min": float(dict_get(raw, "force.target_min", 50.0)),
            "target_max": float(dict_get(raw, "force.target_max", 500.0)),
            "adjustment_rate_up": float(dict_get(raw, "force.adjustment_rate_up", 20.0)),
            "adjustment_rate_down": float(dict_get(raw, "force.adjustment_rate_down", 20.0)),
            "wrist": {
                "horizontal_deg": float(dict_get(raw, "wrist.horizontal_deg", 0.0)),
                "vertical_deg": float(dict_get(raw, "wrist.vertical_deg", 90.0)),
                "velocity_deg_s": float(dict_get(raw, "wrist.control_velocity_deg_s", 30.0)),
                "min_deg": float(dict_get(raw, "wrist.min_deg", -10.0)),
                "max_deg": float(dict_get(raw, "wrist.max_deg", 100.0)),
            },
            "timings": {
                "vertical_delay_s": float(dict_get(raw, "wrist.vertical_delay_s", 3.0)),
                "return_delay_s": float(dict_get(raw, "wrist.return_after_open_delay_s", 1.0)),
                "opening_min_s": float(dict_get(raw, "hand.opening_min_s", 1.0)),
                "opening_timeout_s": float(dict_get(raw, "hand.opening_timeout_s", 5.0)),
            },
            "force": {
                "min_grasp_force": float(dict_get(raw, "haptics.force_min_grasp_force", 50.0)),
                "max_grasp_force": float(dict_get(raw, "haptics.force_max_grasp_force", 500.0)),
            },
        }

    # ── callbacks ────────────────────────────────────────────────────────────

    def _on_gesture_label(self, msg: Int32) -> None:
        with self._lock:
            self._gesture_label = msg.data

    def _on_gesture_name(self, msg: String) -> None:
        with self._lock:
            self._gesture_name = msg.data

    def _on_forces(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._forces = list(msg.data[:FINGER_COUNT]) if msg.data else [0.0] * FINGER_COUNT

    # ── state helpers ────────────────────────────────────────────────────────

    def _enter(self, stage: Stage, detail: str = "") -> None:
        with self._lock:
            old = self._stage
            self._stage = stage
            self._stage_start = _now()
        self.get_logger().info(f"Stage {old.value} -> {stage.value} {detail}".strip())
        self._publish_stage()
        if detail:
            self._publish_event("stage_change", f"{old.value} -> {stage.value}: {detail}")
        else:
            self._publish_event("stage_change", f"{old.value} -> {stage.value}")

    def _stage_elapsed(self) -> float:
        with self._lock:
            return _now() - self._stage_start

    def _publish_stage(self) -> None:
        with self._lock:
            self._stage_pub.publish(String(data=self._stage.value))

    def _publish_event(self, event: str, detail: str = "") -> None:
        self._event_pub.publish(String(data=json.dumps({"event": event, "detail": detail})))

    def _publish_all(self) -> None:
        with self._lock:
            stage = self._stage
            target_force = self._target_force
            target_wrist = self._target_wrist
            mode = self._mode
            hold = self._hold_mode
            enable = self._enable

        self._target_force_pub.publish(Float64MultiArray(data=target_force))
        self._target_wrist_pub.publish(Float64MultiArray(data=[target_wrist, 0.0]))
        self._mode_pub.publish(String(data=mode))
        self._hold_mode_pub.publish(String(data=hold))
        self._enable_pub.publish(Bool(data=enable))
        self._stage_pub.publish(String(data=stage.value))

    # ── state machine ────────────────────────────────────────────────────────

    def _update(self) -> None:
        with self._lock:
            stage = self._stage
            label = self._gesture_label
            name = self._gesture_name
            forces = self._forces

        if stage == Stage.INITIALISING:
            # Wait for hardware / settle.
            if self._stage_elapsed() > 1.0:
                self._enter(Stage.WAITING_FOR_ACTIVATION)
            return

        # Global stop gesture
        if label == EMG_OPEN_LABEL and name == "OPEN" and stage not in (
            Stage.OPENING_HAND,
            Stage.RETURN_DELAY,
            Stage.RETURN_WRIST,
            Stage.COMPLETE,
        ):
            self._enter(Stage.OPENING_HAND, "OPEN gesture")
            return

        if stage == Stage.WAITING_FOR_ACTIVATION:
            if label == EMG_ACTIVATION_LABEL:
                self._enter(Stage.ROTATING_TO_VERTICAL, "POWER gesture")
            return

        if stage == Stage.ROTATING_TO_VERTICAL:
            cfg_wrist = self._cfg["wrist"]
            self._target_wrist = clamp(
                self._target_wrist + math.copysign(cfg_wrist["velocity_deg_s"] * self._dt, cfg_wrist["vertical_deg"] - self._target_wrist),
                cfg_wrist["min_deg"],
                cfg_wrist["max_deg"],
            )
            if abs(self._target_wrist - cfg_wrist["vertical_deg"]) < 2.0:
                self._enter(Stage.VERTICAL_DELAY)
            return

        if stage == Stage.VERTICAL_DELAY:
            if self._stage_elapsed() >= self._cfg["timings"]["vertical_delay_s"]:
                self._enter(Stage.FORCE_CLOSING)
            return

        if stage == Stage.FORCE_CLOSING:
            with self._lock:
                self._mode = "velocity"
            self._closing_velocity = max(
                float(self._cfg["closing_velocity_end_rad_s"]),
                self._closing_velocity
                - (float(self._cfg["closing_velocity_start_rad_s"]) - float(self._cfg["closing_velocity_end_rad_s"]))
                / max(int(self._cfg["closing_decay_steps"]), 1),
            )
            # Detect contact on any finger.
            if any(f >= t for f, t in zip(forces, self._cfg["contact_thresholds"])):
                self._enter(Stage.FORCE_HOLD, "contact detected")
            return

        if stage == Stage.FORCE_HOLD:
            with self._lock:
                self._mode = "hold"
                self._hold_mode = "force" if not self._wrist_control else "wrist"
            self._handle_hold_adjustments(label)
            return

        if stage == Stage.OPENING_HAND:
            with self._lock:
                self._mode = "velocity"
                self._target_force = list(self._cfg["open_positions"])
            elapsed = self._stage_elapsed()
            if elapsed >= self._cfg["timings"]["opening_min_s"]:
                self._enter(Stage.RETURN_DELAY)
            return

        if stage == Stage.RETURN_DELAY:
            if self._stage_elapsed() >= self._cfg["timings"]["return_delay_s"]:
                self._enter(Stage.RETURN_WRIST)
            return

        if stage == Stage.RETURN_WRIST:
            cfg_wrist = self._cfg["wrist"]
            self._target_wrist = clamp(
                self._target_wrist - math.copysign(cfg_wrist["velocity_deg_s"] * self._dt, self._target_wrist - cfg_wrist["horizontal_deg"]),
                cfg_wrist["min_deg"],
                cfg_wrist["max_deg"],
            )
            if abs(self._target_wrist - cfg_wrist["horizontal_deg"]) < 2.0:
                self._enter(Stage.COMPLETE)
            return

        if stage == Stage.COMPLETE:
            with self._lock:
                self._enable = False
            return

    def _handle_hold_adjustments(self, label: int) -> None:
        now = _now()
        if now - self._last_toggle_time > self._toggle_debounce_s and label == EMG_WRIST_TOGGLE_LABEL:
            self._wrist_control = not self._wrist_control
            self._last_toggle_time = now
            self._publish_event("toggle", f"wrist_control={self._wrist_control}")

        if now - self._last_adjustment_time < self._adjustment_interval_s:
            return

        if self._wrist_control:
            cfg_wrist = self._cfg["wrist"]
            delta = 0.0
            if label == EMG_WRIST_POSITIVE_LABEL:
                delta = cfg_wrist["velocity_deg_s"] * self._dt * 5
            elif label == EMG_WRIST_NEGATIVE_LABEL:
                delta = -cfg_wrist["velocity_deg_s"] * self._dt * 5
            if delta != 0.0:
                self._target_wrist = clamp(self._target_wrist + delta, cfg_wrist["min_deg"], cfg_wrist["max_deg"])
                self._last_adjustment_time = now
        else:
            rate = 0.0
            if label == EMG_INCREASE_FORCE_LABEL:
                rate = float(self._cfg["adjustment_rate_up"])
            elif label == EMG_DECREASE_FORCE_LABEL:
                rate = -float(self._cfg["adjustment_rate_down"])
            if rate != 0.0:
                target_min = float(self._cfg["target_min"])
                target_max = float(self._cfg["target_max"])
                for i in range(FINGER_COUNT):
                    self._target_force[i] = clamp(
                        self._target_force[i] + rate * self._dt,
                        target_min,
                        target_max,
                    )
                self._last_adjustment_time = now

    def _loop(self) -> None:
        while rclpy.ok() and self._running:
            t0 = time.monotonic()
            self._update()
            self._publish_all()
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
    node = SupervisorNode()
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
