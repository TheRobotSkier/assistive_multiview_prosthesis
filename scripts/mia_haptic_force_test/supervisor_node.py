#!/usr/bin/env python3
"""MVP-N6M: supervisor_node.

Test state machine.  Decides target force, wrist angle, controller mode, and
hold modality based on EMG gestures and contact-force feedback.  Publishes
setpoints for the controller and orchestration state for the rest of the stack.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64, Float64MultiArray, Int32, String

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
    FINGER_JOINTS,
    FINGER_LABELS,
    MOTOR_COUNT,
    Stage,
    TOPIC_CONTROL_ENABLE,
    TOPIC_CONTROL_HOLD_MODE,
    TOPIC_CONTROL_MODE,
    TOPIC_CONTROL_TARGET_FORCE,
    TOPIC_CONTROL_TARGET_WRIST,
    TOPIC_EMG_CONFIDENCE,
    TOPIC_EMG_GESTURE,
    TOPIC_EMG_GESTURE_LABEL,
    TOPIC_EMG_PROPORTIONAL,
    TOPIC_GROUP_POS_FF_COMMANDS,
    TOPIC_GROUP_VEL_FF_COMMANDS,
    TOPIC_HAPTIC_BAND_MOTORS,
    TOPIC_HAND_FORCES,
    TOPIC_HAND_FORCE_SOURCE,
    TOPIC_TEST_EVENT,
    TOPIC_TEST_STAGE,
    TOPIC_WRIST_SET_POSITION,
    TOPIC_WRIST_STATE,
)
from scripts.mia_haptic_force_test.common.conversions import as_bool, circ_delta_deg, clamp, dict_get, finger_values


def _now() -> float:
    return time.monotonic()


class SupervisorNode(Node):
    """High-level test supervisor / state machine."""

    def __init__(self, config_path: Optional[str] = None) -> None:
        super().__init__("supervisor_node")
        self._config_path = config_path or os.path.join(_REPO_ROOT, "config", "mia_haptic_force_test.yaml")
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
        # Hardware-level publishers (for safety/destroy — bypass controller)
        self._haptic_pub = self.create_publisher(Float32MultiArray, TOPIC_HAPTIC_BAND_MOTORS, 10)
        self._vel_cmd_pub = self.create_publisher(Float64MultiArray, TOPIC_GROUP_VEL_FF_COMMANDS, 10)
        self._pos_cmd_pub = self.create_publisher(Float64MultiArray, TOPIC_GROUP_POS_FF_COMMANDS, 10)
        self._wrist_cmd_pub = self.create_publisher(Float64MultiArray, TOPIC_WRIST_SET_POSITION, 10)

        # Subscriptions
        self.create_subscription(Int32, TOPIC_EMG_GESTURE_LABEL, self._on_gesture_label, 10)
        self.create_subscription(String, TOPIC_EMG_GESTURE, self._on_gesture_name, 10)
        self.create_subscription(Float32, TOPIC_EMG_CONFIDENCE, self._on_confidence, 10)
        self.create_subscription(Float32, TOPIC_EMG_PROPORTIONAL, self._on_proportional, 10)
        self.create_subscription(Float32, "/emg/age", self._on_emg_age, 10)
        self.create_subscription(Float32MultiArray, TOPIC_HAND_FORCES, self._on_forces, 10)
        self.create_subscription(Float64MultiArray, TOPIC_WRIST_STATE, self._on_wrist_state, 10)
        self.create_subscription(String, TOPIC_HAND_FORCE_SOURCE, self._on_force_source, 10)
        self.create_subscription(JointState, "/hand/joint_states", self._on_joint_positions, 10)

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
        self._force_percent: float = 0.0
        # Force safety state
        self._closing_velocity: float = float(self._cfg["closing_velocity_start_rad_s"])
        self._force_source: str = "none"
        self._last_force_time: float = 0.0
        self._finished: bool = False
        self._fault_reason: str = ""
        self._joint_positions: list[float] = [0.0] * FINGER_COUNT

        # Wrist feedback
        self._wrist_actual_deg: float = 0.0
        self._got_wrist_state: bool = False
        self._last_wrist_state_time: float = 0.0
        self._wrist_reached_since: Optional[float] = None

        # Transition debounce
        self._last_gesture_label: int = EMG_REST_LABEL
        self._last_activation_time: float = 0.0
        self._last_toggle_time: float = 0.0
        self._last_adjustment_time: float = 0.0
        self._activation_debounce_s: float = 0.5
        self._toggle_debounce_s: float = 0.5
        self._adjustment_interval_s: float = 0.2

        # EMG gating state (legacy freshness / confidence / hold / power)
        self._confidence: float = 0.0
        self._proportional: float = 0.0
        self._emg_age: float = -1.0
        self._last_emg_time: float = 0.0
        self._gesture_started_at: float = 0.0
        self._power_armed: bool = True
        self._last_power_toggle: float = 0.0

        self.get_logger().info("Supervisor initialised")
        self._publish_all()

        # State machine loop
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="supervisor_loop")
        self._thread.start()

    def _load_config(self) -> dict[str, Any]:
        import yaml

        path = self._config_path
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
            "force_stale_timeout_s": float(dict_get(raw, "force.stale_timeout_s", 1.0)),
            "require_force_data": bool(dict_get(raw, "force.require_force_data", True)),
            "emergency_threshold": float(dict_get(raw, "force.emergency_threshold", 800.0)),
            "emergency_backoff_velocity_rad_s": float(dict_get(raw, "force.emergency_backoff_velocity_rad_s", -0.1)),
            "complete_shutdown_delay_s": float(dict_get(raw, "runtime.complete_shutdown_delay_s", 0.2)),
            "wrist": {
                "horizontal_deg": float(dict_get(raw, "wrist.horizontal_deg", 0.0)),
                "vertical_deg": float(dict_get(raw, "wrist.vertical_deg", 90.0)),
                "velocity_deg_s": float(dict_get(raw, "wrist.control_velocity_deg_s", 30.0)),
                "min_deg": float(dict_get(raw, "wrist.min_deg", -10.0)),
                "max_deg": float(dict_get(raw, "wrist.max_deg", 100.0)),
                "position_tolerance_deg": float(dict_get(raw, "wrist.position_tolerance_deg", 4.0)),
                "settle_s": float(dict_get(raw, "wrist.settle_s", 0.2)),
                "move_timeout_s": float(dict_get(raw, "wrist.move_timeout_s", 8.0)),
                "stale_timeout_s": float(dict_get(raw, "wrist.stale_timeout_s", 1.0)),
                "assume_target_on_timeout": bool(dict_get(raw, "wrist.assume_target_on_timeout", True)),
                "enabled": bool(dict_get(raw, "wrist.enabled", True)),
            },
            "timings": {
                "vertical_delay_s": float(dict_get(raw, "wrist.vertical_delay_s", 3.0)),
                "return_delay_s": float(dict_get(raw, "wrist.return_after_open_delay_s", 1.0)),
                "opening_min_s": float(dict_get(raw, "hand.opening_min_s", 1.0)),
                "opening_timeout_s": float(dict_get(raw, "hand.opening_timeout_s", 5.0)),
            },
            "emg_stale_timeout_s": float(dict_get(raw, "emg.stale_timeout_s", 1.0)),
            "emg_confidence_threshold": float(dict_get(raw, "emg.confidence_threshold", 0.55)),
            "emg_open_proportional_threshold": float(dict_get(raw, "emg.open_proportional_threshold", 0.5)),
            "emg_activation_hold_s": float(dict_get(raw, "emg.activation_hold_s", 0.2)),
            "emg_open_hold_s": float(dict_get(raw, "emg.open_hold_s", 1.0)),
            "emg_power_rearm_s": float(dict_get(raw, "emg.power_rearm_s", 0.4)),
            "emg_power_toggle_hold_s": float(dict_get(raw, "emg.power_toggle_hold_s", 0.5)),
            "emg_open_label": int(dict_get(raw, "emg.open_label", EMG_OPEN_LABEL)),
            "emg_wrist_toggle_label": int(dict_get(raw, "emg.wrist_toggle_label", EMG_WRIST_TOGGLE_LABEL)),
            "emg_activation_label": int(dict_get(raw, "emg.activation_label", EMG_ACTIVATION_LABEL)),
            "emg_increase_force_label": int(dict_get(raw, "emg.increase_force_label", EMG_INCREASE_FORCE_LABEL)),
            "emg_decrease_force_label": int(dict_get(raw, "emg.decrease_force_label", EMG_DECREASE_FORCE_LABEL)),
            "emg_wrist_positive_label": int(dict_get(raw, "emg.wrist_positive_label", EMG_WRIST_POSITIVE_LABEL)),
            "emg_wrist_negative_label": int(dict_get(raw, "emg.wrist_negative_label", EMG_WRIST_NEGATIVE_LABEL)),
            "force": {
                "min_grasp_force": float(dict_get(raw, "haptics.force_min_grasp_force", 50.0)),
                "max_grasp_force": float(dict_get(raw, "haptics.force_max_grasp_force", 500.0)),
            },
        }

    # ── callbacks ────────────────────────────────────────────────────────────

    def _on_gesture_label(self, msg: Int32) -> None:
        label = msg.data
        now = _now()
        with self._lock:
            if label != self._gesture_label:
                self._gesture_label = label
                self._gesture_started_at = now
                if label != self._cfg["emg_wrist_toggle_label"]:
                    self._power_armed = True
            self._last_emg_time = now

    def _on_gesture_name(self, msg: String) -> None:
        with self._lock:
            self._gesture_name = msg.data
            self._last_emg_time = _now()

    def _on_confidence(self, msg: Float32) -> None:
        with self._lock:
            self._confidence = float(msg.data)
            self._last_emg_time = _now()

    def _on_proportional(self, msg: Float32) -> None:
        with self._lock:
            self._proportional = max(0.0, min(1.0, float(msg.data)))
            self._last_emg_time = _now()

    def _on_emg_age(self, msg: Float32) -> None:
        with self._lock:
            self._emg_age = float(msg.data)

    def _on_forces(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._forces = list(msg.data[:FINGER_COUNT]) if msg.data else [0.0] * FINGER_COUNT

    def _on_wrist_state(self, msg: Float64MultiArray) -> None:
        with self._lock:
            if msg.data:
                self._wrist_actual_deg = float(msg.data[0])
                self._got_wrist_state = True
                self._last_wrist_state_time = _now()
    def _on_force_source(self, msg: String) -> None:
        with self._lock:
            self._force_source = msg.data
            self._last_force_time = _now()
    def _on_joint_positions(self, msg: JointState) -> None:
        with self._lock:
            if msg.position:
                self._joint_positions = list(msg.position[:FINGER_COUNT])

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


    # ── EMG gating (legacy freshness / confidence / hold / power) ─────────

    def _active_gesture(self, label: int) -> bool:
        """True when *label* is the current gesture, fresh, and confident."""
        if self._last_emg_time <= 0.0:
            return False
        if _now() - self._last_emg_time > self._cfg["emg_stale_timeout_s"]:
            return False
        return self._gesture_label == label and self._confidence >= self._cfg["emg_confidence_threshold"]

    def _gesture_held(self, label: int, seconds: float) -> bool:
        """True when *label* has been the active gesture for at least *seconds*."""
        return self._gesture_label == label and (_now() - self._gesture_started_at) >= seconds

    def _open_safety_active(self) -> bool:
        """OPEN gesture must be active, proportional above threshold, and held."""
        label = self._cfg["emg_open_label"]
        return (
            self._active_gesture(label)
            and self._proportional >= self._cfg["emg_open_proportional_threshold"]
            and self._gesture_held(label, self._cfg["emg_open_hold_s"])
        )
    # ── state machine ────────────────────────────────────────────────────────

    def _update(self) -> None:
        with self._lock:
            stage = self._stage
            forces = self._forces

        if stage == Stage.INITIALISING:
            # Wait for hardware / settle.
            if self._stage_elapsed() > 1.0:
                self._enter(Stage.WAITING_FOR_ACTIVATION)
            return

        # Global stop gesture — must pass freshness, confidence, proportional, and hold gates
        if self._open_safety_active() and stage not in (
            Stage.OPENING_HAND,
            Stage.FAULT,
            Stage.RETURN_DELAY,
            Stage.RETURN_WRIST,
            Stage.COMPLETE,
        ):
            self._enter(Stage.OPENING_HAND, "OPEN gesture")
            return

        if stage == Stage.WAITING_FOR_ACTIVATION:
            act_label = self._cfg["emg_activation_label"]
            if self._active_gesture(act_label) and self._gesture_held(act_label, self._cfg["emg_activation_hold_s"]):
                self._enter(Stage.ROTATING_TO_VERTICAL, "POWER gesture")
            return

        if stage == Stage.ROTATING_TO_VERTICAL:
            cfg_wrist = self._cfg["wrist"]
            self._target_wrist = clamp(
                self._target_wrist + math.copysign(cfg_wrist["velocity_deg_s"] * self._dt, cfg_wrist["vertical_deg"] - self._target_wrist),
                cfg_wrist["min_deg"],
                cfg_wrist["max_deg"],
            )
            if self._wrist_target_complete():
                self._wrist_reached_since = None
                self._enter(Stage.VERTICAL_DELAY)
            return

        if stage == Stage.VERTICAL_DELAY:
            if self._stage_elapsed() >= self._cfg["timings"]["vertical_delay_s"]:
                self._enter(Stage.FORCE_CLOSING)
            return

        if stage == Stage.FORCE_CLOSING:
            if self._force_faulted():
                return
            stop = self._stop_position_reason()
            if stop:
                self._fault(f"max closure reached before contact: {stop}")
                return
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
            if self._force_faulted():
                return
            with self._lock:
                self._mode = "hold"
                self._hold_mode = "force" if not self._wrist_control else "wrist"
            self._handle_hold_adjustments()
            return

        if stage == Stage.OPENING_HAND:
            # Zero velocity, switch to position, reset wrist control (legacy _begin_open_release)
            with self._lock:
                self._mode = "position"
                self._target_force = list(self._cfg["open_positions"])
                self._wrist_control = False
                self._hold_mode = "force"
            elapsed = self._stage_elapsed()
            if elapsed >= self._cfg["timings"]["opening_min_s"]:
                self._enter(Stage.RETURN_DELAY)
            elif elapsed >= self._cfg["timings"]["opening_timeout_s"]:
                self._fault("opening hand timed out")
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
            if self._wrist_target_complete():
                self._wrist_reached_since = None
                self._enter(Stage.COMPLETE)
            return
        if stage == Stage.COMPLETE:
            with self._lock:
                self._mode = "position"
                self._target_force = list(self._cfg["open_positions"])
                self._enable = False
            # Zero haptics on entry (legacy _run_complete)
            if self._stage_elapsed() < self._cfg["complete_shutdown_delay_s"]:
                self._haptic_pub.publish(Float32MultiArray(data=[0.0] * MOTOR_COUNT))
            # POWER gesture restarts the test from the top
            act_label = self._cfg["emg_activation_label"]
            if self._active_gesture(act_label) and self._gesture_held(act_label, self._cfg["emg_activation_hold_s"]):
                with self._lock:
                    self._enable = True
                    self._wrist_control = False
                self._enter(Stage.ROTATING_TO_VERTICAL, "POWER restart")
                return
            # After delay, shut down
            if self._stage_elapsed() >= self._cfg["complete_shutdown_delay_s"]:
                self._finished = True
            return

        if stage == Stage.FAULT:
            with self._lock:
                self._mode = "position"
                self._target_force = list(self._cfg["open_positions"])
            # After recovery delay, attempt to re-open (legacy _run_fault_recovery)
            if self._stage_elapsed() >= 1.0:
                self.get_logger().info(f"Fault recovery: {self._fault_reason}")
                self._enter(Stage.OPENING_HAND, f"fault recovery: {self._fault_reason}")
            return

    def _fault(self, reason: str) -> None:
        """Transition to FAULT — zero velocity, log reason (legacy _fault)."""
        self._fault_reason = reason
        self.get_logger().error(f"FAULT: {reason}")
        self._vel_cmd_pub.publish(Float64MultiArray(data=[0.0] * FINGER_COUNT))
        self._enter(Stage.FAULT, reason)

    def _wrist_target_complete(self) -> bool:
        """Check if the wrist has reached the target position using actual feedback."""
        cfg_wrist = self._cfg["wrist"]
        if not cfg_wrist.get("enabled", True):
            return True
        now = _now()
        stale = (
            not self._got_wrist_state
            or now - self._last_wrist_state_time > cfg_wrist["stale_timeout_s"]
        )
        if not stale:
            error = abs(circ_delta_deg(self._target_wrist, self._wrist_actual_deg))
            if error <= cfg_wrist["position_tolerance_deg"]:
                if self._wrist_reached_since is None:
                    self._wrist_reached_since = now
                return now - self._wrist_reached_since >= cfg_wrist["settle_s"]
            self._wrist_reached_since = None
        if cfg_wrist.get("assume_target_on_timeout", True) and self._stage_elapsed() >= cfg_wrist["move_timeout_s"]:
            return True
        return False

    def _handle_hold_adjustments(self) -> None:
        now = _now()
        toggle_label = self._cfg["emg_wrist_toggle_label"]

        # Power-toggle with armed / rearm logic (legacy _maybe_toggle_hold_control)
        if not self._active_gesture(toggle_label):
            if self._gesture_label != toggle_label:
                self._power_armed = True
        elif self._power_armed:
            if now - self._last_power_toggle >= self._cfg["emg_power_rearm_s"]:
                if self._gesture_held(toggle_label, self._cfg["emg_power_toggle_hold_s"]):
                    self._power_armed = False
                    self._last_power_toggle = now
                    self._wrist_control = not self._wrist_control
                    self._publish_event("toggle", f"wrist_control={self._wrist_control}")

        if now - self._last_adjustment_time < self._adjustment_interval_s:
            return

        if self._wrist_control:
            cfg_wrist = self._cfg["wrist"]
            direction = 0.0
            if self._active_gesture(self._cfg["emg_wrist_positive_label"]):
                direction = 1.0
            elif self._active_gesture(self._cfg["emg_wrist_negative_label"]):
                direction = -1.0
            if direction != 0.0:
                step = direction * cfg_wrist["velocity_deg_s"] * max(self._proportional, 0.15) * self._dt
                self._target_wrist = clamp(self._target_wrist + step, cfg_wrist["min_deg"], cfg_wrist["max_deg"])
                self._last_adjustment_time = now
        else:
            delta = 0.0
            if self._active_gesture(self._cfg["emg_increase_force_label"]):
                delta = float(self._cfg["adjustment_rate_up"]) * self._proportional * self._dt
            elif self._active_gesture(self._cfg["emg_decrease_force_label"]):
                delta = -float(self._cfg["adjustment_rate_down"]) * self._proportional * self._dt
            if delta != 0.0:
                target_min = float(self._cfg["target_min"])
                target_max = float(self._cfg["target_max"])
                for i in range(FINGER_COUNT):
                    self._target_force[i] = clamp(
                        self._target_force[i] + delta,
                        target_min,
                        target_max,
                    )
                self._last_adjustment_time = now

    def _force_faulted(self) -> bool:
        """Check force safety — stale, unavailable, emergency (legacy _force_faulted)."""
        with self._lock:
            source = self._force_source
            forces = list(self._forces)
        elapsed = self._stage_elapsed()
        now = _now()

        if source == "none":
            if self._cfg["require_force_data"] and elapsed > self._cfg["force_stale_timeout_s"]:
                self._fault("force data unavailable")
                return True
            return False

        force_age = now - self._last_force_time
        if force_age > self._cfg["force_stale_timeout_s"]:
            self._fault("force data stale")
            return True

        if max(forces) >= self._cfg["emergency_threshold"]:
            self._vel_cmd_pub.publish(
                Float64MultiArray(data=[self._cfg["emergency_backoff_velocity_rad_s"]] * FINGER_COUNT)
            )
            self._fault(
                f"emergency force exceeded: max={max(forces):.1f} "
                f"threshold={self._cfg['emergency_threshold']:.1f}"
            )
            return True

        return False

    def _stop_position_reason(self) -> str:
        """Check if any joint reached max closure (legacy _stop_position_reason)."""
        with self._lock:
            positions = list(self._joint_positions)
        for i, name in enumerate(FINGER_JOINTS):
            if positions[i] >= self._cfg["max_closure_positions"][i]:
                return (
                    f"{name} position {positions[i]:.3f} >= "
                    f"{self._cfg['max_closure_positions'][i]:.3f}"
                )
        return ""

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

    def destroy_node(self) -> bool:
        """Shutdown: zero haptics, zero velocity, open hand, return wrist.

        Matches legacy destroy_node behavior.  Publishes are best-effort;
        if the ROS 2 context is already torn down the exceptions are swallowed.
        """
        try:
            self._haptic_pub.publish(Float32MultiArray(data=[0.0] * MOTOR_COUNT))
            self._vel_cmd_pub.publish(Float64MultiArray(data=[0.0] * FINGER_COUNT))
            self._pos_cmd_pub.publish(Float64MultiArray(data=list(self._cfg["open_positions"])))
            self._wrist_cmd_pub.publish(
                Float64MultiArray(data=[float(self._cfg["wrist"]["horizontal_deg"]), 0.0])
            )
        except Exception:
            pass
        try:
            self._publish_event("shutdown", "node destroyed")
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config-path", default=None)
    known, remaining = parser.parse_known_args(args or [])
    rclpy.init(args=remaining)
    node = SupervisorNode(config_path=known.config_path)

    def _signal_handler(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _signal_handler)

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
