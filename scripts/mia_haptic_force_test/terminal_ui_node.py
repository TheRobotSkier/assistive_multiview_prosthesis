#!/usr/bin/env python3
"""MVP-8UV.17: terminal_ui_node.

Lightweight status printer at ~2 Hz that matches the legacy monolithic
script's terminal output layout.  Subscribes to the full topic contract
and renders ANSI-coloured status to stdout.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray, String

_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and __package__ is None and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.constants import (
    FINGER_LABELS,
    MOTOR_COUNT,
    TOPIC_CONTROL_HOLD_MODE,
    TOPIC_CONTROL_MODE,
    TOPIC_CONTROL_TARGET_FORCE,
    TOPIC_CONTROL_TARGET_WRIST,
    TOPIC_CONTROLLER_FORCE_ERROR,
    TOPIC_EMG_CONFIDENCE,
    TOPIC_EMG_GESTURE,
    TOPIC_EMG_PROPORTIONAL,
    TOPIC_HAND_FORCES,
    TOPIC_HAPTIC_BAND_MOTORS,
    TOPIC_TEST_EVENT,
    TOPIC_TEST_STAGE,
    TOPIC_WRIST_STATE,
)
from scripts.mia_haptic_force_test.common.conversions import circ_delta_deg, hold_velocity

# ── ANSI helpers ──────────────────────────────────────────────────────────────

_BOLD = "\033[1m"
_DIM = "\033[2m"
_GREEN = "\033[92m"
_YELLOW = "\033[93m"
_RED = "\033[91m"
_RESET = "\033[0m"
_CLEAR = "\033[2J\033[H"

# ── Config defaults ───────────────────────────────────────────────────────────

_DEFAULT_CONFIDENCE_THRESHOLD = 0.55
_DEFAULT_OPEN_HOLD_S = 1.0
_DEFAULT_ACTIVATION_HOLD_S = 0.2
_DEFAULT_VERTICAL_DELAY_S = 3.0
_DEFAULT_RETURN_DELAY_S = 1.0
_DEFAULT_WRIST_VELOCITY_DEG_S = 30.0
_DEFAULT_HOLD_DEADZONE = 20.0
_DEFAULT_HOLD_MAX_VEL = 0.08
_DEFAULT_HOLD_MIN_OS = 20.0
_DEFAULT_HOLD_MAX_OS = 150.0
_DEFAULT_FORCE_ADJUSTMENT_RATE_UP = 100.0
_DEFAULT_FORCE_ADJUSTMENT_RATE_DOWN = 200.0


def _load_ui_config(config_path: Optional[str] = None) -> dict[str, Any]:
    """Load the haptic-force-test YAML config for UI constants."""
    import yaml

    path = config_path or os.path.join(_REPO_ROOT, "config", "mia_haptic_force_test.yaml")
    try:
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
    except Exception:
        raw = {}
    return raw


class TerminalUINode(Node):
    """Terminal status UI matching the legacy layout."""

    def __init__(self, config_path: Optional[str] = None) -> None:
        super().__init__("terminal_ui_node")
        self._config_path = config_path or os.path.join(_REPO_ROOT, "config", "mia_haptic_force_test.yaml")
        self.declare_parameter("refresh_rate_hz", 2.0)
        rate_hz = float(self.get_parameter("refresh_rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 0.1)

        # ── Config snapshot for display values ────────────────────────────
        cfg = _load_ui_config(self._config_path)
        emg = cfg.get("emg", {})
        wrist = cfg.get("wrist", {})
        hand = cfg.get("hand", {})
        force = cfg.get("force", {})
        haptics = cfg.get("haptics", {})
        self._config_name = os.path.basename(self._config_path)
        self._confidence_threshold: float = float(emg.get("confidence_threshold", _DEFAULT_CONFIDENCE_THRESHOLD))
        self._open_hold_s: float = float(emg.get("open_hold_s", _DEFAULT_OPEN_HOLD_S))
        self._activation_hold_s: float = float(emg.get("activation_hold_s", _DEFAULT_ACTIVATION_HOLD_S))
        self._vertical_delay_s: float = float(wrist.get("vertical_delay_s", _DEFAULT_VERTICAL_DELAY_S))
        self._return_delay_s: float = float(wrist.get("return_after_open_delay_s", _DEFAULT_RETURN_DELAY_S))
        self._wrist_velocity_deg_s: float = float(wrist.get("control_velocity_deg_s", _DEFAULT_WRIST_VELOCITY_DEG_S))
        self._hold_deadzone: float = float(hand.get("hold_deadzone", _DEFAULT_HOLD_DEADZONE))
        self._hold_max_vel: float = float(hand.get("hold_max_velocity_rad_s", _DEFAULT_HOLD_MAX_VEL))
        self._hold_min_os: float = float(hand.get("hold_min_overshoot", _DEFAULT_HOLD_MIN_OS))
        self._hold_max_os: float = float(hand.get("hold_max_overshoot", _DEFAULT_HOLD_MAX_OS))
        self._force_adjustment_rate_up: float = float(
            force.get("adjustment_rate_up", _DEFAULT_FORCE_ADJUSTMENT_RATE_UP)
        )
        self._force_adjustment_rate_down: float = float(
            force.get("adjustment_rate_down", _DEFAULT_FORCE_ADJUSTMENT_RATE_DOWN)
        )
        self._force_min: float = float(haptics.get("force_min_grasp_force", 50.0))
        self._force_max: float = float(haptics.get("force_max_grasp_force", 500.0))

        # ── Shared state (all guarded by _lock) ──────────────────────────
        self._lock = threading.Lock()
        self._start_time: float = time.monotonic()
        self._stage: str = "initialising"
        self._stage_changed_at: float = self._start_time
        self._prev_stage: str = "initialising"
        self._mode: str = "velocity"
        self._hold_mode: str = "force"
        self._gesture: str = "REST"
        self._confidence: float = 0.0
        self._proportional: float = 0.0
        self._last_emg_time: float = 0.0
        self._forces: list[float] = [0.0] * len(FINGER_LABELS)
        self._target_force: list[float] = [0.0] * len(FINGER_LABELS)
        self._target_wrist: float = 0.0
        self._wrist_position: float = 0.0
        self._wrist_velocity: float = 0.0
        self._force_error: list[float] = [0.0] * len(FINGER_LABELS)
        self._haptics: list[float] = [0.0] * MOTOR_COUNT
        self._joint_positions: list[float] = [0.0] * len(FINGER_LABELS)
        self._contact_reason: str = ""
        self._fault_reason: str = ""

        # ── Subscriptions ────────────────────────────────────────────────
        self.create_subscription(String, TOPIC_TEST_STAGE, self._on_stage, 10)
        self.create_subscription(String, TOPIC_EMG_GESTURE, self._on_gesture, 10)
        self.create_subscription(Float32, TOPIC_EMG_CONFIDENCE, self._on_confidence, 10)
        self.create_subscription(Float32, TOPIC_EMG_PROPORTIONAL, self._on_proportional, 10)
        self.create_subscription(Float32MultiArray, TOPIC_HAND_FORCES, self._on_forces, 10)
        self.create_subscription(Float64MultiArray, TOPIC_CONTROL_TARGET_FORCE, self._on_target_force, 10)
        self.create_subscription(Float64MultiArray, TOPIC_CONTROL_TARGET_WRIST, self._on_target_wrist, 10)
        self.create_subscription(String, TOPIC_CONTROL_MODE, self._on_string("mode"), 10)
        self.create_subscription(String, TOPIC_CONTROL_HOLD_MODE, self._on_string("hold_mode"), 10)
        self.create_subscription(Float64MultiArray, TOPIC_WRIST_STATE, self._on_wrist_state, 10)
        self.create_subscription(Float64MultiArray, TOPIC_CONTROLLER_FORCE_ERROR, self._on_force_error, 10)
        self.create_subscription(Float32MultiArray, TOPIC_HAPTIC_BAND_MOTORS, self._on_haptics, 10)
        self.create_subscription(JointState, "/hand/joint_states", self._on_joint_states, 10)
        self.create_subscription(String, TOPIC_TEST_EVENT, self._on_event, 10)

        # ── Render thread ────────────────────────────────────────────────
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="ui_loop")
        self._thread.start()

    # ── Callback factories ───────────────────────────────────────────────

    def _on_string(self, key: str):
        def cb(msg: String) -> None:
            with self._lock:
                setattr(self, f"_{key}", msg.data)
        return cb

    # ── Individual callbacks ─────────────────────────────────────────────

    def _on_stage(self, msg: String) -> None:
        with self._lock:
            if msg.data != self._stage:
                self._prev_stage = self._stage
                self._stage = msg.data
                self._stage_changed_at = time.monotonic()

    def _on_gesture(self, msg: String) -> None:
        with self._lock:
            self._gesture = msg.data
            self._last_emg_time = time.monotonic()

    def _on_confidence(self, msg: Float32) -> None:
        with self._lock:
            self._confidence = float(msg.data)

    def _on_proportional(self, msg: Float32) -> None:
        with self._lock:
            self._proportional = float(msg.data)

    def _on_forces(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._forces = list(msg.data[: len(FINGER_LABELS)])

    def _on_target_force(self, msg: Float64MultiArray) -> None:
        with self._lock:
            self._target_force = list(msg.data[: len(FINGER_LABELS)])

    def _on_target_wrist(self, msg: Float64MultiArray) -> None:
        with self._lock:
            if msg.data:
                self._target_wrist = float(msg.data[0])

    def _on_wrist_state(self, msg: Float64MultiArray) -> None:
        with self._lock:
            if len(msg.data) >= 2:
                self._wrist_position = float(msg.data[0])
                self._wrist_velocity = float(msg.data[1])
            elif msg.data:
                self._wrist_position = float(msg.data[0])

    def _on_force_error(self, msg: Float64MultiArray) -> None:
        with self._lock:
            self._force_error = list(msg.data[: len(FINGER_LABELS)])

    def _on_haptics(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._haptics = list(msg.data[:MOTOR_COUNT])

    def _on_joint_states(self, msg: JointState) -> None:
        with self._lock:
            self._joint_positions = list(msg.position[: len(FINGER_LABELS)])

    def _on_event(self, msg: String) -> None:
        """Parse event JSON and track contact / fault reasons."""
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        event = payload.get("event", "")
        detail = payload.get("detail", "")
        with self._lock:
            if "contact" in event.lower():
                self._contact_reason = detail
            elif "fault" in event.lower():
                self._fault_reason = detail

    # ── Render loop ──────────────────────────────────────────────────────

    def _loop(self) -> None:
        while rclpy.ok() and self._running:
            t0 = time.monotonic()
            self._render()
            remaining = self._dt - (time.monotonic() - t0)
            if remaining > 0.0:
                time.sleep(remaining)

    # ── UI rendering ─────────────────────────────────────────────────────

    def _render(self) -> None:
        """Clear and repaint the terminal, matching legacy layout."""
        sys.stdout.write(_CLEAR)

        with self._lock:
            stage = self._stage
            mode = self._mode
            hold = self._hold_mode
            gesture = self._gesture
            conf = self._confidence
            prop = self._proportional
            emg_age = (
                time.monotonic() - self._last_emg_time if self._last_emg_time > 0.0 else 999.0
            )
            forces = list(self._forces)
            targets = list(self._target_force)
            wrist_pos = self._wrist_position
            wrist_target = self._target_wrist
            haptics = list(self._haptics)
            contact = self._contact_reason
            fault = self._fault_reason

        now = time.monotonic()
        elapsed = now - self._start_time
        stage_elapsed = now - self._stage_changed_at
        fc = len(FINGER_LABELS)

        lines: list[str] = []

        # ── Header ───────────────────────────────────────────────────────
        lines.append("Mia Hand EMG Haptic Force Test")
        lines.append(f"Config: {self._config_name}")

        # ── Control hint ─────────────────────────────────────────────────
        hint = self._control_hint(stage, mode, hold)
        if hint:
            lines.append(f"Controls: {hint}")

        lines.append("------")

        # ── Dynamic info ─────────────────────────────────────────────────
        lines.append(f"  Elapsed      {elapsed:.1f}s")

        # Countdown for timed stages
        countdown = self._countdown_s(stage, stage_elapsed)
        if countdown is not None:
            lines.append(f"  Countdown    {countdown:.1f}s")

        # State + mode
        stage_display = stage.replace("_", " ")
        lines.append(f"  State        {_BOLD}{stage_display}{_RESET}  ·  {mode}")

        # EMG — green when above confidence threshold, dim otherwise
        if conf >= self._confidence_threshold:
            lines.append(
                f"  EMG          {_GREEN}{gesture}{_RESET}"
                f"  conf={conf:.2f}  prop={prop:.2f}  age={emg_age:.3f}s"
            )
        else:
            lines.append(
                f"  EMG          {_DIM}{gesture}{_RESET}"
                f"  conf={conf:.2f}  prop={prop:.2f}  age={emg_age:.3f}s"
            )

        # Force: avg, max, target, haptic%
        avg_f = sum(forces) / fc if fc else 0.0
        max_f = max(forces) if forces else 0.0
        target_avg = sum(targets) / fc if fc else 0.0
        fpct = self._force_percent(avg_f)
        vel_str = ""
        if stage == "force_hold" and forces:
            vels = [
                hold_velocity(
                    forces[i],
                    targets[i],
                    self._hold_deadzone,
                    self._hold_max_vel,
                    self._hold_min_os,
                    self._hold_max_os,
                )
                for i in range(fc)
            ]
            if any(abs(v) > 0.001 for v in vels):
                vel_str = (
                    f"  {_YELLOW}vel=[{vels[0]:+.3f}"
                    f" {vels[1]:+.3f}"
                    f" {vels[2]:+.3f}]{_RESET}"
                )
        lines.append(
            f"  Force        avg={avg_f:.1f}  max={max_f:.1f}"
            f"  target={target_avg:.1f}  haptic={fpct:.0f}%{vel_str}"
        )

        # Wrist: actual → target (error)
        w_err = circ_delta_deg(wrist_target, wrist_pos)
        lines.append(
            f"  Wrist        {wrist_pos:.1f}\u00b0 \u2192 {wrist_target:.1f}\u00b0"
            f"  (err={w_err:+.1f}\u00b0)"
        )

        # Adjustment status (FORCE_HOLD only)
        if stage == "force_hold":
            if conf >= self._confidence_threshold and gesture in ("FLEXION", "EXTENSION"):
                if hold == "force":
                    if gesture == "FLEXION":
                        arrow = "\u25b2"
                        delta = prop * self._force_adjustment_rate_up
                    else:
                        arrow = "\u25bc"
                        delta = -prop * self._force_adjustment_rate_down
                    lines.append(
                        f"  Adjust       {_YELLOW}{arrow} adjusting force{_RESET}"
                        f"  \u0394={delta:+.1f}/s"
                    )
                else:
                    vel = self._wrist_velocity_deg_s * max(prop, 0.15)
                    lines.append(
                        f"  Adjust       {_YELLOW}\u25c0\u25b6 adjusting wrist{_RESET}"
                        f"  vel={vel:.1f}\u00b0/s"
                    )
            else:
                lines.append(
                    f"  Adjust       idle"
                    f"  (need conf \u2265 {self._confidence_threshold:.2f}, got {conf:.2f})"
                )

        # Haptics
        haptic_phase = self._derive_haptic_phase(stage, haptics, fpct)
        haptic_str = " ".join(f"{v:.0f}" for v in haptics)
        lines.append(f"  Haptics      {haptic_phase:20s} [{haptic_str}]")

        # ── Footer ───────────────────────────────────────────────────────
        lines.append("------")
        lines.append(
            f"  Gesture      {gesture}  conf={conf:.2f}  prop={prop:.2f}"
        )
        if contact:
            lines.append(f"  Contact      {contact}")
        if fault:
            lines.append(f"  {_RED}Fault        {fault}{_RESET}")

        print("\n".join(lines), flush=True)

    # ── UI helpers ───────────────────────────────────────────────────────

    def _countdown_s(self, stage: str, stage_elapsed: float) -> Optional[float]:
        """Return remaining countdown seconds for timed stages, or None."""
        if stage == "vertical_delay":
            return max(0.0, self._vertical_delay_s - stage_elapsed)
        if stage == "return_delay":
            return max(0.0, self._return_delay_s - stage_elapsed)
        return None

    def _control_hint(self, stage: str, mode: str, hold: str) -> str:
        """Single-line control hint matching legacy _ui_control_hint."""
        open_s = self._open_hold_s
        if stage == "waiting_for_activation":
            return (
                f"keep hand open; POWER held {self._activation_hold_s:.1f}s starts;"
                f" OPEN held {open_s:.1f}s stops"
            )
        if stage == "rotating_to_vertical":
            return f"keep hand open; OPEN held {open_s:.1f}s stops"
        if stage == "vertical_delay":
            return f"OPEN held {open_s:.1f}s stops; closure starts after countdown"
        if stage == "force_closing":
            return f"OPEN held {open_s:.1f}s stops; waiting for force threshold"
        if stage == "force_hold" and hold == "force":
            return (
                f"FLEXION \u2191force  EXTENSION \u2193force"
                f"  POWER\u2192wrist  OPEN held {open_s:.1f}s stops"
            )
        if stage == "force_hold" and hold == "wrist":
            return (
                f"FLEXION/EXTENSION move wrist"
                f"  POWER\u2192force  OPEN held {open_s:.1f}s stops"
            )
        if stage == "opening_hand":
            return "opening hand before wrist return"
        if stage == "return_delay":
            return "waiting before returning wrist horizontal"
        if stage == "return_wrist":
            return "returning wrist to horizontal"
        if stage == "complete":
            return "complete; haptics zeroed, shutdown pending"
        if stage == "fault":
            return f"fault recovery underway: {self._fault_reason}"
        return f"OPEN held {open_s:.1f}s stops"

    def _force_percent(self, avg_force: float) -> float:
        """Approximate haptic force percentage (matches legacy logic)."""
        lo = self._force_min
        hi = self._force_max
        if hi <= lo:
            return 0.0
        return max(0.0, min(100.0, (avg_force - lo) / (hi - lo) * 100.0))

    def _derive_haptic_phase(
        self, stage: str, haptics: list[float], fpct: float
    ) -> str:
        """Derive a human-readable haptic phase label."""
        if any(v > 0.0 for v in haptics):
            if stage == "force_hold":
                return f"force:all_motors"
            if stage in (
                "rotating_to_vertical",
                "vertical_delay",
                "return_wrist",
            ):
                return "wrist_angle"
            return "event_buzz"
        if stage in ("initialising", "complete", "fault"):
            return "disabled"
        return "idle_zero"

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def destroy_node(self) -> None:
        self.stop()
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config-path", default=None)
    known, remaining = parser.parse_known_args(args or [])
    rclpy.init(args=remaining)
    node = TerminalUINode(config_path=known.config_path)

    def _signal_handler(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _signal_handler)

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
