#!/usr/bin/env python3
"""MVP-8UV.17: terminal_ui_node.

Lightweight status printer at ~2 Hz that matches the legacy monolithic
script's terminal output layout.  Subscribes to the full topic contract
and renders ANSI-coloured status to the controlling terminal.

The UI writes directly to ``/dev/tty`` so it owns a clean screen region
even while ``ros2 launch`` multiplexes every other node's stdout into
the same terminal.  When ``/dev/tty`` is unavailable (e.g. CI) it falls
back to stdout.
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
from scripts.mia_haptic_force_test.common.ui_render import (
    ANSI_CLEAR,
    UiSnapshot,
    render_terminal_frame,
)

_BOLD = "\033[1m"
_DIM = "\033[2m"
_GREEN = "\033[92m"
_YELLOW = "\033[93m"
_RED = "\033[91m"
_RESET = "\033[0m"
_HIDE_CURSOR = "\033[?25l"
_SHOW_CURSOR = "\033[?25h"
_TTY_DEVICE = "/dev/tty"
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
        self._run_id: str = os.environ.get("MIA_HAPTIC_FORCE_TEST_RUN_ID", "")
        self._gesture_label: int = 0

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
        # ── Terminal handle (private TTY channel) ─────────────────────────
        # ROS 2's launch multiplexes every node's stdout into one prefixed
        # stream, which makes a redrawing TUI unreadable: each frame's
        # clear-screen ANSI collides with other nodes' log lines.  We open
        # the controlling terminal directly so the UI owns a clean channel
        # and other nodes' logs scroll past it normally.  Falls back to
        # stdout when /dev/tty is unavailable (e.g. CI without a TTY).
        self._tty_fd: Optional[int] = None
        self._tty_owned: bool = False
        try:
            fd = os.open(_TTY_DEVICE, os.O_WRONLY | os.O_NOCTTY)
            self._tty_fd = fd
            self._tty_owned = True
        except OSError:
            self._tty_fd = None
            self._tty_owned = False
        if self._tty_fd is not None:
            try:
                os.write(self._tty_fd, _HIDE_CURSOR.encode("utf-8"))
            except OSError:
                pass

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
    def _build_snapshot(self) -> UiSnapshot:
        """Build a ``UiSnapshot`` from the guarded node state.

        Holds ``self._lock`` just long enough to copy the fields the
        renderer needs, so the DDS callbacks are never blocked.
        """
        with self._lock:
            forces = tuple(self._forces[: len(FINGER_LABELS)])
            targets = tuple(self._target_force[: len(FINGER_LABELS)])
            errors = tuple(self._force_error[: len(FINGER_LABELS)])
            haptics = tuple(self._haptics[:MOTOR_COUNT])
            joints = tuple(self._joint_positions[: len(FINGER_LABELS)])
            emg_age = (
                time.monotonic() - self._last_emg_time
                if self._last_emg_time > 0.0
                else -1.0
            )
            snapshot = UiSnapshot(
                config_name=self._config_name,
                run_id=self._run_id,
                stage=self._stage,
                stage_elapsed_s=time.monotonic() - self._stage_changed_at,
                stage_countdown_s=self._countdown_s(
                    self._stage, time.monotonic() - self._stage_changed_at
                ),
                mode=self._mode,
                hold_mode=self._hold_mode,
                enable=self._enable,
                control_hint=self._control_hint(self._stage, self._mode, self._hold_mode),
                gesture=self._gesture,
                gesture_label=self._gesture_label,
                confidence=self._confidence,
                proportional=self._proportional,
                emg_age_s=emg_age,
                confidence_threshold=self._confidence_threshold,
                forces=forces,
                target_forces=targets,
                force_errors=errors,
                force_min=self._force_min,
                force_max=self._force_max,
                joint_positions=joints,
                wrist_position_deg=self._wrist_position,
                wrist_velocity_deg_s=self._wrist_velocity,
                wrist_target_deg=self._target_wrist,
                haptics=haptics,
                haptic_phase=self._derive_haptic_phase(
                    self._stage, list(haptics), 0.0
                ),
                contact_reason=self._contact_reason,
                fault_reason=self._fault_reason,
            )
        return snapshot

    def _write(self, payload: str, *, sink=None) -> None:
        """Write a UI frame to the controlling TTY (or stdout fallback).

        Targets the dedicated ``/dev/tty`` handle when available so the
        UI is not interleaved with other nodes' stdout, and ends every
        frame with a newline so the next ``2J`` clears cleanly.

        ``sink`` is an optional writable object (e.g. ``BytesIO``) used
        by the offline test suite to capture frames without a TTY.
        """
        if sink is not None:
            try:
                sink.write(payload.encode("utf-8", errors="replace"))
                return
            except Exception:
                pass
        if self._tty_fd is not None:
            try:
                os.write(self._tty_fd, payload.encode("utf-8", errors="replace"))
                return
            except OSError:
                self._tty_fd = None
        try:
            sys.stdout.write(payload)
            sys.stdout.flush()
        except Exception:
            pass

    def _render(self) -> None:
        """Clear and repaint the terminal using the pure renderer."""
        snapshot = self._build_snapshot()
        frame = ANSI_CLEAR + render_terminal_frame(snapshot, color=True)
        self._write(frame)

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
        if self._tty_fd is not None:
            try:
                os.write(self._tty_fd, (_SHOW_CURSOR + _RESET).encode("utf-8"))
            except OSError:
                pass
            if self._tty_owned:
                try:
                    os.close(self._tty_fd)
                except OSError:
                    pass
            self._tty_fd = None
            self._tty_owned = False
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
