#!/usr/bin/env python3
"""MVP-8U7/8UV.15: haptic_node.

Maps test stage, wrist angle, and grasp-force information to haptic-band motor
intensities.  Publishes ``/haptic_band/motors`` (std_msgs/Float32MultiArray) at
~10 Hz.

Behaviour matches the legacy ``_compute_haptics`` / ``_trigger_buzz`` from
``scripts/mia_haptic_force_test.py`` (lines 1344–1375, 1137–1147).
"""

from __future__ import annotations

import argparse
import json
import os
import signal
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

from scripts.mia_haptic_force_test.common.constants import (
    FINGER_COUNT,
    MOTOR_COUNT,
    TOPIC_CONTROL_HOLD_MODE,
    TOPIC_EMG_GESTURE,
    TOPIC_TEST_EVENT,
    TOPIC_TEST_STAGE,
)
from scripts.mia_haptic_force_test.common.conversions import clamp, force_haptics, wrist_haptics


# ── Buzz-event stage mapping ────────────────────────────────────────────────
# Maps stage names (as published on /test/stage) to (config_key_prefix,
# default_duration, default_intensity).  Legacy _trigger_buzz calls:
#   activation          → rotating_to_vertical entry
#   force_closure_start → force_closing entry
#   force_threshold_crossed → force_hold entry
_BUZZ_MAP: dict[str, tuple[str, float, float]] = {
    "rotating_to_vertical": ("activation", 0.12, 12.0),
    "force_closing": ("closure_start", 0.15, 22.0),
    "force_hold": ("contact", 0.20, 35.0),
}

# Stages where wrist-angle haptics apply (legacy ``wrist_stages``).
_WRIST_STAGES: frozenset[str] = frozenset({
    "rotating_to_vertical",
    "vertical_delay",
    "return_wrist",
})


class HapticNode(Node):
    """Haptic feedback generator."""

    def __init__(self, config_path: Optional[str] = None) -> None:
        super().__init__("haptic_node")
        self._config_path = config_path or os.path.join(
            _REPO_ROOT, "config", "mia_haptic_force_test.yaml"
        )
        self.declare_parameter("publish_rate_hz", 10.0)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)

        # ── Publishers ──────────────────────────────────────────────────────
        self._pub = self.create_publisher(Float32MultiArray, "/haptic_band/motors", 10)

        # ── Subscriptions ───────────────────────────────────────────────────
        self.create_subscription(String, TOPIC_TEST_STAGE, self._on_stage, 10)
        self.create_subscription(String, TOPIC_TEST_EVENT, self._on_event, 10)
        self.create_subscription(String, TOPIC_CONTROL_HOLD_MODE, self._on_hold_mode, 10)
        self.create_subscription(String, TOPIC_EMG_GESTURE, self._on_gesture, 10)
        self.create_subscription(Float32, "/emg/proportional", self._on_proportional, 10)
        self.create_subscription(Float64MultiArray, "/wrist/state", self._on_wrist_state, 10)
        self.create_subscription(Float32MultiArray, "/hand/forces", self._on_forces, 10)

        # ── State ───────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._stage: str = "initialising"
        self._hold_mode: str = "force"  # "force" or "wrist"
        self._gesture: str = "REST"
        self._proportional: float = 0.0
        self._wrist_angle: float = 0.0
        self._forces: list[float] = [0.0] * FINGER_COUNT

        # Buzz state (matches legacy _buzz_until / _buzz_values / _buzz_label)
        self._buzz_until: float = 0.0
        self._buzz_values: list[float] = [0.0] * MOTOR_COUNT
        self._buzz_label: str = ""

        # Phase tracking (matches legacy _haptic_phase)
        self._haptic_phase: str = "idle_zero"

        # Last non-zero haptics (for publish_zero_when_idle=false fallback)
        self._last_haptics: list[float] = [0.0] * MOTOR_COUNT

        # ── Config ──────────────────────────────────────────────────────────
        self._cfg: dict = self._load_config()

        # ── Loop thread ─────────────────────────────────────────────────────
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="haptic_loop")
        self._thread.start()

    # ── Config loading ──────────────────────────────────────────────────────

    def _load_config(self) -> dict:
        """Load the ``haptics:`` section from YAML (and keep
        ``haptic_percent_source`` from ``force:`` if present)."""
        import yaml

        path = self._config_path
        try:
            with open(path) as f:
                raw = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(f"Config load failed ({path}): {exc}")
            raw = {}
        haptics = dict(raw.get("haptics", {}))
        # Pull the force percent source strategy from the top-level force config.
        haptics.setdefault(
            "haptic_percent_source",
            raw.get("force", {}).get("haptic_percent_source", "measured_average"),
        )
        return haptics

    # ── ROS callbacks ───────────────────────────────────────────────────────

    def _on_stage(self, msg: String) -> None:
        with self._lock:
            new_stage = msg.data
            if new_stage != self._stage:
                self._stage = new_stage
                self._trigger_buzz_for_stage(new_stage)

    def _on_event(self, msg: String) -> None:
        """Parse /test/event JSON and trigger buzz on stage_change events.

        The supervisor publishes ``{"event": "stage_change", "detail": "..."}``
        on every transition.  This provides a second trigger path (in addition
        to ``/test/stage``) so that haptic feedback is guaranteed even if stage
        messages arrive slightly out of order.
        """
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if payload.get("event") != "stage_change":
            return
        detail: str = payload.get("detail", "")
        # detail format: "old_stage -> new_stage: reason" or "old_stage -> new_stage"
        arrow = detail.find(" -> ")
        if arrow < 0:
            return
        after = detail[arrow + 4:]
        colon = after.find(":")
        new_stage = after[:colon] if colon >= 0 else after
        new_stage = new_stage.strip()
        with self._lock:
            if new_stage != self._stage:
                self._stage = new_stage
                self._trigger_buzz_for_stage(new_stage)

    def _on_hold_mode(self, msg: String) -> None:
        with self._lock:
            self._hold_mode = msg.data

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
            self._forces = list(msg.data[:FINGER_COUNT]) if msg.data else [0.0] * FINGER_COUNT

    # ── Buzz triggering (matches legacy _trigger_buzz) ──────────────────────

    def _trigger_buzz_for_stage(self, stage: str) -> None:
        """Set buzz deadline + values when a stage transition occurs.

        Mirrors the three ``_trigger_buzz`` call sites in the legacy script:
        * activation          → ``rotating_to_vertical`` entry
        * force_closure_start → ``force_closing`` entry
        * force_threshold_crossed → ``force_hold`` entry
        """
        mapping = _BUZZ_MAP.get(stage)
        if mapping is None:
            return
        prefix, default_dur, default_int = mapping
        duration_s = float(self._cfg.get(f"{prefix}_buzz_duration_s", default_dur))
        intensity_pct = float(self._cfg.get(f"{prefix}_buzz_intensity_pct", default_int))

        motor_count = int(self._cfg.get("motor_count", MOTOR_COUNT))
        event_indices = self._cfg.get("event_motor_indices", list(range(motor_count)))
        event_set = set(int(i) for i in event_indices)
        values = [0.0] * motor_count
        clamped_int = clamp(intensity_pct, 0.0, 100.0)
        for i in range(motor_count):
            if i in event_set:
                values[i] = clamped_int
        self._buzz_label = f"{prefix}_buzz"
        self._buzz_values = values
        self._buzz_until = time.monotonic() + max(0.0, duration_s)

    # ── Force percentage (matches legacy _force_percent) ────────────────────

    def _force_percent(self, cfg: dict) -> float:
        """Compute the grasp-force percentage for haptic mapping.

        Respects the ``haptic_percent_source`` config key:
        * ``"measured_average"`` (default) — average of measured finger forces
        * ``"measured_max"``               — max of measured finger forces
        * ``"target_average"``             — not available in split-node
          (falls back to measured_average)
        """
        source = cfg.get("haptic_percent_source", "measured_average")
        forces = self._forces

        if source == "measured_max" and forces:
            value = max(forces)
        else:
            # measured_average / target_average (target not available → average)
            value = sum(forces) / FINGER_COUNT

        lo = float(cfg.get("force_min_grasp_force", 50.0))
        hi = float(cfg.get("force_max_grasp_force", 500.0))
        if hi <= lo:
            return 0.0
        return clamp((value - lo) / (hi - lo) * 100.0, 0.0, 100.0)

    # ── Motor computation (matches legacy _compute_haptics) ─────────────────

    def _compute_motors(self) -> list[float]:
        """Core haptic logic — must match legacy ``_compute_haptics``."""
        motor_count = int(self._cfg.get("motor_count", MOTOR_COUNT))
        now = time.monotonic()

        # 1. Buzz overrides everything.
        if now < self._buzz_until:
            self._haptic_phase = f"buzz:{self._buzz_label}"
            return list(self._buzz_values)
        self._buzz_label = ""

        # 2. Disabled check.
        if not bool(self._cfg.get("enabled", True)):
            self._haptic_phase = "disabled"
            return [0.0] * motor_count

        with self._lock:
            stage = self._stage
            hold_mode = self._hold_mode
            wrist_angle = self._wrist_angle
            forces = list(self._forces)

        cfg = self._cfg

        # 3. Wrist haptics: wrist stages OR force_hold + wrist mode.
        if stage in _WRIST_STAGES or (stage == "force_hold" and hold_mode == "wrist"):
            self._haptic_phase = "wrist_angle"
            result = wrist_haptics(wrist_angle, cfg)
            self._last_haptics = list(result)
            return result

        # 4. Force haptics: force_hold + force mode ONLY (not force_closing).
        if stage == "force_hold" and hold_mode == "force":
            pct = self._force_percent(cfg)
            values, phase = force_haptics(pct, cfg)
            self._haptic_phase = f"force:{phase}"
            self._last_haptics = list(values)
            return values

        # 5. Default: idle.
        self._haptic_phase = "idle_zero"
        if bool(cfg.get("publish_zero_when_idle", True)):
            return [0.0] * motor_count
        return list(self._last_haptics)

    # ── Publish loop ────────────────────────────────────────────────────────

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

    def destroy_node(self) -> None:
        self.stop()
        self._pub.publish(Float32MultiArray(data=[0.0] * MOTOR_COUNT))
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config-path", default=None)
    known, remaining = parser.parse_known_args(args or [])
    rclpy.init(args=remaining)
    node = HapticNode(config_path=known.config_path)

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
