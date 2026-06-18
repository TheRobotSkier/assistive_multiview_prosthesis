#!/usr/bin/env python3
"""MVP-6GI: logger_node.

Asynchronously logs a configurable set of topics to CSV in legacy-compatible
format.  Subscriptions are kept lightweight; rows are written in a background
thread so publishers are never blocked.

Output directory and file names are read from the YAML config's ``logging``
section, matching the legacy ``CsvLogger`` behaviour exactly.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import (
    Bool,
    Float32,
    Float32MultiArray,
    Float64,
    Float64MultiArray,
    Int32,
    String,
)

# ── sys.path bootstrap for direct execution ──────────────────────────────
_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from mia_hand_msgs.msg import ForceData, JointData, MotorData

from scripts.mia_haptic_force_test.common.constants import (
    FINGER_LABELS,
    MOTOR_COUNT,
    TOPIC_CONTROL_ENABLE,
    TOPIC_CONTROL_HOLD_MODE,
    TOPIC_CONTROL_MODE,
    TOPIC_CONTROL_TARGET_FORCE,
    TOPIC_CONTROL_TARGET_WRIST,
    TOPIC_EMG_CONFIDENCE,
    TOPIC_EMG_GESTURE,
    TOPIC_EMG_GESTURE_LABEL,
    TOPIC_EMG_PROPORTIONAL,
    TOPIC_GROUP_VEL_FF_COMMANDS,
    TOPIC_HAPTIC_BAND_MOTORS,
    TOPIC_HAND_FORCES,
    TOPIC_HAND_FORCE_SOURCE,
    TOPIC_HAND_JOINT_STATES,
    TOPIC_HW_FINGER_FORCES,
    TOPIC_HW_JOINT_POSITIONS,
    TOPIC_HW_JOINT_SPEEDS,
    TOPIC_HW_MOTOR_CURRENTS,
    TOPIC_HW_MOTOR_POSITIONS,
    TOPIC_HW_MOTOR_SPEEDS,
    TOPIC_TEST_EVENT,
    TOPIC_TEST_STAGE,
    TOPIC_WRIST_STATE,
)


class LoggerNode(Node):
    """CSV logger for the multi-node haptic force test.

    Subscribes to both inter-node ``/hand/*`` topics and raw hardware data
    streams, logging them in the legacy CsvLogger format.
    """

    def __init__(self, config_path: Optional[str] = None) -> None:
        super().__init__("logger_node")
        self._config_path = config_path or os.path.join(
            _REPO_ROOT, "config", "mia_haptic_force_test.yaml"
        )

        # ── Load logging config from YAML ────────────────────────────────
        raw_cfg = self._load_yaml(self._config_path)
        log_cfg: dict[str, Any] = dict(raw_cfg.get("logging", {}))

        output_base = log_cfg.get(
            "output_dir",
            os.path.join(_REPO_ROOT, "data", "mia_haptic_force_test"),
        )
        run_prefix = log_cfg.get("run_name_prefix", "mia_haptic_force_test")
        samples_csv_name = log_cfg.get("samples_csv", "samples.csv")
        events_csv_name = log_cfg.get("events_csv", "events.csv")
        config_snapshot_name = log_cfg.get("config_snapshot_yaml", "config_snapshot.yaml")
        self._flush_every = max(1, int(log_cfg.get("flush_every_rows", 10)))

        # ── Create run directory ─────────────────────────────────────────
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_id = f"{run_prefix}_{stamp}"
        self._run_dir = Path(output_base) / self.run_id
        self._run_dir.mkdir(parents=True, exist_ok=True)

        # ── Config snapshot ──────────────────────────────────────────────
        snapshot_path = self._run_dir / config_snapshot_name
        with snapshot_path.open("w", encoding="utf-8") as fh:
            yaml.safe_dump(raw_cfg, fh, sort_keys=False)

        # ── CSV files with legacy-compatible headers ─────────────────────
        self._sample_fields = self._build_sample_fields()
        self._event_fields = [
            "run_id",
            "wall_time_ns",
            "elapsed_s",
            "event",
            "state",
            "control_mode",
            "detail",
            "gesture_label",
            "gesture_name",
            "confidence",
            "proportional",
        ]

        self._samples_path = self._run_dir / samples_csv_name
        self._events_path = self._run_dir / events_csv_name
        self._samples_file = self._samples_path.open("w", newline="", encoding="utf-8")
        self._events_file = self._events_path.open("w", newline="", encoding="utf-8")
        self._samples_writer = csv.DictWriter(
            self._samples_file, fieldnames=self._sample_fields
        )
        self._events_writer = csv.DictWriter(
            self._events_file, fieldnames=self._event_fields
        )
        self._samples_writer.writeheader()
        self._events_writer.writeheader()

        # ── Shared state ─────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._state: dict[str, Any] = {}
        self._event_queue: deque[dict[str, Any]] = deque()
        self._sample_count = 0

        # ── Subscriptions ────────────────────────────────────────────────
        # Hand state (force_input_node)
        self.create_subscription(
            Float32MultiArray, TOPIC_HAND_FORCES, self._on_forces, 10
        )
        self.create_subscription(
            String, TOPIC_HAND_FORCE_SOURCE, self._on_str("force_source"), 10
        )
        self.create_subscription(
            JointState, TOPIC_HAND_JOINT_STATES, self._on_joint_states, 10
        )

        # EMG (emg_input_node)
        self.create_subscription(
            String, TOPIC_EMG_GESTURE, self._on_str("gesture_name"), 10
        )
        self.create_subscription(
            Int32, TOPIC_EMG_GESTURE_LABEL, self._on_int("gesture_label"), 10
        )
        self.create_subscription(
            Float32, TOPIC_EMG_CONFIDENCE, self._on_float("confidence"), 10
        )
        self.create_subscription(
            Float32, TOPIC_EMG_PROPORTIONAL, self._on_float("proportional"), 10
        )

        # Control (supervisor_node)
        self.create_subscription(
            Float64MultiArray, TOPIC_CONTROL_TARGET_FORCE,
            self._on_target_force, 10,
        )
        self.create_subscription(
            Float64MultiArray, TOPIC_CONTROL_TARGET_WRIST,
            self._on_target_wrist, 10,
        )
        self.create_subscription(
            String, TOPIC_CONTROL_MODE, self._on_str("controller_mode"), 10
        )
        self.create_subscription(
            String, TOPIC_CONTROL_HOLD_MODE, self._on_str("control_mode"), 10
        )
        self.create_subscription(
            Bool, TOPIC_CONTROL_ENABLE, self._on_bool("enable"), 10
        )

        # Test orchestration (supervisor_node)
        self.create_subscription(
            String, TOPIC_TEST_STAGE, self._on_str("state"), 10
        )
        self.create_subscription(
            String, TOPIC_TEST_EVENT, self._on_event, 10
        )

        # Controller feedback (hand_controller_node)
        self.create_subscription(
            Float64MultiArray, "/controller/force_error",
            self._on_force_error, 10,
        )
        self.create_subscription(
            Bool, "/controller/active", self._on_bool("controller_active"), 10
        )
        self.create_subscription(
            String, "/controller/loop_timing",
            self._on_str("loop_timing_json"), 10,
        )

        # Velocity commands (hand_controller_node)
        self.create_subscription(
            Float64MultiArray, TOPIC_GROUP_VEL_FF_COMMANDS,
            self._on_vel_cmd, 10,
        )

        # Wrist state
        self.create_subscription(
            Float64MultiArray, TOPIC_WRIST_STATE, self._on_wrist_state, 10
        )

        # Haptic band (haptic_node)
        self.create_subscription(
            Float32MultiArray, TOPIC_HAPTIC_BAND_MOTORS,
            self._on_haptics, 10,
        )

        # Raw hardware streams (Mia Hand driver)
        self.create_subscription(
            ForceData, TOPIC_HW_FINGER_FORCES, self._on_raw_forces, 10
        )
        self.create_subscription(
            MotorData, TOPIC_HW_MOTOR_POSITIONS, self._on_motor_pos, 10
        )
        self.create_subscription(
            MotorData, TOPIC_HW_MOTOR_SPEEDS, self._on_motor_speed, 10
        )
        self.create_subscription(
            MotorData, TOPIC_HW_MOTOR_CURRENTS, self._on_motor_current, 10
        )
        self.create_subscription(
            JointData, TOPIC_HW_JOINT_POSITIONS, self._on_raw_joint_pos, 10
        )
        self.create_subscription(
            JointData, TOPIC_HW_JOINT_SPEEDS, self._on_raw_joint_speed, 10
        )

        # ── Background writer thread ────────────────────────────────────
        rate_hz = float(
            self.declare_parameter("log_rate_hz", 50.0).value
        )
        self._dt = 1.0 / max(rate_hz, 1.0)
        self._start_time = time.monotonic()
        self._running = True
        self._thread = threading.Thread(
            target=self._write_loop, daemon=True, name="logger_loop"
        )
        self._thread.start()

        self.get_logger().info(f"Logging to {self._run_dir}")

    # ────────────────────────────────────────────────────────────────────
    # YAML config loading
    # ────────────────────────────────────────────────────────────────────

    @staticmethod
    def _load_yaml(path: str) -> dict[str, Any]:
        try:
            with open(path, "r", encoding="utf-8") as fh:
                data = yaml.safe_load(fh)
                return data if isinstance(data, dict) else {}
        except Exception:
            return {}

    # ────────────────────────────────────────────────────────────────────
    # Field definitions (must match legacy CsvLogger exactly)
    # ────────────────────────────────────────────────────────────────────

    @staticmethod
    def _build_sample_fields() -> list[str]:
        """Sample CSV column names — must match legacy ``CsvLogger._sample_fields()``."""
        fields = [
            "run_id",
            "wall_time_ns",
            "elapsed_s",
            "ros_time_s",
            "state",
            "state_elapsed_s",
            "control_mode",
            "last_event",
            "buzz_label",
            "buzz_active",
            "contact_reason",
            "fault_reason",
            "controller_mode",
            "gesture_label",
            "gesture_name",
            "confidence",
            "proportional",
            "emg_age_s",
            "open_hold_s",
            "power_hold_s",
            "force_source",
            "force_percent",
            "haptic_phase",
            "wrist_position_deg",
            "wrist_velocity_deg_s",
            "wrist_target_deg",
            "wrist_error_deg",
        ]
        for label in FINGER_LABELS:
            fields.extend([
                f"hand_pos_{label}_rad",
                f"hand_vel_{label}_rad_s",
                f"joint_effort_{label}",
                f"force_normal_{label}",
                f"force_tangential_{label}",
                f"target_force_{label}",
                f"velocity_cmd_{label}_rad_s",
                f"position_cmd_{label}_rad",
                f"raw_joint_pos_{label}",
                f"raw_joint_speed_{label}",
                f"raw_motor_pos_{label}",
                f"raw_motor_speed_{label}",
                f"raw_motor_current_{label}",
            ])
        for i in range(MOTOR_COUNT):
            fields.append(f"haptic_motor_{i}_pct")
        return fields

    # ────────────────────────────────────────────────────────────────────
    # Subscription callbacks — minimal, lock-only
    # ────────────────────────────────────────────────────────────────────

    def _on_str(self, key: str):
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
        def cb(msg: Float32) -> None:
            with self._lock:
                self._state[key] = msg.data
        return cb

    def _on_bool(self, key: str):
        def cb(msg: Bool) -> None:
            with self._lock:
                self._state[key] = msg.data
        return cb

    def _on_forces(self, msg: Float32MultiArray) -> None:
        """Normal forces from force_input_node."""
        with self._lock:
            for i, label in enumerate(FINGER_LABELS):
                if i < len(msg.data):
                    self._state[f"force_normal_{label}"] = msg.data[i]

    def _on_joint_states(self, msg: JointState) -> None:
        """Filtered joint states from force_input_node."""
        with self._lock:
            for i, label in enumerate(FINGER_LABELS):
                if i < len(msg.position):
                    self._state[f"hand_pos_{label}_rad"] = msg.position[i]
                if i < len(msg.velocity):
                    self._state[f"hand_vel_{label}_rad_s"] = msg.velocity[i]
                if i < len(msg.effort):
                    self._state[f"joint_effort_{label}"] = msg.effort[i]

    def _on_target_force(self, msg: Float64MultiArray) -> None:
        """Per-finger force targets from supervisor."""
        with self._lock:
            for i, label in enumerate(FINGER_LABELS):
                if i < len(msg.data):
                    self._state[f"target_force_{label}"] = msg.data[i]

    def _on_target_wrist(self, msg: Float64MultiArray) -> None:
        """Wrist target from supervisor: [target_deg, 0.0]."""
        with self._lock:
            if msg.data:
                self._state["wrist_target_deg"] = msg.data[0]

    def _on_force_error(self, msg: Float64MultiArray) -> None:
        """Force error per finger from hand_controller_node."""
        with self._lock:
            for i, label in enumerate(FINGER_LABELS):
                if i < len(msg.data):
                    self._state[f"force_error_{label}"] = msg.data[i]

    def _on_wrist_state(self, msg: Float64MultiArray) -> None:
        """Wrist state: [position_deg, velocity_deg_s]."""
        with self._lock:
            if msg.data:
                self._state["wrist_position_deg"] = msg.data[0]
            if len(msg.data) > 1:
                self._state["wrist_velocity_deg_s"] = msg.data[1]

    def _on_vel_cmd(self, msg: Float64MultiArray) -> None:
        """Velocity commands from hand_controller_node."""
        with self._lock:
            for i, label in enumerate(FINGER_LABELS):
                if i < len(msg.data):
                    self._state[f"velocity_cmd_{label}_rad_s"] = msg.data[i]

    def _on_haptics(self, msg: Float32MultiArray) -> None:
        """Haptic motor intensities from haptic_node."""
        with self._lock:
            for i in range(MOTOR_COUNT):
                if i < len(msg.data):
                    self._state[f"haptic_motor_{i}_pct"] = msg.data[i]

    def _on_raw_forces(self, msg: ForceData) -> None:
        """Raw ForceData from driver — normal and tangential forces."""
        with self._lock:
            self._state["raw_force_normal_thumb"] = msg.thumb_nfor
            self._state["raw_force_normal_index"] = msg.index_nfor
            self._state["raw_force_normal_mrl"] = msg.mrl_nfor
            self._state["raw_force_tangential_thumb"] = msg.thumb_tfor
            self._state["raw_force_tangential_index"] = msg.index_tfor
            self._state["raw_force_tangential_mrl"] = msg.mrl_tfor

    def _on_motor_pos(self, msg: MotorData) -> None:
        with self._lock:
            self._state["raw_motor_pos_thumb"] = msg.thumb_data
            self._state["raw_motor_pos_index"] = msg.index_data
            self._state["raw_motor_pos_mrl"] = msg.mrl_data

    def _on_motor_speed(self, msg: MotorData) -> None:
        with self._lock:
            self._state["raw_motor_speed_thumb"] = msg.thumb_data
            self._state["raw_motor_speed_index"] = msg.index_data
            self._state["raw_motor_speed_mrl"] = msg.mrl_data

    def _on_motor_current(self, msg: MotorData) -> None:
        with self._lock:
            self._state["raw_motor_current_thumb"] = msg.thumb_data
            self._state["raw_motor_current_index"] = msg.index_data
            self._state["raw_motor_current_mrl"] = msg.mrl_data

    def _on_raw_joint_pos(self, msg: JointData) -> None:
        with self._lock:
            self._state["raw_joint_pos_thumb"] = msg.thumb_data
            self._state["raw_joint_pos_index"] = msg.index_data
            self._state["raw_joint_pos_mrl"] = msg.mrl_data

    def _on_raw_joint_speed(self, msg: JointData) -> None:
        with self._lock:
            self._state["raw_joint_speed_thumb"] = msg.thumb_data
            self._state["raw_joint_speed_index"] = msg.index_data
            self._state["raw_joint_speed_mrl"] = msg.mrl_data

    def _on_event(self, msg: String) -> None:
        """Parse event JSON, update state, and queue for CSV."""
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"event": msg.data, "detail": ""}
        event = payload.get("event", "")
        detail = payload.get("detail", "")
        with self._lock:
            self._state["last_event"] = event
        self._event_queue.append({"event": event, "detail": detail})

    # ────────────────────────────────────────────────────────────────────
    # Background writer thread
    # ────────────────────────────────────────────────────────────────────

    def _write_loop(self) -> None:
        while rclpy.ok() and self._running:
            t0 = time.monotonic()
            now_s = time.time()
            now_ns = time.time_ns()
            ros_time_s = self.get_clock().now().nanoseconds / 1e9
            elapsed_s = time.monotonic() - self._start_time

            with self._lock:
                state = dict(self._state)
                events = list(self._event_queue)
                self._event_queue.clear()

            # ── Sample row ───────────────────────────────────────────────
            row: dict[str, Any] = {
                "run_id": self.run_id,
                "wall_time_ns": now_ns,
                "elapsed_s": f"{elapsed_s:.6f}",
                "ros_time_s": f"{ros_time_s:.6f}",
                "state": state.get("state", ""),
                "state_elapsed_s": "",
                "control_mode": state.get("control_mode", ""),
                "last_event": state.get("last_event", ""),
                "buzz_label": "",
                "buzz_active": "",
                "contact_reason": "",
                "fault_reason": "",
                "controller_mode": state.get("controller_mode", ""),
                "gesture_label": state.get("gesture_label", ""),
                "gesture_name": state.get("gesture_name", ""),
                "confidence": f"{float(state.get('confidence', 0.0)):.6f}",
                "proportional": f"{float(state.get('proportional', 0.0)):.6f}",
                "emg_age_s": "",
                "open_hold_s": "",
                "power_hold_s": "",
                "force_source": state.get("force_source", ""),
                "force_percent": "",
                "haptic_phase": "",
                "wrist_position_deg": f"{float(state.get('wrist_position_deg', 0.0)):.6f}",
                "wrist_velocity_deg_s": f"{float(state.get('wrist_velocity_deg_s', 0.0)):.6f}",
                "wrist_target_deg": f"{float(state.get('wrist_target_deg', 0.0)):.6f}",
                "wrist_error_deg": f"{(float(state.get('wrist_target_deg', 0.0)) - float(state.get('wrist_position_deg', 0.0))):.6f}",
            }
            for label in FINGER_LABELS:
                row[f"hand_pos_{label}_rad"] = self._fmt(state.get(f"hand_pos_{label}_rad"))
                row[f"hand_vel_{label}_rad_s"] = self._fmt(state.get(f"hand_vel_{label}_rad_s"))
                row[f"joint_effort_{label}"] = self._fmt(state.get(f"joint_effort_{label}"))
                row[f"force_normal_{label}"] = self._fmt(state.get(f"force_normal_{label}"))
                row[f"force_tangential_{label}"] = self._fmt(state.get(f"force_tangential_{label}"))
                row[f"target_force_{label}"] = self._fmt(state.get(f"target_force_{label}"))
                row[f"velocity_cmd_{label}_rad_s"] = self._fmt(
                    state.get(f"velocity_cmd_{label}_rad_s")
                )
                row[f"position_cmd_{label}_rad"] = self._fmt(
                    state.get(f"position_cmd_{label}_rad")
                )
                row[f"raw_joint_pos_{label}"] = self._fmt(state.get(f"raw_joint_pos_{label}"))
                row[f"raw_joint_speed_{label}"] = self._fmt(state.get(f"raw_joint_speed_{label}"))
                row[f"raw_motor_pos_{label}"] = self._fmt(state.get(f"raw_motor_pos_{label}"))
                row[f"raw_motor_speed_{label}"] = self._fmt(state.get(f"raw_motor_speed_{label}"))
                row[f"raw_motor_current_{label}"] = self._fmt(
                    state.get(f"raw_motor_current_{label}")
                )
            for i in range(MOTOR_COUNT):
                row[f"haptic_motor_{i}_pct"] = self._fmt(state.get(f"haptic_motor_{i}_pct"))

            self._samples_writer.writerow(row)
            self._sample_count += 1
            if self._sample_count % self._flush_every == 0:
                self._samples_file.flush()

            # ── Event rows ───────────────────────────────────────────────
            for ev in events:
                self._events_writer.writerow({
                    "run_id": self.run_id,
                    "wall_time_ns": time.time_ns(),
                    "elapsed_s": f"{elapsed_s:.6f}",
                    "event": ev.get("event", ""),
                    "state": state.get("state", ""),
                    "control_mode": state.get("control_mode", ""),
                    "detail": ev.get("detail", ""),
                    "gesture_label": state.get("gesture_label", ""),
                    "gesture_name": state.get("gesture_name", ""),
                    "confidence": f"{float(state.get('confidence', 0.0)):.6f}",
                    "proportional": f"{float(state.get('proportional', 0.0)):.6f}",
                })
            if events:
                self._events_file.flush()

            elapsed = time.monotonic() - t0
            remaining = self._dt - elapsed
            if remaining > 0.0:
                time.sleep(remaining)

    @staticmethod
    def _fmt(value: Any) -> str:
        """Format a numeric value to 6 decimal places, or empty string."""
        if value is None:
            return ""
        try:
            return f"{float(value):.6f}"
        except (TypeError, ValueError):
            return ""

    # ────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ────────────────────────────────────────────────────────────────────

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._samples_file.flush()
        self._events_file.flush()
        self._samples_file.close()
        self._events_file.close()

    def destroy_node(self) -> None:
        self.stop()
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config-path", default=None)
    known, remaining = parser.parse_known_args(args or [])
    rclpy.init(args=remaining)
    node = LoggerNode(config_path=known.config_path)

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
