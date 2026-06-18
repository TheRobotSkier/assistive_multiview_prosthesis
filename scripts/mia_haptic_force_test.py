#!/usr/bin/env python3
"""EMG-driven Mia Hand haptic force test.

This node is intentionally self-contained for bench testing. It does not launch
camera, segmentation, preshaping, or the production pipeline manager. The flow is:

1. Start open and horizontal.
2. POWER activates the test and emits a light haptic buzz.
3. Wrist rotates to vertical while haptics encode wrist/thumb angle.
4. Wait, then close the hand using velocity control.
5. Contact force enters force-hold and emits a brief haptic buzz.
6. In force-hold, FLEXION/EXTENSION adjust force target. POWER toggles wrist
   control, where FLEXION/EXTENSION adjust wrist target instead.
7. Holding OPEN stops the test, opens the hand, waits, returns wrist horizontal,
   zeros haptics, and exits.
"""

from __future__ import annotations

import csv
import json
import math
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Optional

import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray, Int32, String
from std_srvs.srv import Trigger

from force_controller.controller_manager_client import ControllerManagerClient
from mia_hand_msgs.msg import ForceData, JointData, MotorData


FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_LABELS = ["thumb", "index", "mrl"]
FINGER_COUNT = 3
MOTOR_COUNT = 8

POSITION_CONTROLLERS = [
    "group_pos_ff_controller",
    "thumb_pos_ff_controller",
    "index_pos_ff_controller",
    "mrl_pos_ff_controller",
]
VELOCITY_CONTROLLERS = [
    "group_vel_ff_controller",
    "thumb_vel_ff_controller",
    "index_vel_ff_controller",
    "mrl_vel_ff_controller",
]


class Stage(str, Enum):
    INITIALISING = "initialising"
    WAITING_FOR_ACTIVATION = "waiting_for_activation"
    ROTATING_TO_VERTICAL = "rotating_to_vertical"
    VERTICAL_DELAY = "vertical_delay"
    FORCE_CLOSING = "force_closing"
    FORCE_HOLD = "force_hold"
    OPENING_HAND = "opening_hand"
    RETURN_DELAY = "return_delay"
    RETURN_WRIST = "return_wrist"
    COMPLETE = "complete"
    FAULT = "fault"


class HoldControl(str, Enum):
    FORCE = "force"
    WRIST = "wrist"


@dataclass
class Config:
    raw: dict[str, Any]
    control_rate_hz: float
    csv_rate_hz: float
    haptics_publish_rate_hz: float
    terminal_rate_hz: float
    startup_controller_timeout_s: float
    status_publish_period_s: float
    complete_shutdown_delay_s: float
    topics: dict[str, str]
    emg: dict[str, float | int]
    open_positions: list[float]
    max_closure_positions: list[float]
    open_position_tolerance_rad: float
    opening_min_s: float
    opening_timeout_s: float
    closing_velocity_start_rad_s: float
    closing_velocity_end_rad_s: float
    closing_decay_steps: int
    closing_step_interval_s: float
    hold_deadzone: float
    hold_min_overshoot: float
    hold_max_overshoot: float
    hold_max_velocity_rad_s: float
    contact_thresholds: list[float]
    initial_hold_targets: list[float]
    target_min: float
    target_max: float
    adjustment_rate_up: float
    adjustment_rate_down: float
    emergency_threshold: float
    emergency_backoff_velocity_rad_s: float
    force_stale_timeout_s: float
    require_force_data: bool
    haptic_percent_source: str
    wrist: dict[str, float | bool]
    haptics: dict[str, Any]
    logging: dict[str, Any]


def _get(raw: dict[str, Any], path: str, default: Any) -> Any:
    cur: Any = raw
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


def _finger_values(raw: dict[str, Any], path: str, default: float) -> list[float]:
    section = _get(raw, path, {})
    if not isinstance(section, dict):
        section = {}
    return [float(section.get(name, default)) for name in FINGER_JOINTS]


def _load_config(path: str) -> Config:
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    topics = dict(_get(raw, "topics", {}))
    emg = dict(_get(raw, "emg", {}))
    wrist = dict(_get(raw, "wrist", {}))
    haptics = dict(_get(raw, "haptics", {}))
    logging = dict(_get(raw, "logging", {}))

    return Config(
        raw=raw,
        control_rate_hz=float(_get(raw, "runtime.control_rate_hz", 20.0)),
        startup_controller_timeout_s=float(
            _get(raw, "runtime.startup_controller_timeout_s", 30.0)
        ),
        status_publish_period_s=float(_get(raw, "runtime.status_publish_period_s", 1.0)),
        csv_rate_hz=float(_get(raw, "runtime.csv_rate_hz", 10.0)),
        haptics_publish_rate_hz=float(_get(raw, "runtime.haptics_publish_rate_hz", 10.0)),
        terminal_rate_hz=float(_get(raw, "runtime.terminal_rate_hz", 2.0)),
        complete_shutdown_delay_s=float(
            _get(raw, "runtime.complete_shutdown_delay_s", 0.2)
        ),
        topics={
            "joint_states": topics.get("joint_states", "/joint_states"),
            "force_data": topics.get("force_data", "data_streams/fingers/forces/data"),
            "motor_positions": topics.get(
                "motor_positions", "data_streams/motors/positions/data"
            ),
            "motor_speeds": topics.get("motor_speeds", "data_streams/motors/speeds/data"),
            "motor_currents": topics.get(
                "motor_currents", "data_streams/motors/currents/data"
            ),
            "joint_positions": topics.get(
                "joint_positions", "data_streams/joints/positions/data"
            ),
            "joint_speeds": topics.get("joint_speeds", "data_streams/joints/speeds/data"),
            "emg_gesture_label": topics.get("emg_gesture_label", "/emg/gesture_label"),
            "emg_gesture_name": topics.get("emg_gesture_name", "/emg/gesture_name"),
            "emg_confidence": topics.get("emg_confidence", "/emg/confidence"),
            "emg_proportional": topics.get("emg_proportional", "/emg/proportional"),
            "hand_velocity_commands": topics.get(
                "hand_velocity_commands", "/group_vel_ff_controller/commands"
            ),
            "hand_position_commands": topics.get(
                "hand_position_commands", "/group_pos_ff_controller/commands"
            ),
            "wrist_state": topics.get("wrist_state", "/wrist/state"),
            "wrist_command": topics.get("wrist_command", "/wrist/set_position"),
            "haptic_motors": topics.get("haptic_motors", "/haptic_band/motors"),
            "test_state": topics.get("test_state", "/mia_haptic_force_test/state"),
            "test_status": topics.get("test_status", "/mia_haptic_force_test/status"),
        },
        emg={
            "rest_label": int(emg.get("rest_label", 0)),
            "activation_label": int(emg.get("activation_label", 1)),
            "wrist_toggle_label": int(emg.get("wrist_toggle_label", 1)),
            "open_label": int(emg.get("open_label", 2)),
            "increase_force_label": int(emg.get("increase_force_label", 3)),
            "decrease_force_label": int(emg.get("decrease_force_label", 4)),
            "wrist_positive_label": int(emg.get("wrist_positive_label", 3)),
            "wrist_negative_label": int(emg.get("wrist_negative_label", 4)),
            "confidence_threshold": float(emg.get("confidence_threshold", 0.55)),
            "activation_hold_s": float(emg.get("activation_hold_s", 0.20)),
            "open_hold_s": float(emg.get("open_hold_s", 1.00)),
            "open_proportional_threshold": float(
                emg.get("open_proportional_threshold", 0.50)
            ),
            "power_toggle_hold_s": float(emg.get("power_toggle_hold_s", 0.50)),
            "power_rearm_s": float(emg.get("power_rearm_s", 0.40)),
            "stale_timeout_s": float(emg.get("stale_timeout_s", 1.00)),
        },
        open_positions=_finger_values(raw, "hand.open_positions", 0.0),
        max_closure_positions=_finger_values(raw, "hand.max_closure_positions", 1.5),
        open_position_tolerance_rad=float(
            _get(raw, "hand.open_position_tolerance_rad", 0.08)
        ),
        opening_min_s=float(_get(raw, "hand.opening_min_s", 0.50)),
        opening_timeout_s=float(_get(raw, "hand.opening_timeout_s", 4.00)),
        closing_velocity_start_rad_s=float(
            _get(raw, "hand.closing_velocity_start_rad_s", 0.30)
        ),
        closing_velocity_end_rad_s=float(
            _get(raw, "hand.closing_velocity_end_rad_s", 0.10)
        ),
        closing_decay_steps=int(_get(raw, "hand.closing_decay_steps", 5)),
        closing_step_interval_s=float(_get(raw, "hand.closing_step_interval_s", 0.20)),
        hold_deadzone=float(_get(raw, "hand.hold_deadzone", 20.0)),
        hold_min_overshoot=float(_get(raw, "hand.hold_min_overshoot", 20.0)),
        hold_max_overshoot=float(_get(raw, "hand.hold_max_overshoot", 150.0)),
        hold_max_velocity_rad_s=float(_get(raw, "hand.hold_max_velocity_rad_s", 0.08)),
        contact_thresholds=_finger_values(raw, "force.contact_thresholds", 300.0),
        initial_hold_targets=_finger_values(raw, "force.initial_hold_targets", 300.0),
        target_min=float(_get(raw, "force.target_min", 50.0)),
        target_max=float(_get(raw, "force.target_max", 500.0)),
        adjustment_rate_up=float(_get(raw, "force.adjustment_rate_up", 100.0)),
        adjustment_rate_down=float(_get(raw, "force.adjustment_rate_down", 200.0)),
        emergency_threshold=float(_get(raw, "force.emergency_threshold", 800.0)),
        emergency_backoff_velocity_rad_s=float(
            _get(raw, "force.emergency_backoff_velocity_rad_s", -0.10)
        ),
        force_stale_timeout_s=float(_get(raw, "force.stale_timeout_s", 1.0)),
        require_force_data=_as_bool(_get(raw, "force.require_force_data", True)),
        haptic_percent_source=str(
            _get(raw, "force.haptic_percent_source", "measured_average")
        ),
        wrist={
            "enabled": _as_bool(wrist.get("enabled", True)),
            "horizontal_deg": float(wrist.get("horizontal_deg", 180.0)),
            "vertical_deg": float(wrist.get("vertical_deg", 90.0)),
            "min_deg": float(wrist.get("min_deg", 5.0)),
            "max_deg": float(wrist.get("max_deg", 300.0)),
            "acceleration_deg_s2": float(wrist.get("acceleration_deg_s2", 180.0)),
            "control_velocity_deg_s": float(wrist.get("control_velocity_deg_s", 45.0)),
            "position_tolerance_deg": float(wrist.get("position_tolerance_deg", 4.0)),
            "move_timeout_s": float(wrist.get("move_timeout_s", 8.0)),
            "settle_s": float(wrist.get("settle_s", 0.20)),
            "stale_timeout_s": float(wrist.get("stale_timeout_s", 1.00)),
            "assume_target_on_timeout": _as_bool(
                wrist.get("assume_target_on_timeout", True)
            ),
            "return_after_open_delay_s": float(
                wrist.get("return_after_open_delay_s", 3.00)
            ),
            "vertical_delay_s": float(wrist.get("vertical_delay_s", 3.00)),
        },
        haptics={
            "enabled": _as_bool(haptics.get("enabled", True)),
            "motor_count": int(haptics.get("motor_count", MOTOR_COUNT)),
            "publish_zero_when_idle": _as_bool(
                haptics.get("publish_zero_when_idle", True)
            ),
            "event_motor_indices": list(
                haptics.get("event_motor_indices", list(range(MOTOR_COUNT)))
            ),
            "activation_buzz_duration_s": float(
                haptics.get("activation_buzz_duration_s", 0.12)
            ),
            "activation_buzz_intensity_pct": float(
                haptics.get("activation_buzz_intensity_pct", 12.0)
            ),
            "closure_start_buzz_duration_s": float(
                haptics.get("closure_start_buzz_duration_s", 0.15)
            ),
            "closure_start_buzz_intensity_pct": float(
                haptics.get("closure_start_buzz_intensity_pct", 22.0)
            ),
            "contact_buzz_duration_s": float(
                haptics.get("contact_buzz_duration_s", 0.20)
            ),
            "contact_buzz_intensity_pct": float(
                haptics.get("contact_buzz_intensity_pct", 35.0)
            ),
            "wrist_motor_angles_deg": [
                float(v)
                for v in haptics.get(
                    "wrist_motor_angles_deg",
                    [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0],
                )
            ],
            "wrist_thumb_offset_deg": float(
                haptics.get("wrist_thumb_offset_deg", 0.0)
            ),
            "wrist_intensity_pct": float(haptics.get("wrist_intensity_pct", 45.0)),
            "wrist_min_motor_pct": float(haptics.get("wrist_min_motor_pct", 3.0)),
            "force_min_grasp_force": float(
                haptics.get("force_min_grasp_force", 50.0)
            ),
            "force_max_grasp_force": float(
                haptics.get("force_max_grasp_force", 500.0)
            ),
            "force_phase_change_threshold_pct": float(
                haptics.get("force_phase_change_threshold_pct", 40.0)
            ),
            "force_motor_step_pct": float(haptics.get("force_motor_step_pct", 5.0)),
            "force_low_intensity_pct": float(
                haptics.get("force_low_intensity_pct", 18.0)
            ),
            "force_high_intensity_pct": float(
                haptics.get("force_high_intensity_pct", 85.0)
            ),
            "force_motor_order": list(haptics.get("force_motor_order", list(range(8)))),
        },
        logging={
            "output_dir": logging.get(
                "output_dir", "/prosthesis_ws/data/mia_haptic_force_test"
            ),
            "run_name_prefix": logging.get(
                "run_name_prefix", "mia_haptic_force_test"
            ),
            "samples_csv": logging.get("samples_csv", "samples.csv"),
            "events_csv": logging.get("events_csv", "events.csv"),
            "config_snapshot_yaml": logging.get(
                "config_snapshot_yaml", "config_snapshot.yaml"
            ),
            "flush_every_rows": int(logging.get("flush_every_rows", 1)),
        },
    )


def _competitors(active: list[str]) -> list[str]:
    active_set = set(active)
    return [c for c in POSITION_CONTROLLERS + VELOCITY_CONTROLLERS if c not in active_set]


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _circ_delta_deg(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


def _circ_dist_deg(a: float, b: float) -> float:
    return abs(_circ_delta_deg(a, b))


def _velocity_ramp(start: float, end: float, steps: int) -> list[float]:
    if steps <= 1:
        return [start]
    return [start + (i / (steps - 1)) * (end - start) for i in range(steps)]


def _hold_velocity(
    force: float,
    target: float,
    deadzone: float,
    max_velocity: float,
    min_overshoot: float,
    max_overshoot: float,
) -> float:
    error = target - force
    abs_error = abs(error)
    if abs_error <= deadzone:
        return 0.0
    if max_overshoot <= 0.0:
        return max_velocity if error > 0.0 else -max_velocity
    scaled = min(max(abs_error, min_overshoot), max_overshoot)
    velocity = max_velocity * (scaled / max_overshoot)
    return velocity if error > 0.0 else -velocity


def _wrist_haptics(angle_deg: float, cfg: dict[str, Any]) -> list[float]:
    motor_count = int(cfg["motor_count"])
    motor_angles = list(cfg["wrist_motor_angles_deg"])
    if len(motor_angles) != motor_count:
        motor_angles = [i * (360.0 / motor_count) for i in range(motor_count)]

    thumb_angle = (angle_deg + float(cfg["wrist_thumb_offset_deg"])) % 360.0
    dists = [_circ_dist_deg(thumb_angle, motor_angle) for motor_angle in motor_angles]
    nearest = sorted(range(motor_count), key=lambda i: dists[i])[:2]

    out = [0.0] * motor_count
    if not nearest:
        return out
    if len(nearest) == 1 or dists[nearest[0]] < 1e-6:
        out[nearest[0]] = float(cfg["wrist_intensity_pct"])
    else:
        i0, i1 = nearest
        d0, d1 = dists[i0], dists[i1]
        span = max(d0 + d1, 1e-6)
        total = float(cfg["wrist_intensity_pct"])
        out[i0] = total * (d1 / span)
        out[i1] = total * (d0 / span)

    min_pct = float(cfg["wrist_min_motor_pct"])
    return [v if v >= min_pct else 0.0 for v in out]


def _force_haptics(percent: float, cfg: dict[str, Any]) -> tuple[list[float], str]:
    motor_count = int(cfg["motor_count"])
    out = [0.0] * motor_count
    pct = _clamp(percent, 0.0, 100.0)

    phase_threshold = max(0.0, min(100.0, float(cfg["force_phase_change_threshold_pct"])))
    step_pct = max(0.1, float(cfg["force_motor_step_pct"]))
    low = _clamp(float(cfg["force_low_intensity_pct"]), 0.0, 100.0)
    high = _clamp(float(cfg["force_high_intensity_pct"]), low, 100.0)
    order = [int(i) for i in cfg["force_motor_order"] if 0 <= int(i) < motor_count]
    if not order:
        order = list(range(motor_count))

    if pct <= 0.0:
        return out, "zero"

    if pct < phase_threshold:
        active_count = int(math.ceil(pct / step_pct))
        active_count = max(1, min(motor_count, active_count))
        for idx in order[:active_count]:
            out[idx] = low
        return out, "adding_motors"

    if phase_threshold >= 100.0:
        intensity = low
    else:
        frac = (pct - phase_threshold) / (100.0 - phase_threshold)
        intensity = low + frac * (high - low)
    for idx in range(motor_count):
        out[idx] = intensity
    return out, "all_motors_intensity"


class CsvLogger:
    def __init__(self, cfg: Config) -> None:
        prefix = str(cfg.logging["run_name_prefix"])
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.run_id = f"{prefix}_{stamp}"
        self.run_dir = Path(str(cfg.logging["output_dir"])) / self.run_id
        self.run_dir.mkdir(parents=True, exist_ok=True)

        snapshot = self.run_dir / str(cfg.logging["config_snapshot_yaml"])
        with snapshot.open("w", encoding="utf-8") as f:
            yaml.safe_dump(cfg.raw, f, sort_keys=False)

        self.sample_fields = self._sample_fields()
        self.event_fields = [
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
        self._samples_file = (self.run_dir / str(cfg.logging["samples_csv"])).open(
            "w", newline="", encoding="utf-8"
        )
        self._events_file = (self.run_dir / str(cfg.logging["events_csv"])).open(
            "w", newline="", encoding="utf-8"
        )
        self.samples = csv.DictWriter(self._samples_file, fieldnames=self.sample_fields)
        self.events = csv.DictWriter(self._events_file, fieldnames=self.event_fields)
        self.samples.writeheader()
        self.events.writeheader()
        self.flush_every_rows = max(1, int(cfg.logging["flush_every_rows"]))
        self._sample_rows = 0

    def _sample_fields(self) -> list[str]:
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
            fields.extend(
                [
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
                ]
            )
        for i in range(MOTOR_COUNT):
            fields.append(f"haptic_motor_{i}_pct")
        return fields

    def write_sample(self, row: dict[str, Any]) -> None:
        self.samples.writerow({key: row.get(key, "") for key in self.sample_fields})
        self._sample_rows += 1
        if self._sample_rows % self.flush_every_rows == 0:
            self._samples_file.flush()

    def write_event(self, row: dict[str, Any]) -> None:
        self.events.writerow({key: row.get(key, "") for key in self.event_fields})
        self._events_file.flush()

    def close(self) -> None:
        self._samples_file.flush()
        self._events_file.flush()
        self._samples_file.close()
        self._events_file.close()


class MiaHapticForceTest(Node):
    def __init__(self) -> None:
        super().__init__("mia_haptic_force_test")
        config_path = self.declare_parameter(
            "config_path", "/prosthesis_ws/config/mia_haptic_force_test.yaml"
        ).value
        self._config_path = str(config_path)
        self._cfg = _load_config(str(config_path))
        self._log = CsvLogger(self._cfg)

        t = self._cfg.topics
        self._vel_pub = self.create_publisher(
            Float64MultiArray, t["hand_velocity_commands"], 10
        )
        self._pos_pub = self.create_publisher(
            Float64MultiArray, t["hand_position_commands"], 10
        )
        self._wrist_pub = self.create_publisher(Float64MultiArray, t["wrist_command"], 10)
        self._haptic_pub = self.create_publisher(
            Float32MultiArray, t["haptic_motors"], 10
        )
        self._state_pub = self.create_publisher(String, t["test_state"], 10)
        self._status_pub = self.create_publisher(String, t["test_status"], 10)

        self.create_subscription(JointState, t["joint_states"], self._on_joint_states, 10)
        self.create_subscription(ForceData, t["force_data"], self._on_force_data, 10)
        self.create_subscription(MotorData, t["motor_positions"], self._on_motor_pos, 10)
        self.create_subscription(MotorData, t["motor_speeds"], self._on_motor_speed, 10)
        self.create_subscription(MotorData, t["motor_currents"], self._on_motor_current, 10)
        self.create_subscription(JointData, t["joint_positions"], self._on_raw_joint_pos, 10)
        self.create_subscription(JointData, t["joint_speeds"], self._on_raw_joint_speed, 10)
        self.create_subscription(Int32, t["emg_gesture_label"], self._on_gesture_label, 10)
        self.create_subscription(String, t["emg_gesture_name"], self._on_gesture_name, 10)
        self.create_subscription(Float32, t["emg_confidence"], self._on_confidence, 10)
        self.create_subscription(Float32, t["emg_proportional"], self._on_proportional, 10)
        self.create_subscription(Float64MultiArray, t["wrist_state"], self._on_wrist_state, 10)

        self._cm_client = ControllerManagerClient(self, use_private_executor=True)

        now = time.monotonic()
        self._start_time = now
        self._last_tick_time = now
        self._last_csv_time = now
        self._last_haptic_time = now
        self._last_terminal_time = now
        self._last_force_delta = 0.0
        self._stage = Stage.INITIALISING
        self._stage_started_at = now
        self._control_mode = HoldControl.FORCE
        self._controller_mode = "unknown"
        self._last_event = "startup"
        self._contact_reason = ""
        self._fault_reason = ""
        self._finished = False
        self._initialised = False
        self._fault_recovery_started = False

        self._positions = [0.0] * FINGER_COUNT
        self._velocities = [0.0] * FINGER_COUNT
        self._joint_efforts = [0.0] * FINGER_COUNT
        self._got_joint_states = False
        self._got_joint_effort = False

        self._force_normal = [0.0] * FINGER_COUNT
        self._force_tangential = [0.0] * FINGER_COUNT
        self._got_force_data = False
        self._last_force_data_time = 0.0
        self._last_joint_effort_time = 0.0

        self._raw_joint_pos = [None] * FINGER_COUNT
        self._raw_joint_speed = [None] * FINGER_COUNT
        self._raw_motor_pos = [None] * FINGER_COUNT
        self._raw_motor_speed = [None] * FINGER_COUNT
        self._raw_motor_current = [None] * FINGER_COUNT

        e = self._cfg.emg
        self._gesture_label = int(e["rest_label"])
        self._gesture_name = "REST"
        self._confidence = 0.0
        self._proportional = 0.0
        self._gesture_started_at = now
        self._last_emg_time = 0.0
        self._power_armed = True
        self._last_power_toggle = 0.0

        wrist = self._cfg.wrist
        self._wrist_position_deg = float(wrist["horizontal_deg"])
        self._wrist_velocity_deg_s = 0.0
        self._wrist_target_deg = float(wrist["horizontal_deg"])
        self._wrist_reached_since: Optional[float] = None
        self._last_wrist_state_time = 0.0
        self._got_wrist_state = False
        self._last_wrist_cmd: Optional[list[float]] = None

        self._target_forces = [
            _clamp(v, self._cfg.target_min, self._cfg.target_max)
            for v in self._cfg.initial_hold_targets
        ]
        self._ramp = _velocity_ramp(
            self._cfg.closing_velocity_start_rad_s,
            self._cfg.closing_velocity_end_rad_s,
            self._cfg.closing_decay_steps,
        )
        self._ramp_idx = 0
        self._last_ramp_advance = 0.0
        self._current_closing_velocity = 0.0

        self._last_vel_cmd = [0.0] * FINGER_COUNT
        self._last_pos_cmd = list(self._cfg.open_positions)
        self._last_haptics = [0.0] * int(self._cfg.haptics["motor_count"])
        self._haptic_phase = "zero"
        self._buzz_until = 0.0
        self._buzz_label = ""
        self._buzz_values = [0.0] * int(self._cfg.haptics["motor_count"])
        self._last_status_time = 0.0
        self._complete_started_at: Optional[float] = None
        self._activation_seen = False
        self._ui_is_tty = sys.stdout.isatty()
        self._ui_last_render = 0.0
        self._ui_last_stage: Optional[Stage] = None
        self._ui_last_mode: Optional[HoldControl] = None

        self._timer = self.create_timer(1.0 / max(self._cfg.control_rate_hz, 1.0), self._tick)
        self._write_event("startup", f"config={config_path}; output_dir={self._log.run_dir}")
        self.get_logger().info(
            f"Mia haptic force test starting; CSV output: {self._log.run_dir}"
        )
        self._render_terminal_ui(force=True)

    @property
    def finished(self) -> bool:
        return self._finished

    def _on_joint_states(self, msg: JointState) -> None:
        got_effort = False
        for i, name in enumerate(FINGER_JOINTS):
            try:
                idx = msg.name.index(name)
            except ValueError:
                continue
            if idx < len(msg.position):
                self._positions[i] = float(msg.position[idx])
            if idx < len(msg.velocity):
                self._velocities[i] = float(msg.velocity[idx])
            if idx < len(msg.effort):
                self._joint_efforts[i] = float(msg.effort[idx])
                got_effort = True
        self._got_joint_states = True
        if got_effort:
            self._got_joint_effort = True
            self._last_joint_effort_time = time.monotonic()

    def _on_force_data(self, msg: ForceData) -> None:
        self._force_normal = [
            float(msg.thumb_nfor),
            float(msg.index_nfor),
            float(msg.mrl_nfor),
        ]
        self._force_tangential = [
            float(msg.thumb_tfor),
            float(msg.index_tfor),
            float(msg.mrl_tfor),
        ]
        self._got_force_data = True
        self._last_force_data_time = time.monotonic()

    def _on_motor_pos(self, msg: MotorData) -> None:
        self._raw_motor_pos = [float(msg.thumb_data), float(msg.index_data), float(msg.mrl_data)]

    def _on_motor_speed(self, msg: MotorData) -> None:
        self._raw_motor_speed = [
            float(msg.thumb_data),
            float(msg.index_data),
            float(msg.mrl_data),
        ]

    def _on_motor_current(self, msg: MotorData) -> None:
        self._raw_motor_current = [
            float(msg.thumb_data),
            float(msg.index_data),
            float(msg.mrl_data),
        ]

    def _on_raw_joint_pos(self, msg: JointData) -> None:
        self._raw_joint_pos = [float(msg.thumb_data), float(msg.index_data), float(msg.mrl_data)]

    def _on_raw_joint_speed(self, msg: JointData) -> None:
        self._raw_joint_speed = [
            float(msg.thumb_data),
            float(msg.index_data),
            float(msg.mrl_data),
        ]

    def _on_gesture_label(self, msg: Int32) -> None:
        label = int(msg.data)
        if label != self._gesture_label:
            self._gesture_label = label
            self._gesture_started_at = time.monotonic()
            if label != int(self._cfg.emg["wrist_toggle_label"]):
                self._power_armed = True
        self._last_emg_time = time.monotonic()

    def _on_gesture_name(self, msg: String) -> None:
        self._gesture_name = msg.data
        self._last_emg_time = time.monotonic()

    def _on_confidence(self, msg: Float32) -> None:
        self._confidence = float(msg.data)
        self._last_emg_time = time.monotonic()

    def _on_proportional(self, msg: Float32) -> None:
        self._proportional = _clamp(float(msg.data), 0.0, 1.0)
        self._last_emg_time = time.monotonic()

    def _on_wrist_state(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return
        self._wrist_position_deg = float(msg.data[0])
        if len(msg.data) > 1:
            self._wrist_velocity_deg_s = float(msg.data[1])
        self._got_wrist_state = True
        self._last_wrist_state_time = time.monotonic()

    _tick_running: bool = False

    def _tick(self) -> None:
        """Timer callback — guard against overload pile-up."""
        if self._tick_running:
            self.get_logger().warn(
                "Tick overload — skipping (previous tick still running)"
            )
            return
        self._tick_running = True
        try:
            self._tick_impl()
        finally:
            self._tick_running = False

    def _tick_impl(self) -> None:
        now = time.monotonic()
        dt = max(0.001, now - self._last_tick_time)
        self._last_tick_time = now

        if self._stage == Stage.INITIALISING and not self._initialised:
            self._initialise_runtime()
        elif self._stage not in (Stage.COMPLETE, Stage.FAULT) and self._open_safety_active():
            self._begin_open_release("OPEN held")
        elif self._stage == Stage.WAITING_FOR_ACTIVATION:
            self._run_waiting()
        elif self._stage == Stage.ROTATING_TO_VERTICAL:
            self._run_rotating_to_vertical()
        elif self._stage == Stage.VERTICAL_DELAY:
            self._run_vertical_delay()
        elif self._stage == Stage.FORCE_CLOSING:
            self._run_force_closing()
        elif self._stage == Stage.FORCE_HOLD:
            self._run_force_hold(dt)
        elif self._stage == Stage.OPENING_HAND:
            self._run_opening_hand()
        elif self._stage == Stage.RETURN_DELAY:
            self._run_return_delay()
        elif self._stage == Stage.RETURN_WRIST:
            self._run_return_wrist()
        # ── Rate-limited: haptics ──────────────────────────────────
        if now - self._last_haptic_time >= 1.0 / max(self._cfg.haptics_publish_rate_hz, 1.0):
            self._publish_haptics(self._compute_haptics())
            self._last_haptic_time = now

        # ── Rate-limited: CSV ──────────────────────────────────────
        if now - self._last_csv_time >= 1.0 / max(self._cfg.csv_rate_hz, 1.0):
            self._write_sample()
            self._last_csv_time = now

        # ── Always: status (internally rate-limited) ────────────────
        self._publish_status()

        # ── Rate-limited: terminal ─────────────────────────────────
        if now - self._last_terminal_time >= 1.0 / max(self._cfg.terminal_rate_hz, 1.0):
            self._render_terminal_ui()
            self._last_terminal_time = now


    def _initialise_runtime(self) -> None:
        try:
            if not self._cm_client.wait_for_services(
                timeout_sec=self._cfg.startup_controller_timeout_s
            ):
                self._fault("controller_manager services unavailable")
                return
            # Clear any residual emergency-stop state in the hardware interface.
            # The CppDriver emergency_stop_on_ flag (which gates all movement
            # commands) may be spuriously set after process restart due to an
            # uninitialised bool in the upstream driver.  The system interface
            # exposes a ~/play service on its diagnostics node that calls
            # CppDriver::play() to clear the flag.  Fire-and-forget is safe
            # here because we are inside a spin_once timer callback and cannot
            # spin_until_future_complete without deadlocking.
            play_cli = self.create_client(
                Trigger, "/mia_hand_system_interface_diagnostics/play"
            )
            if play_cli.wait_for_service(timeout_sec=2.0):
                play_cli.call_async(Trigger.Request())
                self.get_logger().info("Called play service to clear emergency-stop flag.")
            else:
                self.get_logger().warn(
                    "Play service (/mia_hand_system_interface_diagnostics/play) "
                    "unavailable — hand may be in emergency stop."
                )
            # Wait for the spawner to configure and activate the position
            # controller.  Calling switch_controllers ourselves would race:
            # the controller may still be unconfigured, causing a silent
            # BEST_EFFORT "success" that leaves jnt_cmd_modes_ at kNone.
            start = time.monotonic()
            while time.monotonic() - start < self._cfg.startup_controller_timeout_s:
                states = self._cm_client.list_controller_states()
                if states.get("group_pos_ff_controller") == "active":
                    break
                time.sleep(0.1)
            else:
                self._fault("position controller not active after timeout")
                return
            self.get_logger().info(
                f"Controller active after {time.monotonic() - start:.1f}s; "
                "position commands will be processed."
            )
            self._publish_velocity([0.0] * FINGER_COUNT, force=True)
            self._publish_position(self._cfg.open_positions, force=True)
            self._publish_wrist(float(self._cfg.wrist["horizontal_deg"]), force=True)
            self._initialised = True
            self._transition(Stage.WAITING_FOR_ACTIVATION, "open hand and horizontal wrist commanded")
        except Exception as exc:
            self._fault(f"initialisation failed: {exc}")

    def _run_waiting(self) -> None:
        self._publish_position(self._cfg.open_positions)
        self._publish_wrist(float(self._cfg.wrist["horizontal_deg"]))
        if self._active_gesture(int(self._cfg.emg["activation_label"])) and self._gesture_held(
            int(self._cfg.emg["activation_label"]),
            float(self._cfg.emg["activation_hold_s"]),
        ):
            self._begin_activation()

    def _begin_activation(self) -> None:
        self._activation_seen = True
        self._power_armed = False
        self._last_power_toggle = time.monotonic()
        self._trigger_buzz(
            "activation",
            float(self._cfg.haptics["activation_buzz_duration_s"]),
            float(self._cfg.haptics["activation_buzz_intensity_pct"]),
        )
        self._wrist_target_deg = float(self._cfg.wrist["vertical_deg"])
        self._publish_wrist(self._wrist_target_deg, force=True)
        self._transition(Stage.ROTATING_TO_VERTICAL, "activation accepted")

    def _run_rotating_to_vertical(self) -> None:
        self._publish_position(self._cfg.open_positions)
        self._publish_wrist(self._wrist_target_deg)
        if self._wrist_target_complete():
            self._transition(Stage.VERTICAL_DELAY, "vertical wrist target reached")

    def _run_vertical_delay(self) -> None:
        self._publish_position(self._cfg.open_positions)
        self._publish_wrist(self._wrist_target_deg)
        if self._stage_elapsed() >= float(self._cfg.wrist["vertical_delay_s"]):
            self._begin_force_closure()

    def _begin_force_closure(self) -> None:
        try:
            self._trigger_buzz(
                "force_closure_start",
                float(self._cfg.haptics["closure_start_buzz_duration_s"]),
                float(self._cfg.haptics["closure_start_buzz_intensity_pct"]),
            )
            self._switch_velocity_controller()
            self._ramp_idx = 0
            self._last_ramp_advance = 0.0
            self._current_closing_velocity = 0.0
            self._target_forces = [
                _clamp(v, self._cfg.target_min, self._cfg.target_max)
                for v in self._cfg.initial_hold_targets
            ]
            self._contact_reason = ""
            self._transition(Stage.FORCE_CLOSING, "velocity closure started")
        except Exception as exc:
            self._fault(f"failed to start force closure: {exc}")

    def _run_force_closing(self) -> None:
        if self._force_faulted():
            return
        contact = self._first_contact_reason()
        if contact:
            self._contact_reason = contact
            self._publish_velocity([0.0] * FINGER_COUNT, force=True)
            self._trigger_buzz(
                "force_threshold_crossed",
                float(self._cfg.haptics["contact_buzz_duration_s"]),
                float(self._cfg.haptics["contact_buzz_intensity_pct"]),
            )
            self._transition(Stage.FORCE_HOLD, contact)
            return
        stop_position = self._stop_position_reason()
        if stop_position:
            self._fault(f"max closure reached before force contact: {stop_position}")
            return

        now = time.monotonic()
        if self._ramp_idx < len(self._ramp):
            if self._ramp_idx == 0 or now - self._last_ramp_advance >= self._cfg.closing_step_interval_s:
                self._current_closing_velocity = self._ramp[self._ramp_idx]
                self._ramp_idx += 1
                self._last_ramp_advance = now
        else:
            self._current_closing_velocity = self._cfg.closing_velocity_end_rad_s
        self._publish_velocity([self._current_closing_velocity] * FINGER_COUNT)

    def _run_force_hold(self, dt: float) -> None:
        if self._force_faulted():
            return
        self._maybe_toggle_hold_control()
        if self._control_mode == HoldControl.FORCE:
            self._adjust_force_target(dt)
        else:
            self._adjust_wrist_target(dt)

        normal_forces, _ = self._current_normal_forces()
        velocities = [
            _hold_velocity(
                normal_forces[i],
                self._target_forces[i],
                self._cfg.hold_deadzone,
                self._cfg.hold_max_velocity_rad_s,
                self._cfg.hold_min_overshoot,
                self._cfg.hold_max_overshoot,
            )
            for i in range(FINGER_COUNT)
        ]
        self._publish_velocity(velocities)
        if self._control_mode == HoldControl.WRIST:
            self._publish_wrist(self._wrist_target_deg)

    def _run_opening_hand(self) -> None:
        self._publish_velocity([0.0] * FINGER_COUNT)
        self._publish_position(self._cfg.open_positions)
        elapsed = self._stage_elapsed()
        if elapsed < self._cfg.opening_min_s:
            return
        open_complete = all(
            abs(self._positions[i] - self._cfg.open_positions[i])
            <= self._cfg.open_position_tolerance_rad
            for i in range(FINGER_COUNT)
        )
        if open_complete or elapsed >= self._cfg.opening_timeout_s:
            self._transition(Stage.RETURN_DELAY, "hand open command complete")

    def _run_return_delay(self) -> None:
        self._publish_position(self._cfg.open_positions)
        if self._stage_elapsed() >= float(self._cfg.wrist["return_after_open_delay_s"]):
            self._wrist_target_deg = float(self._cfg.wrist["horizontal_deg"])
            self._publish_wrist(self._wrist_target_deg, force=True)
            self._transition(Stage.RETURN_WRIST, "returning wrist horizontal")

    def _run_return_wrist(self) -> None:
        self._publish_position(self._cfg.open_positions)
        self._publish_wrist(self._wrist_target_deg)
        if self._wrist_target_complete():
            self._publish_haptics([0.0] * int(self._cfg.haptics["motor_count"]))
            self._complete_started_at = time.monotonic()
            self._transition(Stage.COMPLETE, "test stopped and wrist returned horizontal")

    def _run_complete(self) -> None:
        self._publish_haptics([0.0] * int(self._cfg.haptics["motor_count"]))
        if self._complete_started_at is None:
            self._complete_started_at = time.monotonic()
        if time.monotonic() - self._complete_started_at >= self._cfg.complete_shutdown_delay_s:
            self._finished = True

    def _run_fault_recovery(self) -> None:
        if self._fault_recovery_started:
            return
        self._fault_recovery_started = True
        self._begin_open_release(f"fault recovery: {self._fault_reason}")

    def _begin_open_release(self, reason: str) -> None:
        self._publish_velocity([0.0] * FINGER_COUNT, force=True)
        try:
            self._switch_position_controller()
        except Exception as exc:
            self.get_logger().error(f"Failed to switch to position controller: {exc}")
        self._publish_position(self._cfg.open_positions, force=True)
        self._control_mode = HoldControl.FORCE
        self._transition(Stage.OPENING_HAND, reason)

    def _fault(self, reason: str) -> None:
        self._fault_reason = reason
        self.get_logger().error(f"FAULT: {reason}")
        self._publish_velocity([0.0] * FINGER_COUNT, force=True)
        self._transition(Stage.FAULT, reason)

    def _switch_position_controller(self) -> None:
        self._cm_client.switch_controllers(
            ["group_pos_ff_controller"],
            _competitors(["group_pos_ff_controller"]),
        )
        self._controller_mode = "position"
        self._verify_controller_active("group_pos_ff_controller")

    def _switch_velocity_controller(self) -> None:
        self._cm_client.switch_controllers(
            ["group_vel_ff_controller"],
            _competitors(["group_vel_ff_controller"]),
        )
        self._controller_mode = "velocity"
        self._verify_controller_active("group_vel_ff_controller")

    def _verify_controller_active(self, name: str) -> None:
        try:
            states = self._cm_client.list_controller_states()
            state = states.get(name, "unknown")
            if state != "active":
                self.get_logger().error(
                    f"Controller '{name}' is '{state}' after switch! "
                    "Velocity/position commands may be silently dropped."
                )
            else:
                self.get_logger().info(f"Controller '{name}' confirmed active.")
        except Exception as exc:
            self.get_logger().warn(f"Could not verify controller state: {exc}")

    def _publish_velocity(self, velocities: list[float], force: bool = False) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in velocities]
        self._vel_pub.publish(msg)
        self._last_vel_cmd = list(msg.data)

    def _publish_position(self, positions: list[float], force: bool = False) -> None:
        msg = Float64MultiArray()
        msg.data = [float(p) for p in positions]
        self._pos_pub.publish(msg)
        self._last_pos_cmd = list(msg.data)

    def _publish_wrist(self, target_deg: float, force: bool = False) -> None:
        if not bool(self._cfg.wrist["enabled"]):
            return
        target = _clamp(
            float(target_deg),
            float(self._cfg.wrist["min_deg"]),
            float(self._cfg.wrist["max_deg"]),
        )
        self._wrist_target_deg = target
        cmd = [target, float(self._cfg.wrist["acceleration_deg_s2"])]
        msg = Float64MultiArray()
        msg.data = cmd
        self._wrist_pub.publish(msg)
        self._last_wrist_cmd = list(cmd)

    def _publish_haptics(self, values: list[float]) -> None:
        motor_count = int(self._cfg.haptics["motor_count"])
        out = list(values[:motor_count]) + [0.0] * max(0, motor_count - len(values))
        out = [_clamp(float(v), 0.0, 100.0) for v in out]
        self._last_haptics = out
        if not bool(self._cfg.haptics["enabled"]):
            return
        msg = Float32MultiArray()
        msg.data = out
        self._haptic_pub.publish(msg)

    def _transition(self, new_stage: Stage, detail: str) -> None:
        if new_stage == self._stage:
            return
        old_stage = self._stage
        self._stage = new_stage
        self._stage_started_at = time.monotonic()
        self._wrist_reached_since = None
        self._last_event = f"{old_stage.value}->{new_stage.value}"
        self._state_pub.publish(String(data=new_stage.value))
        self._write_event("state_transition", f"{old_stage.value}->{new_stage.value}: {detail}")
        self.get_logger().info(f"{old_stage.value} -> {new_stage.value}: {detail}")
        self._render_terminal_ui(force=True)

    def _trigger_buzz(self, label: str, duration_s: float, intensity_pct: float) -> None:
        motor_count = int(self._cfg.haptics["motor_count"])
        values = [0.0] * motor_count
        for idx in self._cfg.haptics["event_motor_indices"]:
            i = int(idx)
            if 0 <= i < motor_count:
                values[i] = _clamp(intensity_pct, 0.0, 100.0)
        self._buzz_label = label
        self._buzz_values = values
        self._buzz_until = time.monotonic() + max(0.0, duration_s)
        self._write_event("haptic_buzz", f"{label}: duration={duration_s:.3f}s intensity={intensity_pct:.1f}")

    def _active_gesture(self, label: int) -> bool:
        if self._last_emg_time <= 0.0:
            return False
        if time.monotonic() - self._last_emg_time > float(self._cfg.emg["stale_timeout_s"]):
            return False
        return self._gesture_label == label and self._confidence >= float(
            self._cfg.emg["confidence_threshold"]
        )

    def _gesture_held(self, label: int, seconds: float) -> bool:
        return self._gesture_label == label and (time.monotonic() - self._gesture_started_at) >= seconds

    def _open_safety_active(self) -> bool:
        label = int(self._cfg.emg["open_label"])
        return (
            self._active_gesture(label)
            and self._proportional >= float(self._cfg.emg["open_proportional_threshold"])
            and self._gesture_held(label, float(self._cfg.emg["open_hold_s"]))
        )

    def _maybe_toggle_hold_control(self) -> None:
        label = int(self._cfg.emg["wrist_toggle_label"])
        now = time.monotonic()
        if not self._active_gesture(label):
            if self._gesture_label != label:
                self._power_armed = True
            return
        if not self._power_armed:
            return
        if now - self._last_power_toggle < float(self._cfg.emg["power_rearm_s"]):
            return
        if not self._gesture_held(label, float(self._cfg.emg["power_toggle_hold_s"])):
            return

        self._power_armed = False
        self._last_power_toggle = now
        if self._control_mode == HoldControl.FORCE:
            self._control_mode = HoldControl.WRIST
            self._wrist_target_deg = self._wrist_position_deg
            self._write_event("control_mode", "force->wrist")
        else:
            self._control_mode = HoldControl.FORCE
            self._write_event("control_mode", "wrist->force")


    def _adjustment_active(self) -> bool:
        """True when FLEXION or EXTENSION gesture is currently adjusting."""
        if self._control_mode == HoldControl.FORCE:
            return self._active_gesture(int(self._cfg.emg["increase_force_label"])) or \
                   self._active_gesture(int(self._cfg.emg["decrease_force_label"]))
        return self._active_gesture(int(self._cfg.emg["wrist_positive_label"])) or \
               self._active_gesture(int(self._cfg.emg["wrist_negative_label"]))

    def _update_force_delta(self, dt: float) -> None:
        """Update _last_force_delta for UI display."""
        if self._active_gesture(int(self._cfg.emg["increase_force_label"])):
            self._last_force_delta = self._cfg.adjustment_rate_up * self._proportional
        elif self._active_gesture(int(self._cfg.emg["decrease_force_label"])):
            self._last_force_delta = -self._cfg.adjustment_rate_down * self._proportional
        else:
            self._last_force_delta = 0.0

    def _adjust_force_target(self, dt: float) -> None:
        delta = 0.0
        if self._active_gesture(int(self._cfg.emg["increase_force_label"])):
            delta = self._cfg.adjustment_rate_up * self._proportional * dt
        elif self._active_gesture(int(self._cfg.emg["decrease_force_label"])):
            delta = -self._cfg.adjustment_rate_down * self._proportional * dt
        self._update_force_delta(dt)
        if delta == 0.0:
            return
        self._target_forces = [
            _clamp(t + delta, self._cfg.target_min, self._cfg.target_max)
            for t in self._target_forces
        ]

    def _adjust_wrist_target(self, dt: float) -> None:
        if not bool(self._cfg.wrist["enabled"]):
            return
        direction = 0.0
        if self._active_gesture(int(self._cfg.emg["wrist_positive_label"])):
            direction = 1.0
        elif self._active_gesture(int(self._cfg.emg["wrist_negative_label"])):
            direction = -1.0
        if direction == 0.0:
            return
        step = direction * float(self._cfg.wrist["control_velocity_deg_s"]) * max(
            self._proportional, 0.15
        ) * dt
        self._wrist_target_deg = _clamp(
            self._wrist_target_deg + step,
            float(self._cfg.wrist["min_deg"]),
            float(self._cfg.wrist["max_deg"]),
        )
        self._publish_wrist(self._wrist_target_deg)

    def _wrist_target_complete(self) -> bool:
        if not bool(self._cfg.wrist["enabled"]):
            return True
        now = time.monotonic()
        stale = (
            not self._got_wrist_state
            or now - self._last_wrist_state_time > float(self._cfg.wrist["stale_timeout_s"])
        )
        if not stale:
            error = abs(_circ_delta_deg(self._wrist_target_deg, self._wrist_position_deg))
            if error <= float(self._cfg.wrist["position_tolerance_deg"]):
                if self._wrist_reached_since is None:
                    self._wrist_reached_since = now
                return now - self._wrist_reached_since >= float(self._cfg.wrist["settle_s"])
            self._wrist_reached_since = None
        if (
            bool(self._cfg.wrist["assume_target_on_timeout"])
            and self._stage_elapsed() >= float(self._cfg.wrist["move_timeout_s"])
        ):
            return True
        return False

    def _current_normal_forces(self) -> tuple[list[float], str]:
        now = time.monotonic()
        if self._got_force_data:
            force_age = now - self._last_force_data_time
            if force_age <= self._cfg.force_stale_timeout_s or not self._got_joint_effort:
                return list(self._force_normal), "force_data"
        if self._got_joint_effort:
            return list(self._joint_efforts), "joint_state_effort"
        if self._got_force_data:
            return list(self._force_normal), "force_data"
        return [0.0] * FINGER_COUNT, "none"

    def _force_age(self, source: str) -> Optional[float]:
        now = time.monotonic()
        if source == "force_data":
            return now - self._last_force_data_time
        if source == "joint_state_effort":
            return now - self._last_joint_effort_time
        return None

    def _force_faulted(self) -> bool:
        normal_forces, source = self._current_normal_forces()
        if source == "none":
            if self._cfg.require_force_data and self._stage_elapsed() > self._cfg.force_stale_timeout_s:
                self._fault("force data unavailable")
                return True
            return False
        force_age = self._force_age(source)
        if force_age is not None and force_age > self._cfg.force_stale_timeout_s:
            self._fault("force data stale")
            return True
        if max(normal_forces) >= self._cfg.emergency_threshold:
            self._publish_velocity(
                [self._cfg.emergency_backoff_velocity_rad_s] * FINGER_COUNT,
                force=True,
            )
            self._fault(
                f"emergency force exceeded: max={max(normal_forces):.1f} threshold={self._cfg.emergency_threshold:.1f}"
            )
            return True
        return False

    def _first_contact_reason(self) -> str:
        normal_forces, source = self._current_normal_forces()
        if source != "none":
            for i, name in enumerate(FINGER_JOINTS):
                if normal_forces[i] >= self._cfg.contact_thresholds[i]:
                    return (
                        f"{name} force {normal_forces[i]:.1f} >= "
                        f"{self._cfg.contact_thresholds[i]:.1f}"
                    )
        return ""

    def _stop_position_reason(self) -> str:
        for i, name in enumerate(FINGER_JOINTS):
            if self._positions[i] >= self._cfg.max_closure_positions[i]:
                return (
                    f"{name} position {self._positions[i]:.3f} >= "
                    f"{self._cfg.max_closure_positions[i]:.3f}"
                )
        return ""

    def _force_percent(self) -> float:
        source = self._cfg.haptic_percent_source
        normal_forces, _ = self._current_normal_forces()
        if source == "target_average":
            value = sum(self._target_forces) / FINGER_COUNT
        elif source == "measured_max":
            value = max(normal_forces)
        else:
            value = sum(normal_forces) / FINGER_COUNT
        lo = float(self._cfg.haptics["force_min_grasp_force"])
        hi = float(self._cfg.haptics["force_max_grasp_force"])
        if hi <= lo:
            return 0.0
        return _clamp((value - lo) / (hi - lo) * 100.0, 0.0, 100.0)

    def _compute_haptics(self) -> list[float]:
        motor_count = int(self._cfg.haptics["motor_count"])
        now = time.monotonic()
        if now < self._buzz_until:
            self._haptic_phase = f"buzz:{self._buzz_label}"
            return list(self._buzz_values)
        self._buzz_label = ""

        if not bool(self._cfg.haptics["enabled"]):
            self._haptic_phase = "disabled"
            return [0.0] * motor_count

        wrist_stages = {
            Stage.ROTATING_TO_VERTICAL,
            Stage.VERTICAL_DELAY,
            Stage.RETURN_WRIST,
        }
        if self._stage in wrist_stages or (
            self._stage == Stage.FORCE_HOLD and self._control_mode == HoldControl.WRIST
        ):
            self._haptic_phase = "wrist_angle"
            return _wrist_haptics(self._wrist_position_deg, self._cfg.haptics)

        if self._stage == Stage.FORCE_HOLD and self._control_mode == HoldControl.FORCE:
            values, phase = _force_haptics(self._force_percent(), self._cfg.haptics)
            self._haptic_phase = f"force:{phase}"
            return values

        self._haptic_phase = "idle_zero"
        if bool(self._cfg.haptics["publish_zero_when_idle"]):
            return [0.0] * motor_count
        return self._last_haptics

    def _render_terminal_ui(self, force: bool = False) -> None:
        now = time.monotonic()
        stage_changed = self._ui_last_stage != self._stage
        mode_changed = self._ui_last_mode != self._control_mode
        period = 1.0 if not self._activation_seen else 0.5
        if not force and not stage_changed and not mode_changed and now - self._ui_last_render < period:
            return

        self._ui_last_render = now
        self._ui_last_stage = self._stage
        self._ui_last_mode = self._control_mode

        if self._ui_is_tty:
            sys.stdout.write("\033[2J\033[H")
        else:
            sys.stdout.write("\n")

        if not self._activation_seen and self._stage in (
            Stage.INITIALISING,
            Stage.WAITING_FOR_ACTIVATION,
        ):
            self._print_intro_ui()
        else:
            self._print_state_ui()
        sys.stdout.flush()

    def _print_intro_ui(self) -> None:
        print("Mia Hand EMG Haptic Force Test")
        print(f"Config: {Path(self._config_path).name}  |  CSV: {self._log.run_dir}")
        print("")
        print("Stages:  1. POWER starts → 2. Wrist vertical → 3. Countdown")
        print("         4. Force closure → 5. Force-hold (adjust with EMG)")
        print("         6. POWER toggles force/wrist control")
        print(f"         7. Hold OPEN {float(self._cfg.emg['open_hold_s']):.1f}s → open → return horizontal")
        print("")
        print("Controls now: POWER starts.  OPEN held stops.  Ctrl-C aborts.")
        print("------")

        # Minimal live info
        elapsed = self._elapsed()
        gest_str = self._gesture_name or "?"
        conf = self._confidence
        prop = self._proportional
        emg_age = time.monotonic() - self._last_emg_time if self._last_emg_time > 0.0 else 999.0
        print(f"  State        \033[1m{self._human_stage()}\033[0m")
        if conf >= float(self._cfg.emg["confidence_threshold"]):
            print(f"  EMG          \033[92m{gest_str}\033[0m  conf={conf:.2f}  prop={prop:.2f}  age={emg_age:.3f}s")
        else:
            print(f"  EMG          \033[2m{gest_str}\033[0m  conf={conf:.2f}  prop={prop:.2f}  age={emg_age:.3f}s")
        print("------")
        print(f"CSV           {self._log.run_dir}")
    def _print_state_ui(self) -> None:
        """Terminal UI: header → ----- → dynamic data → ----- → footer."""
        open_s = float(self._cfg.emg["open_hold_s"])

        # ── Header ──────────────────────────────────────────────────
        print("Mia Hand EMG Haptic Force Test")
        print(f"Config: {Path(self._config_path).name}")

        # ── Control hint ────────────────────────────────────────────
        hint = self._ui_control_hint()
        if hint:
            print(f"Controls: {hint}")

        print("------")

        # ── Dynamic info (most important at bottom) ─────────────────
        elapsed = self._elapsed()
        state_elapsed = self._stage_elapsed()
        stage_str = self._human_stage()
        mode_str = self._control_mode.value
        gest_str = self._gesture_name or "?"
        conf = self._confidence
        prop = self._proportional
        emg_age = time.monotonic() - self._last_emg_time if self._last_emg_time > 0.0 else 999.0

        print(f"  Elapsed      {elapsed:.1f}s")
        countdown = self._ui_countdown_s()
        if countdown is not None:
            print(f"  Countdown    {countdown:.1f}s")
        print(f"  State        \033[1m{stage_str}\033[0m  ·  {mode_str}")

        # EMG — green if above confidence threshold
        if conf >= float(self._cfg.emg["confidence_threshold"]):
            print(f"  EMG          \033[92m{gest_str}\033[0m  conf={conf:.2f}  prop={prop:.2f}  age={emg_age:.3f}s")
        else:
            print(f"  EMG          \033[2m{gest_str}\033[0m  conf={conf:.2f}  prop={prop:.2f}  age={emg_age:.3f}s")

        # Force + wrist
        normal_forces, source = self._current_normal_forces()
        avg_f = sum(normal_forces) / FINGER_COUNT if normal_forces else 0.0
        max_f = max(normal_forces) if normal_forces else 0.0
        target_avg = sum(self._target_forces) / FINGER_COUNT
        fpct = self._force_percent()
        # Show hold velocity if in FORCE_HOLD
        vel_str = ""
        if self._stage == Stage.FORCE_HOLD and normal_forces:
            vels = [
                _hold_velocity(normal_forces[i], self._target_forces[i],
                               self._cfg.hold_deadzone, self._cfg.hold_max_velocity_rad_s,
                               self._cfg.hold_min_overshoot, self._cfg.hold_max_overshoot)
                for i in range(FINGER_COUNT)
            ]
            if any(abs(v) > 0.001 for v in vels):
                vel_str = f"  \033[93mvel=[{vels[0]:+.3f} {vels[1]:+.3f} {vels[2]:+.3f}]\033[0m"
        print(f"  Force        avg={avg_f:.1f}  max={max_f:.1f}  target={target_avg:.1f}  haptic={fpct:.0f}%{vel_str}")

        w_err = _circ_delta_deg(self._wrist_target_deg, self._wrist_position_deg)
        print(f"  Wrist        {self._wrist_position_deg:.1f}° → {self._wrist_target_deg:.1f}°  (err={w_err:+.1f}°)")

        # Adjustment status (only in FORCE_HOLD)
        if self._stage == Stage.FORCE_HOLD:
            if self._adjustment_active():
                if self._control_mode == HoldControl.FORCE:
                    print(f"  Adjust       \033[93m▲ adjusting force\033[0m  Δ={self._last_force_delta:+.1f}/s")
                else:
                    print(f"  Adjust       \033[93m◀▶ adjusting wrist\033[0m  vel={float(self._cfg.wrist['control_velocity_deg_s'])*max(prop,0.15):.1f}°/s")
            else:
                print(f"  Adjust       idle  (need conf ≥ {float(self._cfg.emg['confidence_threshold']):.2f}, got {conf:.2f})")

        # Haptics
        haptic = " ".join(f"{v:.0f}" for v in self._last_haptics)
        print(f"  Haptics      {self._haptic_phase:20s} [{haptic}]")

        print("------")
        # EMG prediction as the main "what's happening now" line
        print(f"  Gesture      {gest_str}  conf={conf:.2f}  prop={prop:.2f}")
        print(f"  CSV          {self._log.run_dir}")
        if self._contact_reason:
            print(f"  Contact      {self._contact_reason}")
        if self._fault_reason:
            print(f"  \033[91mFault        {self._fault_reason}\033[0m")

    def _ui_countdown_s(self) -> Optional[float]:
        if self._stage == Stage.VERTICAL_DELAY:
            return max(0.0, float(self._cfg.wrist["vertical_delay_s"]) - self._stage_elapsed())
        if self._stage == Stage.RETURN_DELAY:
            return max(
                0.0,
                float(self._cfg.wrist["return_after_open_delay_s"]) - self._stage_elapsed(),
            )
        return None

    def _ui_control_hint(self) -> str:
        """Single-line control hint for the current stage."""
        open_s = float(self._cfg.emg["open_hold_s"])
        if self._stage == Stage.WAITING_FOR_ACTIVATION:
            return f"keep hand open; POWER held {float(self._cfg.emg['activation_hold_s']):.1f}s starts; OPEN held {open_s:.1f}s stops"
        if self._stage == Stage.ROTATING_TO_VERTICAL:
            return f"keep hand open; OPEN held {open_s:.1f}s stops"
        if self._stage == Stage.VERTICAL_DELAY:
            return f"OPEN held {open_s:.1f}s stops; closure starts after countdown"
        if self._stage == Stage.FORCE_CLOSING:
            return f"OPEN held {open_s:.1f}s stops; waiting for force threshold"
        if self._stage == Stage.FORCE_HOLD and self._control_mode == HoldControl.FORCE:
            return f"FLEXION ↑force  EXTENSION ↓force  POWER→wrist  OPEN held {open_s:.1f}s stops"
        if self._stage == Stage.FORCE_HOLD and self._control_mode == HoldControl.WRIST:
            return f"FLEXION/EXTENSION move wrist  POWER→force  OPEN held {open_s:.1f}s stops"
        if self._stage == Stage.OPENING_HAND:
            return "opening hand before wrist return"
        if self._stage == Stage.RETURN_DELAY:
            return "waiting before returning wrist horizontal"
        if self._stage == Stage.RETURN_WRIST:
            return "returning wrist to horizontal"
        if self._stage == Stage.COMPLETE:
            return "complete; haptics zeroed, shutdown pending"
        if self._stage == Stage.FAULT:
            return f"fault recovery underway: {self._fault_reason}"
        return f"OPEN held {open_s:.1f}s stops"

    def _human_stage(self) -> str:
        return self._stage.value.replace("_", " ")

    def _publish_status(self) -> None:
        now = time.monotonic()
        if now - self._last_status_time < self._cfg.status_publish_period_s:
            return
        self._last_status_time = now
        normal_forces, source = self._current_normal_forces()
        payload = {
            "state": self._stage.value,
            "state_elapsed_s": round(self._stage_elapsed(), 3),
            "control_mode": self._control_mode.value,
            "gesture": self._gesture_name,
            "gesture_label": self._gesture_label,
            "confidence": round(self._confidence, 3),
            "proportional": round(self._proportional, 3),
            "force_source": source,
            "force_normal": [round(v, 1) for v in normal_forces],
            "target_force": [round(v, 1) for v in self._target_forces],
            "wrist_deg": round(self._wrist_position_deg, 1),
            "wrist_target_deg": round(self._wrist_target_deg, 1),
            "haptic_phase": self._haptic_phase,
            "csv_dir": str(self._log.run_dir),
        }
        self._status_pub.publish(String(data=json.dumps(payload)))

    def _write_sample(self) -> None:
        elapsed = self._elapsed()
        now_ns = time.time_ns()
        ros_time = self.get_clock().now().nanoseconds / 1e9
        normal_forces, source = self._current_normal_forces()
        emg_age = "" if self._last_emg_time <= 0.0 else time.monotonic() - self._last_emg_time
        row: dict[str, Any] = {
            "run_id": self._log.run_id,
            "wall_time_ns": now_ns,
            "elapsed_s": f"{elapsed:.6f}",
            "ros_time_s": f"{ros_time:.6f}",
            "state": self._stage.value,
            "state_elapsed_s": f"{self._stage_elapsed():.6f}",
            "control_mode": self._control_mode.value,
            "last_event": self._last_event,
            "buzz_label": self._buzz_label,
            "buzz_active": int(time.monotonic() < self._buzz_until),
            "contact_reason": self._contact_reason,
            "fault_reason": self._fault_reason,
            "controller_mode": self._controller_mode,
            "gesture_label": self._gesture_label,
            "gesture_name": self._gesture_name,
            "confidence": f"{self._confidence:.6f}",
            "proportional": f"{self._proportional:.6f}",
            "emg_age_s": "" if emg_age == "" else f"{emg_age:.6f}",
            "open_hold_s": f"{self._held_time(int(self._cfg.emg['open_label'])):.6f}",
            "power_hold_s": f"{self._held_time(int(self._cfg.emg['wrist_toggle_label'])):.6f}",
            "force_source": source,
            "force_percent": f"{self._force_percent():.6f}",
            "haptic_phase": self._haptic_phase,
            "wrist_position_deg": f"{self._wrist_position_deg:.6f}",
            "wrist_velocity_deg_s": f"{self._wrist_velocity_deg_s:.6f}",
            "wrist_target_deg": f"{self._wrist_target_deg:.6f}",
            "wrist_error_deg": f"{_circ_delta_deg(self._wrist_target_deg, self._wrist_position_deg):.6f}",
        }
        for i, label in enumerate(FINGER_LABELS):
            row[f"hand_pos_{label}_rad"] = f"{self._positions[i]:.6f}"
            row[f"hand_vel_{label}_rad_s"] = f"{self._velocities[i]:.6f}"
            row[f"joint_effort_{label}"] = f"{self._joint_efforts[i]:.6f}"
            row[f"force_normal_{label}"] = f"{normal_forces[i]:.6f}"
            row[f"force_tangential_{label}"] = f"{self._force_tangential[i]:.6f}"
            row[f"target_force_{label}"] = f"{self._target_forces[i]:.6f}"
            row[f"velocity_cmd_{label}_rad_s"] = f"{self._last_vel_cmd[i]:.6f}"
            row[f"position_cmd_{label}_rad"] = f"{self._last_pos_cmd[i]:.6f}"
            row[f"raw_joint_pos_{label}"] = self._optional(self._raw_joint_pos[i])
            row[f"raw_joint_speed_{label}"] = self._optional(self._raw_joint_speed[i])
            row[f"raw_motor_pos_{label}"] = self._optional(self._raw_motor_pos[i])
            row[f"raw_motor_speed_{label}"] = self._optional(self._raw_motor_speed[i])
            row[f"raw_motor_current_{label}"] = self._optional(self._raw_motor_current[i])
        for i in range(MOTOR_COUNT):
            value = self._last_haptics[i] if i < len(self._last_haptics) else 0.0
            row[f"haptic_motor_{i}_pct"] = f"{value:.6f}"
        self._log.write_sample(row)

    def _write_event(self, event: str, detail: str) -> None:
        self._log.write_event(
            {
                "run_id": self._log.run_id,
                "wall_time_ns": time.time_ns(),
                "elapsed_s": f"{self._elapsed():.6f}" if hasattr(self, "_start_time") else "0.000000",
                "event": event,
                "state": self._stage.value if hasattr(self, "_stage") else "",
                "control_mode": self._control_mode.value if hasattr(self, "_control_mode") else "",
                "detail": detail,
                "gesture_label": getattr(self, "_gesture_label", ""),
                "gesture_name": getattr(self, "_gesture_name", ""),
                "confidence": f"{getattr(self, '_confidence', 0.0):.6f}",
                "proportional": f"{getattr(self, '_proportional', 0.0):.6f}",
            }
        )

    def _stage_elapsed(self) -> float:
        return time.monotonic() - self._stage_started_at

    def _elapsed(self) -> float:
        return time.monotonic() - self._start_time

    def _held_time(self, label: int) -> float:
        if self._gesture_label != label:
            return 0.0
        return time.monotonic() - self._gesture_started_at

    def _optional(self, value: Any) -> str:
        if value is None:
            return ""
        return f"{float(value):.6f}"

    def destroy_node(self) -> bool:
        """Shutdown: open hand, return wrist horizontal, zero haptics.

        Runs on ANY termination path — clean COMPLETE, FAULT, SIGTERM,
        or KeyboardInterrupt.  Publishes are best-effort; if the ROS 2
        context is already torn down the exceptions are swallowed.
        """
        try:
            self._publish_haptics([0.0] * int(self._cfg.haptics["motor_count"]))
            self._publish_velocity([0.0] * FINGER_COUNT, force=True)
            if self._initialised and self._cm_client.services_ready():
                self._switch_position_controller()
            self._publish_position(self._cfg.open_positions, force=True)
            self._publish_wrist(
                float(self._cfg.wrist["horizontal_deg"]), force=True
            )
        except Exception:
            pass
        try:
            self._write_event("shutdown", "node destroyed")
            self._log.close()
        except Exception:
            pass
        try:
            self._cm_client.shutdown()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    node = MiaHapticForceTest()

    def _signal_handler(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _signal_handler)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping Mia haptic force test")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
