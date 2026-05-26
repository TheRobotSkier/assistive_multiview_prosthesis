#!/usr/bin/env python3
"""Bridge live EMG gestures to Mia hand force hold and wrist control."""

from __future__ import annotations

import json
import math
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray, Int32, String

from force_controller.controller_manager_client import ControllerManagerClient


FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_COUNT = 3

REST = 0
POWER = 1
OPEN = 2
FLEXION = 3
EXTENSION = 4

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


class Mode(Enum):
    NOT_GRASPING = auto()
    CONTROL_GRASP = auto()
    CONTROL_WRIST = auto()


class HandPhase(Enum):
    OPEN = auto()
    CLOSING = auto()
    FORCE_HOLD = auto()
    RELEASING = auto()
    FAULT = auto()


@dataclass
class HoldConfig:
    duration_s: float = 0.0
    targets: list[float] = field(default_factory=lambda: [300.0] * FINGER_COUNT)
    deadzone: float = 20.0
    min_overshoot: float = 20.0
    max_overshoot: float = 150.0
    max_velocity: float = 0.08


@dataclass
class RuntimeConfig:
    closing_start: float = 0.3
    closing_end: float = 0.1
    decay_steps: int = 5
    step_interval_s: float = 0.2
    stop_positions: list[float] = field(default_factory=lambda: [1.5] * FINGER_COUNT)
    force_thresholds: list[float] = field(default_factory=lambda: [300.0] * FINGER_COUNT)
    open_positions: list[float] = field(default_factory=lambda: [0.0] * FINGER_COUNT)
    relaxed_wait_s: float = 0.5
    hold: HoldConfig = field(default_factory=HoldConfig)
    force_target_min: float = 50.0
    force_target_max: float = 500.0
    force_adjust_rate_up: float = 100.0
    force_adjust_rate_down: float = 200.0
    max_force_emergency: float = 800.0
    emergency_backoff_velocity: float = -0.1
    emg_confidence_threshold: float = 0.55
    open_hold_s: float = 0.5
    open_prop_threshold: float = 0.7
    power_hold_s: float = 0.5
    power_rearm_s: float = 0.3
    stale_emg_timeout_s: float = 1.0
    stale_force_timeout_s: float = 1.0
    control_rate_hz: float = 20.0
    wrist_enabled: bool = True
    wrist_step_deg: float = 1.5
    wrist_min_deg: float = -45.0
    wrist_max_deg: float = 45.0
    wrist_accel_deg_s2: float = 180.0
    wrist_stale_timeout_s: float = 1.0


def _dict_values_by_finger(raw: dict, default: float) -> list[float]:
    return [float(raw.get(name, default)) for name in FINGER_JOINTS]


def _load_config(path: str) -> RuntimeConfig:
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    cfg = RuntimeConfig()

    if "closing_velocity_start" in raw:
        cfg.closing_start = float(raw.get("closing_velocity_start", cfg.closing_start))
        cfg.closing_end = float(raw.get("closing_velocity_end", cfg.closing_end))
        cfg.decay_steps = int(raw.get("decay_steps", cfg.decay_steps))
        cfg.step_interval_s = float(raw.get("step_interval_s", cfg.step_interval_s))
        cfg.stop_positions = _dict_values_by_finger(raw.get("stop_positions", {}), 1.5)
        cfg.force_thresholds = _dict_values_by_finger(raw.get("force_thresholds", {}), 300.0)
        cfg.relaxed_wait_s = float(raw.get("relaxed_wait_s", cfg.relaxed_wait_s))
        cfg.emg_confidence_threshold = float(
            raw.get("emg_grasp_confidence_threshold", cfg.emg_confidence_threshold)
        )
        cfg.power_hold_s = float(raw.get("gesture_hold_timeout_s", cfg.power_hold_s))
        cfg.open_hold_s = float(raw.get("open_safety_hold_s", cfg.open_hold_s))

        wh = raw.get("force_hold", {})
        cfg.hold.targets = _dict_values_by_finger(
            wh.get("target_forces", raw.get("force_thresholds", {})), 300.0
        )
        cfg.hold.deadzone = float(wh.get("deadzone", cfg.hold.deadzone))
        cfg.hold.min_overshoot = float(wh.get("min_overshoot", cfg.hold.min_overshoot))
        cfg.hold.max_overshoot = float(wh.get("max_overshoot", cfg.hold.max_overshoot))
        cfg.hold.max_velocity = float(wh.get("max_velocity", cfg.hold.max_velocity))

        wc = raw.get("wrist_control", {})
        cfg.wrist_enabled = bool(wc.get("enabled", cfg.wrist_enabled))
        cfg.wrist_step_deg = float(wc.get("step_size_deg", cfg.wrist_step_deg))
        cfg.wrist_min_deg = float(wc.get("min_position_deg", cfg.wrist_min_deg))
        cfg.wrist_max_deg = float(wc.get("max_position_deg", cfg.wrist_max_deg))
        cfg.wrist_accel_deg_s2 = float(wc.get("wrist_accel_deg_s2", cfg.wrist_accel_deg_s2))
        cfg.wrist_stale_timeout_s = float(
            wc.get("stale_state_timeout_s", cfg.wrist_stale_timeout_s)
        )
        if "command_rate_hz" in wc:
            cfg.control_rate_hz = float(wc["command_rate_hz"])
    else:
        velocity = raw.get("velocity", {})
        force = raw.get("force", {})
        positions = raw.get("positions", {})
        safety = raw.get("safety", {})
        command = raw.get("command", {})
        wrist = raw.get("wrist", {})
        emg = raw.get("emg", {})

        cfg.closing_start = float(velocity.get("closing_start", cfg.closing_start))
        cfg.closing_end = float(velocity.get("closing_end", cfg.closing_end))
        cfg.decay_steps = int(velocity.get("decay_steps", cfg.decay_steps))
        cfg.step_interval_s = float(velocity.get("step_interval_s", cfg.step_interval_s))
        cfg.stop_positions = _dict_values_by_finger(
            positions.get("max_closure", {}), 1.5
        )
        cfg.open_positions = _dict_values_by_finger(positions.get("open", {}), 0.0)
        targets = force.get("targets", {})
        cfg.force_thresholds = _dict_values_by_finger(targets, 300.0)
        cfg.hold.targets = list(cfg.force_thresholds)
        cfg.hold.deadzone = float(force.get("stability_tolerance", cfg.hold.deadzone))
        cfg.hold.max_velocity = float(velocity.get("max_velocity", cfg.hold.max_velocity))
        cfg.force_target_min = float(force.get("target_min", cfg.force_target_min))
        cfg.force_target_max = float(force.get("target_max", cfg.force_target_max))
        cfg.force_adjust_rate_up = float(force.get("adjustment_rate_up", cfg.force_adjust_rate_up))
        cfg.force_adjust_rate_down = float(
            force.get("adjustment_rate_down", cfg.force_adjust_rate_down)
        )
        cfg.max_force_emergency = float(
            force.get("emergency_threshold", cfg.max_force_emergency)
        )
        cfg.emg_confidence_threshold = float(
            emg.get("confidence_threshold", cfg.emg_confidence_threshold)
        )
        cfg.open_hold_s = float(safety.get("open_safety_hold_s", cfg.open_hold_s))
        cfg.power_hold_s = float(safety.get("power_toggle_hold_s", cfg.power_hold_s))
        cfg.power_rearm_s = float(safety.get("power_debounce_s", cfg.power_rearm_s))
        cfg.stale_emg_timeout_s = float(
            command.get("stale_emg_timeout_s", cfg.stale_emg_timeout_s)
        )
        cfg.stale_force_timeout_s = float(
            command.get("stale_force_timeout_s", cfg.stale_force_timeout_s)
        )
        cfg.control_rate_hz = float(command.get("emg_update_rate_hz", cfg.control_rate_hz))
        cfg.wrist_step_deg = float(
            wrist.get("max_velocity", 30.0) / max(cfg.control_rate_hz, 1.0)
        )
        cfg.wrist_min_deg = float(wrist.get("position_min", cfg.wrist_min_deg))
        cfg.wrist_max_deg = float(wrist.get("position_max", cfg.wrist_max_deg))

    cfg.hold.targets = [
        max(cfg.force_target_min, min(cfg.force_target_max, t)) for t in cfg.hold.targets
    ]
    return cfg


def _competitors(active: list[str]) -> list[str]:
    active_set = set(active)
    return [c for c in POSITION_CONTROLLERS + VELOCITY_CONTROLLERS if c not in active_set]

def _velocity_ramp(start: float, end: float, steps: int) -> list[float]:
    if steps <= 1:
        return [start]
    return [start + (i / (steps - 1)) * (end - start) for i in range(steps)]


def _hold_velocity(force: float, target: float, cfg: HoldConfig) -> float:
    error = target - force
    abs_error = abs(error)
    if abs_error <= cfg.deadzone:
        return 0.0
    if cfg.max_overshoot <= 0.0:
        return cfg.max_velocity if error > 0.0 else -cfg.max_velocity
    scaled = min(max(abs_error, cfg.min_overshoot), cfg.max_overshoot)
    velocity = cfg.max_velocity * (scaled / cfg.max_overshoot)
    return velocity if error > 0.0 else -velocity


class EmgForceGraspBridge(Node):
    def __init__(self) -> None:
        super().__init__("emg_force_grasp_bridge")

        config_path = self.declare_parameter(
            "config_path", "/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml"
        ).value
        self._cfg = _load_config(config_path)

        self._vel_pub = self.create_publisher(
            Float64MultiArray, "/group_vel_ff_controller/commands", 10
        )
        self._pos_pub = self.create_publisher(
            Float64MultiArray, "/group_pos_ff_controller/commands", 10
        )
        self._wrist_pub = self.create_publisher(
            Float64MultiArray, "/wrist/set_position", 10
        )
        self._mode_pub = self.create_publisher(String, "/emg_force_grasp/mode", 10)
        self._status_pub = self.create_publisher(String, "/emg_force_grasp/status", 10)

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(Int32, "/emg/gesture_label", self._on_gesture, 10)
        self.create_subscription(String, "/emg/gesture_name", self._on_gesture_name, 10)
        self.create_subscription(Float32, "/emg/confidence", self._on_confidence, 10)
        self.create_subscription(Float32, "/emg/proportional", self._on_proportional, 10)
        self.create_subscription(Float64MultiArray, "/wrist/state", self._on_wrist_state, 10)

        self._positions = [0.0] * FINGER_COUNT
        self._efforts = [0.0] * FINGER_COUNT
        self._got_joint_states = False
        self._last_force_time = 0.0

        self._gesture = REST
        self._gesture_name = "REST"
        self._confidence = 0.0
        self._proportional = 0.0
        self._gesture_started_at = time.monotonic()
        self._last_emg_time = 0.0
        self._power_armed = True
        self._last_power_toggle = 0.0

        self._mode = Mode.NOT_GRASPING
        self._hand_phase = HandPhase.OPEN
        self._ramp = _velocity_ramp(
            self._cfg.closing_start, self._cfg.closing_end, self._cfg.decay_steps
        )
        self._ramp_idx = 0
        self._target_forces = list(self._cfg.hold.targets)
        self._last_vel_cmd: Optional[list[float]] = None
        self._last_pos_cmd: Optional[list[float]] = None
        self._last_status_time = 0.0

        self._wrist_position_deg = 0.0
        self._wrist_neutral_deg: Optional[float] = None
        self._wrist_relative_target_deg = 0.0
        self._last_wrist_cmd: Optional[list[float]] = None
        self._wrist_state_time = 0.0
        self._got_wrist_state = False
        self._settling_timer: Optional[object] = None

        self._cm_client = ControllerManagerClient(self)
        if not self._cm_client.wait_for_services(timeout_sec=30):
            raise RuntimeError("controller_manager did not become available")
        self._cm_client.switch_controllers(["group_pos_ff_controller"], _competitors(["group_pos_ff_controller"]))
        self._publish_position(self._cfg.open_positions, force=True)

        rate = max(self._cfg.control_rate_hz, 1.0)
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "EMG force grasp bridge ready: POWER starts/toggles, OPEN safety resets, "
            "FLEXION/EXTENSION drive wrist or force target by mode"
        )

    def _on_joint_states(self, msg: JointState) -> None:
        try:
            have_effort = True
            for i, name in enumerate(FINGER_JOINTS):
                idx = msg.name.index(name)
                self._positions[i] = float(msg.position[idx])
                if idx < len(msg.effort):
                    self._efforts[i] = float(msg.effort[idx])
                else:
                    have_effort = False
            self._got_joint_states = True
            if have_effort:
                self._last_force_time = time.monotonic()
        except ValueError:
            return

    def _on_gesture(self, msg: Int32) -> None:
        self._update_gesture(int(msg.data), None)

    def _on_gesture_name(self, msg: String) -> None:
        self._gesture_name = msg.data

    def _on_confidence(self, msg: Float32) -> None:
        self._confidence = float(msg.data)
        self._last_emg_time = time.monotonic()

    def _on_proportional(self, msg: Float32) -> None:
        self._proportional = max(0.0, min(1.0, float(msg.data)))
        self._last_emg_time = time.monotonic()

    def _on_wrist_state(self, msg: Float64MultiArray) -> None:
        if msg.data:
            self._wrist_position_deg = float(msg.data[0])
            if self._wrist_neutral_deg is None:
                self._wrist_neutral_deg = self._wrist_position_deg
                self._wrist_relative_target_deg = 0.0
                self.get_logger().info(
                    f"Wrist neutral set to {self._wrist_neutral_deg:.1f} deg"
                )
            self._wrist_state_time = time.monotonic()
            self._got_wrist_state = True

    def _update_gesture(self, gesture: int, name: Optional[str]) -> None:
        now = time.monotonic()
        if gesture != self._gesture:
            self._gesture = gesture
            self._gesture_started_at = now
            if gesture != POWER:
                self._power_armed = True
        if name is not None:
            self._gesture_name = name
        self._last_emg_time = now

    def _gesture_held(self, gesture: int, seconds: float) -> bool:
        return self._gesture == gesture and (time.monotonic() - self._gesture_started_at) >= seconds

    def _emg_active(self, gesture: int) -> bool:
        return self._gesture == gesture and self._confidence >= self._cfg.emg_confidence_threshold

    def _publish_velocity(self, velocities: list[float], force: bool = False) -> None:
        if not force and self._last_vel_cmd is not None:
            if all(math.isclose(a, b, abs_tol=1e-4) for a, b in zip(velocities, self._last_vel_cmd)):
                return
        msg = Float64MultiArray()
        msg.data = [float(v) for v in velocities]
        self._vel_pub.publish(msg)
        self._last_vel_cmd = list(msg.data)

    def _publish_position(self, positions: list[float], force: bool = False) -> None:
        if not force and self._last_pos_cmd is not None:
            if all(math.isclose(a, b, abs_tol=1e-4) for a, b in zip(positions, self._last_pos_cmd)):
                return
        msg = Float64MultiArray()
        msg.data = [float(p) for p in positions]
        self._pos_pub.publish(msg)
        self._last_pos_cmd = list(msg.data)

    def _publish_wrist(self, target_deg: float, force: bool = False) -> None:
        cmd = [float(target_deg), float(self._cfg.wrist_accel_deg_s2)]
        if not force and self._last_wrist_cmd is not None:
            if all(math.isclose(a, b, abs_tol=0.1) for a, b in zip(cmd, self._last_wrist_cmd)):
                return
        msg = Float64MultiArray()
        msg.data = cmd
        self._wrist_pub.publish(msg)
        self._last_wrist_cmd = list(cmd)

    def _set_mode(self, mode: Mode, reason: str) -> None:
        if mode != self._mode:
            self.get_logger().info(f"Mode {self._mode.name} -> {mode.name}: {reason}")
            self._mode = mode
            self._mode_pub.publish(String(data=mode.name))

    def _toggle_power_if_ready(self) -> None:
        now = time.monotonic()
        if not self._emg_active(POWER):
            if self._gesture != POWER:
                self._power_armed = True
            return
        if not self._power_armed:
            return
        if now - self._last_power_toggle < self._cfg.power_rearm_s:
            return
        if not self._gesture_held(POWER, self._cfg.power_hold_s):
            return

        self._power_armed = False
        self._last_power_toggle = now
        if self._mode == Mode.NOT_GRASPING:
            self._set_mode(Mode.CONTROL_GRASP, "POWER held")
            self._start_grasp()
        elif self._mode == Mode.CONTROL_GRASP:
            self._set_mode(Mode.CONTROL_WRIST, "POWER held")
        else:
            self._set_mode(Mode.CONTROL_GRASP, "POWER held")
            if self._hand_phase == HandPhase.OPEN:
                self._start_grasp()

    def _open_safety_active(self) -> bool:
        return (
            self._gesture == OPEN
            and self._confidence >= self._cfg.emg_confidence_threshold
            and self._proportional >= self._cfg.open_prop_threshold
            and self._gesture_held(OPEN, self._cfg.open_hold_s)
        )

    def _start_grasp(self) -> None:
        if not self._got_joint_states:
            self.get_logger().warn("POWER received but no joint_states yet; waiting")
            return
        self._publish_velocity([0.0] * FINGER_COUNT, force=True)
        self._publish_position(self._cfg.open_positions, force=True)

        # Use a one-shot timer instead of blocking sleep to keep the
        # executor responsive during the settling period.
        self._settling_timer = self.create_timer(
            self._cfg.relaxed_wait_s, self._finish_start_grasp
        )
        self._settling_timer  # suppress unused warning

    def _finish_start_grasp(self) -> None:
        """Called after the settling delay to perform the actual controller switch."""
        if self._settling_timer is not None:
            self.destroy_timer(self._settling_timer)
            self._settling_timer = None

        try:
            self._cm_client.switch_controllers(
                ["group_vel_ff_controller"], _competitors(["group_vel_ff_controller"])
            )
        except Exception as exc:
            self._fault(f"Failed to switch to velocity controller: {exc}")
            return
        self._ramp_idx = 0
        self._hand_phase = HandPhase.CLOSING
        self.get_logger().info("Hand CLOSING: velocity ramp active")

    def _release_hand(self, reason: str) -> None:
        self.get_logger().info(f"Hand release/reset: {reason}")
        self._hand_phase = HandPhase.RELEASING
        self._publish_velocity([0.0] * FINGER_COUNT, force=True)
        # Cancel any pending settling timer
        if self._settling_timer is not None:
            self.destroy_timer(self._settling_timer)
            self._settling_timer = None
        try:
            self._cm_client.switch_controllers(
                ["group_pos_ff_controller"], _competitors(["group_pos_ff_controller"])
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to switch to position controller during release: {exc}")
        self._publish_position(self._cfg.open_positions, force=True)
        self._hand_phase = HandPhase.OPEN
        self._set_mode(Mode.NOT_GRASPING, reason)

    def _fault(self, reason: str) -> None:
        self.get_logger().error(f"FAULT: {reason}")
        self._hand_phase = HandPhase.FAULT
        self._publish_velocity([0.0] * FINGER_COUNT, force=True)

    def _adjust_force_target(self, dt: float) -> None:
        if self._mode != Mode.CONTROL_GRASP:
            return
        delta = 0.0
        if self._emg_active(FLEXION):
            delta = self._cfg.force_adjust_rate_up * self._proportional * dt
        elif self._emg_active(EXTENSION):
            delta = -self._cfg.force_adjust_rate_down * self._proportional * dt
        if delta == 0.0:
            return
        self._target_forces = [
            max(self._cfg.force_target_min, min(self._cfg.force_target_max, t + delta))
            for t in self._target_forces
        ]

    def _run_hand(self) -> None:
        now = time.monotonic()
        if self._hand_phase in (HandPhase.CLOSING, HandPhase.FORCE_HOLD):
            if now - self._last_force_time > self._cfg.stale_force_timeout_s:
                self._fault("force data stale")
                return
            if max(self._efforts) >= self._cfg.max_force_emergency:
                self._publish_velocity([self._cfg.emergency_backoff_velocity] * FINGER_COUNT, force=True)
                self._fault(f"emergency force exceeded: {self._efforts}")
                return

        if self._hand_phase == HandPhase.CLOSING:
            contact = self._first_contact_reason()
            if contact:
                self.get_logger().info(f"Force hold entry: {contact}")
                self._hand_phase = HandPhase.FORCE_HOLD
                self._publish_velocity([0.0] * FINGER_COUNT, force=True)
                return

            if self._ramp_idx < len(self._ramp):
                vel = self._ramp[self._ramp_idx]
                self._ramp_idx += 1
            else:
                vel = self._cfg.closing_end
            self._publish_velocity([vel] * FINGER_COUNT)

        elif self._hand_phase == HandPhase.FORCE_HOLD:
            velocities = [
                _hold_velocity(self._efforts[i], self._target_forces[i], self._cfg.hold)
                for i in range(FINGER_COUNT)
            ]
            self._publish_velocity(velocities)

    def _first_contact_reason(self) -> Optional[str]:
        for i, name in enumerate(FINGER_JOINTS):
            if self._efforts[i] >= self._cfg.force_thresholds[i]:
                return f"{name} force {self._efforts[i]:.0f} >= {self._cfg.force_thresholds[i]:.0f}"
        for i, name in enumerate(FINGER_JOINTS):
            if self._positions[i] >= self._cfg.stop_positions[i]:
                return f"{name} position {self._positions[i]:.3f} >= {self._cfg.stop_positions[i]:.2f}"
        return None

    def _run_wrist(self) -> None:
        if not self._cfg.wrist_enabled:
            return
        if self._mode not in (Mode.NOT_GRASPING, Mode.CONTROL_WRIST):
            return
        if self._gesture not in (FLEXION, EXTENSION):
            return
        if self._confidence < self._cfg.emg_confidence_threshold:
            return
        if not self._got_wrist_state:
            self.get_logger().warn("Wrist gesture received, but no /wrist/state yet")
            return
        if self._wrist_neutral_deg is None:
            return
        if time.monotonic() - self._wrist_state_time > self._cfg.wrist_stale_timeout_s:
            self.get_logger().warn("Wrist state stale; not commanding wrist")
            return

        direction = 1.0 if self._gesture == FLEXION else -1.0
        step = direction * self._cfg.wrist_step_deg * max(self._proportional, 0.2)

        # The hardware driver accepts absolute 0..360 degree setpoints. The
        # config limits are user-facing relative limits around startup neutral.
        min_rel = max(self._cfg.wrist_min_deg, -self._wrist_neutral_deg)
        max_rel = min(self._cfg.wrist_max_deg, 359.0 - self._wrist_neutral_deg)
        self._wrist_relative_target_deg = max(
            min_rel,
            min(max_rel, self._wrist_relative_target_deg + step),
        )
        target = self._wrist_neutral_deg + self._wrist_relative_target_deg
        self._publish_wrist(target)

    def _publish_status(self) -> None:
        now = time.monotonic()
        if now - self._last_status_time < 1.0:
            return
        self._last_status_time = now
        payload = {
            "mode": self._mode.name,
            "hand_phase": self._hand_phase.name,
            "gesture": self._gesture_name,
            "gesture_id": self._gesture,
            "confidence": round(self._confidence, 3),
            "proportional": round(self._proportional, 3),
            "force": [round(v, 1) for v in self._efforts],
            "target": [round(v, 1) for v in self._target_forces],
            "wrist_deg": round(self._wrist_position_deg, 1),
            "wrist_rel_deg": round(self._wrist_position_deg - self._wrist_neutral_deg, 1)
            if self._wrist_neutral_deg is not None
            else 0.0,
        }
        self._status_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f"{payload['mode']} {payload['hand_phase']} "
            f"gesture={payload['gesture']} conf={payload['confidence']:.2f} "
            f"prop={payload['proportional']:.2f} "
            f"force={payload['force']} target={payload['target']} "
            f"wrist={payload['wrist_deg']} rel={payload['wrist_rel_deg']}"
        )

    def _tick(self) -> None:
        dt = 1.0 / max(self._cfg.control_rate_hz, 1.0)
        if self._last_emg_time and time.monotonic() - self._last_emg_time > self._cfg.stale_emg_timeout_s:
            self._gesture = REST
            self._gesture_name = "REST"
            self._confidence = 0.0
            self._proportional = 0.0

        if self._open_safety_active():
            self._release_hand("OPEN safety")
            self._publish_status()
            return

        self._toggle_power_if_ready()
        self._adjust_force_target(dt)
        self._run_hand()
        self._run_wrist()
        self._publish_status()

    def destroy_node(self) -> bool:
        try:
            self._publish_velocity([0.0] * FINGER_COUNT, force=True)
            if self._cm_client.services_ready():
                self._cm_client.switch_controllers(
                    ["group_pos_ff_controller"], _competitors(["group_pos_ff_controller"])
                )
            self._publish_position(self._cfg.open_positions, force=True)
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)
    node = EmgForceGraspBridge()

    def _signal_handler(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _signal_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping EMG force grasp bridge")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
