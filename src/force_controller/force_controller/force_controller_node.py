#!/usr/bin/env python3
"""Velocity-Based Force Controller Node for Mia Hand.

Uses the same velocity-ramp closing + deadzone force-hold pattern as
``scripts/emg_force_grasp_bridge.py``, but is fully drop-in compatible with
the production pipeline manager.

Pipeline integration:
    - Subscribes to /pipeline/state, activates on GRASPING, deactivates on
      leaving GRASPING/HOLDING/VOLITIONAL, resets on RELEASING.
    - Publishes ForceControllerStatus on /force_controller/status for the
      pipeline manager's state transitions.
    - Accepts manual force-target adjustments via /force_controller/manual_adjust
      for volitional control.

Control flow:
    1. On GRASPING entry: switch from position controllers to
       ``group_vel_ff_controller``, start decaying velocity ramp (close).
    2. Contact detected (force >= threshold or position >= stop limit)
       → FORCE_HOLD phase: deadzone-based proportional velocity hold per finger.
    3. Stable force detected → force_stable flag → pipeline transitions to
       HOLDING → VOLITIONAL.
    4. On RELEASING: switch to ``group_pos_ff_controller``, publish open
       positions, reset internal state.
    5. On shutdown: safe stop + open hand.

Controller switching uses ``ros2 service call`` on the controller_manager
services (same as the EMG bridge), ensuring runtime compatibility with the
ros2_control stack.
"""

from __future__ import annotations

import math
import re
import subprocess
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from mia_hand_msgs.msg import ForceData, ForceControllerStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, Int32
from std_srvs.srv import SetBool


STATE_IDLE = 0
STATE_TWISTING = 1
STATE_SEGMENTING = 2
STATE_PLANNING = 3
STATE_APPROACHING = 4
STATE_GRASPING = 5
STATE_HOLDING = 6
STATE_VOLITIONAL = 7
STATE_RELEASING = 8

STATE_NAMES = {
    STATE_IDLE: "IDLE",
    STATE_TWISTING: "TWISTING",
    STATE_SEGMENTING: "SEGMENTING",
    STATE_PLANNING: "PLANNING",
    STATE_APPROACHING: "APPROACHING",
    STATE_GRASPING: "GRASPING",
    STATE_HOLDING: "HOLDING",
    STATE_VOLITIONAL: "VOLITIONAL",
    STATE_RELEASING: "RELEASING",
}

FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_COUNT = 3

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


def _ros(*args: str, timeout: float = 10.0, check: bool = True) -> str:
    cmd = ["ros2"] + list(args)
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if check and result.returncode != 0:
        raise RuntimeError(
            f"{' '.join(cmd)} failed: {result.stderr.strip() or result.stdout.strip()}"
        )
    return result.stdout


def _wait_for_controller_manager(timeout_s: float = 20.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        result = subprocess.run(
            ["ros2", "service", "list"],
            capture_output=True,
            text=True,
            timeout=3,
        )
        if "/controller_manager/list_controllers" in result.stdout:
            return True
        time.sleep(0.25)
    return False


def _controller_states() -> dict[str, str]:
    out = _ros(
        "service",
        "call",
        "/controller_manager/list_controllers",
        "controller_manager_msgs/srv/ListControllers",
        "{}",
        timeout=5,
        check=False,
    )
    return {name: state for name, state in re.findall(r"name='([^']+)'.*?state='([^']+)'", out)}


def _ensure_controller_loaded(ctrl: str) -> None:
    states = _controller_states()
    if ctrl in states:
        return
    req = f"{{name: '{ctrl}'}}"
    out = _ros(
        "service",
        "call",
        "/controller_manager/load_controller",
        "controller_manager_msgs/srv/LoadController",
        req,
        timeout=10,
        check=False,
    )
    if "ok=True" not in out:
        raise RuntimeError(f"Failed to load {ctrl}: {out.strip()}")
    out = _ros(
        "service",
        "call",
        "/controller_manager/configure_controller",
        "controller_manager_msgs/srv/ConfigureController",
        req,
        timeout=10,
        check=False,
    )
    if "ok=True" not in out:
        raise RuntimeError(f"Failed to configure {ctrl}: {out.strip()}")


def _competitors(active: list[str]) -> list[str]:
    active_set = set(active)
    return [c for c in POSITION_CONTROLLERS + VELOCITY_CONTROLLERS if c not in active_set]


def _switch_controllers(activate: list[str], deactivate: list[str]) -> None:
    if not _wait_for_controller_manager(timeout_s=5):
        raise RuntimeError("controller_manager service is not available")

    states = _controller_states()
    activate = [ctrl for ctrl in activate if states.get(ctrl) != "active"]
    deactivate = [ctrl for ctrl in deactivate if states.get(ctrl) == "active"]
    if not activate and not deactivate:
        return
    for ctrl in activate:
        _ensure_controller_loaded(ctrl)
    req = (
        "{"
        f"activate_controllers: [{', '.join(repr(c) for c in activate)}], "
        f"deactivate_controllers: [{', '.join(repr(c) for c in deactivate)}], "
        "strictness: 2, activate_asap: true, timeout: {sec: 10, nanosec: 0}"
        "}"
    )
    out = _ros(
        "service",
        "call",
        "/controller_manager/switch_controller",
        "controller_manager_msgs/srv/SwitchController",
        req,
        timeout=15,
        check=False,
    )
    if "ok=True" not in out:
        req2 = req.replace("strictness: 2", "strictness: 1")
        out = _ros(
            "service",
            "call",
            "/controller_manager/switch_controller",
            "controller_manager_msgs/srv/SwitchController",
            req2,
            timeout=15,
            check=False,
        )
        if "ok=True" not in out:
            raise RuntimeError(f"Controller switch failed: {out.strip()}")


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


class ForceControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("force_controller")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("target_force_min", 50.0)
        self.declare_parameter("target_force_max", 200.0)
        self.declare_parameter("update_rate_hz", 20.0)

        self.declare_parameter("stability_window", 1.0)
        self.declare_parameter("stability_tolerance", 30.0)
        self.declare_parameter("slip_threshold", 100.0)
        self.declare_parameter("max_force_emergency", 500.0)
        self.declare_parameter("stale_data_timeout_s", 0.5)
        self.declare_parameter("force_filter_window", 5)
        self.declare_parameter("manual_adjust_topic", "/force_controller/manual_adjust")
        self.declare_parameter("manual_adjust_step", 10.0)

        self.declare_parameter("closing_velocity_start", 0.3)
        self.declare_parameter("closing_velocity_end", 0.1)
        self.declare_parameter("decay_steps", 5)
        self.declare_parameter("step_interval_s", 0.2)
        self.declare_parameter("stop_positions", [1.5, 1.5, 1.5])
        self.declare_parameter("force_thresholds", [300.0, 300.0, 300.0])
        self.declare_parameter("open_positions", [0.0, 0.0, 0.0])
        self.declare_parameter("relaxed_wait_s", 0.5)
        self.declare_parameter("hold_deadzone", 20.0)
        self.declare_parameter("hold_max_velocity", 0.08)
        self.declare_parameter("hold_min_overshoot", 20.0)
        self.declare_parameter("hold_max_overshoot", 150.0)
        self.declare_parameter("emergency_backoff_velocity", -0.1)
        self.declare_parameter("force_adjust_rate_up", 15.0)
        self.declare_parameter("force_adjust_rate_down", 30.0)

        self.declare_parameter("force_data_topic", "data_streams/fingers/forces/data")
        self.declare_parameter("force_stream_switch_service", "data_streams/fingers/forces/switch")
        self.declare_parameter("pipeline_state_topic", "/pipeline/state")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("vel_cmd_topic", "/group_vel_ff_controller/commands")
        self.declare_parameter("pos_cmd_topic", "/group_pos_ff_controller/commands")
        self.declare_parameter("force_status_topic", "/force_controller/status")
        self.declare_parameter("joint_position_limits", [3.14, 3.14, 3.14])
        self.declare_parameter("joint_position_min", [0.0, 0.0, 0.0])

        self._target_min = self.get_parameter("target_force_min").value
        self._target_max = self.get_parameter("target_force_max").value
        self._target_mid = (self._target_min + self._target_max) / 2.0

        self._stability_window = self.get_parameter("stability_window").value
        self._stability_tol = self.get_parameter("stability_tolerance").value
        self._slip_thresh = self.get_parameter("slip_threshold").value
        self._max_force_emerg = self.get_parameter("max_force_emergency").value
        self._filter_win = max(1, self.get_parameter("force_filter_window").value)
        self._stale_timeout = self.get_parameter("stale_data_timeout_s").value
        rate_hz = self.get_parameter("update_rate_hz").value
        self._dt = 1.0 / max(rate_hz, 1.0)

        self._closing_start = self.get_parameter("closing_velocity_start").value
        self._closing_end = self.get_parameter("closing_velocity_end").value
        self._decay_steps = int(self.get_parameter("decay_steps").value)
        self._step_interval_s = self.get_parameter("step_interval_s").value
        self._stop_positions = list(self.get_parameter("stop_positions").value)
        self._force_thresholds = list(self.get_parameter("force_thresholds").value)
        self._open_positions = list(self.get_parameter("open_positions").value)
        self._relaxed_wait_s = self.get_parameter("relaxed_wait_s").value
        self._hold_deadzone = self.get_parameter("hold_deadzone").value
        self._hold_max_velocity = self.get_parameter("hold_max_velocity").value
        self._hold_min_overshoot = self.get_parameter("hold_min_overshoot").value
        self._hold_max_overshoot = self.get_parameter("hold_max_overshoot").value
        self._emergency_backoff_vel = self.get_parameter("emergency_backoff_velocity").value
        self._force_adj_up = self.get_parameter("force_adjust_rate_up").value
        self._force_adj_down = self.get_parameter("force_adjust_rate_down").value

        self._force_data_topic = self.get_parameter("force_data_topic").value
        self._stream_switch_service = self.get_parameter("force_stream_switch_service").value
        self._pipeline_state_topic = self.get_parameter("pipeline_state_topic").value
        self._joint_states_topic = self.get_parameter("joint_states_topic").value
        self._vel_cmd_topic = self.get_parameter("vel_cmd_topic").value
        self._pos_cmd_topic = self.get_parameter("pos_cmd_topic").value
        self._force_status_topic = self.get_parameter("force_status_topic").value
        self._joint_pos_limits = list(self.get_parameter("joint_position_limits").value)
        self._joint_pos_min = list(self.get_parameter("joint_position_min").value)
        self._manual_adjust_topic = self.get_parameter("manual_adjust_topic").value
        self._manual_adjust_step = self.get_parameter("manual_adjust_step").value

        # ── Internal state ────────────────────────────────────────────────
        self._pipeline_state: int = STATE_IDLE
        self._controller_active: bool = False
        self._streaming_active: bool = False
        self._hand_phase: str = "IDLE"

        self._positions: list[float] = [0.0] * FINGER_COUNT
        self._efforts: list[float] = [0.0] * FINGER_COUNT
        self._joint_pos_received: bool = False

        self._normal_forces: list[float] = [0.0] * FINGER_COUNT
        self._tangential_forces: list[float] = [0.0] * FINGER_COUNT
        self._prev_tangential_forces: list[float] = [0.0] * FINGER_COUNT
        self._force_data_received: bool = False
        self._last_force_time: float = 0.0

        self._normal_bufs: list[deque[float]] = [
            deque(maxlen=self._filter_win) for _ in range(FINGER_COUNT)
        ]
        self._tangential_bufs: list[deque[float]] = [
            deque(maxlen=self._filter_win) for _ in range(FINGER_COUNT)
        ]

        self._stable_since: Optional[float] = None
        self._force_stable: bool = False
        self._slip_detected: bool = False

        self._target_forces: list[float] = [self._target_mid] * FINGER_COUNT

        self._ramp: list[float] = _velocity_ramp(
            self._closing_start, self._closing_end, self._decay_steps
        )
        self._ramp_idx: int = 0
        self._ramp_last_advance: float = 0.0

        self._phase_entry_time: float = 0.0

        self._last_vel_cmd: Optional[list[float]] = None
        self._last_pos_cmd: Optional[list[float]] = None

        # ── Service clients ───────────────────────────────────────────────
        self._stream_switch = self.create_client(
            SetBool, self._stream_switch_service
        )

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            ForceData,
            self._force_data_topic,
            self._on_forces,
            10,
        )
        self.create_subscription(
            Int32,
            self._pipeline_state_topic,
            self._on_pipeline_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._on_joint_states,
            10,
        )
        self.create_subscription(
            Float64,
            self._manual_adjust_topic,
            self._on_manual_adjust,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._vel_pub = self.create_publisher(
            Float64MultiArray, self._vel_cmd_topic, 10
        )
        self._pos_pub = self.create_publisher(
            Float64MultiArray, self._pos_cmd_topic, 10
        )
        self._status_pub = self.create_publisher(
            ForceControllerStatus, self._force_status_topic, 10
        )

        # ── Control timer ─────────────────────────────────────────────────
        self.create_timer(self._dt, self._control_tick)

        self.get_logger().info(
            f"Force controller started (velocity-based). "
            f"Target range: [{self._target_min}, {self._target_max}] raw, "
            f"rate={rate_hz} Hz, close_vel={self._closing_start}→{self._closing_end} rad/s, "
            f"hold_deadzone={self._hold_deadzone}, hold_max_v={self._hold_max_velocity}"
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _on_forces(self, msg: ForceData) -> None:
        raw_normal = [float(msg.thumb_nfor), float(msg.index_nfor), float(msg.mrl_nfor)]
        raw_tangential = [
            float(msg.thumb_tfor),
            float(msg.index_tfor),
            float(msg.mrl_tfor),
        ]
        for i in range(FINGER_COUNT):
            self._normal_bufs[i].append(raw_normal[i])
            self._tangential_bufs[i].append(raw_tangential[i])
        self._prev_tangential_forces = self._tangential_forces[:]
        self._normal_forces = [
            sum(self._normal_bufs[i]) / len(self._normal_bufs[i]) for i in range(FINGER_COUNT)
        ]
        self._tangential_forces = [
            sum(self._tangential_bufs[i]) / len(self._tangential_bufs[i])
            for i in range(FINGER_COUNT)
        ]
        self._force_data_received = True
        self._last_force_time = time.monotonic()

    def _on_pipeline_state(self, msg: Int32) -> None:
        new_state = msg.data
        if new_state == self._pipeline_state:
            return
        old_state = self._pipeline_state
        self._pipeline_state = new_state
        self.get_logger().info(
            f"Pipeline state: {STATE_NAMES.get(old_state, '?')} -> "
            f"{STATE_NAMES.get(new_state, '?')}"
        )

        if new_state == STATE_GRASPING and old_state in (
            STATE_APPROACHING,
            STATE_PLANNING,
        ):
            self._activate_controller()

        if new_state == STATE_RELEASING:
            self._reset_controller()
        elif old_state in (STATE_GRASPING, STATE_HOLDING, STATE_VOLITIONAL) and new_state not in (
            STATE_GRASPING,
            STATE_HOLDING,
            STATE_VOLITIONAL,
        ):
            self._on_leave_active_states()

    def _on_joint_states(self, msg: JointState) -> None:
        for i, name in enumerate(FINGER_JOINTS):
            try:
                idx = msg.name.index(name)
                self._positions[i] = float(msg.position[idx])
                if idx < len(msg.effort):
                    self._efforts[i] = float(msg.effort[idx])
            except (ValueError, IndexError):
                pass
        self._joint_pos_received = True

    def _on_manual_adjust(self, msg: Float64) -> None:
        delta = msg.data
        self._target_forces = [
            max(self._target_min, min(self._target_max, t + delta))
            for t in self._target_forces
        ]
        self.get_logger().info(
            f"Manual adjust: delta={delta:.1f}, targets={[round(t, 1) for t in self._target_forces]}"
        )

    # ── Controller lifecycle ───────────────────────────────────────────────

    def _activate_controller(self) -> None:
        self.get_logger().info("Activating velocity-based force controller...")
        self._controller_active = True
        self._force_stable = False
        self._slip_detected = False
        self._stable_since = None
        self._target_forces = [self._target_mid] * FINGER_COUNT

        if not self._joint_pos_received:
            self.get_logger().warn("No joint feedback yet; force controller may malfunction.")

        if not self._stream_switch.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Force stream switch service not available.")
        elif not self._streaming_active:
            req = SetBool.Request()
            req.data = True
            future = self._stream_switch.call_async(req)

            def on_response(fut: object) -> None:
                try:
                    result = fut.result()
                    if result.success:
                        self._streaming_active = True
                        self.get_logger().info("Force streaming activated.")
                    else:
                        self.get_logger().warn(f"Force streaming activation failed: {result.message}")
                except Exception as exc:
                    self.get_logger().error(f"Force streaming service error: {exc}")

            future.add_done_callback(on_response)

        self._publish_velocity([0.0] * FINGER_COUNT, force=True)
        self._publish_position(self._open_positions, force=True)
        time.sleep(self._relaxed_wait_s)

        try:
            _switch_controllers(["group_vel_ff_controller"], _competitors(["group_vel_ff_controller"]))
        except Exception as exc:
            self.get_logger().error(f"Failed to switch to velocity controller: {exc}")
            self._controller_active = False
            return

        self._ramp_idx = 0
        self._ramp_last_advance = time.monotonic()
        self._hand_phase = "CLOSING"
        self._phase_entry_time = time.monotonic()
        self.get_logger().info("Hand CLOSING: velocity ramp active")

    def _on_leave_active_states(self) -> None:
        self.get_logger().info("Leaving active force-control states; stopping velocity.")
        self._controller_active = False
        self._force_stable = False
        self._stable_since = None
        self._hand_phase = "IDLE"
        self._publish_velocity([0.0] * FINGER_COUNT, force=True)
        self._publish_position(self._open_positions, force=True)

    def _reset_controller(self) -> None:
        self.get_logger().info("RESET: releasing grasp, switching to position control.")
        self._controller_active = False
        self._force_stable = False
        self._slip_detected = False
        self._stable_since = None
        self._hand_phase = "IDLE"
        self._publish_velocity([0.0] * FINGER_COUNT, force=True)

        try:
            _switch_controllers(["group_pos_ff_controller"], _competitors(["group_pos_ff_controller"]))
        except Exception as exc:
            self.get_logger().error(f"Failed to switch to position controller: {exc}")

        self._publish_position(self._open_positions, force=True)

        self._normal_forces = [0.0, 0.0, 0.0]
        self._tangential_forces = [0.0, 0.0, 0.0]
        self._prev_tangential_forces = [0.0, 0.0, 0.0]
        self._force_data_received = False
        for buf in self._normal_bufs:
            buf.clear()
        for buf in self._tangential_bufs:
            buf.clear()

        if self._streaming_active and self._stream_switch.service_is_ready():
            req = SetBool.Request()
            req.data = False
            self._stream_switch.call_async(req)
            self._streaming_active = False
            self.get_logger().info("Force streaming deactivated.")

    # ── Control loop ───────────────────────────────────────────────────────

    def _control_tick(self) -> None:
        if self._pipeline_state not in (STATE_GRASPING, STATE_HOLDING, STATE_VOLITIONAL):
            self._publish_status()
            return
        if not self._controller_active:
            self._publish_status()
            return
        if not self._joint_pos_received:
            self._publish_status()
            return

        now = time.monotonic()
        if self._force_data_received and (now - self._last_force_time) > self._stale_timeout:
            self.get_logger().error("Force data stale; skipping commands.")
            self._publish_status()
            return
        if not self._force_data_received:
            self._publish_status()
            return

        # Emergency force release
        if max(self._normal_forces) >= self._max_force_emerg:
            self.get_logger().warn(
                f"EMERGENCY: Force exceeded {self._max_force_emerg}! "
                f"Forces: {self._normal_forces} — backing off"
            )
            self._publish_velocity([self._emergency_backoff_vel] * FINGER_COUNT, force=True)
            self._publish_status()
            return

        # Slip detection from tangential force rate-of-change (uses ForceData sensors)
        self._slip_detected = False
        for i in range(FINGER_COUNT):
            tang_rate = abs(
                self._tangential_forces[i] - self._prev_tangential_forces[i]
            ) / self._dt
            if tang_rate > self._slip_thresh:
                self._slip_detected = True
                self.get_logger().warn(
                    f"Slip detected on finger {i}: tangential rate = {tang_rate:.1f}"
                )
                break

        # Phase-specific control
        if self._hand_phase == "CLOSING":
            self._run_closing()
        elif self._hand_phase == "FORCE_HOLD":
            self._run_force_hold()

        # Stability check
        self._update_stability()

        self._publish_status()

    def _run_closing(self) -> None:
        contact = self._first_contact_reason()
        if contact:
            self.get_logger().info(f"Force hold entry: {contact}")
            self._hand_phase = "FORCE_HOLD"
            self._phase_entry_time = time.monotonic()
            self._publish_velocity([0.0] * FINGER_COUNT, force=True)
            return

        now = time.monotonic()
        if self._ramp_idx < len(self._ramp):
            if (now - self._ramp_last_advance) >= self._step_interval_s:
                vel = self._ramp[self._ramp_idx]
                self._ramp_idx += 1
                self._ramp_last_advance = now
                self._publish_velocity([vel] * FINGER_COUNT)
        else:
            self._publish_velocity([self._closing_end] * FINGER_COUNT)

    def _run_force_hold(self) -> None:
        velocities = [
            _hold_velocity(
                self._normal_forces[i],
                self._target_forces[i],
                self._hold_deadzone,
                self._hold_max_velocity,
                self._hold_min_overshoot,
                self._hold_max_overshoot,
            )
            for i in range(FINGER_COUNT)
        ]
        # Slip response: boost velocity toward target if slip detected
        if self._slip_detected:
            for i in range(FINGER_COUNT):
                if self._normal_forces[i] < self._target_forces[i]:
                    velocities[i] = max(velocities[i], self._hold_max_velocity)
        self._publish_velocity(velocities)

    def _first_contact_reason(self) -> Optional[str]:
        for i, name in enumerate(FINGER_JOINTS):
            if self._normal_forces[i] >= self._force_thresholds[i]:
                return f"{name} force {self._normal_forces[i]:.0f} >= {self._force_thresholds[i]:.0f}"
        for i, name in enumerate(FINGER_JOINTS):
            if self._positions[i] >= self._stop_positions[i]:
                return f"{name} position {self._positions[i]:.3f} >= {self._stop_positions[i]:.2f}"
        return None

    def _update_stability(self) -> None:
        if self._hand_phase != "FORCE_HOLD":
            return
        now = time.monotonic()
        force_errors = [
            abs(self._target_forces[i] - self._normal_forces[i]) for i in range(FINGER_COUNT)
        ]
        all_within = all(e <= self._stability_tol for e in force_errors)
        if all_within:
            if self._stable_since is None:
                self._stable_since = now
            elif (now - self._stable_since) >= self._stability_window:
                if not self._force_stable:
                    self.get_logger().info("Forces stabilized.")
                self._force_stable = True
        else:
            self._stable_since = None
            self._force_stable = False

    # ── Publishers ─────────────────────────────────────────────────────────

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
        for i in range(FINGER_COUNT):
            positions[i] = max(self._joint_pos_min[i], min(self._joint_pos_limits[i], positions[i]))
        msg = Float64MultiArray()
        msg.data = [float(p) for p in positions]
        self._pos_pub.publish(msg)
        self._last_pos_cmd = list(msg.data)

    def _publish_status(self) -> None:
        force_errors = [
            self._target_forces[i] - self._normal_forces[i] for i in range(FINGER_COUNT)
        ]
        status = ForceControllerStatus()
        status.active = self._controller_active
        status.current_normal_forces = self._normal_forces
        status.current_tangential_forces = self._tangential_forces
        status.force_errors = force_errors
        status.force_stable = self._force_stable
        status.slip_detected = self._slip_detected
        status.state = self._hand_phase if self._hand_phase != "IDLE" else STATE_NAMES.get(
            self._pipeline_state, "UNKNOWN"
        )
        self._status_pub.publish(status)

    def destroy_node(self) -> bool:
        try:
            self._publish_velocity([0.0] * FINGER_COUNT, force=True)
            _switch_controllers(["group_pos_ff_controller"], _competitors(["group_pos_ff_controller"]))
            self._publish_position(self._open_positions, force=True)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ForceControllerNode()
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
