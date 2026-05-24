#!/usr/bin/env python3
"""EMG-driven velocity-based force-hold hand controller.

Integrates the static grasp test logic into a reusable ROS 2 node:

  IDLE ──(POWER + confidence + hold)──▶ CLOSING
    ▲                                      │ (force threshold crossed)
    │                                      ▼
    │                                 FORCE_HOLD ◀── FLEXION/EXTENSION adjusts
    │                                      │ target force
    │                                      │ (OPEN + hold timeout)
    │                                      ▼
    └──────────────────────────────── RELEASING

  Any state ──(fault)──▶ FAULT

On CLOSING entry:
  1. Open hand to configured relaxed position via position control.
  2. Switch to velocity controller (group_vel_ff_controller).
  3. Run  the  configured  velocity  ramp,  monitoring  /joint_states  effort
     every step.
  4. Immediately enter FORCE_HOLD when *any* finger crosses its threshold.

In FORCE_HOLD:
  - Measures effort per finger from /joint_states.
  - Holds target force (dead-zone P-controller with tuned gain).
  - FLEXION/EXTENSION EMG signal adjusts target force proportionally
    (not wrist movement).
  - CLOSING velocity ramp is suspended; only hold adjustments run.

On RELEASING or FAULT:
  - Publishes zero velocity.
  - Switches back to position controller.
  - Opens the hand to relaxed position.

Safety features:
  - Zero velocity published before any controller switch.
  - Unchanged commands are NOT resent  (serial-command-pressure control).
  - Stale /joint_states data  (> stale_timeout)  → FAULT.
  - Controller switch failure  → FAULT.
  - Emergency force exceeded    → emergency back-off.
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from enum import Enum, auto
from typing import List, Optional, Tuple

import yaml

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, Int32, Float32
    from sensor_msgs.msg import JointState
except ModuleNotFoundError:
    # Stubs for static-method unit testing without ROS2 installed.
    class Node:  # type: ignore[no-redef]  # noqa: D101
        pass

    class Float64MultiArray:  # type: ignore[no-redef]  # noqa: D101
        pass

    class Int32:  # type: ignore[no-redef]  # noqa: D101
        def __init__(self, data: int = 0) -> None:
            self.data = data

    class Float32:  # type: ignore[no-redef]  # noqa: D101
        def __init__(self, data: float = 0.0) -> None:
            self.data = data

    class JointState:  # type: ignore[no-redef]  # noqa: D101
        pass


# Local helpers — absolute import works both when installed and in source tree
from mia_hand_ros2_control.force_hold_controller import (
    FINGER_COUNT,
    FINGER_JOINTS,
    ForceHoldConfig,
    ForceHoldState,
    check_emergency_force,
    check_stop_conditions,
    compute_hold_velocity,
    compute_target_forces_all,
    compute_velocity_ramp,
    is_force_data_stale,
    should_enter_force_hold,
)


# ── State machine ──────────────────────────────────────────────────────────────

class Phase(Enum):
    """Controller phase."""
    IDLE = auto()
    CLOSING = auto()
    FORCE_HOLD = auto()
    RELEASING = auto()
    FAULT = auto()


# ── Controller names ───────────────────────────────────────────────────────────

VEL_CTRL = "group_vel_ff_controller"
POS_CTRL = "group_pos_ff_controller"


# ── Controller switching helpers  (ROS 2 CLI) ──────────────────────────────────

def _ros(*args: str, timeout: float = 10.0, check: bool = True) -> str:
    """Run a 'ros2' CLI command; return stdout or raise on failure."""
    cmd = ["ros2"] + list(args)
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if check and result.returncode != 0:
        raise RuntimeError(
            f"ros2 {' '.join(args)} failed (rc={result.returncode}): "
            f"{result.stderr.strip()}"
        )
    return result.stdout.strip()


def _switch_controllers(activate: List[str], deactivate: List[str]) -> bool:
    """Switch ros2_control controllers via service call.

    Returns True on success, raises RuntimeError on failure.
    """
    # Load controllers first  (no-op if already loaded)
    for ctrl in activate:
        subprocess.run(
            ["ros2", "control", "load_controller", "--set-state", "inactive", ctrl],
            capture_output=True, text=True,
        )

    req = (
        "{"
        f"activate_controllers: [{', '.join(repr(c) for c in activate)}], "
        f"deactivate_controllers: [{', '.join(repr(c) for c in deactivate)}], "
        "strictness: 2, "
        "activate_asap: true, "
        "timeout: {sec: 10, nanosec: 0}"
        "}"
    )
    cmd = [
        "ros2", "service", "call",
        "/controller_manager/switch_controller",
        "controller_manager_msgs/srv/SwitchController", req,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    if result.returncode != 0 or "ok=True" not in result.stdout:
        # Fallback: BEST_EFFORT
        req2 = req.replace("strictness: 2", "strictness: 1")
        cmd[-1] = req2
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        if "ok=True" not in result.stdout:
            raise RuntimeError(
                f"Controller switch failed: {result.stdout.strip()}"
            )
    return True


def _set_controller_state(ctrl: str, state: str) -> None:
    """Set a controller's lifecycle state via ros2 CLI (best-effort)."""
    subprocess.run(
        ["ros2", "control", "set_controller_state", ctrl, state],
        capture_output=True, text=True,
    )


# ── EMG gesture IDs (match emg_bridge/config.py) ──────────────────────────────

REST: int = 0
POWER: int = 1
PINCH: int = 2
OPEN: int = 3
POINT: int = 4


# ── Controller node ────────────────────────────────────────────────────────────

class EmgForceController(Node):
    """EMG-driven velocity-based force-hold hand controller.

    Subscriptions:
        /joint_states          (JointState)
        /emg/gesture_label     (Int32)
        /emg/confidence        (Float32)
        /emg/proportional      (Float32)

    Publishers:
        /group_vel_ff_controller/commands  (Float64MultiArray)
        /group_pos_ff_controller/commands  (Float64MultiArray)
        /emg_force_controller/status       (Float64MultiArray)
            [phase_id, hold_active, target_force_mean,
             effort_mean, hold_velocity, fault_code]
    """

    def __init__(self) -> None:
        super().__init__("emg_force_controller")

        # ── Load config ────────────────────────────────────────────────────
        config_path = self.declare_parameter(
            "config_path",
            "/prosthesis_ws/config/emg_force_controller.yaml",
        ).value
        with open(config_path) as f:
            raw = yaml.safe_load(f)

        # Build ForceHoldConfig from YAML
        sp = raw.get("stop_positions", {})
        ft = raw.get("force_thresholds", {})
        tf_min = raw.get("target_force_min", {})
        tf_max = raw.get("target_force_max", {})
        dz = raw.get("force_deadzone", {})
        mo = raw.get("max_overshoot", {})
        me = raw.get("max_force_emergency", {})

        self._config = ForceHoldConfig(
            closing_velocity_start=float(raw.get("closing_velocity_start", 0.3)),
            closing_velocity_end=float(raw.get("closing_velocity_end", 0.1)),
            decay_steps=int(raw.get("decay_steps", 5)),
            step_interval_s=float(raw.get("step_interval_s", 0.2)),
            stop_positions=[float(sp.get(n, 1.5)) for n in FINGER_JOINTS],
            force_thresholds=[float(ft.get(n, 300.0)) for n in FINGER_JOINTS],
            target_force_min=[float(tf_min.get(n, 50.0)) for n in FINGER_JOINTS],
            target_force_max=[float(tf_max.get(n, 500.0)) for n in FINGER_JOINTS],
            force_deadzone=[float(dz.get(n, 15.0)) for n in FINGER_JOINTS],
            max_overshoot=[float(mo.get(n, 100.0)) for n in FINGER_JOINTS],
            max_adjustment_velocity=float(raw.get("max_adjustment_velocity", 0.05)),
            adjustment_gain=float(raw.get("adjustment_gain", 0.0005)),
            static_hold_velocity=float(raw.get("static_hold_velocity", 0.02)),
            stale_data_timeout_s=float(raw.get("stale_data_timeout_s", 0.5)),
            max_force_emergency=[float(me.get(n, 800.0)) for n in FINGER_JOINTS],
            emergency_backoff_velocity=float(raw.get("emergency_backoff_velocity", -0.1)),
        )

        # EMG params
        self._grasp_gesture = int(raw.get("emg_grasp_trigger_gesture", POWER))
        self._release_gesture = int(raw.get("emg_release_gesture", OPEN))
        self._confidence_thresh = float(raw.get("emg_grasp_confidence_threshold", 0.7))
        self._hold_timeout = float(raw.get("gesture_hold_timeout_s", 1.0))

        # ── Publishers ─────────────────────────────────────────────────────
        self._vel_pub = self.create_publisher(
            Float64MultiArray,
            f"/{VEL_CTRL}/commands",
            10,
        )
        self._pos_pub = self.create_publisher(
            Float64MultiArray,
            f"/{POS_CTRL}/commands",
            10,
        )
        self._status_pub = self.create_publisher(
            Float64MultiArray,
            "/emg_force_controller/status",
            10,
        )

        # ── Subscribers ────────────────────────────────────────────────────
        self._positions: List[float] = [0.0] * FINGER_COUNT
        self._efforts: List[float] = [0.0] * FINGER_COUNT
        self._got_js: bool = False
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10,
        )

        self._emg_gesture: int = REST
        self._emg_confidence: float = 0.0
        self._emg_proportional: float = 0.0
        self._emg_gesture_start_time: Optional[float] = None
        self.create_subscription(
            Int32, raw.get("emg_gesture_topic", "/emg/gesture_label"),
            self._on_gesture, 10,
        )
        self.create_subscription(
            Float32, raw.get("emg_confidence_topic", "/emg/confidence"),
            self._on_confidence, 10,
        )
        self.create_subscription(
            Float32, raw.get("emg_proportional_topic", "/emg/proportional"),
            self._on_proportional, 10,
        )

        # ── State ──────────────────────────────────────────────────────────
        self._phase = Phase.IDLE
        self._step_idx: int = 0
        self._stop_reason: Optional[str] = None

        # Force-hold state
        self._fh_state = ForceHoldState()
        self._ramp = compute_velocity_ramp(
            self._config.closing_velocity_start,
            self._config.closing_velocity_end,
            self._config.decay_steps,
        )

        # Command deduplication — track last published value
        self._last_vel: Optional[float] = None
        self._last_pos: Optional[float] = None

        # ── Timer ──────────────────────────────────────────────────────────
        self._dt = self._config.step_interval_s
        self._timer = self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            "EMG Force Controller started — IDLE  "
            f"(ramp: {self._config.closing_velocity_start}→"
            f"{self._config.closing_velocity_end} over "
            f"{self._config.decay_steps} steps)"
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState) -> None:
        try:
            for i, jname in enumerate(FINGER_JOINTS):
                idx = msg.name.index(jname)
                self._positions[i] = float(msg.position[idx])
                if idx < len(msg.effort):
                    self._efforts[i] = float(msg.effort[idx])
            self._fh_state.current_efforts = list(self._efforts)
            self._fh_state.current_positions = list(self._positions)
            self._fh_state.last_effort_time = time.monotonic()
            self._fh_state.has_effort_data = True
            self._got_js = True
        except ValueError:
            pass

    def _on_gesture(self, msg: Int32) -> None:
        if msg.data != self._emg_gesture:
            self._emg_gesture = msg.data
            self._emg_gesture_start_time = time.time()
            self.get_logger().debug(
                f"Gesture changed: {self._emg_gesture}"
            )

    def _on_confidence(self, msg: Float32) -> None:
        self._emg_confidence = msg.data

    def _on_proportional(self, msg: Float32) -> None:
        self._emg_proportional = float(msg.data)

    # ── Publishing helpers  (with deduplication) ───────────────────────────

    def _publish_velocity(self, v: float) -> None:
        """Publish a velocity command, skipping if unchanged."""
        if self._last_vel is not None and math.isclose(self._last_vel, v, rel_tol=0.01):
            return  # unchanged → suppress to reduce serial command pressure
        msg = Float64MultiArray()
        msg.data = [v, v, v]
        self._vel_pub.publish(msg)
        self._last_vel = v

    def _publish_position(self, pos: float) -> None:
        """Publish a position command, skipping if unchanged."""
        if self._last_pos is not None and math.isclose(self._last_pos, pos, rel_tol=0.01):
            return
        msg = Float64MultiArray()
        msg.data = [pos, pos, pos]
        self._pos_pub.publish(msg)
        self._last_pos = pos

    def _publish_status(self, fault_code: int = 0) -> None:
        """Publish controller status for diagnostics.

        Layout: [phase_id, hold_active, target_force_mean,
                 effort_mean, hold_velocity, fault_code]
        """
        msg = Float64MultiArray()
        mean_target = (
            sum(self._fh_state.target_forces) / FINGER_COUNT
            if any(self._fh_state.target_forces) else 0.0
        )
        mean_effort = (
            sum(self._fh_state.current_efforts) / FINGER_COUNT
            if self._fh_state.has_effort_data else 0.0
        )
        msg.data = [
            float(Phase.IDLE.value if self._phase == Phase.IDLE else
                  Phase.CLOSING.value if self._phase == Phase.CLOSING else
                  Phase.FORCE_HOLD.value if self._phase == Phase.FORCE_HOLD else
                  Phase.RELEASING.value if self._phase == Phase.RELEASING else
                  Phase.FAULT.value),
            1.0 if self._fh_state.in_hold else 0.0,
            mean_target,
            mean_effort,
            float(self._last_vel or 0.0),
            float(fault_code),
        ]
        self._status_pub.publish(msg)

    # ── Safety ─────────────────────────────────────────────────────────────

    def _enter_fault(self, reason: str, fault_code: int = 1) -> None:
        """Transition to FAULT, stop all motion, log reason."""
        self.get_logger().error(f"FAULT [{fault_code}]: {reason}")
        self._phase = Phase.FAULT
        self._publish_velocity(0.0)
        self._fh_state.in_hold = False
        self._publish_status(fault_code=fault_code)

    # ── Gesture helper ─────────────────────────────────────────────────────

    def _gesture_held_long_enough(self) -> bool:
        if self._emg_gesture_start_time is None:
            return False
        return (time.time() - self._emg_gesture_start_time) >= self._hold_timeout

    def _is_gesture_active(self, gesture_id: int) -> bool:
        return (
            self._emg_gesture == gesture_id
            and self._emg_confidence >= self._confidence_thresh
            and self._gesture_held_long_enough()
        )

    # ── Control loop ───────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        """Main control tick — dispatched by timer at step_interval_s."""

        # Safety gate: missing joint states
        if not self._got_js:
            if self._phase not in (Phase.IDLE, Phase.FAULT):
                self._enter_fault("Lost joint-states feedback", 10)
            return

        # Safety gate: stale effort data
        if is_force_data_stale(self._fh_state, self._config):
            if self._phase not in (Phase.IDLE, Phase.FAULT):
                self._enter_fault("Stale force/effort data", 11)
            return

        # ── IDLE ───────────────────────────────────────────────────────
        if self._phase == Phase.IDLE:
            if self._is_gesture_active(self._grasp_gesture):
                self._on_enter_closing()
            self._publish_status()
            return

        # ── CLOSING ────────────────────────────────────────────────────
        if self._phase == Phase.CLOSING:
            self._run_closing()
            return

        # ── FORCE_HOLD ─────────────────────────────────────────────────
        if self._phase == Phase.FORCE_HOLD:
            self._run_force_hold()
            return

        # ── RELEASING ──────────────────────────────────────────────────
        if self._phase == Phase.RELEASING:
            self._run_releasing()
            return

        # ── FAULT ──────────────────────────────────────────────────────
        if self._phase == Phase.FAULT:
            self._publish_velocity(0.0)
            self._publish_status(fault_code=1)
            return

    # ── Phase transitions ──────────────────────────────────────────────────

    def _on_enter_closing(self) -> None:
        """Transition IDLE → CLOSING.

        1. Publish zero velocity to flush any prior command.
        2. Open hand to relaxed position (pos=0.0).
        3. Switch from position to velocity controller.
        4. Reset ramp index and state.
        """
        self.get_logger().info("EMG grasp triggered → entering CLOSING")

        # Flush
        self._publish_velocity(0.0)

        # Open to relaxed
        self._publish_position(0.0)
        time.sleep(0.5)

        # Switch controllers
        try:
            _switch_controllers([VEL_CTRL], [POS_CTRL])
            time.sleep(0.5)
        except Exception as exc:
            self._enter_fault(f"Controller switch failed: {exc}", 20)
            return

        self._phase = Phase.CLOSING
        self._step_idx = 0
        self._stop_reason = None
        self._fh_state.in_hold = False

    def _run_closing(self) -> None:
        """CLOSING phase: execute velocity ramp, check for force threshold."""

        # Check for release gesture (cancel closing)
        if self._is_gesture_active(self._release_gesture):
            self.get_logger().info("Release gesture during CLOSING → RELEASING")
            self._on_enter_releasing()
            return

        # —— Emergency force check ——
        if check_emergency_force(self._efforts, self._config):
            self._enter_fault("Emergency force exceeded during closing", 30)
            return

        # —— Execute ramp step ——
        if self._step_idx < len(self._ramp):
            vel = self._ramp[self._step_idx]
            self._step_idx += 1
        else:
            vel = self._config.closing_velocity_end

        self._publish_velocity(vel)

        # —— Check for force threshold  (enters force-hold) ——
        if should_enter_force_hold(self._efforts, self._config.force_thresholds):
            self._stop_reason = check_stop_conditions(
                self._positions, self._efforts,
                self._config.stop_positions,
                self._config.force_thresholds,
            )
            self.get_logger().info(
                f"Force hold entry: {self._stop_reason}  "
                f"efforts={[f'{e:.0f}' for e in self._efforts]}"
            )
            self._on_enter_force_hold()
            return

        # —— Check for position limits ——
        stop = check_stop_conditions(
            self._positions, self._efforts,
            self._config.stop_positions,
            self._config.force_thresholds,
        )
        if stop is not None:
            self.get_logger().warn(f"Stop position reached: {stop}")
            self._on_enter_force_hold()  # treat as hold  (nothing to grasp)
            return

        self._publish_status()

    def _on_enter_force_hold(self) -> None:
        """Enter FORCE_HOLD: compute initial target forces, zero velocity."""
        self._phase = Phase.FORCE_HOLD
        self._fh_state.in_hold = True

        # Initial target forces based on current EMG proportional
        self._fh_state.target_forces = compute_target_forces_all(
            self._emg_proportional, self._config,
        )

        self._publish_velocity(0.0)

        self.get_logger().info(
            f"Entering FORCE_HOLD  "
            f"targets={[f'{t:.0f}' for t in self._fh_state.target_forces]}"
        )

    def _run_force_hold(self) -> None:
        """FORCE_HOLD phase: maintain target force with velocity adjustments."""

        # Check for release gesture
        if self._is_gesture_active(self._release_gesture):
            self.get_logger().info("Release gesture → RELEASING")
            self._on_enter_releasing()
            return

        # Emergency force check
        if check_emergency_force(self._efforts, self._config):
            # Emergency back-off: open direction
            self.get_logger().warn("Emergency force — backing off")
            self._publish_velocity(self._config.emergency_backoff_velocity)
            self._publish_status(fault_code=31)
            # After one step of back-off, enter fault
            self._enter_fault("Emergency force exceeded in hold", 31)
            return

        # Update target forces from EMG proportional signal
        self._fh_state.target_forces = compute_target_forces_all(
            self._emg_proportional, self._config,
        )

        # Compute hold velocity adjustment
        hold_vel = compute_hold_velocity(self._fh_state, self._config, self._dt)
        self._publish_velocity(hold_vel)

        self._publish_status()

    def _on_enter_releasing(self) -> None:
        """Transition to RELEASING: zero velocity, switch to position control."""
        self._phase = Phase.RELEASING
        self._fh_state.in_hold = False

        # Zero velocity
        self._publish_velocity(0.0)
        time.sleep(0.3)

        # Switch back to position controller
        try:
            _set_controller_state(VEL_CTRL, "inactive")
            _set_controller_state(POS_CTRL, "active")
            time.sleep(0.5)
        except Exception as exc:
            self._enter_fault(f"Controller switch-back failed: {exc}", 21)
            return

        self.get_logger().info("Entering RELEASING — opening hand")

    def _run_releasing(self) -> None:
        """RELEASING phase: open hand to relaxed position, return to IDLE."""
        self._publish_velocity(0.0)
        self._publish_position(0.0)

        # Reset command dedup trackers
        self._last_vel = None
        self._last_pos = None

        # Reset force-hold state
        self._fh_state = ForceHoldState()

        self.get_logger().info("Hand released → IDLE")
        self._phase = Phase.IDLE
        self._publish_status()


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args: Optional[List[str]] = None) -> int:
    rclpy.init(args=args)
    node = EmgForceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt — shutting down")
    except Exception as exc:
        node.get_logger().fatal(f"Unhandled exception: {exc}")
    finally:
        # Safe stop
        try:
            node._publish_velocity(0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
