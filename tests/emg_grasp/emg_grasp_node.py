#!/usr/bin/env python3
"""EMG-driven mode-based velocity force grasp test node.

Subscribes to EMG gestures and /joint_states, publishes velocity commands to
/group_vel_ff_controller/commands, position commands to
/group_pos_ff_controller/commands, and wrist position commands to
/wrist/set_position.

Mode state machine:
  moving    → Hand open/relaxed. Wrist can be positioned. POWER starts grasping.
  grasping  → POWER closes, EXTENSION opens slowly, REST holds finger position.
              FLEXION switches to holding; held OPEN releases to moving.
  holding   → Fingers hold position with velocity feedback while FLEXION/
              EXTENSION rotate the wrist. POWER returns to grasping.
"""

import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple

import yaml

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32, Float64MultiArray, Int32, String

    ROS_AVAILABLE = True
except ModuleNotFoundError:
    ROS_AVAILABLE = False

    # Stubs allow unit tests without ROS2 installed
    class Node:  # type: ignore[misc]
        def __init__(self, name):
            self._name = name

        def declare_parameter(self, name, default):
            class _Param:
                def __init__(self, value):
                    self.value = value

            return _Param(default)

        def create_publisher(self, *args, **kwargs):
            class _Pub:
                def publish(self, msg):
                    pass

            return _Pub()

        def create_subscription(self, *args, **kwargs):
            pass

        def create_timer(self, *args, **kwargs):
            pass

        def get_logger(self):
            class _Logger:
                def info(self, *args, **kwargs):
                    pass

                def warn(self, *args, **kwargs):
                    pass

                def error(self, *args, **kwargs):
                    pass

            return _Logger()

        def destroy_node(self):
            pass

    class Float64MultiArray:  # type: ignore[no-redef]
        def __init__(self):
            self.data = []

    class Int32:  # type: ignore[no-redef]
        def __init__(self, data=0):
            self.data = data

    class Float32:  # type: ignore[no-redef]
        def __init__(self, data=0.0):
            self.data = data

    class String:  # type: ignore[no-redef]
        def __init__(self, data=""):
            self.data = data

    class JointState:  # type: ignore[no-redef]
        def __init__(self):
            self.name = []
            self.position = []
            self.effort = []


FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_COUNT = 3

# Mode strings
MODE_MOVING = "moving"
MODE_GRASPING = "grasping"
MODE_HOLDING = "holding"

# Pipeline states (must match force_controller_node and pipeline_manager)
STATE_IDLE = 0
STATE_APPROACHING = 3
STATE_GRASPING = 4
STATE_HOLDING = 5
STATE_RELEASING = 6


class EmgGraspNode(Node):
    def __init__(self):
        super().__init__("emg_grasp_test")

        # ── Load config ───────────────────────────────────────────────────────
        config_path = self.declare_parameter(
            "config_path", "/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml"
        ).value
        with open(config_path) as f:
            self._cfg = yaml.safe_load(f)

        # ── Parameters from config ────────────────────────────────────────────
        self._v_start = float(self._cfg["closing_velocity_start"])
        self._v_end = float(self._cfg["closing_velocity_end"])
        self._decay_steps = int(self._cfg["decay_steps"])
        self._step_interval = float(self._cfg["step_interval_s"])
        self._relaxed_wait = float(self._cfg["relaxed_wait_s"])
        self._grasp_close_velocity = float(
            self._cfg.get("grasp_close_velocity", 0.1)
        )
        self._grasp_open_velocity = float(
            self._cfg.get("grasp_open_velocity", -0.1)
        )
        self._hold_position_kp = float(self._cfg.get("hold_position_kp", 1.2))
        self._hold_velocity_limit = abs(
            float(self._cfg.get("hold_velocity_limit", 0.08))
        )
        self._hold_position_deadband = abs(
            float(self._cfg.get("hold_position_deadband", 0.01))
        )

        sp = self._cfg["stop_positions"]
        ft = self._cfg["force_thresholds"]
        self._stop_positions = [float(sp[n]) for n in FINGER_JOINTS]
        self._force_thresholds = [float(ft[n]) for n in FINGER_JOINTS]

        self._ramp = self._compute_velocity_ramp(
            self._v_start, self._v_end, self._decay_steps
        )

        # Mode configuration
        self._modes = self._cfg["modes"]
        if "grasp_close" in self._modes.get(MODE_GRASPING, {}):
            self._modes[MODE_GRASPING]["grasp_close"]["velocity"] = (
                self._grasp_close_velocity
            )
        if "grasp_open" in self._modes.get(MODE_GRASPING, {}):
            self._modes[MODE_GRASPING]["grasp_open"]["velocity"] = (
                self._grasp_open_velocity
            )
        self._mode = MODE_MOVING

        # Wrist control
        self._wrist_control_enabled = bool(
            self._cfg.get("wrist_control_enabled", False)
        )
        self._wrist_accel = float(self._cfg.get("wrist_accel_deg_s2", 180.0))
        self._wrist_position = 0.0

        # ros2_control command mode arbitration. The Mia hardware interface can
        # only accept one command interface per joint at a time, so keep the
        # position controller active while moving and switch to velocity for
        # grasping/force control.
        self._position_controller = self._cfg.get(
            "position_controller", "group_pos_ff_controller"
        )
        self._velocity_controller = self._cfg.get(
            "velocity_controller", "group_vel_ff_controller"
        )
        self._controller_switch_enabled = bool(
            self._cfg.get("controller_switch_enabled", True)
        ) and ROS_AVAILABLE
        self._controller_switch_timeout_s = float(
            self._cfg.get("controller_switch_timeout_s", 10.0)
        )
        self._controller_mode = "position"
        self._emergency_stop_on_shutdown = bool(
            self._cfg.get("emergency_stop_on_shutdown", True)
        ) and ROS_AVAILABLE
        self._emergency_stop_service = self._cfg.get(
            "emergency_stop_service", "/mia_hand/emergency_stop"
        )

        # Force controller integration
        self._use_force_controller = bool(
            self._cfg.get("use_force_controller", False)
        )
        self._per_finger_position_controllers = list(
            self._cfg.get("per_finger_position_controllers", [])
        )
        self._pipeline_state = STATE_IDLE

        # ── Publishers ────────────────────────────────────────────────────────
        self._vel_pub = self.create_publisher(
            Float64MultiArray, "/group_vel_ff_controller/commands", 10
        )
        self._pos_pub = self.create_publisher(
            Float64MultiArray, "/group_pos_ff_controller/commands", 10
        )
        if self._wrist_control_enabled:
            self._wrist_pub = self.create_publisher(
                Float64MultiArray, self._cfg["wrist_cmd_topic"], 10
            )
        self._mode_pub = self.create_publisher(String, "/emg_grasp/mode", 10)
        if self._use_force_controller:
            self._pipeline_state_pub = self.create_publisher(
                Int32, "/pipeline/state", 10
            )

        # ── Subscribers ───────────────────────────────────────────────────────
        self._positions = [0.0] * FINGER_COUNT
        self._efforts = [0.0] * FINGER_COUNT
        self._got_js = False
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        self._emg_gesture = 0
        self._emg_confidence = 0.0
        self._emg_gesture_start_time = None
        self.create_subscription(
            Int32, self._cfg["emg_gesture_topic"], self._on_gesture, 10
        )
        self.create_subscription(
            Float32, self._cfg["emg_confidence_topic"], self._on_confidence, 10
        )

        # ── State ─────────────────────────────────────────────────────────────
        self._step_idx = 0
        self._stop_reason = None
        self._hold_positions = [0.0] * FINGER_COUNT
        self._has_hold_positions = False

        # ── Timer ─────────────────────────────────────────────────────────────
        self._timer = self.create_timer(self._step_interval, self._control_loop)

        self.get_logger().info("EMG Grasp Test node started — waiting in moving mode")
        self._publish_mode()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState):
        try:
            for i, jname in enumerate(FINGER_JOINTS):
                idx = msg.name.index(jname)
                self._positions[i] = float(msg.position[idx])
                if idx < len(msg.effort):
                    self._efforts[i] = float(msg.effort[idx])
            self._got_js = True
        except ValueError:
            pass

    def _on_gesture(self, msg: Int32):
        if msg.data != self._emg_gesture:
            self._emg_gesture = msg.data
            self._emg_gesture_start_time = time.time()
            self.get_logger().info(
                f"EMG gesture changed: {self._emg_gesture} "
                f"(confidence={self._emg_confidence:.2f}, mode={self._mode})"
            )

    def _on_confidence(self, msg: Float32):
        self._emg_confidence = msg.data

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _compute_velocity_ramp(
        v_start: float, v_end: float, decay_steps: int
    ) -> List[float]:
        if decay_steps <= 1:
            return [v_start]
        ramp = []
        for i in range(decay_steps):
            t = i / (decay_steps - 1)
            ramp.append(v_start + t * (v_end - v_start))
        return ramp

    def _publish_velocity(self, v: float):
        self._publish_velocity_vector([v, v, v])

    def _publish_velocity_vector(self, velocities: List[float]):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in velocities]
        self._vel_pub.publish(msg)

    def _publish_position(self, pos: float):
        msg = Float64MultiArray()
        msg.data = [pos, pos, pos]
        self._pos_pub.publish(msg)

    @staticmethod
    def _switch_ok(output: str) -> bool:
        return (
            "ok=True" in output
            or "ok: true" in output
            or "ok: True" in output
        )

    @staticmethod
    def _controller_list(names: List[str]) -> str:
        return ", ".join(repr(name) for name in names)

    def _call_switch_controller(
        self, activate: List[str], deactivate: List[str]
    ) -> bool:
        timeout_sec = max(1, int(self._controller_switch_timeout_s))
        request = (
            "{"
            f"activate_controllers: [{self._controller_list(activate)}], "
            f"deactivate_controllers: [{self._controller_list(deactivate)}], "
            "strictness: 2, "
            "activate_asap: true, "
            f"timeout: {{sec: {timeout_sec}, nanosec: 0}}"
            "}"
        )
        cmd = [
            "ros2",
            "service",
            "call",
            "/controller_manager/switch_controller",
            "controller_manager_msgs/srv/SwitchController",
            request,
        ]
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout_sec + 5,
            )
        except (subprocess.SubprocessError, OSError) as exc:
            self.get_logger().error(f"Controller switch command failed: {exc}")
            return False

        if result.returncode == 0 and self._switch_ok(result.stdout):
            return True

        self.get_logger().error(
            "Controller switch rejected: "
            f"activate={activate} deactivate={deactivate} "
            f"stdout={result.stdout.strip()} stderr={result.stderr.strip()}"
        )
        return False

    def _call_emergency_stop(self) -> bool:
        if not self._emergency_stop_on_shutdown:
            return False

        cmd = [
            "ros2",
            "service",
            "call",
            self._emergency_stop_service,
            "std_srvs/srv/Trigger",
            "{}",
        ]
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=3,
            )
        except (subprocess.SubprocessError, OSError) as exc:
            self.get_logger().error(f"Emergency stop command failed: {exc}")
            return False

        if result.returncode == 0 and (
            "success=True" in result.stdout
            or "success: true" in result.stdout
            or "success: True" in result.stdout
        ):
            self.get_logger().info("Emergency stop triggered on shutdown")
            return True

        self.get_logger().error(
            "Emergency stop rejected: "
            f"stdout={result.stdout.strip()} stderr={result.stderr.strip()}"
        )
        return False

    def _switch_controller_mode(self, mode: str) -> bool:
        if mode == self._controller_mode:
            return True

        if mode == "velocity":
            activate = [self._velocity_controller]
            deactivate = [self._position_controller]
        elif mode == "position":
            activate = [self._position_controller]
            deactivate = [self._velocity_controller]
        else:
            raise ValueError(f"Unknown controller mode: {mode}")

        if self._controller_switch_enabled:
            self.get_logger().info(
                f"Switching hand command mode: {self._controller_mode} -> {mode}"
            )
            if not self._call_switch_controller(activate, deactivate):
                return False

        self._controller_mode = mode
        return True

    def _switch_to_velocity(self) -> bool:
        return self._switch_controller_mode("velocity")

    def _switch_to_position(self) -> bool:
        return self._switch_controller_mode("position")

    def _publish_mode(self):
        self._mode_pub.publish(String(data=self._mode))

    def _publish_pipeline_state(self, state: int):
        if self._use_force_controller and hasattr(self, "_pipeline_state_pub"):
            self._pipeline_state = state
            self._pipeline_state_pub.publish(Int32(data=state))
            self.get_logger().info(f"Pipeline state -> {state}")

    def _switch_to_per_finger_position(self) -> bool:
        """Switch from group controller to per-finger position controllers."""
        if not self._per_finger_position_controllers:
            self.get_logger().error(
                "Cannot switch to per-finger: no controllers configured"
            )
            return False
        if self._controller_mode == "per_finger_position":
            return True
        deactivate = [self._position_controller]
        activate = list(self._per_finger_position_controllers)
        self.get_logger().info(
            f"Switching to per-finger position: {activate}"
        )
        if self._controller_switch_enabled:
            if not self._call_switch_controller(activate, deactivate):
                return False
        self._controller_mode = "per_finger_position"
        return True

    def _switch_to_group_position(self) -> bool:
        """Switch from per-finger controllers back to group position."""
        if self._controller_mode == "position":
            return True
        if self._controller_mode == "per_finger_position":
            deactivate = list(self._per_finger_position_controllers)
            activate = [self._position_controller]
            self.get_logger().info(
                f"Switching to group position: {activate}"
            )
            if self._controller_switch_enabled:
                if not self._call_switch_controller(activate, deactivate):
                    return False
            self._controller_mode = "position"
            return True
        return self._switch_to_position()

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _capture_hold_positions(self):
        self._hold_positions = list(self._positions)
        self._has_hold_positions = True

    def _hold_grasp_position(self):
        if not self._switch_to_velocity():
            return
        if not self._has_hold_positions:
            self._capture_hold_positions()

        velocities = []
        for target, current in zip(self._hold_positions, self._positions):
            err = target - current
            if abs(err) <= self._hold_position_deadband:
                velocities.append(0.0)
            else:
                velocities.append(
                    self._clamp(
                        self._hold_position_kp * err, self._hold_velocity_limit
                    )
                )
        self._publish_velocity_vector(velocities)

    def _apply_grasp_velocity(self, velocity: float):
        if not self._switch_to_velocity():
            return
        self._has_hold_positions = False
        self._publish_velocity(velocity)

    @staticmethod
    def check_stop_conditions(
        positions, efforts, stop_positions, force_thresholds
    ) -> Optional[str]:
        for i, name in enumerate(FINGER_JOINTS):
            if efforts[i] >= force_thresholds[i]:
                return f"FORCE CONTACT on {name} ({efforts[i]:.0f} >= {force_thresholds[i]})"
        for i, name in enumerate(FINGER_JOINTS):
            if positions[i] >= stop_positions[i]:
                return f"STOP POSITION reached on {name} ({positions[i]:.3f} >= {stop_positions[i]:.2f})"
        return None

    def _check_stop_conditions(self) -> Optional[str]:
        return self.check_stop_conditions(
            self._positions, self._efforts, self._stop_positions, self._force_thresholds
        )

    def _gesture_held_long_enough(self, hold_time_s: float) -> bool:
        if self._emg_gesture_start_time is None:
            return False
        return (time.time() - self._emg_gesture_start_time) >= hold_time_s

    def _dispatch_gesture(
        self,
        gesture_id: Optional[int] = None,
        confidence: Optional[float] = None,
        mode: Optional[str] = None,
    ) -> Tuple[Optional[str], Optional[Dict]]:
        """Look up what function (if any) the gesture maps to in the given mode.

        Uses instance state when arguments are not provided.
        Returns (func_name, func_cfg) or (None, None).
        """
        g = gesture_id if gesture_id is not None else self._emg_gesture
        c = confidence if confidence is not None else self._emg_confidence
        m = mode if mode is not None else self._mode

        mode_cfg = self._modes.get(m, {})
        for func_name, func_cfg in mode_cfg.items():
            if func_cfg is None:
                continue
            cfg_gesture_id = func_cfg.get("gesture_id")
            if cfg_gesture_id is None:
                continue
            if g == int(cfg_gesture_id):
                conf_thresh = func_cfg.get("confidence_threshold")
                if conf_thresh is None:
                    continue
                if c >= float(conf_thresh):
                    return func_name, func_cfg
        return None, None

    def _execute_continuous(self, func_name: str, func_cfg: Dict):
        """Fire-while-held continuous actions."""
        if func_name in ("wrist_pos", "wrist_neg"):
            if not self._wrist_control_enabled:
                return
            dt = self._step_interval
            vel = float(func_cfg["velocity"])
            self._wrist_position += vel * dt
            msg = Float64MultiArray()
            msg.data = [self._wrist_position, self._wrist_accel]
            self._wrist_pub.publish(msg)

    def _stop_continuous_action(self):
        """Stop any active continuous motion."""
        if not self._controller_switch_enabled or self._controller_mode == "velocity":
            self._publish_velocity(0.0)

    def _transition_to_grasping(self):
        """Enter grasp-adjustment mode."""
        self._step_idx = 0
        self._stop_reason = None

        if self._use_force_controller:
            self._publish_pipeline_state(STATE_APPROACHING)
            if not self._switch_to_per_finger_position():
                self.get_logger().error(
                    "Cannot enter grasping: per-finger controller switch failed"
                )
                return False
            self._publish_pipeline_state(STATE_GRASPING)
        else:
            if not self._switch_to_velocity():
                self.get_logger().error(
                    "Cannot enter grasping: velocity controller switch failed"
                )
                return False
            self._capture_hold_positions()

        self.get_logger().info("Entering grasping mode")
        self._mode = MODE_GRASPING
        self._publish_mode()
        return True

    def _transition_to_holding(self):
        """Enter wrist-control mode while holding the current finger positions."""
        if self._use_force_controller:
            self._publish_pipeline_state(STATE_HOLDING)
        else:
            if not self._switch_to_velocity():
                self.get_logger().error(
                    "Cannot enter holding: velocity controller switch failed"
                )
                return False
            self._capture_hold_positions()

        self.get_logger().info("Entering holding mode")
        self._mode = MODE_HOLDING
        self._publish_mode()
        return True

    def _transition_to_moving(self):
        """One-shot: open hand and return to moving mode."""
        self.get_logger().info("EMG release — entering moving")

        if self._use_force_controller:
            self._publish_pipeline_state(STATE_RELEASING)
            if not self._switch_to_group_position():
                self.get_logger().error(
                    "Cannot enter moving: group position switch failed"
                )
                return False
            self._publish_position(0.0)
            self._publish_pipeline_state(STATE_IDLE)
        else:
            self._publish_velocity(0.0)
            time.sleep(0.2)
            if not self._switch_to_position():
                self.get_logger().error(
                    "Cannot enter moving: position controller switch failed"
                )
                return False
            self._publish_position(0.0)

        self._mode = MODE_MOVING
        self._has_hold_positions = False
        self._publish_mode()
        return True

    def safe_shutdown(self):
        """Best-effort final safety actions before this node exits."""
        try:
            self._publish_velocity(0.0)
            time.sleep(0.05)
            self._publish_velocity(0.0)
        except Exception as exc:
            self.get_logger().error(f"Failed to publish shutdown zero velocity: {exc}")

        if self._use_force_controller:
            try:
                self._switch_to_group_position()
                self._publish_position(0.0)
            except Exception as exc:
                self.get_logger().error(f"Failed to publish shutdown group position: {exc}")
            return

        if self._emergency_stop_on_shutdown:
            # Keep this last: after shutdown, the hand should require an
            # explicit /mia_hand/play before any further movement.
            self._call_emergency_stop()
        else:
            try:
                self._switch_to_position()
                self._publish_position(0.0)
            except Exception as exc:
                self.get_logger().error(f"Failed to publish shutdown reset: {exc}")

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        self._publish_mode()
        if not self._got_js:
            self.get_logger().warn("No joint states yet — skipping cycle")
            return

        func_name, func_cfg = self._dispatch_gesture()

        # ---- MOVING: wrist can move, POWER starts grasp adjustment ----
        if self._mode == MODE_MOVING:
            if func_name in ("wrist_pos", "wrist_neg"):
                self._execute_continuous(func_name, func_cfg)
            elif func_name == "grasp_activate":
                self._transition_to_grasping()
            return

        # ---- GRASPING: fingers move or hold; wrist does not move ----
        if self._mode == MODE_GRASPING:
            if self._use_force_controller:
                # Force controller handles finger regulation via per-finger pos
                if func_name == "grasp_release":
                    if self._gesture_held_long_enough(float(func_cfg["hold_time_s"])):
                        self._transition_to_moving()
                elif func_name == "grasp_open":
                    self._transition_to_moving()
                elif func_name == "switch_to_holding":
                    self._transition_to_holding()
            else:
                if func_name == "grasp_release":
                    if self._gesture_held_long_enough(float(func_cfg["hold_time_s"])):
                        self._transition_to_moving()
                    else:
                        self._hold_grasp_position()
                elif func_name == "switch_to_holding":
                    self._transition_to_holding()
                elif func_name == "grasp_close":
                    self._apply_grasp_velocity(float(func_cfg["velocity"]))
                elif func_name == "grasp_open":
                    self._apply_grasp_velocity(float(func_cfg["velocity"]))
                else:
                    self._hold_grasp_position()
            return

        # ---- HOLDING: fingers hold; wrist moves; POWER returns to grasping ----
        if self._mode == MODE_HOLDING:
            if self._use_force_controller:
                if func_name == "grasp_release":
                    if self._gesture_held_long_enough(float(func_cfg["hold_time_s"])):
                        self._transition_to_moving()
                        return
                elif func_name == "switch_to_grasping":
                    self._transition_to_grasping()
                    return
                if func_name in ("wrist_pos", "wrist_neg"):
                    self._execute_continuous(func_name, func_cfg)
                return
            else:
                if func_name == "grasp_release":
                    if self._gesture_held_long_enough(float(func_cfg["hold_time_s"])):
                        self._transition_to_moving()
                        return
                elif func_name == "switch_to_grasping":
                    self._transition_to_grasping()
                    return

                self._hold_grasp_position()
                if func_name in ("wrist_pos", "wrist_neg"):
                    self._execute_continuous(func_name, func_cfg)
                return


def main(args=None):
    rclpy.init(args=args)
    node = EmgGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
