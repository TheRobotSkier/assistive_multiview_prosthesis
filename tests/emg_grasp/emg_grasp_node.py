#!/usr/bin/env python3
"""EMG-driven mode-based velocity force grasp test node.

Subscribes to EMG gestures and /joint_states, publishes velocity commands to
/group_vel_ff_controller/commands, position commands to
/group_pos_ff_controller/commands, and wrist position commands to
/wrist/set_position.

Mode state machine:
  moving    → Hand open/relaxed. Wrist can be positioned. Grasp can be started.
  grasping  → Velocity ramp closure. Wrist can still be adjusted.
              Release gesture cancels grasp and returns to moving.
              Contact detection auto-transitions to holding.
  holding   → Contact detected. Grasp force can be adjusted.
              Release gesture opens hand and returns to moving.
"""

import os
import sys
import time
import math
from typing import Dict, List, Optional, Tuple

import yaml

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, Int32, Float32
    from sensor_msgs.msg import JointState
except ModuleNotFoundError:
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

        sp = self._cfg["stop_positions"]
        ft = self._cfg["force_thresholds"]
        self._stop_positions = [float(sp[n]) for n in FINGER_JOINTS]
        self._force_thresholds = [float(ft[n]) for n in FINGER_JOINTS]

        self._ramp = self._compute_velocity_ramp(self._v_start, self._v_end, self._decay_steps)

        # Mode configuration
        self._modes = self._cfg["modes"]
        self._mode = MODE_MOVING

        # Wrist control
        self._wrist_control_enabled = bool(self._cfg.get("wrist_control_enabled", False))
        self._wrist_accel = float(self._cfg.get("wrist_accel_deg_s2", 180.0))
        self._wrist_position = 0.0

        # ── Publishers ────────────────────────────────────────────────────────
        self._vel_pub = self.create_publisher(
            Float64MultiArray, "/group_vel_ff_controller/commands", 10)
        self._pos_pub = self.create_publisher(
            Float64MultiArray, "/group_pos_ff_controller/commands", 10)
        if self._wrist_control_enabled:
            self._wrist_pub = self.create_publisher(
                Float64MultiArray, self._cfg["wrist_cmd_topic"], 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self._positions = [0.0] * FINGER_COUNT
        self._efforts = [0.0] * FINGER_COUNT
        self._got_js = False
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        self._emg_gesture = 0
        self._emg_confidence = 0.0
        self._emg_gesture_start_time = None
        self.create_subscription(Int32, self._cfg["emg_gesture_topic"], self._on_gesture, 10)
        self.create_subscription(Float32, self._cfg["emg_confidence_topic"], self._on_confidence, 10)

        # ── State ─────────────────────────────────────────────────────────────
        self._step_idx = 0
        self._stop_reason = None

        # ── Timer ─────────────────────────────────────────────────────────────
        self._timer = self.create_timer(self._step_interval, self._control_loop)

        self.get_logger().info("EMG Grasp Test node started — waiting in moving mode")

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

    def _on_confidence(self, msg: Float32):
        self._emg_confidence = msg.data

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _compute_velocity_ramp(v_start: float, v_end: float, decay_steps: int) -> List[float]:
        if decay_steps <= 1:
            return [v_start]
        ramp = []
        for i in range(decay_steps):
            t = i / (decay_steps - 1)
            ramp.append(v_start + t * (v_end - v_start))
        return ramp

    def _publish_velocity(self, v: float):
        msg = Float64MultiArray()
        msg.data = [v, v, v]
        self._vel_pub.publish(msg)

    def _publish_position(self, pos: float):
        msg = Float64MultiArray()
        msg.data = [pos, pos, pos]
        self._pos_pub.publish(msg)

    @staticmethod
    def check_stop_conditions(positions, efforts, stop_positions, force_thresholds) -> Optional[str]:
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
        elif func_name in ("force_inc", "force_dec"):
            vel = float(func_cfg["velocity"])
            self._publish_velocity(vel)

    def _stop_continuous_action(self):
        """Stop any active continuous motion."""
        self._publish_velocity(0.0)

    def _update_grasping(self):
        """Velocity ramp closure and auto-transition to holding on contact."""
        if self._step_idx < len(self._ramp):
            vel = self._ramp[self._step_idx]
            self._publish_velocity(vel)
            self._step_idx += 1
        else:
            # Post-ramp: continue at v_end
            self._publish_velocity(self._v_end)

        self._stop_reason = self._check_stop_conditions()
        if self._stop_reason:
            self.get_logger().info(f"Contact detected: {self._stop_reason}")
            self._publish_velocity(0.0)
            self._mode = MODE_HOLDING

    def _transition_to_grasping(self):
        """One-shot: start grasp closure, enter grasping mode."""
        self.get_logger().info("EMG grasp triggered — entering grasping")
        self._mode = MODE_GRASPING
        self._step_idx = 0
        self._stop_reason = None
        # Ensure position controller is active before we start (hand is open)
        self._publish_position(0.0)

    def _transition_to_moving(self):
        """One-shot: open hand and return to moving mode."""
        self.get_logger().info("EMG release — entering moving")
        self._publish_velocity(0.0)
        time.sleep(0.2)
        self._publish_position(0.0)
        self._mode = MODE_MOVING

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if not self._got_js:
            self.get_logger().warn("No joint states yet — skipping cycle")
            return

        # ---- GRASPING: velocity ramp with optional wrist control ----
        if self._mode == MODE_GRASPING:
            self._update_grasping()

            # If we auto-transitioned to holding, don't check gestures this cycle
            if self._mode == MODE_HOLDING:
                self._stop_continuous_action()
                return

            # Check for release gesture or wrist control
            func_name, func_cfg = self._dispatch_gesture()
            if func_name == "grasp_release":
                if self._gesture_held_long_enough(float(func_cfg["hold_time_s"])):
                    self._transition_to_moving()
                else:
                    self._stop_continuous_action()
            elif func_name in ("wrist_pos", "wrist_neg"):
                if self._gesture_held_long_enough(float(func_cfg["hold_time_s"])):
                    self._execute_continuous(func_name, func_cfg)
                else:
                    self._stop_continuous_action()
            else:
                self._stop_continuous_action()
            return

        # ---- MOVING and HOLDING: gesture dispatch ----
        func_name, func_cfg = self._dispatch_gesture()

        if func_name is None:
            self._stop_continuous_action()
            return

        if not self._gesture_held_long_enough(float(func_cfg["hold_time_s"])):
            self._stop_continuous_action()
            return

        # Execute based on function type
        if func_name in ("wrist_pos", "wrist_neg", "force_inc", "force_dec"):
            self._execute_continuous(func_name, func_cfg)
        elif func_name == "grasp_activate":
            self._transition_to_grasping()
        elif func_name == "grasp_release":
            self._transition_to_moving()


def main(args=None):
    rclpy.init(args=args)
    node = EmgGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_velocity(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
