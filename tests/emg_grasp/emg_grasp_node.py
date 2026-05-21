#!/usr/bin/env python3
"""EMG-driven velocity-based force grasp test node.

Subscribes to EMG gestures and /joint_states, publishes velocity commands to
/group_vel_ff_controller/commands and position commands to
/group_pos_ff_controller/commands.

State machine:
  IDLE      → Hand open, waiting for EMG grasp trigger (POWER gesture hold)
  CLOSING   → Velocity ramp closure, monitoring forces/positions
  HOLDING   → Contact detected, zero velocity, waiting for release
  RELEASING → EMG release gesture (OPEN) or fault, switch to position control, open hand
  FAULT     → Safety fault, stop all motion
"""

import os
import sys
import time
import math
from enum import Enum, auto
from typing import List, Optional

import yaml

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, Int32, Float32
    from sensor_msgs.msg import JointState
except ModuleNotFoundError:
    # Stubs allow static-method unit tests without ROS2 installed
    class Node:  # type: ignore[misc]
        pass

    class Float64MultiArray:  # type: ignore[no-redef]
        pass

    class Int32:  # type: ignore[no-redef]
        def __init__(self, data=0):
            self.data = data

    class Float32:  # type: ignore[no-redef]
        def __init__(self, data=0.0):
            self.data = data

    class JointState:  # type: ignore[no-redef]
        pass

FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_COUNT = 3


class Phase(Enum):
    IDLE = auto()
    CLOSING = auto()
    HOLDING = auto()
    RELEASING = auto()
    FAULT = auto()


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

        # EMG params
        self._grasp_gesture = int(self._cfg["emg_grasp_trigger_gesture"])
        self._release_gesture = int(self._cfg["emg_release_gesture"])
        self._confidence_thresh = float(self._cfg["emg_grasp_confidence_threshold"])
        self._hold_timeout = float(self._cfg["gesture_hold_timeout_s"])

        # ── Publishers ────────────────────────────────────────────────────────
        self._vel_pub = self.create_publisher(
            Float64MultiArray, "/group_vel_ff_controller/commands", 10)
        self._pos_pub = self.create_publisher(
            Float64MultiArray, "/group_pos_ff_controller/commands", 10)

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
        self._phase = Phase.IDLE
        self._step_idx = 0
        self._stop_reason = None

        # ── Timer ─────────────────────────────────────────────────────────────
        self._timer = self.create_timer(self._step_interval, self._control_loop)

        self.get_logger().info("EMG Grasp Test node started — waiting in IDLE")

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

    def _gesture_held_long_enough(self) -> bool:
        if self._emg_gesture_start_time is None:
            return False
        return (time.time() - self._emg_gesture_start_time) >= self._hold_timeout

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if not self._got_js:
            self.get_logger().warn("No joint states yet — skipping cycle")
            return

        # ---- IDLE: waiting for grasp trigger ----
        if self._phase == Phase.IDLE:
            if (self._emg_gesture == self._grasp_gesture
                    and self._emg_confidence >= self._confidence_thresh
                    and self._gesture_held_long_enough()):
                self.get_logger().info("EMG grasp triggered — entering CLOSING")
                self._phase = Phase.CLOSING
                self._step_idx = 0
                self._stop_reason = None
                # Ensure position controller is active before we start (hand is open)
                self._publish_position(0.0)
            return

        # ---- CLOSING: velocity ramp ----
        if self._phase == Phase.CLOSING:
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
                self._phase = Phase.HOLDING

            # EMG release gesture cancels closing
            if (self._emg_gesture == self._release_gesture
                    and self._emg_confidence >= self._confidence_thresh):
                self.get_logger().info("EMG release during CLOSING")
                self._publish_velocity(0.0)
                self._phase = Phase.RELEASING
            return

        # ---- HOLDING: maintain grasp, wait for release ----
        if self._phase == Phase.HOLDING:
            if (self._emg_gesture == self._release_gesture
                    and self._emg_confidence >= self._confidence_thresh
                    and self._gesture_held_long_enough()):
                self.get_logger().info("EMG release triggered — entering RELEASING")
                self._phase = Phase.RELEASING
            return

        # ---- RELEASING: open hand ----
        if self._phase == Phase.RELEASING:
            self._publish_velocity(0.0)
            time.sleep(0.2)
            self._publish_position(0.0)
            self.get_logger().info("Hand released — returning to IDLE")
            self._phase = Phase.IDLE
            return

        # ---- FAULT: safety stop ----
        if self._phase == Phase.FAULT:
            self._publish_velocity(0.0)
            return


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
