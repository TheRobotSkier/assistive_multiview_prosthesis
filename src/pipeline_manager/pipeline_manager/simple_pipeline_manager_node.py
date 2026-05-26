#!/usr/bin/env python3
"""Simple Pipeline Manager -- EMG-triggered grasp without perception.

States:
  IDLE        Waiting for EMG POWER gesture (held >= 1s)
  APPROACHING Wrist preshapes, brief gate before force closure (state 4)
  GRASPING    Force controller closes the grasp (state 5)
  HOLDING     Object held, forces stable (state 6)
  VOLITIONAL  User-in-the-loop EMG control (state 7)
  RELEASING   Opening hand to release object (state 8)

Transitions:
  IDLE -> APPROACHING  (EMG POWER held >= gesture_hold_timeout_s)
  APPROACHING -> GRASPING (automatic after wrist_preshape_delay_s)
  GRASPING -> HOLDING  (force_stable from force controller)
  HOLDING -> VOLITIONAL (sustained force stable + volitional_entry_delay)
  VOLITIONAL -> RELEASING (EMG OPEN held >= gesture_hold_timeout_s)
  RELEASING -> IDLE       (hand physically open or timeout)

State enum values (4-8) match force_controller expectations:
  APPROACHING=4  GRASPING=5  HOLDING=6  VOLITIONAL=7  RELEASING=8
The force controller activates on PLANNING/APPROACHING -> GRASPING.
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64, Float64MultiArray, Int32, String
from mia_hand_msgs.msg import ForceControllerStatus


class State(enum.IntEnum):
    IDLE = 0
    APPROACHING = 4
    GRASPING = 5
    HOLDING = 6
    VOLITIONAL = 7
    RELEASING = 8


GESTURE_REST = 0
GESTURE_POWER = 1
GESTURE_OPEN = 2
GESTURE_FLEXION = 3
GESTURE_EXTENSION = 4


@dataclass
class Transition:
    from_state: State
    to_state: State
    reason: str
    timestamp: float


class SimplePipelineManagerNode(Node):
    def __init__(self):
        super().__init__('simple_pipeline_manager')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('release_gesture', GESTURE_OPEN)
        self.declare_parameter('state_publish_rate_hz', 5.0)

        self.declare_parameter('gesture_hold_timeout_s', 1.0)
        self.declare_parameter('gesture_confidence_threshold', 0.7)

        self.declare_parameter('release_open_position', [0.0, 0.0, 0.0])
        self.declare_parameter('release_joint_threshold', 0.1)
        self.declare_parameter('release_timeout_s', 3.0)
        self.declare_parameter('release_debounce_frames', 3)

        self.declare_parameter('emg_live_config_path', '')

        self.declare_parameter('wrist_cmd_topic', '/wrist/set_position')
        self.declare_parameter('wrist_velocity_scale', 45.0)

        self.declare_parameter('volitional_force_adjust_step', 1.0)
        self.declare_parameter('volitional_wrist_velocity_scale', 45.0)
        self.declare_parameter('volitional_entry_delay_s', 0.5)

        self.declare_parameter('default_wrist_preshape_deg', 180.0)
        self.declare_parameter('wrist_preshape_accel_deg_s2', 180.0)
        self.declare_parameter('wrist_preshape_delay_s', 0.5)

        self.declare_parameter('emg_gesture_topic', '/emg/gesture_label')
        self.declare_parameter('emg_confidence_topic', '/emg/confidence')
        self.declare_parameter('emg_proportional_topic', '/emg/proportional')
        self.declare_parameter('force_status_topic', '/force_controller/status')
        self.declare_parameter('pipeline_state_topic', '/pipeline/state')
        self.declare_parameter('pipeline_state_name_topic', '/pipeline/state_name')

        self._release_gesture = self.get_parameter('release_gesture').value
        self._gesture_hold_timeout_s = self.get_parameter('gesture_hold_timeout_s').value
        self._gesture_confidence_threshold = self.get_parameter('gesture_confidence_threshold').value
        self._release_open_position = self.get_parameter('release_open_position').value
        self._release_joint_threshold = self.get_parameter('release_joint_threshold').value
        self._release_timeout_s = self.get_parameter('release_timeout_s').value
        self._release_debounce_frames = self.get_parameter('release_debounce_frames').value
        self._emg_live_config_path = self.get_parameter('emg_live_config_path').value
        self._wrist_velocity_scale = self.get_parameter('wrist_velocity_scale').value
        self._volitional_force_step = self.get_parameter('volitional_force_adjust_step').value
        self._volitional_wrist_scale = self.get_parameter('volitional_wrist_velocity_scale').value
        self._volitional_delay = self.get_parameter('volitional_entry_delay_s').value
        self._wrist_preshape_deg = self.get_parameter('default_wrist_preshape_deg').value
        self._wrist_preshape_accel = self.get_parameter('wrist_preshape_accel_deg_s2').value
        self._wrist_preshape_delay_s = self.get_parameter('wrist_preshape_delay_s').value

        rate = self.get_parameter('state_publish_rate_hz').value

        # ── State ─────────────────────────────────────────────────────────
        self._state = State.IDLE
        self._history: list[Transition] = []
        self._latest_confidence: float = 0.0
        self._latest_proportional: float = 0.0

        self._pending_gesture: int = GESTURE_REST
        self._pending_gesture_start: float = time.monotonic()
        self._gesture_action_fired: bool = False

        self._release_start_time: Optional[float] = None
        self._release_joint_positions: list[float] = [0.0, 0.0, 0.0]
        self._release_debounce_counter: int = 0

        self._emg_wrist_target_deg: Optional[float] = None
        self._emg_volitional_mode: str = "force"
        self._volitional_entry_time: Optional[float] = None

        # ── Publishers ────────────────────────────────────────────────────
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(
            Int32, self.get_parameter('pipeline_state_topic').value, latched)
        self._state_name_pub = self.create_publisher(
            String, self.get_parameter('pipeline_state_name_topic').value, latched)

        self._thumb_cmd_pub = self.create_publisher(
            Float64MultiArray, '/thumb_pos_ff_controller/commands', 10)
        self._index_cmd_pub = self.create_publisher(
            Float64MultiArray, '/index_pos_ff_controller/commands', 10)
        self._mrl_cmd_pub = self.create_publisher(
            Float64MultiArray, '/mrl_pos_ff_controller/commands', 10)
        self._manual_adjust_pub = self.create_publisher(
            Float64, '/force_controller/manual_adjust', 10)
        self._wrist_cmd_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter('wrist_cmd_topic').value,
            10,
        )

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            Int32, self.get_parameter('emg_gesture_topic').value,
            self._on_emg_gesture, 10)
        self.create_subscription(
            Float32, self.get_parameter('emg_confidence_topic').value,
            self._on_emg_confidence, 10)
        self.create_subscription(
            Float32, self.get_parameter('emg_proportional_topic').value,
            self._on_emg_proportional, 10)
        self.create_subscription(
            ForceControllerStatus,
            self.get_parameter('force_status_topic').value,
            self._on_force_status,
            10,
        )
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)
        self.create_subscription(
            Float64MultiArray, '/wrist/state', self._on_wrist_state, 10)

        # ── Timer ─────────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._publish_state)

        # ── Live EMG config polling ───────────────────────────────────────
        if self._emg_live_config_path:
            self._load_emg_live_config()
            self.create_timer(1.0, self._poll_emg_live_config)

        self._publish_state()
        self.get_logger().info('Simple pipeline manager started -- state: IDLE')

    # ── State machine ─────────────────────────────────────────────────────

    def _transition(self, new_state: State, reason: str) -> bool:
        if new_state == self._state:
            return False

        old = self._state
        self._state = new_state
        transition = Transition(
            from_state=old,
            to_state=new_state,
            reason=reason,
            timestamp=time.time(),
        )
        self._history.append(transition)
        self.get_logger().info(
            f'State: {old.name} -> {new_state.name} ({reason})')
        self._publish_state()
        return True

    # ── Helpers ───────────────────────────────────────────────────────────

    def _ensure_wrist_target(self) -> float:
        if self._emg_wrist_target_deg is None:
            self._emg_wrist_target_deg = 0.0
        return self._emg_wrist_target_deg

    def _set_wrist_target(self, deg: float):
        self._emg_wrist_target_deg = max(0.0, min(359.0, deg))

    def _publish_wrist_cmd(self, target_deg: float):
        cmd = Float64MultiArray()
        cmd.data = [target_deg, self._wrist_preshape_accel]
        self._wrist_cmd_pub.publish(cmd)
        self._set_wrist_target(target_deg)

    def _publish_finger_command(self, publisher, position: float):
        msg = Float64MultiArray()
        msg.data = [position]
        publisher.publish(msg)

    def _schedule_release_complete(self):
        self._release_start_time = time.monotonic()
        self._release_debounce_counter = 0
        timer_ref = {'timer': None}

        def _check_release():
            elapsed = time.monotonic() - self._release_start_time

            if elapsed >= self._release_timeout_s:
                self._transition(State.IDLE, 'Release timed out')
                timer_ref['timer'].cancel()
                return

            all_open = all(
                self._release_joint_positions[i] <= self._release_open_position[i] + self._release_joint_threshold
                for i in range(3)
            )
            if all_open:
                self._release_debounce_counter += 1
                if self._release_debounce_counter >= self._release_debounce_frames:
                    self._transition(State.IDLE, 'Release complete - all fingers open')
                    timer_ref['timer'].cancel()
            else:
                self._release_debounce_counter = 0

        timer_ref['timer'] = self.create_timer(0.1, _check_release)

    def _enter_approaching(self):
        """Publish wrist preshape and schedule automatic transition to GRASPING.

        The force_controller activates on APPROACHING -> GRASPING.
        The wrist starts rotating concurrently with the hand closing.
        """
        self.get_logger().info(
            f'Wrist preshape: {self._wrist_preshape_deg:.0f} deg')
        self._publish_wrist_cmd(self._wrist_preshape_deg)

        def _transition_to_grasping():
            self._transition(State.GRASPING, 'EMG force command -- closing grasp')

        self.create_timer(self._wrist_preshape_delay_s, _transition_to_grasping, oneshot=True)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_force_status(self, msg: ForceControllerStatus):
        if self._state == State.GRASPING and msg.force_stable and msg.active:
            self._transition(State.HOLDING, 'Force stable -- grip secured')
            self._volitional_entry_time = time.monotonic()

        if self._state == State.HOLDING and msg.active:
            if msg.force_stable:
                if self._volitional_entry_time is not None:
                    elapsed = time.monotonic() - self._volitional_entry_time
                    if elapsed >= self._volitional_delay:
                        self._transition(State.VOLITIONAL, 'Sustained stable force -- volitional mode')
                        self._volitional_entry_time = None
                        self._emg_volitional_mode = "force"
                else:
                    self._volitional_entry_time = time.monotonic()
            else:
                self._volitional_entry_time = None

        if self._state in (State.HOLDING, State.VOLITIONAL) and msg.slip_detected:
            self.get_logger().warn('Slip detected -- grip may be unstable')

    def _on_emg_gesture(self, msg: Int32):
        gesture = msg.data
        now = time.monotonic()

        if gesture != self._pending_gesture:
            self.get_logger().debug(
                f'Gesture changed: {self._pending_gesture} -> {gesture}, hold timer reset')
            self._pending_gesture = gesture
            self._pending_gesture_start = now
            self._gesture_action_fired = False

        held = now - self._pending_gesture_start
        held_long_enough = held >= self._gesture_hold_timeout_s
        high_confidence = self._latest_confidence >= self._gesture_confidence_threshold

        # ── OPEN release (from any active state) ──────────────────────────
        if gesture == self._release_gesture:
            if self._state not in (State.IDLE, State.RELEASING):
                if held_long_enough and high_confidence and not self._gesture_action_fired:
                    self._transition(State.RELEASING, 'EMG: OPEN')
                    self._publish_finger_command(self._thumb_cmd_pub, self._release_open_position[0])
                    self._publish_finger_command(self._index_cmd_pub, self._release_open_position[1])
                    self._publish_finger_command(self._mrl_cmd_pub, self._release_open_position[2])
                    self._schedule_release_complete()
                    self._volitional_entry_time = None
                    self._gesture_action_fired = True
            return

        # ── POWER: activate from IDLE ─────────────────────────────────────
        if gesture == GESTURE_POWER and self._state == State.IDLE:
            if held_long_enough and high_confidence and not self._gesture_action_fired:
                self.get_logger().info(
                    f'POWER gesture held {held:.1f}s -- entering APPROACHING')
                self._transition(State.APPROACHING, f'EMG: POWER (held {held:.1f}s)')
                self._enter_approaching()
                self._gesture_action_fired = True
            return

        # ── POWER: toggle mode in VOLITIONAL ──────────────────────────────
        if gesture == GESTURE_POWER and self._state == State.VOLITIONAL:
            if held_long_enough and high_confidence and not self._gesture_action_fired:
                if self._emg_volitional_mode == "force":
                    self._emg_volitional_mode = "wrist"
                    self.get_logger().info('VOLITIONAL: switched to WRIST mode')
                else:
                    self._emg_volitional_mode = "force"
                    self.get_logger().info('VOLITIONAL: switched to FORCE mode')
                self._gesture_action_fired = True
            return

        # ── Continuous gestures in VOLITIONAL (proportional, no hold) ─────
        if self._state == State.VOLITIONAL:
            prop = self._latest_proportional

            if gesture == GESTURE_EXTENSION:
                if prop <= 0.0:
                    return
                if self._emg_volitional_mode == "force":
                    self._manual_adjust_pub.publish(
                        Float64(data=-self._volitional_force_step * prop))
                    self.get_logger().debug('VOLITIONAL: force -', throttle_duration_sec=0.5)
                else:
                    delta = self._volitional_wrist_scale * 0.1 * prop
                    self._set_wrist_target(self._ensure_wrist_target() - delta)
                    cmd = Float64MultiArray()
                    cmd.data = [self._emg_wrist_target_deg, 180.0]
                    self._wrist_cmd_pub.publish(cmd)
                    self.get_logger().debug('VOLITIONAL: wrist -', throttle_duration_sec=0.5)
                return

            if gesture == GESTURE_FLEXION:
                if prop <= 0.0:
                    return
                if self._emg_volitional_mode == "force":
                    self._manual_adjust_pub.publish(
                        Float64(data=self._volitional_force_step * prop))
                    self.get_logger().debug('VOLITIONAL: force +', throttle_duration_sec=0.5)
                else:
                    delta = self._volitional_wrist_scale * 0.1 * prop
                    self._set_wrist_target(self._ensure_wrist_target() + delta)
                    cmd = Float64MultiArray()
                    cmd.data = [self._emg_wrist_target_deg, 180.0]
                    self._wrist_cmd_pub.publish(cmd)
                    self.get_logger().debug('VOLITIONAL: wrist +', throttle_duration_sec=0.5)
                return

            return

    def _on_emg_confidence(self, msg: Float32):
        self._latest_confidence = msg.data

    def _on_emg_proportional(self, msg: Float32):
        self._latest_proportional = msg.data

    def _on_joint_states(self, msg: JointState):
        for i, name in enumerate(['j_thumb_fle', 'j_index_fle', 'j_mrl_fle']):
            try:
                idx = msg.name.index(name)
                self._release_joint_positions[i] = msg.position[idx]
            except (ValueError, IndexError):
                pass

    def _on_wrist_state(self, msg: Float64MultiArray):
        if len(msg.data) >= 1 and self._emg_wrist_target_deg is None:
            self._emg_wrist_target_deg = msg.data[0]
            self.get_logger().info(
                f'Wrist target initialized from hardware: {self._emg_wrist_target_deg:.1f} deg')

    def _publish_state(self):
        msg = Int32()
        msg.data = int(self._state)
        self._state_pub.publish(msg)
        self._state_name_pub.publish(String(data=self._state.name))

    def _load_emg_live_config(self):
        path = Path(self._emg_live_config_path)
        if not path.is_file():
            self.get_logger().warn(f'EMG live config not found: {path}')
            return
        try:
            with open(path) as f:
                config = yaml.safe_load(f)
            if config is None:
                return
            if 'gesture_hold_timeout_s' in config:
                val = float(config['gesture_hold_timeout_s'])
                if val > 0:
                    self._gesture_hold_timeout_s = val
            if 'gesture_confidence_threshold' in config:
                val = float(config['gesture_confidence_threshold'])
                if 0.0 <= val <= 1.0:
                    self._gesture_confidence_threshold = val
            if 'release_gesture' in config:
                self._release_gesture = int(config['release_gesture'])
            self.get_logger().info(f'Loaded EMG live config: {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load EMG live config: {e}')

    def _poll_emg_live_config(self):
        path = Path(self._emg_live_config_path)
        if not hasattr(self, '_last_config_mtime'):
            self._last_config_mtime = 0.0
        try:
            mtime = path.stat().st_mtime
            if mtime > self._last_config_mtime:
                self._last_config_mtime = mtime
                self._load_emg_live_config()
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SimplePipelineManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
