#!/usr/bin/env python3
"""Pipeline Manager — orchestrates the grasp pipeline state machine.

States:
  IDLE        Waiting for EMG gesture to trigger a grasp
  TWISTING    Twist propagation active, waiting for collision hit
  SEGMENTING  Point cloud segmentation in progress
  PLANNING    Grasp preshaping computation in progress
  APPROACHING Hand moving toward target (partial closure + wrist)
  GRASPING   Full closure applied, force controller active
  HOLDING     Object held, monitoring forces
  VOLITIONAL  User-in-the-loop EMG control (force adjust or wrist control)
  RELEASING   Opening hand to release object

Transitions:
  IDLE -> TWISTING          (EMG POWER held >= gesture_hold_timeout_s)
  TWISTING -> SEGMENTING    (twist hit detected -> segmentation triggered)
  SEGMENTING -> PLANNING    (object cloud received)
  PLANNING -> APPROACHING   (preshaping complete)
  APPROACHING -> GRASPING   (proximity near zone entered)
  GRASPING -> HOLDING       (force stable)
  HOLDING -> VOLITIONAL     (sustained force stable -> user control)
  VOLITIONAL -> RELEASING   (EMG OPEN held >= gesture_hold_timeout_s)
  RELEASING -> IDLE         (hand physically open)

EMG Gesture Contract (all discrete gestures require hold duration + confidence):
  POWER (held >= 1.0s, conf >= 0.7) -> Enter TWISTING from IDLE
  POWER (held >= 1.0s, conf >= 0.7) -> Toggle force/wrist mode in VOLITIONAL
  OPEN  (held >= 1.0s, conf >= 0.7) -> Release from any active state
  REST  -> Ignored (no action)
  In VOLITIONAL (continuous, no hold required, proportional):
    FLEXION   -> Force+ (force mode) or Wrist+ (wrist mode)
    EXTENSION -> Force- (force mode) or Wrist- (wrist mode)"""

from __future__ import annotations

import enum
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool, Float32, Float64, Float64MultiArray, Int32, String
from std_srvs.srv import Trigger
from mia_hand_msgs.msg import ForceControllerStatus





class State(enum.IntEnum):
    IDLE = 0
    TWISTING = 1
    SEGMENTING = 2
    PLANNING = 3
    APPROACHING = 4
    GRASPING = 5
    HOLDING = 6
    VOLITIONAL = 7
    RELEASING = 8


STATE_NAMES = {s: s.name for s in State}

# Gesture labels from EMG (must match config.py GESTURE_NAMES)
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


class PipelineManagerNode(Node):
    def __init__(self):
        super().__init__('pipeline_manager')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('release_gesture', GESTURE_OPEN)
        self.declare_parameter('state_publish_rate_hz', 5.0)

        # Grace period parameters (uniform for all discrete gestures)
        self.declare_parameter('gesture_hold_timeout_s', 1.0)
        self.declare_parameter('gesture_confidence_threshold', 0.7)

        # Release behavior parameters
        self.declare_parameter('release_open_position', [0.0, 0.0, 0.0])
        self.declare_parameter('release_joint_states_topic', '/joint_states')
        self.declare_parameter('release_timeout_s', 3.0)
        self.declare_parameter('release_joint_threshold', 0.1)
        self.declare_parameter('release_debounce_frames', 3)

        # Live config parameters
        self.declare_parameter('emg_live_config_path', '')

        # EMG parameters
        self.declare_parameter('wrist_cmd_topic', '/wrist/set_position')
        self.declare_parameter('wrist_velocity_scale', 45.0)   # deg/s per unit proportional

        # Twist stop-distance & hit-detected parameters
        self.declare_parameter('twist_stop_distance_m', 0.20)
        self.declare_parameter('twist_hit_detected_topic', '/twist_propagation/hit_detected')
        self.declare_parameter('collision_distance_topic', '/twist_propagation/collision_distance')

        # Volitional mode parameters
        self.declare_parameter('volitional_force_adjust_step', 1.0)
        self.declare_parameter('volitional_wrist_velocity_scale', 45.0)
        self.declare_parameter('volitional_entry_delay_s', 0.5)

        # Topic / service name parameters
        self.declare_parameter('emg_gesture_topic', '/emg/gesture_label')
        self.declare_parameter('emg_confidence_topic', '/emg/confidence')
        self.declare_parameter('emg_proportional_topic', '/emg/proportional')
        self.declare_parameter('grasp_type_topic', '/grasp_preshaping/grasp_type')
        self.declare_parameter('force_status_topic', '/force_controller/status')
        self.declare_parameter('compute_grasp_service', '/grasp_preshaping/compute_grasp')
        self.declare_parameter('twist_activate_service', '/twist_propagation/activate')
        self.declare_parameter('twist_deactivate_service', '/twist_propagation/deactivate')
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
        self._twist_stop_distance = self.get_parameter('twist_stop_distance_m').value
        self._volitional_force_step = self.get_parameter('volitional_force_adjust_step').value
        self._volitional_wrist_scale = self.get_parameter('volitional_wrist_velocity_scale').value
        self._volitional_delay = self.get_parameter('volitional_entry_delay_s').value
        rate = self.get_parameter('state_publish_rate_hz').value

        # ── State ─────────────────────────────────────────────────────────
        self._state = State.IDLE
        self._history: list[Transition] = []
        self._grasp_type: int = 0
        self._latest_confidence: float = 0.0
        self._latest_proportional: float = 0.0
        self._segmenting_start_time: Optional[rclpy.time.Time] = None

        # Gesture hold-duration tracking
        self._pending_gesture: int = GESTURE_REST
        self._pending_gesture_start: float = time.monotonic()
        self._gesture_action_fired: bool = False

        # Release monitoring state
        self._release_start_time: Optional[float] = None
        self._release_joint_positions: list[float] = [0.0, 0.0, 0.0]
        self._release_debounce_counter: int = 0

        # EMG wrist relay state — initialized from hardware on first state msg
        self._emg_wrist_target_deg: Optional[float] = None

        # Volitional mode tracking
        self._emg_volitional_mode: str = "force"  # "force" or "wrist"
        self._volitional_entry_time: Optional[float] = None

        # Twist hit tracking
        self._twist_distance_to_hit: float = -1.0  # -1 = no active hit

        # ── Publishers ────────────────────────────────────────────────────
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(
            Int32, self.get_parameter('pipeline_state_topic').value, latched)
        self._state_name_pub = self.create_publisher(
            String, self.get_parameter('pipeline_state_name_topic').value, latched)

        # Per-finger position command publishers (for manual adjust + release)
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
            Int32, '/grasp_preshaping/grasp_type', self._on_grasp_type, 10)
        self.create_subscription(
            PointCloud2, '/segmentation/object_cloud', self._on_object_cloud, 10)
        self.create_subscription(
            ForceControllerStatus,
            self.get_parameter('force_status_topic').value,
            self._on_force_status,
            10,
        )
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)
        self.create_subscription(
            Bool, '/proximity/near_zone_entered', self._on_proximity_near_zone,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.create_subscription(
            Bool, self.get_parameter('twist_hit_detected_topic').value,
            self._on_twist_hit_detected,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            Float32, self.get_parameter('collision_distance_topic').value,
            self._on_collision_distance, 10)
        # Subscribe to wrist state to initialize target from hardware position
        self.create_subscription(
            Float64MultiArray, '/wrist/state', self._on_wrist_state, 10)

        # ── Service clients ───────────────────────────────────────────────
        self._compute_client = self.create_client(
            Trigger, self.get_parameter('compute_grasp_service').value)
        self._twist_activate_client = self.create_client(
            Trigger, self.get_parameter('twist_activate_service').value)
        self._twist_deactivate_client = self.create_client(
            Trigger, self.get_parameter('twist_deactivate_service').value)

        # ── Timer ─────────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._publish_state)

        # ── Live EMG config polling ───────────────────────────────────────
        if self._emg_live_config_path:
            self._load_emg_live_config()
            self.create_timer(1.0, self._poll_emg_live_config)

        self._publish_state()
        self.get_logger().info('Pipeline manager started — state: IDLE')

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

        if new_state in (State.IDLE, State.RELEASING) and old not in (State.IDLE, State.RELEASING):
            self._deactivate_twist_propagation()

        if old in (State.SEGMENTING, State.PLANNING):
            self._segmenting_start_time = None

        return True

    def _can_transition(self, new_state: State) -> bool:
        """Validate state transitions."""
        valid = {
            State.IDLE: {State.TWISTING, State.SEGMENTING},
            State.TWISTING: {State.SEGMENTING, State.IDLE},
            State.SEGMENTING: {State.PLANNING, State.IDLE},
            State.PLANNING: {State.APPROACHING, State.IDLE},
            State.APPROACHING: {State.GRASPING, State.IDLE, State.RELEASING},
            State.GRASPING: {State.HOLDING, State.IDLE, State.RELEASING},
            State.HOLDING: {State.VOLITIONAL, State.RELEASING, State.IDLE},
            State.VOLITIONAL: {State.RELEASING, State.IDLE},
            State.RELEASING: {State.IDLE},
        }
        return new_state in valid.get(self._state, set())

    # ── Helpers ───────────────────────────────────────────────────────────

    def _ensure_wrist_target(self) -> float:
        """Return the wrist target, initializing from hardware if needed."""
        if self._emg_wrist_target_deg is None:
            self._emg_wrist_target_deg = 0.0
        return self._emg_wrist_target_deg

    def _set_wrist_target(self, deg: float):
        """Set wrist target, clamped to [0, 359]."""
        self._emg_wrist_target_deg = max(0.0, min(359.0, deg))

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_force_status(self, msg: ForceControllerStatus):
        """Handle force controller status updates.

        GRASPING -> HOLDING on stable force.
        HOLDING -> VOLITIONAL on sustained stable force (debounced).
        """
        if self._state == State.GRASPING and msg.force_stable and msg.active:
            self._transition(State.HOLDING, 'Force stable \u2014 grip secured')
            self._volitional_entry_time = time.monotonic()

        # HOLDING -> VOLITIONAL after sustained stable force
        if self._state == State.HOLDING and msg.active:
            if msg.force_stable:
                if self._volitional_entry_time is not None:
                    elapsed = time.monotonic() - self._volitional_entry_time
                    if elapsed >= self._volitional_delay:
                        self._transition(State.VOLITIONAL, 'Sustained stable force \u2014 volitional mode')
                        self._volitional_entry_time = None
                        self._emg_volitional_mode = "force"
                else:
                    self._volitional_entry_time = time.monotonic()
            else:
                # Force became unstable — reset the timer
                self._volitional_entry_time = None

        # Slip detection during HOLDING or VOLITIONAL
        if self._state in (State.HOLDING, State.VOLITIONAL) and msg.slip_detected:
            self.get_logger().warn('Slip detected \u2014 grip may be unstable')

    def _activate_twist_propagation(self):
        if not self._twist_activate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Twist propagation activate service not available')
            return
        future = self._twist_activate_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_twist_activate_response)

    def _on_twist_activate_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Twist propagation activated')
            else:
                self.get_logger().warn(f'Twist propagation activation failed: {response.message}')
        except Exception as exc:
            self.get_logger().error(f'Twist propagation activation error: {exc}')

    def _deactivate_twist_propagation(self):
        if not self._twist_deactivate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Twist propagation deactivate service not available')
            return
        future = self._twist_deactivate_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_twist_deactivate_response)

    def _on_twist_deactivate_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Twist propagation deactivated')
            else:
                self.get_logger().warn(f'Twist propagation deactivation failed: {response.message}')
        except Exception as exc:
            self.get_logger().error(f'Twist propagation deactivation error: {exc}')

    def _on_emg_gesture(self, msg: Int32):
        gesture = msg.data
        now = time.monotonic()

        # ── Gesture change detection ──────────────────────────────────────
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
                    self._deactivate_twist_propagation()
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
                self._transition(State.TWISTING, 'EMG: POWER (held {:.1f}s)'.format(held))
                self._activate_twist_propagation()
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

            # EXTENSION — decrease force / wrist negative
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

            # FLEXION — increase force / wrist positive
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

            return  # Other gestures ignored in VOLITIONAL

        # ── All other gestures (REST, etc.) — ignored ────────────────────

    def _publish_finger_command(self, publisher, position: float):
        """Publish a single-joint position command as Float64MultiArray."""
        msg = Float64MultiArray()
        msg.data = [position]
        publisher.publish(msg)

    def _schedule_release_complete(self):
        """Monitor joint positions to detect when the hand has physically opened.

        Every 100 ms, check whether all fingers have reached release_open_position
        (with hysteresis via release_joint_threshold). After release_timeout_s,
        force-complete the release regardless.
        """
        self._release_start_time = time.monotonic()
        self._release_debounce_counter = 0
        timer_ref = {'timer': None}

        def _check_release():
            elapsed = time.monotonic() - self._release_start_time

            # Timeout: force-complete
            if elapsed >= self._release_timeout_s:
                self._transition(State.IDLE, 'Release timed out')
                timer_ref['timer'].cancel()
                return

            # Check if all fingers are at or near open position
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

    def _on_emg_confidence(self, msg: Float32):
        self._latest_confidence = msg.data

    def _on_emg_proportional(self, msg: Float32):
        self._latest_proportional = msg.data

    def _on_grasp_type(self, msg: Int32):
        self._grasp_type = msg.data

    def _on_joint_states(self, msg: JointState):
        """Track current joint positions for release monitoring."""
        for i, name in enumerate(['j_thumb_fle', 'j_index_fle', 'j_mrl_fle']):
            try:
                idx = msg.name.index(name)
                self._release_joint_positions[i] = msg.position[idx]
            except (ValueError, IndexError):
                pass

    def _on_wrist_state(self, msg: Float64MultiArray):
        """Track wrist position from hardware to initialize target on first msg."""
        if len(msg.data) >= 1 and self._emg_wrist_target_deg is None:
            self._emg_wrist_target_deg = msg.data[0]
            self.get_logger().info(
                f'Wrist target initialized from hardware: {self._emg_wrist_target_deg:.1f} deg')

    def _on_twist_hit_detected(self, msg: Bool):
        """Transition TWISTING -> SEGMENTING when a collision hit is detected."""
        if msg.data and self._state == State.TWISTING:
            self._segmenting_start_time = self.get_clock().now()
            self._transition(State.SEGMENTING, 'Twist: collision hit detected')

    def _on_collision_distance(self, msg: Float32):
        """Store distance to hit and deactivate twist when within stop-distance.

        The collision distance is published by twist_propagation as the Euclidean
        distance from the hand to the predicted collision point. When the hand
        gets within twist_stop_distance_m (default 0.20 m), twist is deactivated.
        A negative value means no active collision (no hit found).
        """
        self._twist_distance_to_hit = msg.data

        # Deactivate twist when close enough to collision point
        if self._state == State.TWISTING:
            if 0.0 < msg.data < self._twist_stop_distance:
                self._deactivate_twist_propagation()
                self.get_logger().info(
                    f'Twist deactivated: collision distance {msg.data:.3f}m < '
                    f'stop threshold {self._twist_stop_distance:.3f}m')

    def _on_proximity_near_zone(self, msg: Bool):
        """Transition APPROACHING -> GRASPING when near zone is entered."""
        if msg.data and self._state == State.APPROACHING:
            self._transition(State.GRASPING, 'Proximity: near zone entered')



    def _load_emg_live_config(self):
        """Load EMG live config from emg_live.yaml into dynamic parameters."""
        path = Path(self._emg_live_config_path)
        if not path.is_file():
            self.get_logger().warn(f'EMG live config not found: {path}')
            return
        try:
            with open(path) as f:
                config = yaml.safe_load(f)
            if config is None:
                return
            # Update parameters from live config
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
        """Poll EMG live config file for changes (called at 1 Hz)."""
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

    def _on_object_cloud(self, msg: PointCloud2):
        if self._state not in (State.SEGMENTING, State.TWISTING):
            return
        if msg.width * msg.height == 0:
            return
        # Staleness check: reject clouds produced before entering SEGMENTING
        if self._segmenting_start_time is not None:
            cloud_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            if cloud_stamp < self._segmenting_start_time:
                self.get_logger().info(
                    f'Ignoring stale object cloud (stamp={cloud_stamp.nanoseconds} '
                    f'< segmenting_start={self._segmenting_start_time.nanoseconds})',
                    throttle_duration_sec=5.0)
                return
        if self._grasp_type == 0:
            self.get_logger().info(
                'Object cloud received but grasp_type is 0 \u2014 proceeding anyway',
                throttle_duration_sec=5.0)
        self._transition(State.PLANNING, 'Segmentation complete: object cloud received')
        self._request_preshaping()

    def _request_preshaping(self):
        """Call the grasp preshaping compute service.

        NOTE: pipeline_manager is the single owner of preshaping orchestration
        per segmentation target. twist_propagation should NOT independently
        call /grasp_preshaping/compute_grasp; it receives target poses from
        the pipeline_manager state machine instead.
        """
        if not self._compute_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Preshaping service not available')
            self._deactivate_twist_propagation()
            self._transition(State.IDLE, 'Preshaping service unavailable')
            return

        future = self._compute_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_preshaping_response)

    def _on_preshaping_response(self, future):
        try:
            response = future.result()
            if response.success:
                self._transition(State.APPROACHING, f'Preshaping: {response.message[:80]}')
                self._deactivate_twist_propagation()
            else:
                self.get_logger().warn(f'Preshaping failed: {response.message}')
                self._transition(State.IDLE, f'Preshaping failed: {response.message[:60]}')
        except Exception as e:
            self.get_logger().error(f'Preshaping service error: {e}')
            self._transition(State.IDLE, f'Preshaping error: {e}')

    # ── Publishing ────────────────────────────────────────────────────────
    def _publish_state(self):
        msg = Int32()
        msg.data = int(self._state)
        self._state_pub.publish(msg)
        self._state_name_pub.publish(String(data=self._state.name))


def main(args=None):
    rclpy.init(args=args)
    node = PipelineManagerNode()
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
