#!/usr/bin/env python3
"""Pipeline Manager — orchestrates the grasp pipeline state machine.

States:
  IDLE        Waiting for EMG gesture to trigger a grasp
  SEGMENTING  Point cloud segmentation in progress
  PLANNING    Grasp preshaping computation in progress
  APPROACHING Hand moving toward target (partial closure + wrist)
  GRASPING   Full closure applied, force controller active
  HOLDING     Object held, monitoring forces
  RELEASING   Opening hand to release object

Transitions are triggered by:
  - EMG gestures (POWER/PINCH/POINT -> start, OPEN -> release)
  - Pipeline events (segmentation complete, preshaping complete)
  - Proximity thresholds (approaching -> grasping)
  - Force feedback (grasping -> holding)

EMG Gesture Contract:
  POWER/PINCH/POINT (in grasp_gestures) -> Start grasp from IDLE
  OPEN -> Release from any active state
  REST (in abort_gestures) -> Abort/cancel from any active state
  (optional) manual_tighten_gesture -> Increase grip force during GRASPING/HOLDING
  (optional) manual_loosen_gesture -> Decrease grip force during GRASPING/HOLDING
"""

from __future__ import annotations

import enum
import os
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool, Float32, Float64, Int32, String
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from mia_hand_msgs.msg import ForceControllerStatus


class State(enum.IntEnum):
    IDLE = 0
    SEGMENTING = 1
    PLANNING = 2
    APPROACHING = 3
    GRASPING = 4
    HOLDING = 5
    RELEASING = 6


STATE_NAMES = {s: s.name for s in State}

# Gesture labels from EMG (must match config.py GESTURE_NAMES)
GESTURE_REST = 0
GESTURE_POWER = 1
GESTURE_PINCH = 2
GESTURE_OPEN = 3
GESTURE_POINT = 4


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
        self.declare_parameter('confidence_threshold', 0.55)
        self.declare_parameter('release_confidence_threshold', 0.25)
        self.declare_parameter('grasp_gestures', [GESTURE_POWER, GESTURE_PINCH, GESTURE_POINT])
        self.declare_parameter('release_gesture', GESTURE_OPEN)
        self.declare_parameter('state_publish_rate_hz', 5.0)

        # Release behavior parameters
        self.declare_parameter('release_open_position', [0.0, 0.0, 0.0])
        self.declare_parameter('release_joint_states_topic', '/joint_states')
        self.declare_parameter('release_timeout_s', 3.0)
        self.declare_parameter('release_joint_threshold', 0.1)

        # Gesture contract parameters
        self.declare_parameter('abort_gestures', [GESTURE_REST])
        self.declare_parameter('emergency_stop_gesture', -1)
        self.declare_parameter('manual_tighten_gesture', -1)
        self.declare_parameter('manual_loosen_gesture', -1)

        # Release safety parameters
        self.declare_parameter('release_debounce_frames', 3)

        # Live config parameters
        self.declare_parameter('emg_live_config_path', '/prosthesis_ws/config/emg_live.yaml')

        # Topic / service name parameters
        self.declare_parameter('emg_gesture_topic', '/emg/gesture_label')
        self.declare_parameter('emg_confidence_topic', '/emg/confidence')
        self.declare_parameter('grasp_type_topic', '/grasp_preshaping/grasp_type')
        self.declare_parameter('force_status_topic', '/force_controller/status')
        self.declare_parameter('compute_grasp_service', '/grasp_preshasing/compute_grasp')
        self.declare_parameter('twist_activate_service', '/twist_propagation/activate')
        self.declare_parameter('twist_deactivate_service', '/twist_propagation/deactivate')
        self.declare_parameter('pipeline_state_topic', '/pipeline/state')
        self.declare_parameter('pipeline_state_name_topic', '/pipeline/state_name')

        self._confidence_threshold = self.get_parameter('confidence_threshold').value
        self._release_confidence_threshold = self.get_parameter('release_confidence_threshold').value
        self._grasp_gestures = self.get_parameter('grasp_gestures').value
        self._release_gesture = self.get_parameter('release_gesture').value
        self._release_debounce_frames = self.get_parameter('release_debounce_frames').value
        self._abort_gestures = self.get_parameter('abort_gestures').value
        self._emergency_stop_gesture = self.get_parameter('emergency_stop_gesture').value
        self._manual_tighten_gesture = self.get_parameter('manual_tighten_gesture').value
        self._manual_loosen_gesture = self.get_parameter('manual_loosen_gesture').value

        self._release_open_position = self.get_parameter('release_open_position').value
        self._release_joint_states_topic = self.get_parameter('release_joint_states_topic').value
        self._release_timeout_s = self.get_parameter('release_timeout_s').value
        self._release_joint_threshold = self.get_parameter('release_joint_threshold').value

        rate = self.get_parameter('state_publish_rate_hz').value

        # ── State ─────────────────────────────────────────────────────────
        self._state = State.IDLE
        self._history: list[Transition] = []
        self._grasp_type: int = 0
        self._latest_confidence: float = 0.0
        self._latest_joint_positions: dict[str, float] = {}
        self._release_confidence_frames: int = 0
        self._release_monitor_timer = None
        self._release_monitor_start_time: float = 0.0
        self._force_emergency: bool = False
        self._segmenting_start_time: rclpy.time.Time | None = None

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
            Float64, '/pipeline/manual_adjust', 10)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            Int32, self.get_parameter('emg_gesture_topic').value,
            self._on_emg_gesture, 10)
        self.create_subscription(
            Float32, self.get_parameter('emg_confidence_topic').value,
            self._on_emg_confidence, 10)
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
            Bool, '/proximity/near_zone_entered',
            self._on_proximity_near_zone, latched)
        self.create_subscription(
            JointState, self._release_joint_states_topic,
            self._on_joint_states, 10)

        # ── Service clients ───────────────────────────────────────────────
        self._compute_client = self.create_client(
            Trigger, self.get_parameter('compute_grasp_service').value)
        self._twist_activate_client = self.create_client(
            Trigger, self.get_parameter('twist_activate_service').value)
        self._twist_deactivate_client = self.create_client(
            Trigger, self.get_parameter('twist_deactivate_service').value)

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._publish_state)

        # ── Live EMG config ───────────────────────────────────────────────
        self._emg_live_config_path = self.get_parameter('emg_live_config_path').value
        self._live_config_mtime: float | None = None
        self._load_live_config_at_startup()
        self.create_timer(1.0, self._poll_live_config)

        self._publish_state()
        self.get_logger().info('Pipeline manager started — state: IDLE')

    # ── Live config loading ─────────────────────────────────────────────

    def _load_live_config_at_startup(self):
        path = Path(self._emg_live_config_path)
        if not path.exists():
            self.get_logger().info(
                f'Live EMG config not found at {path}; using ROS parameters')
            return
        self._load_live_config(path)

    def _load_live_config(self, path: Path) -> bool:
        try:
            mtime = path.stat().st_mtime
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f'Failed to load live EMG config: {exc}')
            return False

        if not self._validate_and_apply_live_config(data):
            return False

        self._live_config_mtime = mtime
        return True

    def _validate_and_apply_live_config(self, data: dict) -> bool:
        errors = []
        changed = []

        # Validate confidence_threshold
        ct = data.get('confidence_threshold')
        if ct is not None:
            if not isinstance(ct, (int, float)) or not (0.0 <= ct <= 1.0):
                errors.append(f'confidence_threshold must be numeric in [0, 1], got {ct!r}')
            else:
                if ct != self._confidence_threshold:
                    changed.append(f'confidence_threshold: {self._confidence_threshold} -> {ct}')
                    self._confidence_threshold = float(ct)

        # Validate release_confidence_threshold
        rct = data.get('release_confidence_threshold')
        if rct is not None:
            if not isinstance(rct, (int, float)) or not (0.0 <= rct <= 1.0):
                errors.append(f'release_confidence_threshold must be numeric in [0, 1], got {rct!r}')
            else:
                if rct != self._release_confidence_threshold:
                    changed.append(f'release_confidence_threshold: {self._release_confidence_threshold} -> {rct}')
                    self._release_confidence_threshold = float(rct)

        # Validate grasp_gestures
        gg = data.get('grasp_gestures')
        if gg is not None:
            if not isinstance(gg, list) or not all(isinstance(x, int) for x in gg):
                errors.append(f'grasp_gestures must be a list of ints, got {gg!r}')
            else:
                if gg != self._grasp_gestures:
                    changed.append(f'grasp_gestures: {self._grasp_gestures} -> {gg}')
                    self._grasp_gestures = list(gg)

        # Validate release_gesture
        rg = data.get('release_gesture')
        if rg is not None:
            if not isinstance(rg, int):
                errors.append(f'release_gesture must be an int, got {rg!r}')
            else:
                if rg != self._release_gesture:
                    changed.append(f'release_gesture: {self._release_gesture} -> {rg}')
                    self._release_gesture = int(rg)

        if errors:
            self.get_logger().error(
                f'Live EMG config validation failed — keeping previous values. Errors: {"; ".join(errors)}'
            )
            return False

        if changed:
            self.get_logger().info(f'Live EMG config updated: {", ".join(changed)}')
        return True

    def _poll_live_config(self):
        path = Path(self._emg_live_config_path)
        if not path.exists():
            return
        try:
            mtime = path.stat().st_mtime
        except OSError:
            return
        if self._live_config_mtime is not None and mtime <= self._live_config_mtime:
            return
        self._load_live_config(path)

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

    def _can_transition(self, new_state: State) -> bool:
        """Validate state transitions."""
        valid = {
            State.IDLE: {State.SEGMENTING},
            State.SEGMENTING: {State.PLANNING, State.IDLE},
            State.PLANNING: {State.APPROACHING, State.IDLE},
            State.APPROACHING: {State.GRASPING, State.IDLE, State.RELEASING},
            State.GRASPING: {State.HOLDING, State.IDLE, State.RELEASING},
            State.HOLDING: {State.RELEASING, State.IDLE},
            State.RELEASING: {State.IDLE},
        }
        return new_state in valid.get(self._state, set())

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_force_status(self, msg: ForceControllerStatus):
        """Handle force controller status updates.

        When force_controller reports stable forces during GRASPING,
        transition to HOLDING. This is the primary trigger for the
        GRASPING -> HOLDING state transition.
        """
        self._force_emergency = getattr(msg, 'max_force_emergency', False)

        if self._state == State.GRASPING and msg.force_stable and msg.active:
            self._transition(State.HOLDING, 'Force stable — grip secured')

        # If force controller detects slip during HOLDING, we could
        # optionally re-enter GRASPING for re-grip. For now, just log.
        if self._state == State.HOLDING and msg.slip_detected:
            self.get_logger().warn('Slip detected during HOLDING — grip may be unstable')

    def _on_joint_states(self, msg: JointState):
        """Track joint positions for release completion monitoring."""
        for name, pos in zip(msg.name, msg.position):
            self._latest_joint_positions[name] = pos

    def _on_proximity_near_zone(self, msg: Bool):
        """Transition from APPROACHING to GRASPING when proximity near zone is entered."""
        if msg.data and self._state == State.APPROACHING:
            self._transition(State.GRASPING, 'Proximity: entered near zone')

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

    def _publish_finger_command(self, publisher, position: float):
        """Publish a single-joint position command as Float64MultiArray."""
        msg = Float64MultiArray()
        msg.data = [position]
        publisher.publish(msg)

    def _start_release_monitor(self):
        """Begin monitoring joint states to detect when hand is fully open."""
        self._release_monitor_start_time = time.monotonic()
        if self._release_monitor_timer is not None:
            self._release_monitor_timer.cancel()
        self._release_monitor_timer = self.create_timer(0.1, self._check_release_complete)

    def _check_release_complete(self):
        """Check if fingers have opened sufficiently or timeout reached."""
        thumb = abs(self._latest_joint_positions.get('j_thumb_fle', float('inf')))
        index = abs(self._latest_joint_positions.get('j_index_fle', float('inf')))
        mrl = abs(self._latest_joint_positions.get('j_mrl_fle', float('inf')))

        elapsed = time.monotonic() - self._release_monitor_start_time

        if thumb < self._release_joint_threshold and \
           index < self._release_joint_threshold and \
           mrl < self._release_joint_threshold:
            self._transition(State.IDLE, 'Release complete — hand open')
            if self._release_monitor_timer is not None:
                self._release_monitor_timer.cancel()
                self._release_monitor_timer = None
            return

        if elapsed >= self._release_timeout_s:
            self.get_logger().warn(
                f'Release timeout reached ({self._release_timeout_s}s) — '
                f'joints: thumb={thumb:.3f}, index={index:.3f}, mrl={mrl:.3f}'
            )
            self._transition(State.IDLE, 'Release complete — timeout')
            if self._release_monitor_timer is not None:
                self._release_monitor_timer.cancel()
                self._release_monitor_timer = None

    def _on_emg_gesture(self, msg: Int32):
        gesture = msg.data

        # Emergency stop — highest priority, from any state
        if self._emergency_stop_gesture >= 0 and gesture == self._emergency_stop_gesture:
            self.get_logger().warn('EMG: EMERGENCY STOP')
            self._publish_finger_command(self._thumb_cmd_pub, 0.0)
            self._publish_finger_command(self._index_cmd_pub, 0.0)
            self._publish_finger_command(self._mrl_cmd_pub, 0.0)
            self._deactivate_twist_propagation()
            if self._release_monitor_timer is not None:
                self._release_monitor_timer.cancel()
                self._release_monitor_timer = None
            self._transition(State.IDLE, 'EMG: emergency stop')
            return

        # Abort gesture — cancel from any active state
        if gesture in self._abort_gestures:
            if self._state not in (State.IDLE, State.RELEASING):
                self._deactivate_twist_propagation()
                if self._release_monitor_timer is not None:
                    self._release_monitor_timer.cancel()
                    self._release_monitor_timer = None
                self._transition(State.IDLE, 'EMG: abort gesture')
            return

        # Release gesture works from any active state
        if gesture == self._release_gesture:
            if self._state not in (State.IDLE, State.RELEASING):
                # Safety override: release immediately if force emergency
                if self._force_emergency:
                    self._transition(State.RELEASING, 'EMG: OPEN (force emergency override)')
                    self._deactivate_twist_propagation()
                    self._publish_finger_command(self._thumb_cmd_pub, self._release_open_position[0])
                    self._publish_finger_command(self._index_cmd_pub, self._release_open_position[1])
                    self._publish_finger_command(self._mrl_cmd_pub, self._release_open_position[2])
                    self._start_release_monitor()
                    return

                if self._latest_confidence >= self._release_confidence_threshold:
                    self._release_confidence_frames += 1
                    if self._release_confidence_frames >= self._release_debounce_frames:
                        self._transition(State.RELEASING, 'EMG: OPEN')
                        self._deactivate_twist_propagation()
                        self._publish_finger_command(self._thumb_cmd_pub, self._release_open_position[0])
                        self._publish_finger_command(self._index_cmd_pub, self._release_open_position[1])
                        self._publish_finger_command(self._mrl_cmd_pub, self._release_open_position[2])
                        self._start_release_monitor()
                else:
                    self._release_confidence_frames = 0
                    self.get_logger().debug(
                        f'Release gesture ignored: confidence {self._latest_confidence:.2f} '
                        f'< threshold {self._release_confidence_threshold}')
            return

        # Manual tighten during GRASPING or HOLDING
        if self._manual_tighten_gesture >= 0 and gesture == self._manual_tighten_gesture:
            if self._state in (State.GRASPING, State.HOLDING):
                self._manual_adjust_pub.publish(Float64(data=1.0))
                self.get_logger().debug('EMG: manual tighten')
            return

        # Manual loosen during GRASPING or HOLDING
        if self._manual_loosen_gesture >= 0 and gesture == self._manual_loosen_gesture:
            if self._state in (State.GRASPING, State.HOLDING):
                self._manual_adjust_pub.publish(Float64(data=-1.0))
                self.get_logger().debug('EMG: manual loosen')
            return

        # Grasp trigger gesture — require confidence threshold
        if gesture in self._grasp_gestures and self._state == State.IDLE:
            if self._latest_confidence < self._confidence_threshold:
                self.get_logger().debug(
                    f'Grasp gesture {gesture} ignored: confidence {self._latest_confidence:.2f} '
                    f'below threshold {self._confidence_threshold}')
                return
            self._transition(State.SEGMENTING, f'EMG: gesture={gesture}')
            self._segmenting_start_time = self.get_clock().now()
            self._activate_twist_propagation()
            # Segmentation object cloud will trigger PLANNING

    def _on_emg_confidence(self, msg: Float32):
        self._latest_confidence = msg.data

    def _on_grasp_type(self, msg: Int32):
        self._grasp_type = msg.data

    def _on_object_cloud(self, msg: PointCloud2):
        if self._state != State.SEGMENTING:
            return
        if msg.width * msg.height == 0:
            return
        # Staleness check: reject clouds that were produced before we entered SEGMENTING
        if self._segmenting_start_time is not None:
            cloud_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            if cloud_stamp < self._segmenting_start_time:
                self.get_logger().warn(
                    f'Ignoring stale object cloud (stamp={cloud_stamp.nanoseconds} '
                    f'< segmenting_start={self._segmenting_start_time.nanoseconds})')
                return
        if self._grasp_type == 0:
            self.get_logger().warn(
                'Object cloud received but grasp_type is 0 — proceeding anyway')
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
            self._transition(State.IDLE, 'Preshaping service unavailable')
            return

        future = self._compute_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_preshaping_response)

    def _on_preshaping_response(self, future):
        try:
            response = future.result()
            if response.success:
                self._transition(State.APPROACHING, f'Preshaping: {response.message[:80]}')
            else:
                self.get_logger().warn(f'Preshaping failed: {response.message}')
                self._transition(State.IDLE, f'Preshaping failed: {response.message[:60]}')
        except Exception as e:
            self.get_logger().error(f'Preshaping service error: {e}')
            self._transition(State.IDLE, f'Preshaping error: {e}')
        finally:
            self._deactivate_twist_propagation()

    # ── Publishing ────────────────────────────────────────────────────────

    def _publish_state(self):
        self._state_pub.publish(Int32(data=int(self._state)))
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
