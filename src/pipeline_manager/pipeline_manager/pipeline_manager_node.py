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
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import PointCloud2
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
        self.declare_parameter('release_confidence_threshold', 0.30)
        self.declare_parameter('grasp_gestures', [GESTURE_POWER, GESTURE_PINCH, GESTURE_POINT])
        self.declare_parameter('release_gesture', GESTURE_OPEN)
        self.declare_parameter('state_publish_rate_hz', 5.0)

        # Topic / service name parameters
        self.declare_parameter('emg_gesture_topic', '/emg/gesture_label')
        self.declare_parameter('emg_confidence_topic', '/emg/confidence')
        self.declare_parameter('grasp_type_topic', '/grasp_preshaping/grasp_type')
        self.declare_parameter('force_status_topic', '/force_controller/status')
        self.declare_parameter('compute_grasp_service', '/grasp_preshaping/compute_grasp')
        self.declare_parameter('twist_activate_service', '/twist_propagation/activate')
        self.declare_parameter('twist_deactivate_service', '/twist_propagation/deactivate')
        self.declare_parameter('pipeline_state_topic', '/pipeline/state')
        self.declare_parameter('pipeline_state_name_topic', '/pipeline/state_name')

        self._confidence_threshold = self.get_parameter('confidence_threshold').value
        self._release_confidence_threshold = self.get_parameter('release_confidence_threshold').value
        self._grasp_gestures = self.get_parameter('grasp_gestures').value
        self._release_gesture = self.get_parameter('release_gesture').value
        rate = self.get_parameter('state_publish_rate_hz').value

        # ── State ─────────────────────────────────────────────────────────
        self._state = State.IDLE
        self._history: list[Transition] = []
        self._grasp_type: int = 0
        self._latest_confidence: float = 0.0

        # ── Publishers ────────────────────────────────────────────────────
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(
            Int32, self.get_parameter('pipeline_state_topic').value, latched)
        self._state_name_pub = self.create_publisher(
            String, self.get_parameter('pipeline_state_name_topic').value, latched)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            Int32, self.get_parameter('emg_gesture_topic').value,
            self._on_emg_gesture, 10)
        self.create_subscription(
            Float32, self.get_parameter('emg_confidence_topic').value,
            self._on_emg_confidence, 10)  # noqa: F821
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

        # ── Service clients ───────────────────────────────────────────────
        self._compute_client = self.create_client(
            Trigger, self.get_parameter('compute_grasp_service').value)
        self._twist_activate_client = self.create_client(
            Trigger, self.get_parameter('twist_activate_service').value)
        self._twist_deactivate_client = self.create_client(
            Trigger, self.get_parameter('twist_deactivate_service').value)

        # ── Timer ─────────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._publish_state)

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
        if self._state == State.GRASPING and msg.force_stable and msg.active:
            self._transition(State.HOLDING, 'Force stable — grip secured')

        # If force controller detects slip during HOLDING, we could
        # optionally re-enter GRASPING for re-grip. For now, just log.
        if self._state == State.HOLDING and msg.slip_detected:
            self.get_logger().warn('Slip detected during HOLDING — grip may be unstable')

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

        # Release gesture works from any active state
        if gesture == self._release_gesture:
            if self._state not in (State.IDLE, State.RELEASING):
                # Release is allowed even at lower confidence for safety
                if self._latest_confidence >= self._release_confidence_threshold:
                    self._transition(State.RELEASING, 'EMG: OPEN')
                    self._deactivate_twist_propagation()
                    self._schedule_release_complete()
                else:
                    self.get_logger().debug(
                        f'Release gesture ignored: confidence {self._latest_confidence:.2f} '
                        f'< threshold {self._release_confidence_threshold}')
            return

        # Grasp trigger gesture — require confidence threshold
        if gesture in self._grasp_gestures and self._state == State.IDLE:
            if self._latest_confidence < self._confidence_threshold:
                self.get_logger().debug(
                    f'Grasp gesture {gesture} ignored: confidence {self._latest_confidence:.2f} '
                    f'below threshold {self._confidence_threshold}')
                return
            self._transition(State.SEGMENTING, f'EMG: gesture={gesture}')
            self._activate_twist_propagation()
            # Segmentation object cloud will trigger PLANNING

    def _schedule_release_complete(self):
        """Return to IDLE after release without relying on a one-shot timer API."""
        timer_ref = {'timer': None}

        def _complete_release():
            self._transition(State.IDLE, 'Release complete')
            timer_ref['timer'].cancel()

        timer_ref['timer'] = self.create_timer(1.0, _complete_release)

    def _on_emg_confidence(self, msg: Float32):
        self._latest_confidence = msg.data

    def _on_grasp_type(self, msg: Int32):
        self._grasp_type = msg.data

    def _on_object_cloud(self, msg: PointCloud2):
        if self._state != State.SEGMENTING:
            return
        if msg.width * msg.height == 0:
            return
        if self._grasp_type == 0:
            self.get_logger().warn(
                'Object cloud received but grasp_type is 0 — proceeding anyway')
        self._transition(State.PLANNING, 'Segmentation complete: object cloud received')
        self._request_preshaping()

    def _request_preshaping(self):
        """Call the grasp preshaping compute service."""
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
