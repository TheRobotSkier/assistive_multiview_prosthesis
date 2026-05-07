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
        self.declare_parameter('grasp_gestures', [GESTURE_POWER, GESTURE_PINCH, GESTURE_POINT])
        self.declare_parameter('release_gesture', GESTURE_OPEN)
        self.declare_parameter('state_publish_rate_hz', 5.0)

        self._confidence_threshold = self.get_parameter('confidence_threshold').value
        self._grasp_gestures = self.get_parameter('grasp_gestures').value
        self._release_gesture = self.get_parameter('release_gesture').value
        rate = self.get_parameter('state_publish_rate_hz').value

        # ── State ─────────────────────────────────────────────────────────
        self._state = State.IDLE
        self._history: list[Transition] = []
        self._grasp_type: int = 0

        # ── Publishers ────────────────────────────────────────────────────
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(Int32, '/pipeline/state', latched)
        self._state_name_pub = self.create_publisher(String, '/pipeline/state_name', latched)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            Int32, '/emg/gesture_label', self._on_emg_gesture, 10)
        self.create_subscription(
            Float32, '/emg/confidence', self._on_emg_confidence, 10)  # noqa: F821
        self.create_subscription(
            Int32, '/grasp_preshaping/grasp_type', self._on_grasp_type, 10)
        self.create_subscription(
            PointCloud2, '/segmentation/object_cloud', self._on_object_cloud, 10)

        # ── Service clients ───────────────────────────────────────────────
        self._compute_client = self.create_client(
            Trigger, '/grasp_preshaping/compute_grasp')

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
            f'State: {old.name} -> {new.name} ({reason})')
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

    def _on_emg_gesture(self, msg: Int32):
        gesture = msg.data

        # Release gesture works from any active state
        if gesture == self._release_gesture:
            if self._state not in (State.IDLE, State.RELEASING):
                self._transition(State.RELEASING, 'EMG: OPEN')
                # Auto-transition to IDLE after a short delay
                self.create_timer(1.0, lambda: self._transition(State.IDLE, 'Release complete'),
                                  one_shot=True)  # type: ignore[arg-type]
            return

        # Grasp trigger gesture
        if gesture in self._grasp_gestures and self._state == State.IDLE:
            self._transition(State.SEGMENTING, f'EMG: gesture={gesture}')
            # In a full pipeline, segmentation completion triggers PLANNING.
            # For now, immediately call the compute service.
            self._request_preshaping()

    def _on_emg_confidence(self, msg):
        # Could be used for gesture validation
        pass

    def _on_grasp_type(self, msg: Int32):
        self._grasp_type = msg.data

    def _on_object_cloud(self, msg: PointCloud2):
        if self._state != State.SEGMENTING:
            return
        if msg.width * msg.height == 0:
            return
        if self._grasp_type == 0:
            self.get_logger().warn(
                'Object cloud received but no grasp type set — staying in SEGMENTING')
            return
        self._transition(State.PLANNING, 'Segmentation complete: object cloud received')

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
