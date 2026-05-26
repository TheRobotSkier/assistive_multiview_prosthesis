#!/usr/bin/env python3
"""Grasp Proximity Controller Node.

Subscribes to planner output topics and the current hand position, then applies
proximity-based logic before issuing joint and wrist commands:

  - If the current hand position is within the "near" threshold of the planned
    final hand-frame position → command joints to the full planned closure.
  - Otherwise → command the wrist to the planned rotation and command joints to
    (partial_closure_factor × planned_closure).

"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int32

try:
    from scipy.spatial.transform import Rotation as R
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


def _quat_rotate(q: list[float], v: list[float]) -> list[float]:
    """Rotate vector *v* by unit quaternion *q* = [x, y, z, w].

    Uses the identity:  v' = v + 2·w·(u × v) + 2·(u × (u × v))
    where u = (x, y, z) is the vector part of the quaternion.
    """
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # u × v
    uvx = qy * vz - qz * vy
    uvy = qz * vx - qx * vz
    uvz = qx * vy - qy * vx
    # u × (u × v)
    uuvx = qy * uvz - qz * uvy
    uuvy = qz * uvx - qx * uvz
    uuvz = qx * uvy - qy * uvx
    return [
        vx + 2.0 * qw * uvx + 2.0 * uuvx,
        vy + 2.0 * qw * uvy + 2.0 * uuvy,
        vz + 2.0 * qw * uvz + 2.0 * uuvz,
    ]


class GraspProximityControllerNode(Node):
    # Pipeline state constants (must match pipeline_manager_node.py)
    _IDLE = 0
    _TWISTING = 1
    _SEGMENTING = 2
    _PLANNING = 3
    _APPROACHING = 4
    _GRASPING = 5
    _HOLDING = 6
    _VOLITIONAL = 7
    _RELEASING = 8

    def __init__(self) -> None:
        super().__init__('grasp_proximity_controller')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('proximity_enter_threshold_m', 0.08)
        self.declare_parameter('proximity_exit_threshold_m', 0.20)
        self.declare_parameter('partial_closure_factor', 0.3)
        self.declare_parameter('min_closure_amount', 0.1)
        self.declare_parameter('wrist_accel_deg_s2', 180.0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('grasp_contact_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('max_valid_proximity_distance_m', 0.75)
        self.declare_parameter('max_proximity_step_m', 0.25)
        self.declare_parameter('wrist_min_deg', 0.0)
        self.declare_parameter('wrist_max_deg', 300.0)
        self.declare_parameter('wrist_neutral_deg', 140.0)
        self.declare_parameter('max_wrist_delta_deg', 90.0)
        self.declare_parameter('command_repeat_period_s', 1.0)
        self.declare_parameter('command_epsilon', 0.005)
        self.declare_parameter('near_enter_consecutive_samples', 3)
        self.declare_parameter('near_exit_consecutive_samples', 3)

        # Topic name parameters
        self.declare_parameter('target_closures_topic', '/grasp_preshaping/target_finger_closures')
        self.declare_parameter('wrist_pose_topic', '/grasp_preshaping/wrist_pose')
        self.declare_parameter('target_hand_pose_topic', '/grasp_preshaping/target_hand_pose')
        self.declare_parameter('hand_pose_topic', '/hand_pose')
        self.declare_parameter('pipeline_state_topic', '/pipeline/state')
        self.declare_parameter('thumb_cmd_topic', '/thumb_pos_ff_controller/commands')
        self.declare_parameter('index_cmd_topic', '/index_pos_ff_controller/commands')
        self.declare_parameter('mrl_cmd_topic', '/mrl_pos_ff_controller/commands')
        self.declare_parameter('wrist_cmd_topic', '/wrist/set_position')

        self._enter_thresh = self.get_parameter('proximity_enter_threshold_m').value
        self._exit_thresh = self.get_parameter('proximity_exit_threshold_m').value
        self._partial_factor = self.get_parameter('partial_closure_factor').value
        self._min_closure = self.get_parameter('min_closure_amount').value
        self._wrist_accel = self.get_parameter('wrist_accel_deg_s2').value
        rate = self.get_parameter('control_rate_hz').value

        # Offset from tracked pose origin (camera) to grasp contact point
        # (fingertips), expressed in the pose's local frame.  Applied by
        # rotating by pose orientation before computing distance.
        # Same value as twist_propagation's propagation_origin_offset.
        self._grasp_contact_offset = self.get_parameter('grasp_contact_offset').value
        if self._grasp_contact_offset and any(v != 0.0 for v in self._grasp_contact_offset):
            self.get_logger().info(
                f'Grasp contact offset enabled: {self._grasp_contact_offset}')

        self._max_valid_dist = self.get_parameter('max_valid_proximity_distance_m').value
        self._max_prox_step = self.get_parameter('max_proximity_step_m').value
        self._wrist_min_deg = self.get_parameter('wrist_min_deg').value
        self._wrist_max_deg = self.get_parameter('wrist_max_deg').value
        self._wrist_neutral_deg = self.get_parameter('wrist_neutral_deg').value
        self._max_wrist_delta_deg = self.get_parameter('max_wrist_delta_deg').value
        self._command_repeat_period_s = self.get_parameter('command_repeat_period_s').value
        self._command_epsilon = self.get_parameter('command_epsilon').value
        self._near_enter_consecutive_samples = self.get_parameter('near_enter_consecutive_samples').value
        self._near_exit_consecutive_samples = self.get_parameter('near_exit_consecutive_samples').value

        # ── State ─────────────────────────────────────────────────────────────
        # Pipeline state — used to gate commands during GRASPING/HOLDING
        # when the force controller has taken over joint commands.
        self._pipeline_state: int = 0  # Default IDLE
        self._prev_pipeline_state: int = 0

        # Planned outputs from the preshaping bridge (set atomically when all arrive)
        self._planned_closures: list[float] | None = None    # [thumb, index, mrl]
        self._planned_wrist_deg: float | None = None         # scalar wrist rotation (deg)
        self._planned_hand_frame: Pose | None = None         # hand pose in world frame

        # Intermediate buffers — filled by individual topic callbacks
        self._buf_closures: list[float] | None = None
        self._buf_wrist_deg: float | None = None
        self._buf_hand_frame: Pose | None = None

        # Current hand pose
        self._current_hand_pose: PoseStamped | None = None

        # Hysteresis state: True = currently in "near" mode
        self._is_near: bool = False

        # Proximity distance sanity
        self._last_valid_proximity_distance: float | None = None

        # Wrist state from /wrist/state
        self._current_wrist_deg: float | None = None

        # Command caching
        self._last_published_thumb: float | None = None
        self._last_published_index: float | None = None
        self._last_published_mrl: float | None = None
        self._last_published_wrist: float | None = None
        self._last_command_publish_time: float = 0.0
        self._last_mode: str | None = None

        # Near-zone debounce
        self._near_enter_count: int = 0
        self._near_exit_count: int = 0

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter('target_closures_topic').value,
            self._on_planned_closures,
            10,
        )

        self.create_subscription(
            Float64,
            self.get_parameter('wrist_pose_topic').value,
            self._on_planned_wrist_rotation,
            10,
        )

        self.create_subscription(
            PoseStamped,
            self.get_parameter('target_hand_pose_topic').value,
            self._on_planned_hand_frame,
            10,
        )

        self.create_subscription(
            PoseStamped,
            self.get_parameter('hand_pose_topic').value,
            self._on_current_hand_pose,
            10,
        )

        # Pipeline state — gate proximity commands during force control phases.
        # During GRASPING (4) and HOLDING (5), the force controller owns
        # the joint command topics; proximity controller must not publish.
        self.create_subscription(
            Int32,
            self.get_parameter('pipeline_state_topic').value,
            self._on_pipeline_state,
            10,
        )

        self.create_subscription(
            Float64MultiArray,
            '/wrist/state',
            self._on_wrist_state,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._thumb_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter('thumb_cmd_topic').value, 10)
        self._index_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter('index_cmd_topic').value, 10)
        self._mrl_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter('mrl_cmd_topic').value, 10)
        self._wrist_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter('wrist_cmd_topic').value, 10)
        self._near_zone_pub = self.create_publisher(
            Bool, '/proximity/near_zone_entered',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # ── Control timer ─────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f'GraspProximityController started — '
            f'enter_thresh={self._enter_thresh:.3f} m, '
            f'exit_thresh={self._exit_thresh:.3f} m, '
            f'partial_factor={self._partial_factor}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_planned_closures(self, msg: Float64MultiArray) -> None:
        """New plan signal: cache closures and attempt to commit the plan."""
        if len(msg.data) < 3:
            self.get_logger().warn(
                f'planned_joint_closures expected ≥3 values, got {len(msg.data)} — ignoring')
            return
        self._buf_closures = list(msg.data[:3])
        self._try_commit_plan()

    def _on_planned_wrist_rotation(self, msg: Float64) -> None:
        self._buf_wrist_deg = msg.data
        self._try_commit_plan()

    def _on_planned_hand_frame(self, msg: PoseStamped) -> None:
        self._buf_hand_frame = msg.pose
        self._try_commit_plan()

    def _on_current_hand_pose(self, msg: PoseStamped) -> None:
        if self._current_hand_pose is None:
            self.get_logger().info('Received first hand pose — control loop now active')
        self._current_hand_pose = msg

    def _on_pipeline_state(self, msg: Int32) -> None:
        self._prev_pipeline_state = self._pipeline_state
        self._pipeline_state = msg.data
        prev = self._prev_pipeline_state
        new = self._pipeline_state
        if prev == self._APPROACHING and new != self._APPROACHING:
            self._clear_plan('pipeline leaving APPROACHING')
        elif new in (self._IDLE, self._TWISTING, self._SEGMENTING,
                     self._PLANNING, self._RELEASING):
            state_names = {0: 'IDLE', 1: 'TWISTING', 2: 'SEGMENTING',
                           3: 'PLANNING', 8: 'RELEASING'}
            self._clear_plan(f'pipeline entering {state_names.get(new, str(new))}')

    def _on_wrist_state(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 2:
            self._current_wrist_deg = float(msg.data[0])

    def _clear_plan(self, reason: str) -> None:
        if (self._planned_closures is None
                and self._planned_wrist_deg is None
                and self._planned_hand_frame is None
                and self._buf_closures is None
                and self._buf_wrist_deg is None
                and self._buf_hand_frame is None
                and not self._is_near):
            return
        self._planned_closures = None
        self._planned_wrist_deg = None
        self._planned_hand_frame = None
        self._buf_closures = None
        self._buf_wrist_deg = None
        self._buf_hand_frame = None
        was_near = self._is_near
        self._is_near = False
        self._near_enter_count = 0
        self._near_exit_count = 0
        if was_near:
            self._near_zone_pub.publish(Bool(data=False))
        self.get_logger().info(f'Plan cleared: {reason}')

    def _try_commit_plan(self) -> None:
        """Atomically commit buffered plan when all three parts have arrived."""
        if (self._buf_closures is not None
                and self._buf_wrist_deg is not None
                and self._buf_hand_frame is not None):
            self._planned_closures = self._buf_closures
            self._planned_hand_frame = self._buf_hand_frame

            # Compute safe absolute wrist target from signed delta
            planned_delta = self._buf_wrist_deg
            planned_delta = max(-self._max_wrist_delta_deg,
                                min(self._max_wrist_delta_deg, planned_delta))
            if self._current_wrist_deg is not None:
                target = self._current_wrist_deg + planned_delta
            else:
                target = self._wrist_neutral_deg + planned_delta
            target = max(self._wrist_min_deg, min(self._wrist_max_deg, target))
            self._planned_wrist_deg = target

            # Clear buffers so the next plan starts fresh
            self._buf_closures = None
            self._buf_wrist_deg = None
            self._buf_hand_frame = None
            # Reset state so the new plan re-evaluates proximity
            self._is_near = False
            self._near_enter_count = 0
            self._near_exit_count = 0
            # Reset caching so new mode forces a publish
            self._last_mode = None
            self.get_logger().info(
                f'New plan committed — closures={self._planned_closures}, '
                f'wrist={self._planned_wrist_deg:.1f}\u00b0'
            )
    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        if self._pipeline_state != self._APPROACHING:
            return

        if (self._planned_closures is None
                or self._planned_wrist_deg is None
                or self._planned_hand_frame is None
                or self._current_hand_pose is None):
            missing = []
            if self._planned_closures is None:
                missing.append('closures')
            if self._planned_wrist_deg is None:
                missing.append('wrist')
            if self._planned_hand_frame is None:
                missing.append('hand_frame')
            if self._current_hand_pose is None:
                missing.append('current_pose')
            self.get_logger().debug(
                f'Control loop waiting for: {missing}',
                throttle_duration_sec=5.0)
            return

        dist = self._compute_proximity_distance(
            self._current_hand_pose, self._planned_hand_frame)

        # Sanity guards
        if not math.isfinite(dist):
            self.get_logger().warn(f'Non-finite proximity distance: {dist}, clearing plan')
            self._clear_plan('non-finite proximity distance')
            return

        if dist > self._max_valid_dist:
            self.get_logger().warn(
                f'Proximity distance {dist:.1f}m exceeds max {self._max_valid_dist}m, '
                f'clearing plan')
            self._clear_plan('proximity distance exceeds limit')
            return

        if self._last_valid_proximity_distance is not None:
            jump = abs(dist - self._last_valid_proximity_distance)
            if jump > self._max_prox_step:
                self.get_logger().warn(
                    f'Proximity distance jumped {jump:.2f}m '
                    f'(> {self._max_prox_step}m), clearing plan')
                self._clear_plan('proximity distance jump too large')
                return

        self._last_valid_proximity_distance = dist

        # Debounced hysteretic state transitions
        if self._is_near:
            if dist > self._exit_thresh:
                self._near_exit_count += 1
                if self._near_exit_count >= self._near_exit_consecutive_samples:
                    self._is_near = False
                    self._near_exit_count = 0
                    self.get_logger().info(
                        f'Left near zone (dist={dist:.3f}m > '
                        f'exit={self._exit_thresh:.3f}m)')
                    self._near_zone_pub.publish(Bool(data=False))
            else:
                self._near_exit_count = 0
        else:
            if dist < self._enter_thresh:
                self._near_enter_count += 1
                if self._near_enter_count >= self._near_enter_consecutive_samples:
                    self._is_near = True
                    self._near_enter_count = 0
                    self.get_logger().info(
                        f'Entered near zone (dist={dist:.3f}m < '
                        f'enter={self._enter_thresh:.3f}m)')
                    self._near_zone_pub.publish(Bool(data=True))
            else:
                self._near_enter_count = 0

        thumb, index, mrl = self._planned_closures
        mode = "NEAR" if self._is_near else "FAR"

        if self._is_near:
            self.get_logger().info(
                f'NEAR mode (dist={dist:.3f} m): full closure',
                throttle_duration_sec=1.0)
            self._publish_joint_commands(thumb, index, mrl, mode)
        else:
            self.get_logger().info(
                f'FAR mode (dist={dist:.3f} m): partial closure + wrist',
                throttle_duration_sec=1.0)
            self._publish_wrist_command(self._planned_wrist_deg, mode)
            self._publish_joint_commands(
                self._partial_factor * thumb,
                self._partial_factor * index,
                self._partial_factor * mrl,
                mode,
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _compute_proximity_distance(self, current: PoseStamped, planned: Pose) -> float:
        """Compute distance between current and planned hand positions.

        Both positions are shifted from the tracked pose origin (camera) to
        the grasp contact point (fingertips) using ``_grasp_contact_offset``
        before computing the Euclidean distance.  The offset is expressed in
        each pose's local frame and rotated by that pose's orientation.
        """
        cur_pos = self._apply_offset(current.pose, self._grasp_contact_offset)
        plan_pos = self._apply_offset(planned, self._grasp_contact_offset)
        dx = cur_pos[0] - plan_pos[0]
        dy = cur_pos[1] - plan_pos[1]
        dz = cur_pos[2] - plan_pos[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def _apply_offset(pose: Pose, offset: list[float]) -> tuple[float, float, float]:
        """Shift a pose position by *offset* expressed in the pose's local frame.

        Returns the world-frame position of the offset point.
        """
        if not offset or all(v == 0.0 for v in offset):
            return (pose.position.x, pose.position.y, pose.position.z)

        q = pose.orientation
        if _HAS_SCIPY:
            rot = R.from_quat([q.x, q.y, q.z, q.w])
            rotated = rot.apply(offset)
        else:
            # Fallback: manual rotation using quaternion
            rotated = _quat_rotate([q.x, q.y, q.z, q.w], offset)

        return (
            pose.position.x + rotated[0],
            pose.position.y + rotated[1],
            pose.position.z + rotated[2],
        )

    def _floor_closure(self, planned: float) -> float:
        """Apply min_closure_amount only to non-zero planned values.

        Intentional zeros (e.g. MRL for pinch/lateral grasps) are preserved.
        """
        if planned <= 0.0:
            return 0.0
        return max(planned, self._min_closure)

    def _publish_joint_commands(self, thumb: float, index: float, mrl: float,
                                 mode: str) -> None:
        t_thumb = self._floor_closure(thumb)
        t_index = self._floor_closure(index)
        t_mrl = self._floor_closure(mrl)

        now = time.time()
        should_publish = False
        if mode != self._last_mode:
            should_publish = True
        elif self._last_published_thumb is None:
            should_publish = True
        elif (abs(t_thumb - self._last_published_thumb) > self._command_epsilon
              or abs(t_index - self._last_published_index) > self._command_epsilon
              or abs(t_mrl - self._last_published_mrl) > self._command_epsilon):
            should_publish = True
        elif now - self._last_command_publish_time >= self._command_repeat_period_s:
            should_publish = True

        if not should_publish:
            return

        msg = Float64MultiArray()
        msg.data = [t_thumb]
        self._thumb_pub.publish(msg)
        msg.data = [t_index]
        self._index_pub.publish(msg)
        msg.data = [t_mrl]
        self._mrl_pub.publish(msg)

        self._last_published_thumb = t_thumb
        self._last_published_index = t_index
        self._last_published_mrl = t_mrl
        self._last_command_publish_time = now
        self._last_mode = mode

    def _publish_wrist_command(self, target_deg: float, mode: str) -> None:
        target_deg = max(self._wrist_min_deg, min(self._wrist_max_deg, target_deg))

        now = time.time()
        should_publish = False
        if mode != self._last_mode:
            should_publish = True
        elif self._last_published_wrist is None:
            should_publish = True
        elif abs(target_deg - self._last_published_wrist) > self._command_epsilon:
            should_publish = True
        elif now - self._last_command_publish_time >= self._command_repeat_period_s:
            should_publish = True

        if not should_publish:
            return

        msg = Float64MultiArray()
        msg.data = [target_deg, self._wrist_accel]
        self._wrist_pub.publish(msg)

        self._last_published_wrist = target_deg
        self._last_command_publish_time = now
        self._last_mode = mode


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GraspProximityControllerNode()
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
