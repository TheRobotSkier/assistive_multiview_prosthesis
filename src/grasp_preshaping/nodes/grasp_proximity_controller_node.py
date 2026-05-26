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
    def __init__(self) -> None:
        super().__init__('grasp_proximity_controller')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('proximity_enter_threshold_m', 0.08)
        self.declare_parameter('proximity_exit_threshold_m', 0.10)
        self.declare_parameter('partial_closure_factor', 0.3)
        self.declare_parameter('min_closure_amount', 0.1)
        self.declare_parameter('wrist_accel_deg_s2', 180.0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('grasp_contact_offset', [0.0, 0.0, 0.0])

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

        # ── State ─────────────────────────────────────────────────────────────
        # Pipeline state — used to gate commands during GRASPING/HOLDING
        # when the force controller has taken over joint commands.
        self._pipeline_state: int = 0  # Default IDLE

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
        self._pipeline_state = msg.data

    def _try_commit_plan(self) -> None:
        """Atomically commit buffered plan when all three parts have arrived."""
        if (self._buf_closures is not None
                and self._buf_wrist_deg is not None
                and self._buf_hand_frame is not None):
            self._planned_closures = self._buf_closures
            self._planned_wrist_deg = self._buf_wrist_deg
            self._planned_hand_frame = self._buf_hand_frame
            # Clear buffers so the next plan starts fresh
            self._buf_closures = None
            self._buf_wrist_deg = None
            self._buf_hand_frame = None
            # Reset hysteresis so the new plan re-evaluates proximity
            self._is_near = False
            self.get_logger().info(
                f'New plan committed — closures={self._planned_closures}, '
                f'wrist={self._planned_wrist_deg:.1f}\u00b0'
            )
            # Immediately send initial preshape (far mode)
            self._publish_wrist_command(self._planned_wrist_deg)
            self._publish_joint_commands(
                self._partial_factor * self._planned_closures[0],
                self._partial_factor * self._planned_closures[1],
                self._partial_factor * self._planned_closures[2],
            )
    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        # Only run the proximity control loop during APPROACHING (state 4).
        # In GRASPING/HOLDING/VOLITIONAL the force controller owns the joints.
        # In all other states (IDLE, TWISTING, etc.) there is no plan to execute.
        if self._pipeline_state != 4:  # not APPROACHING
            # Clear near zone signal when leaving APPROACHING (but not when
            # transitioning to GRASPING — the pipeline manager handles that).
            if self._is_near and self._pipeline_state not in (5,):
                self._is_near = False
                self._near_zone_pub.publish(Bool(data=False))
            return

        if (self._planned_closures is None
                or self._planned_wrist_deg is None
                or self._planned_hand_frame is None
                or self._current_hand_pose is None):
            # Throttled diagnostic: report which prerequisites are missing
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

        dist, (dx, dy, dz) = self._compute_proximity_distance(
            self._current_hand_pose, self._planned_hand_frame)

        # Hysteretic state transitions
        if self._is_near:
            if dist > self._exit_thresh:
                self._is_near = False
                self.get_logger().info(
                    f'Left near zone (dist={dist:.3f} m, '
                    f'offset=[{dx:+.3f}, {dy:+.3f}, {dz:+.3f}] m'
                    f' > exit={self._exit_thresh:.3f} m)'
                )
                self._near_zone_pub.publish(Bool(data=False))
        else:
            if dist < self._enter_thresh:
                self._is_near = True
                self.get_logger().info(
                    f'Entered near zone (dist={dist:.3f} m, '
                    f'offset=[{dx:+.3f}, {dy:+.3f}, {dz:+.3f}] m'
                    f' < enter={self._enter_thresh:.3f} m)'
                )
                self._near_zone_pub.publish(Bool(data=True))

        thumb, index, mrl = self._planned_closures

        if self._is_near:
            self.get_logger().info(
                f'NEAR mode (dist={dist:.3f} m, '
                f'offset=[{dx:+.3f}, {dy:+.3f}, {dz:+.3f}] m): '
                f'full closure',
                throttle_duration_sec=1.0)
            self._publish_joint_commands(thumb, index, mrl)
        else:
            self.get_logger().info(
                f'FAR mode (dist={dist:.3f} m, '
                f'offset=[{dx:+.3f}, {dy:+.3f}, {dz:+.3f}] m): '
                f'partial closure + wrist',
                throttle_duration_sec=1.0)
            self._publish_wrist_command(self._planned_wrist_deg)
            self._publish_joint_commands(
                self._partial_factor * thumb,
                self._partial_factor * index,
                self._partial_factor * mrl,
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _compute_proximity_distance(
        self, current: PoseStamped, planned: Pose,
    ) -> tuple[float, tuple[float, float, float]]:
        """Compute distance between current and planned hand positions.

        Both positions are shifted from the tracked pose origin (camera) to
        the grasp contact point (fingertips) using ``_grasp_contact_offset``
        before computing the Euclidean distance.  The offset is expressed in
        each pose's local frame and rotated by that pose's orientation.

        Returns:
            (euclidean_distance, (dx, dy, dz)) where dx/dy/dz are the
            signed per-axis offsets from planned to current grasp contact.
        """
        cur_pos = self._apply_offset(current.pose, self._grasp_contact_offset)
        plan_pos = self._apply_offset(planned, self._grasp_contact_offset)
        dx = cur_pos[0] - plan_pos[0]
        dy = cur_pos[1] - plan_pos[1]
        dz = cur_pos[2] - plan_pos[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz), (dx, dy, dz)

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

    def _publish_joint_commands(self, thumb: float, index: float, mrl: float) -> None:
        msg = Float64MultiArray()
        msg.data = [self._floor_closure(thumb)]
        self._thumb_pub.publish(msg)
        msg.data = [self._floor_closure(index)]
        self._index_pub.publish(msg)
        msg.data = [self._floor_closure(mrl)]
        self._mrl_pub.publish(msg)

    def _publish_wrist_command(self, target_deg: float) -> None:
        msg = Float64MultiArray()
        msg.data = [target_deg, self._wrist_accel]
        self._wrist_pub.publish(msg)


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
