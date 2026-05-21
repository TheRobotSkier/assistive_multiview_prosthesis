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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int32

try:
    from tf2_ros import Buffer, TransformListener
    import tf2_geometry_msgs  # noqa: F401
    _HAS_TF2 = True
except ImportError:
    _HAS_TF2 = False


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

        # ── State ─────────────────────────────────────────────────────────────
        # Pipeline state — used to gate commands during GRASPING/HOLDING
        # when the force controller has taken over joint commands.
        self._pipeline_state: int = 0  # Default IDLE

        # Planned outputs from the preshaping bridge (set atomically when all arrive)
        self._planned_closures: list[float] | None = None    # [thumb, index, mrl]
        self._planned_wrist_deg: float | None = None         # scalar wrist rotation (deg)
        self._planned_hand_frame: PoseStamped | None = None  # hand pose in world frame

        # Intermediate buffers — filled by individual topic callbacks
        self._buf_closures: list[float] | None = None
        self._buf_wrist_deg: float | None = None
        self._buf_hand_frame: PoseStamped | None = None

        # Current hand pose
        self._current_hand_pose: PoseStamped | None = None

        # Hysteresis state: True = currently in "near" mode
        self._is_near: bool = False
        self._near_zone_published: bool = False

        # TF2
        self._tf_buffer = None
        self._tf_listener = None
        if _HAS_TF2:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

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

        # Near-zone notification (latched, depth=1) for pipeline_manager APPROACHING->GRASPING
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
        self._buf_hand_frame = msg
        self._try_commit_plan()

    def _on_current_hand_pose(self, msg: PoseStamped) -> None:
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
            self._near_zone_published = False
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
        # Do not publish joint commands during GRASPING/HOLDING —
        # the force controller owns the joint topics in those states.
        if self._pipeline_state in (4, 5):  # GRASPING, HOLDING
            return

        if (self._planned_closures is None
                or self._planned_wrist_deg is None
                or self._planned_hand_frame is None
                or self._current_hand_pose is None):
            return

        dist = self._euclidean_distance(self._current_hand_pose, self._planned_hand_frame)

        # Hysteretic state transitions
        if self._is_near:
            if dist > self._exit_thresh:
                self._is_near = False
                self.get_logger().info(
                    f'Left near zone (dist={dist:.3f} m > exit={self._exit_thresh:.3f} m)'
                )
        else:
            if dist < self._enter_thresh:
                self._is_near = True
                self.get_logger().info(
                    f'Entered near zone (dist={dist:.3f} m < enter={self._enter_thresh:.3f} m)'
                )
                if not self._near_zone_published:
                    self._near_zone_pub.publish(Bool(data=True))
                    self._near_zone_published = True

        thumb, index, mrl = self._planned_closures

        if self._is_near:
            self.get_logger().debug(f'NEAR mode (dist={dist:.3f} m): full closure')
            self._publish_joint_commands(thumb, index, mrl)
        else:
            self.get_logger().debug(f'FAR mode (dist={dist:.3f} m): partial closure + wrist')
            self._publish_wrist_command(self._planned_wrist_deg)
            self._publish_joint_commands(
                self._partial_factor * thumb,
                self._partial_factor * index,
                self._partial_factor * mrl,
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _euclidean_distance(self, a: PoseStamped, b: PoseStamped) -> float:
        """Compute Euclidean distance between two poses in a common frame.

        Uses TF2 to transform both poses into the same frame when possible.
        Falls back to direct comparison with a WARN log if TF2 is unavailable
        or the transform fails.
        """
        frame_a = a.header.frame_id
        frame_b = b.header.frame_id

        if frame_a and frame_b and frame_a != frame_b and _HAS_TF2 and self._tf_buffer is not None:
            try:
                a_in_b = self._tf_buffer.transform(
                    a, frame_b,
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                dx = a_in_b.pose.position.x - b.pose.position.x
                dy = a_in_b.pose.position.y - b.pose.position.y
                dz = a_in_b.pose.position.z - b.pose.position.z
                return math.sqrt(dx * dx + dy * dy + dz * dz)
            except Exception as exc:
                self.get_logger().warn(
                    f"TF transform from '{frame_a}' to '{frame_b}' failed: {exc}. "
                    "Falling back to raw Euclidean distance."
                )
        elif frame_a and frame_b and frame_a != frame_b:
            self.get_logger().warn(
                f"Frame mismatch ('{frame_a}' vs '{frame_b}') but TF2 unavailable. "
                "Falling back to raw Euclidean distance."
            )

        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dz = a.pose.position.z - b.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

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
