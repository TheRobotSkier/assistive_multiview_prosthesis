#!/usr/bin/env python3
"""Grasp Proximity Controller Node.

Subscribes to planner output topics and the current hand position, then applies
proximity-based logic before issuing preshape/close/reset commands via the
hand_control_interface_node:

  - FAR:  command partial planned closure and wrist pose via preshape messages.
  - NEAR: trigger grasp close via the hand control interface.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import List, Optional

import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, Float64MultiArray, Int32

FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_COUNT = 3

_PKG_SHARE = "/prosthesis_ws/install/grasp_preshaping/share/grasp_preshaping"
_CONFIG_CANDIDATES = [
    Path(_PKG_SHARE) / ".." / ".." / ".." / "config" / "prosthesis_config.yaml",
    Path(__file__).resolve().parents[3] / "config" / "prosthesis_config.yaml",
]


class GraspProximityControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_proximity_controller")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("proximity_enter_threshold_m", 0.08)
        self.declare_parameter("proximity_exit_threshold_m", 0.10)
        self.declare_parameter("partial_closure_factor", 0.3)
        self.declare_parameter("min_closure_amount", 0.1)
        self.declare_parameter("wrist_accel_deg_s2", 180.0)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("release_duration_s", 2.0)
        self.declare_parameter("config_file", "")

        self._enter_thresh = float(self.get_parameter("proximity_enter_threshold_m").value)
        self._exit_thresh = float(self.get_parameter("proximity_exit_threshold_m").value)
        self._partial_factor = float(self.get_parameter("partial_closure_factor").value)
        self._min_closure = float(self.get_parameter("min_closure_amount").value)
        self._wrist_accel = float(self.get_parameter("wrist_accel_deg_s2").value)
        rate = float(self.get_parameter("control_rate_hz").value)
        self._release_duration_s = float(self.get_parameter("release_duration_s").value)

        # ── Config ─────────────────────────────────────────────────────────────
        config = self._load_config()
        force_cfg = config.get("force", {})
        self._force_enabled = bool(force_cfg.get("enabled", False))

        # ── State ─────────────────────────────────────────────────────────────
        self._planned_closures: Optional[List[float]] = None
        self._planned_wrist_deg: Optional[float] = None
        self._planned_hand_frame: Optional[Pose] = None

        self._buf_closures: Optional[List[float]] = None
        self._buf_wrist_deg: Optional[float] = None
        self._buf_hand_frame: Optional[Pose] = None

        self._current_hand_pose: Optional[PoseStamped] = None
        self._is_near: bool = False

        self._last_wrist_deg: float = 0.0

        self._pipeline_state: int = 0  # State.IDLE
        self._release_start_time: Optional[rclpy.time.Time] = None
        self._joint_positions: List[float] = [0.0] * FINGER_COUNT

        # ── Subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray,
            "/grasp_preshaping/target_finger_closures",
            self._on_planned_closures,
            10,
        )
        self.create_subscription(
            Float64,
            "/grasp_preshaping/wrist_pose",
            self._on_planned_wrist_rotation,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/grasp_preshaping/target_hand_pose",
            self._on_planned_hand_frame,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/hand_pose",
            self._on_current_hand_pose,
            10,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            10,
        )
        self.create_subscription(
            Int32,
            "/pipeline/state",
            self._on_pipeline_state,
            10,
        )

        # ── Publishers ─────────────────────────────────────────────────────────
        self._preshape_pub = self.create_publisher(Float64MultiArray, "/hand_control/preshape", 10)
        self._close_pub = self.create_publisher(Empty, "/hand_control/close", 10)
        self._reset_pub = self.create_publisher(Empty, "/hand_control/reset", 10)

        # ── Control timer ──────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f"GraspProximityController started — "
            f"enter_thresh={self._enter_thresh:.3f} m, "
            f"exit_thresh={self._exit_thresh:.3f} m, "
            f"partial_factor={self._partial_factor}, "
            f"force_enabled={self._force_enabled}"
        )

    # ── Config loading ─────────────────────────────────────────────────────

    def _load_config(self) -> dict:
        config_path = Path(self.get_parameter("config_file").value or "")
        if config_path.name and config_path.exists():
            self.get_logger().info(f"Loading config from {config_path}")
            return yaml.safe_load(config_path.read_text()) or {}

        for candidate in _CONFIG_CANDIDATES:
            if candidate.exists():
                self.get_logger().info(f"Loading config from {candidate}")
                return yaml.safe_load(candidate.read_text()) or {}

        self.get_logger().warn("No prosthesis_config.yaml found; using defaults.")
        return {}

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _on_planned_closures(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < FINGER_COUNT:
            self.get_logger().warn(
                f"planned closures expected >= {FINGER_COUNT} values, got {len(msg.data)}"
            )
            return
        self._buf_closures = list(msg.data[:FINGER_COUNT])
        self._try_commit_plan()

    def _on_planned_wrist_rotation(self, msg: Float64) -> None:
        self._buf_wrist_deg = msg.data
        self._try_commit_plan()

    def _on_planned_hand_frame(self, msg: PoseStamped) -> None:
        self._buf_hand_frame = msg.pose
        self._try_commit_plan()

    def _on_current_hand_pose(self, msg: PoseStamped) -> None:
        self._current_hand_pose = msg

    def _on_joint_states(self, msg: JointState) -> None:
        try:
            indices = [msg.name.index(jname) for jname in FINGER_JOINTS]
        except ValueError:
            return

        for i, idx in enumerate(indices):
            self._joint_positions[i] = float(msg.position[idx])

    def _on_pipeline_state(self, msg: Int32) -> None:
        new_state = msg.data
        if new_state == self._pipeline_state:
            return
        self.get_logger().info(
            f"Pipeline state changed: {self._pipeline_state} -> {new_state}"
        )
        self._pipeline_state = new_state

        if new_state == 6:  # RELEASING
            self._handle_release_start()

    def _handle_release_start(self) -> None:
        self._reset_pub.publish(Empty())

        self._planned_closures = None
        self._planned_wrist_deg = None
        self._planned_hand_frame = None
        self._is_near = False

        self._release_start_time = self.get_clock().now()
        self.get_logger().info("Release started — commanding fingers open via control interface")

    def _try_commit_plan(self) -> None:
        if (
            self._buf_closures is not None
            and self._buf_wrist_deg is not None
            and self._buf_hand_frame is not None
        ):
            self._planned_closures = self._buf_closures
            self._planned_wrist_deg = self._buf_wrist_deg
            self._planned_hand_frame = self._buf_hand_frame
            self._buf_closures = None
            self._buf_wrist_deg = None
            self._buf_hand_frame = None
            self._is_near = False
            self.get_logger().info(
                f"New plan committed — closures={self._planned_closures}, "
                f"wrist={self._planned_wrist_deg:.1f} deg"
            )
            self._publish_wrist_command(self._planned_wrist_deg)
            approach = [self._partial_factor * c for c in self._planned_closures]
            self._publish_joint_commands(approach[0], approach[1], approach[2])

    # ── Control loop ───────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        if self._pipeline_state == 6:  # RELEASING
            if self._release_start_time is not None:
                elapsed = (
                    self.get_clock().now() - self._release_start_time
                ).nanoseconds / 1e9
                if elapsed < self._release_duration_s:
                    self._publish_joint_commands(0.0, 0.0, 0.0)
                else:
                    self.get_logger().info("Release duration expired")
                    self._release_start_time = None
            return

        if (
            self._planned_closures is None
            or self._planned_wrist_deg is None
            or self._planned_hand_frame is None
            or self._current_hand_pose is None
        ):
            return

        dist = self._euclidean_distance(self._current_hand_pose, self._planned_hand_frame)

        # Hysteretic near/far transitions
        if self._is_near:
            if dist > self._exit_thresh:
                self._is_near = False
                self.get_logger().info(
                    f"Left near zone (dist={dist:.3f} > exit={self._exit_thresh:.3f})"
                )
        else:
            if dist < self._enter_thresh:
                self._is_near = True
                self.get_logger().info(
                    f"Entered near zone (dist={dist:.3f} < enter={self._enter_thresh:.3f})"
                )

        thumb, index, mrl = self._planned_closures

        if self._is_near:
            if self._force_enabled:
                self._control_force_closure(thumb, index, mrl)
            else:
                self._publish_joint_commands(thumb, index, mrl)
        else:
            self.get_logger().debug(f"FAR (dist={dist:.3f}): partial closure + wrist")
            self._publish_wrist_command(self._planned_wrist_deg)
            self._publish_joint_commands(
                self._partial_factor * thumb,
                self._partial_factor * index,
                self._partial_factor * mrl,
            )

    def _control_force_closure(self, thumb: float, index: float, mrl: float) -> None:
        self._close_pub.publish(Empty())
        self.get_logger().info("Triggered grasp close via control interface")

    def _stop_all_fingers(self) -> None:
        self._reset_pub.publish(Empty())

    # ── Helpers ────────────────────────────────────────────────────────────

    @staticmethod
    def _euclidean_distance(a: PoseStamped, b: Pose) -> float:
        dx = a.pose.position.x - b.position.x
        dy = a.pose.position.y - b.position.y
        dz = a.pose.position.z - b.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _publish_joint_commands(self, thumb: float, index: float, mrl: float) -> None:
        msg = Float64MultiArray()
        msg.data = [self._last_wrist_deg, thumb, index, mrl]
        self._preshape_pub.publish(msg)

    def _publish_velocity_commands(self, thumb_vel: float, index_vel: float, mrl_vel: float) -> None:
        if thumb_vel != 0.0 or index_vel != 0.0 or mrl_vel != 0.0:
            self._close_pub.publish(Empty())

    def _publish_wrist_command(self, target_deg: float) -> None:
        self._last_wrist_deg = target_deg


def main(args: Optional[list[str]] = None) -> None:
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


if __name__ == "__main__":
    main()
