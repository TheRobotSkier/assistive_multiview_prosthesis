#!/usr/bin/env python3
"""Grasp Proximity Controller Node with force-aware final closure.

Subscribes to planner output topics and the current hand position, then applies
proximity-based logic before issuing joint and wrist commands:

  - FAR:  command partial planned closure and wrist pose via position controllers.
  - NEAR: switch to velocity controllers, apply force-aware final closure.

Per-finger closure states (from force_aware_closure.py):
  open_loop, contact_seek, contacted, released, safety_stopped.
"""

from __future__ import annotations

import math
from collections import deque
from enum import Enum, auto
from pathlib import Path
from typing import List, Optional

import yaml

import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from force_aware_closure import (
    ClosureProfile,
    FingerState,
    ForceReading,
    compute_baseline,
    compute_closure_speed,
    step_finger_state,
)

FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_COUNT = 3

_PKG_SHARE = "/prosthesis_ws/install/grasp_preshaping/share/grasp_preshaping"
_CONFIG_CANDIDATES = [
    Path(_PKG_SHARE) / ".." / ".." / ".." / "config" / "prosthesis_config.yaml",
    Path(__file__).resolve().parents[3] / "config" / "prosthesis_config.yaml",
]

PROFILE_KEYS = [
    "max_closing_speed",
    "min_closing_speed",
    "decay_distance",
    "decay_exponent",
    "contact_force_threshold",
    "contact_force_spike_threshold",
    "max_extra_closure",
    "contact_hold_velocity",
    "final_closure_timeout_s",
]


def _load_profile(raw: dict) -> ClosureProfile:
    return ClosureProfile(**{k: float(raw[k]) for k in PROFILE_KEYS})


class _ClosurePhase(Enum):
    IDLE = auto()
    APPROACHING = auto()
    FINAL_CLOSURE = auto()


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
        self.declare_parameter("config_file", "")

        self._enter_thresh = float(self.get_parameter("proximity_enter_threshold_m").value)
        self._exit_thresh = float(self.get_parameter("proximity_exit_threshold_m").value)
        self._partial_factor = float(self.get_parameter("partial_closure_factor").value)
        self._min_closure = float(self.get_parameter("min_closure_amount").value)
        self._wrist_accel = float(self.get_parameter("wrist_accel_deg_s2").value)
        rate = float(self.get_parameter("control_rate_hz").value)

        # ── Config ─────────────────────────────────────────────────────────────
        config = self._load_config()
        force_cfg = config.get("force", {})
        self._force_enabled = bool(force_cfg.get("enabled", False))
        self._force_active_profile = str(force_cfg.get("active_profile", "soft"))
        self._force_profiles: dict[str, ClosureProfile] = {}
        profiles_raw = force_cfg.get("profiles", {})
        for name, raw in profiles_raw.items():
            self._force_profiles[name] = _load_profile(raw)

        self._force_baseline_window_s = float(force_cfg.get("baseline_window_s", 0.5))
        self._force_stale_timeout_s = float(force_cfg.get("stale_force_timeout_s", 0.5))
        self._force_reading_timeout_ms = int(force_cfg.get("force_reading_timeout_ms", 500))
        self._joint_states_topic = str(force_cfg.get("joint_states_topic", "/joint_states"))
        self._controller_manager_service = str(
            force_cfg.get("controller_manager_service", "/controller_manager/switch_controller")
        )

        topic_cfg = config.get("topics", {})
        controllers_cfg = config.get("controllers", {})

        self._pos_topics: List[str] = [
            str(topic_cfg.get("thumb_pos_cmd", "/thumb_pos_ff_controller/commands")),
            str(topic_cfg.get("index_pos_cmd", "/index_pos_ff_controller/commands")),
            str(topic_cfg.get("mrl_pos_cmd", "/mrl_pos_ff_controller/commands")),
        ]
        self._vel_topics: List[str] = [
            str(topic_cfg.get("thumb_vel_cmd", "/thumb_vel_ff_controller/commands")),
            str(topic_cfg.get("index_vel_cmd", "/index_vel_ff_controller/commands")),
            str(topic_cfg.get("mrl_vel_cmd", "/mrl_vel_ff_controller/commands")),
        ]
        self._finger_pos_controllers: List[str] = list(
            controllers_cfg.get("finger_position", ["thumb_pos_ff_controller", "index_pos_ff_controller", "mrl_pos_ff_controller"])
        )
        self._finger_vel_controllers: List[str] = list(
            controllers_cfg.get("finger_velocity", ["thumb_vel_ff_controller", "index_vel_ff_controller", "mrl_vel_ff_controller"])
        )

        # ── State ─────────────────────────────────────────────────────────────
        self._planned_closures: Optional[List[float]] = None
        self._planned_wrist_deg: Optional[float] = None
        self._planned_hand_frame: Optional[Pose] = None

        self._buf_closures: Optional[List[float]] = None
        self._buf_wrist_deg: Optional[float] = None
        self._buf_hand_frame: Optional[Pose] = None

        self._current_hand_pose: Optional[PoseStamped] = None
        self._is_near: bool = False

        # Force-aware closure state
        self._closure_phase = _ClosurePhase.IDLE
        self._closure_start_stamp: Optional[rclpy.time.Time] = None
        self._finger_states: List[FingerState] = [FingerState.RELEASED] * FINGER_COUNT
        self._joint_positions: List[float] = [0.0] * FINGER_COUNT
        self._joint_efforts: List[float] = [0.0] * FINGER_COUNT
        self._force_history: List[deque] = [
            deque(maxlen=50) for _ in range(FINGER_COUNT)
        ]
        self._last_force_time: Optional[rclpy.time.Time] = None

        # Controller switch state
        self._controllers_active: Optional[str] = None  # "position" or "velocity"
        self._switch_pending: bool = False

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
            self._joint_states_topic,
            self._on_joint_states,
            10,
        )

        # ── Publishers ─────────────────────────────────────────────────────────
        self._pos_pubs = [
            self.create_publisher(Float64MultiArray, t, 10) for t in self._pos_topics
        ]
        self._vel_pubs = [
            self.create_publisher(Float64MultiArray, t, 10) for t in self._vel_topics
        ]
        self._wrist_pub = self.create_publisher(Float64MultiArray, "/wrist/set_position", 10)

        # ── Controller switch client ───────────────────────────────────────────
        if self._force_enabled:
            self._switch_client = self.create_client(
                SwitchController, self._controller_manager_service
            )

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
        now = self.get_clock().now()
        try:
            for i, jname in enumerate(FINGER_JOINTS):
                idx = msg.name.index(jname)
                self._joint_positions[i] = float(msg.position[idx])
                if idx < len(msg.effort):
                    self._joint_efforts[i] = float(msg.effort[idx])
                    self._force_history[i].append(self._joint_efforts[i])
        except ValueError:
            return
        self._last_force_time = now

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
            self._closure_phase = _ClosurePhase.IDLE
            self._closure_start_stamp = None
            self._finger_states = [FingerState.RELEASED] * FINGER_COUNT
            self.get_logger().info(
                f"New plan committed — closures={self._planned_closures}, "
                f"wrist={self._planned_wrist_deg:.1f} deg"
            )
            self._publish_wrist_command(self._planned_wrist_deg)
            approach = [self._partial_factor * c for c in self._planned_closures]
            self._publish_joint_commands(approach[0], approach[1], approach[2])
            # Ensure position controllers are active
            if self._force_enabled:
                self._switch_to_position_controllers()

    # ── Control loop ───────────────────────────────────────────────────────

    def _control_loop(self) -> None:
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
                self._closure_phase = _ClosurePhase.IDLE
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
        """Force-aware final closure state machine."""
        now = self.get_clock().now()

        # Transition into final closure
        if self._closure_phase == _ClosurePhase.IDLE:
            self._closure_phase = _ClosurePhase.FINAL_CLOSURE
            self._closure_start_stamp = now
            self._finger_states = [FingerState.OPEN_LOOP] * FINGER_COUNT
            for i in range(FINGER_COUNT):
                self._force_history[i].clear()
            self.get_logger().info("Entering force-aware final closure")
            self._switch_to_velocity_controllers()

        if self._switch_pending:
            return  # Wait for controller switch to complete

        if self._controllers_active != "velocity":
            return  # Still switching

        predicted = self._planned_closures or [0.0, 0.0, 0.0]
        elapsed = (now - self._closure_start_stamp).nanoseconds / 1e9 if self._closure_start_stamp else 0.0

        profile = self._force_profiles.get(self._force_active_profile)
        if profile is None:
            self.get_logger().error(f"Unknown force profile '{self._force_active_profile}'")
            return

        # Check force data freshness
        force_stale = False
        if self._last_force_time is not None:
            since_force = (now - self._last_force_time).nanoseconds / 1e9
            if since_force > self._force_stale_timeout_s:
                force_stale = True
        else:
            force_stale = True

        if force_stale:
            if elapsed * 1000.0 > self._force_reading_timeout_ms:
                self.get_logger().error(
                    f"Force data has not arrived within {self._force_reading_timeout_ms} ms — stopping closure"
                )
                self._stop_all_fingers()
                return

        velocities = [0.0, 0.0, 0.0]
        all_contacted = True

        for i in range(FINGER_COUNT):
            if force_stale:
                force_reading = ForceReading(force=0.0, baseline=0.0)
            else:
                hist = list(self._force_history[i])
                baseline = compute_baseline(hist) if hist else 0.0
                force_reading = ForceReading(force=self._joint_efforts[i], baseline=baseline)

            vel, new_state = step_finger_state(
                profile=profile,
                state=self._finger_states[i],
                predicted_closure=predicted[i],
                current_position=self._joint_positions[i],
                current_force=force_reading,
                elapsed_s=elapsed,
            )
            self._finger_states[i] = new_state
            velocities[i] = vel
            if new_state not in (FingerState.CONTACTED, FingerState.SAFETY_STOPPED):
                all_contacted = False
                self.get_logger().debug(
                    f"Finger {i}: state={new_state.name} vel={vel:.3f} "
                    f"pos={self._joint_positions[i]:.3f} pred={predicted[i]:.3f} "
                    f"force={self._joint_efforts[i]:.1f}"
                )

        if all_contacted:
            self.get_logger().info("All fingers contacted — grasp complete")

        self._publish_velocity_commands(velocities[0], velocities[1], velocities[2])

    def _stop_all_fingers(self) -> None:
        self._publish_velocity_commands(0.0, 0.0, 0.0)
        for i in range(FINGER_COUNT):
            self._finger_states[i] = FingerState.SAFETY_STOPPED

    # ── Controller switching ───────────────────────────────────────────────

    def _switch_to_velocity_controllers(self) -> None:
        if self._controllers_active == "velocity":
            return
        self._switch_pending = True
        self.get_logger().info("Switching to velocity controllers...")
        if not self._switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("SwitchController service not available")
            self._switch_pending = False
            return
        req = SwitchController.Request()
        req.start_controllers = self._finger_vel_controllers
        req.stop_controllers = self._finger_pos_controllers
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = self._switch_client.call_async(req)
        future.add_done_callback(self._on_switch_result)

    def _switch_to_position_controllers(self) -> None:
        if self._controllers_active == "position":
            return
        self._switch_pending = True
        if not self._switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("SwitchController service not available")
            self._switch_pending = False
            return
        req = SwitchController.Request()
        req.start_controllers = self._finger_pos_controllers
        req.stop_controllers = self._finger_vel_controllers
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = self._switch_client.call_async(req)
        future.add_done_callback(self._on_switch_result)

    def _on_switch_result(self, future) -> None:
        self._switch_pending = False
        try:
            result = future.result()
            if result.ok:
                # Determine which controllers are active
                # Check if velocity controllers are running
                any_vel_active = any(
                    c in self._finger_vel_controllers for c in getattr(result, 'active_controllers', [])
                )
                self._controllers_active = "velocity" if any_vel_active else "position"
                self.get_logger().info(
                    f"Controller switch succeeded — active={self._controllers_active}"
                )
            else:
                self.get_logger().error(
                    f"Controller switch failed — result.ok={result.ok}"
                )
        except Exception as e:
            self.get_logger().error(f"Controller switch exception: {e}")

    # ── Helpers ────────────────────────────────────────────────────────────

    @staticmethod
    def _euclidean_distance(a: PoseStamped, b: Pose) -> float:
        dx = a.pose.position.x - b.position.x
        dy = a.pose.position.y - b.position.y
        dz = a.pose.position.z - b.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _floor_closure(self, planned: float) -> float:
        if planned <= 0.0:
            return 0.0
        return max(planned, self._min_closure)

    def _publish_joint_commands(self, thumb: float, index: float, mrl: float) -> None:
        values = [thumb, index, mrl]
        for i in range(FINGER_COUNT):
            msg = Float64MultiArray()
            msg.data = [self._floor_closure(values[i])]
            self._pos_pubs[i].publish(msg)

    def _publish_velocity_commands(self, thumb_vel: float, index_vel: float, mrl_vel: float) -> None:
        velocities = [thumb_vel, index_vel, mrl_vel]
        for i in range(FINGER_COUNT):
            msg = Float64MultiArray()
            msg.data = [float(velocities[i])]
            self._vel_pubs[i].publish(msg)

    def _publish_wrist_command(self, target_deg: float) -> None:
        msg = Float64MultiArray()
        msg.data = [target_deg, self._wrist_accel]
        self._wrist_pub.publish(msg)


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
