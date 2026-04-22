#!/usr/bin/env python3
"""Approach-and-grasp controller for realistic preshaping validation.

Orchestrates the approach workflow:
  1. Hand starts at a configurable distance from the object.
  2. On trigger, the hand approaches the object via smooth motion.
  3. At the ``preshaping_distance`` threshold, the preshaping service is called.
  4. The hand continues until ``grasp_distance``, then stops.

Usage:
  ros2 run mia_hand_mujoco approach_controller_node.py
  ros2 service call /approach_controller/start std_srvs/srv/Trigger
"""

from __future__ import annotations

import math
import threading
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger


class State(Enum):
    IDLE = auto()
    APPROACHING = auto()
    PRESHAPING_TRIGGERED = auto()
    GRASP_READY = auto()


class ApproachControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("approach_controller")

        # --- Parameters ---
        self.declare_parameter("hand_pose_topic", "/mujoco/hand_pose")
        self.declare_parameter("object_pose_topic", "/mujoco/object_pose")
        self.declare_parameter("hand_motion_topic", "/mujoco/move_hand")
        self.declare_parameter("preshaping_service", "/grasp_preshaping/compute_grasp")
        self.declare_parameter("preshaping_distance", 0.12)
        self.declare_parameter("grasp_distance", 0.04)
        self.declare_parameter("approach_duration", 3.0)
        self.declare_parameter("check_hz", 20.0)
        self.declare_parameter("grasp_offset_y", 0.11)
        self.declare_parameter("grasp_offset_z", 0.05)

        self.hand_pose_topic = str(self.get_parameter("hand_pose_topic").value)
        self.object_pose_topic = str(self.get_parameter("object_pose_topic").value)
        self.hand_motion_topic = str(self.get_parameter("hand_motion_topic").value)
        self.preshaping_service_name = str(self.get_parameter("preshaping_service").value)
        self.preshaping_distance = float(self.get_parameter("preshaping_distance").value)
        self.grasp_distance = float(self.get_parameter("grasp_distance").value)
        self.approach_duration = float(self.get_parameter("approach_duration").value)
        self.grasp_offset_y = float(self.get_parameter("grasp_offset_y").value)
        self.grasp_offset_z = float(self.get_parameter("grasp_offset_z").value)

        # --- State ---
        self._lock = threading.Lock()
        self._state = State.IDLE
        self._hand_pose: Pose | None = None
        self._object_pose: Pose | None = None

        # --- Subscribers ---
        self._hand_sub = self.create_subscription(
            Pose, self.hand_pose_topic, self._on_hand_pose, 10)
        self._obj_sub = self.create_subscription(
            Pose, self.object_pose_topic, self._on_object_pose, 10)

        # --- Publishers ---
        self._motion_pub = self.create_publisher(PoseStamped, self.hand_motion_topic, 10)

        # --- Service clients ---
        self._preshaping_client = self.create_client(
            Trigger, self.preshaping_service_name)

        # --- Trigger service ---
        self._trigger_srv = self.create_service(
            Trigger, "/approach_controller/start", self._on_trigger)

        # --- Monitor timer ---
        check_hz = max(1.0, float(self.get_parameter("check_hz").value))
        self._timer = self.create_timer(1.0 / check_hz, self._on_timer)

        self.get_logger().info(
            f"Approach controller ready. "
            f"preshaping_distance={self.preshaping_distance:.3f}m, "
            f"grasp_distance={self.grasp_distance:.3f}m, "
            f"approach_duration={self.approach_duration:.1f}s"
        )

    # --- Callbacks ---

    def _on_hand_pose(self, msg: Pose) -> None:
        with self._lock:
            self._hand_pose = msg

    def _on_object_pose(self, msg: Pose) -> None:
        with self._lock:
            self._object_pose = msg

    def _on_trigger(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        with self._lock:
            if self._state != State.IDLE:
                response.success = False
                response.message = f"Controller is in state {self._state.name}, not IDLE"
                return response

            if self._hand_pose is None:
                response.success = False
                response.message = "No hand pose received yet"
                return response

            if self._object_pose is None:
                response.success = False
                response.message = "No object pose received yet"
                return response

            # Compute grasp target: object position minus the hand-to-EE offset
            # The hand's end-effector (hand_ee_link_r) is at (0, 0.11, 0.05) in
            # the palm frame. To place the EE at the object, we back-compute the
            # palm position. For simplicity, we just move the palm to a position
            # that puts the EE near the object.
            obj = self._object_pose.position
            # Target palm position = object - (0, offset_y, offset_z) in world
            # (This is a simplification assuming the hand orientation is correct)
            target = Pose()
            target.position.x = obj.x
            target.position.y = obj.y - self.grasp_offset_y
            target.position.z = obj.z - self.grasp_offset_z
            target.orientation = self._hand_pose.orientation

            # Send motion command (duration encoded in stamp)
            motion_msg = PoseStamped()
            motion_msg.header.stamp.sec = int(self.approach_duration)
            motion_msg.header.stamp.nanosec = int(
                (self.approach_duration % 1) * 1e9)
            motion_msg.pose = target
            self._motion_pub.publish(motion_msg)

            self._state = State.APPROACHING
            self.get_logger().info(
                f"Approach started: moving hand to "
                f"({target.position.x:.3f}, {target.position.y:.3f}, {target.position.z:.3f}) "
                f"over {self.approach_duration:.1f}s"
            )

        response.success = True
        response.message = "Approach started"
        return response

    def _on_timer(self) -> None:
        with self._lock:
            if self._state == State.IDLE or self._state == State.GRASP_READY:
                return

            if self._hand_pose is None or self._object_pose is None:
                return

            distance = self._compute_distance(
                self._hand_pose, self._object_pose)

            if self._state == State.APPROACHING:
                if distance <= self.preshaping_distance:
                    self._state = State.PRESHAPING_TRIGGERED
                    self.get_logger().info(
                        f"Distance {distance:.4f}m <= preshaping threshold "
                        f"{self.preshaping_distance:.3f}m. Triggering preshaping."
                    )
                    # Call preshaping service (non-blocking)
                    self._trigger_preshaping()

            elif self._state == State.PRESHAPING_TRIGGERED:
                if distance <= self.grasp_distance:
                    self._state = State.GRASP_READY
                    self.get_logger().info(
                        f"Distance {distance:.4f}m <= grasp threshold "
                        f"{self.grasp_distance:.3f}m. Grasp ready."
                    )

    def _trigger_preshaping(self) -> None:
        """Call the preshaping service in a background thread."""
        if not self._preshaping_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"Preshaping service '{self.preshaping_service_name}' not available")
            return

        def _call():
            request = Trigger.Request()
            future = self._preshaping_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            result = future.result()
            if result is not None:
                self.get_logger().info(
                    f"Preshaping result: success={result.success}, "
                    f"message='{result.message}'"
                )
            else:
                self.get_logger().warn("Preshaping service call timed out")

        thread = threading.Thread(target=_call, daemon=True)
        thread.start()

    @staticmethod
    def _compute_distance(pose_a: Pose, pose_b: Pose) -> float:
        dx = pose_a.position.x - pose_b.position.x
        dy = pose_a.position.y - pose_b.position.y
        dz = pose_a.position.z - pose_b.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = ApproachControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
