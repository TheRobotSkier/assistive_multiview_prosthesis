#!/usr/bin/env python3
"""Command Bridge Node — forwards ros2_control-style topics to the raw Mia Hand driver.

This bridge sits between the high-level controllers (proximity controller, force
controller) and the raw Mia Hand driver.  The controllers publish position
commands to ``*_pos_ff_controller/commands`` topics (the ros2_control convention),
but the raw driver only accepts ``joints/*/set_trajectory`` service calls.  This
node translates between the two.

It also subscribes to the raw driver's joint position data stream and republishes
it as ``/joint_states`` so the force controller can track current joint positions.

Subscriptions:
    /thumb_pos_ff_controller/commands   (std_msgs/Float64MultiArray)
    /index_pos_ff_controller/commands   (std_msgs/Float64MultiArray)
    /mrl_pos_ff_controller/commands     (std_msgs/Float64MultiArray)
    data_streams/joints/positions/data  (mia_hand_msgs/JointData)

Service clients:
    joints/thumb/set_trajectory  (mia_hand_msgs/SetJointTraj)
    joints/index/set_trajectory  (mia_hand_msgs/SetJointTraj)
    joints/mrl/set_trajectory    (mia_hand_msgs/SetJointTraj)
    data_streams/joints/positions/switch  (std_srvs/SetBool)

Publishers:
    /joint_states  (sensor_msgs/JointState)

Parameters:
    thumb_cmd_topic          (str)  — Thumb position command topic
    index_cmd_topic          (str)  — Index position command topic
    mrl_cmd_topic            (str)  — MRL position command topic
    thumb_traj_service       (str)  — Thumb trajectory service name
    index_traj_service       (str)  — Index trajectory service name
    mrl_traj_service         (str)  — MRL trajectory service name
    joint_positions_topic    (str)  — Raw driver joint positions topic
    joint_states_topic       (str)  — Output JointState topic
    joint_stream_switch      (str)  — Service to activate joint position streaming
    default_speed_percent    (int)  — Default trajectory speed/force percentage
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from mia_hand_msgs.msg import JointData
from mia_hand_msgs.srv import SetJointTraj
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool


# Joint names expected by the force controller (matches ros2_control URDF)
_JOINT_NAMES = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]


class CommandBridgeNode(Node):
    """Bridge node that forwards position command topics to raw driver services."""

    def __init__(self) -> None:
        super().__init__("command_bridge")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("thumb_cmd_topic", "/thumb_pos_ff_controller/commands")
        self.declare_parameter("index_cmd_topic", "/index_pos_ff_controller/commands")
        self.declare_parameter("mrl_cmd_topic", "/mrl_pos_ff_controller/commands")
        self.declare_parameter("thumb_traj_service", "joints/thumb/set_trajectory")
        self.declare_parameter("index_traj_service", "joints/index/set_trajectory")
        self.declare_parameter("mrl_traj_service", "joints/mrl/set_trajectory")
        self.declare_parameter("joint_positions_topic", "data_streams/joints/positions/data")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("joint_stream_switch", "data_streams/joints/positions/switch")
        self.declare_parameter("default_speed_percent", 80)

        speed_pct = self.get_parameter("default_speed_percent").value

        # ── Service clients ───────────────────────────────────────────────
        self._thumb_srv = self.create_client(
            SetJointTraj, self.get_parameter("thumb_traj_service").value
        )
        self._index_srv = self.create_client(
            SetJointTraj, self.get_parameter("index_traj_service").value
        )
        self._mrl_srv = self.create_client(
            SetJointTraj, self.get_parameter("mrl_traj_service").value
        )
        self._jnt_stream_switch = self.create_client(
            SetBool, self.get_parameter("joint_stream_switch").value
        )

        # ── Subscriptions (command topics → service calls) ────────────────
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("thumb_cmd_topic").value,
            lambda msg: self._on_cmd(msg, self._thumb_srv, "thumb"),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("index_cmd_topic").value,
            lambda msg: self._on_cmd(msg, self._index_srv, "index"),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("mrl_cmd_topic").value,
            lambda msg: self._on_cmd(msg, self._mrl_srv, "mrl"),
            10,
        )

        # ── Subscription (joint positions → /joint_states) ────────────────
        self._js_pub = self.create_publisher(
            JointState, self.get_parameter("joint_states_topic").value, 10
        )
        self.create_subscription(
            JointData,
            self.get_parameter("joint_positions_topic").value,
            self._on_joint_positions,
            10,
        )

        # ── State ─────────────────────────────────────────────────────────
        self._speed_pct = speed_pct
        self._stream_activated = False

        # ── Latched commands ──────────────────────────────────────────────
        self._latched_commands: dict[str, float] = {}
        self._latched_timestamps: dict[str, float] = {}
        self._retry_timer = self.create_timer(2.0, self._retry_latched_commands)

        # ── Activate joint position stream ────────────────────────────────
        # Retry periodically until the driver is available.
        self._activate_timer = self.create_timer(2.0, self._try_activate_stream)

        self.get_logger().info(
            f"Command bridge started — speed={self._speed_pct}%, "
            f"joint_states → {self.get_parameter('joint_states_topic').value}"
        )

    # ── Command forwarding ────────────────────────────────────────────────

    def _get_client_for_finger(self, finger: str) -> rclpy.client.Client:
        """Return the trajectory service client for a given finger."""
        if finger == "thumb":
            return self._thumb_srv
        elif finger == "index":
            return self._index_srv
        else:
            return self._mrl_srv

    def _send_cmd(
        self, finger: str, target_angle: float, srv_client: rclpy.client.Client
    ) -> None:
        """Send a latched or direct position command via the trajectory service."""
        req = SetJointTraj.Request()
        req.target_angle = target_angle
        req.spe_for_percent = self._speed_pct

        future = srv_client.call_async(req)

        def on_response(fut: object) -> None:
            try:
                result = fut.result()
                if not result.success:
                    self.get_logger().warn(
                        f"Trajectory rejected for {finger}: {result.err_message}"
                    )
            except Exception as exc:
                self.get_logger().error(
                    f"Service call failed for {finger}: {exc}"
                )

        future.add_done_callback(on_response)

    def _on_cmd(
        self, msg: Float64MultiArray, srv_client: rclpy.client.Client, finger: str
    ) -> None:
        """Forward a position command to the corresponding trajectory service."""
        if not msg.data:
            self.get_logger().warn(f"Empty command on {finger} — ignoring")
            return

        target_angle = float(msg.data[0])

        if not srv_client.service_is_ready():
            self._latched_commands[finger] = target_angle
            self._latched_timestamps[finger] = time.monotonic()
            self.get_logger().debug(
                f"Service for {finger} not ready — latching command "
                f"(angle={target_angle:.4f} rad)"
            )
            return

        self._send_cmd(finger, target_angle, srv_client)

    def _retry_latched_commands(self) -> None:
        """Retry latched commands when their services become ready."""
        now = time.monotonic()
        to_remove: list[str] = []
        for finger, target_angle in list(self._latched_commands.items()):
            srv_client = self._get_client_for_finger(finger)
            if srv_client.service_is_ready():
                self.get_logger().info(
                    f"Service for {finger} ready — sending latched command "
                    f"(angle={target_angle:.4f} rad)"
                )
                self._send_cmd(finger, target_angle, srv_client)
                to_remove.append(finger)
            elif now - self._latched_timestamps[finger] > 10.0:
                self.get_logger().error(
                    f"Latched command for {finger} timed out after 10s — dropping "
                    f"(angle={target_angle:.4f} rad)"
                )
                to_remove.append(finger)
        for finger in to_remove:
            del self._latched_commands[finger]
            del self._latched_timestamps[finger]

    # ── Joint state translation ───────────────────────────────────────────

    def _on_joint_positions(self, msg: JointData) -> None:
        """Convert JointData from raw driver to sensor_msgs/JointState."""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(_JOINT_NAMES)
        js.position = [float(msg.thumb_data), float(msg.index_data), float(msg.mrl_data)]
        self._js_pub.publish(js)

    # ── Stream activation ─────────────────────────────────────────────────

    def _try_activate_stream(self) -> None:
        """Periodically try to activate the joint position data stream."""
        if self._stream_activated:
            self._activate_timer.cancel()
            return

        if not self._jnt_stream_switch.service_is_ready():
            self.get_logger().debug(
                "Joint position stream switch not ready — will retry"
            )
            return

        req = SetBool.Request()
        req.data = True
        future = self._jnt_stream_switch.call_async(req)

        def on_response(fut: object) -> None:
            try:
                result = fut.result()
                if result.success:
                    self._stream_activated = True
                    self.get_logger().info("Joint position stream activated.")
                else:
                    self.get_logger().warn(
                        f"Joint position stream activation failed: {result.message}"
                    )
            except Exception as exc:
                self.get_logger().error(
                    f"Joint position stream switch error: {exc}"
                )

        future.add_done_callback(on_response)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CommandBridgeNode()
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
