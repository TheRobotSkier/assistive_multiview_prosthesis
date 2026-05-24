#!/usr/bin/env python3
"""Force Bridge Node — bridges ros2_control joint effort → ForceData topic.

ros2_control exposes fingertip force/torque sensor readings as effort in
/joint_states. The force_controller_node expects a dedicated ForceData topic
(data_streams/fingers/forces/data). This node translates between the two.

Subscriptions:
    /joint_states  (sensor_msgs/JointState)
        Joint position/velocity/effort from ros2_control.

Publishers:
    data_streams/fingers/forces/data  (mia_hand_msgs/ForceData)
        Force sensor data in the format expected by force_controller_node.
        Normal forces = joint state efforts; tangential forces = 0 (not
        available from effort interface).

Service servers:
    data_streams/fingers/forces/switch  (std_srvs/SetBool)
        No-op — always returns success. The real switch service lives on the
        Mia Hand hardware driver; this bridge provides a compatible endpoint
        so force_controller_node does not fail on startup.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from mia_hand_msgs.msg import ForceData
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]


class ForceBridgeNode(Node):
    """Bridge ros2_control effort → ForceData for force_controller."""

    def __init__(self) -> None:
        super().__init__("force_bridge")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("force_data_topic", "data_streams/fingers/forces/data")
        self.declare_parameter("force_switch_service", "data_streams/fingers/forces/switch")
        self.declare_parameter("publish_rate_hz", 100.0)

        self._joint_states_topic = self.get_parameter("joint_states_topic").value
        self._force_data_topic = self.get_parameter("force_data_topic").value
        self._force_switch_service = self.get_parameter("force_switch_service").value
        rate_hz = self.get_parameter("publish_rate_hz").value
        self._dt = 1.0 / rate_hz

        # ── State ─────────────────────────────────────────────────────────
        self._efforts: list[int] = [0, 0, 0]

        # ── Subscription ──────────────────────────────────────────────────
        self.create_subscription(
            JointState, self._joint_states_topic, self._on_joint_states, 10
        )

        # ── Publisher ─────────────────────────────────────────────────────
        self._force_pub = self.create_publisher(
            ForceData, self._force_data_topic, 10
        )

        # ── Service: switch (no-op) ───────────────────────────────────────
        self.create_service(
            SetBool, self._force_switch_service, self._on_switch
        )

        # ── Timer: publish at rate ────────────────────────────────────────
        self.create_timer(self._dt, self._publish_force)

        self.get_logger().info(
            f"Force bridge started: {self._joint_states_topic} → "
            f"{self._force_data_topic} @ {rate_hz} Hz"
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState) -> None:
        """Extract efforts from joint_states and cache them."""
        for i, jname in enumerate(FINGER_JOINTS):
            try:
                idx = msg.name.index(jname)
                self._efforts[i] = int(round(msg.effort[idx]))
            except (ValueError, IndexError):
                pass

    def _on_switch(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """No-op — the real switch lives on the Mia Hand hardware driver."""
        self.get_logger().debug(
            f"Force switch requested (data={request.data}) — no-op bridge"
        )
        response.success = True
        response.message = "Force bridge switch is a no-op (hardware switch not available)"
        return response

    # ── Publishing ─────────────────────────────────────────────────────────

    def _publish_force(self) -> None:
        """Publish cached efforts as ForceData."""
        msg = ForceData()
        msg.thumb_nfor = self._efforts[0]
        msg.index_nfor = self._efforts[1]
        msg.mrl_nfor = self._efforts[2]
        msg.thumb_tfor = 0
        msg.index_tfor = 0
        msg.mrl_tfor = 0
        self._force_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ForceBridgeNode()
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
