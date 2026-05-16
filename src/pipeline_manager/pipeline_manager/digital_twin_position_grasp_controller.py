#!/usr/bin/env python3
"""Position-only grasp controller for the Mia hand digital twin.

The physical pipeline can regulate a grasp with force feedback. The digital
twin has no real finger forces, so this node executes the planner's requested
finger closures directly by publishing the same forward-position controller
topics used by the hardware-facing stack.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray, Int32


STATE_IDLE = 0
STATE_RELEASING = 6


def _clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return min(max(value, low), high)


class DigitalTwinPositionGraspController(Node):
    def __init__(self) -> None:
        super().__init__("digital_twin_position_grasp_controller")

        self.declare_parameter(
            "target_closures_topic", "/grasp_preshaping/target_finger_closures"
        )
        self.declare_parameter("pipeline_state_topic", "/pipeline/state")
        self.declare_parameter("thumb_cmd_topic", "/thumb_pos_ff_controller/commands")
        self.declare_parameter("index_cmd_topic", "/index_pos_ff_controller/commands")
        self.declare_parameter("mrl_cmd_topic", "/mrl_pos_ff_controller/commands")
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("closure_scale", 1.0)
        self.declare_parameter("min_closure_amount", 0.1)
        self.declare_parameter("open_on_idle", True)

        self._closure_scale = float(self.get_parameter("closure_scale").value)
        self._min_closure = float(self.get_parameter("min_closure_amount").value)
        self._open_on_idle = bool(self.get_parameter("open_on_idle").value)
        self._target = [0.0, 0.0, 0.0]
        self._has_target = False

        self._thumb_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("thumb_cmd_topic").value,
            10,
        )
        self._index_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("index_cmd_topic").value,
            10,
        )
        self._mrl_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("mrl_cmd_topic").value,
            10,
        )

        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("target_closures_topic").value,
            self._on_target_closures,
            latched,
        )
        self.create_subscription(
            Int32,
            self.get_parameter("pipeline_state_topic").value,
            self._on_pipeline_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        rate = float(self.get_parameter("command_rate_hz").value)
        self.create_timer(1.0 / rate, self._publish_commands)

        self.get_logger().info(
            "Digital twin position grasp controller started "
            "(/grasp_preshaping/target_finger_closures -> /.../commands)"
        )

    def _on_target_closures(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 3:
            self.get_logger().warn(
                f"Expected at least 3 target closures, got {len(msg.data)}"
            )
            return

        self._target = [
            _clamp(float(msg.data[0]) * self._closure_scale),
            _clamp(float(msg.data[1]) * self._closure_scale),
            _clamp(float(msg.data[2]) * self._closure_scale),
        ]
        self._has_target = True
        self.get_logger().info(
            "Digital twin target closures: "
            f"thumb={self._target[0]:.3f}, "
            f"index={self._target[1]:.3f}, "
            f"mrl={self._target[2]:.3f}"
        )
        self._publish_commands()

    def _on_pipeline_state(self, msg: Int32) -> None:
        if not self._open_on_idle:
            return
        if msg.data in (STATE_IDLE, STATE_RELEASING):
            if self._has_target or any(self._target):
                self.get_logger().info("Opening digital twin hand")
            self._target = [0.0, 0.0, 0.0]
            self._has_target = False
            self._publish_commands()

    def _floor_closure(self, closure: float) -> float:
        if closure <= 0.0:
            return 0.0
        return max(closure, self._min_closure)

    def _publish_commands(self) -> None:
        thumb, index, mrl = (self._floor_closure(v) for v in self._target)
        self._thumb_pub.publish(Float64MultiArray(data=[thumb]))
        self._index_pub.publish(Float64MultiArray(data=[index]))
        self._mrl_pub.publish(Float64MultiArray(data=[mrl]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DigitalTwinPositionGraspController()
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
