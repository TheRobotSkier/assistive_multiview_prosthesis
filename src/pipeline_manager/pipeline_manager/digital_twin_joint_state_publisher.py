#!/usr/bin/env python3
"""Publish Mia hand joint states from controller command topics.

This replaces the physical Mia hand hardware interface during digital-twin
tests. The rest of the grasp pipeline still publishes the same controller
commands; this node turns those commanded closure values into JointState
messages for robot_state_publisher and RViz.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


class DigitalTwinJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("digital_twin_joint_state_publisher")

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("prefix", "")
        self.declare_parameter("thumb_max_rad", 1.134500027)
        self.declare_parameter("index_max_rad", 1.399999976)
        self.declare_parameter("mrl_max_rad", 1.396260023)
        self.declare_parameter("thumb_opp_min_rad", -0.628)
        self.declare_parameter("thumb_opp_max_rad", 0.0)
        self.declare_parameter("thumb_opp_start_index_rad", 0.0)
        self.declare_parameter("thumb_opp_stop_index_rad", 1.399999976)

        self._prefix = self.get_parameter("prefix").value
        self._thumb_max = float(self.get_parameter("thumb_max_rad").value)
        self._index_max = float(self.get_parameter("index_max_rad").value)
        self._mrl_max = float(self.get_parameter("mrl_max_rad").value)
        self._thumb_opp_min = float(self.get_parameter("thumb_opp_min_rad").value)
        self._thumb_opp_max = float(self.get_parameter("thumb_opp_max_rad").value)
        self._thumb_opp_start = float(self.get_parameter("thumb_opp_start_index_rad").value)
        self._thumb_opp_stop = float(self.get_parameter("thumb_opp_stop_index_rad").value)

        self._thumb = 0.0
        self._index = 0.0
        self._mrl = 0.0

        self.create_subscription(
            Float64MultiArray,
            "/thumb_pos_ff_controller/commands",
            lambda msg: self._set_closure("thumb", msg),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/index_pos_ff_controller/commands",
            lambda msg: self._set_closure("index", msg),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/mrl_pos_ff_controller/commands",
            lambda msg: self._set_closure("mrl", msg),
            10,
        )

        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            "Digital twin joint-state publisher started "
            "(/.../commands -> /joint_states)"
        )

    def _set_closure(self, joint_group: str, msg: Float64MultiArray) -> None:
        if not msg.data:
            return
        value = _clamp(float(msg.data[0]), 0.0, 1.0)
        if joint_group == "thumb":
            self._thumb = value
        elif joint_group == "index":
            self._index = value
        elif joint_group == "mrl":
            self._mrl = value

    def _thumb_opposition(self, index_rad: float) -> float:
        span = self._thumb_opp_stop - self._thumb_opp_start
        if math.isclose(span, 0.0):
            return self._thumb_opp_max
        ratio = _clamp((index_rad - self._thumb_opp_start) / span, 0.0, 1.0)
        return self._thumb_opp_min + ratio * (self._thumb_opp_max - self._thumb_opp_min)

    def _publish(self) -> None:
        thumb_rad = self._thumb * self._thumb_max
        index_rad = self._index * self._index_max
        mrl_rad = self._mrl * self._mrl_max
        thumb_opp = self._thumb_opposition(index_rad)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            self._prefix + "j_thumb_fle",
            self._prefix + "j_index_fle",
            self._prefix + "j_mrl_fle",
            self._prefix + "j_thumb_opp",
        ]
        msg.position = [thumb_rad, index_rad, mrl_rad, thumb_opp]
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DigitalTwinJointStatePublisher()
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
