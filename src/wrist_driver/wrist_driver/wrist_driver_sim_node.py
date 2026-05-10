"""Simulated wrist motor driver for digital twin (no hardware needed).

Subscribes to /wrist/set_position and publishes joint_states so the URDF
wrist rotates in RViz. Also publishes simulated /wrist/state feedback.

Subscribes:
  /wrist/set_position  std_msgs/Float64MultiArray [position_deg, acceleration_deg_s2]

Publishes:
  /wrist/state         std_msgs/Float64MultiArray [position_deg, velocity_deg_s]
  /joint_states        sensor_msgs/JointState     (wrist_rotation joint)
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class WristDriverSimNode(Node):
    def __init__(self):
        super().__init__("wrist_driver_sim")

        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("joint_name", "wrist_rotation")
        self.declare_parameter("smooth_factor", 0.15)

        rate = self.get_parameter("update_rate_hz").value
        self._joint_name = self.get_parameter("joint_name").value
        self._smooth = self.get_parameter("smooth_factor").value

        self._current_pos_deg = 0.0
        self._target_pos_deg = 0.0
        self._velocity_deg_s = 0.0

        self._pub_state = self.create_publisher(Float64MultiArray, "/wrist/state", 10)
        self._pub_joint = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(
            Float64MultiArray, "/wrist/set_position", self._on_position_cmd, 10
        )
        self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(
            f"Wrist sim started — joint={self._joint_name}, rate={rate} Hz"
        )

    def _on_position_cmd(self, msg: Float64MultiArray):
        if len(msg.data) < 1:
            return
        self._target_pos_deg = msg.data[0]

    def _update(self):
        err = self._target_pos_deg - self._current_pos_deg
        if abs(err) < 0.05:
            self._current_pos_deg = self._target_pos_deg
            self._velocity_deg_s = 0.0
        else:
            step = err * self._smooth
            self._current_pos_deg += step
            self._velocity_deg_s = step * 50.0

        msg = Float64MultiArray()
        msg.data = [self._current_pos_deg, self._velocity_deg_s]
        self._pub_state.publish(msg)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self._joint_name]
        js.position = [math.radians(self._current_pos_deg)]
        self._pub_joint.publish(js)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WristDriverSimNode()
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
