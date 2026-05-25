#!/usr/bin/env python3
"""Dynamixel wrist motor driver node.

Interfaces with a single Dynamixel servo (wrist rotation) via the dynamixel-sdk.
Provides position control and publishes current state.

Subscribes:
  /wrist/set_position  std_msgs/Float64MultiArray [position_deg, acceleration_deg_s2]

Publishes:
  /wrist/state         std_msgs/Float64MultiArray [position_deg, velocity_deg_s]
"""

from __future__ import annotations

import math
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

try:
    from dynamixel_sdk import (
        PortHandler,
        PacketHandler,
        COMM_SUCCESS,
    )
    HAS_DYNAMIXEL = True
except ImportError:
    HAS_DYNAMIXEL = False


# Control table addresses for Dynamixel X series
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# Dynamixel position range: 0-4095 maps to 0-360 degrees
DXL_POSITION_RANGE = 4095.0
DXL_ANGLE_RANGE = 360.0


def _deg_to_dx(deg: float) -> int:
    """Convert degrees to Dynamixel position units."""
    return int((deg % 360.0) / DXL_ANGLE_RANGE * DXL_POSITION_RANGE)


def _dx_to_deg(dx: int) -> float:
    """Convert Dynamixel position units to degrees."""
    return float(dx) / DXL_POSITION_RANGE * DXL_ANGLE_RANGE


class WristDriverNode(Node):
    def __init__(self):
        super().__init__('wrist_driver')

        self.declare_parameter('port', os.environ.get('WRIST_SERIAL_PORT', '/dev/ttyDynamixel'))
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('protocol_version', 2.0)
        self.declare_parameter('publish_rate_hz', 20.0)

        if not HAS_DYNAMIXEL:
            self.get_logger().error(
                'dynamixel-sdk not installed. Install with: pip install dynamixel-sdk')
            return

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self._motor_id = self.get_parameter('motor_id').value
        protocol = self.get_parameter('protocol_version').value
        rate = self.get_parameter('publish_rate_hz').value

        self._port_handler = PortHandler(port)
        self._packet_handler = PacketHandler(protocol)

        if not self._port_handler.openPort():
            self.get_logger().error(f'Failed to open port {port}')
            return

        if not self._port_handler.setBaudRate(baudrate):
            self.get_logger().error(f'Failed to set baudrate to {baudrate}')
            return

        # Enable torque
        dxl_comm_result, dxl_error = self._packet_handler.write1ByteTxRx(
            self._port_handler, self._motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to enable torque: {self._packet_handler.getTxRxResult(dxl_comm_result)}')
            return
        self.get_logger().info(f'Wrist motor connected on {port}, ID={self._motor_id}')

        self._pub = self.create_publisher(Float64MultiArray, '/wrist/state', 10)
        self.create_subscription(
            Float64MultiArray, '/wrist/set_position', self._on_position_cmd, 10)
        self.create_timer(1.0 / rate, self._publish_state)

    def _on_position_cmd(self, msg: Float64MultiArray):
        if len(msg.data) < 1:
            return
        target_deg = msg.data[0]
        accel = msg.data[1] if len(msg.data) > 1 else 0.0

        dxl_pos = _deg_to_dx(target_deg)
        dxl_comm_result, dxl_error = self._packet_handler.write4ByteTxRx(
            self._port_handler, self._motor_id, ADDR_GOAL_POSITION, dxl_pos)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().warn(f'Failed to set position: {self._packet_handler.getTxRxResult(dxl_comm_result)}')

        if accel > 0:
            accel_val = int(accel)
            self._packet_handler.write4ByteTxRx(
                self._port_handler, self._motor_id, ADDR_PROFILE_ACCELERATION, accel_val)

    def _publish_state(self):
        dxl_pos, comm_result, _ = self._packet_handler.read4ByteTxRx(
            self._port_handler, self._motor_id, ADDR_PRESENT_POSITION)
        dxl_vel, comm_result_v, _ = self._packet_handler.read4ByteTxRx(
            self._port_handler, self._motor_id, ADDR_PRESENT_VELOCITY)

        if comm_result == COMM_SUCCESS and comm_result_v == COMM_SUCCESS:
            pos_deg = _dx_to_deg(dxl_pos)
            vel_deg = float(dxl_vel) * 0.229  # Approximate RPM to deg/s
            msg = Float64MultiArray()
            msg.data = [pos_deg, vel_deg]
            self._pub.publish(msg)

    def destroy_node(self):
        if HAS_DYNAMIXEL and self._port_handler.is_open:
            self._packet_handler.write1ByteTxRx(
                self._port_handler, self._motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self._port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WristDriverNode()
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
