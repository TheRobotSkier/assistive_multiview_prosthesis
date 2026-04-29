#!/usr/bin/env python3
"""
Wrist rotation controller node.

Accepts position or velocity commands with acceleration, routing to:
  - simulate=True  → /wrist_pos_ff_controller/commands (MuJoCo ros2_control)
  - simulate=False → Dynamixel SDK on /dev/wrist_motor

Topics:
  Sub: /wrist/set_position  Float64MultiArray [deg, accel_deg_s2]
  Sub: /wrist/set_velocity  Float64MultiArray [deg_s, accel_deg_s2]
  Sub: /joint_states        JointState (for feedback in sim mode)
  Pub: /wrist/state         Float64MultiArray [pos_deg, vel_deg_s]
  Pub: /wrist_pos_ff_controller/commands  Float64MultiArray [pos_rad]  (sim only)
"""

import math
import sys
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# ---------------------------------------------------------------------------
# Dynamixel register addresses (X series, Protocol 2.0)
# ---------------------------------------------------------------------------
ADDR_OPERATING_MODE       = 11
ADDR_TORQUE_ENABLE        = 64
ADDR_GOAL_VELOCITY        = 104
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY     = 112
ADDR_GOAL_POSITION        = 116
ADDR_PRESENT_POSITION     = 132
ADDR_PRESENT_VELOCITY     = 128

OP_MODE_VELOCITY = 1
OP_MODE_POSITION = 3

TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

# Unit conversions
POS_DEG_PER_UNIT  = 0.088          # 1 Dynamixel unit = 0.088°
POS_CENTER        = 2048           # 0° corresponds to raw value 2048
VEL_DEG_S_PER_UNIT = 0.229 * 6.0  # 1 unit ≈ 1.374 deg/s
ACCEL_DEG_S2_PER_UNIT = 214.577 * (360.0 / 60.0) ** 2 / (60.0 ** 2)
# Dynamixel accel unit = 214.577 rev/min²  → deg/s²
# 214.577 rev/min² × (360 deg/rev) / (60 s/min)² = 214.577 × 360 / 3600 ≈ 21.4577 deg/s² ... wait
# Per e-manual: 1 unit = 214.577 [rev/min²]  → deg/s²: 214.577 × 360 / 3600 = 21.4577
# But empirical/common use sees ~0.00614 deg/s² (from plan). Let's use documented:
# 214.577 [rev/min²] × (6°/s per rpm) / 60 = ... actually just use the known value from plan
ACCEL_DEG_S2_PER_UNIT = 0.00614   # 1 unit → 0.00614 deg/s² (derived from 214.577 rev/min²)

WRIST_MIN_DEG = -180.0
WRIST_MAX_DEG =  180.0


class ControlMode(Enum):
    IDLE     = auto()
    POSITION = auto()
    VELOCITY = auto()


class WristControllerNode(Node):
    def __init__(self):
        super().__init__('wrist_controller')

        self.declare_parameter('simulate', True)
        self.declare_parameter('rate_hz', 50)
        self.declare_parameter('port', '/dev/wrist_motor')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('prefix', '')

        self.simulate       = self.get_parameter('simulate').value
        self.rate_hz        = self.get_parameter('rate_hz').value
        self.dt             = 1.0 / self.rate_hz
        self.port_name      = self.get_parameter('port').value
        self.baud           = self.get_parameter('baud').value
        self.motor_id       = self.get_parameter('motor_id').value
        self.wrist_joint    = self.get_parameter('prefix').value + 'wrist_rotation'

        # State
        self.mode           = ControlMode.IDLE
        self.pos_deg        = 0.0     # current position from feedback
        self.vel_deg_s      = 0.0     # current velocity from feedback
        self.target_pos_deg = 0.0     # position set-point
        self.cmd_vel_deg_s  = 0.0     # velocity set-point (velocity mode)
        self.ramp_vel_deg_s = 0.0     # ramped velocity (velocity mode)
        self.accel_deg_s2   = 180.0   # current acceleration limit
        self.hw_op_mode     = -1      # track Dynamixel operating mode to avoid redundant writes

        # Subscriptions
        self.create_subscription(
            Float64MultiArray, '/wrist/set_position', self._cb_set_position, 10)
        self.create_subscription(
            Float64MultiArray, '/wrist/set_velocity', self._cb_set_velocity, 10)
        if self.simulate:
            self.create_subscription(
                JointState, '/joint_states', self._cb_joint_states, 10)

        # Publishers
        self.state_pub = self.create_publisher(Float64MultiArray, '/wrist/state', 10)
        if self.simulate:
            self.cmd_pub = self.create_publisher(
                Float64MultiArray, '/wrist_pos_ff_controller/commands', 10)

        # Hardware init
        if not self.simulate:
            self._init_hardware()

        self.create_timer(self.dt, self._control_loop)
        self.get_logger().info(
            f'WristController started — mode={"sim" if self.simulate else "hardware"}, '
            f'rate={self.rate_hz} Hz')

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _cb_set_position(self, msg: Float64MultiArray):
        if len(msg.data) < 1:
            return
        target_deg  = float(msg.data[0])
        accel       = float(msg.data[1]) if len(msg.data) > 1 else self.accel_deg_s2

        self.target_pos_deg = max(WRIST_MIN_DEG, min(WRIST_MAX_DEG, target_deg))
        self.accel_deg_s2   = max(1.0, accel)
        self.ramp_vel_deg_s = 0.0
        self.mode           = ControlMode.POSITION

    def _cb_set_velocity(self, msg: Float64MultiArray):
        if len(msg.data) < 1:
            return
        vel   = float(msg.data[0])
        accel = float(msg.data[1]) if len(msg.data) > 1 else self.accel_deg_s2

        self.cmd_vel_deg_s = vel
        self.accel_deg_s2  = max(1.0, accel)
        self.mode          = ControlMode.VELOCITY

    def _cb_joint_states(self, msg: JointState):
        if self.wrist_joint in msg.name:
            idx = msg.name.index(self.wrist_joint)
            if idx < len(msg.position):
                self.pos_deg = math.degrees(msg.position[idx])
            if idx < len(msg.velocity):
                self.vel_deg_s = math.degrees(msg.velocity[idx])

    # -----------------------------------------------------------------------
    # Control loop
    # -----------------------------------------------------------------------

    def _control_loop(self):
        if self.mode == ControlMode.POSITION:
            self._step_position()
        elif self.mode == ControlMode.VELOCITY:
            self._step_velocity()

        if not self.simulate:
            self._hw_poll_state()

        self._publish_state()

    def _step_position(self):
        error = self.target_pos_deg - self.pos_deg
        if abs(error) < 0.5:
            self.ramp_vel_deg_s = 0.0
            self.mode = ControlMode.IDLE
            # hold at target
            self._send_position_cmd(self.target_pos_deg)
            return

        # Trapezoidal profile: cap velocity by deceleration ramp
        v_decel = math.sqrt(2.0 * self.accel_deg_s2 * abs(error))
        sign    = 1.0 if error > 0 else -1.0

        # Accelerate toward decel-capped max
        target_vel = sign * v_decel
        step = self.accel_deg_s2 * self.dt
        if abs(target_vel - self.ramp_vel_deg_s) <= step:
            self.ramp_vel_deg_s = target_vel
        else:
            self.ramp_vel_deg_s += step * (1.0 if target_vel > self.ramp_vel_deg_s else -1.0)

        next_pos = self.pos_deg + self.ramp_vel_deg_s * self.dt
        next_pos = max(WRIST_MIN_DEG, min(WRIST_MAX_DEG, next_pos))
        self._send_position_cmd(next_pos)

    def _step_velocity(self):
        # Ramp actual velocity toward commanded velocity
        step = self.accel_deg_s2 * self.dt
        diff = self.cmd_vel_deg_s - self.ramp_vel_deg_s
        if abs(diff) <= step:
            self.ramp_vel_deg_s = self.cmd_vel_deg_s
        else:
            self.ramp_vel_deg_s += step * (1.0 if diff > 0 else -1.0)

        # If commanded zero and ramped to zero, return to IDLE
        if self.cmd_vel_deg_s == 0.0 and abs(self.ramp_vel_deg_s) < 0.01:
            self.ramp_vel_deg_s = 0.0
            self.mode = ControlMode.IDLE
            self._send_position_cmd(self.pos_deg)
            return

        if self.simulate:
            # Integrate position and command it
            next_pos = self.pos_deg + self.ramp_vel_deg_s * self.dt
            next_pos = max(WRIST_MIN_DEG, min(WRIST_MAX_DEG, next_pos))
            if next_pos in (WRIST_MIN_DEG, WRIST_MAX_DEG):
                self.ramp_vel_deg_s = 0.0
                self.cmd_vel_deg_s  = 0.0
            self._send_position_cmd(next_pos)
        else:
            self._hw_set_velocity(self.ramp_vel_deg_s, self.accel_deg_s2)

    # -----------------------------------------------------------------------
    # Command helpers
    # -----------------------------------------------------------------------

    def _send_position_cmd(self, pos_deg: float):
        if self.simulate:
            pos_rad = math.radians(pos_deg)
            msg = Float64MultiArray()
            msg.data = [pos_rad]
            self.cmd_pub.publish(msg)
        else:
            self._hw_set_position(pos_deg, self.accel_deg_s2)

    def _publish_state(self):
        msg = Float64MultiArray()
        msg.data = [self.pos_deg, self.vel_deg_s]
        self.state_pub.publish(msg)

    # -----------------------------------------------------------------------
    # Hardware interface (Dynamixel SDK)
    # -----------------------------------------------------------------------

    def _init_hardware(self):
        try:
            from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
        except ImportError:
            self.get_logger().fatal('dynamixel_sdk not installed. Install: pip3 install dynamixel-sdk')
            sys.exit(1)

        self._COMM_SUCCESS = COMM_SUCCESS
        self._port = PortHandler(self.port_name)
        self._pkt  = PacketHandler(2.0)

        if not self._port.openPort():
            self.get_logger().fatal(f'Cannot open port {self.port_name}')
            sys.exit(1)
        if not self._port.setBaudRate(self.baud):
            self.get_logger().fatal(f'Cannot set baud {self.baud}')
            sys.exit(1)

        # Enable torque (position mode default)
        self._hw_write1(ADDR_OPERATING_MODE, OP_MODE_POSITION)
        self._hw_write1(ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        self.hw_op_mode = OP_MODE_POSITION
        self.get_logger().info(f'Dynamixel ID {self.motor_id} on {self.port_name} ready')

    def _hw_write1(self, addr: int, val: int):
        result, err = self._pkt.write1ByteTxRx(self._port, self.motor_id, addr, val)
        if result != self._COMM_SUCCESS:
            self.get_logger().warn(f'write1 addr={addr} failed: {self._pkt.getTxRxResult(result)}')

    def _hw_write4(self, addr: int, val: int):
        result, err = self._pkt.write4ByteTxRx(self._port, self.motor_id, addr, val)
        if result != self._COMM_SUCCESS:
            self.get_logger().warn(f'write4 addr={addr} failed: {self._pkt.getTxRxResult(result)}')

    def _hw_read4(self, addr: int) -> int | None:
        val, result, _ = self._pkt.read4ByteTxRx(self._port, self.motor_id, addr)
        if result != self._COMM_SUCCESS:
            return None
        return val

    def _hw_ensure_op_mode(self, op_mode: int):
        if self.hw_op_mode == op_mode:
            return
        self._hw_write1(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self._hw_write1(ADDR_OPERATING_MODE, op_mode)
        self._hw_write1(ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        self.hw_op_mode = op_mode

    def _hw_set_position(self, pos_deg: float, accel_deg_s2: float):
        self._hw_ensure_op_mode(OP_MODE_POSITION)
        accel_raw = max(1, int(accel_deg_s2 / ACCEL_DEG_S2_PER_UNIT))
        self._hw_write4(ADDR_PROFILE_ACCELERATION, accel_raw)
        pos_deg_clamped = max(WRIST_MIN_DEG, min(WRIST_MAX_DEG, pos_deg))
        pos_raw = int(pos_deg_clamped / POS_DEG_PER_UNIT) + POS_CENTER
        pos_raw = max(0, min(4095, pos_raw))
        self._hw_write4(ADDR_GOAL_POSITION, pos_raw)

    def _hw_set_velocity(self, vel_deg_s: float, accel_deg_s2: float):
        self._hw_ensure_op_mode(OP_MODE_VELOCITY)
        accel_raw = max(1, int(accel_deg_s2 / ACCEL_DEG_S2_PER_UNIT))
        self._hw_write4(ADDR_PROFILE_ACCELERATION, accel_raw)
        vel_raw = int(vel_deg_s / VEL_DEG_S_PER_UNIT)
        # Dynamixel XM430 velocity: signed int, but SDK sends as uint32 (two's complement)
        if vel_raw < 0:
            vel_raw = vel_raw & 0xFFFFFFFF
        self._hw_write4(ADDR_GOAL_VELOCITY, vel_raw)

    def _hw_poll_state(self):
        raw_pos = self._hw_read4(ADDR_PRESENT_POSITION)
        raw_vel = self._hw_read4(ADDR_PRESENT_VELOCITY)
        if raw_pos is not None:
            self.pos_deg = (raw_pos - POS_CENTER) * POS_DEG_PER_UNIT
        if raw_vel is not None:
            # Two's complement for signed velocity
            if raw_vel > 0x7FFFFFFF:
                raw_vel -= 0x100000000
            self.vel_deg_s = raw_vel * VEL_DEG_S_PER_UNIT

    def destroy_node(self):
        if not self.simulate and hasattr(self, '_port'):
            self._hw_write1(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self._port.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WristControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
