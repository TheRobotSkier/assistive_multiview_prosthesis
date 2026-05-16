#!/usr/bin/env python3
"""Publish sensor_msgs/Imu from an MPU-9250/MPU-6500-compatible I2C IMU.

This is the temporary D435 + external GY-91 path for the mixed camera bench
setup. It intentionally avoids third-party Python I2C packages so it can run
inside the existing Jetson container as long as /dev/i2c-* is mounted.
"""

from __future__ import annotations

import fcntl
import math
import os
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

I2C_SLAVE = 0x0703

MPU_PWR_MGMT_1 = 0x6B
MPU_SMPLRT_DIV = 0x19
MPU_CONFIG = 0x1A
MPU_GYRO_CONFIG = 0x1B
MPU_ACCEL_CONFIG = 0x1C
MPU_ACCEL_CONFIG2 = 0x1D
MPU_ACCEL_XOUT_H = 0x3B
MPU_WHO_AM_I = 0x75

G_TO_MPS2 = 9.80665
DPS_TO_RAD = math.pi / 180.0


def _s16(msb: int, lsb: int) -> int:
    value = (msb << 8) | lsb
    if value & 0x8000:
        value -= 65536
    return value


class I2CRegisterDevice:
    def __init__(self, bus: int, address: int) -> None:
        self.path = f"/dev/i2c-{bus}"
        self.fd = os.open(self.path, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, address)

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def write_u8(self, reg: int, value: int) -> None:
        os.write(self.fd, bytes([reg & 0xFF, value & 0xFF]))

    def read_u8(self, reg: int) -> int:
        os.write(self.fd, bytes([reg & 0xFF]))
        return os.read(self.fd, 1)[0]

    def read_block(self, reg: int, length: int) -> bytes:
        os.write(self.fd, bytes([reg & 0xFF]))
        return os.read(self.fd, length)


class I2CMpu9250ImuNode(Node):
    def __init__(self) -> None:
        super().__init__("i2c_mpu9250_imu_node")

        self.declare_parameter("bus", 7)
        self.declare_parameter("address", 0x68)
        self.declare_parameter("frame_id", "head_imu")
        self.declare_parameter("topic", "/head/d435i_head/imu")
        self.declare_parameter("publish_rate_hz", 200.0)
        self.declare_parameter("accel_noise_std", 0.25)
        self.declare_parameter("gyro_noise_std", 0.03)

        bus = int(self.get_parameter("bus").value)
        address = int(self.get_parameter("address").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        topic = str(self.get_parameter("topic").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._accel_var = float(self.get_parameter("accel_noise_std").value) ** 2
        self._gyro_var = float(self.get_parameter("gyro_noise_std").value) ** 2

        self._device = I2CRegisterDevice(bus, address)
        self._configure_device()

        self._pub = self.create_publisher(Imu, topic, 10)
        self.create_timer(1.0 / rate, self._publish_sample)

        self.get_logger().info(
            f"Publishing external IMU {self._device.path}@0x{address:02x} "
            f"as {topic} frame={self._frame_id} at {rate:.1f} Hz"
        )

    def destroy_node(self) -> bool:
        self._device.close()
        return super().destroy_node()

    def _configure_device(self) -> None:
        self._device.write_u8(MPU_PWR_MGMT_1, 0x00)
        self._device.write_u8(MPU_SMPLRT_DIV, 0x04)
        self._device.write_u8(MPU_CONFIG, 0x03)
        self._device.write_u8(MPU_GYRO_CONFIG, 0x00)
        self._device.write_u8(MPU_ACCEL_CONFIG, 0x00)
        self._device.write_u8(MPU_ACCEL_CONFIG2, 0x03)
        who = self._device.read_u8(MPU_WHO_AM_I)
        if who not in (0x70, 0x71, 0x73):
            self.get_logger().warn(
                f"Unexpected MPU WHO_AM_I=0x{who:02x}; continuing anyway"
            )
        else:
            self.get_logger().info(f"MPU WHO_AM_I=0x{who:02x}")

    def _publish_sample(self) -> None:
        try:
            data = self._device.read_block(MPU_ACCEL_XOUT_H, 14)
        except OSError as exc:
            self.get_logger().warn(f"I2C IMU read failed: {exc}", throttle_duration_sec=2.0)
            return

        values = struct.unpack(">hhhhhhh", data)
        ax_g = values[0] / 16384.0
        ay_g = values[1] / 16384.0
        az_g = values[2] / 16384.0
        gx_dps = values[4] / 131.0
        gy_dps = values[5] / 131.0
        gz_dps = values[6] / 131.0

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = gx_dps * DPS_TO_RAD
        msg.angular_velocity.y = gy_dps * DPS_TO_RAD
        msg.angular_velocity.z = gz_dps * DPS_TO_RAD
        msg.linear_acceleration.x = ax_g * G_TO_MPS2
        msg.linear_acceleration.y = ay_g * G_TO_MPS2
        msg.linear_acceleration.z = az_g * G_TO_MPS2

        msg.angular_velocity_covariance = [
            self._gyro_var, 0.0, 0.0,
            0.0, self._gyro_var, 0.0,
            0.0, 0.0, self._gyro_var,
        ]
        msg.linear_acceleration_covariance = [
            self._accel_var, 0.0, 0.0,
            0.0, self._accel_var, 0.0,
            0.0, 0.0, self._accel_var,
        ]
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = I2CMpu9250ImuNode()
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
