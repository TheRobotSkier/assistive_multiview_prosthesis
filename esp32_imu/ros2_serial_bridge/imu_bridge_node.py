#!/usr/bin/env python3
"""
ESP32 Dual GY-91 → ROS2 IMU Bridge
===================================
Reads binary packets from ESP32 over USB serial and publishes
sensor_msgs/Imu on two topics — one per camera-mounted IMU.

Binary packet format (33 bytes, little-endian):
  [0-1]   Header:    0xAA 0x55
  [2-3]   Seq:       uint16
  [4-7]   Timestamp: uint32 (microseconds)
  [8-13]  Head accel: int16 x, y, z  (raw)
  [14-19] Head gyro:  int16 x, y, z  (raw)
  [20-25] Arm accel:  int16 x, y, z  (raw)
  [26-31] Arm gyro:   int16 x, y, z  (raw)
  [32]    Checksum:   uint8 XOR of bytes [2..31]

Usage:
  ros2 run esp32_imu_bridge imu_bridge_node --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p config_file:=esp32_imu/config.yaml
"""

import struct
import time
from pathlib import Path
from typing import Optional

import rclpy
import serial
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Imu

# ── MPU9250 scale factors ────────────────────────────────────────────────
# From MPU-9250 datasheet rev 1.1
GYRO_SCALE = {
    250:  131.0,
    500:  65.5,
    1000: 32.8,
    2000: 16.4,
}

ACCEL_SCALE = {
    2:  16384.0,
    4:  8192.0,
    8:  4096.0,
    16: 2048.0,
}

# Standard gravity in m/s²
G_STD = 9.80665

PACKET_STRUCT = struct.Struct("<BB H I 3h 3h 3h 3h B")
PACKET_SIZE = PACKET_STRUCT.size  # 33 bytes


def parse_packet(data: bytes) -> Optional[dict]:
    """Parse a 33-byte binary packet. Returns dict or None on failure."""
    if len(data) != PACKET_SIZE:
        return None
    try:
        fields = PACKET_STRUCT.unpack(data)
    except struct.error:
        return None

    h0, h1, seq, ts = fields[0:4]
    if h0 != 0xAA or h1 != 0x55:
        return None

    # Verify checksum: XOR of bytes [2..31]
    ck = 0
    for b in data[2:32]:
        ck ^= b
    if ck != fields[-1]:
        return None

    return {
        "seq": seq,
        "timestamp_us": ts,
        "head_accel": list(fields[4:7]),
        "head_gyro":  list(fields[7:10]),
        "arm_accel":  list(fields[10:13]),
        "arm_gyro":   list(fields[13:16]),
    }


class IMUBridgeNode(Node):
    def __init__(self):
        super().__init__("imu_bridge_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("config_file", "esp32_imu/config.yaml")
        self.declare_parameter("baud_rate", 921600)

        serial_port = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value
        config_file = self.get_parameter("config_file").value

        # Load config
        cfg = yaml.safe_load(Path(config_file).read_text())
        mpu_cfg = cfg["mpu9250"]

        self.gyro_scale = GYRO_SCALE[mpu_cfg["gyro_range_dps"]]
        self.accel_scale = ACCEL_SCALE[mpu_cfg["accel_range_g"]]

        # IMU → camera offsets (for covariance, not used in IMU msg directly)
        offsets = cfg["offsets"]
        self.head_offset = offsets["head"]
        self.arm_offset = offsets["arm"]

        # Publishers
        self.head_pub = self.create_publisher(Imu, "/head/imu", 50)
        self.arm_pub = self.create_publisher(Imu, "/arm/imu", 50)

        # Open serial
        self.get_logger().info(f"Opening {serial_port} at {baud_rate}")
        self.ser = serial.Serial(serial_port, baud_rate, timeout=0.05)
        self.buf = bytearray()

        # Stats
        self.pkt_count = 0
        self.err_count = 0
        self.last_stats = time.time()

    def spin_once(self):
        """Read all available bytes, parse packets, publish."""
        try:
            chunk = self.ser.read(self.ser.in_waiting or 1)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            return

        if not chunk:
            return

        self.buf.extend(chunk)

        # Scan for header 0xAA 0x55
        while len(self.buf) >= PACKET_SIZE:
            # Find header
            idx = self.buf.find(b"\xAA\x55")
            if idx < 0:
                # No header found; keep last byte (could be partial 0xAA)
                self.buf = self.buf[-1:]
                break
            if idx > 0:
                self.buf = self.buf[idx:]  # discard junk before header

            if len(self.buf) < PACKET_SIZE:
                break

            candidate = bytes(self.buf[:PACKET_SIZE])
            parsed = parse_packet(candidate)
            if parsed is None:
                # Bad packet — skip the header and try again
                self.err_count += 1
                self.buf = self.buf[2:]
                continue

            self.pkt_count += 1
            self.buf = self.buf[PACKET_SIZE:]
            self._publish(parsed)

        # Periodic stats
        now = time.time()
        if now - self.last_stats >= 5.0:
            rate = self.pkt_count / (now - self.last_stats) if self.pkt_count else 0
            self.get_logger().info(
                f"IMU rate: {rate:.0f} Hz | "
                f"packets: {self.pkt_count} | errors: {self.err_count}"
            )
            self.pkt_count = 0
            self.err_count = 0
            self.last_stats = now

    def _publish(self, pkt: dict):
        """Convert raw sensor values to ROS Imu messages."""
        stamp = self.get_clock().now().to_msg()

        # ── Head IMU ─────────────────────────────────────────────────
        head_msg = Imu()
        head_msg.header.stamp = stamp
        head_msg.header.frame_id = "head_imu_link"

        # Gyro: raw / scale → rad/s
        # MPU9250 axes: x-right, y-forward, z-up (when chip is flat)
        # ROS standard:  x-forward, y-left, z-up
        # Remap: ros_x = -mpu_y, ros_y = mpu_x, ros_z = mpu_z
        hgx_raw, hgy_raw, hgz_raw = pkt["head_gyro"]
        head_msg.angular_velocity.x = -hgy_raw / self.gyro_scale
        head_msg.angular_velocity.y =  hgx_raw / self.gyro_scale
        head_msg.angular_velocity.z =  hgz_raw / self.gyro_scale

        # Accel: raw / scale → m/s²
        hax_raw, hay_raw, haz_raw = pkt["head_accel"]
        head_msg.linear_acceleration.x = -hay_raw / self.accel_scale * G_STD
        head_msg.linear_acceleration.y =  hax_raw / self.accel_scale * G_STD
        head_msg.linear_acceleration.z =  haz_raw / self.accel_scale * G_STD

        # Covariance: unknown — set to -1 (not available)
        head_msg.orientation_covariance[0] = -1.0
        head_msg.angular_velocity_covariance[0] = 0.01
        head_msg.angular_velocity_covariance[4] = 0.01
        head_msg.angular_velocity_covariance[8] = 0.01
        head_msg.linear_acceleration_covariance[0] = 0.01
        head_msg.linear_acceleration_covariance[4] = 0.01
        head_msg.linear_acceleration_covariance[8] = 0.01

        self.head_pub.publish(head_msg)

        # ── Arm IMU ──────────────────────────────────────────────────
        arm_msg = Imu()
        arm_msg.header.stamp = stamp
        arm_msg.header.frame_id = "arm_imu_link"

        agx_raw, agy_raw, agz_raw = pkt["arm_gyro"]
        arm_msg.angular_velocity.x = -agy_raw / self.gyro_scale
        arm_msg.angular_velocity.y =  agx_raw / self.gyro_scale
        arm_msg.angular_velocity.z =  agz_raw / self.gyro_scale

        aax_raw, aay_raw, aaz_raw = pkt["arm_accel"]
        arm_msg.linear_acceleration.x = -aay_raw / self.accel_scale * G_STD
        arm_msg.linear_acceleration.y =  aax_raw / self.accel_scale * G_STD
        arm_msg.linear_acceleration.z =  aaz_raw / self.accel_scale * G_STD

        arm_msg.orientation_covariance[0] = -1.0
        arm_msg.angular_velocity_covariance[0] = 0.01
        arm_msg.angular_velocity_covariance[4] = 0.01
        arm_msg.angular_velocity_covariance[8] = 0.01
        arm_msg.linear_acceleration_covariance[0] = 0.01
        arm_msg.linear_acceleration_covariance[4] = 0.01
        arm_msg.linear_acceleration_covariance[8] = 0.01

        self.arm_pub.publish(arm_msg)

    def close(self):
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()


def main():
    rclpy.init()
    node = IMUBridgeNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            node.spin_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
