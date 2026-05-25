#!/usr/bin/env python3
"""Wrist motor test script.

Moves the wrist motor slowly (2.5 deg/s) in one direction for 1 second,
then in the opposite direction for 1 second, while logging encoder values.

Usage:
    python3 wrist_test.py [--port /dev/ttyUSB0] [--output logs/wrist/test.log]
"""

import argparse
import os
import sys
import time
from datetime import datetime

try:
    from dynamixel_sdk import (
        PortHandler,
        PacketHandler,
        COMM_SUCCESS,
    )
except ImportError:
    print("ERROR: dynamixel-sdk not installed. Install with: pip install dynamixel-sdk")
    sys.exit(1)

# Control table addresses for Dynamixel X series
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# Dynamixel position range: 0-4095 maps to 0-360 degrees
DXL_POSITION_RANGE = 4095.0
DXL_ANGLE_RANGE = 360.0

# Velocity conversion: Dynamixel velocity units to deg/s
# For X-series: 1 unit = 0.229 RPM = 0.229 * 6 = 1.374 deg/s (approx)
VELOCITY_TO_DEG_S = 0.229 * 6.0

# Test parameters
TEST_VELOCITY_DEG_S = 2.5  # degrees per second
TEST_DURATION_S = 1.0  # seconds per direction


def deg_to_dx(deg):
    return int((deg % 360.0) / DXL_ANGLE_RANGE * DXL_POSITION_RANGE)


def dx_to_deg(dx):
    return float(dx) / DXL_POSITION_RANGE * DXL_ANGLE_RANGE


def velocity_deg_s_to_dx(deg_s):
    # Convert deg/s to Dynamixel velocity units
    # 1 unit = 0.229 RPM = 0.229 * 360 / 60 = 1.374 deg/s
    raw = int(deg_s / VELOCITY_TO_DEG_S)
    # Convert to unsigned 32-bit for SDK
    return raw & 0xFFFFFFFF


def main():
    parser = argparse.ArgumentParser(description='Wrist motor test')
    parser.add_argument('--port', default=os.environ.get('WRIST_SERIAL_PORT', '/dev/ttyUSB0'),
                        help='Serial port (default: /dev/ttyUSB0 or WRIST_SERIAL_PORT env)')
    parser.add_argument('--baud', type=int, default=57600, help='Baudrate (default: 57600)')
    parser.add_argument('--motor-id', type=int, default=1, help='Motor ID (default: 1)')
    parser.add_argument('--output', default='logs/wrist/test.log', help='Output log file')
    args = parser.parse_args()

    # Create output directory
    os.makedirs(os.path.dirname(args.output), exist_ok=True)

    log_lines = []

    def log(msg):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        line = f"[{timestamp}] {msg}"
        print(line)
        log_lines.append(line)

    log(f"Starting wrist test")
    log(f"Port: {args.port}, Baud: {args.baud}, Motor ID: {args.motor_id}")

    # Open port
    port_handler = PortHandler(args.port)
    packet_handler = PacketHandler(2.0)

    if not port_handler.openPort():
        log(f"ERROR: Failed to open port {args.port}")
        sys.exit(1)

    if not port_handler.setBaudRate(args.baud):
        log(f"ERROR: Failed to set baudrate {args.baud}")
        sys.exit(1)

    log(f"Port opened successfully")

    # Enable torque
    dxl_comm, dxl_error = packet_handler.write1ByteTxRx(
        port_handler, args.motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm != COMM_SUCCESS:
        log(f"ERROR: Failed to enable torque: {packet_handler.getTxRxResult(dxl_comm)}")
        port_handler.closePort()
        sys.exit(1)

    log(f"Torque enabled")

    # Read initial position
    dxl_pos, comm_result = packet_handler.read4ByteTxRx(
        port_handler, args.motor_id, ADDR_PRESENT_POSITION)
    if comm_result == COMM_SUCCESS:
        initial_pos = dxl_pos
        log(f"Initial position: {initial_pos} ({dx_to_deg(initial_pos):.2f} deg)")
    else:
        log(f"WARNING: Could not read initial position: {packet_handler.getTxRxResult(comm_result)}")
        initial_pos = 0

    # Set velocity mode by writing goal velocity
    # First, disable position control by setting velocity
    test_vel_dx = velocity_deg_s_to_dx(TEST_VELOCITY_DEG_S)
    log(f"Test velocity: {TEST_VELOCITY_DEG_S} deg/s (Dynamixel units: {test_vel_dx})")

    # Set velocity control mode
    # For X-series, we can use velocity control mode (1) or position control mode (3)
    # Let's use position control with profile velocity for smooth movement
    ADDR_OPERATING_MODE = 11
    VELOCITY_CONTROL_MODE = 1
    POSITION_CONTROL_MODE = 3

    log(f"Setting operating mode to velocity control")
    dxl_comm, dxl_error = packet_handler.write1ByteTxRx(
        port_handler, args.motor_id, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
    if dxl_comm != COMM_SUCCESS:
        log(f"ERROR: Failed to set velocity control mode: {packet_handler.getTxRxResult(dxl_comm)}")
    else:
        log(f"Velocity control mode set")

    # Test 1: Move in positive direction
    log(f"\n=== Test 1: Moving +{TEST_VELOCITY_DEG_S} deg/s for {TEST_DURATION_S}s ===")
    dxl_comm, dxl_error = packet_handler.write4ByteTxRx(
        port_handler, args.motor_id, ADDR_GOAL_VELOCITY, test_vel_dx)
    if dxl_comm != COMM_SUCCESS:
        log(f"ERROR: Failed to set velocity: {packet_handler.getTxRxResult(dxl_comm)}")
    else:
        log(f"Velocity set to +{test_vel_dx}")

    start_time = time.time()
    sample_count = 0
    while time.time() - start_time < TEST_DURATION_S:
        elapsed = time.time() - start_time
        dxl_pos, comm_result = packet_handler.read4ByteTxRx(
            port_handler, args.motor_id, ADDR_PRESENT_POSITION)
        dxl_vel, comm_result_v = packet_handler.read4ByteTxRx(
            port_handler, args.motor_id, ADDR_PRESENT_VELOCITY)

        pos_deg = dx_to_deg(dxl_pos) if comm_result == COMM_SUCCESS else None
        vel_deg = float(dxl_vel) * VELOCITY_TO_DEG_S if comm_result_v == COMM_SUCCESS else None

        log(f"  t={elapsed:.3f}s pos={pos_deg:.2f} deg vel={vel_deg:.2f} deg/s")
        sample_count += 1
        time.sleep(0.1)

    log(f"Test 1 complete: {sample_count} samples")

    # Stop motor
    dxl_comm, dxl_error = packet_handler.write4ByteTxRx(
        port_handler, args.motor_id, ADDR_GOAL_VELOCITY, 0)
    time.sleep(0.1)

    # Read position after test 1
    dxl_pos, comm_result = packet_handler.read4ByteTxRx(
        port_handler, args.motor_id, ADDR_PRESENT_POSITION)
    if comm_result == COMM_SUCCESS:
        log(f"Position after test 1: {dx_to_deg(dxl_pos):.2f} deg")

    # Test 2: Move in negative direction
    log(f"\n=== Test 2: Moving -{TEST_VELOCITY_DEG_S} deg/s for {TEST_DURATION_S}s ===")
    neg_vel_dx = velocity_deg_s_to_dx(-TEST_VELOCITY_DEG_S)
    dxl_comm, dxl_error = packet_handler.write4ByteTxRx(
        port_handler, args.motor_id, ADDR_GOAL_VELOCITY, neg_vel_dx)
    if dxl_comm != COMM_SUCCESS:
        log(f"ERROR: Failed to set velocity: {packet_handler.getTxRxResult(dxl_comm)}")
    else:
        log(f"Velocity set to -{test_vel_dx} (unsigned: {neg_vel_dx})")

    start_time = time.time()
    sample_count = 0
    while time.time() - start_time < TEST_DURATION_S:
        elapsed = time.time() - start_time
        dxl_pos, comm_result = packet_handler.read4ByteTxRx(
            port_handler, args.motor_id, ADDR_PRESENT_POSITION)
        dxl_vel, comm_result_v = packet_handler.read4ByteTxRx(
            port_handler, args.motor_id, ADDR_PRESENT_VELOCITY)

        pos_deg = dx_to_deg(dxl_pos) if comm_result == COMM_SUCCESS else None
        vel_deg = float(dxl_vel) * VELOCITY_TO_DEG_S if comm_result_v == COMM_SUCCESS else None

        log(f"  t={elapsed:.3f}s pos={pos_deg:.2f} deg vel={vel_deg:.2f} deg/s")
        sample_count += 1
        time.sleep(0.1)

    log(f"Test 2 complete: {sample_count} samples")

    # Stop motor
    dxl_comm, dxl_error = packet_handler.write4ByteTxRx(
        port_handler, args.motor_id, ADDR_GOAL_VELOCITY, 0)
    time.sleep(0.1)

    # Read final position
    dxl_pos, comm_result = packet_handler.read4ByteTxRx(
        port_handler, args.motor_id, ADDR_PRESENT_POSITION)
    if comm_result == COMM_SUCCESS:
        final_pos = dx_to_deg(dxl_pos)
        log(f"Final position: {final_pos:.2f} deg")
        log(f"Net displacement: {final_pos - dx_to_deg(initial_pos):.2f} deg")

    # Disable torque
    packet_handler.write1ByteTxRx(
        port_handler, args.motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    port_handler.closePort()

    # Write log file
    with open(args.output, 'w') as f:
        f.write('\n'.join(log_lines) + '\n')

    log(f"\nLog saved to: {args.output}")
    log(f"Test complete")


if __name__ == '__main__':
    main()
