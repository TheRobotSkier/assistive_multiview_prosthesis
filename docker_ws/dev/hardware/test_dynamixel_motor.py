#!/usr/bin/env python3
"""
Standalone Dynamixel motor test script for hardware debugging.

This script tests basic communication with a Dynamixel motor using the SDK
without requiring ROS2 or ros2_control. It's useful for verifying:
- USB serial connection
- Motor responds to ping
- Can read/write basic registers
- Motor moves in position and velocity mode

Usage:
    python3 test_dynamixel_motor.py [--port /dev/ttyUSB0] [--baud 57600] [--motor-id 1]
"""

import argparse
import sys
import time

try:
    from dynamixel_sdk import *  # Uses Dynamixel SDK library
except ImportError:
    print("ERROR: dynamixel_sdk not found. Please install it:")
    print("  pip3 install dynamixel-sdk")
    sys.exit(1)


def test_motor_connection(port_name, baudrate, motor_id):
    """Test basic connection and communication with Dynamixel motor."""
    print(f"Testing Dynamixel motor on {port_name} at {baudrate} baud, ID {motor_id}")
    
    # Initialize PortHandler instance
    portHandler = PortHandler(port_name)
    
    # Initialize PacketHandler instance
    packetHandler = PacketHandler(2.0)  # Protocol version 2.0
    
    # Open port
    if portHandler.openPort():
        print(f"SUCCESS: Port {port_name} opened")
    else:
        print(f"ERROR: Failed to open port {port_name}")
        return False
    
    # Set port baudrate
    if portHandler.setBaudRate(baudrate):
        print(f"SUCCESS: Baudrate set to {baudrate}")
    else:
        print(f"ERROR: Failed to set baudrate to {baudrate}")
        portHandler.closePort()
        return False
    
    # Try to ping the motor. Different SDK versions return either:
    #   (comm_result, error) or (model_number, comm_result, error)
    ping_result = packetHandler.ping(portHandler, motor_id)
    if len(ping_result) == 3:
        dxl_model_number, dxl_comm_result, dxl_error = ping_result
    elif len(ping_result) == 2:
        dxl_comm_result, dxl_error = ping_result
        dxl_model_number = None
    else:
        print(f"ERROR: Unexpected ping response format: {ping_result}")
        portHandler.closePort()
        return False

    if dxl_comm_result == COMM_SUCCESS:
        print(f"SUCCESS: Motor ID {motor_id} responded to ping")
        if dxl_model_number is not None:
            print(f"  Model number: {dxl_model_number}")
    else:
        print(f"ERROR: Motor ID {motor_id} did not respond to ping")
        print(f"  Communication result: {packetHandler.getTxRxResult(dxl_comm_result)}")
        print(f"  Hardware error: {packetHandler.getRxPacketError(dxl_error)}")
        portHandler.closePort()
        return False
    
    # Read present position
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
        portHandler, motor_id, ADDR_PRESENT_POSITION)
    if dxl_comm_result == COMM_SUCCESS:
        print(f"SUCCESS: Present position = {dxl_present_position}")
    else:
        print(f"ERROR: Failed to read present position")
        print(f"  Communication result: {packetHandler.getTxRxResult(dxl_comm_result)}")
        print(f"  Hardware error: {packetHandler.getRxPacketError(dxl_error)}")
    
    # Read present velocity
    dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
        portHandler, motor_id, ADDR_PRESENT_VELOCITY)
    if dxl_comm_result == COMM_SUCCESS:
        print(f"SUCCESS: Present velocity = {dxl_present_velocity}")
    else:
        print(f"ERROR: Failed to read present velocity")
        print(f"  Communication result: {packetHandler.getTxRxResult(dxl_comm_result)}")
        print(f"  Hardware error: {packetHandler.getRxPacketError(dxl_error)}")
    
    # Test torque enable
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result == COMM_SUCCESS:
        print("SUCCESS: Torque enabled")
    else:
        print(f"ERROR: Failed to enable torque")
        print(f"  Communication result: {packetHandler.getTxRxResult(dxl_comm_result)}")
        print(f"  Hardware error: {packetHandler.getRxPacketError(dxl_error)}")
    
    # Test position control (move to middle position)
    print("Testing position control: moving to position 2048 (middle)...")
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, motor_id, ADDR_GOAL_POSITION, 2048) # Maybe not working
    if dxl_comm_result == COMM_SUCCESS:
        print("SUCCESS: Goal position set to 2048")
        time.sleep(2)  # Wait for movement
        
        # Read back position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
            portHandler, motor_id, ADDR_PRESENT_POSITION)
        if dxl_comm_result == COMM_SUCCESS:
            print(f"Present position after move: {dxl_present_position}")
        else:
            print(f"ERROR: Failed to read position after move")
    else:
        print(f"ERROR: Failed to set goal position")
        print(f"  Communication result: {packetHandler.getTxRxResult(dxl_comm_result)}")
        print(f"  Hardware error: {packetHandler.getRxPacketError(dxl_error)}")
    
    # Test velocity control
    print("Testing velocity control: setting velocity to 100...")
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, motor_id, ADDR_GOAL_VELOCITY, -20) # Tried -100, 0, and 100 with success
    if dxl_comm_result == COMM_SUCCESS:
        print("SUCCESS: Goal velocity set to 100")
        time.sleep(2)  # Wait for movement
        
        # Read back velocity
        dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
            portHandler, motor_id, ADDR_PRESENT_VELOCITY)
        if dxl_comm_result == COMM_SUCCESS:
            print(f"Present velocity after move: {dxl_present_velocity}")
        else:
            print(f"ERROR: Failed to read velocity after move")
    else:
        print(f"ERROR: Failed to set goal velocity")
        print(f"  Communication result: {packetHandler.getTxRxResult(dxl_comm_result)}")
        print(f"  Hardware error: {packetHandler.getRxPacketError(dxl_error)}")
    
    # Disable torque for safety
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result == COMM_SUCCESS:
        print("SUCCESS: Torque disabled (motor is now free to rotate)")
    else:
        print(f"WARNING: Failed to disable torque")
        print(f"  Communication result: {packetHandler.getTxRxResult(dxl_comm_result)}")
        print(f"  Hardware error: {packetHandler.getRxPacketError(dxl_error)}")
    
    # Close port
    portHandler.closePort()
    print("Port closed")
    return True


# Control table addresses for Dynamixel X series (adjust if using different model)
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_GOAL_VELOCITY          = 104
ADDR_PRESENT_VELOCITY       = 128
ADDR_PROFILE_ACCELERATION   = 108
ADDR_PROFILE_VELOCITY       = 112

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0         # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4095      # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20        # Dynamixel moving status threshold


def main():
    parser = argparse.ArgumentParser(description='Test Dynamixel motor communication')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                        help='Serial port device (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=57600,
                        help='Baud rate (default: 57600)')
    parser.add_argument('--motor-id', type=int, default=1,
                        help='Motor ID (default: 1)')
    parser.add_argument('--timeout', type=float, default=2.0,
                        help='Response timeout in seconds (default: 2.0)')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("Dynamixel Motor Test Script")
    print("=" * 60)
    print(f"Port: {args.port}")
    print(f"Baud rate: {args.baud}")
    print(f"Motor ID: {args.motor_id}")
    print(f"Timeout: {args.timeout}s")
    print("=" * 60)
    
    success = test_motor_connection(args.port, args.baud, args.motor_id)
    
    print("=" * 60)
    if success:
        print("RESULT: Motor test completed successfully!")
        print("The Dynamixel motor is communicating properly.")
    else:
        print("RESULT: Motor test FAILED!")
        print("Please check:")
        print("  1. Motor is powered and connected")
        print("  2. Correct serial port specified")
        print("  3. Correct baud rate (should be 57600 for wrist motor)")
        print("  4. Motor ID matches (should be 1 for wrist motor)")
        print("  5. No other device is using the same serial port")
    print("=" * 60)
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())