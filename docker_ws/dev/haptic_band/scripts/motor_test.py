"""
motor_test.py

Activates each of the 8 haptic motors one at a time.
For each motor:
  - Ramp intensity 0 → 100 % in steps of 10 (≈ 0.1 s per step → ~1 s ramp-up)
  - Hold at 100 % briefly
  - Turn off before moving to the next motor

Usage (inside the haptic_band or haptic_band_test container):
    python3 /scripts/motor_test.py
"""

import time
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

NUM_MOTORS = 8
RAMP_STEP = 10          # % per step
STEP_DURATION = 0.1     # seconds per ramp step  (10 steps → ~1 s ramp)
HOLD_DURATION = 0.2     # seconds at full intensity before turning off
PAUSE_BETWEEN = 0.3     # seconds of silence between motors


def make_msg(motor_index: int, intensity: float) -> Float32MultiArray:
    msg = Float32MultiArray()
    msg.data = [0.0] * NUM_MOTORS
    msg.data[motor_index] = float(intensity)
    return msg


def main():
    rclpy.init()
    node = Node('haptic_motor_test')
    pub = node.create_publisher(Float32MultiArray, '/haptic_band/motors', 10)

    # Give the publisher a moment to connect to subscribers
    time.sleep(0.5)

    node.get_logger().info('Starting motor sweep test (motors 1 – 8)')

    try:
        for motor in range(NUM_MOTORS):
            node.get_logger().info(f'Motor {motor + 1} / {NUM_MOTORS}')

            # Ramp up 0 → 100 %
            for intensity in range(0, 101, RAMP_STEP):
                pub.publish(make_msg(motor, intensity))
                time.sleep(STEP_DURATION)

            # Hold at 100 %
            time.sleep(HOLD_DURATION)

            # Turn off
            pub.publish(make_msg(motor, 0.0))
            time.sleep(PAUSE_BETWEEN)

        node.get_logger().info('Motor sweep complete.')
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure all motors off on exit
        all_off = Float32MultiArray()
        all_off.data = [0.0] * NUM_MOTORS
        pub.publish(all_off)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
