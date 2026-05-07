#!/usr/bin/env python3
"""
Force Controller Node for Mia Hand.

Regulates grasp force by reading finger force feedback from the Mia Hand
and adjusting motor commands to maintain a target force range.

Subscribes:
    /hand/forces          (Float32MultiArray) - Finger force readings [thumb, index, middle]
    /pipeline/state       (Int32)             - Pipeline state from pipeline_manager
    /hand/grasp_commands  (Float32MultiArray) - Desired motor positions from preshaping

Publishes:
    /hand/motor_commands  (Float32MultiArray) - Adjusted motor positions with force regulation

Parameters:
    target_force          (double) - Target grasp force in Newtons (default: 2.0)
    force_tolerance       (double) - Acceptable deviation from target (default: 0.5)
    max_force             (double) - Maximum allowed force before emergency release (default: 8.0)
    adjustment_rate       (double) - How fast to adjust motor position per tick (default: 0.02)
    min_motor_pos         (double) - Minimum motor position (fully open) (default: 0.0)
    max_motor_pos         (double) - Maximum motor position (fully closed) (default: 1.0)
    control_hz            (double) - Control loop frequency (default: 50.0)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray
import numpy as np


# Pipeline states (must match pipeline_manager)
STATE_IDLE = 0
STATE_APPROACHING = 3
STATE_GRASPING = 4
STATE_HOLDING = 5
STATE_RELEASING = 6


class ForceControllerNode(Node):
    def __init__(self):
        super().__init__("force_controller")

        # Parameters
        self.declare_parameter("target_force", 2.0)
        self.declare_parameter("force_tolerance", 0.5)
        self.declare_parameter("max_force", 8.0)
        self.declare_parameter("adjustment_rate", 0.02)
        self.declare_parameter("min_motor_pos", 0.0)
        self.declare_parameter("max_motor_pos", 1.0)
        self.declare_parameter("control_hz", 50.0)

        self.target_force = self.get_parameter("target_force").value
        self.force_tolerance = self.get_parameter("force_tolerance").value
        self.max_force = self.get_parameter("max_force").value
        self.adjustment_rate = self.get_parameter("adjustment_rate").value
        self.min_motor_pos = self.get_parameter("min_motor_pos").value
        self.max_motor_pos = self.get_parameter("max_motor_pos").value
        control_hz = self.get_parameter("control_hz").value

        # State
        self.pipeline_state = STATE_IDLE
        self.current_forces = np.zeros(3)  # [thumb, index, middle]
        self.desired_motor_pos = np.zeros(3)
        self.adjusted_motor_pos = np.zeros(3)
        self.force_exceeded = False

        # Subscribers
        self.force_sub = self.create_subscription(
            Float32MultiArray,
            "/hand/forces",
            self._on_forces,
            10,
        )
        self.state_sub = self.create_subscription(
            Int32,
            "/pipeline/state",
            self._on_pipeline_state,
            10,
        )
        self.grasp_cmd_sub = self.create_subscription(
            Float32MultiArray,
            "/hand/grasp_commands",
            self._on_grasp_commands,
            10,
        )

        # Publisher
        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            "/hand/motor_commands",
            10,
        )

        # Control timer
        self.control_timer = self.create_timer(
            1.0 / control_hz, self._control_tick
        )

        self.get_logger().info(
            f"Force controller started. Target: {self.target_force}N, "
            f"Tolerance: +/-{self.force_tolerance}N, "
            f"Max: {self.max_force}N"
        )

    def _on_forces(self, msg: Float32MultiArray):
        """Update current force readings."""
        forces = np.array(msg.data)
        if len(forces) >= 3:
            self.current_forces = forces[:3]

        # Emergency: if any finger exceeds max force, flag it
        if np.any(self.current_forces > self.max_force):
            if not self.force_exceeded:
                self.get_logger().warn(
                    f"Force exceeded max! Forces: {self.current_forces}"
                )
                self.force_exceeded = True
        else:
            self.force_exceeded = False

    def _on_pipeline_state(self, msg: Int32):
        """Track pipeline state."""
        old_state = self.pipeline_state
        self.pipeline_state = msg.data
        if old_state != self.pipeline_state:
            self.get_logger().info(
                f"Pipeline state: {old_state} -> {self.pipeline_state}"
            )

    def _on_grasp_commands(self, msg: Float32MultiArray):
        """Receive desired motor positions from preshaping."""
        positions = np.array(msg.data)
        if len(positions) >= 3:
            self.desired_motor_pos = positions[:3]

    def _control_tick(self):
        """Main control loop - adjust motor positions based on force feedback."""
        # Only regulate force during GRASPING and HOLDING states
        if self.pipeline_state not in (STATE_GRASPING, STATE_HOLDING):
            # Pass through desired positions unchanged
            self.adjusted_motor_pos = self.desired_motor_pos.copy()
            return

        # Emergency release if force exceeded
        if self.force_exceeded:
            self.get_logger().warn("Emergency force release - opening fingers")
            # Back off motor positions
            self.adjusted_motor_pos = np.clip(
                self.desired_motor_pos - 0.1,
                self.min_motor_pos,
                self.max_motor_pos,
            )
            self._publish_motor_commands()
            return

        # Force regulation for each finger
        adjustments = np.zeros(3)
        for i in range(3):
            force_error = self.current_forces[i] - self.target_force

            if abs(force_error) > self.force_tolerance:
                if force_error > 0:
                    # Too much force - open finger slightly
                    adjustments[i] = -self.adjustment_rate * min(
                        force_error / self.target_force, 2.0
                    )
                else:
                    # Not enough force - close finger slightly
                    adjustments[i] = self.adjustment_rate * min(
                        abs(force_error) / self.target_force, 2.0
                    )

        self.adjusted_motor_pos = np.clip(
            self.desired_motor_pos + adjustments,
            self.min_motor_pos,
            self.max_motor_pos,
        )

        self._publish_motor_commands()

    def _publish_motor_commands(self):
        """Publish adjusted motor commands."""
        msg = Float32MultiArray()
        msg.data = self.adjusted_motor_pos.tolist()
        self.motor_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ForceControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
