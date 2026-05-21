#!/usr/bin/env python3
"""EMG volitional hand controller node.

Maps EMG gestures directly to joint velocities for real-time opening/closing
of the Mia Hand. Displays the hand in RViz (launched separately).

Usage:
  ros2 launch prosthesis_launch emg_volitional_test.launch.py
  # Or standalone:
  python3 emg_volitional_node.py --ros-args -p config_path:=/path/to/config.yaml
"""

import os
import sys
from typing import Dict, List, Optional

import yaml

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, Int32, Float32
    from sensor_msgs.msg import JointState
except ModuleNotFoundError:
    class Node:  # type: ignore[misc]
        pass

    class Float64MultiArray:  # type: ignore[no-redef]
        pass

    class Int32:  # type: ignore[no-redef]
        def __init__(self, data=0):
            self.data = data

    class Float32:  # type: ignore[no-redef]
        def __init__(self, data=0.0):
            self.data = data

    class JointState:  # type: ignore[no-redef]
        pass

FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_COUNT = 3


class EmgVolitionalNode(Node):
    def __init__(self):
        super().__init__("emg_volitional_controller")

        # -- Load config ------------------------------------------------------
        config_path = self.declare_parameter(
            "config_path", "/prosthesis_ws/tests/emg_volitional/emg_volitional_config.yaml"
        ).value
        with open(config_path) as f:
            self._cfg = yaml.safe_load(f)

        # -- Parameters from config -------------------------------------------
        self._gesture_velocities: Dict[int, float] = {
            int(k): float(v) for k, v in self._cfg["gesture_velocities"].items()
        }
        self._confidence_threshold = float(self._cfg["confidence_threshold"])
        self._position_min = float(self._cfg["position_limits"]["min"])
        self._position_max = float(self._cfg["position_limits"]["max"])
        self._emergency_stop_gesture = int(self._cfg["emergency_stop_gesture"])
        self._command_rate = float(self._cfg["command_rate_hz"])

        # -- Publishers ---------------------------------------------------------
        self._vel_pub = self.create_publisher(
            Float64MultiArray, "/group_vel_ff_controller/commands", 10)

        # -- Subscribers --------------------------------------------------------
        self._positions = [0.0] * FINGER_COUNT
        self._got_js = False
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        self._emg_gesture = 0
        self._emg_confidence = 0.0
        self.create_subscription(
            Int32, self._cfg["emg_gesture_topic"], self._on_gesture, 10)
        self.create_subscription(
            Float32, self._cfg["emg_confidence_topic"], self._on_confidence, 10)

        # -- Timer --------------------------------------------------------------
        self._timer = self.create_timer(1.0 / self._command_rate, self._control_loop)

        self.get_logger().info(
            "EMG Volitional Controller started -- "
            f"rate={self._command_rate} Hz, "
            f"confidence_threshold={self._confidence_threshold}"
        )

    # -- Callbacks ------------------------------------------------------------

    def _on_joint_states(self, msg: JointState):
        try:
            for i, jname in enumerate(FINGER_JOINTS):
                idx = msg.name.index(jname)
                self._positions[i] = float(msg.position[idx])
            self._got_js = True
        except ValueError:
            pass

    def _on_gesture(self, msg: Int32):
        self._emg_gesture = msg.data

    def _on_confidence(self, msg: Float32):
        self._emg_confidence = msg.data

    # -- Helpers --------------------------------------------------------------

    @staticmethod
    def lookup_velocity(
        gesture: int,
        confidence: float,
        gesture_velocities: Dict[int, float],
        confidence_threshold: float,
        emergency_stop_gesture: int,
    ) -> float:
        """Return commanded velocity for the current gesture.

        Rules:
          - Emergency stop gesture always returns 0.0
          - Confidence below threshold returns 0.0
          - Unknown gesture ID returns 0.0
        """
        if gesture == emergency_stop_gesture:
            return 0.0
        if confidence < confidence_threshold:
            return 0.0
        return gesture_velocities.get(gesture, 0.0)

    @staticmethod
    def clamp_velocity_by_position_limits(
        velocity: float,
        positions: List[float],
        pos_min: float,
        pos_max: float,
    ) -> float:
        """Clamp velocity to prevent exceeding joint position limits.

        If any finger is at or beyond max and velocity is positive, return 0.0.
        If any finger is at or below min and velocity is negative, return 0.0.
        """
        if velocity > 0.0:
            if any(p >= pos_max for p in positions):
                return 0.0
        elif velocity < 0.0:
            if any(p <= pos_min for p in positions):
                return 0.0
        return velocity

    def _publish_velocity(self, v: float):
        msg = Float64MultiArray()
        msg.data = [v, v, v]
        self._vel_pub.publish(msg)

    # -- Control loop ---------------------------------------------------------

    def _control_loop(self):
        if not self._got_js:
            self.get_logger().warn("No joint states yet -- skipping cycle")
            return

        raw_velocity = self.lookup_velocity(
            self._emg_gesture,
            self._emg_confidence,
            self._gesture_velocities,
            self._confidence_threshold,
            self._emergency_stop_gesture,
        )

        clamped_velocity = self.clamp_velocity_by_position_limits(
            raw_velocity,
            self._positions,
            self._position_min,
            self._position_max,
        )

        if raw_velocity != clamped_velocity:
            self.get_logger().warn(
                f"Velocity clamped: {raw_velocity:.3f} -> {clamped_velocity:.3f} "
                f"(positions={[f'{p:.2f}' for p in self._positions]})"
            )

        self._publish_velocity(clamped_velocity)

        # Log on change (throttled by rate)
        if not hasattr(self, "_last_logged_velocity"):
            self._last_logged_velocity = None
        if clamped_velocity != self._last_logged_velocity:
            self.get_logger().info(
                f"Gesture={self._emg_gesture} conf={self._emg_confidence:.2f} "
                f"-> velocity={clamped_velocity:.3f} rad/s"
            )
            self._last_logged_velocity = clamped_velocity


def main(args=None):
    rclpy.init(args=args)
    node = EmgVolitionalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_velocity(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
