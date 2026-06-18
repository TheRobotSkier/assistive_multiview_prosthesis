#!/usr/bin/env python3
"""Simulator health test.

Spins up the ``HandSimulatorNode`` in-process via rclpy, subscribes to
each topic the simulator publishes, and asserts each publishes within
2s.  Then publishes a velocity command and asserts ``/hand_sim/forces``
rises above 0 within 1s and ``/wrist/state`` reflects the commanded
position.

This test does NOT require a full launch graph — it exercises the
simulator node in isolation.  The offline PTY suite (mvp-1dc.2) exercises
the full graph end-to-end.
"""

from __future__ import annotations

import os
import sys
import threading
import time
import unittest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)


# These imports require rclpy and the mia_hand_msgs package.  Skip the
# test gracefully if they are not available (e.g. local dev without the
# Docker container).
try:
    import rclpy  # noqa: F401
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32MultiArray, Float64MultiArray
    from mia_hand_msgs.msg import ForceData, JointData, MotorData

    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False

from scripts.mia_haptic_force_test.common.constants import (
    FINGER_COUNT,
    TOPIC_HW_FINGER_FORCES,
    TOPIC_HW_JOINT_POSITIONS,
    TOPIC_HW_JOINT_SPEEDS,
    TOPIC_HW_MOTOR_CURRENTS,
    TOPIC_HW_MOTOR_POSITIONS,
    TOPIC_HW_MOTOR_SPEEDS,
)


@unittest.skipUnless(_HAS_ROS, "rclpy/mia_hand_msgs not available")
class SimulatorHealthTest(unittest.TestCase):
    """Spin up the simulator node in-process and verify all topics publish."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.executor = SingleThreadedExecutor()

    @classmethod
    def tearDownClass(cls) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self) -> None:
        from scripts.mia_haptic_force_test.hand_simulator_node import (
            HandSimulatorNode,
        )

        self._node = HandSimulatorNode(
            config_path=os.path.join(_REPO_ROOT, "config", "mia_haptic_force_test.yaml")
        )
        self.executor.add_node(self._node)
        self._received: dict[str, list] = {topic: [] for topic in self._all_topics()}
        self._locks: dict[str, threading.Lock] = {
            topic: threading.Lock() for topic in self._all_topics()
        }
        self._events: dict[str, threading.Event] = {
            topic: threading.Event() for topic in self._all_topics()
        }
        self._make_subscribers()

    def tearDown(self) -> None:
        self.executor.remove_node(self._node)
        self._node.destroy_node()

    def _all_topics(self) -> list[str]:
        return [
            TOPIC_HW_FINGER_FORCES,
            TOPIC_HW_MOTOR_POSITIONS,
            TOPIC_HW_MOTOR_SPEEDS,
            TOPIC_HW_MOTOR_CURRENTS,
            TOPIC_HW_JOINT_POSITIONS,
            TOPIC_HW_JOINT_SPEEDS,
            "data_streams/joints/efforts/data",
            "/wrist/state",
            "/hand_sim/joint_states",
            "/hand_sim/forces",
        ]

    def _make_subscribers(self) -> None:
        from rclpy.node import Node

        helper = Node("sim_health_helper")
        self.executor.add_node(helper)
        self._helper = helper
        msg_types = {
            TOPIC_HW_FINGER_FORCES: ForceData,
            TOPIC_HW_MOTOR_POSITIONS: MotorData,
            TOPIC_HW_MOTOR_SPEEDS: MotorData,
            TOPIC_HW_MOTOR_CURRENTS: MotorData,
            TOPIC_HW_JOINT_POSITIONS: JointData,
            TOPIC_HW_JOINT_SPEEDS: JointData,
            "data_streams/joints/efforts/data": JointData,
            "/wrist/state": Float64MultiArray,
            "/hand_sim/joint_states": JointState,
            "/hand_sim/forces": Float32MultiArray,
        }
        for topic, msg_type in msg_types.items():
            helper.create_subscription(
                msg_type, topic, self._make_callback(topic), 10
            )

    def _make_callback(self, topic: str):
        def cb(msg) -> None:
            with self._locks[topic]:
                self._received[topic].append(msg)
            self._events[topic].set()
        return cb

    def _spin_until(self, topic: str, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            if self._events[topic].is_set():
                return True
        return False

    def test_all_topics_publish_within_2s(self) -> None:
        for topic in self._all_topics():
            with self.subTest(topic=topic):
                self.assertTrue(
                    self._spin_until(topic, 2.0),
                    f"{topic} did not publish within 2s",
                )

    def test_velocity_command_makes_force_rise(self) -> None:
        # Wait for the simulator to publish at least one frame.
        self.assertTrue(self._spin_until("/hand_sim/forces", 2.0))
        # Clear the event so we can detect a new message.
        self._events["/hand_sim/forces"].clear()
        # Publish a velocity command that will move the fingers past the contact point.
        pub = self._node.create_publisher(
            Float64MultiArray, "/group_vel_ff_controller/commands", 10
        )
        msg = Float64MultiArray()
        msg.data = [0.1, 0.1, 0.1]
        for _ in range(20):
            pub.publish(msg)
            self.executor.spin_once(timeout_sec=0.05)
        # Spin for up to 2s and collect messages.
        deadline = time.monotonic() + 2.0
        max_force = 0.0
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            with self._locks["/hand_sim/forces"]:
                if self._received["/hand_sim/forces"]:
                    for m in self._received["/hand_sim/forces"]:
                        if m.data:
                            max_force = max(max_force, max(m.data))
        self.assertGreater(max_force, 0.0, "expected /hand_sim/forces to rise above 0")

    def test_wrist_command_propagates(self) -> None:
        self.assertTrue(self._spin_until("/wrist/state", 2.0))
        pub = self._node.create_publisher(
            Float64MultiArray, "/wrist/set_position", 10
        )
        msg = Float64MultiArray()
        msg.data = [90.0, 45.0]
        for _ in range(20):
            pub.publish(msg)
            self.executor.spin_once(timeout_sec=0.05)
        # Spin for up to 2s and collect wrist state.
        deadline = time.monotonic() + 2.0
        wrist_positions = []
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            with self._locks["/wrist/state"]:
                for m in self._received["/wrist/state"]:
                    if len(m.data) >= 1:
                        wrist_positions.append(m.data[0])
        self.assertGreater(len(wrist_positions), 0)
        # The wrist should have moved from 180 (open) toward 90 (vertical).
        # After ~2s, the position should be less than 180.
        self.assertLess(wrist_positions[-1], 180.0)


if __name__ == "__main__":
    unittest.main()
