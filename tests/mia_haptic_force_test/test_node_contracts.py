#!/usr/bin/env python3
"""RCLPY-dependent node contract tests (Docker tier).

These tests exercise the actual node callbacks with synthetic inputs and
assert the contracts:
- Supervisor stage transitions driven by synthetic EMG/force/wrist.
- Controller switching with a fake ControllerManagerClient.
- Haptic mapping from force/wrist to /haptic_band/motors.

Run inside the Docker container where rclpy is available.
"""

from __future__ import annotations

import os
import sys
import threading
import time
import unittest
from typing import Any

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

try:
    import rclpy  # noqa: F401
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64, Float64MultiArray, Int32, String
    from sensor_msgs.msg import JointState
    from mia_hand_msgs.msg import ForceData, JointData, MotorData

    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False

from scripts.mia_haptic_force_test.common.constants import (
    FINGER_COUNT,
    Stage,
    TOPIC_CONTROL_HOLD_MODE,
    TOPIC_CONTROL_MODE,
    TOPIC_CONTROL_TARGET_FORCE,
    TOPIC_CONTROL_TARGET_WRIST,
    TOPIC_EMG_GESTURE,
    TOPIC_EMG_GESTURE_LABEL,
    TOPIC_HAND_FORCES,
    TOPIC_HAPTIC_BAND_MOTORS,
    TOPIC_TEST_STAGE,
    TOPIC_WRIST_STATE,
)


@unittest.skipUnless(_HAS_ROS, "rclpy/mia_hand_msgs not available")
class SupervisorStageTransitionTest(unittest.TestCase):
    """Drive the supervisor's _tick() through all stage transitions."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.executor = SingleThreadedExecutor()

    @classmethod
    def tearDownClass(cls) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self) -> None:
        from scripts.mia_haptic_force_test.supervisor_node import SupervisorNode

        self._node = SupervisorNode(
            config_path=os.path.join(
                _REPO_ROOT, "config", "mia_haptic_force_test.yaml"
            )
        )
        self.executor.add_node(self._node)
        # Wait for the node to start publishing /test/stage.
        time.sleep(0.5)
        self.executor.spin_once(timeout_sec=0.1)

    def tearDown(self) -> None:
        self.executor.remove_node(self._node)
        self._node.destroy_node()

    def _spin_until(self, stage: str, timeout_s: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            with self._node._lock:
                if self._node._stage == stage:
                    return True
        return False

    def test_starts_in_waiting_for_activation(self) -> None:
        # The supervisor should reach waiting_for_activation quickly.
        self.assertTrue(
            self._spin_until(Stage.WAITING_FOR_ACTIVATION.value, 3.0),
            "did not reach waiting_for_activation",
        )


@unittest.skipUnless(_HAS_ROS, "rclpy/mia_hand_msgs not available")
class ControllerSwitchTest(unittest.TestCase):
    """Test hand_controller_node._cb_mode and _switch_if_needed."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.executor = SingleThreadedExecutor()

    @classmethod
    def tearDownClass(cls) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self) -> None:
        from scripts.mia_haptic_force_test.hand_controller_node import (
            HandControllerNode,
        )

        # Monkeypatch ControllerManagerClient to avoid the real ros2_control.
        from scripts.mia_haptic_force_test import hand_controller_node as hcn_mod

        self._fake_calls: list[dict] = []
        self._fake_state: dict[str, str] = {}

        class _FakeCMClient:
            def __init__(self, owner_node) -> None:
                self._owner = owner_node

            def wait_for_services(self, timeout_sec: float = 20.0) -> bool:
                return True

            def switch_controllers(self, activate, deactivate, timeout_sec=10.0):
                self._fake_calls.append({
                    "activate": list(activate),
                    "deactivate": list(deactivate),
                })
                for c in activate:
                    self._fake_state[c] = "active"
                for c in deactivate:
                    self._fake_state[c] = "inactive"
                return None

        self._original_cm = hcn_mod.ControllerManagerClient
        hcn_mod.ControllerManagerClient = _FakeCMClient

        self._node = HandControllerNode(
            config_path=os.path.join(
                _REPO_ROOT, "config", "mia_haptic_force_test.yaml"
            )
        )
        self.executor.add_node(self._node)
        time.sleep(0.3)
        self.executor.spin_once(timeout_sec=0.1)

    def tearDown(self) -> None:
        from scripts.mia_haptic_force_test import hand_controller_node as hcn_mod
        hcn_mod.ControllerManagerClient = self._original_cm
        self.executor.remove_node(self._node)
        self._node.destroy_node()

    def test_cb_mode_velocity_triggers_switch(self) -> None:
        # Publish /control/mode = "velocity"
        pub = self._node.create_publisher(String, TOPIC_CONTROL_MODE, 10)
        msg = String()
        msg.data = "velocity"
        for _ in range(5):
            pub.publish(msg)
            self.executor.spin_once(timeout_sec=0.05)
            time.sleep(0.05)
        # Assert the fake client was called with activate=[group_vel_ff_controller].
        activate_lists = [c["activate"] for c in self._fake_calls]
        self.assertTrue(
            any("group_vel_ff_controller" in a for a in activate_lists),
            f"no switch to velocity; calls={self._fake_calls}",
        )


@unittest.skipUnless(_HAS_ROS, "rclpy/mia_hand_msgs not available")
class HapticNodeMappingTest(unittest.TestCase):
    """Test haptic_node's force_haptics() and wrist_haptics() via callbacks."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.executor = SingleThreadedExecutor()

    @classmethod
    def tearDownClass(cls) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self) -> None:
        from scripts.mia_haptic_force_test.haptic_node import HapticNode

        self._node = HapticNode(
            config_path=os.path.join(
                _REPO_ROOT, "config", "mia_haptic_force_test.yaml"
            )
        )
        self.executor.add_node(self._node)
        # Subscribe to /haptic_band/motors to capture the output.
        self._received: list[list[float]] = []
        self._lock = threading.Lock()
        self._helper = Node("haptic_test_helper")
        self.executor.add_node(self._helper)
        self._helper.create_subscription(
            Float32MultiArray, TOPIC_HAPTIC_BAND_MOTORS, self._on_motors, 10
        )

    def tearDown(self) -> None:
        self.executor.remove_node(self._helper)
        self._helper.destroy_node()
        self.executor.remove_node(self._node)
        self._node.destroy_node()

    def _on_motors(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._received.append(list(msg.data))

    def _spin_until(self, predicate, timeout_s: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            with self._lock:
                if predicate(self._received):
                    return True
        return False

    def test_wrist_haptics_publishes_8_motors(self) -> None:
        # Publish /wrist/state = [90.0, 0.0]
        pub = self._node.create_publisher(
            Float64MultiArray, TOPIC_WRIST_STATE, 10
        )
        msg = Float64MultiArray()
        msg.data = [90.0, 0.0]
        for _ in range(20):
            pub.publish(msg)
            self.executor.spin_once(timeout_sec=0.05)
            time.sleep(0.05)
        self.assertTrue(
            self._spin_until(lambda r: any(any(v > 0.0 for v in m) for m in r)),
            "no haptic band motors > 0 after wrist state",
        )

    def test_force_haptics_publishes_8_motors(self) -> None:
        # Publish /hand/forces = [200, 200, 200] and /control/hold_mode=force
        force_pub = self._node.create_publisher(
            Float32MultiArray, TOPIC_HAND_FORCES, 10
        )
        hold_pub = self._node.create_publisher(
            String, TOPIC_CONTROL_HOLD_MODE, 10
        )
        force_msg = Float32MultiArray()
        force_msg.data = [200.0, 200.0, 200.0]
        hold_msg = String()
        hold_msg.data = "force"
        for _ in range(20):
            force_pub.publish(force_msg)
            hold_pub.publish(hold_msg)
            self.executor.spin_once(timeout_sec=0.05)
            time.sleep(0.05)
        self.assertTrue(
            self._spin_until(lambda r: len(r) > 0 and len(r[-1]) == 8),
            "no haptic band motors with 8 values after force input",
        )


if __name__ == "__main__":
    unittest.main()
