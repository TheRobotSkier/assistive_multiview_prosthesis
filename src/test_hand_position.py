#!/usr/bin/env python3
"""Minimal hand position test: open fingers slightly, wait, move back.

Usage (inside container, after source install/setup.bash):
    python3 /prosthesis_ws/src/test_hand_position.py

Requires ros2_control to be running with the group_pos_ff_controller loaded.
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Insert force_controller path so we can import ControllerManagerClient
sys.path.insert(0, "/prosthesis_ws/src/force_controller")

FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_COUNT = len(FINGER_JOINTS)


class HandPositionTest(Node):
    def __init__(self):
        super().__init__("hand_position_test")
        self._pub = self.create_publisher(
            Float64MultiArray, "/group_pos_ff_controller/commands", 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )
        self._positions: list[float] = [0.0] * FINGER_COUNT
        self._got_joints = False

    def _on_joint_states(self, msg: JointState):
        for i, name in enumerate(FINGER_JOINTS):
            try:
                idx = msg.name.index(name)
                self._positions[i] = float(msg.position[idx])
                self._got_joints = True
            except ValueError:
                pass

    def read_positions(self, timeout: float = 2.0) -> list[float] | None:
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._got_joints:
                return list(self._positions)
        return None

    def publish_and_wait(self, positions: list[float], duration: float = 3.0):
        msg = Float64MultiArray()
        msg.data = [float(p) for p in positions]
        self._pub.publish(msg)
        start = time.monotonic()
        while time.monotonic() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._got_joints:
                errs = [
                    abs(self._positions[i] - positions[i])
                    for i in range(FINGER_COUNT)
                ]
                print(
                    f"\r  cmd=[{positions[0]:.3f} {positions[1]:.3f} {positions[2]:.3f}]  "
                    f"pos=[{self._positions[0]:.3f} {self._positions[1]:.3f} "
                    f"{self._positions[2]:.3f}]  err=[{errs[0]:.3f} {errs[1]:.3f} "
                    f"{errs[2]:.3f}]",
                    end="",
                    flush=True,
                )
        print()

    def run(self):
        from force_controller.controller_manager_client import ControllerManagerClient

        print("Waiting for controller_manager...")
        cm = ControllerManagerClient(self, use_private_executor=True)
        if not cm.wait_for_services(timeout_sec=30.0):
            print("FAIL: controller_manager services not available")
            return 1

        print("Switching to group_pos_ff_controller...")
        ok = cm.switch_controllers(
            activate=["group_pos_ff_controller"],
            deactivate=["group_pos_vel_controller", "group_vel_ff_controller"],
        )

        print("Waiting for joint states...")
        positions = None
        for _ in range(50):
            positions = self.read_positions(timeout=1.0)
            if positions is not None:
                break
            print("  no joint states yet, retrying...")
        if positions is None:
            print("FAIL: no joint states received")
            cm.shutdown()
            return 1

        print(
            f"Initial positions: "
            f"[{positions[0]:.3f} {positions[1]:.3f} {positions[2]:.3f}]"
        )

        # Move to slightly more open (+0.05 rad each)
        target = [p + 0.05 for p in positions]
        print(f"Moving to more open position (delta=+0.05 rad)...")
        self.publish_and_wait(target, duration=3.0)

        # Move back
        print("Moving back to original position...")
        self.publish_and_wait(positions, duration=3.0)

        final = self.read_positions(timeout=2.0)
        if final:
            drift = [abs(final[i] - positions[i]) for i in range(FINGER_COUNT)]
            max_drift = max(drift)
            result = "OK" if max_drift < 0.03 else f"WARN (max drift={max_drift:.3f})"
            print(
                f"Final positions: "
                f"[{final[0]:.3f} {final[1]:.3f} {final[2]:.3f}]  => {result}"
            )

        cm.shutdown()
        return 0


def main():
    rclpy.init()
    node = HandPositionTest()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
