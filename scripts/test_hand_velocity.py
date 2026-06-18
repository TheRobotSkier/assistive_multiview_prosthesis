#!/usr/bin/env python3
"""Minimal hand velocity test: command small opening velocity, wait, stop.

Usage (inside container, after source install/setup.bash):
    python3 /prosthesis_ws/src/test_hand_velocity.py

Requires ros2_control to be running with the group_vel_ff_controller loaded.
"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, "/prosthesis_ws/src/force_controller")

FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_COUNT = len(FINGER_JOINTS)


class HandVelocityTest(Node):
    def __init__(self):
        super().__init__("hand_velocity_test")
        self._pub = self.create_publisher(
            Float64MultiArray, "/group_vel_ff_controller/commands", 10
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

    def command_and_wait(self, velocities: list[float], duration: float = 3.0):
        """Publish velocities and monitor position change."""
        msg = Float64MultiArray()
        msg.data = [float(v) for v in velocities]
        self._pub.publish(msg)
        start = time.monotonic()
        while time.monotonic() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._got_joints:
                print(
                    f"\r  vel=[{velocities[0]:.3f} {velocities[1]:.3f} "
                    f"{velocities[2]:.3f}]  pos=[{self._positions[0]:.3f} "
                    f"{self._positions[1]:.3f} {self._positions[2]:.3f}]",
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

        print("Loading and configuring group_vel_ff_controller...")
        cm.ensure_controller_loaded("group_vel_ff_controller")
        # Jazzy's load_controller does NOT auto-configure — the controller is
        # left in 'unconfigured' state.  switch_controller only activates
        # controllers that are already in 'inactive' state.
        cm.configure_controller("group_vel_ff_controller")
        cm.switch_controllers(
            activate=["group_vel_ff_controller"],
            deactivate=["group_pos_ff_controller", "group_pos_vel_controller"],
        )

        # Poll until active — BEST_EFFORT may return ok=True even when
        # the controller failed to activate.
        start = time.monotonic()
        while time.monotonic() - start < 15.0:
            states = cm.list_controller_states()
            if states.get("group_vel_ff_controller") == "active":
                break
            time.sleep(0.1)
        else:
            # Try one more switch — the first might have only loaded it
            cm.switch_controllers(
                activate=["group_vel_ff_controller"],
                deactivate=["group_pos_ff_controller", "group_pos_vel_controller"],
            )
        print(f"Velocity controller state: {states.get('group_vel_ff_controller', 'not loaded')}")
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

        # Open with small negative velocity (-0.05 rad/s for 3 s = -0.15 rad)
        vel_open = [-0.05, -0.05, -0.05]
        print("Opening with velocity -0.05 rad/s...")
        self.command_and_wait(vel_open, duration=3.0)

        # Stop
        print("Stopping (zero velocity)...")
        self.command_and_wait([0.0, 0.0, 0.0], duration=1.0)

        # Check positions changed
        final = self.read_positions(timeout=2.0)
        if final:
            delta = [final[i] - positions[i] for i in range(FINGER_COUNT)]
            print(
                f"Position deltas: [{delta[0]:+.3f} {delta[1]:+.3f} {delta[2]:+.3f}]"
            )
            if any(abs(d) > 0.01 for d in delta):
                print("OK: positions changed (velocity control works)")
            else:
                print("WARN: no significant position change")

        cm.shutdown()
        return 0


def main():
    rclpy.init()
    node = HandVelocityTest()
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
