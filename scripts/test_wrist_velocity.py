#!/usr/bin/env python3
"""Minimal wrist velocity-like test: sweep 5° in small increments at 10 Hz.

Usage (inside container, after source install/setup.bash):
    python3 /prosthesis_ws/src/test_wrist_velocity.py

Since the Dynamixel only accepts position goals, this test publishes
incremental position targets to simulate velocity control.
"""
from __future__ import annotations

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WristVelocityTest(Node):
    def __init__(self):
        super().__init__("wrist_velocity_test")
        self._pub = self.create_publisher(Float64MultiArray, "/wrist/set_position", 10)
        self.create_subscription(
            Float64MultiArray, "/wrist/state", self._on_state, 10
        )
        self._latest_state: list[float] | None = None

    def _on_state(self, msg: Float64MultiArray):
        self._latest_state = list(msg.data)

    def read_angle(self, timeout: float = 2.0) -> float | None:
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_state is not None:
                return self._latest_state[0]
        return None

    def sweep(self, start_deg: float, end_deg: float, steps: int = 50):
        step = (end_deg - start_deg) / steps
        for i in range(steps + 1):
            target = start_deg + step * i
            msg = Float64MultiArray()
            msg.data = [float(target), 20.0]
            self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            pos = self._latest_state
            err = abs((pos or [0.0])[0] - target) if pos else 999
            print(
                f"\r  step {i:2d}/{steps}  target={target:.1f}°  "
                f"current={(pos or [0.0])[0]:.1f}°  error={err:.1f}°",
                end="",
                flush=True,
            )
        print()

    def run(self):
        print("Waiting for wrist state topic...")
        start_angle = None
        for _ in range(50):
            start_angle = self.read_angle(timeout=1.0)
            if start_angle is not None:
                break
            print("  no state yet, retrying...")
        if start_angle is None:
            print("FAIL: no wrist state received after 5 s")
            return 1

        print(f"Initial angle: {start_angle:.1f}°")

        target = start_angle + 5.0
        print(f"Sweeping to {target:.1f}° over 50 steps...")
        self.sweep(start_angle, target, steps=50)

        print(f"Sweeping back to {start_angle:.1f}° ...")
        self.sweep(target, start_angle, steps=50)

        final = self.read_angle(timeout=2.0)
        print(f"Final angle: {final:.1f}°" if final else "Final angle: unknown")
        return 0


def main():
    rclpy.init()
    node = WristVelocityTest()
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
