#!/usr/bin/env python3
"""Minimal wrist position test: move ±5°, wait, move back.

Usage (inside container, after source install/setup.bash):
    python3 /prosthesis_ws/src/test_wrist_position.py

Expects the wrist_driver_node to be running and subscribed to /wrist/set_position.
"""
from __future__ import annotations

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WristPositionTest(Node):
    def __init__(self):
        super().__init__("wrist_position_test")
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

    def move_to(self, target_deg: float, duration: float = 3.0):
        msg = Float64MultiArray()
        msg.data = [float(target_deg), 10.0]  # 10 deg/s² accel
        self._pub.publish(msg)
        start = time.monotonic()
        while time.monotonic() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_state:
                pos = self._latest_state[0]
                err = abs(pos - target_deg)
                print(
                    f"\r  target={target_deg:.1f}°  current={pos:.1f}°  "
                    f"error={err:.1f}°",
                    end="",
                    flush=True,
                )
        print()

    def run(self):
        print("Waiting for wrist state topic...")
        start_angle = None
        for _ in range(50):  # ~5 s
            start_angle = self.read_angle(timeout=1.0)
            if start_angle is not None:
                break
            print("  no state yet, retrying...")
        if start_angle is None:
            print("FAIL: no wrist state received after 5 s")
            return 1

        print(f"Initial angle: {start_angle:.1f}°")

        target = start_angle + 5.0
        print(f"Moving to {target:.1f}° ...")
        self.move_to(target, duration=3.0)

        print(f"Returning to {start_angle:.1f}° ...")
        self.move_to(start_angle, duration=3.0)

        final_angle = self.read_angle(timeout=2.0)
        drift = abs((final_angle or start_angle) - start_angle)
        result = "OK" if drift < 2.0 else "WARN"
        print(f"Final angle: {final_angle:.1f}°  drift={drift:.1f}°  => {result}")
        return 0 if drift < 2.0 else 1


def main():
    rclpy.init()
    node = WristPositionTest()
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
