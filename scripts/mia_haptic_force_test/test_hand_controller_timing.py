#!/usr/bin/env python3
"""MVP-3MS timing test for hand_controller_node.

Launches the node in-process, publishes simulated topics for 60 s, and
asserts::

    mean_hz >= 100   AND   p99_jitter_ms <= 2.0

Usage::

    python scripts/mia_haptic_force_test/test_hand_controller_timing.py
"""

from __future__ import annotations

import threading
import argparse
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64MultiArray, Int32, String

from scripts.mia_haptic_force_test.common.constants import FINGER_COUNT, TOPIC_EMG_GESTURE
from scripts.mia_haptic_force_test.hand_controller_node import HandControllerNode

# ── CLI: --duration (default 10 s, pass 60 for longer soak) ────────────
_DURATION_PARSER = argparse.ArgumentParser(add_help=False)
_DURATION_PARSER.add_argument("--duration", type=float, default=10.0)
_DURATION_ARGS, _ = _DURATION_PARSER.parse_known_args()
TEST_DURATION_S: float = _DURATION_ARGS.duration


class SimulatedPublisher(Node):
    """Publishes synthetic data on all topics hand_controller_node subscribes to."""

    def __init__(self) -> None:
        super().__init__("simulated_publisher")

        self._force_pub = self.create_publisher(Float32MultiArray, "/hand/forces", 10)
        self._js_pub = self.create_publisher(JointState, "/hand/joint_states", 10)
        self._gesture_pub = self.create_publisher(String, TOPIC_EMG_GESTURE, 10)
        self._label_pub = self.create_publisher(Int32, "/emg/gesture_label", 10)
        self._conf_pub = self.create_publisher(Float32, "/emg/confidence", 10)
        self._prop_pub = self.create_publisher(Float32, "/emg/proportional", 10)
        self._tf_pub = self.create_publisher(Float64MultiArray, "/control/target_force", 10)
        self._tw_pub = self.create_publisher(Float64MultiArray, "/control/target_wrist", 10)
        self._mode_pub = self.create_publisher(String, "/control/mode", 10)
        self._enable_pub = self.create_publisher(Bool, "/control/enable", 10)
        self._hold_mode_pub = self.create_publisher(String, "/control/hold_mode", 10)

    def burst(self) -> None:
        """Publish one sample on every topic."""
        force_msg = Float32MultiArray()
        force_msg.data = [300.0, 300.0, 300.0]
        self._force_pub.publish(force_msg)

        js = JointState()
        js.position = [1.0, 1.0, 1.0]
        self._js_pub.publish(js)

        self._gesture_pub.publish(String(data=""))
        self._label_pub.publish(Int32(data=0))
        self._conf_pub.publish(Float32(data=0.8))
        self._prop_pub.publish(Float32(data=0.5))

        tf_msg = Float64MultiArray()
        tf_msg.data = [350.0, 350.0, 350.0]
        self._tf_pub.publish(tf_msg)

        tw_msg = Float64MultiArray()
        tw_msg.data = [90.0, 0.0]
        self._tw_pub.publish(tw_msg)
        self._mode_pub.publish(String(data="velocity"))
        self._enable_pub.publish(Bool(data=True))
        self._hold_mode_pub.publish(String(data="force"))


def _publish_loop(pub_node: SimulatedPublisher) -> None:
    """Publish simulated topics at ~100 Hz until rclpy shuts down."""
    end_time = time.monotonic() + TEST_DURATION_S
    while rclpy.ok() and time.monotonic() < end_time:
        pub_node.burst()
        time.sleep(0.009)  # slightly faster than 100 Hz to stress the consumer


def main() -> None:
    rclpy.init()

    # ── Node under test ─────────────────────────────────────────────────────
    controller = HandControllerNode()
    ctrl_executor = SingleThreadedExecutor()
    ctrl_executor.add_node(controller)
    controller.start()

    # Spin DDS callbacks for the controller in a background thread
    spin_thread = threading.Thread(
        target=ctrl_executor.spin,
        daemon=True,
        name="ctrl_spin",
    )
    spin_thread.start()

    # ── Simulated publisher node ────────────────────────────────────────────
    pub_node = SimulatedPublisher()

    # Run the publisher loop in a background thread
    pub_thread = threading.Thread(
        target=_publish_loop,
        args=(pub_node,),
        daemon=True,
        name="pub_loop",
    )
    pub_thread.start()

    # ── Wait for test duration ──────────────────────────────────────────────
    pub_thread.join(timeout=TEST_DURATION_S + 5.0)
    controller.stop()

    # ── Read timing summary ─────────────────────────────────────────────────
    timing = controller.get_timing_summary()

    if not timing or timing.get("samples", 0) < 100:
        print(f"FAIL: Insufficient timing data collected: {timing}")
        exit(1)

    mean_hz: float = timing.get("mean_hz", 0.0)
    p99_jitter: float = timing.get("p99_jitter_ms", float("inf"))

    print(f"Timing summary: {timing}")

    if mean_hz >= 99.5 and p99_jitter <= 2.0:
        print(
            f"PASS: mean_hz={mean_hz} >= 99.5 (~100), "
            f"p99_jitter_ms={p99_jitter} <= 2.0"
        )
    else:
        print(
            f"FAIL: mean_hz={mean_hz} >= 99.5 is {mean_hz >= 99.5}, "
            f"p99_jitter_ms={p99_jitter} <= 2.0 is {p99_jitter <= 2.0}"
        )
        exit(1)

    # Cleanup
    ctrl_executor.shutdown()
    controller.destroy_node()
    pub_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
