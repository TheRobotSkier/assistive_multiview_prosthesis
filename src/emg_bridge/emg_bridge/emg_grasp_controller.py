#!/usr/bin/env python3
"""ROS 2 node wrapper for the three-mode EMG grasp state machine.

Subscribes to EMG classifier topics and runs the state machine at a fixed
rate, publishing intents as std_msgs/String on /emg_grasp/intent.

Subscriptions:
    /emg/gesture_label    std_msgs/Int32
    /emg/confidence       std_msgs/Float32
    /emg/proportional     std_msgs/Float32

Publishers:
    /emg_grasp/intent      std_msgs/String  (JSON-encoded intent)
    /emg_grasp/mode        std_msgs/String  (current mode name)

Parameters:
    gesture_rest          (int)   – REST label     (default 0)
    gesture_power         (int)   – POWER label    (default 1)
    gesture_flexion       (int)   – FLEXION label  (default 2)
    gesture_extension     (int)   – EXTENSION label(default 4)
    gesture_open          (int)   – OPEN label     (default 3)
    open_hold_duration_s  (double)– seconds OPEN must be held (default 1.0)
    open_prop_threshold   (double)– proportional threshold for OPEN safety (default 0.7)
    power_hold_duration_s (double)– seconds POWER must be held (default 1.0)
    power_rearm_duration_s(double)– seconds before POWER can re-trigger (default 0.3)
    stale_timeout_s       (double)– EMG data timeout -> neutral (default 1.0)
    force_adjust_step     (double)– force target delta per tick (default 1.0)
    wrist_velocity_scale  (double)– scale proportional to wrist velocity (default 1.0)
    update_rate_hz        (double)– state machine tick rate (default 20.0)
    emg_gesture_topic     (str)   – gesture label topic (default /emg/gesture_label)
    emg_confidence_topic  (str)   – confidence topic    (default /emg/confidence)
    emg_proportional_topic(str)   – proportional topic  (default /emg/proportional)
    intent_topic          (str)   – intent output topic (default /emg_grasp/intent)
    mode_topic            (str)   – mode output topic   (default /emg_grasp/mode)
"""

from __future__ import annotations

import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float32, Int32, String

from emg_bridge.emg_grasp_state_machine import (
    EmgGraspStateMachine,
    EmgInput,
    StateMachineConfig,
)


_LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class EmgGraspControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("emg_grasp_controller")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("gesture_rest", 0)
        self.declare_parameter("gesture_power", 1)
        self.declare_parameter("gesture_flexion", 2)
        self.declare_parameter("gesture_extension", 4)
        self.declare_parameter("gesture_open", 3)

        self.declare_parameter("open_hold_duration_s", 1.0)
        self.declare_parameter("open_prop_threshold", 0.7)
        self.declare_parameter("power_hold_duration_s", 1.0)
        self.declare_parameter("power_rearm_duration_s", 0.3)
        self.declare_parameter("stale_timeout_s", 1.0)
        self.declare_parameter("force_adjust_step", 1.0)
        self.declare_parameter("wrist_velocity_scale", 1.0)
        self.declare_parameter("update_rate_hz", 20.0)

        self.declare_parameter("emg_gesture_topic", "/emg/gesture_label")
        self.declare_parameter("emg_confidence_topic", "/emg/confidence")
        self.declare_parameter("emg_proportional_topic", "/emg/proportional")
        self.declare_parameter("intent_topic", "/emg_grasp/intent")
        self.declare_parameter("mode_topic", "/emg_grasp/mode")

        # ── Build config ────────────────────────────────────────────────────
        cfg = StateMachineConfig(
            gesture_rest=self.get_parameter("gesture_rest").value,
            gesture_power=self.get_parameter("gesture_power").value,
            gesture_flexion=self.get_parameter("gesture_flexion").value,
            gesture_extension=self.get_parameter("gesture_extension").value,
            gesture_open=self.get_parameter("gesture_open").value,
            open_hold_duration_s=self.get_parameter("open_hold_duration_s").value,
            open_proportional_threshold=self.get_parameter("open_prop_threshold").value,
            power_hold_duration_s=self.get_parameter("power_hold_duration_s").value,
            power_rearm_duration_s=self.get_parameter("power_rearm_duration_s").value,
            stale_timeout_s=self.get_parameter("stale_timeout_s").value,
            force_adjust_step=self.get_parameter("force_adjust_step").value,
            wrist_velocity_scale=self.get_parameter("wrist_velocity_scale").value,
        )
        self._sm = EmgGraspStateMachine(cfg)
        self._stale_timeout = cfg.stale_timeout_s

        # ── Topic names ─────────────────────────────────────────────────────
        gesture_topic = self.get_parameter("emg_gesture_topic").value
        confidence_topic = self.get_parameter("emg_confidence_topic").value
        proportional_topic = self.get_parameter("emg_proportional_topic").value
        intent_topic = self.get_parameter("intent_topic").value
        mode_topic = self.get_parameter("mode_topic").value

        # ── Publishers ────────────────────────────────────────────────────
        self._intent_pub = self.create_publisher(String, intent_topic, 10)
        self._mode_pub = self.create_publisher(String, mode_topic, _LATCHED_QOS)

        # ── Subscriptions ───────────────────────────────────────────────────
        self._latest_input = EmgInput()
        self._last_msg_time = time.monotonic()
        self._data_received = False

        self.create_subscription(Int32, gesture_topic, self._on_gesture, 10)
        self.create_subscription(Float32, confidence_topic, self._on_confidence, 10)
        self.create_subscription(Float32, proportional_topic, self._on_proportional, 10)

        # ── Timer ─────────────────────────────────────────────────────────
        rate_hz = self.get_parameter("update_rate_hz").value
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"EMG grasp controller started — rate={rate_hz} Hz, "
            f"stale_timeout={self._stale_timeout}s"
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _on_gesture(self, msg: Int32) -> None:
        self._latest_input.gesture = msg.data
        self._last_msg_time = time.monotonic()
        self._data_received = True

    def _on_confidence(self, msg: Float32) -> None:
        self._latest_input.confidence = msg.data
        self._last_msg_time = time.monotonic()
        self._data_received = True

    def _on_proportional(self, msg: Float32) -> None:
        self._latest_input.proportional = msg.data
        self._last_msg_time = time.monotonic()
        self._data_received = True

    def _tick(self) -> None:
        now = time.monotonic()

        # Stale data -> feed neutral REST input
        if self._data_received and (now - self._last_msg_time) > self._stale_timeout:
            inp = EmgInput(
                gesture=self._sm.config.gesture_rest,
                confidence=0.0,
                proportional=0.0,
                timestamp=now,
            )
            self.get_logger().debug("EMG data stale — injecting REST")
        else:
            inp = EmgInput(
                gesture=self._latest_input.gesture,
                confidence=self._latest_input.confidence,
                proportional=self._latest_input.proportional,
                timestamp=now,
            )

        intents = self._sm.update(inp)

        for intent in intents:
            payload = {
                "intent_type": intent.intent_type.value,
                "value": intent.value,
                "reason": intent.reason,
            }
            self._intent_pub.publish(String(data=json.dumps(payload)))
            self.get_logger().debug(
                f"Intent: {intent.intent_type.value} value={intent.value:.3f} "
                f"reason='{intent.reason}'"
            )

        # Publish current mode (latched, but we republish for monitoring)
        self._mode_pub.publish(String(data=self._sm.mode_name))


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = EmgGraspControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
