#!/usr/bin/env python3
"""MVP-0VA: force_input_node.

Subscribes to raw force sensor data (``mia_hand_msgs/ForceData``) and/or raw
joint states.  Computes per-finger normal forces and publishes them together
with a provenance string.

Publishes::

    /hand/forces       (std_msgs/Float32MultiArray)  normal forces [thumb, index, mrl]
    /hand/force_source (std_msgs/String)             "force_data" | "effort" | "none"
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String

_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and __package__ is None and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from mia_hand_msgs.msg import ForceData

from scripts.mia_haptic_force_test.common.constants import (
    FINGER_COUNT,
    FINGER_JOINTS,
    TOPIC_HAND_FORCE_SOURCE,
    TOPIC_HAND_FORCES,
    TOPIC_HAND_JOINT_STATES,
    TOPIC_HW_FINGER_FORCES,
    TOPIC_HW_JOINT_STATES,
)


class ForceInputNode(Node):
    """Per-finger normal-force publisher."""

    def __init__(self, config_path: Optional[str] = None) -> None:
        super().__init__("force_input_node")
        self._config_path = config_path or os.path.join(_REPO_ROOT, "config", "mia_haptic_force_test.yaml")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("force_window_size", 5)
        self.declare_parameter("force_stale_timeout_s", 1.0)
        self.declare_parameter("force_data_topic", TOPIC_HW_FINGER_FORCES)
        self.declare_parameter("joint_states_topic", TOPIC_HW_JOINT_STATES)
        self.declare_parameter("use_effort_fallback", True)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)
        self._window = max(int(self.get_parameter("force_window_size").value), 1)
        self._stale_timeout = float(self.get_parameter("force_stale_timeout_s").value)
        self._use_effort_fallback = bool(self.get_parameter("use_effort_fallback").value)

        self._forces_pub = self.create_publisher(Float32MultiArray, TOPIC_HAND_FORCES, 10)
        self._source_pub = self.create_publisher(String, TOPIC_HAND_FORCE_SOURCE, 10)
        self._joint_states_pub = self.create_publisher(JointState, TOPIC_HAND_JOINT_STATES, 10)

        self._lock = threading.Lock()
        self._buffers: list[deque[float]] = [deque(maxlen=self._window) for _ in range(FINGER_COUNT)]
        self._source = "none"
        self._last_force_time = 0.0
        self._forces: list[float] = [0.0] * FINGER_COUNT
        self._joint_positions: list[float] = [0.0] * FINGER_COUNT
        self._joint_efforts: list[float] = [0.0] * FINGER_COUNT

        self.create_subscription(
            ForceData,
            str(self.get_parameter("force_data_topic").value),
            self._on_force_data,
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_states,
            10,
        )

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="force_loop")
        self._thread.start()

    def _on_force_data(self, msg: ForceData) -> None:
        raw = [
            float(msg.thumb_nfor),
            float(msg.index_nfor),
            float(msg.mrl_nfor),
        ]
        with self._lock:
            for i in range(FINGER_COUNT):
                self._buffers[i].append(raw[i])
            self._forces = [sum(b) / len(b) for b in self._buffers]
            self._source = "force_data"
            self._last_force_time = time.monotonic()

    def _on_joint_states(self, msg: JointState) -> None:
        with self._lock:
            for i, name in enumerate(FINGER_JOINTS):
                try:
                    idx = msg.name.index(name)
                    self._joint_positions[i] = float(msg.position[idx])
                    if idx < len(msg.effort):
                        self._joint_efforts[i] = float(msg.effort[idx])
                except (ValueError, IndexError):
                    pass

        # Publish filtered joint state for downstream controllers
        filtered = JointState()
        filtered.header = msg.header
        for i, name in enumerate(FINGER_JOINTS):
            try:
                idx = msg.name.index(name)
                filtered.name.append(name)
                filtered.position.append(msg.position[idx])
                if idx < len(msg.velocity):
                    filtered.velocity.append(msg.velocity[idx])
                if idx < len(msg.effort):
                    filtered.effort.append(msg.effort[idx])
            except (ValueError, IndexError):
                filtered.name.append(name)
                filtered.position.append(0.0)
                filtered.velocity.append(0.0)
                filtered.effort.append(0.0)
        self._joint_states_pub.publish(filtered)

    def _maybe_fallback(self) -> None:
        """If ForceData is stale, fall back to joint effort estimate."""
        if self._source != "force_data":
            return
        if time.monotonic() - self._last_force_time > self._stale_timeout:
            with self._lock:
                if self._use_effort_fallback:
                    self._forces = list(self._joint_efforts)
                    self._source = "effort"
                else:
                    self._forces = [0.0] * FINGER_COUNT
                    self._source = "none"

    def _loop(self) -> None:
        while rclpy.ok() and self._running:
            t0 = time.monotonic()
            self._maybe_fallback()

            with self._lock:
                forces = list(self._forces)
                source = self._source

            self._forces_pub.publish(Float32MultiArray(data=forces))
            self._source_pub.publish(String(data=source))

            elapsed = time.monotonic() - t0
            remaining = self._dt - elapsed
            if remaining > 0.0:
                time.sleep(remaining)

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def destroy_node(self) -> None:
        self.stop()
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config-path", default=None)
    known, remaining = parser.parse_known_args(args or [])
    rclpy.init(args=remaining)
    node = ForceInputNode(config_path=known.config_path)

    def _signal_handler(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _signal_handler)

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
