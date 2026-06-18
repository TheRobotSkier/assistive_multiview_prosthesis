#!/usr/bin/env python3
"""MVP-CJ2: hand_controller_node — final 100 Hz Python control loop.

Python 3.12 / ROS 2 Jazzy.
Targets ≥ 100 Hz mean loop rate with ≤ 2 ms p99 period jitter.
No C++ fallback needed (mvp-3MS spike validated Python performance).
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64MultiArray, Int32, String

# ── sys.path bootstrap for direct execution ──────────────────────────────
_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and __package__ is None and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)


from force_controller.controller_manager_client import ControllerManagerClient
from scripts.mia_haptic_force_test.common.constants import (
    FINGER_COUNT,
    FINGER_JOINTS,
    POSITION_CONTROLLERS,
    VELOCITY_CONTROLLERS,
)
from scripts.mia_haptic_force_test.common.conversions import dict_get, hold_velocity

# ── Timing window (not in config; used for statistics) ─────────────────────
TIMING_WINDOW: int = 1000           # ~10 s at 100 Hz
TIMING_PUB_INTERVAL: int = 100      # publish every 100 loops (~1 Hz)


class HandControllerNode(Node):
    """Hand controller node with a dedicated 100 Hz control thread.

    Subscribes to force, joint-state, EMG, and control topics.  All incoming
    messages are cached under a lock; the control loop never blocks on DDS.
    """


    def __init__(self) -> None:
        super().__init__("hand_controller_node")
        self._load_config()

        # ── Cached message state (written by DDS callbacks, read by control loop) ──
        self._lock = threading.Lock()

        self._forces: list[float] = [0.0] * FINGER_COUNT
        self._joint_positions: list[float] = [0.0] * FINGER_COUNT
        self._gesture: str = ""
        self._gesture_label: int = 0
        self._confidence: float = 0.0
        self._proportional: float = 0.0
        self._target_force: list[float] = [0.0] * FINGER_COUNT
        self._target_wrist: float = 90.0
        self._mode: str = "position"
        self._enable: bool = False
        self._hold_mode: str = "force"

        # ── Publishers ──────────────────────────────────────────────────────
        self._vel_pub: Publisher = self.create_publisher(
            Float64MultiArray, "/group_vel_ff_controller/commands", 10
        )
        self._pos_pub: Publisher = self.create_publisher(
            Float64MultiArray, "/group_pos_ff_controller/commands", 10
        )
        self._wrist_pub: Publisher = self.create_publisher(
            Float64MultiArray, "/wrist/set_position", 10
        )
        self._force_err_pub: Publisher = self.create_publisher(
            Float64MultiArray, "/controller/force_error", 10
        )
        self._active_pub: Publisher = self.create_publisher(
            Bool, "/controller/active", 10
        )
        self._timing_pub: Publisher = self.create_publisher(
            String, "/controller/loop_timing", 10
        )

        # ── Subscriptions ───────────────────────────────────────────────────
        self.create_subscription(Float32MultiArray, "/hand/forces", self._cb_forces, 10)
        self.create_subscription(JointState, "/hand/joint_states", self._cb_joint_states, 10)
        self.create_subscription(String, "/emg/gesture", self._cb_gesture, 10)
        self.create_subscription(Int32, "/emg/gesture_label", self._cb_gesture_label, 10)
        self.create_subscription(Float32, "/emg/confidence", self._cb_confidence, 10)
        self.create_subscription(Float32, "/emg/proportional", self._cb_proportional, 10)
        self.create_subscription(Float64MultiArray, "/control/target_force", self._cb_target_force, 10)
        self.create_subscription(Float64MultiArray, "/control/target_wrist", self._cb_target_wrist, 10)
        self.create_subscription(String, "/control/mode", self._cb_mode, 10)
        self.create_subscription(Bool, "/control/enable", self._cb_enable, 10)
        self.create_subscription(String, "/control/hold_mode", self._cb_hold_mode, 10)

        # ── Controller manager client ───────────────────────────────────────
        self._cm = ControllerManagerClient(self)

        # ── Control loop state ──────────────────────────────────────────────
        self._last_switch_mode: Optional[str] = None
        self._running: bool = False
        self._control_thread: Optional[threading.Thread] = None

        # ── Timing statistics ───────────────────────────────────────────────
        self._loop_times: deque[float] = deque(maxlen=TIMING_WINDOW)
        self._timing_ticks: int = 0

        self.get_logger().info("HandControllerNode initialized")

    # ────────────────────────────────────────────────────────────────────────

    # ────────────────────────────────────────────────────────────────────────
    # Config loading
    # ────────────────────────────────────────────────────────────────────────

    def _load_config(self) -> None:
        """Load configuration from config/mia_haptic_force_test.yaml.

        Declares ROS parameters for introspection and sets instance
        attributes used by the control-loop hot path.
        """
        import yaml

        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            "config",
            "mia_haptic_force_test.yaml",
        )

        try:
            with open(config_path) as f:
                raw = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(f"Config load failed ({config_path}): {exc}")
            raw = {}

        def _walk(path: str, default):
            cur = raw
            parts = path.split(".")
            for i, part in enumerate(parts):
                if isinstance(cur, dict):
                    fallback = default if i == len(parts) - 1 else {}
                    cur = cur.get(part, fallback)
                else:
                    return default
            return cur

        # Hand open positions (dict -> list in FINGER_JOINTS order)
        open_raw = _walk("hand.open_positions", {})
        if isinstance(open_raw, dict):
            self._open_positions = [float(open_raw.get(j, 0.0)) for j in FINGER_JOINTS]
        else:
            self._open_positions = [0.0] * FINGER_COUNT
        self.declare_parameter("hand.open_positions", self._open_positions)

        # Hold-velocity parameters
        self._hold_deadzone = float(_walk("hand.hold_deadzone", 0.0))
        self.declare_parameter("hand.hold_deadzone", self._hold_deadzone)

        self._hold_max_velocity = float(_walk("hand.hold_max_velocity_rad_s", 0.08))
        self.declare_parameter("hand.hold_max_velocity_rad_s", self._hold_max_velocity)

        self._hold_min_overshoot = float(_walk("hand.hold_min_overshoot", 0.0))
        self.declare_parameter("hand.hold_min_overshoot", self._hold_min_overshoot)

        self._hold_max_overshoot = float(_walk("hand.hold_max_overshoot", 0.0))
        self.declare_parameter("hand.hold_max_overshoot", self._hold_max_overshoot)

        # Emergency backoff
        self._emergency_backoff_velocity = float(
            _walk("force.emergency_backoff_velocity_rad_s", -0.05)
        )
        self.declare_parameter(
            "force.emergency_backoff_velocity_rad_s", self._emergency_backoff_velocity
        )
        self._emergency_threshold = float(_walk("force.emergency_threshold", float("inf")))
        self.declare_parameter("force.emergency_threshold", self._emergency_threshold)

        # Control rate
        self._control_rate_hz = float(_walk("runtime.control_rate_hz", 100.0))
        self.declare_parameter("runtime.control_rate_hz", self._control_rate_hz)
        self._dt = 1.0 / self._control_rate_hz if self._control_rate_hz > 0.0 else 0.01

        # Wrist acceleration
        self._acceleration_deg_s2 = float(_walk("wrist.acceleration_deg_s2", 180.0))
        self.declare_parameter("wrist.acceleration_deg_s2", self._acceleration_deg_s2)
    # DDS callbacks (run on the executor thread)
    # ────────────────────────────────────────────────────────────────────────

    def _cb_forces(self, msg: Float32MultiArray) -> None:
        with self._lock:
            self._forces = (
                list(msg.data) if len(msg.data) >= FINGER_COUNT else [0.0] * FINGER_COUNT
            )

    def _cb_joint_states(self, msg: JointState) -> None:
        with self._lock:
            self._joint_positions = (
                list(msg.position)
                if len(msg.position) >= FINGER_COUNT
                else [0.0] * FINGER_COUNT
            )

    def _cb_gesture(self, msg: String) -> None:
        with self._lock:
            self._gesture = msg.data

    def _cb_gesture_label(self, msg: Int32) -> None:
        with self._lock:
            self._gesture_label = msg.data

    def _cb_confidence(self, msg: Float32) -> None:
        with self._lock:
            self._confidence = msg.data

    def _cb_proportional(self, msg: Float32) -> None:
        with self._lock:
            self._proportional = msg.data

    def _cb_target_force(self, msg: Float64MultiArray) -> None:
        with self._lock:
            self._target_force = (
                list(msg.data) if len(msg.data) >= FINGER_COUNT else [0.0] * FINGER_COUNT
            )

    def _cb_target_wrist(self, msg: Float64MultiArray) -> None:
        with self._lock:
            self._target_wrist = float(msg.data[0]) if msg.data else 90.0

    def _cb_mode(self, msg: String) -> None:
        with self._lock:
            self._mode = msg.data.lower()

    def _cb_enable(self, msg: Bool) -> None:
        with self._lock:
            self._enable = msg.data

    def _cb_hold_mode(self, msg: String) -> None:
        with self._lock:
            self._hold_mode = msg.data.lower()

    # ────────────────────────────────────────────────────────────────────────
    # Snapshot (atomic copy for the control thread)
    # ────────────────────────────────────────────────────────────────────────

    def _snapshot(self) -> dict:
        """Atomically copy all cached state for the control thread."""
        with self._lock:
            return {
                "forces": list(self._forces),
                "joint_positions": list(self._joint_positions),
                "gesture": self._gesture,
                "gesture_label": self._gesture_label,
                "confidence": self._confidence,
                "proportional": self._proportional,
                "target_force": list(self._target_force),
                "target_wrist": self._target_wrist,
                "mode": self._mode,
                "enable": self._enable,
                "hold_mode": self._hold_mode,
            }

    # ────────────────────────────────────────────────────────────────────────
    # Controller switching
    # ────────────────────────────────────────────────────────────────────────

    def _switch_if_needed(self, new_mode: str) -> None:
        """Switch controller groups when the control mode changes."""
        if new_mode == self._last_switch_mode:
            return
        self.get_logger().info(
            f"Switching mode: {self._last_switch_mode} -> {new_mode}"
        )

        if new_mode == "position":
            self._cm.switch_controllers(
                activate=["group_pos_ff_controller"],
                deactivate=VELOCITY_CONTROLLERS,
            )
        elif new_mode in ("velocity", "emergency_backoff"):
            self._cm.switch_controllers(
                activate=["group_vel_ff_controller"],
                deactivate=POSITION_CONTROLLERS,
            )
        self._last_switch_mode = new_mode

    # ────────────────────────────────────────────────────────────────────────
    # Control loop (runs in a dedicated thread)
    # ────────────────────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        """Dedicated 100 Hz control loop — never blocks on DDS."""
        self._running = True
        self.get_logger().info("Control loop started")
        next_time: float = time.monotonic()

        while rclpy.ok() and self._running:
            loop_start = time.monotonic()
            period: float = loop_start - next_time + self._dt
            next_time += self._dt

            # Snapshot latest DDS state without blocking
            state = self._snapshot()
            mode: str = state["mode"]

            # Autonomous emergency backoff (override mode locally)
            if mode != "position" and max(state["forces"]) > self._emergency_threshold:
                self.get_logger().warn(
                    f"Emergency backoff triggered: force {max(state['forces']):.1f} "
                    f"> {self._emergency_threshold}"
                )
                mode = "emergency_backoff"

            # Switch controllers when mode changes
            try:
                self._switch_if_needed(mode)
            except Exception as exc:
                self.get_logger().warn(f"Controller switch failed: {exc}")

            # ── Control output ──────────────────────────────────────────────
            enabled: bool = state["enable"] and mode != "position"

            if mode == "position":
                pos_msg = Float64MultiArray()
                pos_msg.data = self._open_positions
                self._pos_pub.publish(pos_msg)

            elif mode == "velocity":
                velocities: list[float] = []
                for i in range(FINGER_COUNT):
                    v = hold_velocity(
                        target=state["target_force"][i],
                        force=state["forces"][i],
                        deadzone=self._hold_deadzone,
                        max_velocity=self._hold_max_velocity,
                        min_overshoot=self._hold_min_overshoot,
                        max_overshoot=self._hold_max_overshoot,
                    )
                    velocities.append(v)
                vel_msg = Float64MultiArray()
                vel_msg.data = velocities
                self._vel_pub.publish(vel_msg)

                # Force error
                err_msg = Float64MultiArray()
                err_msg.data = [
                    state["target_force"][i] - state["forces"][i]
                    for i in range(FINGER_COUNT)
                ]
                self._force_err_pub.publish(err_msg)

            elif mode == "emergency_backoff":
                vel_msg = Float64MultiArray()
                vel_msg.data = [self._emergency_backoff_velocity] * FINGER_COUNT
                self._vel_pub.publish(vel_msg)

            # Active flag
            active_msg = Bool()
            active_msg.data = enabled
            self._active_pub.publish(active_msg)

            # Wrist position (target + acceleration)
            wrist_msg = Float64MultiArray()
            wrist_msg.data = [state["target_wrist"], self._acceleration_deg_s2]
            self._wrist_pub.publish(wrist_msg)

            # —— Timing bookkeeping ——————————————————————————————————
            self._loop_times.append(period)
            self._timing_ticks += 1

            if self._timing_ticks % TIMING_PUB_INTERVAL == 0 and self._loop_times:
                self._publish_timing()

            # Sleep to maintain target rate; busy-wait the last millisecond for accuracy
            BUSY_WAIT_THRESHOLD: float = 0.001
            while True:
                now: float = time.monotonic()
                remaining: float = next_time - now
                if remaining <= 0.0:
                    break
                if remaining > BUSY_WAIT_THRESHOLD:
                    time.sleep(remaining - BUSY_WAIT_THRESHOLD)
                # else: spin until next_time

        self.get_logger().info("Control loop exited")

    # ────────────────────────────────────────────────────────────────────────
    # Timing statistics
    # ────────────────────────────────────────────────────────────────────────

    def _publish_timing(self) -> None:
        """Compute period statistics and publish on /controller/loop_timing."""
        if not self._loop_times:
            return

        sorted_times: list[float] = sorted(self._loop_times)
        n: int = len(sorted_times)
        mean_s: float = sum(sorted_times) / n
        mean_hz: float = 1.0 / mean_s if mean_s > 0.0 else 0.0

        # Jitter = absolute deviation from target period (ms)
        target_period: float = self._dt
        jitters_ms: list[float] = [abs(t - target_period) * 1000.0 for t in sorted_times]
        jitters_sorted: list[float] = sorted(jitters_ms)

        p50_ms: float = jitters_sorted[int(n * 0.50)]
        p99_ms: float = jitters_sorted[min(int(n * 0.99), n - 1)]
        max_period_ms: float = sorted_times[-1] * 1000.0

        data: dict = {
            "mean_hz": round(mean_hz, 1),
            "p99_jitter_ms": round(p99_ms, 3),
            "p50_jitter_ms": round(p50_ms, 3),
            "max_period_ms": round(max_period_ms, 3),
            "samples": n,
        }
        msg = String()
        msg.data = json.dumps(data, separators=(",", ":"))
        self._timing_pub.publish(msg)

    def get_timing_summary(self) -> dict:
        """Return the latest timing snapshot (used by the test script)."""
        if not self._loop_times:
            return {}

        sorted_times: list[float] = sorted(self._loop_times)
        n: int = len(sorted_times)
        mean_s: float = sum(sorted_times) / n
        mean_hz: float = 1.0 / mean_s if mean_s > 0.0 else 0.0

        target_period: float = self._dt
        jitters_ms: list[float] = [abs(t - target_period) * 1000.0 for t in sorted_times]
        jitters_sorted: list[float] = sorted(jitters_ms)

        return {
            "mean_hz": round(mean_hz, 1),
            "p99_jitter_ms": round(jitters_sorted[min(int(n * 0.99), n - 1)], 3),
            "p50_jitter_ms": round(jitters_sorted[int(n * 0.50)], 3),
            "max_period_ms": round(sorted_times[-1] * 1000.0, 3),
            "samples": n,
        }

    # ────────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ────────────────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the control loop background thread."""
        if self._control_thread is not None:
            return
        self._control_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="control_loop",
        )
        self._control_thread.start()

    def stop(self) -> None:
        """Signal the control loop to stop and join the thread."""
        self._running = False
        if self._control_thread is not None:
            self._control_thread.join(timeout=2.0)
            self._control_thread = None


def main(args: Optional[list[str]] = None) -> None:
    """Entry point: initialise ROS 2, run the node, spin DDS executor."""
    rclpy.init(args=args)
    node = HandControllerNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    node.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
