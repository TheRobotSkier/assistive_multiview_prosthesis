#!/usr/bin/env python3
"""EMG Grasp Safety Preflight and Cleanup Module.

STARTUP PREFLIGHT:
  1. Restore/disable emergency stop (call /mia_hand/play).
  2. Verify controller_manager and required controllers are alive.
  3. Verify force feedback freshness.
  4. Verify EMG freshness after classifier startup.
  5. Verify wrist availability if wrist is enabled.
  6. Detect likely command-source conflicts (other nodes commanding the hand).

RUNTIME SAFETY (optional heartbeat):
  7. Stale EMG timeout detection.
  8. Stale force timeout detection.
  9. Max force limit monitoring.
  10. Max wrist error monitoring.
  11. Max command rate checking.

CLEANUP (on SIGINT, OPEN safety, exceptions, normal exit):
  12. Zero velocity on all joints.
  13. Stop wrist motion.
  14. Switch hand back to position control.
  15. Open/reset hand to safe joint positions.
  16. Shut down launched processes.

Usage as a module::

    from mia_hand_ros2_control.emg_safety_preflight import EmgSafetyPreflight

    safety = EmgSafetyPreflight(node)         # wraps an existing node
    safety.run_preflight()                     # blocking startup checks
    safety.start_runtime_monitor()             # periodic safety heartbeat
    # ... application logic ...
    safety.cleanup()                           # orderly shutdown

Usage as a standalone node::

    ros2 run mia_hand_ros2_control emg_safety_preflight --ros-args \\
        --params-file config/emg_safety.yaml
"""

from __future__ import annotations

import atexit
import os
import signal
import time
from typing import Optional, Dict, List

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32, Float64MultiArray, Int32
from sensor_msgs.msg import JointState

try:
    from mia_hand_msgs.msg import ForceData, JointData
    _MIA_MSGS_AVAILABLE = True
except ImportError:
    _MIA_MSGS_AVAILABLE = False


# ── Constants ────────────────────────────────────────────────────────────────

_JOINT_NAMES = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
_SAFE_OPEN_POSITIONS = [0.0, 0.0, 0.0]

# Hand command topics we publish to during cleanup
_HAND_CMD_TOPICS = [
    "/thumb_pos_ff_controller/commands",
    "/index_pos_ff_controller/commands",
    "/mrl_pos_ff_controller/commands",
]


class PreflightError(Exception):
    """Raised when a preflight check fails irrecoverably."""


class SafetyViolation(Exception):
    """Raised when a runtime safety check fails."""


# ── EmgSafetyPreflight ──────────────────────────────────────────────────────

class EmgSafetyPreflight:
    """Preflight checks, runtime monitoring, and cleanup for EMG grasp pipeline.

    This can wrap any rclpy Node.  If ``node`` is None, a dedicated internal
    node is created — call ``spin_in_thread()`` or ``run_standalone()``.
    """

    def __init__(self, node: Optional[Node] = None) -> None:
        # ── Node ownership ─────────────────────────────────────────────────
        if node is not None:
            self._own_node = False
            self._node = node
        else:
            self._own_node = True
            self._node = Node("emg_safety_preflight")

        self._logger = self._node.get_logger()

        # ── Parameters ─────────────────────────────────────────────────────
        self._declare_params()

        # ── Permissions ─────────────────────────────────────────────────────
        self._preflight_done = False
        self._cleaned_up = False

        # ── Service clients ─────────────────────────────────────────────────
        self._play_client = self._node.create_client(
            Trigger, self._param("play_service")
        )
        self._estop_client = self._node.create_client(
            Trigger, self._param("emergency_stop_service")
        )

        # List of processes to kill during cleanup (PIDs or process handles)
        self._launched_pids: List[int] = []

        # ── Runtime state ───────────────────────────────────────────────────
        self._last_force_time: float = 0.0
        self._force_data_received: bool = False
        self._max_observed_force: float = 0.0

        self._last_emg_time: float = 0.0
        self._emg_data_received: bool = False

        self._last_cmd_time: float = 0.0
        self._cmd_count: int = 0

        self._wrist_available: bool = False

        # Runtime monitor
        self._runtime_timer = None
        self._runtime_active = False

        # ── Subscriptions (created lazily during preflight) ─────────────────
        self._force_sub = None
        self._emg_sub = None
        self._joint_state_sub = None
        self._wrist_state_sub = None

        # ── Publishers (created lazily) ─────────────────────────────────────
        self._cmd_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self._wrist_pub = None

    # ── Parameter helpers ───────────────────────────────────────────────────

    def _declare_params(self) -> None:
        """Declare all ROS2 parameters with defaults."""
        defaults = _DEFAULT_PARAMS
        for key, value in defaults.items():
            if key not in ("required_controllers", "hand_command_topics",
                           "safe_open_positions", "command_source_patterns"):
                self._node.declare_parameter(key, value)

        # List-type params
        self._node.declare_parameter(
            "required_controllers", defaults.get("required_controllers", []))
        self._node.declare_parameter(
            "hand_command_topics", defaults.get("hand_command_topics", []))
        self._node.declare_parameter(
            "safe_open_positions", defaults.get("safe_open_positions", [0.0, 0.0, 0.0]))
        self._node.declare_parameter(
            "command_source_patterns", defaults.get("command_source_patterns", []))

    def _param(self, key: str):
        """Get a parameter value with fallback to defaults."""
        try:
            return self._node.get_parameter(key).value
        except rclpy.exceptions.ParameterNotDeclaredException:
            return _DEFAULT_PARAMS.get(key)

    # ── Startup Preflight ───────────────────────────────────────────────────

    def run_preflight(
        self,
        *,
        require_force: bool = True,
        require_emg: bool = True,
        require_wrist: Optional[bool] = None,
    ) -> bool:
        """Run all startup preflight checks in order.

        Returns True if all checks pass, otherwise raises PreflightError.

        Args:
            require_force: Whether to verify force data freshness.
            require_emg: Whether to verify EMG data freshness.
            require_wrist: Override wrist_enabled param (None = use param).
        """
        self._logger.info("=" * 60)
        self._logger.info("EMG SAFETY PREFLIGHT STARTING")
        self._logger.info("=" * 60)

        try:
            # 1. Restore/disable emergency stop
            self._step("restore normal operation", self._preflight_play)

            # 2. Verify controller_manager and required controllers
            self._step("verify controller manager", self._preflight_controllers)

            # 3. Verify force feedback freshness
            if require_force:
                self._step("verify force feedback", self._preflight_force)

            # 4. Verify EMG freshness
            if require_emg:
                self._step("verify EMG freshness", self._preflight_emg)

            # 5. Verify wrist availability
            if require_wrist is None:
                require_wrist = self._param("wrist_enabled")
            if require_wrist:
                self._step("verify wrist availability", self._preflight_wrist)

            # 6. Detect command-source conflicts
            self._step("detect command-source conflicts",
                       self._preflight_command_conflicts)

        except PreflightError as exc:
            self._logger.error(f"PREFLIGHT FAILED: {exc}")
            return False

        self._preflight_done = True
        self._logger.info("=" * 60)
        self._logger.info("ALL PREFLIGHT CHECKS PASSED — hand is safe to operate")
        self._logger.info("=" * 60)
        return True

    def _step(self, label: str, fn):
        """Run a preflight step with logging."""
        self._logger.info(f"[PREFLIGHT] {label} ...")
        fn()
        self._logger.info(f"[PREFLIGHT] {label} ✓")

    # ── Preflight: restore normal operation ──────────────────────────────────

    def _preflight_play(self) -> None:
        """Restore normal operation by calling /mia_hand/play.

        This disables any existing emergency-stop state, restoring the hand
        to the normal-operation (play) mode.  The play command is a serial
        command ``@AR.............*\\r`` sent via mia_safety_node.
        """
        service = self._param("play_service")
        timeout = self._param("preflight_service_timeout_s")

        if not self._play_client.wait_for_service(timeout_sec=timeout):
            raise PreflightError(
                f"Play service '{service}' not available after {timeout}s. "
                f"Is mia_safety_node running?"
            )

        req = Trigger.Request()
        future = self._play_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)

        if not future.done():
            raise PreflightError(
                f"Play service '{service}' timed out after {timeout}s."
            )

        result = future.result()
        if not result.success:
            raise PreflightError(
                f"Play service '{service}' failed: {result.message}"
            )

        self._logger.info(f"Normal operation restored: {result.message}")

    # ── Preflight: verify controller manager ─────────────────────────────────

    def _preflight_controllers(self) -> None:
        """Verify controller_manager and required controllers are available.

        Uses the ``/controller_manager/list_controllers`` service to check
        that each required controller is in the ``active`` state.
        """
        from controller_manager_msgs.srv import ListControllers

        ns = self._param("controller_manager_ns") or ""
        topic = f"{ns}/controller_manager/list_controllers"
        required = self._param("required_controllers")

        client = self._node.create_client(ListControllers, topic)
        timeout = self._param("preflight_controller_timeout_s")

        if not client.wait_for_service(timeout_sec=timeout):
            raise PreflightError(
                f"Controller manager service '{topic}' not available "
                f"after {timeout}s. Is ros2_control_node running?"
            )

        req = ListControllers.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)

        if not future.done():
            raise PreflightError(
                f"Controller manager '{topic}' timed out after {timeout}s."
            )

        result = future.result()
        active_names = {
            c.name for c in result.controller if c.state == "active"
        }
        all_names = {c.name: c.state for c in result.controller}

        missing = [name for name in required if name not in active_names]
        if missing:
            details = []
            for name in missing:
                state = all_names.get(name, "not found")
                details.append(f"  {name}: state={state}")
            raise PreflightError(
                f"Required controllers not active:\n" + "\n".join(details)
            )

        self._logger.info(
            f"All {len(required)} required controllers are active: "
            f"{', '.join(sorted(required))}"
        )

    # ── Preflight: verify force feedback ─────────────────────────────────────

    def _preflight_force(self) -> None:
        """Verify force sensor data is arriving on the expected topic.

        Waits for up to ``preflight_force_timeout_s`` seconds for the first
        message on the configured force data topic.
        """
        topic = self._param("force_data_topic")
        timeout = self._param("preflight_force_timeout_s")

        if not _MIA_MSGS_AVAILABLE:
            self._logger.warn(
                "mia_hand_msgs not available — skipping force data check. "
                "Force safety will rely on runtime monitoring."
            )
            # Still create the subscription for runtime monitoring
            self._ensure_force_sub()
            return

        received = {"flag": False}

        def _cb(msg: ForceData):
            received["flag"] = True
            self._last_force_time = time.monotonic()
            self._force_data_received = True

        sub = self._node.create_subscription(
            ForceData, topic, _cb, 10
        )
        self._force_sub = sub

        deadline = time.monotonic() + timeout
        while not received["flag"] and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)

        if not received["flag"]:
            raise PreflightError(
                f"No force data received on '{topic}' within {timeout}s. "
                f"Check that the Mia Hand force stream is active."
            )

        self._logger.info(f"Force data confirmed on '{topic}'")

    def _ensure_force_sub(self) -> None:
        """Ensure force subscription exists (for runtime monitoring)."""
        if self._force_sub is not None:
            return
        if not _MIA_MSGS_AVAILABLE:
            return
        topic = self._param("force_data_topic")

        def _cb(msg: ForceData):
            self._last_force_time = time.monotonic()
            self._force_data_received = True
            nf = max(float(msg.thumb_nfor), float(msg.index_nfor),
                     float(msg.mrl_nfor))
            if nf > self._max_observed_force:
                self._max_observed_force = nf

        self._force_sub = self._node.create_subscription(
            ForceData, topic, _cb, 10
        )

    # ── Preflight: verify EMG freshness ──────────────────────────────────────

    def _preflight_emg(self) -> None:
        """Verify EMG classification data is flowing.

        Waits for the first message on ``/emg/gesture_label`` (TRANSIENT_LOCAL,
        latched topic from emg_ros_bridge).  A message with label != 0 (i.e.
        not REST) confirms the classifier is running and producing output.
        """
        topic = self._param("emg_label_topic")
        timeout = self._param("preflight_emg_timeout_s")

        received = {"flag": False, "label": 0, "non_rest": False}

        def _cb(msg: Int32):
            received["flag"] = True
            received["label"] = msg.data
            if msg.data != 0:
                received["non_rest"] = True
            self._last_emg_time = time.monotonic()
            self._emg_data_received = True

        sub = self._node.create_subscription(
            Int32, topic, _cb, 10
        )
        self._emg_sub = sub

        deadline = time.monotonic() + timeout
        while not received["flag"] and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)

        if not received["flag"]:
            raise PreflightError(
                f"No EMG data received on '{topic}' within {timeout}s. "
                f"Check that emg_ros_bridge is running and classifier is active."
            )

        self._logger.info(
            f"EMG data confirmed on '{topic}' "
            f"(latest label={received['label']})"
        )

    # ── Preflight: verify wrist availability ─────────────────────────────────

    def _preflight_wrist(self) -> None:
        """Verify wrist driver is present and responsive.

        Checks that the wrist state topic is publishing (driver is alive).
        """
        topic = self._param("wrist_state_topic")
        timeout = self._param("preflight_service_timeout_s")

        received = {"flag": False}

        def _cb(msg: Float64MultiArray):
            received["flag"] = True

        sub = self._node.create_subscription(
            Float64MultiArray, topic, _cb, 10
        )
        self._wrist_state_sub = sub

        deadline = time.monotonic() + timeout
        while not received["flag"] and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)

        if not received["flag"]:
            raise PreflightError(
                f"No wrist state data received on '{topic}' within {timeout}s. "
                f"Check that wrist_driver is running."
            )

        self._wrist_available = True
        self._logger.info(f"Wrist driver confirmed on '{topic}'")

    # ── Preflight: command-source conflicts ──────────────────────────────────

    def _preflight_command_conflicts(self) -> None:
        """Detect other nodes that may be commanding the same hand joints.

        Queries the ROS2 graph to find publishers on each hand command topic.
        Warns if the count of publishers exceeds ``max_command_publishers``,
        or if an unexpected node name is seen.
        """
        cmd_topics = self._param("hand_command_topics")
        patterns = self._param("command_source_patterns")
        max_pubs = self._param("max_command_publishers")

        conflicts = []

        for topic in cmd_topics:
            # Get publisher info from the ROS2 graph
            pub_info = self._node.get_publishers_info_by_topic(topic)
            node_names = sorted(set(p.node_name for p in pub_info))

            if len(node_names) > max_pubs:
                conflicts.append(
                    f"  {topic}: {len(node_names)} publishers > {max_pubs} max "
                    f"({', '.join(node_names)})"
                )

            # Check for patterns that don't match expected names
            unexpected = [
                n for n in node_names
                if not any(pat in n for pat in patterns)
            ]
            if unexpected:
                self._logger.warn(
                    f"Unexpected publisher(s) on {topic}: {unexpected}. "
                    f"Expected patterns: {patterns}"
                )

        if conflicts:
            raise PreflightError(
                f"Command-source conflicts detected:\n" + "\n".join(conflicts)
            )

        self._logger.info(
            f"No command-source conflicts detected on {len(cmd_topics)} topics"
        )

    # ── Runtime Safety Monitor ───────────────────────────────────────────────

    def start_runtime_monitor(self) -> None:
        """Start the periodic runtime safety heartbeat.

        This is optional; the preflight checks already verify the system
        at startup.  The runtime monitor provides continuous safety during
        operation.
        """
        if not self._param("enable_runtime_monitor"):
            self._logger.info("Runtime safety monitor disabled by config.")
            return

        rate = self._param("runtime_check_rate_hz")
        self._runtime_timer = self._node.create_timer(
            1.0 / rate, self._runtime_heartbeat
        )
        self._runtime_active = True
        self._logger.info(f"Runtime safety monitor started at {rate} Hz")

    def stop_runtime_monitor(self) -> None:
        """Stop the runtime safety heartbeat."""
        if self._runtime_timer is not None:
            self._runtime_timer.cancel()
            self._runtime_timer = None
        self._runtime_active = False
        self._logger.info("Runtime safety monitor stopped.")

    def _runtime_heartbeat(self) -> None:
        """Periodic safety check called by the runtime timer."""
        try:
            self._check_stale_emg()
            self._check_stale_force()
            self._check_max_force()
            self._check_wrist_error()
            self._check_command_rate()
        except SafetyViolation as exc:
            self._logger.error(f"RUNTIME SAFETY VIOLATION: {exc}")
            if self._param("auto_cleanup_on_violation"):
                self._logger.error("Auto-cleanup triggered due to safety violation.")
                self.cleanup()

    def _check_stale_emg(self) -> None:
        """Check EMG data is not stale."""
        if not self._emg_data_received:
            return  # Preflight hasn't yet confirmed EMG; tolerate
        now = time.monotonic()
        timeout = self._param("stale_emg_timeout_s")
        elapsed = now - self._last_emg_time
        if elapsed > timeout:
            raise SafetyViolation(
                f"EMG data stale: {elapsed:.2f}s since last message "
                f"(threshold={timeout}s)"
            )

    def _check_stale_force(self) -> None:
        """Check force data is not stale."""
        if not self._force_data_received:
            return
        now = time.monotonic()
        timeout = self._param("stale_force_timeout_s")
        emergency = self._param("stale_force_emergency_s")
        elapsed = now - self._last_force_time

        if elapsed > emergency:
            raise SafetyViolation(
                f"Force data STALE EMERGENCY: {elapsed:.2f}s > {emergency}s "
                f"— stopping hand"
            )
        elif elapsed > timeout:
            self._logger.warn(
                f"Force data stale: {elapsed:.2f}s > {timeout}s "
                f"(warning threshold)"
            )

    def _check_max_force(self) -> None:
        """Check that no finger exceeds the maximum force limit."""
        if not self._force_data_received:
            return
        limit = self._param("max_force_limit")
        if self._max_observed_force > limit:
            raise SafetyViolation(
                f"Max force exceeded: {self._max_observed_force:.0f} > "
                f"{limit:.0f} (raw units)"
            )

    def _check_wrist_error(self) -> None:
        """Check wrist position error if wrist is enabled.

        Note: This is a stub — full wrist error tracking requires storing
        the commanded target and comparing with the measured state.
        """
        if not self._param("wrist_enabled"):
            return
        if not self._wrist_available:
            return
        # Full implementation would track target vs. actual wrist position.
        # For now, this is a placeholder that can be extended.

    def _check_command_rate(self) -> None:
        """Check that command rate is within bounds."""
        max_rate = self._param("max_command_rate_hz")
        # This is checked externally via _note_command(); if the caller
        # calls _note_command() too fast, we warn.
        now = time.monotonic()
        elapsed = now - self._last_cmd_time
        if elapsed > 0 and elapsed < (1.0 / max_rate) and self._cmd_count > 1:
            self._logger.warn(
                f"Command rate high: {1.0 / elapsed:.0f} Hz > {max_rate} Hz"
            )
        self._cmd_count = 0

    def note_command(self) -> None:
        """Caller should invoke this each time a hand command is sent.

        Used by the command rate check.
        """
        now = time.monotonic()
        if now - self._last_cmd_time > 1.0:
            self._cmd_count = 0
            self._last_cmd_time = now
        self._cmd_count += 1

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def cleanup(self) -> None:
        """Orderly shutdown: stop motion, open hand, kill subprocesses.

        Idempotent — safe to call multiple times.  Designed to be called from
        a signal handler, exception handler, or normal exit path.
        """
        if self._cleaned_up:
            self._logger.info("Cleanup already performed — skipping.")
            return

        self._logger.info("=" * 60)
        self._logger.info("EMG SAFETY CLEANUP STARTING")
        self._logger.info("=" * 60)

        # 1. Stop runtime monitor
        self.stop_runtime_monitor()

        # 2. Zero velocity on all joints
        self._step_cleanup("zero velocity on all joints", self._zero_velocity)

        # 3. Stop wrist motion
        if self._param("wrist_enabled"):
            self._step_cleanup("stop wrist motion", self._stop_wrist)

        # 4. Switch hand back to position control (already the default)
        self._step_cleanup("ensure position control mode",
                           self._ensure_position_control)

        # 5. Open/reset hand to safe joint positions
        self._step_cleanup("open hand to safe positions", self._open_hand_safe)

        # 6. Shut down launched processes
        self._step_cleanup("shut down launched processes",
                           self._kill_launched_processes)

        self._cleaned_up = True
        self._logger.info("=" * 60)
        self._logger.info("EMG SAFETY CLEANUP COMPLETE")
        self._logger.info("=" * 60)

    def _step_cleanup(self, label: str, fn) -> None:
        """Run a cleanup step; log errors but continue."""
        try:
            self._logger.info(f"[CLEANUP] {label} ...")
            fn()
            self._logger.info(f"[CLEANUP] {label} ✓")
        except Exception as exc:
            self._logger.error(f"[CLEANUP] {label} FAILED: {exc}")

    def _zero_velocity(self) -> None:
        """Command zero velocity on all finger joints.

        Publishes zero-velocity commands to the velocity controller topics.
        If velocity controllers are not active, this is a no-op warning.
        """
        vel_topics = [
            "/thumb_vel_ff_controller/commands",
            "/index_vel_ff_controller/commands",
            "/mrl_vel_ff_controller/commands",
        ]

        for topic in vel_topics:
            if topic not in self._cmd_pubs:
                self._cmd_pubs[topic] = self._node.create_publisher(
                    Float64MultiArray, topic, 10
                )

        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0]

        for topic in vel_topics:
            self._cmd_pubs[topic].publish(zero_msg)

        self._logger.info("Zero-velocity commands sent to all joints.")

    def _stop_wrist(self) -> None:
        """Stop wrist motion by commanding zero velocity."""
        topic = self._param("wrist_command_topic")
        if self._wrist_pub is None:
            self._wrist_pub = self._node.create_publisher(
                Float64MultiArray, topic, 10
            )

        # Send zero speed command (first entry = 0 velocity)
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]
        self._wrist_pub.publish(msg)
        self._logger.info(f"Wrist stop command sent to '{topic}'.")

    def _ensure_position_control(self) -> None:
        """Ensure the hand is in position-control mode.

        The default controller configuration already uses position
        feedforward controllers (``*_pos_ff_controller``).  We publish
        the current safe-open position to each to latch the controllers
        into position mode.
        """
        safe_pos = self._param("safe_open_positions")
        if len(safe_pos) < 3:
            safe_pos = _SAFE_OPEN_POSITIONS

        cmd_topics = [
            "/thumb_pos_ff_controller/commands",
            "/index_pos_ff_controller/commands",
            "/mrl_pos_ff_controller/commands",
        ]

        for i, topic in enumerate(cmd_topics):
            if topic not in self._cmd_pubs:
                self._cmd_pubs[topic] = self._node.create_publisher(
                    Float64MultiArray, topic, 10
                )
            msg = Float64MultiArray()
            pos = safe_pos[i] if i < len(safe_pos) else 0.0
            msg.data = [pos]
            self._cmd_pubs[topic].publish(msg)

        self._logger.info("Position-control commands sent to all finger controllers.")

    def _open_hand_safe(self) -> None:
        """Open/reset the hand to safe joint positions.

        Publishes the ``safe_open_positions`` to each position feedforward
        controller.
        """
        safe_pos = self._param("safe_open_positions")
        if not safe_pos or len(safe_pos) < 3:
            safe_pos = _SAFE_OPEN_POSITIONS

        cmd_topics = [
            "/thumb_pos_ff_controller/commands",
            "/index_pos_ff_controller/commands",
            "/mrl_pos_ff_controller/commands",
        ]

        for i, topic in enumerate(cmd_topics):
            if topic not in self._cmd_pubs:
                self._cmd_pubs[topic] = self._node.create_publisher(
                    Float64MultiArray, topic, 10
                )
            msg = Float64MultiArray()
            pos = safe_pos[i] if i < len(safe_pos) else 0.0
            msg.data = [pos]
            self._cmd_pubs[topic].publish(msg)

        self._logger.info(
            f"Safe open positions commanded: thumb={safe_pos[0]:.3f}, "
            f"index={safe_pos[1]:.3f}, mrl={safe_pos[2]:.3f} rad"
        )

    def _kill_launched_processes(self) -> None:
        """Kill any subprocesses that were launched by the pipeline."""
        if not self._launched_pids:
            self._logger.info("No launched processes to kill.")
            return

        for pid in self._launched_pids:
            try:
                os.kill(pid, signal.SIGTERM)
                self._logger.info(f"Sent SIGTERM to PID {pid}")
            except ProcessLookupError:
                self._logger.debug(f"PID {pid} already exited.")
            except PermissionError:
                self._logger.warn(f"Permission denied killing PID {pid}")

        self._launched_pids.clear()

    def register_child_pid(self, pid: int) -> None:
        """Register a child process PID for cleanup on shutdown."""
        if pid > 0:
            self._launched_pids.append(pid)

    # ── Convenience: register signal handlers ───────────────────────────────

    def install_signal_handlers(self) -> None:
        """Register cleanup on SIGINT, SIGTERM, and atexit."""
        def _handler(signum, frame):
            self._logger.warn(
                f"Received signal {signal.Signals(signum).name} — cleaning up"
            )
            self.cleanup()
            if self._own_node:
                self._node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            os._exit(0)

        signal.signal(signal.SIGINT, _handler)
        signal.signal(signal.SIGTERM, _handler)
        atexit.register(self.cleanup)
        self._logger.info("Signal handlers installed (SIGINT, SIGTERM, atexit).")

    # ── Standalone operation ─────────────────────────────────────────────────

    def spin_in_thread(self):
        """Spin the internal node in the current thread (blocking)."""
        if not self._own_node:
            raise RuntimeError(
                "spin_in_thread() requires internal node (no node passed to __init__)"
            )
        rclpy.spin(self._node)

    def run_standalone(self) -> int:
        """Run preflight + cleanup as a standalone node. Returns exit code."""
        if not self._own_node:
            raise RuntimeError(
                "run_standalone() requires internal node (no node passed to __init__)"
            )

        self.install_signal_handlers()

        ok = self.run_preflight()
        if not ok:
            return 1

        self.start_runtime_monitor()

        try:
            rclpy.spin(self._node)
        except KeyboardInterrupt:
            self._logger.info("KeyboardInterrupt received.")
        except Exception as exc:
            self._logger.error(f"Unhandled exception: {exc}")
        finally:
            self.cleanup()
            self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

        return 0


# ── Default parameter values (fallback if YAML not loaded) ───────────────────

_DEFAULT_PARAMS = {
    "emergency_stop_service":       "/mia_hand/emergency_stop",
    "play_service":                 "/mia_hand/play",
    "controller_manager_ns":        "",
    "required_controllers": [
        "joint_state_broadcaster",
        "thumb_pos_ff_controller",
        "index_pos_ff_controller",
        "mrl_pos_ff_controller",
    ],
    "force_data_topic":             "data_streams/fingers/forces/data",
    "joint_states_topic":           "/joint_states",
    "emg_label_topic":              "/emg/gesture_label",
    "emg_confidence_topic":         "/emg/confidence",
    "wrist_command_topic":          "/wrist/set_position",
    "wrist_state_topic":            "/wrist/state",
    "hand_command_topics": [
        "/thumb_pos_ff_controller/commands",
        "/index_pos_ff_controller/commands",
        "/mrl_pos_ff_controller/commands",
    ],
    "safe_open_positions":          [0.0, 0.0, 0.0],
    "safe_open_speed_pct":          80,
    "preflight_force_timeout_s":    5.0,
    "preflight_emg_timeout_s":      5.0,
    "preflight_controller_timeout_s": 10.0,
    "preflight_service_timeout_s":  3.0,
    "stale_emg_timeout_s":          0.5,
    "stale_force_timeout_s":        0.5,
    "stale_force_emergency_s":      2.0,
    "max_force_limit":              500,
    "max_wrist_error_deg":          45.0,
    "max_command_rate_hz":          50.0,
    "enable_runtime_monitor":       True,
    "runtime_check_rate_hz":        20.0,
    "auto_cleanup_on_violation":    True,
    "wrist_enabled":                False,
    "command_source_patterns": [
        "force_controller",
        "command_bridge",
        "emg_grasp",
        "grasp_proximity",
    ],
    "max_command_publishers":       3,
}


# ── Standalone entry point ───────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    safety = EmgSafetyPreflight()  # creates internal node
    exit_code = safety.run_standalone()
    return exit_code


if __name__ == "__main__":
    import sys
    sys.exit(main())
