"""
EMG grasp status publisher — readable operator diagnostics.

Provides concise console logging and ROS topic publishing for live operation.
Use EmgStatusPublisher from the main control loop; pass optional ROS Node for
topic publishing.

Console output rules (enforced by throttling):
  - Always log: mode changes, force target changes, controller switches,
    safety resets, stale data, faults
  - Routine status summary: throttled to ~every 2 seconds
  - Force detail: throttled to ~every 1 second
  - Minimum 0.5 s between any console lines

Fault codes (explicit, always logged):
  STALE_EMG, STALE_FORCE, CTRL_UNAVAILABLE, WRIST_UNAVAILABLE,
  OVERFORCE, CMD_CONFLICT

ROS topics (when ros_node is provided):
  /emg_grasp/status_text   std_msgs/String   — key=value lines
  /emg_grasp/status        std_msgs/Float64MultiArray — numeric snapshot
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Set

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String as RosString
    from std_msgs.msg import Float64MultiArray
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


# ── Fault codes ────────────────────────────────────────────────────────────────

class FaultCode:
    STALE_EMG = "STALE_EMG"
    STALE_FORCE = "STALE_FORCE"
    CTRL_UNAVAILABLE = "CTRL_UNAVAILABLE"
    WRIST_UNAVAILABLE = "WRIST_UNAVAILABLE"
    OVERFORCE = "OVERFORCE"
    CMD_CONFLICT = "CMD_CONFLICT"

    ALL = frozenset([
        STALE_EMG, STALE_FORCE, CTRL_UNAVAILABLE,
        WRIST_UNAVAILABLE, OVERFORCE, CMD_CONFLICT,
    ])


# ── Modes ─────────────────────────────────────────────────────────────────────

class EmgMode:
    NOT_GRASPING = "not-grasping"
    CONTROL_GRASP = "control-grasp"
    CONTROL_WRIST = "control-wrist"


class GraspPhase:
    IDLE = "idle"
    CLOSING = "closing"
    FORCE_HOLD = "force_hold"
    RELEASING = "releasing"
    FAULT = "fault"


class PowerRearm:
    DISARMED = "disarmed"
    ARMED = "armed"
    HOLDING = "holding"


# ── Mutable status snapshot ───────────────────────────────────────────────────

@dataclass
class EmgGraspStatus:
    """Thread-safe status snapshot shared between main thread and ROS thread."""

    mode: str = EmgMode.NOT_GRASPING
    gesture_name: str = "REST"
    confidence: float = 0.0
    proportional: float = 0.0

    open_hold_s: float = 0.0
    power_rearm: str = PowerRearm.DISARMED

    grasp_phase: str = GraspPhase.IDLE
    target_forces: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    measured_forces: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    force_errors: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    hand_controller: str = "none"

    wrist_state: str = "idle"
    wrist_cmd: float = 0.0

    faults: Set[str] = field(default_factory=set)
    preflight_ok: bool = False

    lock: threading.Lock = field(default_factory=threading.Lock)
    status_seq: int = 0


# ── Status publisher ──────────────────────────────────────────────────────────

class EmgStatusPublisher:
    """Tracks and publishes operator-readable status for EMG grasp test.

    Use from the main control loop (single-threaded callers).  If a *ros_node*
    is supplied the publisher will also populate ROS topics.

    Throttle behaviour
    ------------------
    *Routine* summaries print at most every ``ROUTINE_INTERVAL`` (2 s).
    *Force* detail prints at most every ``FORCE_LOG_INTERVAL`` (1 s).
    *Important* events (mode change, fault, controller switch, …) print
    immediately regardless of throttle state.
    """

    ROUTINE_INTERVAL = 2.0   # seconds between routine summaries
    FORCE_LOG_INTERVAL = 1.0  # seconds between force-target detail lines
    MIN_LOG_INTERVAL = 0.5    # minimum spacing between any console line

    def __init__(self, ros_node: Optional[Node] = None):
        self._state = EmgGraspStatus()

        # Previous values for change detection
        self._prev_mode: str = EmgMode.NOT_GRASPING
        self._prev_gesture: str = "REST"
        self._prev_phase: str = GraspPhase.IDLE
        self._prev_controller: str = "none"
        self._prev_target: List[float] = [0.0, 0.0, 0.0]
        self._prev_faults: Set[str] = set()

        # Throttle timestamps
        self._last_routine_log: float = 0.0
        self._last_any_log: float = 0.0
        self._last_force_log: float = 0.0

        # ROS publishing (optional)
        self._string_pub = None
        self._array_pub = None
        if ros_node is not None and HAS_ROS:
            self._string_pub = ros_node.create_publisher(
                RosString, "/emg_grasp/status_text", 10
            )
            self._array_pub = ros_node.create_publisher(
                Float64MultiArray, "/emg_grasp/status", 10
            )

    # ── State setters ──────────────────────────────────────────────────────

    def set_mode(self, mode: str) -> None:
        """Set current EMG mode (not-grasping / control-grasp / control-wrist)."""
        with self._state.lock:
            self._state.mode = mode
        if mode != self._prev_mode:
            self._always_log(f"MODE: {self._prev_mode} → {mode}")
            self._prev_mode = mode

    def set_gesture(self, name: str, confidence: float,
                    proportional: float = 0.0) -> None:
        """Set active normalized gesture and confidence."""
        with self._state.lock:
            self._state.gesture_name = name
            self._state.confidence = confidence
            self._state.proportional = proportional
        if name != self._prev_gesture:
            self._always_log(
                f"GESTURE: {name}  conf={confidence:.2f}  prop={proportional:.2f}"
            )
            self._prev_gesture = name

    def set_open_hold(self, held_s: float) -> None:
        """Set OPEN safety-hold progress in seconds."""
        with self._state.lock:
            self._state.open_hold_s = held_s

    def set_power_rearm(self, state: str) -> None:
        """Set POWER rearm state (disarmed / armed / holding)."""
        with self._state.lock:
            self._state.power_rearm = state

    def set_grasp_phase(self, phase: str) -> None:
        """Set hand grasp phase (idle / closing / force_hold / releasing / fault)."""
        with self._state.lock:
            self._state.grasp_phase = phase
        if phase != self._prev_phase:
            self._always_log(f"GRASP PHASE: {self._prev_phase} → {phase}")
            self._prev_phase = phase

    def set_target_forces(self, forces: List[float]) -> None:
        """Set target force per finger [thumb, index, mrl]."""
        with self._state.lock:
            self._state.target_forces = list(forces)
        if any(abs(f - p) > 1.0 for f, p in zip(forces, self._prev_target)):
            self._log_force(
                f"TARGET FORCE: [{forces[0]:.0f}, {forces[1]:.0f}, {forces[2]:.0f}]"
            )
            self._prev_target = list(forces)

    def set_measured_forces(self, forces: List[float]) -> None:
        """Set measured (actual) force per finger."""
        with self._state.lock:
            self._state.measured_forces = list(forces)

    def set_force_errors(self, errors: List[float]) -> None:
        """Set force error (target - actual) per finger."""
        with self._state.lock:
            self._state.force_errors = list(errors)

    def set_hand_controller(self, mode: str) -> None:
        """Set active hand controller mode (e.g. velocity, position, force)."""
        with self._state.lock:
            self._state.hand_controller = mode
        if mode != self._prev_controller:
            self._always_log(
                f"HAND CONTROLLER: {self._prev_controller} → {mode}"
            )
            self._prev_controller = mode

    def set_wrist(self, state: str, cmd: float = 0.0) -> None:
        """Set wrist state and command (degrees)."""
        with self._state.lock:
            self._state.wrist_state = state
            self._state.wrist_cmd = cmd

    def add_fault(self, code: str) -> None:
        """Add an active fault code.  No-op if already present."""
        if code not in FaultCode.ALL:
            raise ValueError(f"Unknown fault code: {code}")
        with self._state.lock:
            if code not in self._state.faults:
                self._state.faults.add(code)
                self._always_log(f"FAULT: {code}")
                self._prev_faults = self._state.faults.copy()

    def clear_fault(self, code: str) -> None:
        """Clear a previously active fault code."""
        with self._state.lock:
            if code in self._state.faults:
                self._state.faults.discard(code)
                self._always_log(f"FAULT CLEARED: {code}")
                self._prev_faults = self._state.faults.copy()

    def set_preflight(self, ok: bool) -> None:
        """Set preflight check status."""
        with self._state.lock:
            self._state.preflight_ok = ok

    # ── Logging helpers ────────────────────────────────────────────────────

    def _always_log(self, msg: str) -> None:
        """Log immediately — used for important events."""
        now = time.monotonic()
        self._last_any_log = now
        print(f"[EMG-STATUS] {msg}")

    def _log_force(self, msg: str) -> None:
        """Log force detail (throttled separately)."""
        now = time.monotonic()
        if now - self._last_force_log >= self.FORCE_LOG_INTERVAL:
            self._last_force_log = now
            self._last_any_log = now
            print(f"[EMG-STATUS] {msg}")

    def force_log(self, msg: str) -> None:
        """Unconditional one-shot log."""
        now = time.monotonic()
        self._last_any_log = now
        print(f"[EMG-STATUS] {msg}")

    def throttled_log(self) -> None:
        """Print a routine status summary (throttled to ROUTINE_INTERVAL).

        Call this once per control-loop iteration.  The method is cheap when
        the interval hasn't elapsed.
        """
        now = time.monotonic()
        if now - self._last_routine_log < self.ROUTINE_INTERVAL:
            return
        if now - self._last_any_log < self.MIN_LOG_INTERVAL:
            return

        self._last_routine_log = now
        self._last_any_log = now

        with self._state.lock:
            s = self._state
            faults_str = ", ".join(sorted(s.faults)) if s.faults else "none"
            lines = [
                f"┌─ EMG Grasp @ {time.strftime('%H:%M:%S')}",
                f"├ Mode: {s.mode}  |  Gesture: {s.gesture_name} "
                f"(conf={s.confidence:.2f} prop={s.proportional:.2f})",
                f"├ Grasp Phase: {s.grasp_phase}  |  Hand Ctrl: {s.hand_controller}",
                f"├ Forces — Target: [{s.target_forces[0]:.0f}, "
                f"{s.target_forces[1]:.0f}, {s.target_forces[2]:.0f}]  "
                f"Meas: [{s.measured_forces[0]:.0f}, "
                f"{s.measured_forces[1]:.0f}, {s.measured_forces[2]:.0f}]  "
                f"Err: [{s.force_errors[0]:.0f}, "
                f"{s.force_errors[1]:.0f}, {s.force_errors[2]:.0f}]",
                f"├ Wrist: {s.wrist_state} (cmd={s.wrist_cmd:.1f}°)  "
                f"OPEN hold: {s.open_hold_s:.1f}s  POWER: {s.power_rearm}",
                f"├ Faults: {faults_str}",
                f"└ Preflight: {'OK' if s.preflight_ok else 'NOT READY'}",
            ]
        for line in lines:
            print(line)

    # ── ROS publishing ─────────────────────────────────────────────────────

    def publish_ros(self) -> None:
        """Publish current status to ROS topics.

        Safe to call even when no ROS node was provided (no-op).
        """
        if self._string_pub is None or self._array_pub is None:
            return

        with self._state.lock:
            s = self._state
            s.status_seq += 1
            seq = s.status_seq
            text = _build_status_text(s, seq)
            numeric = _build_numeric(s, seq)

        self._string_pub.publish(RosString(data=text))
        self._array_pub.publish(numeric)

    # ── Convenience: full update ───────────────────────────────────────────

    def update_full(
        self,
        *,
        mode: Optional[str] = None,
        gesture_name: Optional[str] = None,
        confidence: Optional[float] = None,
        proportional: Optional[float] = None,
        open_hold_s: Optional[float] = None,
        power_rearm: Optional[str] = None,
        grasp_phase: Optional[str] = None,
        target_forces: Optional[List[float]] = None,
        measured_forces: Optional[List[float]] = None,
        force_errors: Optional[List[float]] = None,
        hand_controller: Optional[str] = None,
        wrist_state: Optional[str] = None,
        wrist_cmd: Optional[float] = None,
    ) -> None:
        """Bulk update — calls set_* methods only for non-None arguments."""
        if mode is not None:
            self.set_mode(mode)
        if gesture_name is not None:
            conf = confidence if confidence is not None else self._state.confidence
            prop = proportional if proportional is not None else self._state.proportional
            self.set_gesture(gesture_name, conf, prop)
        elif confidence is not None or proportional is not None:
            self.set_gesture(
                self._state.gesture_name,
                confidence if confidence is not None else self._state.confidence,
                proportional if proportional is not None else self._state.proportional,
            )
        if open_hold_s is not None:
            self.set_open_hold(open_hold_s)
        if power_rearm is not None:
            self.set_power_rearm(power_rearm)
        if grasp_phase is not None:
            self.set_grasp_phase(grasp_phase)
        if target_forces is not None:
            self.set_target_forces(target_forces)
        if measured_forces is not None:
            self.set_measured_forces(measured_forces)
        if force_errors is not None:
            self.set_force_errors(force_errors)
        if hand_controller is not None:
            self.set_hand_controller(hand_controller)
        if wrist_state is not None:
            self.set_wrist(wrist_state, wrist_cmd or 0.0)


# ── ROS → text helpers ────────────────────────────────────────────────────────

def _build_status_text(s: EmgGraspStatus, seq: int) -> str:
    """Serialize status snapshot to key=value lines."""
    faults = ",".join(sorted(s.faults)) if s.faults else "none"
    return "\n".join([
        f"seq:{seq}",
        f"mode:{s.mode}",
        f"gesture:{s.gesture_name}",
        f"conf:{s.confidence:.3f}",
        f"prop:{s.proportional:.3f}",
        f"open_hold:{s.open_hold_s:.1f}",
        f"power_rearm:{s.power_rearm}",
        f"grasp_phase:{s.grasp_phase}",
        f"hand_ctrl:{s.hand_controller}",
        f"wrist_state:{s.wrist_state}",
        f"wrist_cmd:{s.wrist_cmd:.1f}",
        f"target_force:{s.target_forces[0]:.0f},{s.target_forces[1]:.0f},{s.target_forces[2]:.0f}",
        f"meas_force:{s.measured_forces[0]:.0f},{s.measured_forces[1]:.0f},{s.measured_forces[2]:.0f}",
        f"force_err:{s.force_errors[0]:.0f},{s.force_errors[1]:.0f},{s.force_errors[2]:.0f}",
        f"faults:{faults}",
        f"preflight:{'ok' if s.preflight_ok else 'not_ready'}",
    ])


def _build_numeric(s: EmgGraspStatus, seq: int):
    """Pack status snapshot into a fixed-layout Float64MultiArray.

    Only usable when ROS 2 is installed (HAS_ROS is True).  Raises
    RuntimeError otherwise.
    """
    if not HAS_ROS:
        raise RuntimeError(
            "Cannot build numeric status without ROS 2 installed."
        )
    msg = Float64MultiArray()
    msg.data = [
        float(seq),
        s.confidence,
        s.proportional,
        s.open_hold_s,
        s.target_forces[0], s.target_forces[1], s.target_forces[2],
        s.measured_forces[0], s.measured_forces[1], s.measured_forces[2],
        s.force_errors[0], s.force_errors[1], s.force_errors[2],
        s.wrist_cmd,
        1.0 if s.preflight_ok else 0.0,
    ]
    return msg


# ── Standalone ROS Node wrapper ───────────────────────────────────────────────

if HAS_ROS:

    class EmgStatusRosNode(Node):
        """Standalone ROS 2 node that publishes EMG grasp status.

        Shares a ``EmgGraspStatus`` with the main application.  Reads are
        lock-protected.  Publishes at 2 Hz.
        """

        def __init__(self, status: EmgGraspStatus):
            super().__init__("emg_status_publisher")
            self._status = status
            self._string_pub = self.create_publisher(
                RosString, "/emg_grasp/status_text", 10
            )
            self._array_pub = self.create_publisher(
                Float64MultiArray, "/emg_grasp/status", 10
            )
            self.create_timer(0.5, self._publish)  # 2 Hz

        def _publish(self):
            with self._status.lock:
                self._status.status_seq += 1
                seq = self._status.status_seq
                text = _build_status_text(self._status, seq)
                numeric = _build_numeric(self._status, seq)
            self._string_pub.publish(RosString(data=text))
            self._array_pub.publish(numeric)


    def start_ros_node(status: EmgGraspStatus) -> threading.Thread:
        """Start EmgStatusRosNode in a daemon thread. Returns thread handle."""

        def _run():
            rclpy.init()
            node = EmgStatusRosNode(status)
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()

        t = threading.Thread(target=_run, daemon=True)
        t.start()
        return t

else:
    def start_ros_node(status: EmgGraspStatus) -> None:
        """No-op when ROS 2 is not installed."""
        return None
