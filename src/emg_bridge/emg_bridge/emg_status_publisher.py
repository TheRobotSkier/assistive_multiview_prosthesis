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

    supply_voltage: float = 0.0
    mcu_temp: float = 0.0

    preflight_ok: bool = False

    sequence_count: int = 0
    faults: Set[str] = field(default_factory=set)
    lock: threading.Lock = field(default_factory=threading.Lock)
    status_seq: int = 0


# ── Status publisher ───────────────────────────────────────────────────────────

class EmgStatusPublisher:
    """Console-only status publisher with throttling.

    Manages its own timing.  Uses the EmgGraspStatus dataclass as shared
    state between the main control loop and any ROS publisher thread.
    """

    def __init__(self, status_snapshot: EmgGraspStatus):
        self._status = status_snapshot

        # Throttling state
        self._last_full_status: float = 0.0
        self._last_force_detail: float = 0.0
        self._last_any_line: float = 0.0

        # Change detection for mode / faults
        self._prev_mode: str = ""
        self._prev_faults: Set[str] = set()
        self._prev_preflight: bool = False
        self._prev_phase: str = ""
        self._prev_hand_ctrl: str = ""

    def update(self) -> None:
        """Read the latest status snapshot, throttle, and print as needed."""

        with self._status.lock:
            s = self._status
            mode = s.mode
            gesture = s.gesture_name
            conf = s.confidence
            prop = s.proportional
            phase = s.grasp_phase
            faults = set(s.faults)
            preflight = s.preflight_ok
            hand_ctrl = s.hand_controller
            target = list(s.target_forces)
            measured = list(s.measured_forces)
            errors = list(s.force_errors)

        now = time.monotonic()

        # ── Always-log events (mode change, fault change, preflight change) ──
        if mode != self._prev_mode:
            self._log(f"[MODE] {mode}", now)
            self._prev_mode = mode

        if faults != self._prev_faults:
            added = faults - self._prev_faults
            removed = self._prev_faults - faults
            if added:
                self._log(f"[FAULT] +{','.join(sorted(added))}", now)
            if removed:
                self._log(f"[FAULT] -{','.join(sorted(removed))}", now)
            self._prev_faults = faults

        if preflight != self._prev_preflight:
            self._log(f"[PREFLIGHT] {'ok' if preflight else 'not_ready'}", now)
            self._prev_preflight = preflight

        if phase != self._prev_phase:
            self._log(f"[PHASE] {phase}", now)
            self._prev_phase = phase

        if hand_ctrl != self._prev_hand_ctrl:
            self._log(f"[HAND_CTRL] {hand_ctrl}", now)
            self._prev_hand_ctrl = hand_ctrl

        # ── Routine status summary (throttled ~2 s) ──────────────────────────
        if now - self._last_full_status >= 2.0:
            fault_str = ",".join(sorted(faults)) if faults else "none"
            self._log(
                f"mode={mode} gesture={gesture} conf={conf:.2f} prop={prop:.2f} "
                f"phase={phase} faults={fault_str} preflight={'ok' if preflight else 'not_ready'}",
                now,
            )
            self._last_full_status = now

        # ── Force detail (throttled ~1 s) ────────────────────────────────────
        if now - self._last_force_detail >= 1.0:
            self._log(
                f"target={target[0]:.0f},{target[1]:.0f},{target[2]:.0f} "
                f"meas={measured[0]:.0f},{measured[1]:.0f},{measured[2]:.0f} "
                f"err={errors[0]:.0f},{errors[1]:.0f},{errors[2]:.0f}",
                now,
            )
            self._last_force_detail = now

    def _log(self, msg: str, now: float) -> None:
        """Print to stderr with minimum 0.5 s throttle."""
        if now - self._last_any_line < 0.5:
            return
        print(f"[EMG_STATUS] {msg}")
        self._last_any_line = now


# ── ROS -> text helpers ────────────────────────────────────────────────────────

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
