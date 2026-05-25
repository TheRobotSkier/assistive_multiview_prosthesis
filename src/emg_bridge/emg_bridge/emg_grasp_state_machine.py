"""Three-mode EMG state machine for prosthesis grasp control.

Emits high-level *intents* rather than directly actuating hardware.
This keeps mode logic separate from low-level motor control.

Modes:
  NOT_GRASPING  – startup / safety mode.  Hand must be open.
  CONTROL_GRASP – force-controlled grasp.
  CONTROL_WRIST – wrist position / velocity control.

Gesture contract (labels are configurable):
  OPEN     – safety reset (pre-empts everything)
  POWER    – mode toggle
  FLEXION  – positive action (wrist + / tighten force)
  EXTENSION– negative action (wrist – / loosen force)
  REST     – neutral / stop
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass, field
from typing import List, Optional


class Mode(enum.IntEnum):
    NOT_GRASPING = 0
    CONTROL_GRASP = 1
    CONTROL_WRIST = 2


class IntentType(enum.Enum):
    RESET_HAND = "reset_hand"
    ENTER_FORCE_HOLD = "enter_force_hold"
    ADJUST_FORCE_TARGET = "adjust_force_target"
    WRIST_VELOCITY_POSITIVE = "wrist_velocity_positive"
    WRIST_VELOCITY_NEGATIVE = "wrist_velocity_negative"
    STOP_ALL = "stop_all"


@dataclass(frozen=True)
class Intent:
    """A single intent emitted by the state machine."""

    intent_type: IntentType
    # Contextual magnitude / direction (e.g. force delta, velocity scale)
    value: float = 0.0
    # Human-readable reason for logging
    reason: str = ""


@dataclass
class EmgInput:
    """Snapshot of EMG classifier output consumed by the state machine."""

    gesture: int = 0
    confidence: float = 0.0
    proportional: float = 0.0
    timestamp: float = field(default_factory=time.monotonic)


@dataclass
class StateMachineConfig:
    """Tunable parameters for the three-mode state machine."""

    # Gesture label mapping (defaults match emg_bridge.config GESTURE_NAMES)
    gesture_rest: int = 0
    gesture_power: int = 1
    gesture_flexion: int = 3   # matches classifier: 3=FLEXION
    gesture_extension: int = 4  # matches classifier: 4=EXTENSION
    gesture_open: int = 2       # matches classifier: 2=OPEN

    # Timing
    open_hold_duration_s: float = 1.0
    open_proportional_threshold: float = 0.7
    power_hold_duration_s: float = 1.0
    power_rearm_duration_s: float = 0.3
    stale_timeout_s: float = 1.0

    # Magnitudes
    force_adjust_step: float = 1.0  # raw sensor units per tick
    wrist_velocity_scale: float = 1.0  # scaling factor for proportional control


class EmgGraspStateMachine:
    """Pure (no-ROS) three-mode EMG state machine.

    Usage:
        cfg = StateMachineConfig()
        sm = EmgGraspStateMachine(cfg)
        intents = sm.update(EmgInput(gesture=1, confidence=0.9, proportional=0.8))
        # intents is a list of Intent objects to be handled by the caller
    """

    def __init__(self, config: Optional[StateMachineConfig] = None) -> None:
        self._cfg = config or StateMachineConfig()
        self._mode: Mode = Mode.NOT_GRASPING

        # Gesture tracking
        self._open_hold_start: Optional[float] = None
        self._power_press_start: Optional[float] = None
        self._power_rearm_time: float = 0.0  # monotonic time before which POWER is ignored

        # Last-seen gesture label (for hold-duration logic)
        self._prev_gesture: int = -1

    # ── Public API ──────────────────────────────────────────────────────────

    @property
    def mode(self) -> Mode:
        return self._mode

    @property
    def mode_name(self) -> str:
        return self._mode.name

    @property
    def config(self) -> StateMachineConfig:
        return self._cfg

    def update(self, inp: EmgInput) -> List[Intent]:
        """Process one EMG input frame. Returns a (possibly empty) list of intents.

        Thread-safe *only* if called from a single thread (the ROS timer).
        """
        now = inp.timestamp

        # ── OPEN is the universal safety reset ─────────────────────────────
        if inp.gesture == self._cfg.gesture_open:
            # Track hold duration
            if self._prev_gesture != self._cfg.gesture_open:
                self._open_hold_start = now
            hold = now - self._open_hold_start if self._open_hold_start else 0.0
            self._prev_gesture = inp.gesture

            if (
                hold >= self._cfg.open_hold_duration_s
                and inp.proportional >= self._cfg.open_proportional_threshold
            ):
                if self._mode != Mode.NOT_GRASPING:
                    self._mode = Mode.NOT_GRASPING
                    self._open_hold_start = None
                    self._power_rearm_time = now + self._cfg.power_rearm_duration_s
                    return [Intent(IntentType.RESET_HAND, reason="OPEN confirmed")]
            return []

        # ── Gesture-POWER toggle logic ──────────────────────────────────────
        power_toggle = False
        if inp.gesture == self._cfg.gesture_power:
            if self._prev_gesture != self._cfg.gesture_power:
                self._power_press_start = now
            hold = now - self._power_press_start if self._power_press_start else 0.0
            self._prev_gesture = inp.gesture

            if hold >= self._cfg.power_hold_duration_s and now >= self._power_rearm_time:
                power_toggle = True
                self._power_rearm_time = now + self._cfg.power_rearm_duration_s
                self._power_press_start = None
        else:
            self._prev_gesture = inp.gesture

        # ── Route to current-mode handler ────────────────────────────────────
        if self._mode == Mode.NOT_GRASPING:
            return self._handle_not_grasping(inp, now, power_toggle)
        elif self._mode == Mode.CONTROL_GRASP:
            return self._handle_control_grasp(inp, now, power_toggle)
        elif self._mode == Mode.CONTROL_WRIST:
            return self._handle_control_wrist(inp, now, power_toggle)
        else:
            return []

    # ── Mode handlers ─────────────────────────────────────────────────────────

    def _handle_not_grasping(
        self, inp: EmgInput, now: float, power_toggle: bool
    ) -> List[Intent]:
        intents: List[Intent] = []

        if power_toggle:
            self._mode = Mode.CONTROL_GRASP
            intents.append(
                Intent(
                    IntentType.ENTER_FORCE_HOLD,
                    reason="POWER in NOT_GRASPING -> CONTROL_GRASP",
                )
            )
            return intents

        if inp.gesture == self._cfg.gesture_flexion:
            intents.append(
                Intent(
                    IntentType.WRIST_VELOCITY_POSITIVE,
                    value=inp.proportional * self._cfg.wrist_velocity_scale,
                    reason="FLEXION in NOT_GRASPING -> wrist +",
                )
            )
        elif inp.gesture == self._cfg.gesture_extension:
            intents.append(
                Intent(
                    IntentType.WRIST_VELOCITY_NEGATIVE,
                    value=inp.proportional * self._cfg.wrist_velocity_scale,
                    reason="EXTENSION in NOT_GRASPING -> wrist -",
                )
            )
        elif inp.gesture == self._cfg.gesture_rest:
            intents.append(
                Intent(IntentType.STOP_ALL, reason="REST in NOT_GRASPING -> stop")
            )

        return intents

    def _handle_control_grasp(
        self, inp: EmgInput, now: float, power_toggle: bool
    ) -> List[Intent]:
        intents: List[Intent] = []

        if power_toggle:
            self._mode = Mode.CONTROL_WRIST
            intents.append(
                Intent(
                    IntentType.STOP_ALL,
                    reason="POWER in CONTROL_GRASP -> CONTROL_WRIST (stop grasp)",
                )
            )
            return intents

        if inp.gesture == self._cfg.gesture_flexion:
            intents.append(
                Intent(
                    IntentType.ADJUST_FORCE_TARGET,
                    value=+self._cfg.force_adjust_step * inp.proportional,
                    reason="FLEXION in CONTROL_GRASP -> increase force target",
                )
            )
        elif inp.gesture == self._cfg.gesture_extension:
            intents.append(
                Intent(
                    IntentType.ADJUST_FORCE_TARGET,
                    value=-self._cfg.force_adjust_step * inp.proportional,
                    reason="EXTENSION in CONTROL_GRASP -> decrease force target",
                )
            )
        elif inp.gesture == self._cfg.gesture_rest:
            intents.append(
                Intent(IntentType.STOP_ALL, reason="REST in CONTROL_GRASP -> stop")
            )

        return intents

    def _handle_control_wrist(
        self, inp: EmgInput, now: float, power_toggle: bool
    ) -> List[Intent]:
        intents: List[Intent] = []

        if power_toggle:
            self._mode = Mode.CONTROL_GRASP
            intents.append(
                Intent(
                    IntentType.ENTER_FORCE_HOLD,
                    reason="POWER in CONTROL_WRIST -> CONTROL_GRASP",
                )
            )
            return intents

        if inp.gesture == self._cfg.gesture_flexion:
            intents.append(
                Intent(
                    IntentType.WRIST_VELOCITY_POSITIVE,
                    value=inp.proportional * self._cfg.wrist_velocity_scale,
                    reason="FLEXION in CONTROL_WRIST -> wrist +",
                )
            )
        elif inp.gesture == self._cfg.gesture_extension:
            intents.append(
                Intent(
                    IntentType.WRIST_VELOCITY_NEGATIVE,
                    value=inp.proportional * self._cfg.wrist_velocity_scale,
                    reason="EXTENSION in CONTROL_WRIST -> wrist -",
                )
            )
        elif inp.gesture == self._cfg.gesture_rest:
            intents.append(
                Intent(IntentType.STOP_ALL, reason="REST in CONTROL_WRIST -> stop")
            )

        return intents
