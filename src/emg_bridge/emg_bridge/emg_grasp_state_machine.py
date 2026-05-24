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
    gesture_flexion: int = 2   # PINCH in current config
    gesture_extension: int = 4  # POINT in current config
    gesture_open: int = 3

    # Timing
    open_hold_duration_s: float = 0.5
    open_proportional_threshold: float = 0.7
    power_hold_duration_s: float = 0.5
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
        self._current_gesture: int = self._cfg.gesture_rest
        self._gesture_start_time: float = 0.0
        self._last_input_time: float = 0.0

        # POWER debounce
        self._power_toggle_armed: bool = True
        self._power_released_since_toggle: bool = True
        self._last_power_toggle_time: float = 0.0

        # OPEN safety tracking
        self._open_hold_start_time: Optional[float] = None

    # ── Public API ──────────────────────────────────────────────────────────

    @property
    def mode(self) -> Mode:
        return self._mode

    @property
    def mode_name(self) -> str:
        return self._mode.name

    def reset(self) -> List[Intent]:
        """Hard reset to NOT_GRASPING.  Returns reset_hand intent."""
        self._mode = Mode.NOT_GRASPING
        self._current_gesture = self._cfg.gesture_rest
        self._gesture_start_time = 0.0
        self._open_hold_start_time = None
        self._power_toggle_armed = True
        self._power_released_since_toggle = True
        return [Intent(IntentType.RESET_HAND, reason="State machine hard reset")]

    def update(self, inp: EmgInput) -> List[Intent]:
        """Process a new EMG input sample and return a list of intents."""
        intents: List[Intent] = []
        now = inp.timestamp
        self._last_input_time = now

        # Detect gesture changes
        if inp.gesture != self._current_gesture:
            self._current_gesture = inp.gesture
            self._gesture_start_time = now
            self._open_hold_start_time = None

        # ── OPEN safety pre-emption ───────────────────────────────────────
        open_intents = self._check_open_safety(inp, now)
        if open_intents:
            return open_intents

        # ── Stale data check ────────────────────────────────────────────────
        # Handled implicitly: caller is expected to call update() at the EMG
        # publishing rate.  If the caller stops calling, no new intents are
        # emitted.  We do not auto-transition on stale data here because the
        # ROS wrapper handles the timeout and feeds neutral REST inputs.

        # ── POWER debounce / toggle ────────────────────────────────────────
        power_toggle = self._check_power_toggle(inp, now)

        # ── Mode-specific behaviour ────────────────────────────────────────
        if self._mode == Mode.NOT_GRASPING:
            intents.extend(self._handle_not_grasping(inp, now, power_toggle))
        elif self._mode == Mode.CONTROL_GRASP:
            intents.extend(self._handle_control_grasp(inp, now, power_toggle))
        elif self._mode == Mode.CONTROL_WRIST:
            intents.extend(self._handle_control_wrist(inp, now, power_toggle))

        return intents

    # ── Internal helpers ──────────────────────────────────────────────────

    def _gesture_duration(self, now: float) -> float:
        return now - self._gesture_start_time

    def _check_open_safety(self, inp: EmgInput, now: float) -> List[Intent]:
        """OPEN held for >= open_hold_duration with high proportional -> reset."""
        if inp.gesture != self._cfg.gesture_open:
            self._open_hold_start_time = None
            return []

        if self._open_hold_start_time is None:
            self._open_hold_start_time = now

        held = now - self._open_hold_start_time
        if (
            held >= self._cfg.open_hold_duration_s
            and inp.proportional >= self._cfg.open_proportional_threshold
        ):
            if self._mode != Mode.NOT_GRASPING:
                self._mode = Mode.NOT_GRASPING
                self._power_toggle_armed = True
                self._power_released_since_toggle = True
                return [
                    Intent(
                        IntentType.RESET_HAND,
                        reason=f"OPEN safety: held {held:.2f}s, prop={inp.proportional:.2f}",
                    )
                ]
            # Already in NOT_GRASPING: no intent needed, but keep tracking
        return []

    def _check_power_toggle(self, inp: EmgInput, now: float) -> bool:
        """Returns True once when POWER has been held long enough and is re-armed.

        Rearm requires: (1) POWER was released since last toggle, and
        (2) rearm_duration seconds elapsed since last toggle.  A continuous
        POWER hold fires exactly one toggle — no repeat toggles from one hold.
        If POWER is re-asserted before the rearm duration elapses, that hold
        is permanently disqualified (user must release again).
        """
        # Track whether POWER was released since the last toggle
        if inp.gesture != self._cfg.gesture_power:
            self._power_released_since_toggle = True
        elif self._power_released_since_toggle:
            # POWER is active again; check if rearm duration has elapsed
            if not self._power_toggle_armed:
                if now - self._last_power_toggle_time >= self._cfg.power_rearm_duration_s:
                    self._power_toggle_armed = True
                else:
                    # Re-asserted too soon — disqualify this hold
                    self._power_released_since_toggle = False

        if inp.gesture != self._cfg.gesture_power:
            return False

        if not self._power_toggle_armed:
            return False

        held = self._gesture_duration(now)
        if held >= self._cfg.power_hold_duration_s:
            self._power_toggle_armed = False
            self._power_released_since_toggle = False
            self._last_power_toggle_time = now
            return True

        return False

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
