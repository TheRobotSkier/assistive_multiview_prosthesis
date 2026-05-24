"""Unit tests for the three-mode EMG grasp state machine.

Run with pytest (no ROS2 required).
"""

from __future__ import annotations

import time

import pytest

from emg_bridge.emg_grasp_state_machine import (
    EmgGraspStateMachine,
    EmgInput,
    IntentType,
    Mode,
    StateMachineConfig,
)


# ── Fixtures ──────────────────────────────────────────────────────────────

@pytest.fixture
def default_cfg() -> StateMachineConfig:
    return StateMachineConfig(
        gesture_rest=0,
        gesture_power=1,
        gesture_flexion=2,
        gesture_extension=4,
        gesture_open=3,
        open_hold_duration_s=0.5,
        open_proportional_threshold=0.7,
        power_hold_duration_s=0.5,
        power_rearm_duration_s=0.3,
        stale_timeout_s=1.0,
        force_adjust_step=1.0,
        wrist_velocity_scale=1.0,
    )


@pytest.fixture
def sm(default_cfg: StateMachineConfig) -> EmgGraspStateMachine:
    return EmgGraspStateMachine(default_cfg)


# ── Helper ──────────────────────────────────────────────────────────────────

def _input(
    gesture: int,
    confidence: float = 1.0,
    proportional: float = 1.0,
    timestamp: float | None = None,
) -> EmgInput:
    return EmgInput(
        gesture=gesture,
        confidence=confidence,
        proportional=proportional,
        timestamp=time.monotonic() if timestamp is None else timestamp,
    )


def _intent_types(intents):
    return [i.intent_type for i in intents]


# ── 1. Startup mode ───────────────────────────────────────────────────────

def test_startup_mode_is_not_grasping(sm: EmgGraspStateMachine) -> None:
    assert sm.mode == Mode.NOT_GRASPING
    assert sm.mode_name == "NOT_GRASPING"


def test_reset_returns_reset_hand(sm: EmgGraspStateMachine) -> None:
    intents = sm.reset()
    assert len(intents) == 1
    assert intents[0].intent_type == IntentType.RESET_HAND


# ── 2. OPEN pre-emption from every mode ────────────────────────────────────

class TestOpenPreemption:
    def test_open_from_not_grasping_no_reset(self, sm: EmgGraspStateMachine) -> None:
        # OPEN in NOT_GRASPING should not emit reset if already there
        t = 0.0
        intents = sm.update(_input(gesture=3, proportional=0.8, timestamp=t))
        assert IntentType.RESET_HAND not in _intent_types(intents)
        assert sm.mode == Mode.NOT_GRASPING

    def test_open_from_not_grasping_after_hold(self, sm: EmgGraspStateMachine) -> None:
        t = 0.0
        sm.update(_input(gesture=3, proportional=0.8, timestamp=t))
        t += 0.6
        intents = sm.update(_input(gesture=3, proportional=0.8, timestamp=t))
        # Already in NOT_GRASPING, so no reset needed
        assert IntentType.RESET_HAND not in _intent_types(intents)

    def test_open_from_control_grasp(self, sm: EmgGraspStateMachine) -> None:
        # Enter CONTROL_GRASP via POWER hold
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        intents = sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        # Now OPEN with high proportional held long enough
        t += 0.1
        sm.update(_input(gesture=3, proportional=0.8, timestamp=t))
        t += 0.6
        intents = sm.update(_input(gesture=3, proportional=0.8, timestamp=t))
        assert sm.mode == Mode.NOT_GRASPING
        assert IntentType.RESET_HAND in _intent_types(intents)

    def test_open_from_control_wrist(self, sm: EmgGraspStateMachine) -> None:
        # Enter CONTROL_GRASP
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        # Toggle to CONTROL_WRIST
        t += 0.4  # re-arm
        sm.update(_input(gesture=0, timestamp=t))  # release
        t += 0.4
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_WRIST

        # OPEN safety
        t += 0.1
        sm.update(_input(gesture=3, proportional=0.8, timestamp=t))
        t += 0.6
        intents = sm.update(_input(gesture=3, proportional=0.8, timestamp=t))
        assert sm.mode == Mode.NOT_GRASPING
        assert IntentType.RESET_HAND in _intent_types(intents)

    def test_open_low_proportional_no_reset(self, sm: EmgGraspStateMachine) -> None:
        # Enter CONTROL_GRASP
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        # OPEN held but proportional too low
        t += 0.1
        sm.update(_input(gesture=3, proportional=0.3, timestamp=t))
        t += 0.6
        intents = sm.update(_input(gesture=3, proportional=0.3, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP
        assert IntentType.RESET_HAND not in _intent_types(intents)

    def test_open_preempts_power(self, sm: EmgGraspStateMachine) -> None:
        # In CONTROL_GRASP, both POWER and OPEN could be active
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        # Simultaneous OPEN (higher priority)
        t += 0.1
        sm.update(_input(gesture=3, proportional=0.8, timestamp=t))
        t += 0.6
        intents = sm.update(_input(gesture=3, proportional=0.8, timestamp=t))
        assert sm.mode == Mode.NOT_GRASPING
        assert IntentType.RESET_HAND in _intent_types(intents)
        # POWER toggle should NOT have fired because OPEN pre-empted
        assert IntentType.ENTER_FORCE_HOLD not in _intent_types(intents)


# ── 3. POWER not-grasping -> control-grasp ────────────────────────────────

def test_power_hold_enters_control_grasp(sm: EmgGraspStateMachine) -> None:
    t = 0.0
    sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
    t += 0.6
    intents = sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
    assert sm.mode == Mode.CONTROL_GRASP
    assert IntentType.ENTER_FORCE_HOLD in _intent_types(intents)


def test_power_too_short_no_toggle(sm: EmgGraspStateMachine) -> None:
    t = 0.0
    sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
    t += 0.3
    intents = sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
    assert sm.mode == Mode.NOT_GRASPING
    assert IntentType.ENTER_FORCE_HOLD not in _intent_types(intents)


# ── 4. POWER control-grasp <-> control-wrist ──────────────────────────────

class TestPowerToggleGraspWrist:
    def test_power_toggles_grasp_to_wrist(self, sm: EmgGraspStateMachine) -> None:
        # Enter CONTROL_GRASP
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        # Release and re-arm
        t += 0.1
        sm.update(_input(gesture=0, timestamp=t))
        t += 0.4  # rearm duration

        # Hold POWER again
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        intents = sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_WRIST
        assert IntentType.STOP_ALL in _intent_types(intents)

    def test_power_toggles_wrist_to_grasp(self, sm: EmgGraspStateMachine) -> None:
        # Enter CONTROL_GRASP
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        # Toggle to CONTROL_WRIST
        t += 0.1
        sm.update(_input(gesture=0, timestamp=t))
        t += 0.4
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_WRIST

        # Toggle back to CONTROL_GRASP
        t += 0.1
        sm.update(_input(gesture=0, timestamp=t))
        t += 0.4
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        intents = sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP
        assert IntentType.ENTER_FORCE_HOLD in _intent_types(intents)


# ── 5. FLEXION / EXTENSION routing in each mode ───────────────────────────

class TestFlexionExtensionRouting:
    def test_flexion_in_not_grasping(self, sm: EmgGraspStateMachine) -> None:
        t = 0.0
        intents = sm.update(_input(gesture=2, proportional=0.8, timestamp=t))
        assert IntentType.WRIST_VELOCITY_POSITIVE in _intent_types(intents)
        assert intents[0].value == pytest.approx(0.8)

    def test_extension_in_not_grasping(self, sm: EmgGraspStateMachine) -> None:
        t = 0.0
        intents = sm.update(_input(gesture=4, proportional=0.6, timestamp=t))
        assert IntentType.WRIST_VELOCITY_NEGATIVE in _intent_types(intents)
        assert intents[0].value == pytest.approx(0.6)

    def test_flexion_in_control_grasp(self, sm: EmgGraspStateMachine) -> None:
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        t += 0.1
        intents = sm.update(_input(gesture=2, proportional=0.5, timestamp=t))
        assert IntentType.ADJUST_FORCE_TARGET in _intent_types(intents)
        assert intents[0].value == pytest.approx(0.5)  # +step * proportional

    def test_extension_in_control_grasp(self, sm: EmgGraspStateMachine) -> None:
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        t += 0.1
        intents = sm.update(_input(gesture=4, proportional=0.5, timestamp=t))
        assert IntentType.ADJUST_FORCE_TARGET in _intent_types(intents)
        assert intents[0].value == pytest.approx(-0.5)  # -step * proportional

    def test_flexion_in_control_wrist(self, sm: EmgGraspStateMachine) -> None:
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        # Toggle to CONTROL_WRIST
        t += 0.1
        sm.update(_input(gesture=0, timestamp=t))
        t += 0.4
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_WRIST

        t += 0.1
        intents = sm.update(_input(gesture=2, proportional=0.9, timestamp=t))
        assert IntentType.WRIST_VELOCITY_POSITIVE in _intent_types(intents)
        assert intents[0].value == pytest.approx(0.9)

    def test_extension_in_control_wrist(self, sm: EmgGraspStateMachine) -> None:
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        # Toggle to CONTROL_WRIST
        t += 0.1
        sm.update(_input(gesture=0, timestamp=t))
        t += 0.4
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_WRIST

        t += 0.1
        intents = sm.update(_input(gesture=4, proportional=0.7, timestamp=t))
        assert IntentType.WRIST_VELOCITY_NEGATIVE in _intent_types(intents)
        assert intents[0].value == pytest.approx(0.7)

    def test_rest_in_all_modes_emits_stop(self, sm: EmgGraspStateMachine) -> None:
        t = 0.0
        intents = sm.update(_input(gesture=0, timestamp=t))
        assert IntentType.STOP_ALL in _intent_types(intents)

        # CONTROL_GRASP
        t += 0.1
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        t += 0.1
        intents = sm.update(_input(gesture=0, timestamp=t))
        assert IntentType.STOP_ALL in _intent_types(intents)

        # CONTROL_WRIST
        t += 0.1
        sm.update(_input(gesture=0, timestamp=t))
        t += 0.4
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_WRIST

        t += 0.1
        intents = sm.update(_input(gesture=0, timestamp=t))
        assert IntentType.STOP_ALL in _intent_types(intents)


# ── 6. Stale EMG neutral behaviour ─────────────────────────────────────────

class TestStaleEmg:
    def test_stale_injected_by_wrapper(self, sm: EmgGraspStateMachine) -> None:
        # The wrapper injects REST when data is stale; verify SM treats REST
        # in CONTROL_GRASP as stop (and stays in mode)
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        # Simulate wrapper injecting REST because data is stale
        t += 2.0
        intents = sm.update(_input(gesture=0, proportional=0.0, timestamp=t))
        assert IntentType.STOP_ALL in _intent_types(intents)
        assert sm.mode == Mode.CONTROL_GRASP  # mode does not change on stale


# ── 7. No repeated toggles from one held POWER gesture ────────────────────

class TestPowerDebounce:
    def test_single_toggle_per_hold(self, sm: EmgGraspStateMachine) -> None:
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        intents = sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP
        assert IntentType.ENTER_FORCE_HOLD in _intent_types(intents)

        # Keep holding POWER — no more toggles
        t += 1.0
        intents = sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP
        assert IntentType.ENTER_FORCE_HOLD not in _intent_types(intents)
        assert len(intents) == 0  # nothing new

    def test_rearm_after_release(self, sm: EmgGraspStateMachine) -> None:
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        # Release to REST
        t += 0.1
        sm.update(_input(gesture=0, timestamp=t))

        # Re-assert POWER before rearm duration — should NOT toggle
        t += 0.1  # < power_rearm_duration_s (0.3)
        intents = sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        intents = sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP  # still grasp, no toggle
        assert len(intents) == 0

    def test_rearm_complete_allows_second_toggle(self, sm: EmgGraspStateMachine) -> None:
        t = 0.0
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_GRASP

        # Release and wait for rearm
        t += 0.1
        sm.update(_input(gesture=0, timestamp=t))
        t += 0.35  # > power_rearm_duration_s (0.3)

        # Re-assert POWER — should toggle to CONTROL_WRIST
        sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        t += 0.6
        intents = sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
        assert sm.mode == Mode.CONTROL_WRIST
        assert IntentType.STOP_ALL in _intent_types(intents)

    def test_power_flutter_no_spurious_toggles(self, sm: EmgGraspStateMachine) -> None:
        # Rapidly on/off POWER should not trigger because hold duration not met
        t = 0.0
        for _ in range(10):
            sm.update(_input(gesture=1, proportional=1.0, timestamp=t))
            t += 0.1
            sm.update(_input(gesture=0, timestamp=t))
            t += 0.1
        assert sm.mode == Mode.NOT_GRASPING


# ── Configuration flexibility ─────────────────────────────────────────────

def test_custom_gesture_labels() -> None:
    cfg = StateMachineConfig(
        gesture_rest=10,
        gesture_power=20,
        gesture_flexion=30,
        gesture_extension=40,
        gesture_open=50,
    )
    sm = EmgGraspStateMachine(cfg)
    t = 0.0
    sm.update(_input(gesture=20, proportional=1.0, timestamp=t))
    t += 0.6
    intents = sm.update(_input(gesture=20, proportional=1.0, timestamp=t))
    assert sm.mode == Mode.CONTROL_GRASP
    assert IntentType.ENTER_FORCE_HOLD in _intent_types(intents)

    # OPEN safety with custom label
    t += 0.1
    sm.update(_input(gesture=50, proportional=0.8, timestamp=t))
    t += 0.6
    intents = sm.update(_input(gesture=50, proportional=0.8, timestamp=t))
    assert sm.mode == Mode.NOT_GRASPING
    assert IntentType.RESET_HAND in _intent_types(intents)
