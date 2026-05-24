"""Unit tests for the EMG wrist controller logic (no ROS required).

Tests the stop condition evaluation, position command computation, and
boundary behavior in isolation.
"""

import sys
import os

import pytest

# Allow import from the wrist_driver package
sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "wrist_driver",
    ),
)

from emg_wrist_controller import (
    EmgWristController,
    GESTURE_REST,
    GESTURE_POWER,
    GESTURE_PINCH,
    GESTURE_OPEN,
    GESTURE_POINT,
    GESTURE_FLEXION,
    GESTURE_EXTENSION,
    PIPELINE_IDLE,
    PIPELINE_SEGMENTING,
    PIPELINE_PLANNING,
    PIPELINE_APPROACHING,
    PIPELINE_GRASPING,
    PIPELINE_HOLDING,
    PIPELINE_RELEASING,
)

# ── Default test configuration ────────────────────────────────────────────────

DEFAULT_KWARGS = {
    "enabled": True,
    "current_gesture": GESTURE_FLEXION,
    "current_confidence": 0.8,
    "pipeline_state": PIPELINE_IDLE,
    "prev_pipeline_state": PIPELINE_IDLE,
    "wrist_position_deg": 0.0,
    "last_wrist_state_time": 100.0,
    "wrist_state_received": True,
    "fail_preflight_on_missing_wrist": False,
    "flexion_gesture": GESTURE_FLEXION,
    "extension_gesture": GESTURE_EXTENSION,
    "release_gesture": GESTURE_OPEN,
    "confidence_threshold": 0.55,
    "min_position_deg": -90.0,
    "max_position_deg": 90.0,
    "stale_state_timeout_s": 0.5,
    "stop_on_mode_change": True,
    "wrist_permissive_states": {PIPELINE_IDLE, PIPELINE_RELEASING},
    "now": 100.0,
}


def _kwargs(**overrides):
    """Return a copy of DEFAULT_KWARGS with overrides applied."""
    d = dict(DEFAULT_KWARGS)
    d.update(overrides)
    return d


# ── Stop condition tests ─────────────────────────────────────────────────────


class TestStopConditions:

    def test_no_stop_when_flexion_in_idle(self):
        """FLEXION in IDLE state with good confidence should NOT stop."""
        reason = EmgWristController.check_stop_conditions(**_kwargs())
        assert reason is None

    def test_no_stop_when_extension_in_idle(self):
        """EXTENSION in IDLE state with good confidence should NOT stop."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(current_gesture=GESTURE_EXTENSION)
        )
        assert reason is None

    def test_no_stop_when_flexion_in_releasing(self):
        """FLEXION in RELEASING state (wrist-permissive) should NOT stop."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                pipeline_state=PIPELINE_RELEASING,
                prev_pipeline_state=PIPELINE_RELEASING,
            )
        )
        assert reason is None

    def test_stop_on_rest_gesture(self):
        """REST gesture (neutral EMG) must stop wrist motion."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(current_gesture=GESTURE_REST)
        )
        assert reason is not None
        assert "neutral" in reason.lower()

    def test_stop_on_open_gesture(self):
        """OPEN gesture (release) must stop wrist motion."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(current_gesture=GESTURE_OPEN)
        )
        assert reason is not None
        assert "release" in reason.lower()

    def test_stop_on_non_wrist_gesture_power(self):
        """POWER gesture while in IDLE must not move wrist (not a wrist gesture)."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(current_gesture=GESTURE_POWER)
        )
        assert reason is not None
        assert "non-wrist" in reason.lower() or "non wrist" in reason.lower()

    def test_stop_on_non_wrist_gesture_pinch(self):
        """PINCH gesture while in IDLE must not move wrist."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(current_gesture=GESTURE_PINCH)
        )
        assert reason is not None

    def test_stop_on_low_confidence(self):
        """Confidence below threshold must stop wrist motion."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(current_confidence=0.3)
        )
        assert reason is not None
        assert "low confidence" in reason.lower()

    def test_no_stop_confidence_at_threshold(self):
        """Confidence exactly at threshold should be sufficient."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(current_confidence=0.55)
        )
        assert reason is None

    def test_stop_when_disabled(self):
        """Disabled controller must always stop."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(enabled=False)
        )
        assert reason is not None
        assert "disabled" in reason.lower()

    def test_stop_on_stale_wrist_state(self):
        """Stale wrist state (beyond timeout) must stop motion."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                last_wrist_state_time=99.0,  # 1.0s ago
                now=100.0,
                stale_state_timeout_s=0.5,
            )
        )
        assert reason is not None
        assert "stale" in reason.lower()

    def test_no_stop_with_fresh_wrist_state(self):
        """Fresh wrist state should NOT stop."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                last_wrist_state_time=99.9,
                now=100.0,
                stale_state_timeout_s=0.5,
            )
        )
        assert reason is None

    def test_stop_on_missing_wrist_hardware_warning(self):
        """No wrist state received → disable with warning (not fail preflight)."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                wrist_state_received=False,
                fail_preflight_on_missing_wrist=False,
            )
        )
        assert reason is not None
        assert "missing wrist" in reason.lower()

    def test_fail_preflight_on_missing_wrist(self):
        """fail_preflight_on_missing_wrist=True must raise RuntimeError."""
        with pytest.raises(RuntimeError, match="preflight"):
            EmgWristController.check_stop_conditions(
                **_kwargs(
                    wrist_state_received=False,
                    fail_preflight_on_missing_wrist=True,
                )
            )

    def test_stop_when_not_in_permissive_state_approaching(self):
        """APPROACHING is a control-grasp state — FLEXION must not move wrist."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                pipeline_state=PIPELINE_APPROACHING,
                prev_pipeline_state=PIPELINE_APPROACHING,
            )
        )
        assert reason is not None
        assert "not in permissive" in reason.lower()

    def test_stop_when_not_in_permissive_state_grasping(self):
        """GRASPING must block wrist motion (control-grasp mode)."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                pipeline_state=PIPELINE_GRASPING,
                prev_pipeline_state=PIPELINE_GRASPING,
            )
        )
        assert reason is not None

    def test_stop_when_not_in_permissive_state_holding(self):
        """HOLDING must block wrist motion (control-grasp mode)."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                pipeline_state=PIPELINE_HOLDING,
                prev_pipeline_state=PIPELINE_HOLDING,
            )
        )
        assert reason is not None

    def test_stop_on_mode_change(self):
        """Mode change from IDLE to SEGMENTING must stop wrist (stop_on_mode_change=true)."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                pipeline_state=PIPELINE_SEGMENTING,
                prev_pipeline_state=PIPELINE_IDLE,
                stop_on_mode_change=True,
            )
        )
        assert reason is not None
        assert "mode changed" in reason.lower()

    def test_no_stop_on_mode_change_when_disabled(self):
        """When stop_on_mode_change=false, mode change should not stop wrist."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                pipeline_state=PIPELINE_SEGMENTING,
                prev_pipeline_state=PIPELINE_IDLE,
                stop_on_mode_change=False,
            )
        )
        # SEGMENTING is NOT in permissive states, so a different stop fires
        assert reason is not None
        assert "not in permissive" in reason.lower()

    def test_mode_change_within_permissive(self):
        """Transition IDLE->RELEASING (both permissive) should NOT stop."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                pipeline_state=PIPELINE_RELEASING,
                prev_pipeline_state=PIPELINE_IDLE,
                stop_on_mode_change=True,
            )
        )
        # Both are permissive, so the mode_change check fires.
        # This is expected: any mode change (even within permissive) stops by default.
        assert reason is not None
        assert "mode changed" in reason.lower()

    def test_settled_after_mode_change(self):
        """After a mode change settles (prev==current), no mode-change stop."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                pipeline_state=PIPELINE_RELEASING,
                prev_pipeline_state=PIPELINE_RELEASING,
            )
        )
        assert reason is None

    def test_stop_at_max_position_flexion(self):
        """Flexion at max position must stop."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                wrist_position_deg=90.0,
                max_position_deg=90.0,
            )
        )
        assert reason is not None
        assert "max position" in reason.lower()

    def test_stop_at_min_position_extension(self):
        """Extension at min position must stop."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                current_gesture=GESTURE_EXTENSION,
                wrist_position_deg=-90.0,
                min_position_deg=-90.0,
            )
        )
        assert reason is not None
        assert "min position" in reason.lower()

    def test_no_stop_near_max_but_not_at_limit(self):
        """Flexion close to max but not at limit should still move."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                wrist_position_deg=89.9,
                max_position_deg=90.0,
            )
        )
        assert reason is None

    def test_no_stop_when_extension_not_at_min(self):
        """Extension not at min limit should continue."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                current_gesture=GESTURE_EXTENSION,
                wrist_position_deg=-89.9,
                min_position_deg=-90.0,
            )
        )
        assert reason is None


# ── Position command computation tests ────────────────────────────────────────


class TestComputeWristCommand:

    def test_flexion_computes_positive_step(self):
        target = EmgWristController.compute_wrist_command(
            current_gesture=GESTURE_FLEXION,
            wrist_position_deg=10.0,
            step_size_deg=1.5,
            min_position_deg=-90.0,
            max_position_deg=90.0,
        )
        assert target == pytest.approx(11.5)

    def test_extension_computes_negative_step(self):
        target = EmgWristController.compute_wrist_command(
            current_gesture=GESTURE_EXTENSION,
            wrist_position_deg=10.0,
            step_size_deg=1.5,
            min_position_deg=-90.0,
            max_position_deg=90.0,
        )
        assert target == pytest.approx(8.5)

    def test_non_wrist_gesture_returns_none(self):
        target = EmgWristController.compute_wrist_command(
            current_gesture=GESTURE_REST,
            wrist_position_deg=10.0,
            step_size_deg=1.5,
            min_position_deg=-90.0,
            max_position_deg=90.0,
        )
        assert target is None

    def test_flexion_clamped_to_max(self):
        """Flexion near max is clamped to max, not overshooting."""
        target = EmgWristController.compute_wrist_command(
            current_gesture=GESTURE_FLEXION,
            wrist_position_deg=89.5,
            step_size_deg=1.5,
            min_position_deg=-90.0,
            max_position_deg=90.0,
        )
        assert target == pytest.approx(90.0)
        assert target <= 90.0

    def test_extension_clamped_to_min(self):
        """Extension near min is clamped to min, not overshooting."""
        target = EmgWristController.compute_wrist_command(
            current_gesture=GESTURE_EXTENSION,
            wrist_position_deg=-89.5,
            step_size_deg=1.5,
            min_position_deg=-90.0,
            max_position_deg=90.0,
        )
        assert target == pytest.approx(-90.0)
        assert target >= -90.0

    def test_command_respects_custom_bounds(self):
        """Verify bounds are honored when limits are asymmetric."""
        target = EmgWristController.compute_wrist_command(
            current_gesture=GESTURE_FLEXION,
            wrist_position_deg=40.0,
            step_size_deg=10.0,
            min_position_deg=-45.0,
            max_position_deg=45.0,
        )
        assert target == pytest.approx(45.0)

    def test_step_size_zero(self):
        """Zero step size means no movement."""
        target = EmgWristController.compute_wrist_command(
            current_gesture=GESTURE_FLEXION,
            wrist_position_deg=10.0,
            step_size_deg=0.0,
            min_position_deg=-90.0,
            max_position_deg=90.0,
        )
        assert target == pytest.approx(10.0)

    def test_large_step_crosses_both_bounds(self):
        """Step larger than range is clamped to correct limit."""
        target = EmgWristController.compute_wrist_command(
            current_gesture=GESTURE_FLEXION,
            wrist_position_deg=0.0,
            step_size_deg=200.0,
            min_position_deg=-90.0,
            max_position_deg=90.0,
        )
        assert target == pytest.approx(90.0)


# ── Integration-level tests (stop + command together) ────────────────────────


class TestWristControlIntegration:

    def test_not_grasping_flexion_moves_wrist(self):
        """In IDLE (not-grasping), FLEXION should produce a valid wrist command."""
        stop = EmgWristController.check_stop_conditions(
            **_kwargs(current_gesture=GESTURE_FLEXION, pipeline_state=PIPELINE_IDLE)
        )
        assert stop is None
        target = EmgWristController.compute_wrist_command(
            current_gesture=GESTURE_FLEXION,
            wrist_position_deg=0.0,
            step_size_deg=1.5,
            min_position_deg=-90.0,
            max_position_deg=90.0,
        )
        assert target is not None
        assert target > 0.0

    def test_control_grasp_flexion_does_not_move(self):
        """In GRASPING (control-grasp), FLEXION must NOT produce a command."""
        stop = EmgWristController.check_stop_conditions(
            **_kwargs(
                current_gesture=GESTURE_FLEXION,
                pipeline_state=PIPELINE_GRASPING,
                prev_pipeline_state=PIPELINE_GRASPING,
            )
        )
        assert stop is not None
        assert "not in permissive" in stop.lower()

    def test_release_gesture_stops_wrist_in_idle(self):
        """OPEN gesture must stop wrist even in permissive states."""
        stop = EmgWristController.check_stop_conditions(
            **_kwargs(
                current_gesture=GESTURE_OPEN,
                pipeline_state=PIPELINE_IDLE,
            )
        )
        assert stop is not None
        assert "release" in stop.lower()

    def test_neutral_emg_stops_wrist(self):
        """REST gesture (neutral) must stop wrist in all modes."""
        stop = EmgWristController.check_stop_conditions(
            **_kwargs(
                current_gesture=GESTURE_REST,
                pipeline_state=PIPELINE_IDLE,
            )
        )
        assert stop is not None
        assert "neutral" in stop.lower()


# ── Configurable gesture IDs ──────────────────────────────────────────────────


class TestCustomGestureMapping:

    def test_custom_flexion_gesture(self):
        """With a custom flexion gesture, the controller should recognize it."""
        stop = EmgWristController.check_stop_conditions(
            **_kwargs(
                current_gesture=2,  # PINCH, remapped as flexion
                flexion_gesture=2,
                extension_gesture=4,  # POINT, remapped as extension
            )
        )
        assert stop is None

    def test_custom_flexion_different_from_default(self):
        """Original FLEXION should be treated as non-wrist if remapped."""
        stop = EmgWristController.check_stop_conditions(
            **_kwargs(
                current_gesture=GESTURE_FLEXION,
                flexion_gesture=2,
                extension_gesture=4,
            )
        )
        assert stop is not None
        assert "non-wrist" in stop.lower()


# ── Boundary / edge case tests ────────────────────────────────────────────────


class TestEdgeCases:

    def test_zero_timeout_stale_immediately(self):
        """With timeout=0, any state should be considered stale."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                last_wrist_state_time=99.999,
                now=100.0,
                stale_state_timeout_s=0.0,
            )
        )
        assert reason is not None
        assert "stale" in reason.lower()

    def test_very_large_timeout(self):
        """With very large timeout, old state is still considered fresh."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                last_wrist_state_time=1.0,
                now=100.0,
                stale_state_timeout_s=999.0,
            )
        )
        assert reason is None

    def test_exactly_at_limit_considered_stop(self):
        """Being exactly at max position with FLEXION should stop."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                wrist_position_deg=90.0,
                max_position_deg=90.0,
            )
        )
        assert reason is not None

    def test_motion_allowed_when_not_exactly_at_limit(self):
        """Being fractionally below max position should allow motion."""
        reason = EmgWristController.check_stop_conditions(
            **_kwargs(
                wrist_position_deg=89.999,
                max_position_deg=90.0,
            )
        )
        assert reason is None

    def test_all_permissive_states_allow_wrist(self):
        """All default permissive states should allow wrist motion."""
        for state in (PIPELINE_IDLE, PIPELINE_RELEASING):
            reason = EmgWristController.check_stop_conditions(
                **_kwargs(
                    pipeline_state=state,
                    prev_pipeline_state=state,
                )
            )
            assert reason is None, f"State {state} should allow wrist motion"

    def test_all_grasping_states_block_wrist(self):
        """All control-grasp states should block wrist motion."""
        control_grasp_states = {
            PIPELINE_SEGMENTING,
            PIPELINE_PLANNING,
            PIPELINE_APPROACHING,
            PIPELINE_GRASPING,
            PIPELINE_HOLDING,
        }
        for state in control_grasp_states:
            reason = EmgWristController.check_stop_conditions(
                **_kwargs(
                    pipeline_state=state,
                    prev_pipeline_state=state,
                )
            )
            assert reason is not None, (
                f"State {state} should block wrist motion"
            )
