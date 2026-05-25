"""Unit tests for EMG force controller logic (no ROS required).

Tests the force-hold controller helpers and state machine logic in isolation.
"""

from __future__ import annotations

import math
import time

import pytest

# Import the pure-logic helpers  (no ROS dependencies)
from mia_hand_ros2_control.force_hold_controller import (
    FINGER_COUNT,
    FINGER_JOINTS,
    ForceHoldConfig,
    ForceHoldState,
    check_emergency_force,
    check_stop_conditions,
    compute_hold_velocity,
    compute_target_force,
    compute_target_forces_all,
    compute_velocity_ramp,
    is_force_data_stale,
    should_enter_force_hold,
)


# ── Velocity ramp ──────────────────────────────────────────────────────────────

class TestVelocityRamp:
    """Tests for compute_velocity_ramp."""

    def test_linear_ramp(self):
        ramp = compute_velocity_ramp(0.3, 0.1, 5)
        assert len(ramp) == 5
        assert ramp[0] == pytest.approx(0.3)
        assert ramp[-1] == pytest.approx(0.1)
        # Midpoint should be ~0.2
        assert ramp[2] == pytest.approx(0.2)

    def test_single_step(self):
        ramp = compute_velocity_ramp(0.5, 0.1, 1)
        assert ramp == [0.5]

    def test_two_steps(self):
        ramp = compute_velocity_ramp(0.0, 1.0, 2)
        assert ramp == [0.0, 1.0]

    def test_zero_steps(self):
        ramp = compute_velocity_ramp(0.5, 0.1, 0)
        assert ramp == [0.5]

    def test_constant_ramp(self):
        ramp = compute_velocity_ramp(0.3, 0.3, 4)
        assert all(v == pytest.approx(0.3) for v in ramp)


# ── Stop conditions ────────────────────────────────────────────────────────────

class TestStopConditions:
    """Tests for check_stop_conditions and should_enter_force_hold."""

    @pytest.fixture
    def thresholds(self):
        return [300.0, 300.0, 300.0]

    @pytest.fixture
    def stops(self):
        return [1.5, 1.5, 1.5]

    def test_no_stop(self, thresholds, stops):
        reason = check_stop_conditions(
            [0.5, 0.5, 0.5], [200, 200, 200], stops, thresholds,
        )
        assert reason is None

    def test_force_contact_thumb(self, thresholds, stops):
        reason = check_stop_conditions(
            [0.5, 0.5, 0.5], [350, 200, 200], stops, thresholds,
        )
        assert reason is not None
        assert "FORCE CONTACT" in reason
        assert "j_thumb_fle" in reason

    def test_force_contact_index(self, thresholds, stops):
        reason = check_stop_conditions(
            [0.5, 0.5, 0.5], [200, 350, 200], stops, thresholds,
        )
        assert "FORCE CONTACT" in reason
        assert "j_index_fle" in reason

    def test_force_contact_mrl(self, thresholds, stops):
        reason = check_stop_conditions(
            [0.5, 0.5, 0.5], [200, 200, 350], stops, thresholds,
        )
        assert "FORCE CONTACT" in reason
        assert "j_mrl_fle" in reason

    def test_position_stop(self, thresholds, stops):
        reason = check_stop_conditions(
            [0.5, 1.6, 0.5], [200, 200, 200], stops, thresholds,
        )
        assert reason is not None
        assert "STOP POSITION" in reason
        assert "j_index_fle" in reason

    def test_force_before_position(self, thresholds, stops):
        """Force threshold should be reported even if position also exceeded."""
        reason = check_stop_conditions(
            [0.5, 1.6, 0.5], [200, 350, 200], stops, thresholds,
        )
        assert "FORCE CONTACT" in reason  # force takes priority

    def test_should_enter_hold_true(self, thresholds):
        assert should_enter_force_hold([350, 350, 350], thresholds) is True

    def test_should_enter_hold_false(self, thresholds):
        assert should_enter_force_hold([350, 350, 200], thresholds) is False

    def test_should_enter_hold_exact(self, thresholds):
        assert should_enter_force_hold([300, 300, 300], thresholds) is True


# ── Target force ───────────────────────────────────────────────────────────────

class TestTargetForce:
    """Tests for compute_target_force."""

    def test_zero_proportional(self):
        assert compute_target_force(0.0, 50.0, 500.0) == pytest.approx(50.0)

    def test_full_proportional(self):
        assert compute_target_force(1.0, 50.0, 500.0) == pytest.approx(500.0)

    def test_half_proportional(self):
        assert compute_target_force(0.5, 100.0, 200.0) == pytest.approx(150.0)

    def test_below_range(self):
        result = compute_target_force(-0.5, 50.0, 500.0)
        assert result == pytest.approx(50.0)

    def test_above_range(self):
        result = compute_target_force(1.5, 50.0, 500.0)
        assert result == pytest.approx(500.0)

    def test_all_fingers(self):
        config = ForceHoldConfig(
            target_force_min=[50.0, 60.0, 70.0],
            target_force_max=[500.0, 400.0, 300.0],
        )
        targets = compute_target_forces_all(0.5, config)
        assert targets[0] == pytest.approx(275.0)   # (50+500)/2
        assert targets[1] == pytest.approx(230.0)   # (60+400)/2
        assert targets[2] == pytest.approx(185.0)   # (70+300)/2


# ── Force-hold velocity ────────────────────────────────────────────────────────

class TestForceHoldVelocity:
    """Tests for compute_hold_velocity."""

    @pytest.fixture
    def config(self):
        return ForceHoldConfig(
            force_deadzone=[15.0, 15.0, 15.0],
            max_overshoot=[100.0, 100.0, 100.0],
            max_adjustment_velocity=0.05,
            adjustment_gain=0.0005,
            static_hold_velocity=0.02,
        )

    @pytest.fixture
    def state(self):
        return ForceHoldState(
            target_forces=[200.0, 200.0, 200.0],
            current_efforts=[200.0, 200.0, 200.0],
            in_hold=True,
            has_effort_data=True,
            last_effort_time=time.monotonic(),
        )

    def test_perfect_match_gives_static(self, config, state):
        """When efforts match targets, static hold velocity is used."""
        vel = compute_hold_velocity(state, config, 0.1)
        assert vel == pytest.approx(config.static_hold_velocity)

    def test_within_deadzone(self, config, state):
        """Small errors within deadzone → static hold."""
        state.current_efforts = [210.0, 195.0, 205.0]
        vel = compute_hold_velocity(state, config, 0.1)
        assert vel == pytest.approx(config.static_hold_velocity)

    def test_need_more_force(self, config, state):
        """Effort below target → positive velocity (close more)."""
        state.current_efforts = [100.0, 100.0, 100.0]
        vel = compute_hold_velocity(state, config, 0.1)
        assert vel > 0.0

    def test_need_less_force(self, config, state):
        """Effort above target → negative velocity (open slightly)."""
        state.target_forces = [200.0, 200.0, 200.0]
        state.current_efforts = [300.0, 300.0, 300.0]
        vel = compute_hold_velocity(state, config, 0.1)
        assert vel < 0.0

    def test_clamped_to_max(self, config, state):
        """Very large error → clamped to max_adjustment_velocity."""
        state.target_forces = [1000.0, 1000.0, 1000.0]
        state.current_efforts = [0.0, 0.0, 0.0]
        vel = compute_hold_velocity(state, config, 0.1)
        assert abs(vel) <= config.max_adjustment_velocity

    def test_mixed_deadzone(self, config, state):
        """Two fingers in deadzone, one significantly off."""
        state.target_forces = [300.0, 300.0, 300.0]
        state.current_efforts = [300.0, 300.0, 100.0]  # mrl only is way off
        vel = compute_hold_velocity(state, config, 0.1)
        # Should be positive (close more) because mrl needs more force
        assert vel > 0.0
        assert vel <= config.max_adjustment_velocity

    def test_overshoot_clamped(self, config, state):
        """Error clamped to max_overshoot before gain is applied."""
        config.max_overshoot = [10.0, 10.0, 10.0]
        config.adjustment_gain = 0.01
        config.force_deadzone = [0.0, 0.0, 0.0]
        state.target_forces = [1000.0, 1000.0, 1000.0]
        state.current_efforts = [0.0, 0.0, 0.0]
        vel = compute_hold_velocity(state, config, 0.1)
        # Error per finger would be 1000, but clamped to 10
        # gain * mean(error) = 0.01 * 10 = 0.1, but max_adjustment_velocity is 0.05
        assert abs(vel) <= config.max_adjustment_velocity


# ── Emergency check ────────────────────────────────────────────────────────────

class TestEmergency:
    """Tests for check_emergency_force."""

    def test_no_emergency(self):
        config = ForceHoldConfig(max_force_emergency=[800.0, 800.0, 800.0])
        assert check_emergency_force([200, 300, 400], config) is False

    def test_emergency_thumb(self):
        config = ForceHoldConfig(max_force_emergency=[800.0, 800.0, 800.0])
        assert check_emergency_force([900, 200, 200], config) is True

    def test_emergency_exact(self):
        config = ForceHoldConfig(max_force_emergency=[800.0, 800.0, 800.0])
        assert check_emergency_force([800, 200, 200], config) is True


# ── Stale data ─────────────────────────────────────────────────────────────────

class TestStaleData:
    """Tests for is_force_data_stale."""

    def test_fresh_data(self):
        config = ForceHoldConfig(stale_data_timeout_s=0.5)
        state = ForceHoldState(
            has_effort_data=True,
            last_effort_time=time.monotonic(),
        )
        assert is_force_data_stale(state, config) is False

    def test_stale_data(self):
        config = ForceHoldConfig(stale_data_timeout_s=0.1)
        state = ForceHoldState(
            has_effort_data=True,
            last_effort_time=time.monotonic() - 1.0,
        )
        assert is_force_data_stale(state, config) is True

    def test_no_data_yet(self):
        config = ForceHoldConfig(stale_data_timeout_s=1.0)
        state = ForceHoldState(has_effort_data=False)
        assert is_force_data_stale(state, config) is True


# ── Config validation ──────────────────────────────────────────────────────────

class TestConfigValidation:
    """Tests for ForceHoldConfig validation."""

    def test_valid_config(self):
        cfg = ForceHoldConfig()  # defaults should work
        assert len(cfg.stop_positions) == FINGER_COUNT
        assert len(cfg.force_thresholds) == FINGER_COUNT

    def test_invalid_list_length(self):
        with pytest.raises(ValueError):
            ForceHoldConfig(stop_positions=[1.0, 2.0])  # too short

    def test_invalid_emergency_length(self):
        with pytest.raises(ValueError):
            ForceHoldConfig(max_force_emergency=[1.0])


# ── ForceHoldState defaults ────────────────────────────────────────────────────

class TestStateDefaults:
    """Tests for ForceHoldState initial values."""

    def test_defaults(self):
        state = ForceHoldState()
        assert state.in_hold is False
        assert state.has_effort_data is False
        assert state.last_effort_time == 0.0
        assert len(state.target_forces) == FINGER_COUNT
        assert len(state.current_efforts) == FINGER_COUNT
        assert all(t == 0.0 for t in state.target_forces)
