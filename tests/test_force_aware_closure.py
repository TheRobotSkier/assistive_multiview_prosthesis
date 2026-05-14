import sys
from pathlib import Path

import pytest

# Allow import from nodes/ directory without ROS
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src" / "grasp_preshaping" / "nodes"))

from force_aware_closure import (
    FingerState,
    compute_closure_speed,
    check_force_contact,
    step_finger_state,
    ClosureProfile,
    ForceReading,
)


def test_compute_closure_speed_decays():
    """Speed decays from max to min as distance_to_predicted shrinks."""
    profile = ClosureProfile(
        max_closing_speed=0.55,
        min_closing_speed=0.12,
        decay_distance=0.30,
        decay_exponent=1.2,
        contact_force_threshold=100.0,
        contact_force_spike_threshold=25.0,
        max_extra_closure=0.30,
        contact_hold_velocity=0.0,
        final_closure_timeout_s=4.0,
    )

    # Far from predicted → max speed
    assert compute_closure_speed(profile, distance_to_predicted=0.50) == pytest.approx(0.55, abs=1e-9)

    # At decay distance → ratio=1.0, speed=max
    mid = compute_closure_speed(profile, distance_to_predicted=0.30)
    assert mid == pytest.approx(0.55, abs=1e-9)

    # At zero (predicted point) → min speed
    assert compute_closure_speed(profile, distance_to_predicted=0.0) == pytest.approx(0.12, abs=1e-9)

    # Past predicted (negative distance) → min speed
    assert compute_closure_speed(profile, distance_to_predicted=-0.10) == pytest.approx(0.12, abs=1e-9)


def test_compute_closure_speed_clamps_zero_distance():
    """Distance=0 yields min_speed regardless of exponent."""
    profile = ClosureProfile(0.6, 0.1, 0.3, 2.0, 50, 10, 0.2, 0.0, 3.0)
    assert compute_closure_speed(profile, 0.0) == pytest.approx(0.1)


def test_check_force_contact_threshold():
    """Absolute threshold triggers contact."""
    profile = ClosureProfile(0.35, 0.08, 0.25, 1.5, 50.0, 15.0, 0.2, 0.0, 3.0)
    assert check_force_contact(profile, ForceReading(force=50.0, baseline=0.0)) is True
    assert check_force_contact(profile, ForceReading(force=49.0, baseline=35.0)) is False


def test_check_force_contact_spike():
    """Spike above baseline triggers contact even when below absolute threshold."""
    profile = ClosureProfile(0.35, 0.08, 0.25, 1.5, 100.0, 15.0, 0.2, 0.0, 3.0)
    assert check_force_contact(profile, ForceReading(force=30.0, baseline=10.0)) is True
    assert check_force_contact(profile, ForceReading(force=30.0, baseline=20.0)) is False


def test_step_finger_state_open_loop_to_contact_seek():
    """Finger transitions from open_loop to contact_seek when past predicted closure."""
    profile = ClosureProfile(0.35, 0.08, 0.25, 1.5, 50.0, 15.0, 0.2, 0.0, 3.0)
    state = FingerState.OPEN_LOOP
    velocity, new_state = step_finger_state(
        profile, state,
        predicted_closure=0.5,
        current_position=0.55,  # past predicted
        current_force=ForceReading(force=0.0, baseline=0.0),
        elapsed_s=0.5,
    )
    assert new_state == FingerState.CONTACT_SEEK
    assert velocity == pytest.approx(0.08)  # min speed past predicted


def test_step_finger_state_contact_seek_to_contacted():
    """Finger transitions from contact_seek to contacted when force exceeds threshold."""
    profile = ClosureProfile(0.35, 0.08, 0.25, 1.5, 50.0, 15.0, 0.2, 0.0, 3.0)
    velocity, new_state = step_finger_state(
        profile, FingerState.CONTACT_SEEK,
        predicted_closure=0.5,
        current_position=0.60,
        current_force=ForceReading(force=55.0, baseline=0.0),
        elapsed_s=1.0,
    )
    assert new_state == FingerState.CONTACTED
    assert velocity == 0.0


def test_step_finger_state_timeout_stops():
    """Timeout past final_closure_timeout_s returns SAFETY_STOPPED with zero velocity."""
    profile = ClosureProfile(0.35, 0.08, 0.25, 1.5, 50.0, 15.0, 0.2, 0.0, 3.0)
    velocity, new_state = step_finger_state(
        profile, FingerState.CONTACT_SEEK,
        predicted_closure=0.5,
        current_position=0.55,
        current_force=ForceReading(force=0.0, baseline=0.0),
        elapsed_s=4.0,
    )
    assert new_state == FingerState.SAFETY_STOPPED
    assert velocity == 0.0


def test_step_finger_state_max_extra_closure_stops():
    """Exceeding predicted + max_extra_closure returns SAFETY_STOPPED."""
    profile = ClosureProfile(0.35, 0.08, 0.25, 1.5, 50.0, 15.0, 0.2, 0.0, 3.0)
    velocity, new_state = step_finger_state(
        profile, FingerState.CONTACT_SEEK,
        predicted_closure=0.5,
        current_position=0.71,  # > 0.5 + 0.2
        current_force=ForceReading(force=0.0, baseline=0.0),
        elapsed_s=0.1,
    )
    assert new_state == FingerState.SAFETY_STOPPED
    assert velocity == 0.0


def test_released_resets_to_open_loop():
    """RELEASED state step returns to OPEN_LOOP with max speed."""
    profile = ClosureProfile(0.35, 0.08, 0.25, 1.5, 50.0, 15.0, 0.2, 0.0, 3.0)
    velocity, new_state = step_finger_state(
        profile, FingerState.RELEASED,
        predicted_closure=0.5,
        current_position=0.3,
        current_force=ForceReading(force=0.0, baseline=0.0),
        elapsed_s=0.0,
    )
    assert new_state == FingerState.OPEN_LOOP
    # Far from predicted at distance 0.2 on 0.25 decay → near max speed
    assert velocity > 0.08


def test_safety_stopped_persists():
    """SAFETY_STOPPED stays terminal with zero velocity across ticks."""
    profile = ClosureProfile(0.35, 0.08, 0.25, 1.5, 50.0, 15.0, 0.2, 0.05, 3.0)
    velocity, new_state = step_finger_state(
        profile, FingerState.SAFETY_STOPPED,
        predicted_closure=0.5,
        current_position=0.55,
        current_force=ForceReading(force=0.0, baseline=0.0),
        elapsed_s=0.0,
    )
    assert new_state == FingerState.SAFETY_STOPPED
    assert velocity == 0.0


def test_compute_baseline_from_history():
    """Rolling average baseline from recent force history."""
    from force_aware_closure import compute_baseline
    history = [10.0, 12.0, 11.0, 13.0, 10.0]
    assert compute_baseline(history) == pytest.approx(11.2, abs=0.01)
    assert compute_baseline([]) == 0.0
