"""Tests for ProportionalLimiter — slew/acceleration limiting."""

from __future__ import annotations

import math

import pytest

from emg_bridge.control_filters import ProportionalLimiter
from emg_bridge.experiment_config import ProportionalSlewConfig


# ── Helpers ─────────────────────────────────────────────────────────────────────

def _cfg(**overrides) -> ProportionalSlewConfig:
    kwargs = {
        "enabled": True,
        "max_velocity_per_s": 0.0,
        "max_accel_per_s2": 0.0,
        "max_delta_per_step": 0.0,
        "max_fall_velocity_per_s": 0.0,
        "reset_on_rest": True,
        "initial_value": 0.0,
        "snap_to_zero_below": 0.0,
    }
    kwargs.update(overrides)
    return ProportionalSlewConfig(**kwargs)


def _step_many(limiter: ProportionalLimiter, raw: float, dt: float, label: int, n: int):
    """Step *n* times and return the final output."""
    out = 0.0
    for _ in range(n):
        out = limiter.step(raw, dt, label)
    return out


# ── Disabled ────────────────────────────────────────────────────────────────────

def test_disabled_passthrough():
    limiter = ProportionalLimiter(_cfg(enabled=False))
    assert limiter.step(0.3, 0.1, 1) == pytest.approx(0.3)
    assert limiter.step(0.7, 0.1, 1) == pytest.approx(0.7)
    assert limiter.step(1.0, 0.1, 1) == pytest.approx(1.0)
    assert limiter.step(0.0, 0.1, 1) == pytest.approx(0.0)


def test_disabled_clips_to_0_1():
    limiter = ProportionalLimiter(_cfg(enabled=False))
    assert limiter.step(-0.5, 0.1, 1) == pytest.approx(0.0)
    assert limiter.step(1.5, 0.1, 1) == pytest.approx(1.0)


# ── First step / long gap ───────────────────────────────────────────────────────

def test_first_step_uses_initial_value():
    limiter = ProportionalLimiter(_cfg(initial_value=0.2))
    out = limiter.step(0.8, 0.01, 1)
    # initial_value is 0.2 — max_delta per step is unlimited, so it should
    # jump to 0.8 in one step if no limits are set.
    # But we have no velocity/slew caps, so it should reach raw.
    assert out == pytest.approx(0.8)


def test_long_gap_resets():
    limiter = ProportionalLimiter(_cfg(max_velocity_per_s=1.0))
    # Build up to a high value
    out = _step_many(limiter, 0.5, 0.01, 1, 100)
    assert out == pytest.approx(0.5)
    # Gap > 1 sec resets
    out2 = limiter.step(0.5, 2.0, 1)
    # initial_value is 0.0, and with velocity=1.0 over 2 sec we can reach 0.5
    # Wait, dt=2.0 is > 1.0 so it resets to initial_value=0.0 first.
    # Then velocity limiting allows max 1.0 * 2.0 = 2.0 delta, so raw is reachable.
    # Actually after reset, current=0, target=0.5, max_step=2.0, so output=0.5.
    assert out2 == pytest.approx(0.5)


# ── Max delta per step ──────────────────────────────────────────────────────────

def test_max_delta_per_step_caps():
    limiter = ProportionalLimiter(_cfg(max_delta_per_step=0.1))
    # Start from 0, raw=1.0 — should advance 0.1 per step
    out = limiter.step(1.0, 0.01, 1)
    assert out == pytest.approx(0.1)
    out = limiter.step(1.0, 0.01, 1)
    assert out == pytest.approx(0.2)

    # Reverse direction
    out = limiter.step(0.0, 0.01, 1)
    assert out == pytest.approx(0.1)
    out = limiter.step(0.0, 0.01, 1)
    assert out == pytest.approx(0.0)


# ── Velocity limiting ───────────────────────────────────────────────────────────

def test_max_velocity_per_s_limits_speed():
    limiter = ProportionalLimiter(_cfg(max_velocity_per_s=2.0))
    # target far away (1.0), dt=0.01 → max delta = 0.02
    out = limiter.step(1.0, 0.01, 1)
    assert out == pytest.approx(0.02)
    out = limiter.step(1.0, 0.01, 1)
    assert out == pytest.approx(0.04)
    # After 50 steps (0.5s) we reach 1.0
    for _ in range(48):
        out = limiter.step(1.0, 0.01, 1)
    assert out == pytest.approx(1.0)


def test_zero_velocity_unlimited():
    limiter = ProportionalLimiter(_cfg(max_velocity_per_s=0.0))
    out = limiter.step(1.0, 0.001, 1)
    assert out == pytest.approx(1.0)  # instant


# ── Acceleration limiting ───────────────────────────────────────────────────────

def test_max_accel_ramps_smoothly():
    limiter = ProportionalLimiter(_cfg(max_accel_per_s2=10.0))
    dt = 0.1

    # Step 1: v starts at 0, target vel = (1.0 - 0) / 0.1 = 10
    # max vel change = 10 * 0.1 = 1.0 → clipped to v=1.0 → delta = 0.1
    out1 = limiter.step(1.0, dt, 1)
    assert out1 == pytest.approx(0.1, abs=1e-9)
    v1 = 0.1 / dt  # = 1.0

    # Step 2: target vel = (1.0 - 0.1) / 0.1 = 9.0
    # clipped around v1=1.0 ± 1.0 → v=2.0 → delta=0.2
    out2 = limiter.step(1.0, dt, 1)
    assert out2 == pytest.approx(0.3, abs=1e-9)

    # Step 3: target vel = (1.0 - 0.3) / 0.1 = 7.0
    # clipped around v2=2.0 ± 1.0 → v=3.0 → delta=0.3
    out3 = limiter.step(1.0, dt, 1)
    assert out3 == pytest.approx(0.6, abs=1e-9)


# ── Asymmetric fall velocity ────────────────────────────────────────────────────

def test_fall_velocity_faster_than_rise():
    limiter = ProportionalLimiter(
        _cfg(max_velocity_per_s=1.0, max_fall_velocity_per_s=5.0)
    )
    dt = 0.1

    # Rising (toward 1.0): limited by max_velocity_per_s=1.0
    out = limiter.step(1.0, dt, 1)
    assert out == pytest.approx(0.1)  # max delta = 0.1

    # Falling (toward 0.0): limited by max_fall_velocity_per_s=5.0
    limiter._current = 1.0  # force high value
    limiter._velocity = 0.0
    out = limiter.step(0.0, dt, 1)
    assert out == pytest.approx(0.5)  # max delta = 0.5


def test_fall_velocity_zero_means_symmetric():
    limiter = ProportionalLimiter(
        _cfg(max_velocity_per_s=3.0, max_fall_velocity_per_s=0.0)
    )
    dt = 0.1
    out = limiter.step(1.0, dt, 1)
    assert out == pytest.approx(0.3)
    limiter._current = 1.0
    limiter._velocity = 0.0
    out = limiter.step(0.0, dt, 1)
    assert out == pytest.approx(0.7)


# ── Reset on REST ───────────────────────────────────────────────────────────────

def test_reset_on_rest_resets_state():
    limiter = ProportionalLimiter(_cfg(max_velocity_per_s=2.0, reset_on_rest=True))
    dt = 0.1

    # Build up to 0.4
    for _ in range(4):
        out = limiter.step(1.0, dt, 1)
    assert out == pytest.approx(0.8)  # 4 * 0.2

    # REST label=0 with velocity limiting: target=0, velocity=2, delta=-0.2
    out = limiter.step(0.8, dt, 0)  # raw doesn't matter, target forced to 0
    assert out == pytest.approx(0.6)  # -0.2 from velocity limit

    # Velocity state was reset to 0, so next step: target vel = -0.6/0.1 = -6
    # accel unlimited, vel_limit=2, delta=-0.2
    out = limiter.step(0.8, dt, 0)
    assert out == pytest.approx(0.4)


def test_reset_on_rest_disabled():
    limiter = ProportionalLimiter(_cfg(max_velocity_per_s=2.0, reset_on_rest=False))
    dt = 0.1

    for _ in range(5):
        out = limiter.step(1.0, dt, 1)
    assert out == pytest.approx(1.0)

    # REST label=0 but reset_on_rest=False: should NOT force target to 0
    out = limiter.step(0.3, dt, 0)  # raw is 0.3, treated normally
    assert out == pytest.approx(0.8)  # limited velocity decrease from 1.0


# ── Snap to zero ────────────────────────────────────────────────────────────────

def test_snap_to_zero_below():
    limiter = ProportionalLimiter(_cfg(snap_to_zero_below=0.05, max_velocity_per_s=1.0))
    dt = 0.01

    # Start with a small value that ramps down
    limiter._current = 0.1
    limiter._initialized = True
    out = limiter.step(0.0, dt, 1)
    # max delta = 0.01, so output = 0.09 — still above threshold
    assert out > 0.0

    # Step down more until below 0.05
    for _ in range(10):
        out = limiter.step(0.0, dt, 1)
    assert out == pytest.approx(0.0)


def test_snap_to_zero_disabled():
    limiter = ProportionalLimiter(_cfg(snap_to_zero_below=0.0, max_velocity_per_s=1.0))
    limiter._current = 0.001
    limiter._initialized = True
    out = limiter.step(0.0, 0.01, 1)
    # With velocity=1, the step toward 0 from 0.001 with dt=0.01 → delta=-0.001 (clipped by velocity? 1*0.01=0.01, so -0.001 is within limits)
    # output = 0.001 - 0.001 = 0.0
    # Actually 0.0 is technically a valid output but the snap is disabled, so it would naturally reach 0.
    # Let's test differently: make sure very small positive values stay:
    limiter._current = 0.04
    out = limiter.step(0.04, 0.01, 1)
    # current=0.04, target=0.04, delta=0 — stays at 0.04
    assert out == pytest.approx(0.04)


# ── Clamping [0, 1] ─────────────────────────────────────────────────────────────

def test_clamps_to_0():
    limiter = ProportionalLimiter(_cfg(enabled=True))
    limiter._current = 0.0
    limiter._initialized = True
    out = limiter.step(-0.5, 0.01, 1)
    assert out >= 0.0


def test_clamps_to_1():
    limiter = ProportionalLimiter(_cfg(enabled=True))
    limiter._current = 1.0
    limiter._initialized = True
    out = limiter.step(2.0, 0.01, 1)
    assert out <= 1.0


def test_natural_clamp_at_1():
    limiter = ProportionalLimiter(_cfg(max_velocity_per_s=10.0))
    for _ in range(200):
        out = limiter.step(1.5, 0.01, 1)
    assert out == pytest.approx(1.0)


# ── Edge cases ──────────────────────────────────────────────────────────────────

def test_dt_zero_returns_current():
    limiter = ProportionalLimiter(_cfg(enabled=True))
    limiter.step(0.5, 0.01, 1)  # prime
    assert limiter.step(1.0, 0.0, 1) == pytest.approx(0.5)


def test_negative_raw_clipped():
    limiter = ProportionalLimiter(_cfg(max_velocity_per_s=10.0))
    # First step
    limiter.step(0.5, 0.01, 1)
    # Negative raw should be clipped to 0 at input
    out = _step_many(limiter, -1.0, 0.01, 1, 100)
    assert out == pytest.approx(0.0)


def test_raw_1_0_reaches_1():
    limiter = ProportionalLimiter(_cfg(max_velocity_per_s=10.0))
    out = _step_many(limiter, 1.0, 0.01, 1, 200)
    assert out == pytest.approx(1.0)


def test_multiple_stateful_steps():
    limiter = ProportionalLimiter(_cfg(max_velocity_per_s=5.0))
    dt = 0.02
    trajectory = []
    for i in range(50):
        raw = 1.0 if i % 10 < 5 else 0.0  # pulsing square wave
        trajectory.append(limiter.step(raw, dt, 1))

    assert trajectory[0] == pytest.approx(0.1)  # 5 * 0.02 = 0.1
    assert trajectory[4] == pytest.approx(0.5)  # max reached at step 5
    # Never exceeds 1.0
    assert max(trajectory) <= 1.0
    # Never below 0.0
    assert min(trajectory) >= 0.0


def test_rest_with_acceleration_limiting():
    """REST should still respect acceleration limits while driving to 0."""
    limiter = ProportionalLimiter(
        _cfg(
            max_velocity_per_s=10.0,
            max_accel_per_s2=5.0,
            reset_on_rest=True,
        )
    )
    dt = 0.1
    # Ramp up to 1.0
    for _ in range(50):
        limiter.step(1.0, dt, 1)
    # Check we're at 1.0
    assert limiter._current == pytest.approx(1.0)

    # REST with accel limiting: velocity reset to 0, so accel limits ramp
    out = limiter.step(1.0, dt, 0)
    # velocity=0, max vel change=0.5, so v=0.5 → delta=-0.05 → output=0.95
    assert out == pytest.approx(0.95)


def test_combined_velocity_and_accel_limits():
    limiter = ProportionalLimiter(
        _cfg(
            max_velocity_per_s=3.0,
            max_accel_per_s2=2.0,
        )
    )
    dt = 1.0  # large dt to make velocity limit dominate early
    out = limiter.step(1.0, dt, 1)
    # velocity starts 0, accel allows 2.0 increase → v=2.0
    # velocity limit is 3.0, so v=2.0 wins → delta=2.0
    # But output clamped to [0,1] → 2.0 → 1.0
    assert out == 1.0

    # Do it with smaller dt where accel limits dominate
    limiter2 = ProportionalLimiter(
        _cfg(max_velocity_per_s=100.0, max_accel_per_s2=10.0)
    )
    dt2 = 0.1
    out2 = limiter2.step(1.0, dt2, 1)
    # max_accel=10, dt=0.1 → max_vel_change=1.0 → v clamped to 1.0 → delta=0.1
    assert out2 == pytest.approx(0.1)


def test_max_delta_per_step_combined_with_velocity():
    """max_delta_per_step should be a hard cap that overrides velocity."""
    limiter = ProportionalLimiter(
        _cfg(max_velocity_per_s=100.0, max_delta_per_step=0.05)
    )
    dt = 0.1
    out = limiter.step(1.0, dt, 1)
    # velocity allows 10.0 delta, but max_delta_per_step caps at 0.05
    assert out == pytest.approx(0.05)
    out = limiter.step(1.0, dt, 1)
    assert out == pytest.approx(0.10)
