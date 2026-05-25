#!/usr/bin/env python3
"""Force-hold control logic for EMG-driven velocity-based hand controller.

Provides pure-logic (non-ROS) helpers for:
  - Velocity ramp computation
  - Force threshold crossing detection  (entry into force-hold)
  - Target force computation from EMG proportional signal
  - Velocity-adjustment computation to maintain target force
  - Stop-condition checking (force + position limits)
  - Stale-data safety checks
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional
import math
import time

# ── Constants ──────────────────────────────────────────────────────────────────

FINGER_JOINTS: List[str] = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_COUNT: int = 3


# ── Configuration ──────────────────────────────────────────────────────────────

@dataclass
class ForceHoldConfig:
    """Configuration parameters for force-hold behaviour.

    All per-finger lists must have length FINGER_COUNT (3).
    """

    # ── Velocity ramp ──────────────────────────────────────────────────────
    closing_velocity_start: float = 0.3
    closing_velocity_end: float = 0.1
    decay_steps: int = 5
    step_interval_s: float = 0.2

    # ── Stop conditions ────────────────────────────────────────────────────
    stop_positions: List[float] = field(default_factory=lambda: [1.5, 1.5, 1.5])
    force_thresholds: List[float] = field(default_factory=lambda: [300.0, 300.0, 300.0])

    # ── Force-hold parameters ──────────────────────────────────────────────
    target_force_min: List[float] = field(default_factory=lambda: [50.0, 50.0, 50.0])
    target_force_max: List[float] = field(default_factory=lambda: [500.0, 500.0, 500.0])
    force_deadzone: List[float] = field(default_factory=lambda: [15.0, 15.0, 15.0])
    max_overshoot: List[float] = field(default_factory=lambda: [100.0, 100.0, 100.0])
    max_adjustment_velocity: float = 0.05  # rad/s — max velocity change during hold
    adjustment_gain: float = 0.0005        # rad/s per raw force unit — P-gain for hold
    static_hold_velocity: float = 0.02     # rad/s — tiny closing velocity to maintain
                                           # contact when error is within deadzone

    # ── Safety ─────────────────────────────────────────────────────────────
    stale_data_timeout_s: float = 0.5
    max_force_emergency: List[float] = field(default_factory=lambda: [800.0, 800.0, 800.0])
    emergency_backoff_velocity: float = -0.1  # rad/s — open direction during emergency

    def __post_init__(self):
        """Validate list lengths."""
        for attr in (
            "stop_positions",
            "force_thresholds",
            "target_force_min",
            "target_force_max",
            "force_deadzone",
            "max_overshoot",
            "max_force_emergency",
        ):
            val = getattr(self, attr)
            if len(val) != FINGER_COUNT:
                raise ValueError(
                    f"{attr} must have length {FINGER_COUNT}, got {len(val)}"
                )


# ── State ──────────────────────────────────────────────────────────────────────

@dataclass
class ForceHoldState:
    """Per-cycle force-hold state for all fingers."""

    # Current target forces (raw ADC), one per finger
    target_forces: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # Most recent reported efforts (raw ADC from /joint_states.effort)
    current_efforts: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    # Most recent positions (rad)
    current_positions: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    # Whether force-hold is currently active
    in_hold: bool = False

    # Last valid effort timestamp (monotonic seconds)
    last_effort_time: float = 0.0
    # Whether we have ever received effort data
    has_effort_data: bool = False


# ── Velocity ramp ──────────────────────────────────────────────────────────────

def compute_velocity_ramp(v_start: float, v_end: float, decay_steps: int) -> List[float]:
    """Return linearly-decaying velocity ramp.

    Args:
        v_start: initial velocity  (rad/s).
        v_end:   final velocity    (rad/s).
        decay_steps: number of discrete steps.

    Returns:
        List of velocities, one per step.
    """
    if decay_steps <= 1:
        return [v_start]
    ramp: List[float] = []
    for i in range(decay_steps):
        t = i / (decay_steps - 1)
        ramp.append(v_start + t * (v_end - v_start))
    return ramp


# ── Stop / threshold checks ────────────────────────────────────────────────────

def should_enter_force_hold(  # noqa: D103
    efforts: List[float],
    thresholds: List[float],
) -> bool:
    """Return True when *all* fingers have crossed their force threshold."""
    return all(e >= t for e, t in zip(efforts, thresholds))


def check_stop_conditions(
    positions: List[float],
    efforts: List[float],
    stop_positions: List[float],
    force_thresholds: List[float],
) -> Optional[str]:
    """Return reason string if any finger is past a stop condition, else None.

    Checks force thresholds first (higher priority), then position limits.
    """
    for i, name in enumerate(FINGER_JOINTS):
        if efforts[i] >= force_thresholds[i]:
            return (
                f"FORCE CONTACT on {name} "
                f"({efforts[i]:.0f} >= {force_thresholds[i]:.0f})"
            )
    for i, name in enumerate(FINGER_JOINTS):
        if positions[i] >= stop_positions[i]:
            return (
                f"STOP POSITION reached on {name} "
                f"({positions[i]:.3f} >= {stop_positions[i]:.2f})"
            )
    return None


# ── Target force ───────────────────────────────────────────────────────────────

def compute_target_force(
    emg_proportional: float,
    cfg_target_min: float,
    cfg_target_max: float,
) -> float:
    """Map EMG proportional signal [0, 1] to a target force in [min, max].

    The EMG proportional value (from compute_proportional in emg_bridge)
    scales the target force linearly between the configured min and max.

    Args:
        emg_proportional: scalar in [0.0, 1.0].
        cfg_target_min:   minimum target force (raw ADC).
        cfg_target_max:   maximum target force (raw ADC).

    Returns:
        Clamped target force.
    """
    clamped = max(0.0, min(1.0, emg_proportional))
    return cfg_target_min + clamped * (cfg_target_max - cfg_target_min)


def compute_target_forces_all(
    emg_proportional: float,
    config: ForceHoldConfig,
) -> List[float]:
    """Compute target forces for all three fingers from a scalar EMG signal."""
    return [
        compute_target_force(emg_proportional, config.target_force_min[i], config.target_force_max[i])
        for i in range(FINGER_COUNT)
    ]


# ── Force-hold velocity adjustment ─────────────────────────────────────────────

def compute_hold_velocity(
    state: ForceHoldState,
    config: ForceHoldConfig,
    dt: float,
) -> float:
    """Compute the velocity adjustment needed to maintain the target force.

    Implements a proportional dead-zone controller:
      error[i] = target[i] - effort[i]
      If |error[i]| < deadzone[i]  →  tiny static hold velocity
      Else  →  adjustment = gain * sum(error[i]) / FINGER_COUNT, clamped

    The velocity is symmetric across all fingers (group velocity controller),
    so the mean error is used.

    Args:
        state:  current force-hold state (targets + efforts).
        config: force-hold configuration.
        dt:     control-loop timestep (unused currently; reserved for rate-limiting).

    Returns:
        Velocity command (rad/s), positive = closing.
    """
    total_error = 0.0
    in_deadzone_count = 0

    for i in range(FINGER_COUNT):
        error = state.target_forces[i] - state.current_efforts[i]
        if abs(error) <= config.force_deadzone[i]:
            in_deadzone_count += 1
        else:
            # Clamp error to max_overshoot to prevent wild swings
            error = max(-config.max_overshoot[i], min(config.max_overshoot[i], error))
            total_error += error

    if in_deadzone_count == FINGER_COUNT:
        # All fingers within deadzone — apply minimal static hold
        return config.static_hold_velocity

    # Average error across fingers that are outside deadzone
    active_count = FINGER_COUNT - in_deadzone_count
    mean_error = total_error / active_count

    # Proportional gain
    adjustment = config.adjustment_gain * mean_error

    # Clamp to max adjustment velocity
    return max(-config.max_adjustment_velocity,
               min(config.max_adjustment_velocity, adjustment))


def check_emergency_force(
    efforts: List[float],
    config: ForceHoldConfig,
) -> bool:
    """Return True if any finger exceeds its emergency force limit."""
    return any(e >= f for e, f in zip(efforts, config.max_force_emergency))


def is_force_data_stale(
    state: ForceHoldState,
    config: ForceHoldConfig,
) -> bool:
    """Return True if the most recent effort data is too old."""
    if not state.has_effort_data:
        return True
    return (time.monotonic() - state.last_effort_time) > config.stale_data_timeout_s
