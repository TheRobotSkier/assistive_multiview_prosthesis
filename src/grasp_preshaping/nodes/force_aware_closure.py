"""Pure-Python force-aware closure logic. No ROS imports so it can be unit-tested."""

from dataclasses import dataclass
from enum import Enum, auto
from typing import List


class FingerState(Enum):
    OPEN_LOOP = auto()
    CONTACT_SEEK = auto()
    CONTACTED = auto()
    RELEASED = auto()
    SAFETY_STOPPED = auto()


@dataclass(frozen=True)
class ClosureProfile:
    max_closing_speed: float
    min_closing_speed: float
    decay_distance: float
    decay_exponent: float
    contact_force_threshold: float
    contact_force_spike_threshold: float
    max_extra_closure: float
    contact_hold_velocity: float
    final_closure_timeout_s: float


@dataclass(frozen=True)
class ForceReading:
    force: float
    baseline: float


def compute_closure_speed(profile: ClosureProfile, distance_to_predicted: float) -> float:
    """Decaying speed from max to min as distance_to_predicted shrinks.

    speed = min_speed + (max_speed - min_speed) * clamp(distance / decay_distance, 0, 1) ^ decay_exponent
    """
    if distance_to_predicted <= 0.0:
        return profile.min_closing_speed

    ratio = distance_to_predicted / profile.decay_distance
    clamped = max(0.0, min(1.0, ratio))
    decayed = clamped ** profile.decay_exponent
    return profile.min_closing_speed + (profile.max_closing_speed - profile.min_closing_speed) * decayed


def check_force_contact(profile: ClosureProfile, reading: ForceReading) -> bool:
    """Returns True if force indicates contact via absolute threshold or spike."""
    if reading.force >= profile.contact_force_threshold:
        return True
    spike = reading.force - reading.baseline
    if spike >= profile.contact_force_spike_threshold and spike >= 0:
        return True
    return False


def compute_baseline(force_history: List[float]) -> float:
    """Rolling average of recent force readings."""
    if not force_history:
        return 0.0
    return sum(force_history) / len(force_history)


def step_finger_state(
    profile: ClosureProfile,
    state: FingerState,
    predicted_closure: float,
    current_position: float,
    current_force: ForceReading,
    elapsed_s: float,
) -> tuple:
    """Advance one finger's closing state machine.

    Returns:
        (velocity, new_state)
    """
    if state == FingerState.RELEASED:
        dist = predicted_closure - current_position
        speed = compute_closure_speed(profile, dist) if dist > 0 else profile.min_closing_speed
        return (speed, FingerState.OPEN_LOOP)

    if state == FingerState.CONTACTED:
        return (profile.contact_hold_velocity, FingerState.CONTACTED)

    if state == FingerState.SAFETY_STOPPED:
        return (0.0, FingerState.SAFETY_STOPPED)

    # Check safety stops — return terminal SAFETY_STOPPED so next tick persists
    if elapsed_s >= profile.final_closure_timeout_s:
        return (0.0, FingerState.SAFETY_STOPPED)

    extra = current_position - predicted_closure
    if extra >= profile.max_extra_closure:
        return (0.0, FingerState.SAFETY_STOPPED)

    # State transitions
    if state == FingerState.OPEN_LOOP:
        dist = predicted_closure - current_position
        if dist <= 0.0:
            speed = profile.min_closing_speed
            if check_force_contact(profile, current_force):
                return (0.0, FingerState.CONTACTED)
            return (speed, FingerState.CONTACT_SEEK)
        else:
            speed = compute_closure_speed(profile, dist)
            if check_force_contact(profile, current_force):
                return (0.0, FingerState.CONTACTED)
            return (speed, FingerState.OPEN_LOOP)

    if state == FingerState.CONTACT_SEEK:
        if check_force_contact(profile, current_force):
            return (0.0, FingerState.CONTACTED)
        return (profile.min_closing_speed, FingerState.CONTACT_SEEK)

    return (0.0, FingerState.CONTACTED)
