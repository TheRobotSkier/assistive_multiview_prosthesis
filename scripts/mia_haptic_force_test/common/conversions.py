"""Pure helper functions copied/derived from the legacy monolithic script
``scripts/mia_haptic_force_test.py``.

All functions are stateless and ROS-agnostic, making them unit-testable.
Function names drop the leading underscore from the legacy names
(e.g. ``_clamp`` → ``clamp``).
"""

from __future__ import annotations

import math
from typing import Any

from .constants import FINGER_JOINTS


# ── Config helpers ──────────────────────────────────────────────────────────


def dict_get(raw: dict[str, Any], path: str, default: Any) -> Any:
    """Traverse a nested dict with a dotted ``path`` (e.g. ``"a.b.c"``).

    Returns ``default`` when any segment is missing or when an intermediate
    value is not a dict.
    """
    cur: Any = raw
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def finger_values(raw: dict[str, Any], path: str, default: float) -> list[float]:
    """Extract a per-joint dict section into a ``[thumb, index, mrl]`` list.

    Looks up the dotted ``path`` in *raw*, then returns one value per finger
    joint (in canonical order).  Missing joint keys fall back to *default*.
    """
    section = dict_get(raw, path, {})
    if not isinstance(section, dict):
        section = {}
    return [float(section.get(name, default)) for name in FINGER_JOINTS]


def as_bool(value: Any) -> bool:
    """Liberal bool coercion.

    Recognises ``"1"``, ``"true"``, ``"yes"``, ``"on"`` (case-insensitive)
    as ``True``.
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


# ── Math helpers ────────────────────────────────────────────────────────────


def clamp(value: float, lo: float, hi: float) -> float:
    """Constrain *value* to the closed interval ``[lo, hi]``."""
    return max(lo, min(hi, value))


def circ_delta_deg(target: float, current: float) -> float:
    """Shortest signed angular distance ``target - current`` in degrees.

    Result is in ``[-180, 180)``.
    """
    return (target - current + 180.0) % 360.0 - 180.0


def circ_dist_deg(a: float, b: float) -> float:
    """Shortest absolute angular distance between *a* and *b* in degrees."""
    return abs(circ_delta_deg(a, b))


# ── Velocity helpers ────────────────────────────────────────────────────────


def velocity_ramp(start: float, end: float, steps: int) -> list[float]:
    """Linearly interpolate from *start* to *end* over *steps* values.

    When ``steps <= 1`` returns ``[start]``.
    """
    if steps <= 1:
        return [start]
    return [start + (i / (steps - 1)) * (end - start) for i in range(steps)]


def hold_velocity(
    force: float,
    target: float,
    deadzone: float,
    max_velocity: float,
    min_overshoot: float,
    max_overshoot: float,
) -> float:
    """Compute the signed hold velocity based on force error.

    * Returns ``0.0`` when ``abs(error) <= deadzone``.
    * Scales linearly between *min_overshoot* and *max_overshoot*.
    * Saturates at *max_velocity*.

    Sign matches the direction of the force error (positive → close further).
    """
    error = target - force
    abs_error = abs(error)
    if abs_error <= deadzone:
        return 0.0
    if max_overshoot <= 0.0:
        return max_velocity if error > 0.0 else -max_velocity
    scaled = min(max(abs_error, min_overshoot), max_overshoot)
    velocity = max_velocity * (scaled / max_overshoot)
    return velocity if error > 0.0 else -velocity


# ── Competitors helper ──────────────────────────────────────────────────────


def competitors(active: list[str]) -> list[str]:
    """Return controllers that are **not** in *active*.

    Controllers sharing a command topic cannot run simultaneously; this
    helper identifies which ones must be deactivated before activating a new
    controller.
    """
    from .constants import POSITION_CONTROLLERS, VELOCITY_CONTROLLERS

    active_set = set(active)
    return [c for c in POSITION_CONTROLLERS + VELOCITY_CONTROLLERS if c not in active_set]


# ── Haptic helpers ──────────────────────────────────────────────────────────


def wrist_haptics(angle_deg: float, cfg: dict[str, Any]) -> list[float]:
    """Map wrist angle to motor intensities via nearest-neighbour blending.

    *cfg* is the ``haptics:`` section of the config dict; it must contain
    the keys listed below (names match the YAML keys in
    ``config/mia_haptic_force_test.yaml``).

    Required *cfg* keys
    -------------------
    ``motor_count``
        Number of haptic motors (int).
    ``wrist_motor_angles_deg``
        Angular position of each motor around the band (list[float]).
    ``wrist_thumb_offset_deg``
        Rotation offset for the thumb datum (float).
    ``wrist_intensity_pct``
        Maximum intensity percentage to distribute (float).
    ``wrist_min_motor_pct``
        Minimum non-zero intensity; values below are zeroed (float).
    """
    motor_count = int(cfg["motor_count"])
    motor_angles = list(cfg.get("wrist_motor_angles_deg", []))
    if len(motor_angles) != motor_count:
        motor_angles = [i * (360.0 / motor_count) for i in range(motor_count)]

    thumb_angle = (angle_deg + float(cfg["wrist_thumb_offset_deg"])) % 360.0
    dists = [circ_dist_deg(thumb_angle, ma) for ma in motor_angles]
    nearest = sorted(range(motor_count), key=lambda i: dists[i])[:2]

    out = [0.0] * motor_count
    if not nearest:
        return out
    if len(nearest) == 1 or dists[nearest[0]] < 1e-6:
        out[nearest[0]] = float(cfg["wrist_intensity_pct"])
    else:
        i0, i1 = nearest
        d0, d1 = dists[i0], dists[i1]
        span = max(d0 + d1, 1e-6)
        total = float(cfg["wrist_intensity_pct"])
        out[i0] = total * (d1 / span)
        out[i1] = total * (d0 / span)

    min_pct = float(cfg["wrist_min_motor_pct"])
    return [v if v >= min_pct else 0.0 for v in out]


def force_haptics(percent: float, cfg: dict[str, Any]) -> tuple[list[float], str]:
    """Map grasp-force percentage to haptic motor intensities.

    *cfg* is the ``haptics:`` section of the config dict.

    Required *cfg* keys
    -------------------
    ``motor_count``
        Number of haptic motors (int).
    ``force_phase_change_threshold_pct``
        Percentage at which intensity switches from motor-spreading to
        uniform scaling (float).
    ``force_motor_step_pct``
        Percentage increment per additional motor (float).
    ``force_low_intensity_pct``
        Base intensity for low-force phase (float).
    ``force_high_intensity_pct``
        Maximum intensity for high-force phase (float).
    ``force_motor_order``
        Order in which motors activate (list[int]).

    Returns
    -------
    ``(motor_intensities, phase_label)``
        *motor_intensities* is a list of length ``motor_count`` (0-100).
        *phase_label* is one of ``"zero"``, ``"adding_motors"``, or
        ``"all_motors_intensity"``.
    """
    motor_count = int(cfg["motor_count"])
    out = [0.0] * motor_count
    pct = clamp(percent, 0.0, 100.0)

    phase_threshold = max(0.0, min(100.0, float(cfg["force_phase_change_threshold_pct"])))
    step_pct = max(0.1, float(cfg["force_motor_step_pct"]))
    low = clamp(float(cfg["force_low_intensity_pct"]), 0.0, 100.0)
    high = clamp(float(cfg["force_high_intensity_pct"]), low, 100.0)
    order = [int(i) for i in cfg.get("force_motor_order", []) if 0 <= int(i) < motor_count]
    if not order:
        order = list(range(motor_count))

    if pct <= 0.0:
        return out, "zero"

    if pct < phase_threshold:
        active_count = int(math.ceil(pct / step_pct))
        active_count = max(1, min(motor_count, active_count))
        for idx in order[:active_count]:
            out[idx] = low
        return out, "adding_motors"

    if phase_threshold >= 100.0:
        intensity = low
    else:
        frac = (pct - phase_threshold) / (100.0 - phase_threshold)
        intensity = low + frac * (high - low)
    for idx in range(motor_count):
        out[idx] = intensity
    return out, "all_motors_intensity"
