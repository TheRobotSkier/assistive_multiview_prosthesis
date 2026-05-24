#!/usr/bin/env python3
"""Validate an emg_grasp_test.yaml config file.

Exit codes:
    0  – config is valid
    1  – config is invalid or missing (actionable error printed to stderr)

Usage:
    python scripts/validate_emg_grasp_config.py config/emg_grasp_test.yaml
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError as exc:  # pragma: no cover
    print(f"ERROR: {exc}.  Install PyYAML (pip install pyyaml).", file=sys.stderr)
    sys.exit(1)


# ── helpers ─────────────────────────────────────────────────────────────────

class ValidationError(Exception):
    """Raised on the first validation failure so we can fail fast."""


def _type_name(value: Any) -> str:
    if value is None:
        return "None"
    t = type(value).__name__
    if isinstance(value, bool):
        return "bool"
    return t


def _fmt(value: Any) -> str:
    if isinstance(value, float):
        return f"{value!r}"
    return str(value)


def _fail(path: str, msg: str) -> None:
    raise ValidationError(f"  [{path}] {msg}")


def _require(data: dict, path: str, key: str, types: tuple[type, ...]) -> Any:
    """Require *key* to exist under *path* and be one of *types*."""
    if key not in data:
        _fail(path, f"missing required key '{key}'")
    value = data[key]
    if not isinstance(value, types):
        _fail(
            f"{path}.{key}",
            f"expected type {' | '.join(t.__name__ for t in types)}, "
            f"got {_type_name(value)}",
        )
    return value


def _require_positive(data: dict, path: str, key: str, *, allow_zero: bool = False) -> float:
    """Require a numeric key whose value is strictly positive (or non-negative)."""
    value = _require(data, path, key, (int, float))
    if allow_zero:
        if value < 0:
            _fail(f"{path}.{key}", f"must be >= 0, got {_fmt(value)}")
    else:
        if value <= 0:
            _fail(f"{path}.{key}", f"must be > 0, got {_fmt(value)}")
    return float(value)


def _require_range(
    data: dict, path: str, key: str, low: float, high: float, *, inclusive: bool = True
) -> float:
    """Require a numeric key inside [low, high] (or (low, high) if inclusive=False)."""
    value = _require(data, path, key, (int, float))
    if inclusive:
        if not (low <= value <= high):
            _fail(f"{path}.{key}", f"must be in [{low}, {high}], got {_fmt(value)}")
    else:
        if not (low < value < high):
            _fail(f"{path}.{key}", f"must be in ({low}, {high}), got {_fmt(value)}")
    return float(value)


def _require_list(data: dict, path: str, key: str, item_type: type, *, min_len: int = 0) -> list:
    """Require a list of *item_type* with at least *min_len* elements."""
    value = _require(data, path, key, (list,))
    if len(value) < min_len:
        _fail(f"{path}.{key}", f"list must have >= {min_len} items, got {len(value)}")
    for i, item in enumerate(value):
        if not isinstance(item, item_type):
            _fail(
                f"{path}.{key}[{i}]",
                f"expected {item_type.__name__}, got {_type_name(item)}",
            )
    return value


def _require_dict(data: dict, path: str, key: str) -> dict:
    """Require a dict sub-key."""
    return _require(data, path, key, (dict,))


def _warn(msg: str) -> None:
    print(f"  WARNING: {msg}", file=sys.stderr)


# ── section validators ──────────────────────────────────────────────────────

REQUIRED_TOP_LEVEL = [
    "emg",
    "modes",
    "safety",
    "force",
    "velocity",
    "positions",
    "wrist",
    "command",
    "hardware",
]

FINGER_JOINTS = ("j_thumb_fle", "j_index_fle", "j_mrl_fle")

CANONICAL_GESTURES = {"REST", "FLEXION", "EXTENSION", "OPEN", "POINT"}


def _validate_emg(data: dict, path: str = "emg") -> None:
    sec = _require_dict(data, "<root>", "emg")
    gesture_names = _require_dict(sec, path, "gesture_names")
    for g in CANONICAL_GESTURES:
        if g not in gesture_names:
            _fail(f"{path}.gesture_names", f"missing canonical gesture '{g}'")
        if not isinstance(gesture_names[g], int):
            _fail(f"{path}.gesture_names.{g}", f"expected int, got {_type_name(gesture_names[g])}")

    # grasp_gestures must be a non-empty list of ints
    grasp = _require_list(sec, path, "grasp_gestures", int, min_len=1)
    release = _require(sec, path, "release_gesture", (int,))
    if release in grasp:
        _fail(f"{path}.release_gesture", f"release gesture ({release}) must NOT be in grasp_gestures")

    _require_range(sec, path, "confidence_threshold", 0.0, 1.0)
    _require_range(sec, path, "release_confidence_threshold", 0.0, 1.0)

    # Aliases are optional but if present must map to canonical names
    if "gesture_aliases" in sec:
        aliases = sec["gesture_aliases"]
        if not isinstance(aliases, dict):
            _fail(f"{path}.gesture_aliases", f"expected dict, got {_type_name(aliases)}")
        for alias, canonical in aliases.items():
            if canonical not in CANONICAL_GESTURES:
                _fail(
                    f"{path}.gesture_aliases.{alias}",
                    f"maps to unknown canonical gesture '{canonical}'",
                )


def _validate_modes(data: dict, path: str = "modes") -> None:
    sec = _require_dict(data, "<root>", "modes")
    _require_range(sec, path, "threshold_open", 0.0, 1.0)
    _require_range(sec, path, "threshold_power", 0.0, 1.0)
    _require_range(sec, path, "deadzone", 0.0, 1.0)
    _require_range(sec, path, "hysteresis", 0.0, 1.0)

    # Sanity: deadzone should be smaller than the lowest mode threshold
    if sec["deadzone"] >= sec.get("threshold_power", 0):
        _warn(f"{path}.deadzone ({sec['deadzone']}) >= threshold_power ({sec.get('threshold_power', 0)}) — modes may never activate")


def _validate_safety(data: dict, path: str = "safety") -> None:
    sec = _require_dict(data, "<root>", "safety")
    _require_positive(sec, path, "open_safety_hold_s")
    _require_positive(sec, path, "power_toggle_hold_s")
    _require_positive(sec, path, "power_debounce_s")


def _validate_force(data: dict, path: str = "force") -> None:
    sec = _require_dict(data, "<root>", "force")

    targets = _require_dict(sec, path, "targets")
    for joint in FINGER_JOINTS:
        _require_positive(targets, f"{path}.targets", joint)

    _require_positive(sec, path, "adjustment_rate_up")
    _require_positive(sec, path, "adjustment_rate_down")
    _require_positive(sec, path, "target_min")
    _require_positive(sec, path, "target_max")

    if sec["target_min"] >= sec["target_max"]:
        _fail(f"{path}.target_min", f"must be < target_max ({sec['target_max']})")

    _require_positive(sec, path, "emergency_threshold")
    if sec["emergency_threshold"] <= sec["target_max"]:
        _warn(f"{path}.emergency_threshold ({sec['emergency_threshold']}) <= target_max ({sec['target_max']}) — emergency will trigger during normal operation")

    _require_positive(sec, path, "kp", allow_zero=True)
    _require_positive(sec, path, "ki", allow_zero=True)
    _require_positive(sec, path, "integral_limit")
    _require_positive(sec, path, "max_position_step")
    _require_positive(sec, path, "emergency_backoff_factor")
    _require_positive(sec, path, "filter_window")
    _require_positive(sec, path, "stability_window_s")
    _require_positive(sec, path, "stability_tolerance")
    _require_positive(sec, path, "slip_threshold")


def _validate_velocity(data: dict, path: str = "velocity") -> None:
    sec = _require_dict(data, "<root>", "velocity")
    _require_positive(sec, path, "max_velocity")
    _require_positive(sec, path, "closing_start")
    _require_positive(sec, path, "closing_end")

    if sec["closing_start"] <= sec["closing_end"]:
        _warn(f"{path}.closing_start ({sec['closing_start']}) <= closing_end ({sec['closing_end']}) — no actual ramp down")

    if sec["closing_start"] > sec["max_velocity"]:
        _fail(f"{path}.closing_start", f"must be <= max_velocity ({sec['max_velocity']})")

    steps = _require(sec, path, "decay_steps", (int,))
    if steps < 1:
        _fail(f"{path}.decay_steps", f"must be >= 1, got {steps}")
    _require_positive(sec, path, "step_interval_s")


def _validate_positions(data: dict, path: str = "positions") -> None:
    sec = _require_dict(data, "<root>", "positions")

    open_pos = _require_dict(sec, path, "open")
    max_cls = _require_dict(sec, path, "max_closure")
    for joint in FINGER_JOINTS:
        _require_range(open_pos, f"{path}.open", joint, 0.0, 3.0)
        _require_range(max_cls, f"{path}.max_closure", joint, 0.0, 3.0)
        if open_pos[joint] >= max_cls[joint]:
            _fail(
                f"{path}",
                f"open.{joint} ({open_pos[joint]}) must be < max_closure.{joint} ({max_cls[joint]})",
            )

    _require_positive(sec, path, "min_overshoot")
    _require_positive(sec, path, "max_overshoot")
    if sec["min_overshoot"] >= sec["max_overshoot"]:
        _fail(f"{path}.min_overshoot", f"must be < max_overshoot ({sec['max_overshoot']})")


def _validate_wrist(data: dict, path: str = "wrist") -> None:
    sec = _require_dict(data, "<root>", "wrist")
    _require_positive(sec, path, "max_velocity")
    _require(sec, path, "position_min", (int, float))
    _require(sec, path, "position_max", (int, float))
    if sec["position_min"] >= sec["position_max"]:
        _fail(f"{path}.position_min", f"must be < position_max ({sec['position_max']})")
    _require_positive(sec, path, "adjustment_speed")


def _validate_command(data: dict, path: str = "command") -> None:
    sec = _require_dict(data, "<root>", "command")
    _require_positive(sec, path, "joint_rate_hz")
    _require_positive(sec, path, "emg_update_rate_hz")
    _require_positive(sec, path, "stale_emg_timeout_s")
    _require_positive(sec, path, "stale_force_timeout_s")
    _require_positive(sec, path, "stale_joint_timeout_s")


def _validate_hardware(data: dict, path: str = "hardware") -> None:
    sec = _require_dict(data, "<root>", "hardware")
    _require(sec, path, "mock_hand", (bool,))
    _require(sec, path, "mock_wrist", (bool,))
    _require(sec, path, "mock_emg", (bool,))

    if not sec["mock_hand"]:
        port = _require(sec, path, "mia_serial_port", (str,))
        if not port:
            _fail(f"{path}.mia_serial_port", "must be a non-empty string when mock_hand is false")

    if not sec["mock_wrist"]:
        port = _require(sec, path, "wrist_port", (str,))
        if not port:
            _fail(f"{path}.wrist_port", "must be a non-empty string when mock_wrist is false")
        _require_positive(sec, path, "wrist_baudrate")
        _require(sec, path, "wrist_motor_id", (int,))


def _validate_mock_overrides(data: dict) -> None:
    """Warn if mock_overrides is present but the hardware mock flags are false."""
    if "mock_overrides" not in data:
        return
    hw = data.get("hardware", {})
    any_mock = hw.get("mock_hand", False) or hw.get("mock_wrist", False) or hw.get("mock_emg", False)
    if not any_mock:
        _warn(
            "'mock_overrides' section is present but all hardware.mock_* flags are false. "
            "Did you forget to enable mock mode for testing?"
        )


# ── main entry point ────────────────────────────────────────────────────────

def validate(raw: dict, *, source: str = "config") -> list[str]:
    """Validate a loaded config dict.

    Returns a list of warning strings.  Raises ValidationError on fatal issues.
    """
    warnings: list[str] = []

    # -- top-level keys -------------------------------------------------------
    missing = [k for k in REQUIRED_TOP_LEVEL if k not in raw]
    if missing:
        _fail("<root>", f"missing required top-level sections: {missing}")

    # -- extra keys -----------------------------------------------------------
    allowed = set(REQUIRED_TOP_LEVEL) | {"mock_overrides"}
    extra = set(raw.keys()) - allowed
    if extra:
        _warn(f"unknown top-level keys will be ignored: {sorted(extra)}")

    # -- sections -------------------------------------------------------------
    _validate_emg(raw)
    _validate_modes(raw)
    _validate_safety(raw)
    _validate_force(raw)
    _validate_velocity(raw)
    _validate_positions(raw)
    _validate_wrist(raw)
    _validate_command(raw)
    _validate_hardware(raw)
    _validate_mock_overrides(raw)

    return warnings


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(f"Usage: {argv[0]} <path-to-emg_grasp_test.yaml>", file=sys.stderr)
        return 1

    path = Path(argv[1])
    if not path.is_file():
        print(f"ERROR: file not found: {path}", file=sys.stderr)
        return 1

    try:
        raw = yaml.safe_load(path.read_text())
    except yaml.YAMLError as exc:
        print(f"ERROR: YAML syntax error in {path}:\n{exc}", file=sys.stderr)
        return 1

    if raw is None:
        print(f"ERROR: {path} is empty or contains only comments.", file=sys.stderr)
        return 1

    if not isinstance(raw, dict):
        print(f"ERROR: {path} does not contain a top-level YAML mapping.", file=sys.stderr)
        return 1

    print(f"Validating {path} …")
    try:
        warnings = validate(raw, source=str(path))
    except ValidationError as exc:
        print(f"\nFAILED\n{exc}", file=sys.stderr)
        return 1

    if warnings:
        print("\nWARNINGS:")
        for w in warnings:
            print(f"  • {w}")

    print("\nOK — config is valid.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
