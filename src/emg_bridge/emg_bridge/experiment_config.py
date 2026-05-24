"""Configurable EMG experiment mode schema.

Command-line and YAML-based toggles for the four experimental features:
    1. Classifier backend selection (sklearn | sklearn_imu | naviflame)
    2. Proportional slew / acceleration limiting
    3. Sticky gesture selection with confidence hysteresis
    4. IMU feature augmentation

Loads from a YAML file and merges with CLI overrides.  The returned
ExperimentConfig is a frozen dataclass that downstream code can rely on
without worrying about mutation.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, get_type_hints

import yaml  # type: ignore

# ── Default config path (relative to workspace root inside the container) ──────
DEFAULT_CONFIG_PATH = "/prosthesis_ws/config/emg_experiment_config.yaml"


# ── Dataclasses ────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class ImuFeaturesConfig:
    """IMU feature toggles.

    When *enabled* the board reader must supply gyro (and optionally accel)
    data, and the sklearn_imu backend concatenates IMU features.
    """

    enabled: bool = False
    gyro: bool = True
    accel: bool = False
    movement_gate: float = 0.0  # gyro magnitude threshold; 0 = disabled


@dataclass(frozen=True)
class NaviFlameConfig:
    """NaviFlame adapter configuration.

    Paths default to the checked-in NaviFlame/config.json inside the
    container.  The adapter itself honours the filter and threshold defaults
    from NaviFlame/naviflame/inference.py.
    """

    enabled: bool = False
    config_path: str = "/prosthesis_ws/NaviFlame/config.json"
    gesture_mapping: dict[str, int] = field(default_factory=lambda: {})


@dataclass(frozen=True)
class ProportionalSlewConfig:
    """Slew / acceleration limiting for the /emg/proportional signal."""

    enabled: bool = False
    max_velocity_per_s: float = 0.0   # 0 → unlimited
    max_accel_per_s2: float = 0.0     # 0 → unlimited
    max_delta_per_step: float = 0.0   # 0 → unlimited (absolute cap per frame)
    max_fall_velocity_per_s: float = 0.0  # asymmetric limit for closing → open
    reset_on_rest: bool = True
    initial_value: float = 0.0
    snap_to_zero_below: float = 0.0   # 0 → disabled; forces output to 0 below threshold


@dataclass(frozen=True)
class GestureStabilityConfig:
    """Confidence-hysteresis (sticky) gesture selection.

    The stabilizer prevents transient misclassifications from flipping the
    output gesture.  It is applied *after* the backend prediction and is
    backend-neutral.
    """

    enabled: bool = False
    min_confidence_to_switch: float = 0.55
    min_margin_to_switch: float = 0.0   # prob gap between top-two classes
    min_frames: int = 3                 # consecutive frames above threshold
    min_hold_s: float = 0.0             # minimum hold time before next switch
    release_behavior: str = "allow_rest_immediately"
    # allow_rest_immediately | require_threshold | hold_previous_when_uncertain
    fallback_behavior: str = "hold_previous"
    # hold_previous | rest
    release_lower_threshold: float = 0.0  # lower confidence bar for release/rest


@dataclass(frozen=True)
class ExperimentConfig:
    """Root configuration for EMG experimental modes.

    All fields have defaults that preserve the current (pre-experiment)
    behaviour exactly.  Features are opt-in: setting *enabled* to False
    gives identical output to the current master-branch code.
    """

    classifier_backend: str = "sklearn"  # sklearn | sklearn_imu | naviflame
    imu_features: ImuFeaturesConfig = field(default_factory=ImuFeaturesConfig)
    naviflame: NaviFlameConfig = field(default_factory=NaviFlameConfig)
    proportional_slew: ProportionalSlewConfig = field(
        default_factory=ProportionalSlewConfig
    )
    gesture_stability: GestureStabilityConfig = field(
        default_factory=GestureStabilityConfig
    )


# ── Merging helpers ────────────────────────────────────────────────────────────


def _merge_nested(default: dict[str, Any], overrides: dict[str, Any]) -> dict[str, Any]:
    """Recursively merge overrides into defaults (dict-level only)."""
    merged = dict(default)
    for key, value in overrides.items():
        if (
            key in merged
            and isinstance(merged[key], dict)
            and isinstance(value, dict)
        ):
            merged[key] = _merge_nested(merged[key], value)
        else:
            merged[key] = value
    return merged


def _dict_to_dataclass(dc: type[Any], data: dict[str, Any]) -> Any:
    """Build a frozen dataclass instance from a flat or nested dict.

    Sub-dataclass fields are recognised by their resolved type annotation.
    """
    resolved = get_type_hints(dc)
    kwargs: dict[str, Any] = {}
    for key, value in data.items():
        if key not in resolved:
            continue  # ignore unknown keys
        ftype = resolved[key]
        if isinstance(value, dict) and hasattr(ftype, "__dataclass_fields__"):
            kwargs[key] = _dict_to_dataclass(ftype, value)
        else:
            kwargs[key] = value
    return dc(**kwargs)


# ── Public API ─────────────────────────────────────────────────────────────────


def load_config(
    config_path: str | Path | None = None,
    cli_overrides: dict[str, Any] | None = None,
) -> ExperimentConfig:
    """Load EMG experiment configuration.

    Resolution order (later wins):
        1. Frozen dataclass defaults (ExperimentConfig defaults).
        2. YAML file at *config_path*.
        3. *cli_overrides* dict (flat keys, e.g. {"classifier_backend": "naviflame"}).

    Args:
        config_path: Path to a YAML experiment config file.  If None, the
            file is skipped gracefully.
        cli_overrides: Optional flat dict with top-level key overrides.

    Returns:
        A frozen ExperimentConfig instance.

    Raises:
        FileNotFoundError: path supplied but does not exist.
        ValueError: YAML contains unknown or invalid values.
    """
    # Dataclass defaults (nested sections provide their own)
    nested_defaults = _dataclass_to_flat(ExperimentConfig())

    merged = dict(nested_defaults)

    # Layer 2: YAML file
    if config_path is not None:
        path = Path(config_path)
        if not path.exists():
            raise FileNotFoundError(f"Experiment config not found: {path}")
        yaml_data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        merged = _merge_nested(merged, yaml_data)

    # Layer 3: CLI overrides (top-level only)
    if cli_overrides:
        merged.update(cli_overrides)

    # Validate top-level classifier_backend
    valid_backends = {"sklearn", "sklearn_imu", "naviflame"}
    backend = merged.get("classifier_backend", "sklearn")
    if backend not in valid_backends:
        raise ValueError(
            f"Unknown classifier_backend '{backend}'. "
            f"Valid: {', '.join(sorted(valid_backends))}"
        )

    return _dict_to_dataclass(ExperimentConfig, merged)


def _dataclass_to_flat(dc: Any) -> dict[str, Any]:
    """Convert a dataclass tree to a flat-ish dict for YAML comparison."""
    result: dict[str, Any] = {}
    for f in fields(dc):
        val = getattr(dc, f.name)
        if hasattr(val, "__dataclass_fields__"):
            result[f.name] = _dataclass_to_flat(val)
        elif isinstance(val, dict):
            result[f.name] = dict(val)
        else:
            result[f.name] = val
    return result
