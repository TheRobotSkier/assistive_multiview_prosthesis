"""Tests for emg_bridge.experiment_config — config schema and YAML loading."""

from __future__ import annotations

import contextlib
import tempfile
from pathlib import Path

import pytest

from emg_bridge.experiment_config import (
    ExperimentConfig,
    GestureStabilityConfig,
    ImuFeaturesConfig,
    NaviFlameConfig,
    ProportionalSlewConfig,
    load_config,
)


# ── Defaults ───────────────────────────────────────────────────────────────────


def test_default_config_is_all_off():
    """Default ExperimentConfig has every experiment disabled."""
    cfg = ExperimentConfig()
    assert cfg.classifier_backend == "sklearn"
    assert cfg.imu_features.enabled is False
    assert cfg.naviflame.enabled is False
    assert cfg.proportional_slew.enabled is False
    assert cfg.gesture_stability.enabled is False


def test_imu_features_defaults():
    cfg = ExperimentConfig().imu_features
    assert cfg.enabled is False
    assert cfg.gyro is True
    assert cfg.accel is False
    assert cfg.movement_gate == 0.0


def test_naviflame_defaults():
    cfg = ExperimentConfig().naviflame
    assert cfg.enabled is False
    assert cfg.config_path == "/prosthesis_ws/NaviFlame/config.json"
    assert cfg.gesture_mapping == {}


def test_proportional_slew_defaults():
    cfg = ExperimentConfig().proportional_slew
    assert cfg.enabled is False
    assert cfg.max_velocity_per_s == 0.0
    assert cfg.max_accel_per_s2 == 0.0
    assert cfg.max_delta_per_step == 0.0
    assert cfg.max_fall_velocity_per_s == 0.0
    assert cfg.reset_on_rest is True
    assert cfg.initial_value == 0.0
    assert cfg.snap_to_zero_below == 0.0


def test_gesture_stability_defaults():
    cfg = ExperimentConfig().gesture_stability
    assert cfg.enabled is False
    assert cfg.min_confidence_to_switch == 0.55
    assert cfg.min_margin_to_switch == 0.0
    assert cfg.min_frames == 3
    assert cfg.min_hold_s == 0.0
    assert cfg.release_behavior == "allow_rest_immediately"
    assert cfg.fallback_behavior == "hold_previous"
    assert cfg.release_lower_threshold == 0.0


def test_experiment_config_is_immutable():
    """ExperimentConfig and sub-configs are frozen."""
    cfg = ExperimentConfig()
    with pytest.raises(Exception):
        cfg.classifier_backend = "naviflame"  # type: ignore[misc]
    with pytest.raises(Exception):
        cfg.proportional_slew.enabled = True  # type: ignore[misc]


# ── YAML loading ──────────────────────────────────────────────────────────────


def test_load_yaml_sklearn_imu_mode():
    yaml_text = """
classifier_backend: sklearn_imu
imu_features:
  enabled: true
  gyro: true
  accel: true
"""
    with _temp_yaml(yaml_text) as path:
        cfg = load_config(path)
    assert cfg.classifier_backend == "sklearn_imu"
    assert cfg.imu_features.enabled is True
    assert cfg.imu_features.gyro is True
    assert cfg.imu_features.accel is True
    # Other features remain off
    assert cfg.naviflame.enabled is False
    assert cfg.proportional_slew.enabled is False
    assert cfg.gesture_stability.enabled is False


def test_load_yaml_naviflame_mode():
    yaml_text = """
classifier_backend: naviflame
naviflame:
  enabled: true
  config_path: /custom/path/config.json
  gesture_mapping:
    "0": 0
    "1": 1
"""
    with _temp_yaml(yaml_text) as path:
        cfg = load_config(path)
    assert cfg.classifier_backend == "naviflame"
    assert cfg.naviflame.enabled is True
    assert cfg.naviflame.config_path == "/custom/path/config.json"
    assert cfg.naviflame.gesture_mapping == {"0": 0, "1": 1}


def test_load_yaml_proportional_slew_mode():
    yaml_text = """
proportional_slew:
  enabled: true
  max_velocity_per_s: 2.5
  max_accel_per_s2: 10.0
  reset_on_rest: false
  snap_to_zero_below: 0.05
"""
    with _temp_yaml(yaml_text) as path:
        cfg = load_config(path)
    assert cfg.classifier_backend == "sklearn"  # default
    assert cfg.proportional_slew.enabled is True
    assert cfg.proportional_slew.max_velocity_per_s == 2.5
    assert cfg.proportional_slew.max_accel_per_s2 == 10.0
    assert cfg.proportional_slew.reset_on_rest is False
    assert cfg.proportional_slew.snap_to_zero_below == 0.05


def test_load_yaml_gesture_stability_mode():
    yaml_text = """
gesture_stability:
  enabled: true
  min_confidence_to_switch: 0.7
  min_frames: 5
  min_hold_s: 0.2
  release_behavior: require_threshold
  fallback_behavior: rest
"""
    with _temp_yaml(yaml_text) as path:
        cfg = load_config(path)
    assert cfg.gesture_stability.enabled is True
    assert cfg.gesture_stability.min_confidence_to_switch == 0.7
    assert cfg.gesture_stability.min_frames == 5
    assert cfg.gesture_stability.min_hold_s == 0.2
    assert cfg.gesture_stability.release_behavior == "require_threshold"
    assert cfg.gesture_stability.fallback_behavior == "rest"


def test_load_yaml_partial_overrides_dont_lose_defaults():
    """Only specified keys override; everything else stays default."""
    yaml_text = """
classifier_backend: naviflame
proportional_slew:
  enabled: true
"""
    with _temp_yaml(yaml_text) as path:
        cfg = load_config(path)
    assert cfg.classifier_backend == "naviflame"
    assert cfg.proportional_slew.enabled is True
    # Other proportional_slew values unchanged
    assert cfg.proportional_slew.reset_on_rest is True
    assert cfg.proportional_slew.max_velocity_per_s == 0.0
    # Other sections unchanged
    assert cfg.imu_features.enabled is False
    assert cfg.naviflame.enabled is False
    assert cfg.gesture_stability.enabled is False


# ── Error handling ─────────────────────────────────────────────────────────────


def test_load_unknown_backend_raises():
    yaml_text = """
classifier_backend: unsupported_thing
"""
    with _temp_yaml(yaml_text) as path:
        with pytest.raises(ValueError, match="classifier_backend"):
            load_config(path)


def test_load_missing_file_raises():
    with pytest.raises(FileNotFoundError):
        load_config("/nonexistent/path/emg_config.yaml")


def test_load_empty_yaml_gives_defaults():
    with _temp_yaml("") as path:
        cfg = load_config(path)
    assert cfg.classifier_backend == "sklearn"
    assert cfg.imu_features.enabled is False


def test_load_none_skips_file():
    """When config_path is None, load_config should use pure defaults."""
    cfg = load_config(None)
    assert cfg.classifier_backend == "sklearn"
    assert cfg.imu_features.enabled is False


# ── Helpers ────────────────────────────────────────────────────────────────────


def test_config_backwards_compat_unchanged():
    """Verify existing classifier constants (confidence_threshold, etc.) are
    not altered — the ExperimentConfig is additive."""
    from emg_bridge.config import (
        CONFIDENCE_THRESHOLD,
        GESTURE_NAMES,
        N_CHANNELS,
        N_GESTURES,
        WINDOW_LEN,
        WINDOW_STEP,
    )
    assert CONFIDENCE_THRESHOLD == 0.55
    assert len(GESTURE_NAMES) == 5
    assert GESTURE_NAMES[0] == "REST"
    assert N_GESTURES == 5
    assert N_CHANNELS == 8
    assert WINDOW_LEN == 100
    assert WINDOW_STEP == 50


@contextlib.contextmanager
def _temp_yaml(content: str):
    """Write *content* to a temp file, yield its Path, then clean up."""
    fd, raw = tempfile.mkstemp(suffix=".yaml", prefix="emg_test_")
    import os

    os.write(fd, content.encode("utf-8"))
    os.close(fd)
    path = Path(raw)
    try:
        yield path
    finally:
        path.unlink(missing_ok=True)
