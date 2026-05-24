"""Tests for openvins_profile.py profile loading and openvins_experiment_profiles.yaml."""

from pathlib import Path
import sys
import math

import yaml
import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "launch"
sys.path.insert(0, str(SCRIPT_DIR))

from openvins_profile import load_profiles, get_profile_overrides  # noqa: E402


PKG_DIR = Path(__file__).resolve().parents[1]
CONFIG_OPENVINS_DIR = PKG_DIR / "config" / "openvins"

EXPECTED_PROFILES = [
    "baseline",
    "marker_strong_ekf",
    "marker_easier_initial_lock",
    "reset_bias_policy",
    "calib_extrinsics",
    "zupt",
    "imu_frame_variant",
    "all_changes",
]


def test_baseline_returns_empty_overrides():
    overrides = get_profile_overrides(PKG_DIR, "baseline")
    assert overrides == {}


def test_baseline_empty_string_returns_empty_overrides():
    overrides = get_profile_overrides(PKG_DIR, "")
    assert overrides == {}


def test_all_profiles_parseable():
    data = load_profiles(PKG_DIR)
    available = set(data["profiles"].keys())
    for profile_name in EXPECTED_PROFILES:
        assert profile_name in available, f"profile '{profile_name}' missing from YAML"
    assert available == set(EXPECTED_PROFILES)


def test_invalid_profile_raises_error():
    with pytest.raises(RuntimeError, match="Unknown experiment profile 'nonexistent'"):
        get_profile_overrides(PKG_DIR, "nonexistent")


def test_marker_strong_ekf_overrides_correct_params():
    overrides = get_profile_overrides(PKG_DIR, "marker_strong_ekf")

    for section in ("head_marker", "arm_marker"):
        assert section in overrides
        o = overrides[section]
        assert math.isclose(o["marker_chi2_gate"], 25.0)
        assert math.isclose(o["marker_noise_multiplier"], 0.25)
        assert math.isclose(o["marker_max_update_translation_m"], 0.50)
        assert math.isclose(o["marker_max_update_rotation_deg"], 40.0)


def test_easier_initial_lock_has_zero_velocity_fallback():
    overrides = get_profile_overrides(PKG_DIR, "marker_easier_initial_lock")

    for section in ("head_marker", "arm_marker"):
        o = overrides[section]
        assert o["marker_initial_lock_allow_zero_velocity"] is True
        assert math.isclose(o["marker_initial_lock_velocity_cov_std"], 0.5)
        assert math.isclose(o["marker_reset_translation_m"], 0.80)
        assert math.isclose(o["marker_reset_rotation_deg"], 30.0)
        assert o["marker_reset_min_samples"] == 3
        assert math.isclose(o["marker_reset_window_s"], 0.30)


def test_reset_bias_policy_has_correct_policy():
    overrides = get_profile_overrides(PKG_DIR, "reset_bias_policy")

    for section in ("head_marker", "arm_marker"):
        o = overrides[section]
        assert o["marker_reset_bias_policy"] == "zero_on_initial_lock"
        assert math.isclose(o["marker_reset_bias_gyro_std"], 0.05)
        assert math.isclose(o["marker_reset_bias_accel_std"], 0.50)


def test_all_changes_combines_all_marker_params():
    overrides = get_profile_overrides(PKG_DIR, "all_changes")

    expected_keys = {
        "marker_chi2_gate",
        "marker_noise_multiplier",
        "marker_max_update_translation_m",
        "marker_max_update_rotation_deg",
        "marker_reset_translation_m",
        "marker_reset_rotation_deg",
        "marker_reset_min_samples",
        "marker_reset_window_s",
        "marker_reset_bias_gyro_std",
        "marker_reset_bias_accel_std",
        "marker_initial_lock_allow_zero_velocity",
        "marker_initial_lock_velocity_cov_std",
        "marker_reset_bias_policy",
    }

    for section in ("head_marker", "arm_marker"):
        o = overrides[section]
        actual_keys = set(o.keys())
        assert actual_keys == expected_keys, f"{section} keys mismatch: {actual_keys} != {expected_keys}"
        assert math.isclose(o["marker_chi2_gate"], 22.46)
        assert math.isclose(o["marker_noise_multiplier"], 0.5)
        assert math.isclose(o["marker_max_update_translation_m"], 0.40)
        assert math.isclose(o["marker_max_update_rotation_deg"], 35.0)
        assert math.isclose(o["marker_reset_translation_m"], 0.80)
        assert math.isclose(o["marker_reset_rotation_deg"], 30.0)
        assert o["marker_reset_min_samples"] == 3
        assert math.isclose(o["marker_reset_window_s"], 0.30)
        assert math.isclose(o["marker_reset_bias_gyro_std"], 0.05)
        assert math.isclose(o["marker_reset_bias_accel_std"], 0.50)
        assert o["marker_initial_lock_allow_zero_velocity"] is True
        assert math.isclose(o["marker_initial_lock_velocity_cov_std"], 0.5)
        assert o["marker_reset_bias_policy"] == "zero_on_initial_lock"


def test_calib_extrinsics_references_valid_config_paths():
    overrides = get_profile_overrides(PKG_DIR, "calib_extrinsics")

    head_config = overrides["head_config"]
    arm_config = overrides["arm_config"]
    assert (CONFIG_OPENVINS_DIR / head_config).exists(), f"missing {head_config}"
    assert (CONFIG_OPENVINS_DIR / arm_config).exists(), f"missing {arm_config}"

    for section in ("head_marker", "arm_marker"):
        o = overrides[section]
        assert math.isclose(o["marker_noise_multiplier"], 10.0)
        assert math.isclose(o["marker_chi2_gate"], 22.46)


def test_zupt_references_valid_config_paths():
    overrides = get_profile_overrides(PKG_DIR, "zupt")

    head_config = overrides["head_config"]
    arm_config = overrides["arm_config"]
    assert (CONFIG_OPENVINS_DIR / head_config).exists(), f"missing {head_config}"
    assert (CONFIG_OPENVINS_DIR / arm_config).exists(), f"missing {arm_config}"


def test_imu_frame_variant_has_no_overrides():
    overrides = get_profile_overrides(PKG_DIR, "imu_frame_variant")
    assert "head_marker" not in overrides
    assert "arm_marker" not in overrides


def test_profile_params_are_finite_and_valid():
    param_sets = []

    for profile_name in EXPECTED_PROFILES:
        if profile_name in ("baseline", "imu_frame_variant"):
            continue
        overrides = get_profile_overrides(PKG_DIR, profile_name)
        for section in ("head_marker", "arm_marker"):
            if section in overrides:
                param_sets.append((profile_name, section, overrides[section]))

    for profile, section, o in param_sets:
        if "marker_chi2_gate" in o:
            assert o["marker_chi2_gate"] > 0.0, f"{profile}/{section} chi2_gate <= 0"
            assert math.isfinite(o["marker_chi2_gate"]), f"{profile}/{section} chi2_gate not finite"
        if "marker_noise_multiplier" in o:
            assert o["marker_noise_multiplier"] > 0.0, f"{profile}/{section} noise_multiplier <= 0"
            assert math.isfinite(o["marker_noise_multiplier"]), f"{profile}/{section} noise_multiplier not finite"
        if "marker_max_update_translation_m" in o:
            assert o["marker_max_update_translation_m"] > 0.0, f"{profile}/{section} max_update_translation <= 0"
            assert math.isfinite(o["marker_max_update_translation_m"]), f"{profile}/{section} max_update_translation not finite"
        if "marker_max_update_rotation_deg" in o:
            assert o["marker_max_update_rotation_deg"] > 0.0, f"{profile}/{section} max_update_rotation <= 0"
            assert math.isfinite(o["marker_max_update_rotation_deg"]), f"{profile}/{section} max_update_rotation not finite"
        if "marker_reset_translation_m" in o:
            assert o["marker_reset_translation_m"] > 0.0, f"{profile}/{section} reset_translation <= 0"
            assert math.isfinite(o["marker_reset_translation_m"]), f"{profile}/{section} reset_translation not finite"
        if "marker_reset_rotation_deg" in o:
            assert o["marker_reset_rotation_deg"] > 0.0, f"{profile}/{section} reset_rotation <= 0"
            assert math.isfinite(o["marker_reset_rotation_deg"]), f"{profile}/{section} reset_rotation not finite"
        if "marker_reset_min_samples" in o:
            assert o["marker_reset_min_samples"] > 0, f"{profile}/{section} reset_min_samples <= 0"
        if "marker_reset_window_s" in o:
            assert o["marker_reset_window_s"] > 0.0, f"{profile}/{section} reset_window_s <= 0"
            assert math.isfinite(o["marker_reset_window_s"]), f"{profile}/{section} reset_window_s not finite"
        if "marker_reset_bias_gyro_std" in o:
            assert o["marker_reset_bias_gyro_std"] > 0.0, f"{profile}/{section} bias_gyro_std <= 0"
            assert math.isfinite(o["marker_reset_bias_gyro_std"]), f"{profile}/{section} bias_gyro_std not finite"
        if "marker_reset_bias_accel_std" in o:
            assert o["marker_reset_bias_accel_std"] > 0.0, f"{profile}/{section} bias_accel_std <= 0"
            assert math.isfinite(o["marker_reset_bias_accel_std"]), f"{profile}/{section} bias_accel_std not finite"
