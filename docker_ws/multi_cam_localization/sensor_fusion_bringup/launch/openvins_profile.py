"""Shared helper for loading and validating OpenVINS experiment profiles.

Import this from any launch file that needs to apply a named experiment
profile from config/openvins_experiment_profiles.yaml.
"""

from pathlib import Path

import yaml


def load_profiles(package_share_dir: Path) -> dict:
    """Load the full profile definitions from the YAML file."""
    profiles_path = package_share_dir / "config" / "openvins_experiment_profiles.yaml"
    if not profiles_path.exists():
        raise FileNotFoundError(f"Profile file not found: {profiles_path}")
    with open(profiles_path, "r") as f:
        data = yaml.safe_load(f)
    data = data or {}
    if "profiles" not in data:
        raise KeyError(
            f"Missing required 'profiles' top-level key in {profiles_path}"
        )
    return data


def get_profile_overrides(package_share_dir: Path, profile_name: str) -> dict:
    """Return the override dict for *profile_name*, or {} for baseline.

    Raises RuntimeError with available profile names when *profile_name*
    is not recognised.
    """
    profile_name = profile_name.strip()
    if not profile_name or profile_name == "baseline":
        return {}
    data = load_profiles(package_share_dir)
    profiles = data["profiles"]
    if profile_name not in profiles:
        available = sorted(profiles.keys())
        raise RuntimeError(
            f"Unknown experiment profile '{profile_name}'. "
            f"Available: {', '.join(available)}"
        )
    return profiles[profile_name]
