"""
Proportional control signal derived from window RMS.

The proportional value maps EMG contraction intensity to [0.0, 1.0] using
per-gesture calibration (min/max RMS recorded during data collection).

Calibration data is saved alongside the classifier model so that inference
and training always use the same reference.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from numpy.typing import NDArray

import joblib

from .config import GESTURE_NAMES, N_CHANNELS, REST_LABEL

PROP_CAL_FILE = "prop_calibration.pkl"

# Fallback when calibration is missing (use global RMS normalised to [0,1])
_GLOBAL_RMS_MAX_FALLBACK = 500.0


def compute_rms(window: NDArray) -> float:
    """Mean RMS across all channels for a (N_channels, window_len) window."""
    per_channel = np.sqrt(np.mean(window ** 2, axis=1))
    return float(np.mean(per_channel))


def calibrate(
    windows_by_class: dict[int, NDArray],
    *,
    percentile_low: float = 5.0,
    percentile_high: float = 95.0,
) -> dict:
    """Compute per-class RMS min/max from recorded training windows.

    Args:
        windows_by_class: mapping of class_label → ndarray (N, N_channels, window_len)
        percentile_low: lower percentile for robust min estimate
        percentile_high: upper percentile for robust max estimate

    Returns:
        calibration dict with keys: rms_min, rms_max (floats per class)
    """
    cal: dict[str, dict[int, float]] = {"rms_min": {}, "rms_max": {}}

    all_rms: list[float] = []
    for label, wins in windows_by_class.items():
        rms_vals = np.array([compute_rms(w) for w in wins])
        cal["rms_min"][label] = float(np.percentile(rms_vals, percentile_low))
        cal["rms_max"][label] = float(np.percentile(rms_vals, percentile_high))
        all_rms.extend(rms_vals.tolist())

    # Global range as fallback for unseen classes
    cal["global_min"] = float(np.percentile(all_rms, percentile_low))
    cal["global_max"] = float(np.percentile(all_rms, percentile_high))

    return cal


def compute_proportional(
    window: NDArray,
    label: int,
    calibration: dict | None = None,
) -> float:
    """Map window RMS to a [0.0, 1.0] proportional control value.

    REST class always returns 0.0.
    If calibration is None, a rough global normalisation is used.
    """
    if label == REST_LABEL:
        return 0.0

    rms = compute_rms(window)

    if calibration is None:
        return float(np.clip(rms / _GLOBAL_RMS_MAX_FALLBACK, 0.0, 1.0))

    rms_min = calibration["rms_min"].get(label, calibration.get("global_min", 0.0))
    rms_max = calibration["rms_max"].get(label, calibration.get("global_max", _GLOBAL_RMS_MAX_FALLBACK))

    if rms_max <= rms_min:
        return 0.0

    return float(np.clip((rms - rms_min) / (rms_max - rms_min), 0.0, 1.0))


def save(calibration: dict, model_dir: str | Path = "models") -> None:
    path = Path(model_dir) / PROP_CAL_FILE
    joblib.dump(calibration, path)


def load(model_dir: str | Path = "models") -> dict | None:
    path = Path(model_dir) / PROP_CAL_FILE
    if not path.exists():
        return None
    return joblib.load(path)
