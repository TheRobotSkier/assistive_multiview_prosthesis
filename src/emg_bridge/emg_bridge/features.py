"""
EMG feature extraction.

All features are computed per-channel over a single window of shape
(N_channels, window_len), then concatenated into a 1-D feature vector.

Feature set (per channel):
    MAV  – Mean Absolute Value
    RMS  – Root Mean Square
    WL   – Waveform Length
    ZC   – Zero-Crossing count  (with dead-zone threshold)
    SSC  – Slope Sign Change count  (with dead-zone threshold)
    VAR  – Variance

Total: N_channels × 6 = 8 × 6 = 48 features by default.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from .config import N_CHANNELS

# Dead-zone thresholds to reduce noise-induced spurious counts
_ZC_THRESHOLD: float = 1e-4    # μV (units depend on board calibration)
_SSC_THRESHOLD: float = 1e-4


def _mav(x: NDArray) -> float:
    return float(np.mean(np.abs(x)))


def _rms(x: NDArray) -> float:
    return float(np.sqrt(np.mean(x ** 2)))


def _wl(x: NDArray) -> float:
    return float(np.sum(np.abs(np.diff(x))))


def _zc(x: NDArray, threshold: float = _ZC_THRESHOLD) -> float:
    """Zero crossings with hysteresis threshold (requires sign flip across zero)."""
    sign_flip = x[:-1] * x[1:] < 0
    large_enough = np.abs(x[:-1] - x[1:]) >= threshold
    return float(np.sum(sign_flip & large_enough))


def _ssc(x: NDArray, threshold: float = _SSC_THRESHOLD) -> float:
    """Slope sign changes: both adjacent slopes must flip sign and be significant."""
    d1 = x[1:-1] - x[:-2]
    d2 = x[2:] - x[1:-1]
    sign_change = d1 * d2 < 0
    large_enough = (np.abs(d1) >= threshold) & (np.abs(d2) >= threshold)
    return float(np.sum(sign_change & large_enough))


def _var(x: NDArray) -> float:
    return float(np.var(x))


_PER_CHANNEL_FUNCS = [_mav, _rms, _wl, _zc, _ssc, _var]
N_FEATURES_PER_CHANNEL = len(_PER_CHANNEL_FUNCS)
N_FEATURES_TOTAL = N_CHANNELS * N_FEATURES_PER_CHANNEL  # 48


def compute_features(window: NDArray) -> NDArray:
    """Compute the feature vector for a single window.

    Args:
        window: ndarray of shape (N_channels, window_len)

    Returns:
        1-D ndarray of length N_FEATURES_TOTAL (48)
    """
    feats: list[float] = []
    for ch in range(window.shape[0]):
        x = window[ch]
        for fn in _PER_CHANNEL_FUNCS:
            feats.append(fn(x))
    return np.array(feats, dtype=np.float64)


def compute_features_batch(windows: NDArray) -> NDArray:
    """Vectorised feature extraction over a batch of windows.

    Args:
        windows: ndarray of shape (N_windows, N_channels, window_len)

    Returns:
        ndarray of shape (N_windows, N_FEATURES_TOTAL)
    """
    return np.vstack([compute_features(w) for w in windows])


def feature_names() -> list[str]:
    """Return human-readable feature names for diagnostics / plots."""
    names: list[str] = []
    short = ["MAV", "RMS", "WL", "ZC", "SSC", "VAR"]
    for ch in range(N_CHANNELS):
        for s in short:
            names.append(f"ch{ch}_{s}")
    return names


# ── IMU feature extraction ─────────────────────────────────────────────────────

# Per-axis features for gyro/accel windows
_IMU_FEATURE_FUNCS = [
    ("mean", lambda x: float(np.mean(x))),
    ("std", lambda x: float(np.std(x))),
    ("min", lambda x: float(np.min(x))),
    ("max", lambda x: float(np.max(x))),
    ("rms", lambda x: float(np.sqrt(np.mean(x**2)))),
    ("abs_mean", lambda x: float(np.mean(np.abs(x)))),
    ("range", lambda x: float(np.max(x) - np.min(x))),
    ("diff_energy", lambda x: float(np.sum(np.diff(x) ** 2) / max(len(x) - 1, 1))),
]

N_IMU_FEATURES_PER_AXIS = len(_IMU_FEATURE_FUNCS)


def compute_imu_features(
    gyro_window: NDArray | None = None,
    accel_window: NDArray | None = None,
) -> NDArray:
    """Compute per-axis IMU features for a gyro and/or accelerometer window.

    Args:
        gyro_window: (N_gyro_channels, window_len) or None
        accel_window: (N_accel_channels, window_len) or None

    Returns:
        1-D ndarray of concatenated gyro features + accel features.
        Empty array if both are None.
    """
    feats: list[float] = []

    for imu_data, prefix in [(gyro_window, "gyro"), (accel_window, "accel")]:
        if imu_data is None:
            continue
        if imu_data.size == 0:
            continue
        if imu_data.ndim == 1:
            imu_data = imu_data.reshape(1, -1)
        for ch in range(imu_data.shape[0]):
            x = imu_data[ch]
            for _name, fn in _IMU_FEATURE_FUNCS:
                feats.append(fn(x))

    return np.array(feats, dtype=np.float64)


def compute_augmented_features(
    emg_window: NDArray,
    gyro_window: NDArray | None = None,
    accel_window: NDArray | None = None,
) -> NDArray:
    """Concatenate EMG features and IMU features for sklearn_imu backend.

    Args:
        emg_window: (N_channels, window_len)
        gyro_window: (N_gyro, window_len) or None
        accel_window: (N_accel, window_len) or None

    Returns:
        1-D ndarray: [EMG features | gyro features | accel features]
    """
    emg_feats = compute_features(emg_window)
    imu_feats = compute_imu_features(gyro_window, accel_window)
    return np.concatenate([emg_feats, imu_feats])


def imu_feature_names(n_gyro_channels: int = 3, n_accel_channels: int = 3) -> list[str]:
    """Return human-readable IMU feature names."""
    names: list[str] = []
    short = [name for name, _ in _IMU_FEATURE_FUNCS]
    for ch in range(n_gyro_channels):
        for s in short:
            names.append(f"gyro{ch}_{s}")
    for ch in range(n_accel_channels):
        for s in short:
            names.append(f"accel{ch}_{s}")
    return names
