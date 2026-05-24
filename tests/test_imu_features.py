"""Tests for IMU feature extraction and sklearn_imu training path."""

from __future__ import annotations

import tempfile
from pathlib import Path

import numpy as np
import pytest

from emg_bridge.features import (
    compute_augmented_features,
    compute_features,
    compute_imu_features,
    imu_feature_names,
)
from emg_bridge.config import WINDOW_LEN


# ── Feature shape tests ────────────────────────────────────────────────────────


def test_compute_features_shape():
    window = np.random.randn(8, WINDOW_LEN)
    feats = compute_features(window)
    assert feats.dtype == np.float64
    assert feats.shape == (48,)  # 8 channels × 6 features


def test_compute_imu_features_gyro_only():
    gyro = np.random.randn(3, WINDOW_LEN)
    feats = compute_imu_features(gyro_window=gyro)
    assert feats.shape == (3 * 8,)  # 3 channels × 8 features


def test_compute_imu_features_accel_only():
    accel = np.random.randn(3, WINDOW_LEN)
    feats = compute_imu_features(accel_window=accel)
    assert feats.shape == (3 * 8,)


def test_compute_imu_features_gyro_and_accel():
    gyro = np.random.randn(3, WINDOW_LEN)
    accel = np.random.randn(2, WINDOW_LEN)
    feats = compute_imu_features(gyro_window=gyro, accel_window=accel)
    assert feats.shape == (3 * 8 + 2 * 8,)  # 3 gyro + 2 accel axes × 8


def test_compute_imu_features_none_returns_empty():
    feats = compute_imu_features(gyro_window=None, accel_window=None)
    assert feats.shape == (0,)


def test_compute_imu_features_1d_window():
    gyro = np.random.randn(WINDOW_LEN)
    feats = compute_imu_features(gyro_window=gyro)
    assert feats.shape == (8,)  # 1 channel × 8 features


def test_augmented_features_shape():
    emg = np.random.randn(8, WINDOW_LEN)
    gyro = np.random.randn(3, WINDOW_LEN)
    accel = np.random.randn(3, WINDOW_LEN)
    feats = compute_augmented_features(emg, gyro_window=gyro, accel_window=accel)
    assert feats.shape == (48 + 24 + 24,)  # EMG + gyro + accel


def test_augmented_features_no_imu():
    emg = np.random.randn(8, WINDOW_LEN)
    feats = compute_augmented_features(emg, gyro_window=None, accel_window=None)
    assert feats.shape == (48,)
    np.testing.assert_array_equal(feats, compute_features(emg))


# ── IMU feature values tests ───────────────────────────────────────────────────


def test_imu_features_zero_input():
    gyro = np.zeros((3, WINDOW_LEN))
    feats = compute_imu_features(gyro_window=gyro)
    # All features should be 0 for zero input (except possibly diff_energy)
    assert np.allclose(feats[:3 * 7], 0.0)  # first 7 features per channel are mean/std/min/max/rms/abs/range


def test_imu_features_constant_input():
    gyro = np.ones((1, WINDOW_LEN)) * 5.0
    feats = compute_imu_features(gyro_window=gyro)
    # mean=5, std=0, min=5, max=5, rms=5, abs_mean=5, range=0, diff_energy=0
    assert feats[0] == pytest.approx(5.0)   # mean
    assert feats[1] == pytest.approx(0.0)   # std
    assert feats[2] == pytest.approx(5.0)   # min
    assert feats[3] == pytest.approx(5.0)   # max
    assert feats[4] == pytest.approx(5.0)   # rms
    assert feats[5] == pytest.approx(5.0)   # abs_mean
    assert feats[6] == pytest.approx(0.0)   # range
    assert feats[7] == pytest.approx(0.0)   # diff_energy


# ── Feature names ──────────────────────────────────────────────────────────────


def test_imu_feature_names():
    names = imu_feature_names(n_gyro_channels=3, n_accel_channels=3)
    assert len(names) == 48  # (3+3) * 8
    assert names[0] == "gyro0_mean"
    assert names[23] == "gyro2_diff_energy"
    assert names[24] == "accel0_mean"


# ── Backward compatibility ─────────────────────────────────────────────────────


def test_compute_features_unchanged():
    """EMG-only compute_features must return exactly the same shape as before."""
    window = np.random.randn(8, WINDOW_LEN)
    feats = compute_features(window)
    assert feats.shape == (48,)
    assert isinstance(feats, np.ndarray)
    # Spot-check a few values are finite
    assert np.all(np.isfinite(feats))


# ── IMU windowing alignment tests ──────────────────────────────────────────────


def test_imu_aligned_window_indices():
    """Simulate how train.py extracts aligned windows."""
    total_samples = 500
    window_len = 100
    step = 50
    gyro = np.random.randn(total_samples, 3)
    n_wins = (total_samples - window_len) // step + 1

    for i in range(n_wins):
        s = i * step
        e = s + window_len
        win = gyro[s:e].T
        feats = compute_imu_features(gyro_window=win)
        assert feats.shape == (24,)  # 3 gyro ch × 8
