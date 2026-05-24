"""Tests for SklearnEmgBackend and SklearnImuBackend."""

from __future__ import annotations

import tempfile
from pathlib import Path

import numpy as np
import pytest

from emg_bridge.experiment_config import ExperimentConfig
from emg_bridge.sklearn_backends import SklearnEmgBackend, SklearnImuBackend
from emg_bridge.config import WINDOW_LEN, N_CHANNELS, GESTURE_NAMES, N_GESTURES


# ── Helpers ────────────────────────────────────────────────────────────────────


@pytest.fixture
def dummy_model_dir():
    """Create a minimal trained sklearn model for testing."""
    with tempfile.TemporaryDirectory() as tmpdir:
        model_dir = Path(tmpdir)
        _train_dummy_model(model_dir, feature_set="emg_only")
        yield model_dir


@pytest.fixture
def dummy_imu_model_dir():
    with tempfile.TemporaryDirectory() as tmpdir:
        model_dir = Path(tmpdir)
        _train_dummy_model(model_dir, feature_set="emg_imu")
        yield model_dir


def _train_dummy_model(model_dir: Path, feature_set: str = "emg_only") -> None:
    import joblib
    from emg_bridge import classifier as clf_mod

    n_features = 48 if feature_set == "emg_only" else 48 + 24  # EMG + gyro
    np.random.seed(42)
    X = np.random.randn(200, n_features)
    y = np.random.randint(0, 5, 200)
    clf_mod.train(
        X, y,
        model_dir=model_dir,
        cv_folds=3,
        verbose=False,
        feature_set=feature_set,
    )


# ── SklearnEmgBackend ──────────────────────────────────────────────────────────


class TestSklearnEmgBackend:
    def test_constructs(self):
        b = SklearnEmgBackend(ExperimentConfig())
        assert b is not None

    def test_start_loads_model(self, dummy_model_dir):
        b = SklearnEmgBackend(ExperimentConfig())
        b.start(dummy_model_dir)
        assert b._pipe is not None

    def test_predict_returns_tuple(self, dummy_model_dir):
        b = SklearnEmgBackend(ExperimentConfig())
        b.start(dummy_model_dir)
        window = np.random.randn(N_CHANNELS, WINDOW_LEN)
        label, conf, probs, debug = b.predict(window)
        assert isinstance(label, (int, np.integer))
        assert 0 <= label < 5
        assert 0.0 <= conf <= 1.0
        assert probs.shape == (N_GESTURES,)
        assert debug["backend"] == "sklearn"
        assert debug["feature_set"] == "emg_only"

    def test_emg_window_property(self, dummy_model_dir):
        b = SklearnEmgBackend(ExperimentConfig())
        b.start(dummy_model_dir)
        window = np.random.randn(N_CHANNELS, WINDOW_LEN)
        b.predict(window)
        assert b.emg_window is not None
        assert b.emg_window.shape == (N_CHANNELS, WINDOW_LEN)

    def test_ignores_spurious_imu(self, dummy_model_dir):
        b = SklearnEmgBackend(ExperimentConfig())
        b.start(dummy_model_dir)
        window = np.random.randn(N_CHANNELS, WINDOW_LEN)
        gyro = np.random.randn(3, WINDOW_LEN)
        label, _, _, _ = b.predict(window, gyro_data=gyro)
        assert isinstance(label, (int, np.integer))


# ── SklearnImuBackend ──────────────────────────────────────────────────────────


class TestSklearnImuBackend:
    def test_constructs(self):
        b = SklearnImuBackend(ExperimentConfig())
        assert b is not None

    def test_start_loads_imu_model(self, dummy_imu_model_dir):
        b = SklearnImuBackend(ExperimentConfig())
        b.start(dummy_imu_model_dir)
        assert b._pipe is not None

    def test_predict_with_imu(self, dummy_imu_model_dir):
        b = SklearnImuBackend(ExperimentConfig())
        b.start(dummy_imu_model_dir)
        window = np.random.randn(N_CHANNELS, WINDOW_LEN)
        gyro = np.random.randn(3, WINDOW_LEN)
        label, conf, probs, debug = b.predict(window, gyro_data=gyro)
        assert 0 <= label < 5
        assert debug["backend"] == "sklearn_imu"

    def test_predict_no_imu_data_raises(self, dummy_imu_model_dir):
        """IMU model needs IMU data — feature dim mismatch is expected."""
        b = SklearnImuBackend(ExperimentConfig())
        b.start(dummy_imu_model_dir)
        window = np.random.randn(N_CHANNELS, WINDOW_LEN)
        with pytest.raises(ValueError, match="features"):
            b.predict(window, gyro_data=None, accel_data=None)

    def test_rejects_emg_only_model(self, dummy_model_dir):
        b = SklearnImuBackend(ExperimentConfig())
        with pytest.raises(ValueError, match="feature_set"):
            b.start(dummy_model_dir)

    def test_movement_gate(self, dummy_imu_model_dir):
        from emg_bridge.experiment_config import ImuFeaturesConfig

        exp_config = ExperimentConfig(
            classifier_backend="sklearn_imu",
            imu_features=ImuFeaturesConfig(movement_gate=10.0),
        )
        b = SklearnImuBackend(exp_config)
        b.start(dummy_imu_model_dir)
        window = np.random.randn(N_CHANNELS, WINDOW_LEN)
        # Large gyro → should be movement-gated to REST
        gyro = np.ones((3, WINDOW_LEN)) * 100.0
        label, conf, _, debug = b.predict(window, gyro_data=gyro)
        assert label == 0  # REST
        assert debug.get("movement_gated") is True

    def test_movement_gate_passes_quiet_gyro(self, dummy_imu_model_dir):
        from emg_bridge.experiment_config import ImuFeaturesConfig

        exp_config = ExperimentConfig(
            classifier_backend="sklearn_imu",
            imu_features=ImuFeaturesConfig(movement_gate=1000.0),
        )
        b = SklearnImuBackend(exp_config)
        b.start(dummy_imu_model_dir)
        window = np.random.randn(N_CHANNELS, WINDOW_LEN)
        gyro = np.ones((3, WINDOW_LEN)) * 5.0  # below gate
        label, _, _, debug = b.predict(window, gyro_data=gyro)
        assert "movement_gated" not in debug or not debug.get("movement_gated")
