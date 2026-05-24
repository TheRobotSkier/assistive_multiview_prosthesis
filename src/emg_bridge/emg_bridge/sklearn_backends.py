"""Pluggable gesture-classification backends for the EMG runtime.

Each backend implements a simple protocol:
    start()      — initialise (load models, filters, etc.)
    predict(emg_window, gyro_data, accel_data) → (label, confidence, probabilities, debug)
    emg_window   — property exposing the raw window for proportional control
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from emg_bridge import classifier as clf_mod
from emg_bridge.config import REST_LABEL, WINDOW_LEN
from emg_bridge.experiment_config import ExperimentConfig
from emg_bridge.features import compute_augmented_features, compute_features


# ── sklearn EMG-only backend ───────────────────────────────────────────────────


class SklearnEmgBackend:
    """Existing sklearn pipeline — EMG features only."""

    def __init__(self, config: ExperimentConfig) -> None:
        self._config = config
        self._pipe: Any = None
        self._meta: dict[str, Any] = {}
        self._emg_window: np.ndarray | None = None

    def start(self, model_dir: str | Path, threshold: float = 0.55) -> None:
        self._pipe = clf_mod.load(model_dir)
        meta_path = Path(model_dir) / "meta.pkl"
        if meta_path.exists():
            import joblib
            self._meta = joblib.load(meta_path) or {}

    def predict(
        self,
        emg_window: np.ndarray,
        gyro_data: np.ndarray | None = None,
        accel_data: np.ndarray | None = None,
    ) -> tuple[int, float, np.ndarray, dict[str, Any]]:
        self._emg_window = emg_window
        features = compute_features(emg_window)
        label, confidence, probs = clf_mod.predict(self._pipe, features)
        return label, confidence, probs, {"backend": "sklearn", "feature_set": "emg_only"}

    @property
    def emg_window(self) -> np.ndarray | None:
        return self._emg_window


# ── sklearn IMU backend ────────────────────────────────────────────────────────


class SklearnImuBackend:
    """Sklearn pipeline trained with EMG + IMU features."""

    def __init__(self, config: ExperimentConfig) -> None:
        self._config = config
        self._pipe: Any = None
        self._meta: dict[str, Any] = {}
        self._emg_window: np.ndarray | None = None

    def start(self, model_dir: str | Path, threshold: float = 0.55) -> None:
        self._pipe = clf_mod.load(model_dir)
        meta_path = Path(model_dir) / "meta.pkl"
        if meta_path.exists():
            import joblib
            self._meta = joblib.load(meta_path) or {}
        # Validate that the model was trained with IMU features
        feature_set = self._meta.get("feature_set", "")
        if feature_set != "emg_imu":
            raise ValueError(
                f"Model at {model_dir} was trained with feature_set='{feature_set}'. "
                "Expected 'emg_imu' for sklearn_imu backend. "
                "Re-train with --classifier-backend sklearn_imu."
            )
        if self._config.imu_features.movement_gate > 0:
            import logging
            logging.getLogger(__name__).info(
                "IMU movement gate: %s", self._config.imu_features.movement_gate
            )

    def predict(
        self,
        emg_window: np.ndarray,
        gyro_data: np.ndarray | None = None,
        accel_data: np.ndarray | None = None,
    ) -> tuple[int, float, np.ndarray, dict[str, Any]]:
        self._emg_window = emg_window

        # Optional movement gate: if gyro magnitude exceeds threshold, force REST
        gate = self._config.imu_features.movement_gate
        if gate > 0 and gyro_data is not None and gyro_data.size > 0:
            gyro_mag = np.sqrt(np.mean(np.square(gyro_data)))
            if gyro_mag > gate:
                from emg_bridge.config import N_GESTURES
                return REST_LABEL, 0.0, np.zeros(N_GESTURES), {
                    "backend": "sklearn_imu",
                    "feature_set": "emg_imu",
                    "movement_gated": True,
                }

        features = compute_augmented_features(
            emg_window,
            gyro_window=gyro_data,
            accel_window=accel_data,
        )
        label, confidence, probs = clf_mod.predict(self._pipe, features)
        return label, confidence, probs, {
            "backend": "sklearn_imu",
            "feature_set": "emg_imu",
        }

    @property
    def emg_window(self) -> np.ndarray | None:
        return self._emg_window
