"""
NaviFlame gesture-classification backend adapter for the EMG project.

IMPORTANT COMPATIBILITY NOTE
----------------------------
NaviFlame requires tensorflow==2.12.0 and Python <3.11, but ROS 2 Jazzy
ships with Python 3.12.  These two cannot coexist in the same Python
interpreter.  When deploying this backend, run it inside a separate
container (e.g. a Docker container based on Python 3.10 with NaviFlame
dependencies) and communicate predictions over a socket or ROS bridge.

All TF-dependent imports are guarded with try/except.  Selecting the
naviflame backend when tensorflow is unavailable raises a clear
RuntimeError explaining the container requirement.
"""

from __future__ import annotations

import json
import logging
import math
import pickle
from enum import Enum
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from .config import GESTURE_NAMES, N_CHANNELS, N_GESTURES, REST_LABEL, SAMPLING_RATE, WINDOW_LEN

logger = logging.getLogger(__name__)

# ── Optional imports (TF-dependent) ──────────────────────────────────────────

_TF_AVAILABLE = False
_load_tf_error: str | None = None
try:
    import tensorflow as tf  # type: ignore
    from tensorflow.keras.layers import Layer  # type: ignore
    from tensorflow.keras.models import Model, load_model  # type: ignore

    _TF_AVAILABLE = True
except ImportError as e:
    _load_tf_error = str(e)

if _TF_AVAILABLE:

    class _MyMagnWarping(Layer):  # type: ignore[no-redef]
        """Dummy magnitude warping layer (from NaviFlame)."""

        def __init__(self, sigma: float = 0.0, divide: float = 0.0, **kwargs: Any) -> None:
            super().__init__(**kwargs)
            self.sigma = sigma
            self.divide = divide

        def call(self, x: Any) -> Any:
            return x

        def get_config(self) -> dict[str, Any]:
            config = super().get_config()
            config.update({"sigma": self.sigma, "divide": self.divide})
            return config

    class _MyScaling(Layer):  # type: ignore[no-redef]
        """Dummy scaling layer (from NaviFlame)."""

        def __init__(self, sigma: float = 0.0, **kwargs: Any) -> None:
            super().__init__(**kwargs)
            self.sigma = sigma

        def call(self, x: Any) -> Any:
            return x

        def get_config(self) -> dict[str, Any]:
            config = super().get_config()
            config.update({"sigma": self.sigma})
            return config


# ── Biquad filter (inlined from NaviFlame/naviflame/utils.py) ────────────────
#   Inlined because naviflame.utils imports tensorflow at module level,
#   which would prevent importing this module when TF is not installed.


class FilterTypes(Enum):
    bq_type_lowpass = 0
    bq_type_highpass = 1
    bq_type_bandpass = 2
    bq_type_notch = 3
    bq_type_peak = 4
    bq_type_lowshelf = 5
    bq_type_highshelf = 6


class BiquadMultiChan:
    """Per-channel biquad filter — replicated from NaviFlame."""

    def __init__(
        self,
        N: int,
        filter_type: FilterTypes,
        Fc: float,
        Q: float,
        peakGainDB: float,
    ) -> None:
        self.Nchan = N
        self.type = filter_type
        self.Fc = Fc
        self.Q = Q
        self.peakGain = peakGainDB
        self.a0: float = 0.0
        self.a1: float = 0.0
        self.a2: float = 0.0
        self.b1: float = 0.0
        self.b2: float = 0.0
        self.z1: list[float] = [0.0] * N
        self.z2: list[float] = [0.0] * N
        self.calc_biquad()

    def process(self, input_sample: float, Ichan: int) -> float:
        output = input_sample * self.a0 + self.z1[Ichan]
        self.z1[Ichan] = input_sample * self.a1 + self.z2[Ichan] - self.b1 * output
        self.z2[Ichan] = input_sample * self.a2 - self.b2 * output
        return output

    def calc_biquad(self) -> None:
        V = math.pow(10, abs(self.peakGain) / 20.0)
        K = math.tan(math.pi * self.Fc)
        norm: float

        if self.type == FilterTypes.bq_type_lowpass:
            norm = 1 / (1 + K / self.Q + K * K)
            self.a0 = K * K * norm
            self.a1 = 2 * self.a0
            self.a2 = self.a0
            self.b1 = 2 * (K * K - 1) * norm
            self.b2 = (1 - K / self.Q + K * K) * norm
        elif self.type == FilterTypes.bq_type_highpass:
            norm = 1 / (1 + K / self.Q + K * K)
            self.a0 = 1 * norm
            self.a1 = -2 * self.a0
            self.a2 = self.a0
            self.b1 = 2 * (K * K - 1) * norm
            self.b2 = (1 - K / self.Q + K * K) * norm
        elif self.type == FilterTypes.bq_type_notch:
            norm = 1 / (1 + K / self.Q + K * K)
            self.a0 = (1 + K * K) * norm
            self.a1 = 2 * (K * K - 1) * norm
            self.a2 = self.a0
            self.b1 = self.a1
            self.b2 = (1 - K / self.Q + K * K) * norm
        else:
            raise ValueError(f"Unsupported filter type: {self.type}")

    def reset(self) -> None:
        self.z1 = [0.0] * self.Nchan
        self.z2 = [0.0] * self.Nchan


# ── Defaults ─────────────────────────────────────────────────────────────────

_NAVIFLAME_DEFAULTS = {
    "highpass_hz": 4.5,
    "notch_hz": 50.0,
    "lowpass_hz": 100.0,
    "gyro_threshold": 500,
    "prediction_threshold": 0.4,
    "batch_size": 5,
    "model_input_len": 100,
}

_DEFAULT_GESTURE_MAPPING: dict[int, int] = {
    0: 0,  # g0 (rest)     → REST
    1: 1,  # g1 (power)    → POWER
    2: 2,  # g2 (open)     → PINCH
    3: 3,  # g3 (flexion)  → OPEN
    4: 4,  # g4 (extension)→ POINT
    5: 0,  # g5 → REST
    6: 0,  # g6 → REST
}


# ── Backend ──────────────────────────────────────────────────────────────────


class NaviFlameBackend:
    """Gesture-classification backend wrapping NaviFlame deep model.

    Constructor receives the full ExperimentConfig; it reads the
    ``naviflame`` section for paths, filter parameters, and gesture
    mapping overrides.

    If tensorflow is not installed, construction succeeds but
    :meth:`start` raises ``RuntimeError`` with a descriptive message.
    """

    def __init__(self, config: Any = None) -> None:
        # Support both ExperimentConfig and NaviFlameConfig for testing flexibility
        try:
            from .experiment_config import ExperimentConfig

            if isinstance(config, ExperimentConfig):
                nf = config.naviflame
            else:
                nf = config
        except ImportError:
            nf = config

        # ── Resolve config.json ──────────────────────────────────────────────
        config_path = getattr(nf, "config_path", "/prosthesis_ws/NaviFlame/config.json")
        self._config_dir = Path(config_path).parent
        nf_config: dict[str, Any] = {}
        if Path(config_path).exists():
            nf_config = json.loads(Path(config_path).read_text(encoding="utf-8"))

        def _abs(key: str) -> str:
            rel = nf_config.get(key, "")
            return str((self._config_dir / rel).resolve()) if rel else ""

        self._feature_extractor_path = _abs("feature_extractor_path")
        self._mlp_model_path = _abs("mlp_model_path")
        self._scaler_path = _abs("scaler_path")

        # ── Filter parameters ────────────────────────────────────────────────
        self._highpass_hz = _NAVIFLAME_DEFAULTS["highpass_hz"]
        self._notch_hz = _NAVIFLAME_DEFAULTS["notch_hz"]
        self._lowpass_hz = _NAVIFLAME_DEFAULTS["lowpass_hz"]

        # ── Inference parameters ─────────────────────────────────────────────
        self._gyro_threshold = _NAVIFLAME_DEFAULTS["gyro_threshold"]
        self._prediction_threshold = _NAVIFLAME_DEFAULTS["prediction_threshold"]
        self._batch_size = _NAVIFLAME_DEFAULTS["batch_size"]
        self._model_input_len = _NAVIFLAME_DEFAULTS["model_input_len"]

        # ── Gesture mapping ──────────────────────────────────────────────────
        raw_mapping: dict = getattr(nf, "gesture_mapping", {}) or {}
        self._mapping: dict[int, int] = dict(_DEFAULT_GESTURE_MAPPING)
        for k, v in raw_mapping.items():
            self._mapping[int(k)] = int(v)

        # ── State ────────────────────────────────────────────────────────────
        self._filters: list[BiquadMultiChan] = []
        self._feature_extractor: Any = None
        self._mlp_model: Any = None
        self._scaler: Any = None
        self._started: bool = False
        self._batch_buffer: list[NDArray] = []
        self._inference_buffer: list[NDArray] = []
        self._last_raw_window: NDArray = np.zeros((N_CHANNELS, WINDOW_LEN), dtype=float)
        self._last_filtered_window: NDArray = np.zeros((N_CHANNELS, WINDOW_LEN), dtype=float)

    # ── Public API ───────────────────────────────────────────────────────────

    @property
    def emg_window(self) -> NDArray:
        """Most recent raw EMG window (N_CHANNELS, WINDOW_LEN) for proportional control."""
        return self._last_raw_window.copy()

    @property
    def is_available(self) -> bool:
        """True if tensorflow is importable."""
        return _TF_AVAILABLE

    def start(self) -> None:
        """Initialise filters, load TF model & MLP, run warm-up inference.

        Raises:
            RuntimeError: if tensorflow is not installed.
            FileNotFoundError: if a model or scaler path does not exist.
        """
        if not _TF_AVAILABLE:
            raise RuntimeError(
                "NaviFlame backend requires tensorflow, which is not installed. "
                "Run NaviFlame in a separate container (Python <3.11, tensorflow==2.12.0) "
                "and communicate predictions over a socket or bridge. "
                f"(Import error: {_load_tf_error})"
            )

        # ── Build filters ────────────────────────────────────────────────────
        self._filters = [
            BiquadMultiChan(
                N_CHANNELS,
                FilterTypes.bq_type_highpass,
                self._highpass_hz / SAMPLING_RATE,
                0.5,
                0.0,
            ),
            BiquadMultiChan(
                N_CHANNELS,
                FilterTypes.bq_type_notch,
                self._notch_hz / SAMPLING_RATE,
                4.0,
                0.0,
            ),
            BiquadMultiChan(
                N_CHANNELS,
                FilterTypes.bq_type_lowpass,
                self._lowpass_hz / SAMPLING_RATE,
                0.5,
                0.0,
            ),
        ]

        # ── Load models ──────────────────────────────────────────────────────
        if not self._feature_extractor_path or not Path(self._feature_extractor_path).exists():
            raise FileNotFoundError(
                f"Feature extractor not found: {self._feature_extractor_path}"
            )
        if not self._mlp_model_path or not Path(self._mlp_model_path).exists():
            raise FileNotFoundError(f"MLP model not found: {self._mlp_model_path}")
        if not self._scaler_path or not Path(self._scaler_path).exists():
            raise FileNotFoundError(f"Scaler not found: {self._scaler_path}")

        custom_objects = {"MyMagnWarping": _MyMagnWarping, "MyScaling": _MyScaling}
        full_model = load_model(self._feature_extractor_path, custom_objects=custom_objects)
        try:
            self._feature_extractor = Model(
                inputs=full_model.input,
                outputs=full_model.get_layer("dense_8").output,
            )
        except (ValueError, AttributeError) as e:
            raise RuntimeError(
                f"Failed to extract 'dense_8' layer from feature extractor model: {e}"
            )

        with open(self._scaler_path, "rb") as f:
            self._scaler = pickle.load(f)  # type: ignore[no-untyped-call]
        with open(self._mlp_model_path, "rb") as f:
            self._mlp_model = pickle.load(f)  # type: ignore[no-untyped-call]

        # ── Warm-up the TF model ─────────────────────────────────────────────
        dummy = np.zeros((1, N_CHANNELS, self._model_input_len, 1), dtype=np.float32)
        for _ in range(3):
            self._feature_extractor.predict(dummy, verbose=0)

        self._started = True
        logger.info("NaviFlame backend started (model_input_len=%d, batch_size=%d)",
                     self._model_input_len, self._batch_size)

    def predict(
        self,
        emg_window: NDArray,
        gyro_data: NDArray | None = None,
    ) -> tuple[int, float, NDArray, str]:
        """Predict gesture from an EMG window.

        Args:
            emg_window: shape (N_CHANNELS, WINDOW_LEN) — raw EMG samples.
            gyro_data: shape (3, WINDOW_LEN) or None — gyroscope channels.

        Returns:
            (label, confidence, probabilities, status)
            - label: mapped gesture index (REST if confidence < threshold).
            - confidence: probability of the predicted class.
            - probabilities: full length-N_GESTURES posterior vector.
            - status: "ok" when a prediction was produced, "no_prediction"
              while the batch is filling or below threshold.

        Raises:
            RuntimeError: if :meth:`start` was not called or TF is unavailable.
        """
        if not self._started:
            raise RuntimeError("NaviFlameBackend.start() must be called before predict()")

        self._last_raw_window = emg_window.copy()

        # ── Movement gate ────────────────────────────────────────────────────
        if gyro_data is not None and np.any(np.abs(gyro_data) > self._gyro_threshold):
            self._batch_buffer.clear()
            self._inference_buffer.clear()
            logger.debug("Movement detected — resetting batch buffer")
            return (REST_LABEL, 0.0, np.zeros(N_GESTURES, dtype=float), "no_prediction")

        # ── Shape normalization: accept (N_CHANNELS, WINDOW_LEN) or (WINDOW_LEN, N_CHANNELS) ──
        if emg_window.shape == (WINDOW_LEN, N_CHANNELS):
            emg_data = emg_window.T  # → (N_CHANNELS, WINDOW_LEN)
        elif emg_window.shape == (N_CHANNELS, WINDOW_LEN):
            emg_data = emg_window
        else:
            raise ValueError(
                f"Expected emg_window shape ({N_CHANNELS}, {WINDOW_LEN}) or "
                f"({WINDOW_LEN}, {N_CHANNELS}), got {emg_window.shape}"
            )

        # ── Apply NaviFlame filters (in-place on copy) ───────────────────────
        filtered = emg_data.copy()  # (N_CHANNELS, WINDOW_LEN)
        for ch in range(N_CHANNELS):
            for filt in self._filters:
                for t in range(filtered.shape[1]):
                    filtered[ch, t] = filt.process(filtered[ch, t], ch)

        self._last_filtered_window = filtered.copy()

        # ── Build TF input ───────────────────────────────────────────────────
        input_tensor = np.expand_dims(filtered, axis=2).astype(np.float32)  # (8, 100, 1)
        input_tensor = np.expand_dims(input_tensor, axis=0)  # (1, 8, 100, 1)

        self._inference_buffer.append(input_tensor)

        if len(self._inference_buffer) < self._batch_size:
            return (REST_LABEL, 0.0, np.zeros(N_GESTURES, dtype=float), "no_prediction")

        # ── Run batch inference ──────────────────────────────────────────────
        batch = np.concatenate(self._inference_buffer, axis=0)  # (batch_size, 8, 100, 1)
        self._inference_buffer.clear()
        self._batch_buffer.clear()

        features = self._feature_extractor.predict(batch, verbose=0)
        features_scaled = self._scaler.transform(features)
        raw_probs = self._mlp_model.predict_proba(features_scaled)
        avg_probs = np.mean(raw_probs, axis=0)  # shape (num_naviflame_classes,)

        final_output = int(np.argmax(avg_probs))
        confidence = float(avg_probs[final_output])

        project_probs = self._map_to_project_space(avg_probs)

        if confidence >= self._prediction_threshold:
            label = self._mapping.get(final_output, REST_LABEL)
            project_probs[label] = max(project_probs[label], confidence)
            return (label, confidence, project_probs, "ok")
        else:
            return (REST_LABEL, confidence, project_probs, "no_prediction")

    # ── Internals ────────────────────────────────────────────────────────────

    def _map_to_project_space(self, nf_probs: NDArray) -> NDArray:
        """Map NaviFlame class probabilities (length 7) to project gesture space."""
        out = np.zeros(N_GESTURES, dtype=float)
        for nf_class, weight in enumerate(nf_probs):
            mapped = self._mapping.get(nf_class, REST_LABEL)
            out[mapped] += weight
        total = float(out.sum())
        if total > 0:
            out /= total
        return out
