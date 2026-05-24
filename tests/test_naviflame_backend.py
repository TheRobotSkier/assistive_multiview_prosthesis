"""Tests for emg_bridge.naviflame_backend — adapter, path resolution, gesture mapping."""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path
from typing import Any
from unittest import mock

import numpy as np
import pytest

# Mock tensorflow BEFORE importing the backend module so TF is never loaded.
_FAKE_TF = mock.MagicMock()
_FAKE_TF_MAGIC = mock.MagicMock()
sys.modules["tensorflow"] = _FAKE_TF_MAGIC
sys.modules["tensorflow.keras"] = _FAKE_TF_MAGIC
sys.modules["tensorflow.keras.models"] = _FAKE_TF_MAGIC
sys.modules["tensorflow.keras.layers"] = _FAKE_TF_MAGIC

# Because naviflame_backend checks _TF_AVAILABLE at import time, we need to reload
# the module after mocking tensorflow to get the right code path. However the
# module-level try/except runs on first import and sets _TF_AVAILABLE = False when
# the mock MagicMock raises on use. The key tests verify that the backend handles
# the unavailable case correctly. For tests that need TF available, we mock the
# module-level flags directly.
import importlib

import emg_bridge.naviflame_backend as _nb_mod


# ── Fixtures ──────────────────────────────────────────────────────────────────


@pytest.fixture
def tmp_config_json():
    """Create a temporary config.json file with valid NaviFlame paths."""
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        (root / "naviflame").mkdir()
        (root / "naviflame" / "models").mkdir()
        (root / "naviflame" / "models" / "og_fine_tune.h5").touch()
        (root / "naviflame" / "models" / "mlp_model.pkl").touch()
        (root / "naviflame" / "models" / "scaler.pkl").touch()

        config = {
            "data_path": "naviflame/data/recorded_gestures.pkl",
            "feature_extractor_path": "naviflame/models/og_fine_tune.h5",
            "mlp_model_path": "naviflame/models/mlp_model.pkl",
            "scaler_path": "naviflame/models/scaler.pkl",
            "gesture_image_path": "naviflame/gestures",
            "record": False,
            "fine_tune": False,
            "show_predicted_image": True,
            "send_to_socket": True,
        }
        config_path = root / "config.json"
        config_path.write_text(json.dumps(config))
        yield config_path


@pytest.fixture
def naviflame_config(tmp_config_json):
    """A NaviFlameConfig-like object pointing at a real config.json."""
    from emg_bridge.experiment_config import NaviFlameConfig

    return NaviFlameConfig(
        enabled=True,
        config_path=str(tmp_config_json),
        gesture_mapping={},
    )


# ── Path resolution ───────────────────────────────────────────────────────────


class TestPathResolution:
    def test_resolves_relative_paths_from_config_json(self, tmp_config_json):
        backend = _nb_mod.NaviFlameBackend(
            _fake_nf_config(config_path=str(tmp_config_json))
        )
        assert backend._feature_extractor_path != ""
        assert Path(backend._feature_extractor_path).exists()
        assert backend._mlp_model_path != ""
        assert Path(backend._mlp_model_path).exists()
        assert backend._scaler_path != ""
        assert Path(backend._scaler_path).exists()

    def test_empty_config_json_yields_empty_paths(self):
        with tempfile.TemporaryDirectory() as td:
            config_path = Path(td) / "config.json"
            config_path.write_text("{}")
            backend = _nb_mod.NaviFlameBackend(
                _fake_nf_config(config_path=str(config_path))
            )
        assert backend._feature_extractor_path == ""
        assert backend._mlp_model_path == ""
        assert backend._scaler_path == ""

    def test_missing_config_json_does_not_crash(self):
        backend = _nb_mod.NaviFlameBackend(
            _fake_nf_config(config_path="/nonexistent/config.json")
        )
        assert backend._feature_extractor_path == ""


# ── Gesture mapping ───────────────────────────────────────────────────────────


class TestGestureMapping:
    def test_default_mapping_is_identity_for_0_4(self):
        backend = _nb_mod.NaviFlameBackend(_fake_nf_config())
        # Default mapping: g0→0, g1→1, g2→2, g3→3, g4→4, g5→0, g6→0
        assert backend._mapping[0] == 0
        assert backend._mapping[1] == 1
        assert backend._mapping[2] == 2
        assert backend._mapping[3] == 3
        assert backend._mapping[4] == 4

    def test_unmapped_classes_default_to_rest(self):
        backend = _nb_mod.NaviFlameBackend(_fake_nf_config())
        assert backend._mapping[5] == 0
        assert backend._mapping[6] == 0

    def test_custom_mapping_overrides_defaults(self):
        backend = _nb_mod.NaviFlameBackend(
            _fake_nf_config(
                gesture_mapping={"0": 0, "1": 2, "2": 1}  # swap POWER and PINCH
            )
        )
        assert backend._mapping[0] == 0  # REST stays
        assert backend._mapping[1] == 2  # NaviFlame g1 → PINCH (instead of POWER)
        assert backend._mapping[2] == 1  # NaviFlame g2 → POWER
        # Unspecified classes use defaults
        assert backend._mapping[3] == 3

    def test_string_keys_parsed_as_ints(self):
        backend = _nb_mod.NaviFlameBackend(
            _fake_nf_config(gesture_mapping={"3": 0, "6": 2})
        )
        assert backend._mapping[3] == 0
        assert backend._mapping[6] == 2

    def test_map_to_project_space(self):
        backend = _nb_mod.NaviFlameBackend(_fake_nf_config())
        nf_probs = np.array([0.1, 0.6, 0.1, 0.1, 0.1, 0.0, 0.0], dtype=float)
        project = backend._map_to_project_space(nf_probs)
        assert project.shape == (5,)
        assert project[1] > 0.5  # class 1 → POWER (index 1)
        assert abs(float(project.sum()) - 1.0) < 1e-9

    def test_map_to_project_space_merge_duplicate_mappings(self):
        backend = _nb_mod.NaviFlameBackend(
            _fake_nf_config(gesture_mapping={"0": 0, "5": 0, "6": 0})
        )
        nf_probs = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.2, 0.3], dtype=float)
        project = backend._map_to_project_space(nf_probs)
        # Normalized: 0.6 / 0.6 = 1.0
        assert project[0] == pytest.approx(1.0)
        assert abs(float(project.sum()) - 1.0) < 1e-9


# ── Constructor / availability ────────────────────────────────────────────────


class TestConstructor:
    def test_constructs_without_crash(self):
        backend = _nb_mod.NaviFlameBackend(_fake_nf_config())
        assert backend is not None

    def test_start_raises_runtime_when_tf_unavailable(self):
        # Reload module with _TF_AVAILABLE forced to False
        mod = importlib.reload(_nb_mod)
        with mock.patch.object(mod, "_TF_AVAILABLE", False):
            backend = mod.NaviFlameBackend(_fake_nf_config())
            with pytest.raises(RuntimeError, match="tensorflow"):
                backend.start()

    def test_predict_raises_if_not_started(self):
        backend = _nb_mod.NaviFlameBackend(_fake_nf_config())
        backend._started = False
        window = np.zeros((8, 100), dtype=float)
        with pytest.raises(RuntimeError, match="start"):
            backend.predict(window)


# ── ExperimentConfig integration ──────────────────────────────────────────────


class TestExperimentConfigIntegration:
    def test_accepts_full_experiment_config(self):
        from emg_bridge.experiment_config import ExperimentConfig, NaviFlameConfig

        cfg = ExperimentConfig(
            classifier_backend="naviflame",
            naviflame=NaviFlameConfig(enabled=True, config_path="/tmp/x.json"),
        )
        backend = _nb_mod.NaviFlameBackend(cfg)
        assert backend._config_dir == Path("/tmp")

    def test_default_experiment_config_naviflame_works(self):
        from emg_bridge.experiment_config import ExperimentConfig

        cfg = ExperimentConfig()
        backend = _nb_mod.NaviFlameBackend(cfg)
        assert backend._mapping[0] == 0

    def test_accepts_naviflame_config_directly(self):
        backend = _nb_mod.NaviFlameBackend(
            _fake_nf_config(gesture_mapping={"1": 3})
        )
        assert backend._mapping[1] == 3


# ── Filter behaviour ──────────────────────────────────────────────────────────


class TestFilters:
    def test_filters_built_on_start(self):
        """Filters are initialised after a successful start()."""
        mod = importlib.reload(_nb_mod)
        fake_full = mock.MagicMock()
        fake_dense = mock.MagicMock()
        fake_full.get_layer.return_value = fake_dense

        nf_cfg = _fake_nf_config()

        with mock.patch.object(mod, "_TF_AVAILABLE", True), mock.patch.object(
            mod, "load_model", create=True, return_value=fake_full
        ), mock.patch.object(mod, "_MyMagnWarping", create=True), mock.patch.object(
            mod, "_MyScaling", create=True
        ), mock.patch.object(
            mod, "Model", create=True, return_value=fake_dense
        ), mock.patch.object(
            mod.Path, "exists", return_value=True
        ), mock.patch.object(
            mod.Path, "read_text", return_value=json.dumps({})
        ), mock.patch(
            "builtins.open", mock.mock_open()
        ), mock.patch(
            "pickle.load", return_value=mock.MagicMock()
        ):
            backend = mod.NaviFlameBackend(nf_cfg)
            backend._feature_extractor_path = "/fake/model.h5"
            backend._mlp_model_path = "/fake/mlp.pkl"
            backend._scaler_path = "/fake/scaler.pkl"

            backend.start()
            assert len(backend._filters) == 3
            mod.load_model.assert_called_once()
            fake_full.get_layer.assert_called_once_with("dense_8")

    def test_filters_process_without_error(self):
        filt = _nb_mod.BiquadMultiChan(8, _nb_mod.FilterTypes.bq_type_highpass, 4.5 / 500, 0.5, 0.0)
        out = filt.process(1.0, 3)
        assert isinstance(out, float)

    def test_filter_reset_clears_state(self):
        filt = _nb_mod.BiquadMultiChan(8, _nb_mod.FilterTypes.bq_type_lowpass, 100.0 / 500, 0.5, 0.0)
        filt.process(5.0, 0)
        filt.reset()
        assert all(z == 0.0 for z in filt.z1)
        assert all(z == 0.0 for z in filt.z2)


# ── Predict shape handling ────────────────────────────────────────────────────


class TestPredictShapeHandling:
    def test_predict_rejects_bad_shape(self):
        mod = importlib.reload(_nb_mod)
        with mock.patch.object(mod, "_TF_AVAILABLE", True), mock.patch.object(
            mod, "load_model", create=True
        ), mock.patch.object(mod, "_MyMagnWarping", create=True), mock.patch.object(
            mod, "_MyScaling", create=True
        ):
            backend = mod.NaviFlameBackend(_fake_nf_config())
            backend._started = True  # bypass start()
            with pytest.raises(ValueError, match="shape"):
                backend.predict(np.zeros((50, 8), dtype=float))

    def test_predict_accepts_channel_first(self):
        mod = importlib.reload(_nb_mod)
        with mock.patch.object(mod, "_TF_AVAILABLE", True), mock.patch.object(
            mod, "load_model", create=True
        ), mock.patch.object(mod, "_MyMagnWarping", create=True), mock.patch.object(
            mod, "_MyScaling", create=True
        ):
            backend = mod.NaviFlameBackend(_fake_nf_config())
            backend._started = True
            # batch not full → should return no_prediction without error
            from emg_bridge.config import N_GESTURES

            label, conf, probs, status = backend.predict(
                np.zeros((8, 100), dtype=float)
            )
            assert status == "no_prediction"
            assert label == 0
            assert probs.shape == (N_GESTURES,)

    def test_predict_accepts_channel_last(self):
        mod = importlib.reload(_nb_mod)
        with mock.patch.object(mod, "_TF_AVAILABLE", True):
            backend = mod.NaviFlameBackend(_fake_nf_config())
            backend._started = True
            from emg_bridge.config import N_GESTURES

            label, conf, probs, status = backend.predict(
                np.zeros((100, 8), dtype=float)
            )
            assert status == "no_prediction"
            assert probs.shape == (N_GESTURES,)

    def test_emg_window_property(self):
        backend = _nb_mod.NaviFlameBackend(_fake_nf_config())
        backend._last_raw_window = np.ones((8, 100), dtype=float)
        window = backend.emg_window
        assert window.shape == (8, 100)
        assert window[0, 0] == 1.0


# ── Movement gate ─────────────────────────────────────────────────────────────


class TestMovementGate:
    def test_movement_resets_buffer(self):
        mod = importlib.reload(_nb_mod)
        with mock.patch.object(mod, "_TF_AVAILABLE", True):
            backend = mod.NaviFlameBackend(_fake_nf_config())
            backend._started = True
            backend._inference_buffer.append("stale")

            gyro = np.full((3, 100), 1000.0)
            label, conf, probs, status = backend.predict(
                np.zeros((8, 100), dtype=float), gyro_data=gyro
            )
            assert status == "no_prediction"
            assert len(backend._inference_buffer) == 0

    def test_no_gyro_passes_through(self):
        mod = importlib.reload(_nb_mod)
        with mock.patch.object(mod, "_TF_AVAILABLE", True):
            backend = mod.NaviFlameBackend(_fake_nf_config())
            backend._started = True
            assert len(backend._inference_buffer) == 0
            backend.predict(np.zeros((8, 100), dtype=float), gyro_data=None)
            assert len(backend._inference_buffer) == 1


# ── Helpers ───────────────────────────────────────────────────────────────────


def _fake_nf_config(**overrides: Any):
    """Build a fake NaviFlameConfig-like object for tests that don't need TF."""
    defaults: dict[str, Any] = {
        "enabled": False,
        "config_path": "/prosthesis_ws/NaviFlame/config.json",
        "gesture_mapping": {},
    }
    defaults.update(overrides)
    # Use a simple namespace so we don't depend on the dataclass being imported
    return type("FakeNaviFlameConfig", (), defaults)()
