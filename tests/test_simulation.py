"""Simulation test — feed deterministic windows through the full pipeline.

Verifies that the experiment config, backend selection, stabilizer, and
limiter all chain together correctly without MindRove hardware.
"""

from __future__ import annotations

import numpy as np
import tempfile
from pathlib import Path

import pytest

from emg_bridge.config import GESTURE_NAMES, N_CHANNELS, N_GESTURES, WINDOW_LEN
from emg_bridge.experiment_config import ExperimentConfig, ProportionalSlewConfig, GestureStabilityConfig
from emg_bridge.control_filters import ProportionalLimiter
from emg_bridge.gesture_stabilizer import GestureStabilizer
from emg_bridge.sklearn_backends import SklearnEmgBackend


def _make_dummy_model(model_dir: Path) -> None:
    import joblib
    from emg_bridge import classifier as clf_mod
    np.random.seed(42)
    X = np.random.randn(200, 48)
    y = np.random.randint(0, 5, 200)
    clf_mod.train(X, y, model_dir=model_dir, cv_folds=3, verbose=False)


class TestFullPipeline:
    def test_disabled_pipeline_preserves_raw_values(self):
        """With all experiments off, output should match input label exactly."""
        with tempfile.TemporaryDirectory() as tmpdir:
            model_dir = Path(tmpdir)
            _make_dummy_model(model_dir)
            backend = SklearnEmgBackend(ExperimentConfig())
            backend.start(model_dir)

            stabilizer = GestureStabilizer(GestureStabilityConfig())
            limiter = ProportionalLimiter(ProportionalSlewConfig())

            window = np.random.randn(N_CHANNELS, WINDOW_LEN)
            label, conf, probs, _ = backend.predict(window)
            stab_label, _ = stabilizer.update(label, conf, probs, 0.1)
            assert stab_label == label  # pass-through

            raw_prop = 0.5
            limited = limiter.step(raw_prop, 0.1, label)
            assert limited == pytest.approx(raw_prop)  # pass-through

    def test_stabilizer_holds_low_confidence(self):
        """Stabilizer should prevent switches on low confidence."""
        stabilizer = GestureStabilizer(GestureStabilityConfig(
            enabled=True,
            min_confidence_to_switch=0.8,
            min_frames=3,
            release_behavior="require_threshold",
        ))
        probs = np.zeros(N_GESTURES)
        probs[0] = 0.9  # REST
        result, _ = stabilizer.update(0, 0.9, probs, 0.1)
        assert result == 0  # starts at rest

        # Try to switch with low confidence → should hold
        probs2 = np.zeros(N_GESTURES)
        probs2[1] = 0.5  # POWER at low confidence
        result, _ = stabilizer.update(1, 0.5, probs2, 0.1)
        assert result == 0  # held

        # Repeat to build up frames — but confidence still too low
        for _ in range(5):
            result, _ = stabilizer.update(1, 0.5, probs2, 0.1)
        assert result == 0  # still held — confidence never met threshold

    def test_stabilizer_switches_after_min_frames(self):
        """Stabilizer should switch after N consecutive high-confidence frames."""
        stabilizer = GestureStabilizer(GestureStabilityConfig(
            enabled=True,
            min_confidence_to_switch=0.6,
            min_frames=2,
        ))
        probs = np.zeros(N_GESTURES)
        probs[1] = 0.9
        # First frame: candidate counter starts
        label, _ = stabilizer.update(1, 0.9, probs, 0.1)
        assert label == 0  # not switched yet (first frame)
        # Second frame: min_frames met → switch
        label, _ = stabilizer.update(1, 0.9, probs, 0.1)
        assert label == 1  # switched

    def test_limiter_ramps_smoothly(self):
        """Limiter should smoothly ramp proportional to target."""
        limiter = ProportionalLimiter(ProportionalSlewConfig(
            enabled=True,
            max_velocity_per_s=2.0,
            initial_value=0.0,
        ))
        # Step from 0 to 1.0 over several frames
        val = limiter.step(1.0, 0.05, 1)  # dt=50ms, max delta = 2.0*0.05 = 0.1
        assert val == pytest.approx(0.1)
        val = limiter.step(1.0, 0.05, 1)
        assert val == pytest.approx(0.2)
        val = limiter.step(1.0, 0.05, 1)
        assert val == pytest.approx(0.3)

    def test_limiter_reset_on_rest(self):
        """Limiter should reset internal state when gesture is REST."""
        limiter = ProportionalLimiter(ProportionalSlewConfig(
            enabled=True,
            max_velocity_per_s=2.0,
            reset_on_rest=True,
        ))
        val = limiter.step(1.0, 0.05, 1)  # non-REST
        assert val > 0.0
        val = limiter.step(1.0, 0.05, 0)  # REST → reset
        assert val == pytest.approx(0.0)

    def test_end_to_end_backend_stabilizer_limiter(self):
        """Feed deterministic windows through the full chain."""
        with tempfile.TemporaryDirectory() as tmpdir:
            model_dir = Path(tmpdir)
            _make_dummy_model(model_dir)

            config = ExperimentConfig(
                classifier_backend="sklearn",
                proportional_slew=ProportionalSlewConfig(
                    enabled=True,
                    max_velocity_per_s=3.0,
                    reset_on_rest=True,
                ),
                gesture_stability=GestureStabilityConfig(
                    enabled=True,
                    min_confidence_to_switch=0.5,
                    min_frames=2,
                    release_behavior="allow_rest_immediately",
                ),
            )
            backend = SklearnEmgBackend(config)
            backend.start(model_dir)
            stabilizer = GestureStabilizer(config.gesture_stability)
            limiter = ProportionalLimiter(config.proportional_slew)

            # Simulate several prediction frames
            window = np.random.randn(N_CHANNELS, WINDOW_LEN)
            last_label = 0
            for i in range(10):
                label, conf, probs, _ = backend.predict(window)
                stab_label, debug = stabilizer.update(label, conf, probs, 0.1)
                raw_prop = 0.5 if stab_label != 0 else 0.0
                limited = limiter.step(raw_prop, 0.1, stab_label)
                assert 0.0 <= limited <= 1.0
                last_label = stab_label

            # Final label should be valid
            assert last_label in range(N_GESTURES)
