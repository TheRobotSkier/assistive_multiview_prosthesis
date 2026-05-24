#!/usr/bin/env python3
"""
Standalone NaviFlame runner — recording, fine-tuning, and real-time inference.

Uses NaviFlame's own 7 gestures (g0-g6), its own models and filters.
Runs in its own Python 3.10 container (tensorflow==2.12.0, no ROS).

Usage:
    python scripts/run_naviflame.py                              # inference only
    python scripts/run_naviflame.py --train                      # record + fine-tune + infer
    python scripts/run_naviflame.py --train --record-only         # record only
    python scripts/run_naviflame.py --train --fine-tune-only      # fine-tune only
"""

from __future__ import annotations

import argparse
import json
import os
import pickle
import sys
import time
from collections import deque
from pathlib import Path
from threading import Event, Thread
from queue import Queue

import numpy as np

# ── ANSI helpers ──────────────────────────────────────────────────────────────

def _bold(s: str) -> str:   return f"\033[1m{s}\033[0m"
def _green(s: str) -> str:  return f"\033[92m{s}\033[0m"
def _yellow(s: str) -> str: return f"\033[93m{s}\033[0m"
def _cyan(s: str) -> str:   return f"\033[96m{s}\033[0m"
def _red(s: str) -> str:    return f"\033[91m{s}\033[0m"
def _dim(s: str) -> str:    return f"\033[2m{s}\033[0m"

_UP = "\033[A"
_CLEAR_LINE = "\033[2K"
_DISPLAY_LINES = 10

# ── NaviFlame gesture names ───────────────────────────────────────────────────

NAVIFLAME_GESTURES = ["g0:Rest", "g1", "g2", "g3", "g4", "g5", "g6"]
N_NF_GESTURES = len(NAVIFLAME_GESTURES)

def _resolve_path(config: dict, key: str, default_rel: str) -> str:
    """Resolve a path from config, making it absolute relative to NF_BASE."""
    val = config.get(key, "")
    if not val:
        val = str(NF_BASE / default_rel)
    elif not os.path.isabs(val):
        val = str(NF_BASE / val)
    return val


# ── Default paths (relative to /prosthesis_ws) ────────────────────────────────

NF_BASE = Path("/prosthesis_ws/NaviFlame")
NF_CONFIG_JSON = NF_BASE / "config.json"
NF_DATA_PKL = NF_BASE / "naviflame" / "data" / "recorded_gestures.pkl"


# ── Terminal display ──────────────────────────────────────────────────────────

def _conf_bar(val: float, width: int = 15) -> str:
    filled = int(val * width)
    bar = "\u2588" * filled + "\u00b7" * (width - filled)
    return f"[{bar}] {val * 100:.0f}%"

def _prop_bar(val: float, width: int = 20) -> str:
    filled = int(val * width)
    bar = "\u2593" * filled + "\u2591" * (width - filled)
    return f"[{bar}] {val:.2f}"


_first_draw = True

def _draw(label: int, confidence: float, proportional: float, probs: np.ndarray, frame_count: int, fps: float) -> None:
    global _first_draw
    if not _first_draw:
        sys.stdout.write((_UP + _CLEAR_LINE) * _DISPLAY_LINES)

    gesture = NAVIFLAME_GESTURES[label] if label < N_NF_GESTURES else f"g{label}"
    rest_active = label == 0

    sys.stdout.write(_bold("\u2500" * 52) + "\n")
    sys.stdout.write(
        f"  {_bold('NaviFlame')} : "
        + (_dim(gesture) if rest_active else _green(_bold(gesture)))
        + f"  {_dim(f'(frame {frame_count})')}\n"
    )
    sys.stdout.write(f"  {_bold('Confidence')}: {_conf_bar(confidence)}\n")
    sys.stdout.write(f"  {_bold('Prop. ctrl')}: {_prop_bar(proportional)}\n")
    prob_parts = []
    for i, name in enumerate(NAVIFLAME_GESTURES):
        pct = f"{probs[i] * 100:.0f}%"
        s = f"{name}:{pct}"
        if i == label:
            s = _green(s)
        else:
            s = _dim(s)
        prob_parts.append(s)
    sys.stdout.write("  " + "  ".join(prob_parts[:4]) + "\n")
    sys.stdout.write("  " + "  ".join(prob_parts[4:]) + "\n")
    sys.stdout.write(_bold("\u2500" * 52) + "\n")
    sys.stdout.write(_dim(f"  {fps:.1f} Hz  |  Ctrl-C to quit") + "\n")
    sys.stdout.flush()
    _first_draw = False


# ── Proportional control (RMS-based, same as project) ──────────────────────────

def compute_rms_proportional(emg_window: np.ndarray) -> float:
    """Map window RMS to [0, 1] proportional control value."""
    rms_per_ch = np.sqrt(np.mean(emg_window**2, axis=1))
    rms = float(np.mean(rms_per_ch))
    GLOBAL_RMS_MAX = 500.0
    return float(np.clip(rms / GLOBAL_RMS_MAX, 0.0, 1.0))


# ── Recording ──────────────────────────────────────────────────────────────────

def _run_recording(config: dict) -> None:
    """Run NaviFlame's record_gestures — collects EMG for all 7 gestures."""
    print(f"\n{_cyan('=== Phase 1: Recording gestures ===')}")
    print(f"  Gestures: {', '.join(NAVIFLAME_GESTURES)}")
    print(f"  Recording time: 8 s per gesture")
    print(f"  Total: 7 gestures \u00d7 8 s = 56 s\n")

    # Setup NaviFlame path
    nf_path = str(NF_BASE)
    if nf_path not in sys.path:
        sys.path.insert(0, nf_path)

    from naviflame.record import record_gestures
    from naviflame.utils import BiquadMultiChan, FilterTypes

    fs = 500.0
    filters = [
        BiquadMultiChan(8, FilterTypes.bq_type_highpass, 4.5 / fs, 0.5, 0.0),
        BiquadMultiChan(8, FilterTypes.bq_type_notch, 50.0 / fs, 4.0, 0.0),
        BiquadMultiChan(8, FilterTypes.bq_type_lowpass, 100.0 / fs, 0.5, 0.0),
    ]

    data_path = str(NF_DATA_PKL)
    gesture_image_path = str(NF_BASE / "naviflame" / "gestures")

    os.makedirs(os.path.dirname(data_path), exist_ok=True)

    record_gestures(
        filters=filters,
        data_path=data_path,
        gesture_image_path=gesture_image_path,
        skip_gestures=[],
        gestures_repeat=1,
        recording_time_sec=8,
        sampling_rate=500,
        model_input_len=100,
        overlap_frac=10,
    )
    print(f"{_green('Recording saved to')} {data_path}\n")


# ── Fine-tuning ────────────────────────────────────────────────────────────────

def _run_fine_tuning(config: dict) -> None:
    """Run NaviFlame's fine_tune_model — trains MLP head on recorded data."""
    nf_path = str(NF_BASE)
    if nf_path not in sys.path:
        sys.path.insert(0, nf_path)

    print(f"\n{_cyan('=== Phase 2: Fine-tuning MLP classifier ===')}")

    from naviflame.fine_tune import fine_tune_model

    data_path = str(NF_DATA_PKL)
    if not os.path.exists(data_path):
        print(_red(f"No recorded data at {data_path}. Run --record-only first."))
        sys.exit(1)

    feat_path = _resolve_path(config, "feature_extractor_path", "naviflame/models/og_fine_tune.h5")
    mlp_path = _resolve_path(config, "mlp_model_path", "naviflame/models/mlp_model.pkl")
    scaler_path = _resolve_path(config, "scaler_path", "naviflame/models/scaler.pkl")

    with open(data_path, "rb") as f:
        recorded_data, recorded_labels = pickle.load(f)

    print(f"  Data: {len(recorded_data)} windows, {len(set(recorded_labels))} classes")
    print(f"  Feature extractor: {feat_path}")

    _scaler, accuracy = fine_tune_model(
        feature_extractor_path=feat_path,
        recorded_data=recorded_data,
        recorded_labels=recorded_labels,
        mlp_model_path=mlp_path,
        scaler_path=scaler_path,
    )
    print(f"{_green(f'Fine-tuning complete — validation accuracy: {accuracy*100:.1f}%')}")
    print(f"{_green('Models saved to')} {mlp_path}\n")


# ── Inference ──────────────────────────────────────────────────────────────────

def _run_inference(config: dict) -> None:
    """Run NaviFlame real-time inference with terminal display.

    real_time_inference() manages its own BoardShim session internally.
    We do NOT create a second BoardShim here — that would cause
    ANOTHER_BOARD_IS_CREATED_ERROR when the generator tries to prepare its
    own session.
    """
    nf_path = str(NF_BASE)
    if nf_path not in sys.path:
        sys.path.insert(0, nf_path)

    from naviflame.inference import real_time_inference
    from naviflame.utils import BiquadMultiChan, FilterTypes

    feat_path = _resolve_path(config, "feature_extractor_path", "naviflame/models/og_fine_tune.h5")
    mlp_path = _resolve_path(config, "mlp_model_path", "naviflame/models/mlp_model.pkl")
    scaler_path = _resolve_path(config, "scaler_path", "naviflame/models/scaler.pkl")

    if not os.path.exists(feat_path):
        print(_red(f"Feature extractor not found: {feat_path}"))
        sys.exit(1)
    if not os.path.exists(mlp_path):
        print(_red(f"MLP model not found: {mlp_path}. Run --train first."))
        sys.exit(1)

    fs = 500.0
    filters = [
        BiquadMultiChan(8, FilterTypes.bq_type_highpass, 4.5 / fs, 0.5, 0.0),
        BiquadMultiChan(8, FilterTypes.bq_type_notch, 50.0 / fs, 4.0, 0.0),
        BiquadMultiChan(8, FilterTypes.bq_type_lowpass, 100.0 / fs, 0.5, 0.0),
    ]

    print(f"\n{_cyan('=== NaviFlame Inference ===')}")
    print(f"  Gestures: {', '.join(NAVIFLAME_GESTURES)}")
    print(f"  Confidence threshold: 0.4")
    print(f"  Gyro movement gate: 500\n")
    print(_cyan("Connecting to MindRove WiFi board …"))
    print(_dim("  (real_time_inference manages the board session)"))

    # real_time_inference is a generator; board connection happens on first next()
    gen = real_time_inference(
        feature_extractor_path=feat_path,
        mlp_model_path=mlp_path,
        scaler_path=scaler_path,
        filters=filters,
        model_input_len=100,
        gyro_threshold=500,
        prediction_threshold=0.4,
        batch_size=5,
    )

    global _first_draw
    _first_draw = True
    frame_count = 0
    last_time = time.monotonic()
    fps_history: deque[float] = deque(maxlen=20)

    print("\n" * _DISPLAY_LINES)  # reserve display area

    try:
        for prediction, avg_probs in gen:
            now = time.monotonic()
            dt = now - last_time
            last_time = now

            label = int(prediction)
            confidence = float(avg_probs[label])

            # Proportional: use confidence as proxy (no separate board access —
            # real_time_inference owns the session and drains the board buffer).
            prop_val = 0.0 if label == 0 else float(confidence)

            fps_history.append(1.0 / max(dt, 1e-6))
            fps = float(np.mean(fps_history))
            frame_count += 1

            _draw(label, confidence, prop_val, avg_probs, frame_count, fps)

    except RuntimeError as exc:
        print(_red(f"\nInference error: {exc}"))
        sys.exit(1)
    except KeyboardInterrupt:
        print(f"\n\n{_yellow('Stopped.')}")
    finally:
        print(_green("Session released. Bye!"))


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="NaviFlame gesture classifier")
    parser.add_argument("--train", action="store_true", help="Record gestures + fine-tune before inference")
    parser.add_argument("--record-only", action="store_true", help="Only record gestures, then exit")
    parser.add_argument("--fine-tune-only", action="store_true", help="Only fine-tune from existing recording")
    args = parser.parse_args()

    # Load config
    config: dict = {}
    if NF_CONFIG_JSON.exists():
        with open(NF_CONFIG_JSON) as f:
            config = json.load(f)

    print()
    print(_bold("=" * 52))
    print(_bold("    NaviFlame — Gesture Classifier"))
    print(_bold("=" * 52))

    if args.train or args.record_only or args.fine_tune_only:
        if not args.fine_tune_only:
            _run_recording(config)
        if not args.record_only:
            _run_fine_tuning(config)
        if args.record_only or args.fine_tune_only:
            print(_green("Done."))
            return

    _run_inference(config)


if __name__ == "__main__":
    main()
