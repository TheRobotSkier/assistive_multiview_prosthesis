#!/usr/bin/env python3
"""Offline prediction simulator — replay raw EMG through the classifier pipeline.

Loads raw EMG sample CSV from a previous latency-benchmark run, re-runs
filter / window / feature-extraction / classification with tunable parameters,
and saves per-window prediction outputs.

Use this to experiment with different window sizes, step sizes, thresholds,
and smoothing values without re-recording live data.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from emg_bridge import classifier as clf_mod
from emg_bridge.classifier import PredictionSmoother, predict
from emg_bridge.config import (
    GESTURE_NAMES,
    N_CHANNELS,
    PREDICTION_SMOOTHING_FRAMES,
    SAMPLING_RATE,
    WINDOW_LEN,
    WINDOW_STEP,
)
from emg_bridge.features import compute_features
from emg_bridge.latency_analysis import (
    PredictionFrame,
    find_first_prediction_frame,
    measure_latency,
)
from emg_bridge.preprocessing import extract_windows, filter_signal


@dataclass
class _SimConfig:
    window_len: int = WINDOW_LEN
    window_step: int = WINDOW_STEP
    confidence_threshold: float = 0.55
    smoothing_frames: int = PREDICTION_SMOOTHING_FRAMES
    z_score: float = 4.0
    min_active_samples: int = 6
    max_gap_samples: int = 2
    pre_onset_search_samples: int = 20
    onset_lookback_windows: int = PREDICTION_SMOOTHING_FRAMES
    baseline_offset_s: float = -1.0
    countdown_s: int = 3
    auto_stop_confidence: float = 0.70
    auto_stop_hold_s: float = 1.0
    max_trial_duration_s: float = 15.0


def _bold(s: str) -> str:   return f"\033[1m{s}\033[0m"
def _green(s: str) -> str:  return f"\033[92m{s}\033[0m"
def _yellow(s: str) -> str: return f"\033[93m{s}\033[0m"
def _cyan(s: str) -> str:   return f"\033[96m{s}\033[0m"
def _dim(s: str) -> str:    return f"\033[2m{s}\033[0m"


def _safe_name(label: int) -> str:
    if 0 <= label < len(GESTURE_NAMES):
        return GESTURE_NAMES[label]
    return f"G{label}"


def _compute_n_vote(recent_runs: list[tuple[int, float]], n: int) -> tuple[int, float]:
    subset = recent_runs[-n:]
    counts: dict[int, int] = {}
    for label, _ in subset:
        counts[label] = counts.get(label, 0) + 1
    winner = max(counts, key=counts.get)
    winner_count = counts[winner]
    if winner_count * 2 > n:
        return winner, winner_count / n
    return 0, 0.0


def _find_latest_raw_dir(data_root: Path) -> Path | None:
    dirs = sorted(
        [d for d in (data_root).iterdir() if d.is_dir()],
        reverse=True,
    )
    for d in dirs:
        if (d / "latency_samples.csv").exists():
            return d
    return None


def _load_raw_samples(samples_path: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    trials: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    with samples_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            tid = row["trial_id"]
            raw = np.array([float(row[f"raw_ch{ch}"]) for ch in range(8)], dtype=float)
            filtered = np.array([float(row[f"filtered_ch{ch}"]) for ch in range(8)], dtype=float)
            if tid not in trials:
                trials[tid] = ([], [])
            trials[tid][0].append(raw)
            trials[tid][1].append(filtered)
    return {tid: (np.array(raws), np.array(filts)) for tid, (raws, filts) in trials.items()}


def _run_simulation(
    trial_id: str,
    gesture_label: int,
    raw_emg: np.ndarray,
    pipe,
    config: _SimConfig,
    writer: csv.DictWriter,
) -> list[PredictionFrame]:
    emg_filt = filter_signal(raw_emg)
    windows = extract_windows(emg_filt, config.window_len, config.window_step)
    frames: list[PredictionFrame] = []
    if windows.shape[0] == 0:
        return frames

    smoother = PredictionSmoother(config.smoothing_frames)
    recent_predictions: list[tuple[int, float]] = []

    window_start_samples = np.arange(windows.shape[0]) * config.window_step

    for win_idx in range(windows.shape[0]):
        features = compute_features(windows[win_idx])
        raw_label, confidence, probs = predict(pipe, features, threshold=config.confidence_threshold)
        smoothed_label = smoother.update(raw_label)
        recent_predictions.append((raw_label, float(confidence)))

        v2_l, v2_c = _compute_n_vote(recent_predictions, 2)
        v3_l, v3_c = _compute_n_vote(recent_predictions, 3)
        v4_l, v4_c = _compute_n_vote(recent_predictions, 4)
        v5_l, v5_c = _compute_n_vote(recent_predictions, 5)

        pred_time = (window_start_samples[win_idx] + config.window_len) / SAMPLING_RATE

        frames.append(PredictionFrame(
            frame_index=win_idx,
            prediction_time_s=pred_time,
            raw_label=raw_label,
            smoothed_label=smoothed_label,
            confidence=float(confidence),
            window_start_sample=window_start_samples[win_idx],
            window_end_sample=window_start_samples[win_idx] + config.window_len,
        ))

        row = {
            "trial_id": trial_id,
            "gesture_label": gesture_label,
            "gesture_name": _safe_name(gesture_label),
            "frame_index": win_idx,
            "prediction_time_s": f"{pred_time:.6f}",
            "window_start_sample": window_start_samples[win_idx],
            "window_end_sample": window_start_samples[win_idx] + config.window_len,
            "raw_label": raw_label,
            "raw_name": _safe_name(raw_label),
            "raw_confidence": f"{confidence:.6f}",
            "smoothed_label": smoothed_label,
            "smoothed_name": _safe_name(smoothed_label),
            "vote2_label": v2_l,
            "vote2_name": _safe_name(v2_l),
            "vote2_confidence": f"{v2_c:.6f}",
            "vote3_label": v3_l,
            "vote3_name": _safe_name(v3_l),
            "vote3_confidence": f"{v3_c:.6f}",
            "vote4_label": v4_l,
            "vote4_name": _safe_name(v4_l),
            "vote4_confidence": f"{v4_c:.6f}",
            "vote5_label": v5_l,
            "vote5_name": _safe_name(v5_l),
            "vote5_confidence": f"{v5_c:.6f}",
        }
        for idx, name in enumerate(GESTURE_NAMES):
            row[f"prob_{name.lower()}"] = f"{probs[idx]:.8f}"
        writer.writerow(row)

    return frames


def _menu_items(config: _SimConfig) -> list[tuple[str, str, str]]:
    return [
        ("1", "Window length", f"{config.window_len} samples ({config.window_len / SAMPLING_RATE * 1000:.0f} ms)"),
        ("2", "Window step", f"{config.window_step} samples ({config.window_step / SAMPLING_RATE * 1000:.0f} ms)"),
        ("3", "Confidence threshold", f"{config.confidence_threshold:.2f}"),
        ("4", "Smoothing frames", f"{config.smoothing_frames}"),
        ("5", "Z-score threshold", f"{config.z_score:.1f}"),
        ("6", "Min active samples", f"{config.min_active_samples}"),
        ("7", "Max gap samples", f"{config.max_gap_samples}"),
        ("8", "Pre-onset search", f"{config.pre_onset_search_samples} samples"),
        ("9", "Onset lookback windows", f"{config.onset_lookback_windows}"),
        ("10", "Baseline offset (s)", f"{config.baseline_offset_s:.1f} (relative to cue)"),
        ("11", "Countdown (s)", f"{config.countdown_s}"),
        ("12", "Auto-stop confidence", f"{config.auto_stop_confidence:.2f}"),
        ("13", "Auto-stop hold (s)", f"{config.auto_stop_hold_s:.1f}"),
        ("14", "Max trial duration (s)", f"{config.max_trial_duration_s:.1f}"),
    ]


def _load_config(path: Path | None) -> _SimConfig:
    config = _SimConfig()
    if path is None or not path.exists():
        return config

    with path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    section = raw.get("simulator", {})
    if not isinstance(section, dict):
        return config

    for key in (
        "window_len",
        "window_step",
        "confidence_threshold",
        "smoothing_frames",
        "z_score",
        "min_active_samples",
        "max_gap_samples",
        "pre_onset_search_samples",
        "onset_lookback_windows",
        "baseline_offset_s",
        "countdown_s",
        "auto_stop_confidence",
        "auto_stop_hold_s",
        "max_trial_duration_s",
    ):
        if key in section:
            current = getattr(config, key)
            setattr(config, key, type(current)(section[key]))
    return config


def _set_config_value(config: _SimConfig, key: str) -> None:
    try:
        raw = input("  New value: ").strip()
        if key == "1":
            config.window_len = int(raw)
        elif key == "2":
            config.window_step = int(raw)
        elif key == "3":
            config.confidence_threshold = float(raw)
        elif key == "4":
            config.smoothing_frames = int(raw)
        elif key == "5":
            config.z_score = float(raw)
        elif key == "6":
            config.min_active_samples = int(raw)
        elif key == "7":
            config.max_gap_samples = int(raw)
        elif key == "8":
            config.pre_onset_search_samples = int(raw)
        elif key == "9":
            config.onset_lookback_windows = int(raw)
        elif key == "10":
            config.baseline_offset_s = float(raw)
        elif key == "11":
            config.countdown_s = int(raw)
        elif key == "12":
            config.auto_stop_confidence = float(raw)
        elif key == "13":
            config.auto_stop_hold_s = float(raw)
        elif key == "14":
            config.max_trial_duration_s = float(raw)
    except ValueError:
        print(_yellow("  Invalid input — value unchanged."))


def _draw_menu(config: _SimConfig) -> None:
    sys.stdout.write("\033[2J\033[H")
    print(_bold("EMG Prediction Simulator"))
    print("Replay the latest latency raw-sample CSV through the classifier pipeline.")
    print("Tune windowing, smoothing, thresholds, and onset logic, then press R to")
    print("write sim_predictions.csv and sim_parameters.txt for later analysis.")
    print()
    print(_bold("Parameter Tuning Menu"))
    print(_bold("=" * 45))
    for key, label, value in _menu_items(config):
        print(f" {_cyan(key):>3}. {label:<22} {_dim(value)}")
    print(_bold("=" * 45))
    print(f" {_green('R')}. Run simulation")
    print(f" {_yellow('Q')}. Quit")
    print()


def _interactive_menu(
    config: _SimConfig,
    *,
    pipe,
    raw_data: dict[str, tuple[np.ndarray, np.ndarray]],
    output_dir: Path,
    model_dir: Path,
) -> None:
    while True:
        _draw_menu(config)
        choice = input("  Choice: ").strip().lower()

        if choice == "q":
            print(_yellow("Quit."))
            return

        if choice == "r":
            print(_cyan("Running simulation ..."))
            output_dir.mkdir(parents=True, exist_ok=True)
            out_path = output_dir / "sim_predictions.csv"
            fieldnames = [
                "trial_id", "gesture_label", "gesture_name",
                "frame_index", "prediction_time_s",
                "window_start_sample", "window_end_sample",
                "raw_label", "raw_name", "raw_confidence",
                "smoothed_label", "smoothed_name",
                "vote2_label", "vote2_name", "vote2_confidence",
                "vote3_label", "vote3_name", "vote3_confidence",
                "vote4_label", "vote4_name", "vote4_confidence",
                "vote5_label", "vote5_name", "vote5_confidence",
            ]
            fieldnames.extend([f"prob_{name.lower()}" for name in GESTURE_NAMES])

            all_frames: dict[str, list[PredictionFrame]] = {}
            total_frames = 0

            with out_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for tid, (raw_emg, _) in sorted(raw_data.items()):
                    parts = tid.rsplit("_", 1)
                    gesture_name = parts[0].upper() if len(parts) >= 1 else "REST"
                    gesture_label = GESTURE_NAMES.index(gesture_name) if gesture_name in GESTURE_NAMES else 0
                    frames = _run_simulation(tid, gesture_label, raw_emg, pipe, config, writer)
                    all_frames[tid] = frames
                    total_frames += len(frames)

            gesture_latencies: dict[str, list[float]] = {}
            baseline_end = int(max(0, config.countdown_s + config.baseline_offset_s) * SAMPLING_RATE)

            for tid, (raw_emg, filtered_emg) in sorted(raw_data.items()):
                frames = all_frames.get(tid, [])
                if not frames:
                    continue
                parts = tid.rsplit("_", 1)
                gesture_name = parts[0].upper() if len(parts) >= 1 else "REST"
                gesture_label = GESTURE_NAMES.index(gesture_name) if gesture_name in GESTURE_NAMES else 0
                if gesture_label == 0:
                    continue

                result = measure_latency(
                    raw_samples=filtered_emg,
                    frames=frames,
                    target_label=gesture_label,
                    cue_time_s=float(config.countdown_s),
                    sampling_rate_hz=float(SAMPLING_RATE),
                    baseline_end_sample=baseline_end,
                    min_confidence=config.confidence_threshold,
                    smoothing_frames=config.smoothing_frames,
                    min_active_samples=config.min_active_samples,
                    max_gap_samples=config.max_gap_samples,
                    pre_onset_search_samples=config.pre_onset_search_samples,
                    onset_lookback_windows=config.onset_lookback_windows,
                    threshold_z_score=config.z_score,
                )
                if result is not None:
                    gesture_latencies.setdefault(gesture_name, []).append(result.latency_ms)

            params_path = output_dir / "sim_parameters.txt"
            lines: list[str] = []
            lines.append(f"window_len={config.window_len}")
            lines.append(f"window_step={config.window_step}")
            lines.append(f"confidence_threshold={config.confidence_threshold}")
            lines.append(f"smoothing_frames={config.smoothing_frames}")
            lines.append(f"z_score={config.z_score}")
            lines.append(f"min_active_samples={config.min_active_samples}")
            lines.append(f"max_gap_samples={config.max_gap_samples}")
            lines.append(f"pre_onset_search_samples={config.pre_onset_search_samples}")
            lines.append(f"onset_lookback_windows={config.onset_lookback_windows}")
            lines.append(f"baseline_offset_s={config.baseline_offset_s}")
            lines.append(f"countdown_s={config.countdown_s}")
            lines.append(f"auto_stop_confidence={config.auto_stop_confidence}")
            lines.append(f"auto_stop_hold_s={config.auto_stop_hold_s}")
            lines.append(f"max_trial_duration_s={config.max_trial_duration_s}")
            lines.append(f"sampling_rate_hz={SAMPLING_RATE}")
            lines.append(f"total_frames={total_frames}")
            for gname in GESTURE_NAMES[1:]:
                vals = gesture_latencies.get(gname, [])
                if vals:
                    lines.append(f"latency_{gname}_mean_ms={np.mean(vals):.2f}")
                    lines.append(f"latency_{gname}_count={len(vals)}")
                else:
                    lines.append(f"latency_{gname}_mean_ms=unresolved")
                    lines.append(f"latency_{gname}_count=0")
            params_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

            print(_green(f"  Wrote {total_frames} frames → {out_path}"))
            print(_green(f"  Parameters → {params_path}"))
            for gname in GESTURE_NAMES[1:]:
                vals = gesture_latencies.get(gname, [])
                if vals:
                    print(f"    {gname:>10}: mean={np.mean(vals):.1f} ms  n={len(vals)}")
            input(_dim("  Press ENTER to continue ..."))
            continue

        valid_keys = {item[0] for item in _menu_items(config)}
        if choice in valid_keys:
            _set_config_value(config, choice)
        else:
            print(_yellow("  Unknown choice."))


def main() -> None:
    parser = argparse.ArgumentParser(description="Offline EMG prediction simulator")
    parser.add_argument("--model-dir", type=Path, default=Path("/app/models"),
                        help="Directory containing trained EMG model files")
    parser.add_argument("--data-root", type=Path, default=Path("/app/data/latency"),
                        help="Root directory of latency benchmark outputs")
    parser.add_argument("--output-dir", type=Path, default=None,
                        help="Output directory (default: <data-root>/<latest>/sim)")
    parser.add_argument("--config", type=Path, default=Path("/prosthesis_ws/config/emg_latency_test.yaml"),
                        help="YAML config containing simulator defaults")
    args = parser.parse_args()

    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print(_yellow("The prediction simulator requires an interactive TTY."))
        sys.exit(2)

    latest_dir = _find_latest_raw_dir(args.data_root)
    if latest_dir is None:
        print(_yellow(f"No latency benchmark data found under {args.data_root}"))
        sys.exit(1)

    print(_cyan(f"Using raw data from: {latest_dir}"))

    pipe = clf_mod.load(args.model_dir)

    raw_data = _load_raw_samples(latest_dir / "latency_samples.csv")
    if not raw_data:
        print(_yellow("No trial data found in latency_samples.csv"))
        sys.exit(1)

    print(_green(f"Loaded {len(raw_data)} trials."))

    output_dir = args.output_dir or (latest_dir / "sim")
    config = _load_config(args.config)

    _interactive_menu(
        config,
        pipe=pipe,
        raw_data=raw_data,
        output_dir=output_dir,
        model_dir=args.model_dir,
    )


if __name__ == "__main__":
    main()
