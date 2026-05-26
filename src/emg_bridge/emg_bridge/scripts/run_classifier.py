#!/usr/bin/env python3
"""
Real-time EMG gesture classification — live terminal display.

Connects to the MindRove WiFi armband, applies the trained classifier, and
continuously prints gesture label, confidence, and proportional control value.

Usage (inside the Docker container):
    python scripts/run_classifier.py
    python scripts/run_classifier.py --model-dir /app/models
    python scripts/run_classifier.py --model-dir /app/models --threshold 0.6
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

from emg_bridge import classifier as clf_mod
from emg_bridge import proportional as prop_mod
from emg_bridge.board_reader import BoardReader
from emg_bridge.classifier import PredictionSmoother, predict
from emg_bridge.config import (
    CONFIDENCE_THRESHOLD,
    GESTURE_NAMES,
    N_CHANNELS,
    PREDICTION_SMOOTHING_FRAMES,
    WINDOW_LEN,
    WINDOW_STEP,
)
from emg_bridge.features import compute_features
from emg_bridge.preprocessing import OnlineFilter, RingBuffer

# ── Optional ROS 2 bridge ──────────────────────────────────────────────────────
_ros_state = None
try:
    from emg_bridge.ros_bridge_node import EmgState, spin_in_thread as _ros_spin
    _ros_state = EmgState()
    _ros_spin(_ros_state)
    print("ROS 2 bridge active — publishing on /emg/* topics.")
except ImportError:
    pass  # rclpy not installed; ROS bridge silently disabled

# ── ANSI helpers ──────────────────────────────────────────────────────────────

def _bold(s: str) -> str:   return f"\033[1m{s}\033[0m"
def _green(s: str) -> str:  return f"\033[92m{s}\033[0m"
def _yellow(s: str) -> str: return f"\033[93m{s}\033[0m"
def _cyan(s: str) -> str:   return f"\033[96m{s}\033[0m"
def _red(s: str) -> str:    return f"\033[91m{s}\033[0m"
def _dim(s: str) -> str:    return f"\033[2m{s}\033[0m"

# ANSI move-up N lines and clear to end
_UP = "\033[A"
_CLEAR_LINE = "\033[2K"


def _prop_bar(val: float, width: int = 20) -> str:
    filled = int(val * width)
    bar = "▓" * filled + "░" * (width - filled)
    return f"[{bar}] {val:.2f}"


def _conf_bar(val: float, width: int = 15) -> str:
    filled = int(val * width)
    bar = "█" * filled + "·" * (width - filled)
    return f"[{bar}] {val * 100:.0f}%"


# ── Display ───────────────────────────────────────────────────────────────────

_DISPLAY_LINES = 8   # how many lines the display occupies (for refresh)
_first_draw = True


def _draw(
    label: int,
    confidence: float,
    proportional: float,
    probs: np.ndarray,
    gesture_names: list[str],
    frame_count: int,
    fps: float,
    mode: str,
) -> None:
    global _first_draw
    if not _first_draw:
        # Move cursor up to overwrite previous display
        sys.stdout.write((_UP + _CLEAR_LINE) * _DISPLAY_LINES)

    gesture = gesture_names[label] if label < len(gesture_names) else f"G{label}"
    rest_active = label == 0

    sys.stdout.write(_bold("─" * 52) + "\n")
    sys.stdout.write(
        f"  {_bold('Gesture')}  : "
        + (_dim(gesture) if rest_active else _green(_bold(gesture)))
        + f"  {_dim(f'(frame {frame_count})')}"
        + "\n"
    )
    sys.stdout.write(
        f"  {_bold('Mode')}     : {_cyan(mode)}\n"
    )
    sys.stdout.write(
        f"  {_bold('Confidence')}: {_conf_bar(confidence)}\n"
    )
    sys.stdout.write(
        f"  {_bold('Prop. ctrl')}: {_prop_bar(proportional)}\n"
    )
    # Per-class probability row
    prob_parts = []
    for i, name in enumerate(gesture_names):
        pct = f"{probs[i] * 100:.0f}%"
        s = f"{name}:{pct}"
        if i == label:
            s = _green(s)
        else:
            s = _dim(s)
        prob_parts.append(s)
    sys.stdout.write("  " + "  ".join(prob_parts) + "\n")
    sys.stdout.write(_bold("─" * 52) + "\n")
    sys.stdout.write(_dim(f"  {fps:.1f} Hz  |  Ctrl-C to quit") + "\n")
    sys.stdout.flush()
    _first_draw = False


# ── Compact output (non-TTY / ROS launch) ──────────────────────────────────

def _print_compact(
    prev_label: int,
    label: int,
    confidence: float,
    proportional: float,
    probs: np.ndarray,
    gesture_names: list[str],
    frame_count: int,
    fps: float,
    mode: str,
) -> None:
    """Print a single-line gesture transition for non-interactive output."""
    prev_name = gesture_names[prev_label] if prev_label < len(gesture_names) else f"G{prev_label}"
    curr_name = gesture_names[label] if label < len(gesture_names) else f"G{label}"
    prob_parts = " ".join(f"{name}:{probs[i] * 100:.0f}%" for i, name in enumerate(gesture_names))
    print(
        f"[emg] {prev_name} -> {curr_name} "
        f"(conf={confidence * 100:.0f}%, prop={proportional:.2f}, mode={mode}) "
        f"{prob_parts}",
        flush=True,
    )


def _print_status(
    label: int,
    confidence: float,
    proportional: float,
    gesture_names: list[str],
    frame_count: int,
    fps: float,
) -> None:
    """Print a periodic heartbeat status line."""
    name = gesture_names[label] if label < len(gesture_names) else f"G{label}"
    print(
        f"[emg] status: {name} (conf={confidence * 100:.0f}%, prop={proportional:.2f}, {fps:.1f} Hz, frame={frame_count})",
        flush=True,
    )


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    global _first_draw

    # Keep EMG output visible in ROS launch/docker logs immediately.
    try:
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
    except AttributeError:
        pass
    os.environ.setdefault("PYTHONUNBUFFERED", "1")

    parser = argparse.ArgumentParser(description="Real-time EMG gesture classifier")
    parser.add_argument("--model-dir", type=Path, default=Path("/app/models"),
                        help="Directory containing trained model files")
    parser.add_argument("--threshold", type=float, default=CONFIDENCE_THRESHOLD,
                        help=f"Confidence threshold (default: {CONFIDENCE_THRESHOLD})")
    parser.add_argument("--smooth", type=int, default=PREDICTION_SMOOTHING_FRAMES,
                        help=f"Smoothing window in frames (default: {PREDICTION_SMOOTHING_FRAMES})")
    parser.add_argument("--quiet", action="store_true",
                        help="Disable live terminal UI; print compact status lines for logs")
    args, _ros_args = parser.parse_known_args()

    if not args.quiet:
        print()
        print(_bold("=" * 52))
        print(_bold("    EMG Classifier — Live"))
        print(_bold("=" * 52))
    else:
        print("[emg] classifier starting", flush=True)

    # ── Load models ───────────────────────────────────────────────────────────
    if not args.quiet:
        print(f"{_cyan('Loading classifier from')} {args.model_dir} …")
    model_path = args.model_dir / "classifier.pkl"
    if not model_path.exists():
        print(_red(
            f"Model file not found: {model_path}\n"
            f"  Please train a model first:\n"
            f"    ros2 run emg_bridge train --data-dir /app/data --model-dir {args.model_dir}\n"
            f"  Or run collect_data to record training data:\n"
            f"    ros2 run emg_bridge collect_data --output-dir /app/data"
        ))
        sys.exit(1)
    try:
        pipe = clf_mod.load(args.model_dir)
    except Exception as e:
        print(_red(f"Failed to load classifier from {args.model_dir}: {e}"))
        sys.exit(1)

    calibration = prop_mod.load(args.model_dir)
    if calibration is None:
        print(_yellow("No proportional calibration found — using fallback normalisation."))

    gesture_names = GESTURE_NAMES
    if not args.quiet:
        print(_green("Models loaded.\n"))

    # ── Connect ───────────────────────────────────────────────────────────────
    if not args.quiet:
        print(_cyan("Connecting to MindRove WiFi board …"))
    try:
        reader = BoardReader()
        reader.connect()
    except Exception as exc:
        print(_red(f"Failed to connect: {exc}"))
        sys.exit(1)
    if not args.quiet:
        print(_green(f"Connected!  {reader.sampling_rate} Hz  {N_CHANNELS} channels\n"))
    else:
        print(f"[emg] connected {reader.sampling_rate}Hz {N_CHANNELS}ch", flush=True)

    # ── Inference loop ────────────────────────────────────────────────────────
    filt = OnlineFilter(N_CHANNELS)
    ring = RingBuffer(WINDOW_LEN, N_CHANNELS)
    smoother = PredictionSmoother(args.smooth)

    # Boot the ring buffer: wait until we have at least one full window
    if not args.quiet:
        print(_cyan("Filling buffer …"))
    while not ring.is_full():
        chunk = reader.read(WINDOW_STEP)
        filtered = filt.process(chunk)
        ring.push(filtered)
    if not args.quiet:
        print(_green("Ready.\n"))
    else:
        print("[emg] ready", flush=True)

    interactive = False
    _first_draw = True
    frame_count = 0
    last_time = time.monotonic()
    fps_history: deque[float] = deque(maxlen=20)
    prev_label = -1
    last_status_time = time.monotonic()
    _STATUS_INTERVAL = 2.0  # seconds between compact log status lines

    if interactive:
        print("\n" * _DISPLAY_LINES)  # reserve display area
    else:
        print("[emg] logging compact classifier status every 2s and on gesture changes", flush=True)

    try:
        while True:
            # Read next step of samples
            chunk = reader.read(WINDOW_STEP)
            filtered = filt.process(chunk)
            ring.push(filtered)

            if not ring.is_full():
                continue

            # Extract features
            window = ring.get_window().T  # (N_channels, WINDOW_LEN)
            features = compute_features(window)

            # Classify
            label, confidence, probs = predict(pipe, features, threshold=args.threshold)
            smoothed_label = smoother.update(label)

            # Proportional control
            prop_val = prop_mod.compute_proportional(window, smoothed_label, calibration)

            # Update ROS state (if bridge is active)
            if _ros_state is not None:
                with _ros_state.lock:
                    _ros_state.label        = smoothed_label
                    _ros_state.name         = gesture_names[smoothed_label]
                    _ros_state.confidence   = float(confidence)
                    _ros_state.proportional = float(prop_val)

            # FPS estimate
            now = time.monotonic()
            fps_history.append(1.0 / max(now - last_time, 1e-6))
            last_time = now
            fps = float(np.mean(fps_history))

            # Fetch current mode from ROS state (if available)
            mode = "NOT_GRASPING"
            if _ros_state is not None:
                with _ros_state.lock:
                    mode = _ros_state.mode

            frame_count += 1

            if interactive:
                _draw(smoothed_label, confidence, prop_val, probs, gesture_names, frame_count, fps, mode)
            else:
                now_wall = time.monotonic()
                if smoothed_label != prev_label:
                    _print_compact(prev_label, smoothed_label, confidence, prop_val, probs, gesture_names, frame_count, fps, mode)
                    prev_label = smoothed_label
                    last_status_time = now_wall  # reset timer on transition
                elif now_wall - last_status_time >= _STATUS_INTERVAL:
                    _print_status(smoothed_label, confidence, prop_val, gesture_names, frame_count, fps)
                    last_status_time = now_wall

    except KeyboardInterrupt:
        if interactive:
            print(f"\n\n{_yellow('Stopped.')}")
        else:
            print("[emg] stopped", flush=True)
    finally:
        reader.disconnect()
        if interactive:
            print(_green("Session released. Bye!"))
        else:
            print("[emg] session released", flush=True)


if __name__ == "__main__":
    main()
