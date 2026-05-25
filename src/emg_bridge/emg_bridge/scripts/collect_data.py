#!/usr/bin/env python3
"""
Interactive EMG data collection / calibration script.

Connects to the MindRove WiFi armband and guides the user through recording
each gesture one by one. Raw (unfiltered) EMG data and labels are saved to a
timestamped .npz file for later offline training.

Usage (inside the Docker container):
    python scripts/collect_data.py
    python scripts/collect_data.py --reps 5 --duration 6 --output-dir /data
    python scripts/collect_data.py --gesture-names REST POWER PINCH OPEN POINT
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

# Allow running from the repo root or from inside the container
from emg_bridge.board_reader import BoardReader
from emg_bridge.config import (
    DEFAULT_RECORD_DURATION_S,
    DEFAULT_REPS,
    GESTURE_NAMES,
    N_CHANNELS,
    SAMPLING_RATE,
    WARMUP_DURATION_S,
    WINDOW_STEP,
)


# ── ANSI helpers ──────────────────────────────────────────────────────────────

def _green(s: str) -> str:  return f"\033[92m{s}\033[0m"
def _yellow(s: str) -> str: return f"\033[93m{s}\033[0m"
def _bold(s: str) -> str:   return f"\033[1m{s}\033[0m"
def _cyan(s: str) -> str:   return f"\033[96m{s}\033[0m"


# ── Progress bar ──────────────────────────────────────────────────────────────

def _progress(elapsed: float, total: float, width: int = 30) -> str:
    frac = min(elapsed / total, 1.0)
    filled = int(frac * width)
    bar = "█" * filled + "░" * (width - filled)
    pct = int(frac * 100)
    return f"[{bar}] {pct:3d}%  {elapsed:.1f}/{total:.0f}s"


# ── Core recording ────────────────────────────────────────────────────────────

def record_gesture(
    reader: BoardReader,
    duration_s: float,
    sampling_rate: int,
) -> np.ndarray:
    """Record `duration_s` seconds of raw EMG.

    Returns ndarray of shape (n_samples, N_CHANNELS).
    """
    n_needed = int(duration_s * sampling_rate)
    reader.flush()  # discard any buffered samples before recording

    # Live countdown / progress display
    t_start = time.monotonic()
    print()
    while True:
        elapsed = time.monotonic() - t_start
        print(f"\r  {_progress(elapsed, duration_s)}", end="", flush=True)
        if reader.available() >= n_needed:
            break
        time.sleep(0.05)

    data = reader.read(n_needed)
    print(f"\r  {_progress(duration_s, duration_s)}  ✓", flush=True)
    return data  # (n_samples, N_CHANNELS)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="MindRove EMG data collector")
    parser.add_argument("--reps", type=int, default=DEFAULT_REPS,
                        help=f"Repetitions per gesture (default: {DEFAULT_REPS})")
    parser.add_argument("--duration", type=float, default=DEFAULT_RECORD_DURATION_S,
                        help=f"Recording duration per rep in seconds (default: {DEFAULT_RECORD_DURATION_S})")
    parser.add_argument("--output-dir", type=Path, default=Path("/app/data"),
                        help="Directory to save .npz files (default: /app/data)")
    parser.add_argument("--gesture-names", nargs="+", default=GESTURE_NAMES,
                        help="Gesture names (space-separated, must start with REST)")
    args = parser.parse_args()

    gesture_names: list[str] = args.gesture_names
    n_gestures = len(gesture_names)
    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    print()
    print(_bold("=" * 58))
    print(_bold("    MindRove EMG Data Collector"))
    print(_bold("=" * 58))
    print(f"  Gestures   : {', '.join(f'{i}={g}' for i, g in enumerate(gesture_names))}")
    print(f"  Repetitions: {args.reps}")
    print(f"  Duration   : {args.duration} s per rep per gesture")
    print(f"  Output dir : {output_dir}")
    print()

    # ── Connect ───────────────────────────────────────────────────────────────
    print(_cyan("Connecting to MindRove WiFi board …"))
    try:
        reader = BoardReader()
        reader.connect()
    except Exception as exc:
        print(f"\033[91mFailed to connect: {exc}\033[0m")
        sys.exit(1)

    sampling_rate = reader.sampling_rate
    print(_green(f"Connected!  sampling_rate={sampling_rate} Hz  channels={N_CHANNELS}"))

    # ── Warm-up ───────────────────────────────────────────────────────────────
    print(f"\n{_yellow('Warming up …')} ({WARMUP_DURATION_S:.0f} s)")
    warmup_samples = int(WARMUP_DURATION_S * sampling_rate)
    t0 = time.monotonic()
    while time.monotonic() - t0 < WARMUP_DURATION_S:
        if reader.available() >= WINDOW_STEP:
            reader.flush()
        time.sleep(0.05)
    print(_green("Board ready.\n"))

    # ── Recording loop ────────────────────────────────────────────────────────
    all_emg: list[np.ndarray] = []   # list of (n_samples, N_CHANNELS)
    all_labels: list[np.ndarray] = []
    segment_ends: list[int] = []   # cumulative end index of each gesture recording

    try:
        for rep in range(1, args.reps + 1):
            print(_bold(f"── Repetition {rep}/{args.reps} ──────────────────────────────"))
            for g_id, g_name in enumerate(gesture_names):
                print(f"\n  [{g_id + 1}/{n_gestures}] {_bold(g_name)}")

                if g_id == 0:
                    print("  Relax your arm completely.")
                else:
                    print(f"  Perform the {_bold(g_name)} gesture and hold it.")

                input(_yellow("  Press ENTER to start recording …"))
                print(_cyan(f"  Recording {args.duration} s …"))

                chunk = record_gesture(reader, args.duration, sampling_rate)
                n_samp = chunk.shape[0]
                labels = np.full(n_samp, g_id, dtype=np.int32)

                all_emg.append(chunk)
                all_labels.append(labels)
                # Track cumulative end of this segment
                prev_end = segment_ends[-1] if segment_ends else 0
                segment_ends.append(prev_end + n_samp)
                print(_green(f"  ✓ Recorded {n_samp} samples for '{g_name}'"))

    except KeyboardInterrupt:
        print("\n\n  Interrupted — saving collected data so far …")
    finally:
        reader.disconnect()

    if not all_emg:
        print("No data collected. Exiting.")
        sys.exit(0)

    # ── Save ──────────────────────────────────────────────────────────────────
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = output_dir / f"session_{timestamp}.npz"

    emg_arr = np.concatenate(all_emg, axis=0)       # (total_samples, N_CHANNELS)
    label_arr = np.concatenate(all_labels, axis=0)   # (total_samples,)

    np.savez_compressed(
        out_path,
        emg=emg_arr,
        labels=label_arr,
        segment_ends=np.array(segment_ends, dtype=np.int64),
        sampling_rate=np.array(sampling_rate),
        gesture_names=np.array(gesture_names),
    )

    print()
    print(_bold("=" * 58))
    print(_green(f"  Saved {emg_arr.shape[0]} samples → {out_path}"))
    class_counts = {gesture_names[i]: int(np.sum(label_arr == i)) for i in range(n_gestures)}
    for name, count in class_counts.items():
        print(f"    {name:>8}: {count} samples  ({count / sampling_rate:.1f} s)")
    print(_bold("=" * 58))
    print()


if __name__ == "__main__":
    main()
