#!/usr/bin/env python3
"""
Offline training script: load recorded .npz files → extract features → train classifier.

Usage (inside the Docker container):
    python scripts/train.py
    python scripts/train.py --data-dir /app/data --model-dir /app/models
    python scripts/train.py --data-dir /app/data --model-dir /app/models --cv-folds 10
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from emg_classifier import classifier as clf_mod
from emg_classifier import proportional as prop_mod
from emg_classifier.config import GESTURE_NAMES, WINDOW_LEN, WINDOW_STEP
from emg_classifier.features import compute_features_batch
from emg_classifier.preprocessing import extract_windows, filter_signal


# ── ANSI helpers ──────────────────────────────────────────────────────────────

def _bold(s: str) -> str:   return f"\033[1m{s}\033[0m"
def _green(s: str) -> str:  return f"\033[92m{s}\033[0m"
def _yellow(s: str) -> str: return f"\033[93m{s}\033[0m"
def _cyan(s: str) -> str:   return f"\033[96m{s}\033[0m"


# ── Data loading ──────────────────────────────────────────────────────────────

def load_sessions(data_dir: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
    """Load and merge all .npz session files in data_dir.

    Returns:
        emg:          (total_samples, N_channels)
        labels:       (total_samples,) integer label per sample
        segment_ends: (n_segments,) cumulative end index of each independent recording
        gesture_names: from the first file found
    """
    files = sorted(data_dir.glob("*.npz"))
    if not files:
        print(f"\033[91mNo .npz files found in {data_dir}\033[0m")
        sys.exit(1)

    emgs, lbls = [], []
    all_seg_ends: list[int] = []
    offset = 0
    gesture_names = GESTURE_NAMES

    for f in files:
        data = np.load(f, allow_pickle=True)
        emg_f = data["emg"]
        lbl_f = data["labels"]
        emgs.append(emg_f)
        lbls.append(lbl_f)

        if "gesture_names" in data:
            gesture_names = data["gesture_names"].tolist()

        if "segment_ends" in data:
            seg_ends = data["segment_ends"].tolist()
            all_seg_ends.extend(offset + e for e in seg_ends)
        else:
            # Legacy files without segment info: treat whole file as one segment
            all_seg_ends.append(offset + emg_f.shape[0])

        offset += emg_f.shape[0]
        print(f"  Loaded {f.name}: {emg_f.shape[0]} samples")

    emg = np.concatenate(emgs, axis=0)
    labels = np.concatenate(lbls, axis=0)
    segment_ends = np.array(all_seg_ends, dtype=np.int64)
    return emg, labels, segment_ends, gesture_names


# ── Feature pipeline ──────────────────────────────────────────────────────────

def build_feature_matrix(
    emg: np.ndarray,
    labels: np.ndarray,
    segment_ends: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Filter → window → extract features, processing each segment independently.

    Each segment is filtered and windowed separately to avoid boundary artifacts
    from concatenating discontinuous recordings.

    Returns:
        X: (N_windows, N_features)
        y: (N_windows,) integer labels
    """
    X_parts, y_parts = [], []

    seg_starts = np.concatenate([[0], segment_ends[:-1]])
    for seg_start, seg_end in zip(seg_starts, segment_ends):
        seg_emg = emg[seg_start:seg_end]
        seg_labels = labels[seg_start:seg_end]

        if len(seg_emg) < WINDOW_LEN:
            continue  # too short to form even one window

        emg_filt = filter_signal(seg_emg)                              # (N_samples, N_ch)
        windows = extract_windows(emg_filt, WINDOW_LEN, WINDOW_STEP)  # (N_win, N_ch, wl)

        n_win = windows.shape[0]
        starts = np.arange(n_win) * WINDOW_STEP
        ends = starts + WINDOW_LEN

        # Majority label per window
        y_seg = np.array([
            np.bincount(seg_labels[s:e].astype(int)).argmax()
            for s, e in zip(starts, ends)
        ], dtype=np.int32)

        X_seg = compute_features_batch(windows)
        X_parts.append(X_seg)
        y_parts.append(y_seg)

    if not X_parts:
        print("\033[91mNo usable segments found. Check data.\033[0m")
        sys.exit(1)

    return np.vstack(X_parts), np.concatenate(y_parts)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Train EMG gesture classifier")
    parser.add_argument("--data-dir", type=Path, default=Path("/app/data"),
                        help="Directory with .npz session files")
    parser.add_argument("--model-dir", type=Path, default=Path("/app/models"),
                        help="Directory to save trained models")
    parser.add_argument("--cv-folds", type=int, default=5,
                        help="Number of stratified cross-validation folds (default: 5)")
    args = parser.parse_args()

    print()
    print(_bold("=" * 58))
    print(_bold("    EMG Classifier Training"))
    print(_bold("=" * 58))

    # ── Load data ─────────────────────────────────────────────────────────────
    print(f"\n{_cyan('Loading data from')} {args.data_dir} …")
    emg, labels, segment_ends, gesture_names = load_sessions(args.data_dir)
    print(_green(f"Total: {emg.shape[0]} samples, {len(gesture_names)} classes, "
                 f"{len(segment_ends)} segments"))

    # ── Feature extraction ────────────────────────────────────────────────────
    print(f"\n{_cyan('Extracting features …')}")
    X, y = build_feature_matrix(emg, labels, segment_ends)
    print(_green(f"Feature matrix: {X.shape}"))

    for i, name in enumerate(gesture_names):
        n = int(np.sum(y == i))
        print(f"  {name:>8}: {n} windows")

    # ── Train ─────────────────────────────────────────────────────────────────
    print(f"\n{_cyan('Training …')}")
    pipe = clf_mod.train(X, y, model_dir=args.model_dir, cv_folds=args.cv_folds, verbose=True)

    # ── Proportional calibration ──────────────────────────────────────────────
    print(f"\n{_cyan('Computing proportional control calibration …')}")
    # Re-use the same segment-aware windowing for consistency
    windows_by_class: dict[int, list[np.ndarray]] = {}
    seg_starts = np.concatenate([[0], segment_ends[:-1]])
    for seg_start, seg_end in zip(seg_starts, segment_ends):
        seg_emg = emg[seg_start:seg_end]
        seg_labels = labels[seg_start:seg_end]
        if len(seg_emg) < WINDOW_LEN:
            continue
        seg_filt = filter_signal(seg_emg)
        wins = extract_windows(seg_filt, WINDOW_LEN, WINDOW_STEP)
        n_win = wins.shape[0]
        starts_w = np.arange(n_win) * WINDOW_STEP
        ends_w = starts_w + WINDOW_LEN
        y_seg = np.array([
            np.bincount(seg_labels[s:e].astype(int)).argmax()
            for s, e in zip(starts_w, ends_w)
        ])
        for i in range(len(gesture_names)):
            mask = y_seg == i
            if mask.sum() > 0:
                windows_by_class.setdefault(i, []).extend(wins[mask].tolist())

    windows_by_class_arr = {k: np.array(v) for k, v in windows_by_class.items()}
    calibration = prop_mod.calibrate(windows_by_class_arr)
    prop_mod.save(calibration, args.model_dir)
    print(_green(f"Proportional calibration saved → {args.model_dir}/{prop_mod.PROP_CAL_FILE}"))

    for i, name in enumerate(gesture_names):
        rmin = calibration["rms_min"].get(i, float("nan"))
        rmax = calibration["rms_max"].get(i, float("nan"))
        print(f"  {name:>8}: RMS [{rmin:.2f}, {rmax:.2f}]")

    print()
    print(_bold("=" * 58))
    print(_green("  Training complete!"))
    print(_bold("=" * 58))
    print()


if __name__ == "__main__":
    main()
