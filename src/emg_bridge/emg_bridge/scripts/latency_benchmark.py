#!/usr/bin/env python3
"""Interactive EMG latency benchmark.

Runs the live classifier against prompted gesture trials, records raw and
filtered EMG data plus prediction frames, and estimates onset-to-prediction
latency for each trial.
"""

from __future__ import annotations

import argparse
import csv
import json
import select
import sys
import termios
import time
import tty
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from emg_bridge import classifier as clf_mod
from emg_bridge import proportional as prop_mod
from emg_bridge.board_reader import BoardReader
from emg_bridge.classifier import PredictionSmoother, predict
from emg_bridge.config import (
    CONFIDENCE_THRESHOLD,
    GESTURE_NAMES,
    PREDICTION_SMOOTHING_FRAMES,
    SAMPLING_RATE,
    WINDOW_LEN,
    WINDOW_STEP,
)
from emg_bridge.features import compute_features
from emg_bridge.latency_analysis import PredictionFrame, measure_latency
from emg_bridge.latency_analysis import find_sustained_prediction_run
from emg_bridge.latency_benchmark_state import build_trial_message_lines
from emg_bridge.latency_protocol import TrialSpec, plan_trials
from emg_bridge.preprocessing import OnlineFilter, RingBuffer


@dataclass(frozen=True)
class TrialOutcome:
    trial_id: str
    gesture_label: int
    gesture_name: str
    repeat_index: int
    cue_time_s: float
    end_time_s: float
    status: str
    onset_sample_index: int | None
    onset_time_s: float | None
    prediction_frame_index: int | None
    prediction_time_s: float | None
    prediction_window_start_sample: int | None
    prediction_window_end_sample: int | None
    latency_ms: float | None
    threshold: float | None
    notes: str


def _bold(text: str) -> str:
    return f"\033[1m{text}\033[0m"


def _green(text: str) -> str:
    return f"\033[92m{text}\033[0m"


def _yellow(text: str) -> str:
    return f"\033[93m{text}\033[0m"


def _cyan(text: str) -> str:
    return f"\033[96m{text}\033[0m"


def _red(text: str) -> str:
    return f"\033[91m{text}\033[0m"


def _safe_name(label: int) -> str:
    if 0 <= label < len(GESTURE_NAMES):
        return GESTURE_NAMES[label]
    return f"G{label}"


@contextmanager
def _raw_keyboard_mode():
    if not sys.stdin.isatty():
        yield False
        return

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield True
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def _poll_key() -> str | None:
    ready, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not ready:
        return None
    return sys.stdin.read(1)


def _draw_status(
    *,
    trial: TrialSpec,
    stage: str,
    latest_label: int,
    latest_confidence: float,
    latest_proportional: float,
    latest_probs: np.ndarray | None,
    elapsed_s: float,
    message_lines: list[str] | None = None,
) -> None:
    gesture_name = _safe_name(latest_label)
    probs_line = ""
    if latest_probs is not None:
        parts = [f"{name}:{latest_probs[idx] * 100:.0f}%" for idx, name in enumerate(GESTURE_NAMES)]
        probs_line = "  probs: " + "  ".join(parts)

    lines = [
        _bold("-" * 64),
        f"trial: {trial.trial_id}  target={_bold(trial.gesture_name)}  stage={_cyan(stage)}  elapsed={elapsed_s:.2f}s",
        f"pred : label={gesture_name}  conf={latest_confidence:.2f}  prop={latest_proportional:.2f}",
        probs_line or "  probs: n/a",
        _yellow("space=end trial, q=abort session"),
        _bold("-" * 64),
    ]
    if message_lines:
        lines.extend(message_lines)
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


def _wait_for_space(
    *,
    trial: TrialSpec,
    latest_label: int,
    latest_confidence: float,
    latest_proportional: float,
    latest_probs: np.ndarray | None,
    elapsed_s: float,
    prompt_lines: list[str],
) -> bool:
    with _raw_keyboard_mode() as raw_mode:
        while True:
            _draw_status(
                trial=trial,
                stage="done",
                latest_label=latest_label,
                latest_confidence=latest_confidence,
                latest_proportional=latest_proportional,
                latest_probs=latest_probs,
                elapsed_s=elapsed_s,
                message_lines=prompt_lines,
            )
            key = _poll_key() if raw_mode else sys.stdin.read(1)
            if key == "q":
                return False
            if key == " ":
                return True
            time.sleep(0.05)


def _next_gesture_name(trials: list[TrialSpec], current_index: int) -> str | None:
    if current_index + 1 >= len(trials):
        return None
    return trials[current_index + 1].gesture_name


def _write_samples(
    writer: csv.DictWriter,
    *,
    trial: TrialSpec,
    stage: str,
    start_sample_index: int,
    sample_times_s: np.ndarray,
    raw_chunk: np.ndarray,
    filtered_chunk: np.ndarray,
) -> None:
    activity = np.mean(np.abs(filtered_chunk), axis=1)
    for row_idx in range(raw_chunk.shape[0]):
        row = {
            "trial_id": trial.trial_id,
            "gesture_label": trial.gesture_label,
            "gesture_name": trial.gesture_name,
            "repeat_index": trial.repeat_index,
            "stage": stage,
            "sample_index": start_sample_index + row_idx,
            "sample_time_s": f"{sample_times_s[row_idx]:.6f}",
            "activity": f"{activity[row_idx]:.8f}",
        }
        for ch in range(raw_chunk.shape[1]):
            row[f"raw_ch{ch}"] = f"{raw_chunk[row_idx, ch]:.8f}"
            row[f"filtered_ch{ch}"] = f"{filtered_chunk[row_idx, ch]:.8f}"
        writer.writerow(row)


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


def _write_prediction_frame(
    writer: csv.DictWriter,
    *,
    trial: TrialSpec,
    stage: str,
    frame: PredictionFrame,
    raw_probs: np.ndarray,
    proportional: float,
    vote2_label: int = 0,
    vote2_conf: float = 0.0,
    vote3_label: int = 0,
    vote3_conf: float = 0.0,
    vote4_label: int = 0,
    vote4_conf: float = 0.0,
    vote5_label: int = 0,
    vote5_conf: float = 0.0,
) -> None:
    row = {
        "trial_id": trial.trial_id,
        "gesture_label": trial.gesture_label,
        "gesture_name": trial.gesture_name,
        "repeat_index": trial.repeat_index,
        "stage": stage,
        "frame_index": frame.frame_index,
        "prediction_time_s": f"{frame.prediction_time_s:.6f}",
        "raw_label": frame.raw_label,
        "raw_name": _safe_name(frame.raw_label),
        "smoothed_label": frame.smoothed_label,
        "smoothed_name": _safe_name(frame.smoothed_label),
        "confidence": f"{frame.confidence:.6f}",
        "proportional": f"{proportional:.6f}",
        "window_start_sample": frame.window_start_sample,
        "window_end_sample": frame.window_end_sample,
        "vote2_label": vote2_label,
        "vote2_name": _safe_name(vote2_label),
        "vote2_confidence": f"{vote2_conf:.6f}",
        "vote3_label": vote3_label,
        "vote3_name": _safe_name(vote3_label),
        "vote3_confidence": f"{vote3_conf:.6f}",
        "vote4_label": vote4_label,
        "vote4_name": _safe_name(vote4_label),
        "vote4_confidence": f"{vote4_conf:.6f}",
        "vote5_label": vote5_label,
        "vote5_name": _safe_name(vote5_label),
        "vote5_confidence": f"{vote5_conf:.6f}",
    }
    for idx, name in enumerate(GESTURE_NAMES):
        row[f"prob_{name.lower()}"] = f"{raw_probs[idx]:.8f}"
    writer.writerow(row)


def _summary_dict(outcome: TrialOutcome) -> dict[str, object]:
    return asdict(outcome)


def _aggregate_rows(outcomes: list[TrialOutcome]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    valid = [o for o in outcomes if o.latency_ms is not None]

    def build_row(name: str, values: list[float]) -> dict[str, object]:
        arr = np.asarray(values, dtype=float)
        return {
            "gesture_name": name,
            "count": int(arr.size),
            "mean_ms": float(np.mean(arr)),
            "median_ms": float(np.median(arr)),
            "min_ms": float(np.min(arr)),
            "max_ms": float(np.max(arr)),
            "std_ms": float(np.std(arr)),
        }

    for gesture_name in GESTURE_NAMES[1:]:
        values = [o.latency_ms for o in valid if o.gesture_name == gesture_name and o.latency_ms is not None]
        if values:
            rows.append(build_row(gesture_name, values))

    all_values = [o.latency_ms for o in valid if o.latency_ms is not None]
    if all_values:
        rows.append(build_row("ALL", all_values))
    return rows


def _run_trial(
    *,
    reader: BoardReader,
    pipe,
    calibration: dict | None,
    trial: TrialSpec,
    samples_writer: csv.DictWriter,
    frames_writer: csv.DictWriter,
    baseline_s: float,
    tail_s: float,
    min_confidence: float,
    smoothing_frames: int,
    min_active_samples: int,
    max_gap_samples: int,
    pre_onset_search_samples: int,
    onset_lookback_windows: int,
    auto_stop_confidence: float,
    auto_stop_hold_s: float,
    countdown_s: int,
    max_trial_duration_s: float,
    next_gesture_name: str | None,
) -> TrialOutcome:
    filt = OnlineFilter()
    ring = RingBuffer(WINDOW_LEN)
    smoother = PredictionSmoother(smoothing_frames)

    reader.flush()

    raw_trial_chunks: list[np.ndarray] = []
    filtered_trial_chunks: list[np.ndarray] = []
    sample_times: list[np.ndarray] = []
    prediction_frames: list[PredictionFrame] = []

    latest_probs: np.ndarray | None = None
    latest_label = 0
    latest_confidence = 0.0
    latest_proportional = 0.0
    total_samples = 0
    frame_index = 0
    recent_predictions: list[tuple[int, float]] = []

    trial_start = time.monotonic()
    cue_time_s = float(countdown_s)
    stage = "countdown"

    done = False
    aborted = False
    active_end_time: float | None = None
    stop_frame_idx: int | None = None
    trial_status = "unresolved"
    message_lines = build_trial_message_lines(
        gesture_name=trial.gesture_name,
        phase="countdown",
        countdown_value=countdown_s,
    )

    with _raw_keyboard_mode() as raw_mode:
        while not done:
            raw_chunk = reader.read(WINDOW_STEP)
            read_end_time = time.monotonic()
            filtered_chunk = filt.process(raw_chunk)
            ring.push(filtered_chunk)

            chunk_times = (
                (read_end_time - trial_start)
                - ((np.arange(raw_chunk.shape[0] - 1, -1, -1, dtype=float)) / reader.sampling_rate)
            )
            sample_start_index = total_samples
            total_samples += raw_chunk.shape[0]

            raw_trial_chunks.append(raw_chunk.copy())
            filtered_trial_chunks.append(filtered_chunk.copy())
            sample_times.append(chunk_times.copy())

            elapsed_s = read_end_time - trial_start
            countdown_remaining = max(0, countdown_s - int(elapsed_s))
            if stage == "countdown":
                if elapsed_s >= countdown_s:
                    stage = "active"
                    message_lines = build_trial_message_lines(
                        gesture_name=trial.gesture_name,
                        phase="active",
                    )
                else:
                    message_lines = build_trial_message_lines(
                        gesture_name=trial.gesture_name,
                        phase="countdown",
                        countdown_value=countdown_remaining,
                    )

            if stage == "tail" and active_end_time is not None and (read_end_time - active_end_time) >= tail_s:
                done = True

            if stage == "active" and elapsed_s > countdown_s + max_trial_duration_s:
                stage = "tail"
                active_end_time = read_end_time
                done = True
                trial_status = "timeout"

            _write_samples(
                samples_writer,
                trial=trial,
                stage=stage,
                start_sample_index=sample_start_index,
                sample_times_s=chunk_times,
                raw_chunk=raw_chunk,
                filtered_chunk=filtered_chunk,
            )

            if ring.is_full():
                window = ring.get_window().T
                features = compute_features(window)
                raw_label, confidence, probs = predict(
                    pipe,
                    features,
                    threshold=min_confidence,
                )
                smoothed_label = smoother.update(raw_label)
                proportional = prop_mod.compute_proportional(window, smoothed_label, calibration)
                recent_predictions.append((raw_label, float(confidence)))
                v2_l, v2_c = _compute_n_vote(recent_predictions, 2)
                v3_l, v3_c = _compute_n_vote(recent_predictions, 3)
                v4_l, v4_c = _compute_n_vote(recent_predictions, 4)
                v5_l, v5_c = _compute_n_vote(recent_predictions, 5)
                frame = PredictionFrame(
                    frame_index=frame_index,
                    prediction_time_s=elapsed_s,
                    raw_label=raw_label,
                    smoothed_label=smoothed_label,
                    confidence=float(confidence),
                    window_start_sample=max(0, total_samples - WINDOW_LEN),
                    window_end_sample=total_samples,
                )
                frame_index += 1
                prediction_frames.append(frame)
                _write_prediction_frame(
                    frames_writer,
                    trial=trial,
                    stage=stage,
                    frame=frame,
                    raw_probs=probs,
                    proportional=proportional,
                    vote2_label=v2_l,
                    vote2_conf=v2_c,
                    vote3_label=v3_l,
                    vote3_conf=v3_c,
                    vote4_label=v4_l,
                    vote4_conf=v4_c,
                    vote5_label=v5_l,
                    vote5_conf=v5_c,
                )

                latest_probs = probs
                latest_label = smoothed_label
                latest_confidence = float(confidence)
                latest_proportional = float(proportional)

                if stage == "active":
                    sustained_run = find_sustained_prediction_run(
                        prediction_frames,
                        target_label=trial.gesture_label,
                        cue_time_s=cue_time_s,
                        min_confidence=auto_stop_confidence,
                        hold_duration_s=auto_stop_hold_s,
                    )
                    if sustained_run is not None:
                        _, stop_frame_idx = sustained_run
                        stage = "tail"
                        active_end_time = read_end_time
                        done = True
                        trial_status = "ok"

            _draw_status(
                trial=trial,
                stage=stage,
                latest_label=latest_label,
                latest_confidence=latest_confidence,
                latest_proportional=latest_proportional,
                latest_probs=latest_probs,
                elapsed_s=elapsed_s,
                message_lines=message_lines,
            )

            key = _poll_key() if raw_mode else None
            if key == "q":
                aborted = True
                done = True
                active_end_time = read_end_time
            elif key == " " and stage == "active":
                stage = "tail"
                active_end_time = read_end_time
                done = True

    raw_samples = np.vstack(raw_trial_chunks) if raw_trial_chunks else np.zeros((0, 8))
    filtered_samples = np.vstack(filtered_trial_chunks) if filtered_trial_chunks else np.zeros((0, 8))
    relative_sample_times = np.concatenate(sample_times) if sample_times else np.zeros(0)
    end_time_s = active_end_time - trial_start if active_end_time is not None else float(relative_sample_times[-1]) if relative_sample_times.size else 0.0

    if aborted:
        return TrialOutcome(
            trial_id=trial.trial_id,
            gesture_label=trial.gesture_label,
            gesture_name=trial.gesture_name,
            repeat_index=trial.repeat_index,
            cue_time_s=cue_time_s,
            end_time_s=end_time_s,
            status="aborted",
            onset_sample_index=None,
            onset_time_s=None,
            prediction_frame_index=None,
            prediction_time_s=None,
            prediction_window_start_sample=None,
            prediction_window_end_sample=None,
            latency_ms=None,
            threshold=None,
            notes="User aborted session with q",
        )

    measurement = measure_latency(
        raw_samples=filtered_samples,
        frames=prediction_frames,
        target_label=trial.gesture_label,
        cue_time_s=cue_time_s,
        sampling_rate_hz=float(reader.sampling_rate),
        baseline_end_sample=int(max(0.0, cue_time_s - 1.0) * reader.sampling_rate),
        min_confidence=min_confidence,
        smoothing_frames=smoothing_frames,
        min_active_samples=min_active_samples,
        max_gap_samples=max_gap_samples,
        pre_onset_search_samples=pre_onset_search_samples,
        onset_lookback_windows=onset_lookback_windows,
        sample_times_s=relative_sample_times if relative_sample_times.size else None,
        search_start_sample_override=int(max(0.0, cue_time_s - 1.0) * reader.sampling_rate),
        support_end_idx_override=stop_frame_idx,
    )

    if measurement is None:
        outcome = TrialOutcome(
            trial_id=trial.trial_id,
            gesture_label=trial.gesture_label,
            gesture_name=trial.gesture_name,
            repeat_index=trial.repeat_index,
            cue_time_s=cue_time_s,
            end_time_s=end_time_s,
            status=trial_status,
            onset_sample_index=None,
            onset_time_s=None,
            prediction_frame_index=None,
            prediction_time_s=None,
            prediction_window_start_sample=None,
            prediction_window_end_sample=None,
            latency_ms=None,
            threshold=None,
            notes="No supported onset/prediction match found",
        )
    else:
        outcome = TrialOutcome(
            trial_id=trial.trial_id,
            gesture_label=trial.gesture_label,
            gesture_name=trial.gesture_name,
            repeat_index=trial.repeat_index,
            cue_time_s=cue_time_s,
            end_time_s=end_time_s,
            status=trial_status,
            onset_sample_index=measurement.onset_sample_index,
            onset_time_s=measurement.onset_time_s,
            prediction_frame_index=measurement.prediction_frame_index,
            prediction_time_s=measurement.prediction_time_s,
            prediction_window_start_sample=measurement.prediction_window_start_sample,
            prediction_window_end_sample=measurement.prediction_window_end_sample,
            latency_ms=measurement.latency_ms,
            threshold=measurement.threshold,
            notes="",
        )

    prompt_lines = build_trial_message_lines(
        gesture_name=trial.gesture_name,
        phase="done",
        latency_ms=outcome.latency_ms,
        next_gesture_name=next_gesture_name,
        wait_for_continue=next_gesture_name is not None,
        status=outcome.status,
    )
    if next_gesture_name is not None and not _wait_for_space(
        trial=trial,
        latest_label=latest_label,
        latest_confidence=latest_confidence,
        latest_proportional=latest_proportional,
        latest_probs=latest_probs,
        elapsed_s=end_time_s,
        prompt_lines=prompt_lines,
    ):
        return TrialOutcome(
            trial_id=trial.trial_id,
            gesture_label=trial.gesture_label,
            gesture_name=trial.gesture_name,
            repeat_index=trial.repeat_index,
            cue_time_s=cue_time_s,
            end_time_s=end_time_s,
            status="aborted",
            onset_sample_index=outcome.onset_sample_index,
            onset_time_s=outcome.onset_time_s,
            prediction_frame_index=outcome.prediction_frame_index,
            prediction_time_s=outcome.prediction_time_s,
            prediction_window_start_sample=outcome.prediction_window_start_sample,
            prediction_window_end_sample=outcome.prediction_window_end_sample,
            latency_ms=outcome.latency_ms,
            threshold=outcome.threshold,
            notes="User aborted session with q",
        )

    return outcome


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive EMG latency benchmark")
    parser.add_argument("--model-dir", type=Path, required=True, help="Directory containing trained EMG model files")
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory to store CSV outputs")
    parser.add_argument("--repeats", type=int, default=5, help="Number of latency trials per non-REST gesture")
    parser.add_argument("--baseline-s", type=float, default=1.5, help="Baseline/rest duration before cue")
    parser.add_argument("--tail-s", type=float, default=0.5, help="Additional capture time after user ends the gesture")
    parser.add_argument("--threshold", type=float, default=CONFIDENCE_THRESHOLD, help="Classifier confidence threshold")
    parser.add_argument("--smooth", type=int, default=PREDICTION_SMOOTHING_FRAMES, help="Prediction smoothing frames")
    parser.add_argument("--countdown-s", type=int, default=3, help="Rest countdown before each gesture cue")
    parser.add_argument("--auto-stop-confidence", type=float, default=0.70, help="Confidence threshold for automatic trial completion")
    parser.add_argument("--auto-stop-hold-s", type=float, default=1.0, help="How long the target gesture must be held before automatic trial completion")
    parser.add_argument("--max-trial-duration-s", type=float, default=15.0, help="Maximum active-phase duration before timeout (excluding countdown)")
    parser.add_argument("--min-active-samples", type=int, default=6, help="Minimum consecutive active samples for onset detection")
    parser.add_argument("--max-gap-samples", type=int, default=2, help="Maximum inactive gap inside one activation run")
    parser.add_argument("--pre-onset-search-samples", type=int, default=20, help="How many samples before prediction support to search for onset")
    parser.add_argument("--onset-lookback-windows", type=int, default=PREDICTION_SMOOTHING_FRAMES, help="How many recent target-predicting windows to require consistent channel support across")
    parser.add_argument("--ip", type=str, default="", help="MindRove board IP (default: auto-detect)")
    parser.add_argument("--port", type=int, default=4210, help="MindRove board port (default: 4210)")
    args = parser.parse_args()

    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print(_red("The latency benchmark requires an interactive TTY for prompts and keypress capture."))
        sys.exit(2)

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    pipe = clf_mod.load(args.model_dir)
    calibration = prop_mod.load(args.model_dir)

    samples_path = output_dir / "latency_samples.csv"
    frames_path = output_dir / "latency_predictions.csv"
    trials_path = output_dir / "latency_trials.csv"
    aggregate_path = output_dir / "latency_aggregate.csv"
    metadata_path = output_dir / "latency_metadata.json"

    sample_fieldnames = [
        "trial_id",
        "gesture_label",
        "gesture_name",
        "repeat_index",
        "stage",
        "sample_index",
        "sample_time_s",
    ]
    sample_fieldnames.extend([f"raw_ch{idx}" for idx in range(8)])
    sample_fieldnames.extend([f"filtered_ch{idx}" for idx in range(8)])
    sample_fieldnames.append("activity")

    frame_fieldnames = [
        "trial_id",
        "gesture_label",
        "gesture_name",
        "repeat_index",
        "stage",
        "frame_index",
        "prediction_time_s",
        "raw_label",
        "raw_name",
        "smoothed_label",
        "smoothed_name",
        "confidence",
        "proportional",
        "window_start_sample",
        "window_end_sample",
        "vote2_label",
        "vote2_name",
        "vote2_confidence",
        "vote3_label",
        "vote3_name",
        "vote3_confidence",
        "vote4_label",
        "vote4_name",
        "vote4_confidence",
        "vote5_label",
        "vote5_name",
        "vote5_confidence",
    ]
    frame_fieldnames.extend([f"prob_{name.lower()}" for name in GESTURE_NAMES])

    outcomes: list[TrialOutcome] = []
    trials = plan_trials(args.repeats)

    with BoardReader(ip_address=args.ip, ip_port=args.port) as reader, \
        samples_path.open("w", newline="", encoding="utf-8") as samples_file, \
        frames_path.open("w", newline="", encoding="utf-8") as frames_file, \
        trials_path.open("w", newline="", encoding="utf-8") as trials_file:

        samples_writer = csv.DictWriter(samples_file, fieldnames=sample_fieldnames)
        frames_writer = csv.DictWriter(frames_file, fieldnames=frame_fieldnames)
        trials_writer = csv.DictWriter(trials_file, fieldnames=list(TrialOutcome.__dataclass_fields__.keys()))
        samples_writer.writeheader()
        frames_writer.writeheader()
        trials_writer.writeheader()

        print(_green(f"Connected to MindRove board at {reader.sampling_rate} Hz."))
        print(_cyan(f"Running {len(trials)} latency trials across {len(GESTURE_NAMES) - 1} gestures."))
        print(_yellow("Keep the band still during countdown. Press q during a trial to abort the session."))

        if trials:
            first_lines = [
                f"First gesture: {trials[0].gesture_name}",
                "Press space to continue...",
            ]
            if not _wait_for_space(
                trial=trials[0],
                latest_label=0,
                latest_confidence=0.0,
                latest_proportional=0.0,
                latest_probs=None,
                elapsed_s=0.0,
                prompt_lines=first_lines,
            ):
                print(_red("Session aborted before start."))
                sys.exit(0)

        for trial_index, trial in enumerate(trials):
            outcome = _run_trial(
                reader=reader,
                pipe=pipe,
                calibration=calibration,
                trial=trial,
                samples_writer=samples_writer,
                frames_writer=frames_writer,
                baseline_s=args.baseline_s,
                tail_s=args.tail_s,
                min_confidence=args.threshold,
                smoothing_frames=args.smooth,
                min_active_samples=args.min_active_samples,
                max_gap_samples=args.max_gap_samples,
                pre_onset_search_samples=args.pre_onset_search_samples,
                onset_lookback_windows=args.onset_lookback_windows,
                auto_stop_confidence=args.auto_stop_confidence,
                auto_stop_hold_s=args.auto_stop_hold_s,
                countdown_s=args.countdown_s,
                max_trial_duration_s=args.max_trial_duration_s,
                next_gesture_name=_next_gesture_name(trials, trial_index),
            )
            outcomes.append(outcome)
            trials_writer.writerow(_summary_dict(outcome))
            trials_file.flush()

            if outcome.status == "aborted":
                print(_red("Session aborted by user."))
                break
            if outcome.latency_ms is None:
                print(_yellow(f"{trial.trial_id}: unresolved ({outcome.notes})"))
            else:
                print(_green(f"{trial.trial_id}: latency {outcome.latency_ms:.2f} ms"))

    aggregate_rows = _aggregate_rows(outcomes)
    with aggregate_path.open("w", newline="", encoding="utf-8") as aggregate_file:
        writer = csv.DictWriter(
            aggregate_file,
            fieldnames=["gesture_name", "count", "mean_ms", "median_ms", "min_ms", "max_ms", "std_ms"],
        )
        writer.writeheader()
        for row in aggregate_rows:
            writer.writerow(row)

    metadata = {
        "model_dir": str(args.model_dir),
        "output_dir": str(output_dir),
        "repeats": args.repeats,
        "baseline_s": args.baseline_s,
        "tail_s": args.tail_s,
        "threshold": args.threshold,
        "smooth": args.smooth,
        "countdown_s": args.countdown_s,
        "auto_stop_confidence": args.auto_stop_confidence,
        "auto_stop_hold_s": args.auto_stop_hold_s,
        "max_trial_duration_s": args.max_trial_duration_s,
        "onset_lookback_windows": args.onset_lookback_windows,
        "sampling_rate_hz": SAMPLING_RATE,
        "window_len": WINDOW_LEN,
        "window_step": WINDOW_STEP,
        "gesture_names": GESTURE_NAMES,
    }
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    print()
    print(_bold("Latency summary"))
    if not aggregate_rows:
        print(_red("No successful latency measurements were produced."))
    else:
        for row in aggregate_rows:
            print(
                f"  {row['gesture_name']:>10}: count={row['count']} mean={row['mean_ms']:.2f} ms "
                f"median={row['median_ms']:.2f} ms min={row['min_ms']:.2f} ms max={row['max_ms']:.2f} ms"
            )
    print(_green(f"Saved sample log      -> {samples_path}"))
    print(_green(f"Saved prediction log  -> {frames_path}"))
    print(_green(f"Saved trial summary   -> {trials_path}"))
    print(_green(f"Saved aggregate stats -> {aggregate_path}"))
    print(_green(f"Saved metadata        -> {metadata_path}"))
