"""Pure helpers for EMG latency measurement.

The live latency runner records raw samples and classifier frames. This module
provides deterministic, testable logic for turning that trace into trial-level
latency measurements.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PredictionFrame:
    frame_index: int
    prediction_time_s: float
    raw_label: int
    smoothed_label: int
    confidence: float
    window_start_sample: int
    window_end_sample: int


@dataclass(frozen=True)
class LatencyMeasurement:
    onset_sample_index: int
    onset_time_s: float
    prediction_frame_index: int
    prediction_time_s: float
    prediction_window_start_sample: int
    prediction_window_end_sample: int
    latency_ms: float
    threshold: float


def compute_sample_activity(raw_samples: np.ndarray) -> np.ndarray:
    """Compute a per-sample scalar activity signal from multichannel EMG."""
    if raw_samples.ndim != 2:
        raise ValueError("raw_samples must have shape (n_samples, n_channels)")
    return np.mean(np.abs(raw_samples), axis=1)


def build_activation_threshold(
    baseline_activity: np.ndarray,
    *,
    z_score: float = 4.0,
    min_threshold: float = 0.01,
) -> float:
    """Build a robust activation threshold from quiet-trial baseline activity."""
    if baseline_activity.size == 0:
        return min_threshold

    baseline = np.asarray(baseline_activity, dtype=float)
    median = float(np.median(baseline))
    mad = float(np.median(np.abs(baseline - median)))
    robust_sigma = 1.4826 * mad
    threshold = median + (z_score * robust_sigma)
    return float(max(min_threshold, threshold))


def find_first_prediction_frame(
    frames: list[PredictionFrame],
    *,
    target_label: int,
    cue_time_s: float,
    min_confidence: float,
) -> int | None:
    """Return the first prediction frame that matches the target gesture."""
    for idx, frame in enumerate(frames):
        if frame.prediction_time_s < cue_time_s:
            continue
        if frame.smoothed_label != target_label:
            continue
        if frame.confidence < min_confidence:
            continue
        return idx
    return None


def derive_prediction_support_range(
    frames: list[PredictionFrame],
    *,
    prediction_idx: int,
    smoothing_frames: int,
) -> tuple[int, int]:
    """Estimate the sample range that supports the accepted prediction.

    The smoothed output depends on a history of recent raw predictions. We map
    that history back into the earliest window start and latest window end.
    """
    if prediction_idx < 0 or prediction_idx >= len(frames):
        raise IndexError("prediction_idx out of range")

    support_start_idx = max(0, prediction_idx - max(smoothing_frames - 1, 0))
    support_frames = frames[support_start_idx : prediction_idx + 1]
    support_start = min(frame.window_start_sample for frame in support_frames)
    support_end = max(frame.window_end_sample for frame in support_frames)
    return support_start, support_end


def _find_active_runs(
    activity: np.ndarray,
    *,
    threshold: float,
    start_sample: int,
    end_sample: int,
    min_active_samples: int,
    max_gap_samples: int,
) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    run_start: int | None = None
    last_active: int | None = None

    for idx in range(start_sample, min(end_sample, len(activity))):
        is_active = activity[idx] >= threshold
        if is_active:
            if run_start is None:
                run_start = idx
            elif last_active is not None and (idx - last_active - 1) > max_gap_samples:
                if (last_active - run_start + 1) >= min_active_samples:
                    runs.append((run_start, last_active))
                run_start = idx
            last_active = idx
            continue

        if run_start is not None and last_active is not None:
            if (idx - last_active - 1) > max_gap_samples:
                if (last_active - run_start + 1) >= min_active_samples:
                    runs.append((run_start, last_active))
                run_start = None
                last_active = None

    if run_start is not None and last_active is not None:
        if (last_active - run_start + 1) >= min_active_samples:
            runs.append((run_start, last_active))

    return runs


def find_supporting_onset(
    activity: np.ndarray,
    *,
    threshold: float,
    search_start_sample: int,
    support_start_sample: int,
    support_end_sample: int,
    min_active_samples: int,
    max_gap_samples: int,
) -> int | None:
    """Find the earliest activation onset that overlaps prediction support."""
    runs = _find_active_runs(
        activity,
        threshold=threshold,
        start_sample=search_start_sample,
        end_sample=support_end_sample,
        min_active_samples=min_active_samples,
        max_gap_samples=max_gap_samples,
    )

    for run_start, run_end in runs:
        if run_end >= support_start_sample:
            return run_start
    return None


def measure_latency(
    *,
    raw_samples: np.ndarray,
    frames: list[PredictionFrame],
    target_label: int,
    cue_time_s: float,
    sampling_rate_hz: float,
    baseline_end_sample: int,
    min_confidence: float,
    smoothing_frames: int,
    min_active_samples: int,
    max_gap_samples: int,
    pre_onset_search_samples: int,
    sample_times_s: np.ndarray | None = None,
    threshold_z_score: float = 4.0,
    min_threshold: float = 0.01,
) -> LatencyMeasurement | None:
    """Measure end-to-end latency for one prompted gesture trial."""
    prediction_idx = find_first_prediction_frame(
        frames,
        target_label=target_label,
        cue_time_s=cue_time_s,
        min_confidence=min_confidence,
    )
    if prediction_idx is None:
        return None

    activity = compute_sample_activity(raw_samples)
    baseline = activity[: max(0, baseline_end_sample)]
    threshold = build_activation_threshold(
        baseline,
        z_score=threshold_z_score,
        min_threshold=min_threshold,
    )

    support_start, support_end = derive_prediction_support_range(
        frames,
        prediction_idx=prediction_idx,
        smoothing_frames=smoothing_frames,
    )
    search_start = max(0, support_start - max(pre_onset_search_samples, 0))

    onset_idx = find_supporting_onset(
        activity,
        threshold=threshold,
        search_start_sample=search_start,
        support_start_sample=support_start,
        support_end_sample=support_end,
        min_active_samples=min_active_samples,
        max_gap_samples=max_gap_samples,
    )
    if onset_idx is None:
        return None

    prediction_frame = frames[prediction_idx]
    if sample_times_s is not None and onset_idx < len(sample_times_s):
        onset_time_s = float(sample_times_s[onset_idx])
    else:
        onset_time_s = onset_idx / sampling_rate_hz
    latency_ms = (prediction_frame.prediction_time_s - onset_time_s) * 1000.0

    return LatencyMeasurement(
        onset_sample_index=onset_idx,
        onset_time_s=onset_time_s,
        prediction_frame_index=prediction_frame.frame_index,
        prediction_time_s=prediction_frame.prediction_time_s,
        prediction_window_start_sample=prediction_frame.window_start_sample,
        prediction_window_end_sample=prediction_frame.window_end_sample,
        latency_ms=latency_ms,
        threshold=threshold,
    )
