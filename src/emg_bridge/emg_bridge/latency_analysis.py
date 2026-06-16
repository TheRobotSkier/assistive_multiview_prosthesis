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


def compute_channel_activity(raw_samples: np.ndarray) -> np.ndarray:
    """Compute per-sample activity for each EMG channel."""
    if raw_samples.ndim != 2:
        raise ValueError("raw_samples must have shape (n_samples, n_channels)")
    return np.abs(raw_samples)


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


def find_sustained_prediction_run(
    frames: list[PredictionFrame],
    *,
    target_label: int,
    cue_time_s: float,
    min_confidence: float,
    hold_duration_s: float,
) -> tuple[int, int] | None:
    run_start_idx: int | None = None

    for idx, frame in enumerate(frames):
        matches = (
            frame.prediction_time_s >= cue_time_s
            and frame.smoothed_label == target_label
            and frame.confidence >= min_confidence
        )
        if matches:
            if run_start_idx is None:
                run_start_idx = idx
            run_duration = frame.prediction_time_s - frames[run_start_idx].prediction_time_s
            if run_duration >= hold_duration_s:
                return run_start_idx, idx
            continue

        run_start_idx = None

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


def _window_has_activation(
    activity: np.ndarray,
    *,
    threshold: float,
    start_sample: int,
    end_sample: int,
    min_active_samples: int,
    max_gap_samples: int,
) -> bool:
    return bool(
        _find_active_runs(
            activity,
            threshold=threshold,
            start_sample=start_sample,
            end_sample=end_sample,
            min_active_samples=min_active_samples,
            max_gap_samples=max_gap_samples,
        )
    )


def _derive_consistent_channel_mask(
    channel_activity: np.ndarray,
    *,
    channel_thresholds: np.ndarray,
    frames: list[PredictionFrame],
    prediction_idx: int,
    target_label: int,
    onset_lookback_windows: int,
    min_active_samples: int,
    max_gap_samples: int,
) -> tuple[np.ndarray, int]:
    lookback = max(1, onset_lookback_windows)
    start_idx = max(0, prediction_idx - lookback + 1)
    candidate_frames = [
        frame
        for frame in frames[start_idx : prediction_idx + 1]
        if frame.raw_label == target_label
    ]
    if not candidate_frames:
        return np.zeros(channel_activity.shape[1], dtype=bool), frames[prediction_idx].window_start_sample

    consistent = np.ones(channel_activity.shape[1], dtype=bool)
    for frame in candidate_frames:
        frame_mask = np.array(
            [
                _window_has_activation(
                    channel_activity[:, channel_idx],
                    threshold=float(channel_thresholds[channel_idx]),
                    start_sample=frame.window_start_sample,
                    end_sample=frame.window_end_sample,
                    min_active_samples=min_active_samples,
                    max_gap_samples=max_gap_samples,
                )
                for channel_idx in range(channel_activity.shape[1])
            ],
            dtype=bool,
        )
        consistent &= frame_mask

    return consistent, min(frame.window_start_sample for frame in candidate_frames)


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
    onset_lookback_windows: int | None = None,
    prediction_idx_override: int | None = None,
    support_end_idx_override: int | None = None,
    search_start_sample_override: int | None = None,
    sample_times_s: np.ndarray | None = None,
    threshold_z_score: float = 4.0,
    min_threshold: float = 0.01,
) -> LatencyMeasurement | None:
    """Measure end-to-end latency for one prompted gesture trial."""
    prediction_idx = prediction_idx_override
    if prediction_idx is None:
        prediction_idx = find_first_prediction_frame(
            frames,
            target_label=target_label,
            cue_time_s=cue_time_s,
            min_confidence=min_confidence,
        )
    if prediction_idx is None:
        return None

    activity = compute_sample_activity(raw_samples)
    channel_activity = compute_channel_activity(raw_samples)
    baseline = activity[: max(0, baseline_end_sample)]
    threshold = build_activation_threshold(
        baseline,
        z_score=threshold_z_score,
        min_threshold=min_threshold,
    )
    channel_thresholds = np.array(
        [
            build_activation_threshold(
                channel_activity[: max(0, baseline_end_sample), channel_idx],
                z_score=threshold_z_score,
                min_threshold=min_threshold,
            )
            for channel_idx in range(channel_activity.shape[1])
        ],
        dtype=float,
    )

    support_start, support_end = derive_prediction_support_range(
        frames,
        prediction_idx=prediction_idx,
        smoothing_frames=smoothing_frames,
    )
    if support_end_idx_override is not None:
        support_end = frames[support_end_idx_override].window_end_sample
    consistent_channels, lookback_start = _derive_consistent_channel_mask(
        channel_activity,
        channel_thresholds=channel_thresholds,
        frames=frames,
        prediction_idx=prediction_idx,
        target_label=target_label,
        onset_lookback_windows=(
            smoothing_frames if onset_lookback_windows is None or onset_lookback_windows <= 0 else onset_lookback_windows
        ),
        min_active_samples=min_active_samples,
        max_gap_samples=max_gap_samples,
    )
    if not np.any(consistent_channels):
        return None

    search_start = max(0, lookback_start - max(pre_onset_search_samples, 0))
    if search_start_sample_override is not None:
        search_start = max(0, search_start_sample_override)
    consistent_activity = np.max(channel_activity[:, consistent_channels], axis=1)
    consistent_threshold = float(np.min(channel_thresholds[consistent_channels]))

    onset_idx = find_supporting_onset(
        consistent_activity,
        threshold=consistent_threshold,
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
