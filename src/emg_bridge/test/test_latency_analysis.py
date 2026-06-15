#!/usr/bin/env python3
"""Unit tests for EMG latency analysis helpers.

These tests cover the pure logic used by the interactive EMG latency runner,
without requiring ROS 2, MindRove hardware, or the live classifier loop.
"""

import os
import sys

import numpy as np


sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'emg_bridge'
))

from latency_analysis import (  # noqa: E402
    PredictionFrame,
    build_activation_threshold,
    compute_sample_activity,
    derive_prediction_support_range,
    find_first_prediction_frame,
    find_supporting_onset,
    measure_latency,
)


def _frame(
    *,
    frame_index: int,
    prediction_time_s: float,
    raw_label: int,
    smoothed_label: int,
    confidence: float,
    window_start_sample: int,
    window_end_sample: int,
) -> PredictionFrame:
    return PredictionFrame(
        frame_index=frame_index,
        prediction_time_s=prediction_time_s,
        raw_label=raw_label,
        smoothed_label=smoothed_label,
        confidence=confidence,
        window_start_sample=window_start_sample,
        window_end_sample=window_end_sample,
    )


def test_compute_sample_activity_uses_mean_absolute_value_per_sample():
    samples = np.array([
        [1.0, -1.0, 3.0],
        [2.0, -4.0, 0.0],
        [-3.0, 3.0, -6.0],
    ])

    activity = compute_sample_activity(samples)

    assert np.allclose(activity, np.array([5.0 / 3.0, 2.0, 4.0]))


def test_build_activation_threshold_rises_above_quiet_baseline():
    baseline = np.array([0.01, 0.011, 0.012, 0.013, 0.0115, 0.0125])

    threshold = build_activation_threshold(baseline, z_score=4.0, min_threshold=0.005)

    assert threshold > baseline.max()
    assert threshold > 0.005


def test_find_first_prediction_frame_filters_by_cue_time_and_confidence():
    frames = [
        _frame(frame_index=0, prediction_time_s=0.10, raw_label=1, smoothed_label=1, confidence=0.90, window_start_sample=0, window_end_sample=100),
        _frame(frame_index=1, prediction_time_s=0.40, raw_label=1, smoothed_label=1, confidence=0.40, window_start_sample=50, window_end_sample=150),
        _frame(frame_index=2, prediction_time_s=0.60, raw_label=1, smoothed_label=1, confidence=0.91, window_start_sample=100, window_end_sample=200),
    ]

    prediction_idx = find_first_prediction_frame(
        frames,
        target_label=1,
        cue_time_s=0.30,
        min_confidence=0.55,
    )

    assert prediction_idx == 2


def test_derive_prediction_support_range_uses_smoothing_window():
    frames = [
        _frame(frame_index=i, prediction_time_s=i * 0.1, raw_label=0, smoothed_label=0, confidence=0.8, window_start_sample=i * 50, window_end_sample=(i * 50) + 100)
        for i in range(7)
    ]

    support_start, support_end = derive_prediction_support_range(frames, prediction_idx=6, smoothing_frames=5)

    assert support_start == frames[2].window_start_sample
    assert support_end == frames[6].window_end_sample


def test_find_supporting_onset_ignores_unrelated_early_spike():
    activity = np.zeros(40)
    activity[3:5] = 1.0
    activity[18:27] = 1.3

    onset_idx = find_supporting_onset(
        activity,
        threshold=0.8,
        search_start_sample=0,
        support_start_sample=20,
        support_end_sample=30,
        min_active_samples=4,
        max_gap_samples=1,
    )

    assert onset_idx == 18


def test_measure_latency_links_onset_to_prediction_support_window():
    raw_samples = np.zeros((60, 2), dtype=float)
    raw_samples[5:8, :] = 2.5
    raw_samples[22:34, :] = 3.0

    frames = [
        _frame(frame_index=0, prediction_time_s=0.10, raw_label=0, smoothed_label=0, confidence=0.9, window_start_sample=0, window_end_sample=20),
        _frame(frame_index=1, prediction_time_s=0.20, raw_label=0, smoothed_label=0, confidence=0.9, window_start_sample=10, window_end_sample=30),
        _frame(frame_index=2, prediction_time_s=0.30, raw_label=1, smoothed_label=0, confidence=0.8, window_start_sample=20, window_end_sample=40),
        _frame(frame_index=3, prediction_time_s=0.40, raw_label=1, smoothed_label=1, confidence=0.9, window_start_sample=30, window_end_sample=50),
    ]

    result = measure_latency(
        raw_samples=raw_samples,
        frames=frames,
        target_label=1,
        cue_time_s=0.15,
        sampling_rate_hz=100.0,
        baseline_end_sample=20,
        min_confidence=0.55,
        smoothing_frames=2,
        min_active_samples=4,
        max_gap_samples=1,
        pre_onset_search_samples=8,
    )

    assert result is not None
    assert result.onset_sample_index == 22
    assert result.prediction_frame_index == 3
    assert abs(result.latency_ms - 180.0) < 1e-6
