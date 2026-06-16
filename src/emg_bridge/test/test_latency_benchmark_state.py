#!/usr/bin/env python3

import os
import sys


sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..'
))

from emg_bridge.latency_benchmark_state import build_trial_message_lines  # noqa: E402


def test_build_trial_message_lines_shows_countdown_and_target_name():
    assert build_trial_message_lines(
        gesture_name="POWER",
        phase="countdown",
        countdown_value=3,
    ) == [
        "POWER gesture in...",
        "3...",
    ]

    assert build_trial_message_lines(
        gesture_name="POWER",
        phase="active",
    ) == ["POWER"]


def test_build_trial_message_lines_shows_done_and_next_prompt():
    assert build_trial_message_lines(
        gesture_name="POWER",
        phase="done",
        latency_ms=185.4,
        next_gesture_name="PINCH",
        wait_for_continue=True,
    ) == [
        "DONE",
        "onset: 185.4 ms",
        "Next gesture: PINCH",
        "Press space to continue...",
    ]


def test_build_trial_message_lines_shows_timeout_message():
    lines = build_trial_message_lines(
        gesture_name="POWER",
        phase="done",
        status="timeout",
        next_gesture_name="PINCH",
        wait_for_continue=True,
    )
    assert lines[0] == "TIMEOUT — no gesture detected"


def test_build_trial_message_lines_shows_unresolved_message():
    lines = build_trial_message_lines(
        gesture_name="POWER",
        phase="done",
        status="unresolved",
        next_gesture_name="PINCH",
        wait_for_continue=True,
    )
    assert lines[0] == "DONE — onset could not be resolved"
