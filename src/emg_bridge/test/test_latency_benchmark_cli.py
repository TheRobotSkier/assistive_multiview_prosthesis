#!/usr/bin/env python3
"""Unit tests for latency benchmark CLI helpers."""

import os
import sys


sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..'
))

from emg_bridge.latency_protocol import plan_trials  # noqa: E402


def test_plan_trials_uses_all_non_rest_gestures_for_each_repeat():
    trials = plan_trials(3)

    assert len(trials) == 12
    assert trials[0].gesture_name == "POWER"
    assert trials[0].repeat_index == 1
    assert trials[-1].gesture_name == "EXTENSION"
    assert trials[-1].repeat_index == 3
