"""Tests for the live EMG classifier retry wrapper."""

from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "emg_bridge" / "scripts"))

import run_classifier_retry


def test_external_emg_requires_publishers() -> None:
    topic_stats = {
        "/emg/gesture_label": (0, 1),
        "/emg/confidence": (0, 1),
    }

    assert (
        run_classifier_retry.external_emg_available(
            topic_info_func=lambda topic: topic_stats[topic],
            topics={"/emg/gesture_label", "/emg/confidence"},
        )
        is False
    )
