"""Pure protocol helpers for the EMG latency benchmark."""

from __future__ import annotations

from dataclasses import dataclass

from .config import GESTURE_NAMES


@dataclass(frozen=True)
class TrialSpec:
    trial_id: str
    gesture_label: int
    gesture_name: str
    repeat_index: int


def plan_trials(repeats: int) -> list[TrialSpec]:
    trials: list[TrialSpec] = []
    for gesture_label, gesture_name in enumerate(GESTURE_NAMES):
        if gesture_label == 0:
            continue
        for repeat_index in range(1, repeats + 1):
            trials.append(
                TrialSpec(
                    trial_id=f"{gesture_name.lower()}_{repeat_index:02d}",
                    gesture_label=gesture_label,
                    gesture_name=gesture_name,
                    repeat_index=repeat_index,
                )
            )
    return trials
