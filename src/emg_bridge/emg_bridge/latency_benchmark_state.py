"""Pure helpers for latency benchmark trial messaging and stop logic."""

from __future__ import annotations


def build_trial_message_lines(
    *,
    gesture_name: str,
    phase: str,
    countdown_value: int | None = None,
    latency_ms: float | None = None,
    next_gesture_name: str | None = None,
    wait_for_continue: bool = False,
    status: str = "ok",
) -> list[str]:
    if phase == "countdown":
        return [f"{gesture_name} gesture in...", f"{countdown_value}..."]

    if phase == "active":
        return [gesture_name]

    if phase == "done":
        if status == "timeout":
            lines = ["TIMEOUT — no gesture detected"]
        elif status == "unresolved":
            lines = ["DONE — onset could not be resolved"]
        elif status == "ok":
            lines = ["DONE"]
        else:
            lines = ["DONE"]

        if latency_ms is not None:
            lines.append(f"onset: {latency_ms:.1f} ms")
        else:
            lines.append("onset: unresolved")
        if next_gesture_name is not None:
            lines.append(f"Next gesture: {next_gesture_name}")
        if wait_for_continue:
            lines.append("Press space to continue...")
        return lines

    return []
