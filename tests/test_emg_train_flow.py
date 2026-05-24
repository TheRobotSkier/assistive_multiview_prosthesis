"""Regression tests for the EMG train/test handoff flow."""

from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "emg_bridge" / "scripts"))

import collect_data_flow


def test_prompt_first_gesture_waits_for_enter() -> None:
    prompts: list[str] = []

    collect_data_flow.prepare_for_recording(
        gesture_name="REST",
        is_first_recording=True,
        auto_advance_delay_s=3,
        input_func=lambda prompt: prompts.append(prompt),
        sleep_func=lambda _: None,
        print_func=lambda *args, **kwargs: None,
        colorize_warning=lambda text: text,
        colorize_info=lambda text: text,
        colorize_bold=lambda text: text,
    )

    assert prompts == ["  Press ENTER to start recording ..."]


def test_prompt_later_gesture_announces_and_counts_down() -> None:
    events: list[tuple[str, str]] = []

    collect_data_flow.prepare_for_recording(
        gesture_name="POWER",
        is_first_recording=False,
        auto_advance_delay_s=3,
        input_func=lambda prompt: events.append(("input", prompt)),
        sleep_func=lambda seconds: events.append(("sleep", f"{seconds}")),
        print_func=lambda *args, **kwargs: events.append(("print", " ".join(str(arg) for arg in args))),
        colorize_warning=lambda text: text,
        colorize_info=lambda text: text,
        colorize_bold=lambda text: text,
    )

    assert not any(kind == "input" for kind, _ in events)
    printed = [message for kind, message in events if kind == "print"]
    assert any("Next gesture: POWER" in message for message in printed)
    assert any("POWER starts in 3 s" in message for message in printed)
    assert any("POWER starts in 2 s" in message for message in printed)
    assert any("POWER starts in 1 s" in message for message in printed)
    assert [value for kind, value in events if kind == "sleep"] == ["1.0", "1.0", "1.0"]


def test_makefile_waits_for_enter_before_auto_launch() -> None:
    makefile_text = (REPO_ROOT / "Makefile").read_text()
    target_start = makefile_text.index("up-grasp-test-train:")
    target_end = makefile_text.index("print-force:", target_start)
    target_body = makefile_text[target_start:target_end]

    assert "Press ENTER to launch the live EMG grasp test" in target_body
    assert "read -r dummy" in target_body
    assert "$(MAKE) up-grasp-test" in target_body
    assert "--profile grasp_test up -d grasp_test" in makefile_text


def test_makefile_has_emg_only_train_target() -> None:
    makefile_text = (REPO_ROOT / "Makefile").read_text()
    target_start = makefile_text.index("up-emg-test-train:")
    target_end = makefile_text.index("print-force:", target_start)
    target_body = makefile_text[target_start:target_end]

    assert "EMG_POST_TRAIN_MODE=classifier" in target_body
    assert "$(DOCKER_CMD) run --rm -it --name emg_test_train" in target_body
    assert "$(MAKE) up-grasp-test" not in target_body


def test_emg_train_script_can_continue_into_classifier_mode() -> None:
    script_text = (REPO_ROOT / "scripts" / "emg_train_and_test.sh").read_text()

    assert 'EMG_POST_TRAIN_MODE="${EMG_POST_TRAIN_MODE:-grasp-test}"' in script_text
    assert 'ros2 run emg_bridge run_classifier \\' in script_text
