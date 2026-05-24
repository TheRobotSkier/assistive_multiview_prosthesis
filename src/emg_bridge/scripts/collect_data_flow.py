"""Interactive flow helpers for EMG data collection."""

from __future__ import annotations

from typing import Callable


PromptFunc = Callable[[str], object]
SleepFunc = Callable[[float], None]
PrintFunc = Callable[..., None]
ColorizeFunc = Callable[[str], str]


def prepare_for_recording(
    *,
    gesture_name: str,
    is_first_recording: bool,
    auto_advance_delay_s: int,
    input_func: PromptFunc,
    sleep_func: SleepFunc,
    print_func: PrintFunc,
    colorize_warning: ColorizeFunc,
    colorize_info: ColorizeFunc,
    colorize_bold: ColorizeFunc,
) -> None:
    if is_first_recording:
        input_func(colorize_warning("  Press ENTER to start recording ..."))
        return

    print_func(colorize_info(f"  Next gesture: {colorize_bold(gesture_name)}"))
    for seconds_remaining in range(auto_advance_delay_s, 0, -1):
        print_func(
            colorize_warning(
                f"  {colorize_bold(gesture_name)} starts in {seconds_remaining} s ..."
            ),
            flush=True,
        )
        sleep_func(1.0)
