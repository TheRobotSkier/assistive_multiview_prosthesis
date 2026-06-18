"""Pure helpers for the terminal UI dashboard.

The split-node ``terminal_ui_node`` and the offline test suite both need to
render the same fixed-width ANSI dashboard.  This module keeps the layout
pure: it takes a ``UiSnapshot`` and returns a string.  No ROS, no I/O —
that makes it easy to snapshot-test.

The dashboard has three columns:

* a top banner with config name and run id;
* a stage/progress line that surfaces the current ``Stage`` value, any
  countdown for timed stages, and the control hint;
* three cards (EMG, Force, Wrist) followed by a compact 8-motor haptic
  ring rendered as a 4-row octagon.

Colours follow the legacy terminal UI: dim when stale or below
confidence, green for healthy, yellow for active adjustment, red for
faults.  When ``color=False`` the renderer emits plain ASCII so CI
captures are readable.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

from .constants import FINGER_LABELS, MOTOR_COUNT


# ── ANSI helpers ───────────────────────────────────────────────────────────

ANSI_BOLD = "\033[1m"
ANSI_DIM = "\033[2m"
ANSI_GREEN = "\033[92m"
ANSI_YELLOW = "\033[93m"
ANSI_RED = "\033[91m"
ANSI_CYAN = "\033[96m"
ANSI_MAGENTA = "\033[95m"
ANSI_BLUE = "\033[94m"
ANSI_RESET = "\033[0m"
ANSI_CLEAR = "\033[2J\033[H"
ANSI_HIDE_CURSOR = "\033[?25l"
ANSI_SHOW_CURSOR = "\033[?25h"


# ── Color gradient (blue → cyan → green → yellow → red) ────────────────────

_GRADIENT_STOPS: tuple[tuple[float, tuple[int, int, int]], ...] = (
    (0.00, (30, 80, 200)),    # blue
    (0.30, (30, 180, 220)),   # cyan
    (0.55, (60, 200, 80)),    # green
    (0.80, (240, 180, 40)),   # yellow
    (1.00, (220, 50, 40)),    # red
)


def _interp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _gradient_rgb(fraction: float) -> tuple[int, int, int]:
    f = max(0.0, min(1.0, float(fraction)))
    for i in range(len(_GRADIENT_STOPS) - 1):
        f0, c0 = _GRADIENT_STOPS[i]
        f1, c1 = _GRADIENT_STOPS[i + 1]
        if f0 <= f <= f1:
            t = (f - f0) / max(f1 - f0, 1e-9)
            return (
                int(_interp(c0[0], c1[0], t)),
                int(_interp(c0[1], c1[1], t)),
                int(_interp(c0[2], c1[2], t)),
            )
    last = _GRADIENT_STOPS[-1][1]
    return last


def _ansi_bg(rgb: tuple[int, int, int]) -> str:
    return f"\033[48;2;{rgb[0]};{rgb[1]};{rgb[2]}m"


# ── Snapshot dataclass ────────────────────────────────────────────────────


@dataclass(frozen=True)
class UiSnapshot:
    """A single frame of UI state.

    The terminal UI node populates this from its DDS callbacks; the offline
    test suite builds one from a topic snapshot.  Fields default to safe
    "no data" values so partial snapshots still render.
    """

    config_name: str = ""
    run_id: str = ""

    stage: str = "initialising"
    stage_elapsed_s: float = 0.0
    stage_countdown_s: float | None = None
    mode: str = "position"
    hold_mode: str = "force"
    enable: bool = False
    control_hint: str = ""

    gesture: str = "REST"
    gesture_label: int = 0
    confidence: float = 0.0
    proportional: float = 0.0
    emg_age_s: float = -1.0
    confidence_threshold: float = 0.55

    forces: tuple[float, ...] = (0.0, 0.0, 0.0)
    target_forces: tuple[float, ...] = (0.0, 0.0, 0.0)
    force_errors: tuple[float, ...] = (0.0, 0.0, 0.0)
    force_min: float = 50.0
    force_max: float = 500.0

    joint_positions: tuple[float, ...] = (0.0, 0.0, 0.0)
    joint_velocities: tuple[float, ...] = (0.0, 0.0, 0.0)
    velocity_commands: tuple[float, ...] = (0.0, 0.0, 0.0)

    wrist_position_deg: float = 0.0
    wrist_velocity_deg_s: float = 0.0
    wrist_target_deg: float = 0.0

    haptics: tuple[float, ...] = (0.0,) * MOTOR_COUNT
    haptic_phase: str = "idle_zero"

    contact_reason: str = ""
    fault_reason: str = ""

    adjust_active: bool = False
    adjust_arrow: str = ""
    adjust_delta: float = 0.0
    adjust_label: str = ""


# ── Helpers ────────────────────────────────────────────────────────────────


def _bar(value: float, lo: float, hi: float, width: int = 16) -> str:
    """Render a small horizontal bar showing *value* within [lo, hi]."""
    if hi <= lo:
        return "[" + " " * width + "]"
    frac = max(0.0, min(1.0, (value - lo) / (hi - lo)))
    fill = int(round(frac * width))
    return "[" + "#" * fill + "-" * (width - fill) + "]"


def _fmt_force(value: float) -> str:
    return f"{value:6.1f}"


def _fmt_angle(value: float) -> str:
    return f"{value:6.1f}\u00b0"


def _color_for_emg(conf: float, threshold: float, color: bool) -> str:
    if not color:
        return ""
    if conf >= threshold:
        return ANSI_GREEN
    return ANSI_DIM


def _color_for_haptic(fraction: float, color: bool) -> str:
    if not color:
        return ""
    rgb = _gradient_rgb(fraction)
    return _ansi_bg(rgb)


# ── Haptic ring ───────────────────────────────────────────────────────────


def render_haptic_ring(
    motor_pct: Sequence[float], *, color: bool = True, width: int = 2
) -> list[str]:
    """Render the 8-motor haptic band as four lines.

    Layout (motor indices shown):

        0  1
      7      2
      6      3
        5  4

    Each cell is rendered as a coloured block whose background follows the
    blue→amber→red gradient.  When ``color=False`` the cells are plain
    ASCII so the snapshot is readable in CI logs.
    """
    if len(motor_pct) < MOTOR_COUNT:
        motor_pct = list(motor_pct) + [0.0] * (MOTOR_COUNT - len(motor_pct))
    else:
        motor_pct = list(motor_pct[:MOTOR_COUNT])

    cells: list[tuple[str, str]] = []  # (ansi_prefix, plain_text)
    reset = ANSI_RESET if color else ""
    for value in motor_pct:
        frac = max(0.0, min(100.0, float(value))) / 100.0
        if color:
            prefix = _color_for_haptic(frac, color=True)
            text = "  "  # two spaces
        else:
            prefix = ""
            text = ".." if value <= 0.0 else "##"
        cells.append((prefix, text))

    # Octagon layout
    def cell(idx: int) -> str:
        prefix, text = cells[idx]
        return f"{prefix}{text}{reset}"

    pad = " " * width
    line1 = f"{pad}{cell(0)}  {cell(1)}"
    line2 = f"{cell(7)}{pad}{cell(2)}"
    line3 = f"{cell(6)}{pad}{cell(3)}"
    line4 = f"{pad}{cell(5)}  {cell(4)}"
    return [line1, line2, line3, line4]


# ── Cards ─────────────────────────────────────────────────────────────────


def _emg_card(snapshot: UiSnapshot, color: bool, width: int) -> list[str]:
    title = f"{ANSI_BOLD}EMG{ANSI_RESET}" if color else "EMG"
    gesture_color = _color_for_emg(
        snapshot.confidence, snapshot.confidence_threshold, color
    )
    gesture_text = f"{gesture_color}{snapshot.gesture}{ANSI_RESET if color else ''}"
    age = (
        f"{snapshot.emg_age_s:.3f}s"
        if snapshot.emg_age_s >= 0.0
        else "n/a"
    )
    lines = [
        f"{title}  {gesture_text}  label={snapshot.gesture_label}",
        f"  conf={snapshot.confidence:.2f}  prop={snapshot.proportional:.2f}"
        f"  age={age}",
    ]
    return [line.ljust(width) for line in lines]


def _force_card(snapshot: UiSnapshot, color: bool, width: int) -> list[str]:
    title = f"{ANSI_BOLD}FORCE{ANSI_RESET}" if color else "FORCE"
    fc = len(FINGER_LABELS)
    forces = list(snapshot.forces)[:fc] + [0.0] * (fc - len(snapshot.forces))
    targets = list(snapshot.target_forces)[:fc] + [0.0] * (fc - len(snapshot.target_forces))
    errors = list(snapshot.force_errors)[:fc] + [0.0] * (fc - len(snapshot.force_errors))
    avg_f = sum(forces) / fc if fc else 0.0
    avg_t = sum(targets) / fc if fc else 0.0
    fpct = 0.0
    if snapshot.force_max > snapshot.force_min:
        fpct = max(0.0, min(100.0, (avg_f - snapshot.force_min) /
                             (snapshot.force_max - snapshot.force_min) * 100.0))
    lines = [
        f"{title}  avg={avg_f:.1f}N  target={avg_t:.1f}N  "
        f"haptic={fpct:.0f}%",
        f"  per-finger  {', '.join(_fmt_force(f) for f in forces)}",
    ]
    for i, label in enumerate(FINGER_LABELS):
        bar = _bar(targets[i], snapshot.force_min, snapshot.force_max, width=10)
        lines.append(
            f"  {label:<5s}  t={_fmt_force(targets[i])}  f={_fmt_force(forces[i])}"
            f"  err={errors[i]:+6.1f}  {bar}"
        )
    return [line.ljust(width) for line in lines]


def _wrist_card(snapshot: UiSnapshot, color: bool, width: int) -> list[str]:
    title = f"{ANSI_BOLD}WRIST{ANSI_RESET}" if color else "WRIST"
    err = snapshot.wrist_target_deg - snapshot.wrist_position_deg
    lines = [
        f"{title}  pos={_fmt_angle(snapshot.wrist_position_deg)}"
        f"  target={_fmt_angle(snapshot.wrist_target_deg)}"
        f"  vel={snapshot.wrist_velocity_deg_s:+5.1f}\u00b0/s"
        f"  err={err:+5.1f}\u00b0",
    ]
    return [line.ljust(width) for line in lines]


def _control_card(snapshot: UiSnapshot, color: bool, width: int) -> list[str]:
    title = f"{ANSI_BOLD}CONTROL{ANSI_RESET}" if color else "CONTROL"
    enable_text = (
        f"{ANSI_GREEN}enabled{ANSI_RESET}"
        if (color and snapshot.enable)
        else (
            f"{ANSI_DIM}disabled{ANSI_RESET}"
            if color
            else ("enabled" if snapshot.enable else "disabled")
        )
    )
    hold_color = ANSI_CYAN if color else ""
    reset = ANSI_RESET if color else ""
    lines = [
        f"{title}  mode={snapshot.mode}  hold={hold_color}{snapshot.hold_mode}{reset}"
        f"  ctrl={enable_text}",
    ]
    if snapshot.adjust_active and color:
        lines.append(
            f"  {ANSI_YELLOW}{snapshot.adjust_arrow} {snapshot.adjust_label}"
            f"  \u0394={snapshot.adjust_delta:+.1f}/s{ANSI_RESET}"
        )
    elif snapshot.adjust_active:
        lines.append(
            f"  {snapshot.adjust_arrow} {snapshot.adjust_label}"
            f"  \u0394={snapshot.adjust_delta:+.1f}/s"
        )
    elif snapshot.stage == "force_hold":
        lines.append(
            f"  adjust idle (need conf \u2265 "
            f"{snapshot.confidence_threshold:.2f})"
        )
    return [line.ljust(width) for line in lines]


# ── Full frame ────────────────────────────────────────────────────────────


def render_terminal_frame(
    snapshot: UiSnapshot, *, cols: int | None = None, color: bool = True
) -> str:
    """Render a complete dashboard frame for *snapshot*.

    *cols* is the terminal width in characters; when omitted, defaults to
    64 (the minimum readable width for the 4-row haptic ring plus cards).
    *color* toggles ANSI escape sequences.
    """
    width = max(48, int(cols) if cols else 64)
    bar = "=" * width
    sep = "-" * width

    title = f"{ANSI_BOLD}Mia Hand EMG Haptic Force Test{ANSI_RESET}" if color else "Mia Hand EMG Haptic Force Test"
    subtitle_parts = []
    if snapshot.config_name:
        subtitle_parts.append(f"config: {snapshot.config_name}")
    if snapshot.run_id:
        subtitle_parts.append(f"run: {snapshot.run_id}")
    subtitle = "  ".join(subtitle_parts)

    stage_display = snapshot.stage.replace("_", " ")
    stage_color = ANSI_CYAN if color else ""
    reset = ANSI_RESET if color else ""
    countdown_text = ""
    if snapshot.stage_countdown_s is not None and snapshot.stage_countdown_s >= 0.0:
        countdown_text = (
            f"  countdown={snapshot.stage_countdown_s:.1f}s"
        )
    stage_line = (
        f"{stage_color}STAGE{reset}  {snapshot.stage}"
        f"  ({stage_display})  elapsed={snapshot.stage_elapsed_s:.1f}s"
        f"{countdown_text}"
    )
    if not color:
        stage_line = (
            f"STAGE  {snapshot.stage}  ({stage_display})"
            f"  elapsed={snapshot.stage_elapsed_s:.1f}s{countdown_text}"
        )

    fault_color = ANSI_RED if color else ""
    fault_text = ""
    if snapshot.fault_reason:
        fault_text = (
            f"  {fault_color}FAULT{reset}  {snapshot.fault_reason}"
            if color
            else f"  FAULT  {snapshot.fault_reason}"
        )

    emg = _emg_card(snapshot, color, width)
    force = _force_card(snapshot, color, width)
    wrist = _wrist_card(snapshot, color, width)
    control = _control_card(snapshot, color, width)
    ring = render_haptic_ring(snapshot.haptics, color=color)
    haptic_title = (
        f"{ANSI_BOLD}HAPTIC BAND{ANSI_RESET}" if color else "HAPTIC BAND"
    )
    haptic_intensities = " ".join(
        f"m{i}={v:4.0f}" for i, v in enumerate(snapshot.haptics[:MOTOR_COUNT])
    )
    ring_block = [f"  {haptic_title}  phase={snapshot.haptic_phase}"]
    for line in ring:
        ring_block.append("  " + line)
    ring_block.append(f"  {haptic_intensities}")

    lines: list[str] = [bar, title, subtitle, bar, stage_line]
    if snapshot.control_hint:
        lines.append(f"hint: {snapshot.control_hint}")
    lines.append(sep)
    lines.extend(emg)
    lines.append(sep)
    lines.extend(force)
    lines.append(sep)
    lines.extend(wrist)
    lines.append(sep)
    lines.extend(control)
    lines.append(sep)
    lines.extend(ring_block)
    if snapshot.contact_reason:
        lines.append(f"contact: {snapshot.contact_reason}")
    if fault_text:
        lines.append(fault_text)
    lines.append(bar)
    return "\n".join(lines) + "\n"


__all__ = [
    "ANSI_BOLD",
    "ANSI_DIM",
    "ANSI_GREEN",
    "ANSI_YELLOW",
    "ANSI_RED",
    "ANSI_CYAN",
    "ANSI_MAGENTA",
    "ANSI_BLUE",
    "ANSI_RESET",
    "ANSI_CLEAR",
    "ANSI_HIDE_CURSOR",
    "ANSI_SHOW_CURSOR",
    "UiSnapshot",
    "render_haptic_ring",
    "render_terminal_frame",
]
