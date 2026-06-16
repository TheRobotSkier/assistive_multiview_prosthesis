#!/usr/bin/env python3
"""Pipeline launch configuration TUI (terminal user interface).

Presents the pipeline.launch.py launch arguments as checkboxes, dropdowns,
and text fields with sensible defaults.  On launch, builds the ros2 launch
command and runs it.

Run inside the container:
    python3 /prosthesis_ws/scripts/launch_tui.py
or via the Makefile:
    make run-tui

Uses only the Python standard library (curses) — no X11, no extra deps.
Works over plain SSH / docker attach / make shell.
"""

from __future__ import annotations

import curses
import os
import sys
from typing import Optional


# ── Launch argument definitions ────────────────────────────────────────────
# Each entry: (key, label, default, kind, choices_or_None)
#   kind = "bool"    -> toggle [x] / [ ]
#   kind = "choice"  -> cycle through choices (Enter / right-arrow)
#   kind = "str"     -> editable text field
#   kind = "float"   -> editable text field (validated as float)
#
# Defaults match pipeline.launch.py DeclareLaunchArgument defaults.
LAUNCH_ARGS: list[tuple[str, str, str, str, Optional[list[str]]]] = [
    # ── Hardware toggles ──
    ("mia_hand",  "MIA Hand driver",        "true",  "bool", None),
    ("wrist",     "Wrist Dynamixel",        "true",  "bool", None),
    ("emg",       "EMG classifier bridge",  "true",  "bool", None),
    ("haptic",    "Haptic bridge",          "true",  "bool", None),
    # ── Perception toggles ──
    ("camera",            "Camera bridge nodes",      "true",  "bool", None),
    ("camera_tf_bridge",  "OpenVINS TF bridge",       "true",  "bool", None),
    ("rviz",              "Launch RViz",              "true",  "bool", None),
    ("tf_diagnostics",    "TF diagnostics node",      "true",  "bool", None),
    ("debug_monitor",     "Pipeline diagnostics",     "false", "bool", None),
    ("require_dual_openvins", "Require dual OpenVINS", "false", "bool", None),
    # ── Fusion mode ──
    ("fusion_mode", "Fusion mode",
     "tsdf_preview", "choice",
     ["legacy", "tsdf_preview", "tsdf_grasp", "gtsam_only"]),
    ("use_tsdf_fusion", "use_tsdf_fusion (legacy flag)",
     "false", "bool", None),
    # ── Tuning knobs ──
    ("roi_radius", "ROI crop radius (m)", "0.1", "float", None),
    ("target_frame", "Target frame", "marker_map", "str", None),
    ("inference_url", "Inference server URL",
     "http://127.0.0.1:5678", "str", None),
    ("camera_mount", "Camera mount name",
     "8_cm_cam_mount", "str", None),
]

# Group layout for display: (section title, keys in display order)
SECTIONS: list[tuple[str, list[str]]] = [
    ("Hardware", ["mia_hand", "wrist", "emg", "haptic"]),
    ("Perception", ["camera", "camera_tf_bridge", "rviz",
                    "tf_diagnostics", "debug_monitor",
                    "require_dual_openvins"]),
    ("Fusion", ["fusion_mode", "use_tsdf_fusion"]),
    ("Tuning", ["roi_radius", "target_frame", "inference_url",
                "camera_mount"]),
]

TITLE = "Prosthesis Pipeline Launcher"
MOUNTS_CONFIG = ("/prosthesis_ws/src/sensor_fusion_bringup/config/"
                 "camera_mounts.yaml")


# ── Helpers ────────────────────────────────────────────────────────────────

def _arg_dict() -> dict[str, tuple]:
    """Return a key -> arg-tuple lookup."""
    return {a[0]: a for a in LAUNCH_ARGS}


def build_command(values: dict[str, str]) -> str:
    """Build the ros2 launch command from the current values."""
    ad = _arg_dict()
    parts = []
    for key, val in values.items():
        default = ad[key][2]
        if val == default:
            continue
        parts.append(f"{key}:={val}")
    base = (
        "ros2 launch prosthesis_launch pipeline.launch.py "
        f"haptic:=false mounts_config:={MOUNTS_CONFIG} "
        "tf_diagnostics:=true"
    )
    return base + (" " + " ".join(parts) if parts else "")


# ── TUI ────────────────────────────────────────────────────────────────────

class LaunchTUI:
    """Curses-based TUI for configuring and launching the pipeline."""

    def __init__(self, stdscr: curses.window) -> None:
        self.stdscr = stdscr
        self.ad = _arg_dict()
        # Flat list of (key, label, kind, choices) in display order, with
        # section headers interspersed as ("__section__", title, ...).
        self.rows: list[tuple] = self._build_rows()
        # Current value for each arg key.
        self.values: dict[str, str] = {a[0]: a[2] for a in LAUNCH_ARGS}
        self.cursor = 0          # index into self.rows (arg rows only)
        self.editing = False     # text-entry mode for str/float fields
        self.edit_buf = ""
        self.should_launch = False
        self.message = ""

    def _build_rows(self) -> list[tuple]:
        """Build the display row list: section headers + arg rows.

        Each arg row: (key, label, kind, choices)
        Each section header: ("__section__", title, None, None)
        """
        rows = []
        for title, keys in SECTIONS:
            rows.append(("__section__", title, None, None))
            for key in keys:
                a = self.ad[key]
                rows.append((key, a[1], a[3], a[4]))
        return rows

    # ── Navigation helpers ────────────────────────────────────────────────

    def _arg_indices(self) -> list[int]:
        """Indices of rows that are args (not section headers)."""
        return [i for i, r in enumerate(self.rows)
                if r[0] != "__section__"]

    def _current_row(self) -> Optional[tuple]:
        idx = self._arg_indices()
        if not idx or self.cursor >= len(idx):
            return None
        return self.rows[idx[self.cursor]]

    # ── Event loop ────────────────────────────────────────────────────────

    def run(self) -> None:
        curses.curs_set(0)  # hide cursor
        self.stdscr.keypad(True)
        while True:
            self._draw()
            ch = self.stdscr.getch()

            if self.editing:
                self._handle_edit_key(ch)
            else:
                if self._handle_nav_key(ch):
                    break

            if self.should_launch:
                break

    def _handle_nav_key(self, ch: int) -> bool:
        """Handle a key in navigation mode. Returns True to quit."""
        if ch in (curses.KEY_UP, ord("k")):
            self.cursor = max(0, self.cursor - 1)
            self.message = ""
        elif ch in (curses.KEY_DOWN, ord("j")):
            total = len(self._arg_indices())
            self.cursor = min(total - 1, self.cursor + 1)
            self.message = ""
        elif ch in (curses.KEY_LEFT, ord("h")):
            self._cycle_choice(-1)
        elif ch in (curses.KEY_RIGHT, ord("l")):
            self._cycle_choice(1)
        elif ch in (ord(" "), 10, 13, curses.KEY_ENTER):  # Space/Enter
            self._toggle_or_edit()
        elif ch == ord("r"):
            self._reset_defaults()
            self.message = "Reset to defaults."
        elif ch in (ord("q"), 27):  # q or Esc
            return True
        elif ch == ord("L"):  # Shift+L = Launch
            self.should_launch = True
        return False

    def _handle_edit_key(self, ch: int) -> None:
        """Handle a key in text-editing mode."""
        if ch in (10, 13, curses.KEY_ENTER):  # Enter — confirm
            self._commit_edit()
        elif ch == 27:  # Esc — cancel
            self.editing = False
            self.edit_buf = ""
            self.message = "Edit cancelled."
        elif ch in (curses.KEY_BACKSPACE, 127, 8):
            self.edit_buf = self.edit_buf[:-1]
        elif 32 <= ch <= 126:
            self.edit_buf += chr(ch)

    # ── Field actions ────────────────────────────────────────────────────

    def _toggle_or_edit(self) -> None:
        row = self._current_row()
        if row is None:
            return
        key, label, kind, choices = row
        if kind == "bool":
            cur = self.values[key]
            self.values[key] = "false" if cur == "true" else "true"
            self.message = ""
        elif kind == "choice":
            self._cycle_choice(1)
        else:  # str / float — enter edit mode
            self.editing = True
            self.edit_buf = self.values[key]
            self.message = "Editing — Enter to confirm, Esc to cancel."

    def _cycle_choice(self, direction: int) -> None:
        row = self._current_row()
        if row is None:
            return
        key, label, kind, choices = row
        if kind == "choice" and choices:
            cur = self.values[key]
            try:
                idx = choices.index(cur)
            except ValueError:
                idx = 0
            idx = (idx + direction) % len(choices)
            self.values[key] = choices[idx]
            self.message = ""
        elif kind == "bool":
            cur = self.values[key]
            self.values[key] = "false" if cur == "true" else "true"
            self.message = ""

    def _commit_edit(self) -> None:
        row = self._current_row()
        if row is None:
            self.editing = False
            return
        key, label, kind, choices = row
        val = self.edit_buf.strip()
        if kind == "float":
            try:
                float(val)
            except ValueError:
                self.message = f"Invalid float: {val!r} — keeping old value."
                self.editing = False
                self.edit_buf = ""
                return
        self.values[key] = val
        self.editing = False
        self.edit_buf = ""
        self.message = ""

    def _reset_defaults(self) -> None:
        for a in LAUNCH_ARGS:
            self.values[a[0]] = a[2]

    # ── Rendering ────────────────────────────────────────────────────────

    def _draw(self) -> None:
        s = self.stdscr
        s.erase()
        h, w = s.getmaxyx()

        # Title
        s.addstr(0, 0, f" {TITLE} ", curses.A_REVERSE | curses.A_BOLD)
        s.addstr(1, 0, " Arrow keys: navigate   Space/Enter: toggle/edit"
                       "   r: reset   L: Launch   q: quit")

        # Arg rows
        arg_indices = self._arg_indices()
        row_y = 3
        for i, row in enumerate(self.rows):
            if row_y >= h - 6:
                break
            if row[0] == "__section__":
                # Section header
                s.addstr(row_y, 1, f"─ {row[1]} ─", curses.A_BOLD | curses.A_UNDERLINE)
                row_y += 1
                continue

            key, label, kind, choices = row
            is_current = (i in arg_indices and
                          arg_indices.index(i) == self.cursor)
            prefix = " > " if is_current else "   "
            attr = curses.A_REVERSE if is_current else curses.A_NORMAL

            # Value display
            if kind == "bool":
                on = self.values[key] == "true"
                val_str = "[x] ON " if on else "[ ] OFF"
            elif kind == "choice":
                val_str = f"< {self.values[key]} >"
            else:
                if self.editing and is_current:
                    val_str = f"[{self.edit_buf}]"
                else:
                    val_str = f"  {self.values[key]}  "

            # Right-align values
            label_col = 3
            val_col = max(label_col + 2, w - len(val_str) - 4)
            if val_col <= label_col + len(label) + 2:
                val_col = label_col + len(label) + 3

            line = f"{prefix}{label}"
            s.addstr(row_y, 0, line[:w], attr)
            pad = val_col - len(line)
            if pad > 0 and is_current:
                s.addstr(row_y, len(line), " " * pad, attr)
            s.addstr(row_y, val_col, val_str[:w - val_col], attr)
            row_y += 1

        # Command preview
        cmd = build_command(self.values)
        preview_y = max(row_y + 1, h - 5)
        s.addstr(preview_y, 0, "─ Command " + "─" * max(0, w - 10),
                 curses.A_DIM)
        # Word-wrap the command across remaining lines
        remaining = h - preview_y - 2
        if remaining > 0:
            wrapped = _wrap_text(cmd, w - 2)
            for li, line in enumerate(wrapped[:remaining]):
                s.addstr(preview_y + 1 + li, 1, line[:w - 2], curses.A_DIM)

        # Message / status bar
        if self.message:
            s.addstr(h - 1, 0, f" {self.message} "[:w - 1],
                     curses.A_REVERSE)
        else:
            hint = (" [L] Launch   [r] Reset   [q] Quit"
                    if not self.editing else
                    " [Enter] Confirm   [Esc] Cancel")
            s.addstr(h - 1, 0, hint[:w - 1], curses.A_DIM)

        s.refresh()


def _wrap_text(text: str, width: int) -> list[str]:
    """Simple word-boundary wrapper for the command preview."""
    words = text.split(" ")
    lines: list[str] = []
    cur = ""
    for word in words:
        if cur and len(cur) + 1 + len(word) > width:
            lines.append(cur)
            cur = word
        else:
            cur = f"{cur} {word}".strip()
    if cur:
        lines.append(cur)
    return lines


def _launch_pipeline(cmd: str) -> None:
    """Source ROS setup and exec the launch command."""
    full = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /prosthesis_ws/install/setup.bash 2>/dev/null; "
        + cmd
    )
    # End curses, then replace this process with bash.
    print(f"\nLaunching:\n  {cmd}\n")
    os.execvp("bash", ["bash", "-lc", full])


def main(stdscr: curses.window) -> int:
    tui = LaunchTUI(stdscr)
    tui.run()
    if tui.should_launch:
        cmd = build_command(tui.values)
        curses.endwin()
        _launch_pipeline(cmd)
    return 0


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except KeyboardInterrupt:
        pass
    sys.exit(0)
