#!/usr/bin/env python3
"""Pure unit tests for the terminal UI render helpers."""

from __future__ import annotations

import os
import re
import sys
import unittest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.constants import MOTOR_COUNT
from scripts.mia_haptic_force_test.common.ui_render import (
    ANSI_BOLD,
    ANSI_CLEAR,
    ANSI_CYAN,
    ANSI_DIM,
    ANSI_GREEN,
    ANSI_HIDE_CURSOR,
    ANSI_RED,
    ANSI_RESET,
    ANSI_SHOW_CURSOR,
    ANSI_YELLOW,
    UiSnapshot,
    render_haptic_ring,
    render_terminal_frame,
)


class HapticRingTest(unittest.TestCase):
    def test_ring_has_four_lines(self) -> None:
        ring = render_haptic_ring([0.0] * MOTOR_COUNT, color=False)
        self.assertEqual(len(ring), 4)

    def test_ring_zero_intensity(self) -> None:
        ring = render_haptic_ring([0.0] * MOTOR_COUNT, color=False)
        for line in ring:
            self.assertIn("..", line)

    def test_ring_full_intensity(self) -> None:
        ring = render_haptic_ring([100.0] * MOTOR_COUNT, color=False)
        for line in ring:
            self.assertIn("##", line)

    def test_ring_short_input_pads(self) -> None:
        ring = render_haptic_ring([50.0, 50.0], color=False)
        # Pads to 8 motors; 6 are zero -> ".."
        flat = "".join(ring)
        self.assertEqual(flat.count("##"), 2)
        self.assertEqual(flat.count(".."), 6)

    def test_ring_uses_gradient_when_color(self) -> None:
        ring = render_haptic_ring([0.0, 50.0, 100.0] + [0.0] * 5, color=True)
        flat = "".join(ring)
        # Gradient uses 48;2;...m escape sequences.
        self.assertIn("\033[48;2;", flat)
        self.assertIn(ANSI_RESET, flat)

    def test_ring_no_ansi_when_plain(self) -> None:
        ring = render_haptic_ring([50.0] * MOTOR_COUNT, color=False)
        flat = "".join(ring)
        self.assertNotIn("\033[", flat)


class TerminalFrameTest(unittest.TestCase):
    def test_banner_present(self) -> None:
        frame = render_terminal_frame(UiSnapshot(stage="waiting_for_activation"), color=False)
        self.assertIn("Mia Hand EMG Haptic Force Test", frame)
        self.assertIn("waiting_for_activation", frame)

    def test_stage_display_replaces_underscores(self) -> None:
        frame = render_terminal_frame(UiSnapshot(stage="rotating_to_vertical"), color=False)
        self.assertIn("rotating to vertical", frame)
        self.assertIn("rotating_to_vertical", frame)

    def test_countdown_rendered_when_set(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(stage="vertical_delay", stage_countdown_s=1.5),
            color=False,
        )
        self.assertIn("countdown=1.5s", frame)

    def test_countdown_absent_when_none(self) -> None:
        frame = render_terminal_frame(UiSnapshot(stage="waiting_for_activation"), color=False)
        self.assertNotIn("countdown=", frame)

    def test_emg_card_renders(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(gesture="POWER", confidence=0.9, proportional=0.8), color=False
        )
        self.assertIn("EMG", frame)
        self.assertIn("POWER", frame)
        self.assertIn("conf=0.90", frame)
        self.assertIn("prop=0.80", frame)

    def test_emg_card_dim_when_low_confidence(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(gesture="POWER", confidence=0.1, confidence_threshold=0.55),
            color=True,
        )
        self.assertIn(ANSI_DIM, frame)

    def test_emg_card_green_when_high_confidence(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(gesture="POWER", confidence=0.9, confidence_threshold=0.55),
            color=True,
        )
        self.assertIn(ANSI_GREEN, frame)

    def test_force_card_per_finger(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(
                forces=(100.0, 200.0, 300.0),
                target_forces=(150.0, 250.0, 350.0),
                force_errors=(50.0, 50.0, 50.0),
            ),
            color=False,
        )
        self.assertIn("FORCE", frame)
        self.assertIn("thumb", frame)
        self.assertIn("index", frame)
        self.assertIn("mrl", frame)
        self.assertIn("100.0", frame)
        self.assertIn("200.0", frame)
        self.assertIn("300.0", frame)

    def test_wrist_card_renders(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(
                wrist_position_deg=120.0,
                wrist_target_deg=90.0,
                wrist_velocity_deg_s=15.0,
            ),
            color=False,
        )
        self.assertIn("WRIST", frame)
        self.assertIn("120.0", frame)
        self.assertIn("90.0", frame)
        self.assertIn("+15.0", frame)

    def test_control_card_renders(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(mode="velocity", hold_mode="force", enable=True),
            color=False,
        )
        self.assertIn("CONTROL", frame)
        self.assertIn("mode=velocity", frame)
        self.assertIn("hold=force", frame)
        self.assertIn("enabled", frame)

    def test_control_card_disabled(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(mode="position", hold_mode="force", enable=False),
            color=False,
        )
        self.assertIn("disabled", frame)

    def test_adjust_active(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(
                stage="force_hold",
                hold_mode="force",
                adjust_active=True,
                adjust_arrow="\u25b2",
                adjust_label="adjusting force",
                adjust_delta=50.0,
            ),
            color=False,
        )
        self.assertIn("adjusting force", frame)
        self.assertIn("+50.0", frame)

    def test_adjust_idle_in_force_hold(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(stage="force_hold", hold_mode="force"),
            color=False,
        )
        self.assertIn("adjust idle", frame)

    def test_fault_reason_renders(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(fault_reason="emergency force exceeded"), color=False
        )
        self.assertIn("FAULT", frame)
        self.assertIn("emergency force exceeded", frame)

    def test_contact_reason_renders(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(contact_reason="thumb contact"), color=False
        )
        self.assertIn("contact: thumb contact", frame)

    def test_haptic_block_renders(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(
                haptics=(0.0, 25.0, 50.0, 75.0, 100.0, 0.0, 0.0, 0.0),
                haptic_phase="force:all_motors",
            ),
            color=False,
        )
        self.assertIn("HAPTIC BAND", frame)
        self.assertIn("phase=force:all_motors", frame)
        for i in range(MOTOR_COUNT):
            self.assertIn(f"m{i}=", frame)

    def test_control_hint_renders(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(stage="waiting_for_activation", control_hint="press POWER to start"),
            color=False,
        )
        self.assertIn("hint: press POWER to start", frame)

    def test_color_escapes_present(self) -> None:
        frame = render_terminal_frame(UiSnapshot(stage="force_hold", enable=True), color=True)
        self.assertIn(ANSI_BOLD, frame)
        self.assertIn(ANSI_CYAN, frame)
        self.assertIn(ANSI_RESET, frame)

    def test_no_color_escapes_when_plain(self) -> None:
        frame = render_terminal_frame(UiSnapshot(stage="force_hold"), color=False)
        self.assertNotIn("\033[", frame)

    def test_config_and_run_id(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(config_name="test.yaml", run_id="run_001"), color=False
        )
        self.assertIn("config: test.yaml", frame)
        self.assertIn("run: run_001", frame)


class UiSnapshotTest(unittest.TestCase):
    def test_defaults(self) -> None:
        s = UiSnapshot()
        self.assertEqual(s.stage, "initialising")
        self.assertEqual(s.mode, "position")
        self.assertEqual(s.hold_mode, "force")
        self.assertEqual(s.gesture, "REST")
        self.assertEqual(s.forces, (0.0, 0.0, 0.0))
        self.assertEqual(len(s.haptics), MOTOR_COUNT)

    def test_frozen(self) -> None:
        s = UiSnapshot()
        with self.assertRaises(Exception):
            s.stage = "complete"  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()
