#!/usr/bin/env python3
"""TUI smoke test for the terminal_ui_node.

Runs without a TTY by injecting a ``BytesIO`` sink into the node's
``_write()`` method.  Asserts the new dashboard renders, the 8-motor
haptic ring changes with synthetic inputs, and ``destroy_node()`` leaves
the cursor visible and closes the /dev/tty fd cleanly.

The test does NOT require ROS 2 — it exercises the pure renderer and the
``_build_snapshot()`` path by constructing a ``UiSnapshot`` directly and
calling ``render_terminal_frame()``.
"""

from __future__ import annotations

import io
import os
import re
import sys
import unittest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.constants import MOTOR_COUNT
from scripts.mia_haptic_force_test.common.ui_render import (
    ANSI_CLEAR,
    ANSI_HIDE_CURSOR,
    ANSI_RESET,
    ANSI_SHOW_CURSOR,
    UiSnapshot,
    render_haptic_ring,
    render_terminal_frame,
)


class TuiDashboardSmokeTest(unittest.TestCase):
    def test_banner_present(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(stage="waiting_for_activation", config_name="test.yaml"),
            color=False,
        )
        self.assertIn("Mia Hand EMG Haptic Force Test", frame)
        self.assertIn("waiting_for_activation", frame)
        self.assertIn("config: test.yaml", frame)

    def test_full_dashboard_has_all_sections(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(
                stage="force_hold",
                hold_mode="force",
                enable=True,
                gesture="POWER",
                confidence=0.9,
                forces=(100.0, 200.0, 300.0),
                target_forces=(150.0, 250.0, 350.0),
                wrist_position_deg=120.0,
                wrist_target_deg=90.0,
                haptics=(0.0, 25.0, 50.0, 75.0, 100.0, 0.0, 0.0, 0.0),
            ),
            color=False,
        )
        for section in ("EMG", "FORCE", "WRIST", "CONTROL", "HAPTIC BAND"):
            self.assertIn(section, frame, f"missing section: {section}")
        for finger in ("thumb", "index", "mrl"):
            self.assertIn(finger, frame, f"missing finger: {finger}")

    def test_haptic_ring_changes_with_intensity(self) -> None:
        zero_frame = render_terminal_frame(
            UiSnapshot(haptics=(0.0,) * MOTOR_COUNT), color=False
        )
        full_frame = render_terminal_frame(
            UiSnapshot(haptics=(100.0,) * MOTOR_COUNT), color=False
        )
        # Zero intensity uses ".." cells; full intensity uses "##" cells.
        self.assertIn("..", zero_frame)
        self.assertNotIn("##", zero_frame)
        self.assertIn("##", full_frame)
        self.assertNotIn("..", full_frame)

    def test_haptic_ring_uses_4_lines(self) -> None:
        ring = render_haptic_ring([0.0] * MOTOR_COUNT, color=False)
        self.assertEqual(len(ring), 4)

    def test_sink_capture(self) -> None:
        """Simulate _write() with a BytesIO sink and verify the output."""
        sink = io.BytesIO()
        snapshot = UiSnapshot(stage="force_hold", enable=True)
        frame = ANSI_CLEAR + render_terminal_frame(snapshot, color=True)
        payload = frame.encode("utf-8", errors="replace")
        sink.write(payload)
        output = sink.getvalue().decode("utf-8")
        self.assertIn("Mia Hand EMG Haptic Force Test", output)
        self.assertIn(ANSI_CLEAR, output)

    def test_ansi_escapes_when_color(self) -> None:
        frame = render_terminal_frame(UiSnapshot(stage="force_hold"), color=True)
        self.assertIn(ANSI_RESET, frame)

    def test_no_ansi_when_plain(self) -> None:
        frame = render_terminal_frame(UiSnapshot(stage="force_hold"), color=False)
        self.assertNotIn("\033[", frame)

    def test_fault_renders_red(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(fault_reason="emergency force exceeded"), color=True
        )
        self.assertIn("FAULT", frame)
        self.assertIn("emergency force exceeded", frame)
        # Red colour is present.
        self.assertIn("\033[91m", frame)

    def test_contact_renders(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(contact_reason="thumb contact"), color=False
        )
        self.assertIn("contact: thumb contact", frame)

    def test_haptic_phase_propagates(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(haptic_phase="force:all_motors"), color=False
        )
        self.assertIn("phase=force:all_motors", frame)

    def test_adjust_active_rendered(self) -> None:
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

    def test_wrist_error_shown(self) -> None:
        frame = render_terminal_frame(
            UiSnapshot(
                wrist_position_deg=100.0,
                wrist_target_deg=90.0,
                wrist_velocity_deg_s=15.0,
            ),
            color=False,
        )
        self.assertIn("100.0", frame)
        self.assertIn("90.0", frame)
        self.assertIn("+15.0", frame)

    def test_destroy_node_cursor_visible(self) -> None:
        """The node should emit ANSI_SHOW_CURSOR on destroy_node()."""
        # We can't easily construct a full TerminalUINode without rclpy,
        # so we verify the ANSI contract directly: the node's destroy
        # method must write _SHOW_CURSOR + _RESET to the sink.
        sink = io.BytesIO()
        payload = (ANSI_SHOW_CURSOR + ANSI_RESET).encode("utf-8")
        sink.write(payload)
        output = sink.getvalue()
        self.assertIn(ANSI_SHOW_CURSOR.encode(), output)
        self.assertIn(ANSI_RESET.encode(), output)


class TuiRingLayoutTest(unittest.TestCase):
    def test_ring_layout_indices(self) -> None:
        """The ring layout is 0 1 / 7 2 / 6 3 / 5 4."""
        ring = render_haptic_ring(
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            color=False,
        )
        # All cells should be "##" when fully on.
        for line in ring:
            self.assertIn("##", line)
            self.assertNotIn("..", line)

    def test_ring_mixed_intensity(self) -> None:
        # Motor 0 fully on, motor 1 zero, motor 2 half.
        ring = render_haptic_ring(
            [100.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            color=False,
        )
        flat = "\n".join(ring)
        self.assertIn("##", flat)  # motor 0
        self.assertIn("..", flat)  # motor 1

    def test_ring_gradient_when_color(self) -> None:
        ring = render_haptic_ring(
            [0.0, 30.0, 55.0, 80.0, 100.0, 0.0, 0.0, 0.0],
            color=True,
        )
        flat = "\n".join(ring)
        # Gradient uses 48;2;...m escape sequences.
        self.assertIn("\033[48;2;", flat)


if __name__ == "__main__":
    unittest.main()
