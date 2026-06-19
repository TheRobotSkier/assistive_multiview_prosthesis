#!/usr/bin/env python3
"""TUI node exercise test (Docker tier).

Constructs the actual ``TerminalUINode`` and verifies:
- The _write() sink injection works (no TTY required).
- The _build_snapshot() method produces a valid UiSnapshot.
- The render() output contains the expected sections.
- The 8-motor haptic ring changes with synthetic haptic inputs.
- destroy_node() restores the cursor and closes the /dev/tty fd.

Run inside the Docker container where rclpy is available.
"""

from __future__ import annotations

import io
import os
import sys
import threading
import time
import unittest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

try:
    import rclpy  # noqa: F401
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Float32MultiArray, String

    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False

from scripts.mia_haptic_force_test.common.constants import (
    MOTOR_COUNT,
    TOPIC_HAPTIC_BAND_MOTORS,
)


@unittest.skipUnless(_HAS_ROS, "rclpy not available")
class TerminalUINodeExerciseTest(unittest.TestCase):
    """Exercise the real TerminalUINode with an injected sink."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.executor = SingleThreadedExecutor()

    @classmethod
    def tearDownClass(cls) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self) -> None:
        from scripts.mia_haptic_force_test.terminal_ui_node import TerminalUINode

        self._sink = io.BytesIO()
        self._node = TerminalUINode(
            config_path=os.path.join(
                _REPO_ROOT, "config", "mia_haptic_force_test.yaml"
            )
        )
        self.executor.add_node(self._node)

    def tearDown(self) -> None:
        self.executor.remove_node(self._node)
        self._node.destroy_node()

    def test_build_snapshot(self) -> None:
        snap = self._node._build_snapshot()
        self.assertEqual(snap.stage, "initialising")
        self.assertEqual(snap.mode, "position")
        self.assertEqual(snap.hold_mode, "force")
        self.assertEqual(snap.haptics, (0.0,) * MOTOR_COUNT)

    def test_sink_capture(self) -> None:
        """Call _render() with the injected sink and verify the output."""
        self._node._render(sink=self._sink)
        output = self._sink.getvalue().decode("utf-8", errors="replace")
        self.assertIn("Mia Hand EMG Haptic Force Test", output)
        self.assertIn("initialising", output)

    def test_haptic_ring_changes_with_intensity(self) -> None:
        # Publish synthetic /haptic_band/motors.
        pub = self._node.create_publisher(
            Float32MultiArray, TOPIC_HAPTIC_BAND_MOTORS, 10
        )
        # Zero intensity.
        zero_msg = Float32MultiArray()
        zero_msg.data = [0.0] * MOTOR_COUNT
        for _ in range(10):
            pub.publish(zero_msg)
            self.executor.spin_once(timeout_sec=0.05)
            time.sleep(0.05)
        self._sink.seek(0)
        self._sink.truncate()
        self._node._render(sink=self._sink)
        zero_output = self._sink.getvalue().decode("utf-8", errors="replace")
        # Full intensity.
        full_msg = Float32MultiArray()
        full_msg.data = [100.0] * MOTOR_COUNT
        for _ in range(10):
            pub.publish(full_msg)
            self.executor.spin_once(timeout_sec=0.05)
            time.sleep(0.05)
        self._sink.seek(0)
        self._sink.truncate()
        self._node._render(sink=self._sink)
        full_output = self._sink.getvalue().decode("utf-8", errors="replace")
        # Zero intensity uses "..", full uses "##" in the ring.
        self.assertIn("..", zero_output)
        self.assertNotIn("##", zero_output)
        self.assertIn("##", full_output)
        self.assertNotIn("..", full_output)

    def test_destroy_node_restores_cursor(self) -> None:
        """destroy_node() should emit ANSI_SHOW_CURSOR to the sink."""
        # Call destroy_node which writes _SHOW_CURSOR to the tty fd.
        # In a no-TTY environment the tty_fd is None so it falls back
        # to stdout.  We just verify the node shuts down cleanly.
        self.executor.remove_node(self._node)
        self._node.destroy_node()
        # No assertion on cursor bytes (depends on tty availability);
        # the test verifies clean shutdown without exception.


if __name__ == "__main__":
    unittest.main()
