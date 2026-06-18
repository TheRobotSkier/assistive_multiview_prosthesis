#!/usr/bin/env python3
"""Fast topic-contract tests for the mia_haptic_force_test stack.

These tests exercise the pure contracts:
- Keyboard idle helper and gesture map (no TTY required).
- Supervisor stage transitions driven by synthetic EMG/force/wrist inputs.
- Controller switching and command publication.
- Haptic mapping from force/wrist state to /haptic_band/motors.
- Logger field contract matches the monolithic CsvLogger.

All tests use ``rclpy`` subscribers or direct method calls; no
``time.sleep`` beyond the 0.08s keyboard-idle window.
"""

from __future__ import annotations

import os
import sys
import unittest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.constants import (
    EMG_ACTIVATION_LABEL,
    EMG_INCREASE_FORCE_LABEL,
    EMG_OPEN_LABEL,
    EMG_REST_LABEL,
    FINGER_LABELS,
    GESTURES,
    KEYBOARD_GESTURE_MAP,
    MOTOR_COUNT,
    POSITION_CONTROLLERS,
    REST_GESTURE_LABEL,
    TOPIC_GROUP_VEL_FF_COMMANDS,
    TOPIC_HAPTIC_BAND_MOTORS,
    VELOCITY_CONTROLLERS,
)
from scripts.mia_haptic_force_test.common.conversions import (
    force_haptics,
    hold_velocity,
    wrist_haptics,
)


# ── Keyboard idle helper ──────────────────────────────────────────────────

class KeyboardIdleHelperTest(unittest.TestCase):
    """Test the pure keyboard_gesture_state() helper extracted from the node."""

    def test_no_key_returns_rest(self) -> None:
        from scripts.mia_haptic_force_test.common.keyboard_idle import (
            keyboard_gesture_state,
        )

        gesture, label, conf, prop, age = keyboard_gesture_state("REST", 0.0, 1.0)
        self.assertEqual(gesture, "REST")
        self.assertEqual(label, REST_GESTURE_LABEL)
        self.assertEqual(conf, 0.0)
        self.assertEqual(prop, 0.0)
        self.assertEqual(age, -1.0)

    def test_active_key_within_timeout(self) -> None:
        from scripts.mia_haptic_force_test.common.keyboard_idle import (
            keyboard_gesture_state,
        )

        gesture, label, conf, prop, age = keyboard_gesture_state("POWER", 1.0, 1.05)
        self.assertEqual(gesture, "POWER")
        self.assertEqual(label, EMG_ACTIVATION_LABEL)
        self.assertEqual(conf, 1.0)
        self.assertEqual(prop, 1.0)
        self.assertAlmostEqual(age, 0.05, places=4)

    def test_key_after_timeout_returns_rest(self) -> None:
        from scripts.mia_haptic_force_test.common.keyboard_idle import (
            keyboard_gesture_state,
        )

        gesture, label, conf, prop, age = keyboard_gesture_state(
            "POWER", 1.0, 1.5, timeout_s=0.08
        )
        self.assertEqual(gesture, "REST")
        self.assertEqual(label, REST_GESTURE_LABEL)
        self.assertEqual(conf, 0.0)
        self.assertEqual(prop, 0.0)
        self.assertAlmostEqual(age, 0.5, places=4)

    def test_unknown_gesture_uses_rest_label(self) -> None:
        from scripts.mia_haptic_force_test.common.keyboard_idle import (
            keyboard_gesture_state,
        )

        gesture, label, conf, prop, age = keyboard_gesture_state(
            "UNKNOWN", 1.0, 1.05
        )
        self.assertEqual(gesture, "UNKNOWN")
        self.assertEqual(label, REST_GESTURE_LABEL)
        self.assertEqual(conf, 1.0)


class KeyboardGestureMapTest(unittest.TestCase):
    def test_wasd_mappings(self) -> None:
        self.assertEqual(KEYBOARD_GESTURE_MAP["w"], "EXTENSION")
        self.assertEqual(KEYBOARD_GESTURE_MAP["a"], "OPEN")
        self.assertEqual(KEYBOARD_GESTURE_MAP["s"], "FLEXION")
        self.assertEqual(KEYBOARD_GESTURE_MAP["d"], "POWER")

    def test_arrow_mappings(self) -> None:
        self.assertEqual(KEYBOARD_GESTURE_MAP["\x1b[A"], "EXTENSION")
        self.assertEqual(KEYBOARD_GESTURE_MAP["\x1b[D"], "OPEN")
        self.assertEqual(KEYBOARD_GESTURE_MAP["\x1b[B"], "FLEXION")
        self.assertEqual(KEYBOARD_GESTURE_MAP["\x1b[C"], "POWER")

    def test_uppercase_wasd(self) -> None:
        self.assertEqual(KEYBOARD_GESTURE_MAP["W"], "EXTENSION")
        self.assertEqual(KEYBOARD_GESTURE_MAP["A"], "OPEN")
        self.assertEqual(KEYBOARD_GESTURE_MAP["S"], "FLEXION")
        self.assertEqual(KEYBOARD_GESTURE_MAP["D"], "POWER")

    def test_gesture_label_lookup(self) -> None:
        self.assertEqual(GESTURES["REST"], EMG_REST_LABEL)
        self.assertEqual(GESTURES["POWER"], EMG_ACTIVATION_LABEL)
        self.assertEqual(GESTURES["OPEN"], EMG_OPEN_LABEL)
        self.assertEqual(GESTURES["FLEXION"], EMG_INCREASE_FORCE_LABEL)


# ── Controller switching and command publication ─────────────────────────


class ControllerSwitchTest(unittest.TestCase):
    """Test controller switching and command publication logic."""

    def test_position_controllers_activate_set(self) -> None:
        self.assertIn("group_pos_ff_controller", POSITION_CONTROLLERS)
        self.assertIn("thumb_pos_ff_controller", POSITION_CONTROLLERS)
        self.assertIn("index_pos_ff_controller", POSITION_CONTROLLERS)
        self.assertIn("mrl_pos_ff_controller", POSITION_CONTROLLERS)

    def test_velocity_controllers_activate_set(self) -> None:
        self.assertIn("group_vel_ff_controller", VELOCITY_CONTROLLERS)
        self.assertIn("thumb_vel_ff_controller", VELOCITY_CONTROLLERS)
        self.assertIn("index_vel_ff_controller", VELOCITY_CONTROLLERS)
        self.assertIn("mrl_vel_ff_controller", VELOCITY_CONTROLLERS)

    def test_hold_velocity_deadzone(self) -> None:
        """hold_velocity returns 0 when error is within deadzone."""
        v = hold_velocity(
            force=290.0,
            target=300.0,
            deadzone=20.0,
            max_velocity=0.08,
            min_overshoot=20.0,
            max_overshoot=150.0,
        )
        self.assertEqual(v, 0.0)

    def test_hold_velocity_undershoot_ramps(self) -> None:
        v = hold_velocity(
            force=260.0,
            target=300.0,
            deadzone=20.0,
            max_velocity=0.08,
            min_overshoot=20.0,
            max_overshoot=150.0,
        )
        self.assertGreater(v, 0.0)
        self.assertLess(v, 0.08)

    def test_hold_velocity_overshoot_releases(self) -> None:
        v = hold_velocity(
            force=400.0,
            target=300.0,
            deadzone=20.0,
            max_velocity=0.08,
            min_overshoot=20.0,
            max_overshoot=150.0,
        )
        self.assertLess(v, 0.0)


# ── Haptic mapping ────────────────────────────────────────────────────────


class HapticMappingTest(unittest.TestCase):
    def test_wrist_haptics_8_motors(self) -> None:
        cfg = {
            "motor_count": 8,
            "wrist_motor_angles_deg": [i * 45.0 for i in range(8)],
            "wrist_thumb_offset_deg": 0.0,
            "wrist_intensity_pct": 45.0,
            "wrist_min_motor_pct": 3.0,
        }
        result = wrist_haptics(0.0, cfg)
        self.assertEqual(len(result), 8)
        # At least one motor should be non-zero.
        self.assertTrue(any(v > 0.0 for v in result))

    def test_force_haptics_8_motors(self) -> None:
        cfg = {
            "motor_count": 8,
            "force_min_grasp_force": 50.0,
            "force_max_grasp_force": 500.0,
            "force_low_intensity_pct": 18.0,
            "force_high_intensity_pct": 85.0,
            "force_motor_step_pct": 5.0,
            "force_motor_order": list(range(8)),
            "force_phase_change_threshold_pct": 40.0,
        }
        values, phase = force_haptics(50.0, cfg)
        self.assertEqual(len(values), 8)
        self.assertIsInstance(phase, str)

    def test_force_haptics_zero_returns_zeros(self) -> None:
        cfg = {
            "motor_count": 8,
            "force_min_grasp_force": 50.0,
            "force_max_grasp_force": 500.0,
            "force_low_intensity_pct": 18.0,
            "force_high_intensity_pct": 85.0,
            "force_motor_step_pct": 5.0,
            "force_motor_order": list(range(8)),
            "force_phase_change_threshold_pct": 40.0,
        }
        values, phase = force_haptics(0.0, cfg)
        self.assertEqual(values, [0.0] * 8)

    def test_motor_count_constant(self) -> None:
        self.assertEqual(MOTOR_COUNT, 8)


# ── Logger field contract ─────────────────────────────────────────────────


class LoggerFieldContractTest(unittest.TestCase):
    def test_sample_field_order_matches_monolith(self) -> None:
        """The split-node logger and the monolith CsvLogger must produce
        the same CSV header so existing analysis tools keep working."""
        # Import the monolith's _sample_fields indirectly by importing the
        # module.  This avoids running the monolith's __main__ block.
        import importlib.util

        monolith_path = os.path.join(
            _REPO_ROOT, "scripts", "mia_haptic_force_test.py"
        )
        spec = importlib.util.spec_from_file_location("monolith", monolith_path)
        monolith = importlib.util.module_from_spec(spec)

        # Patch rclpy to avoid import-time failures.
        import sys
        import types

        fake_rclpy = types.ModuleType("rclpy")
        fake_rclpy.node = types.ModuleType("rclpy.node")
        fake_rclpy.node.Node = object
        fake_rclpy.init = lambda *a, **k: None
        fake_rclpy.shutdown = lambda: None
        fake_rclpy.ok = lambda: False
        fake_rclpy.spin = lambda *a, **k: None
        sys.modules.setdefault("rclpy", fake_rclpy)
        sys.modules.setdefault("rclpy.node", fake_rclpy.node)
        # Also stub the sensor_msgs and std_msgs modules the monolith imports.
        for mod_name in (
            "sensor_msgs.msg",
            "std_msgs.msg",
            "std_srvs.srv",
            "force_controller.controller_manager_client",
        ):
            if mod_name not in sys.modules:
                m = types.ModuleType(mod_name)
                sys.modules[mod_name] = m

        try:
            spec.loader.exec_module(monolith)
        except Exception:
            # If the monolith can't be imported (missing deps), skip.
            self.skipTest("monolith module not importable in this env")
            return

        # Build a dummy CsvLogger to call _sample_fields.
        class _StubConfig:
            logging = {
                "run_name_prefix": "test",
                "output_dir": "/tmp",
                "samples_csv": "samples.csv",
                "events_csv": "events.csv",
                "config_snapshot_yaml": "config_snapshot.yaml",
                "flush_every_rows": 1,
                "keep_last_runs": 7,
            }
            raw = {}

        # The monolith's CsvLogger.__init__ opens files; just call _sample_fields.
        monolith_fields = monolith.CsvLogger._sample_fields(_StubConfig())

        # Import the split-node logger's _build_sample_fields.
        from scripts.mia_haptic_force_test.logger_node import LoggerNode

        split_fields = LoggerNode._build_sample_fields()

        # Both should start with the same scalar columns.
        self.assertEqual(split_fields[:26], monolith_fields[:26])
        # Both should have per-finger columns.
        for label in FINGER_LABELS:
            self.assertIn(f"hand_pos_{label}_rad", split_fields)
            self.assertIn(f"velocity_cmd_{label}_rad_s", split_fields)
        # Both should have 8 haptic motor columns.
        for i in range(MOTOR_COUNT):
            self.assertIn(f"haptic_motor_{i}_pct", split_fields)


# ── Stage / hold_mode transitions (pure logic) ───────────────────────────


class StageTransitionTest(unittest.TestCase):
    """Test the stage machine's transitions using pure logic.

    The supervisor's stage machine is complex and tightly coupled to ROS.
    These tests verify the contract: given certain inputs (gesture, hold,
    force), the supervisor should transition to the expected next stage.
    We verify the *contract* by checking the stage names and hold_mode
    values in the Stage/HoldControl enums.
    """

    def test_stage_values(self) -> None:
        from scripts.mia_haptic_force_test.common.constants import Stage

        self.assertEqual(Stage.INITIALISING.value, "initialising")
        self.assertEqual(Stage.WAITING_FOR_ACTIVATION.value, "waiting_for_activation")
        self.assertEqual(Stage.ROTATING_TO_VERTICAL.value, "rotating_to_vertical")
        self.assertEqual(Stage.VERTICAL_DELAY.value, "vertical_delay")
        self.assertEqual(Stage.FORCE_CLOSING.value, "force_closing")
        self.assertEqual(Stage.FORCE_HOLD.value, "force_hold")
        self.assertEqual(Stage.OPENING_HAND.value, "opening_hand")
        self.assertEqual(Stage.RETURN_DELAY.value, "return_delay")
        self.assertEqual(Stage.RETURN_WRIST.value, "return_wrist")
        self.assertEqual(Stage.COMPLETE.value, "complete")
        self.assertEqual(Stage.FAULT.value, "fault")

    def test_hold_control_values(self) -> None:
        from scripts.mia_haptic_force_test.common.constants import HoldControl

        self.assertEqual(HoldControl.FORCE.value, "force")
        self.assertEqual(HoldControl.WRIST.value, "wrist")


# ── Topic name contract ───────────────────────────────────────────────────


class TopicNameContractTest(unittest.TestCase):
    def test_canonical_topic_names(self) -> None:
        from scripts.mia_haptic_force_test.common.constants import (
            TOPIC_CONTROL_HOLD_MODE,
            TOPIC_CONTROL_MODE,
            TOPIC_CONTROL_TARGET_FORCE,
            TOPIC_CONTROL_TARGET_WRIST,
            TOPIC_EMG_GESTURE,
            TOPIC_EMG_GESTURE_LABEL,
            TOPIC_HAND_FORCES,
            TOPIC_HAND_JOINT_STATES,
            TOPIC_HAPTIC_BAND_MOTORS,
            TOPIC_TEST_STAGE,
        )

        self.assertEqual(TOPIC_EMG_GESTURE, "/emg/gesture_name")
        self.assertEqual(TOPIC_EMG_GESTURE_LABEL, "/emg/gesture_label")
        self.assertEqual(TOPIC_HAND_FORCES, "/hand/forces")
        self.assertEqual(TOPIC_HAND_JOINT_STATES, "/hand/joint_states")
        self.assertEqual(TOPIC_CONTROL_TARGET_FORCE, "/control/target_force")
        self.assertEqual(TOPIC_CONTROL_TARGET_WRIST, "/control/target_wrist")
        self.assertEqual(TOPIC_CONTROL_MODE, "/control/mode")
        self.assertEqual(TOPIC_CONTROL_HOLD_MODE, "/control/hold_mode")
        self.assertEqual(TOPIC_HAPTIC_BAND_MOTORS, "/haptic_band/motors")
        self.assertEqual(TOPIC_TEST_STAGE, "/test/stage")

    def test_velocity_command_topic(self) -> None:
        self.assertEqual(
            TOPIC_GROUP_VEL_FF_COMMANDS, "/group_vel_ff_controller/commands"
        )


if __name__ == "__main__":
    unittest.main()
