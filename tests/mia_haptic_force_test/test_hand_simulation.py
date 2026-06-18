#!/usr/bin/env python3
"""Pure unit tests for the hand/wrist simulator helper."""

from __future__ import annotations

import math
import os
import sys
import unittest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.hand_simulation import (
    DEFAULT_FINGER_CONTACT_RAD,
    DEFAULT_FINGER_MAX_CLOSURE_RAD,
    DEFAULT_FINGER_MAX_FORCE_N,
    DEFAULT_FINGER_OPEN_RAD,
    DEFAULT_FINGER_STIFFNESS_N_PER_RAD,
    FINGER_COUNT,
    HandSimState,
    default_sim_config,
    open_hand,
    sim_config_from,
    step_hand_simulation,
)


class HandSimStateDefaultsTest(unittest.TestCase):
    def test_default_state(self) -> None:
        state = HandSimState()
        self.assertEqual(state.finger_positions_rad, DEFAULT_FINGER_OPEN_RAD)
        self.assertEqual(state.finger_velocities_rad_s, (0.0, 0.0, 0.0))
        self.assertEqual(state.finger_forces_n, (0.0, 0.0, 0.0))
        self.assertEqual(state.finger_contacted, (False, False, False))
        self.assertEqual(state.wrist_position_deg, 180.0)
        self.assertEqual(state.wrist_velocity_deg_s, 0.0)
        self.assertEqual(state.step_count, 0)

    def test_state_is_frozen(self) -> None:
        state = HandSimState()
        with self.assertRaises(Exception):
            state.finger_positions_rad = (0.1,) * FINGER_COUNT  # type: ignore[misc]

    def test_default_config_keys(self) -> None:
        cfg = default_sim_config()
        for key in (
            "open_rad",
            "max_closure_rad",
            "contact_rad",
            "stiffness_n_per_rad",
            "max_force_n",
            "idle_force_n",
            "wrist_open_deg",
            "wrist_max_velocity_deg_s",
            "wrist_acceleration_deg_s2",
            "wrist_min_deg",
            "wrist_max_deg",
        ):
            self.assertIn(key, cfg)
        self.assertEqual(cfg["contact_rad"], DEFAULT_FINGER_CONTACT_RAD)
        self.assertEqual(cfg["max_force_n"], DEFAULT_FINGER_MAX_FORCE_N)
        self.assertEqual(cfg["max_closure_rad"], DEFAULT_FINGER_MAX_CLOSURE_RAD)
        self.assertEqual(cfg["stiffness_n_per_rad"], DEFAULT_FINGER_STIFFNESS_N_PER_RAD)


class SimConfigFromTest(unittest.TestCase):
    def test_legacy_hand_dict(self) -> None:
        raw = {
            "hand": {
                "open_positions": {"j_thumb_fle": 0.1, "j_index_fle": 0.2, "j_mrl_fle": 0.3},
                "max_closure_positions": {"j_thumb_fle": 1.0, "j_index_fle": 1.0, "j_mrl_fle": 1.0},
            },
            "wrist": {
                "horizontal_deg": 90.0,
                "control_velocity_deg_s": 30.0,
                "acceleration_deg_s2": 90.0,
                "min_deg": 0.0,
                "max_deg": 270.0,
            },
        }
        cfg = sim_config_from(raw)
        self.assertEqual(cfg["open_rad"], (0.1, 0.2, 0.3))
        self.assertEqual(cfg["max_closure_rad"], (1.0, 1.0, 1.0))
        self.assertEqual(cfg["wrist_open_deg"], 90.0)
        self.assertEqual(cfg["wrist_max_velocity_deg_s"], 30.0)
        self.assertEqual(cfg["wrist_acceleration_deg_s2"], 90.0)
        self.assertEqual(cfg["wrist_min_deg"], 0.0)
        self.assertEqual(cfg["wrist_max_deg"], 270.0)

    def test_missing_keys_use_defaults(self) -> None:
        cfg = sim_config_from({})
        self.assertEqual(cfg["open_rad"], DEFAULT_FINGER_OPEN_RAD)
        self.assertEqual(cfg["wrist_open_deg"], 180.0)

    def test_non_dict_hand_section_falls_back(self) -> None:
        cfg = sim_config_from({"hand": "not a dict"})
        self.assertEqual(cfg["open_rad"], DEFAULT_FINGER_OPEN_RAD)


class StepHandSimulationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.state = HandSimState()
        self.cfg = default_sim_config()

    def test_no_command_keeps_state(self) -> None:
        next_state = step_hand_simulation(self.state, dt_s=0.01, cfg=self.cfg)
        self.assertEqual(next_state.finger_positions_rad, self.state.finger_positions_rad)
        self.assertEqual(next_state.finger_forces_n, (0.0, 0.0, 0.0))
        self.assertEqual(next_state.finger_contacted, (False, False, False))
        self.assertEqual(next_state.step_count, 1)

    def test_velocity_command_moves_finger(self) -> None:
        next_state = step_hand_simulation(
            self.state,
            finger_vel_cmd=[0.1, 0.1, 0.1],
            dt_s=0.01,
            cfg=self.cfg,
        )
        self.assertAlmostEqual(next_state.finger_positions_rad[0], 0.001, places=6)
        self.assertAlmostEqual(next_state.finger_velocities_rad_s[0], 0.1, places=6)
        self.assertEqual(next_state.finger_forces_n, (0.0, 0.0, 0.0))
        self.assertEqual(next_state.finger_contacted, (False, False, False))

    def test_position_command_clamps_to_max_closure(self) -> None:
        next_state = step_hand_simulation(
            self.state,
            finger_pos_cmd=[5.0, 5.0, 5.0],
            dt_s=0.01,
            cfg=self.cfg,
        )
        self.assertEqual(next_state.finger_positions_rad[0], DEFAULT_FINGER_MAX_CLOSURE_RAD[0])
        self.assertEqual(next_state.finger_velocities_rad_s[0], 0.0)

    def test_force_rises_after_contact(self) -> None:
        cfg = {**self.cfg, "contact_rad": (0.1, 0.1, 0.1)}
        # Move past the contact point in one step.
        next_state = step_hand_simulation(
            self.state,
            finger_pos_cmd=[0.2, 0.2, 0.2],
            dt_s=0.01,
            cfg=cfg,
        )
        self.assertEqual(next_state.finger_contacted, (True, True, True))
        self.assertGreater(next_state.finger_forces_n[0], 0.0)
        # Force = stiffness * (pos - contact).
        expected = DEFAULT_FINGER_STIFFNESS_N_PER_RAD[0] * 0.1
        self.assertAlmostEqual(next_state.finger_forces_n[0], expected, places=4)

    def test_force_clamps_to_max(self) -> None:
        cfg = {**self.cfg, "contact_rad": (0.0, 0.0, 0.0), "stiffness_n_per_rad": (1e6, 1e6, 1e6)}
        next_state = step_hand_simulation(
            self.state,
            finger_pos_cmd=[1.0, 1.0, 1.0],
            dt_s=0.01,
            cfg=cfg,
        )
        self.assertEqual(next_state.finger_forces_n[0], DEFAULT_FINGER_MAX_FORCE_N[0])

    def test_force_drops_when_finger_reopens(self) -> None:
        cfg = {**self.cfg, "contact_rad": (0.1, 0.1, 0.1)}
        contacted = step_hand_simulation(
            self.state,
            finger_pos_cmd=[0.2, 0.2, 0.2],
            dt_s=0.01,
            cfg=cfg,
        )
        reopened = step_hand_simulation(
            contacted,
            finger_pos_cmd=[0.0, 0.0, 0.0],
            dt_s=0.01,
            cfg=cfg,
        )
        self.assertEqual(reopened.finger_contacted, (False, False, False))
        self.assertEqual(reopened.finger_forces_n, (0.0, 0.0, 0.0))

    def test_wrist_slews_toward_target(self) -> None:
        next_state = step_hand_simulation(
            self.state,
            wrist_target_deg=90.0,
            dt_s=0.1,
            cfg={**self.cfg, "wrist_max_velocity_deg_s": 60.0},
        )
        # The wrist was at 180 deg, target 90 deg, max velocity 60 deg/s,
        # dt 0.1s -> max_step=6.0, max_accel_step=45*0.01=0.45.
        # step = clamp(-90, -6.45, 6.45) = -6.45. New position = 173.55.
        self.assertAlmostEqual(next_state.wrist_position_deg, 173.55, places=4)
        self.assertAlmostEqual(next_state.wrist_velocity_deg_s, -64.5, places=4)

    def test_wrist_respects_min_max(self) -> None:
        next_state = step_hand_simulation(
            self.state,
            wrist_target_deg=500.0,
            dt_s=0.1,
            cfg={**self.cfg, "wrist_max_deg": 270.0},
        )
        self.assertLessEqual(next_state.wrist_position_deg, 270.0)

    def test_wrist_unchanged_without_target(self) -> None:
        next_state = step_hand_simulation(self.state, dt_s=0.1, cfg=self.cfg)
        self.assertEqual(next_state.wrist_position_deg, self.state.wrist_position_deg)

    def test_short_command_fills_with_zeros(self) -> None:
        next_state = step_hand_simulation(
            self.state,
            finger_vel_cmd=[0.1],
            dt_s=0.01,
            cfg=self.cfg,
        )
        # Thumb gets the velocity, index and mrl get 0.0.
        self.assertAlmostEqual(next_state.finger_positions_rad[0], 0.001, places=6)
        self.assertEqual(next_state.finger_positions_rad[1], 0.0)
        self.assertEqual(next_state.finger_positions_rad[2], 0.0)

    def test_none_command_falls_back_to_state(self) -> None:
        next_state = step_hand_simulation(
            self.state,
            finger_vel_cmd=None,
            finger_pos_cmd=None,
            dt_s=0.01,
            cfg=self.cfg,
        )
        self.assertEqual(next_state.finger_positions_rad, self.state.finger_positions_rad)

    def test_step_count_increments(self) -> None:
        s1 = step_hand_simulation(self.state, dt_s=0.01, cfg=self.cfg)
        s2 = step_hand_simulation(s1, dt_s=0.01, cfg=self.cfg)
        self.assertEqual(s1.step_count, 1)
        self.assertEqual(s2.step_count, 2)


class OpenHandTest(unittest.TestCase):
    def test_open_hand_resets_to_open(self) -> None:
        state = HandSimState(
            finger_positions_rad=(0.5, 0.5, 0.5),
            finger_velocities_rad_s=(0.1, 0.1, 0.1),
            finger_forces_n=(100.0, 100.0, 100.0),
            finger_contacted=(True, True, True),
        )
        reset = open_hand(state)
        self.assertEqual(reset.finger_positions_rad, DEFAULT_FINGER_OPEN_RAD)
        self.assertEqual(reset.finger_velocities_rad_s, (0.0, 0.0, 0.0))
        self.assertEqual(reset.finger_forces_n, (0.0, 0.0, 0.0))
        self.assertEqual(reset.finger_contacted, (False, False, False))


if __name__ == "__main__":
    unittest.main()
