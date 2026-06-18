#!/usr/bin/env python3
"""Unit tests for pure haptic force test conversion helpers."""

from __future__ import annotations

import os
import sys
import unittest

_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.conversions import hold_velocity


class HoldVelocityTest(unittest.TestCase):
    def test_deadzone_outputs_zero(self) -> None:
        self.assertEqual(
            hold_velocity(
                force=285.0,
                target=300.0,
                deadzone=20.0,
                max_velocity=0.08,
                min_overshoot=20.0,
                max_overshoot=150.0,
            ),
            0.0,
        )

    def test_undershoot_ramps_from_deadzone_to_max_velocity(self) -> None:
        low = hold_velocity(
            force=260.0,
            target=300.0,
            deadzone=20.0,
            max_velocity=0.08,
            min_overshoot=20.0,
            max_overshoot=150.0,
        )
        high = hold_velocity(
            force=100.0,
            target=300.0,
            deadzone=20.0,
            max_velocity=0.08,
            min_overshoot=20.0,
            max_overshoot=150.0,
        )

        self.assertGreater(low, 0.0)
        self.assertLess(low, 0.08)
        self.assertAlmostEqual(low, 0.08 * ((40.0 - 20.0) / (150.0 - 20.0)))
        self.assertEqual(high, 0.08)

    def test_overshoot_remains_negative_and_bounded(self) -> None:
        proportional = hold_velocity(
            force=350.0,
            target=300.0,
            deadzone=20.0,
            max_velocity=0.08,
            min_overshoot=20.0,
            max_overshoot=150.0,
        )
        saturated = hold_velocity(
            force=500.0,
            target=300.0,
            deadzone=20.0,
            max_velocity=0.08,
            min_overshoot=20.0,
            max_overshoot=150.0,
        )

        self.assertLess(proportional, 0.0)
        self.assertAlmostEqual(proportional, -0.08 * (50.0 / 150.0))
        self.assertEqual(saturated, -0.08)


if __name__ == "__main__":
    unittest.main()
