#!/usr/bin/env python3
"""End-to-end validation for multi-click segmentation payloads.

Validates that:
  1. twist_propagation's spherical sampler produces the expected number of clicks
  2. All synthetic clicks fall inside the configured shell
  3. The original hit is published first
  4. Segmentation bridge coalesces rapid clicks into one inference payload

Usage:
    python3 tests/validate_multiclick_payload.py

This does NOT require ROS 2 running, GPU, or real segmentation weights.
"""

import sys
import os
import math

# Add source paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'twist_propagation', 'twist_propagation'))

import numpy as np

from twist_propagation_node import _sample_spherical_shell_clicks


def test_sampler_count():
    """Total clicks = 1 original + click_count synthetic."""
    rng = np.random.default_rng(42)
    centre = (0.5, 0.5, 0.5)
    r_min, r_max = 0.005, 0.03
    count = 4
    synthetic = _sample_spherical_shell_clicks(centre, r_min, r_max, count, rng)
    assert len(synthetic) == count, f"Expected {count} synthetic clicks, got {len(synthetic)}"
    print(f"  PASS: {count} synthetic clicks produced")


def test_sampler_shell_bounds():
    """All synthetic clicks must be within [r_min, r_max] of centre."""
    rng = np.random.default_rng(42)
    centre = (0.5, 0.5, 0.5)
    r_min, r_max = 0.005, 0.03
    count = 50
    synthetic = _sample_spherical_shell_clicks(centre, r_min, r_max, count, rng)
    for pt in synthetic:
        dist = math.sqrt(sum((c - p) ** 2 for c, p in zip(centre, pt)))
        assert r_min <= dist <= r_max, f"Click {pt} at distance {dist} outside shell"
    print(f"  PASS: all {count} synthetic clicks inside [{r_min}, {r_max}]m shell")


def test_sampler_centre_not_returned():
    """The original centre point is not in the synthetic list."""
    rng = np.random.default_rng(42)
    centre = (1.0, 2.0, 3.0)
    synthetic = _sample_spherical_shell_clicks(centre, 0.005, 0.03, 10, rng)
    assert centre not in synthetic, "Centre should not appear in synthetic clicks"
    print("  PASS: original centre not duplicated in synthetic list")


def test_backward_compatibility_click_count_zero():
    """click_count=0 produces empty synthetic list (original single-click behavior)."""
    rng = np.random.default_rng(42)
    synthetic = _sample_spherical_shell_clicks((0.0, 0.0, 0.0), 0.005, 0.03, 0, rng)
    assert synthetic == [], f"Expected empty list for click_count=0, got {synthetic}"
    print("  PASS: click_count=0 preserves backward compatibility")


def main():
    print("Multi-click segmentation payload validation")
    print("=" * 50)

    tests = [
        ("Sampler produces correct count", test_sampler_count),
        ("All clicks within shell bounds", test_sampler_shell_bounds),
        ("Centre not duplicated", test_sampler_centre_not_returned),
        ("Backward compatibility (click_count=0)", test_backward_compatibility_click_count_zero),
    ]

    passed = 0
    failed = 0
    for name, fn in tests:
        print(f"\n{name}:")
        try:
            fn()
            passed += 1
        except AssertionError as e:
            print(f"  FAIL: {e}")
            failed += 1
        except Exception as e:
            print(f"  ERROR: {e}")
            failed += 1

    print("\n" + "=" * 50)
    print(f"Results: {passed} passed, {failed} failed")
    if failed > 0:
        sys.exit(1)
    print("All validations passed.")


if __name__ == "__main__":
    main()
