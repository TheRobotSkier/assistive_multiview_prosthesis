#!/usr/bin/env python3
"""Validation script for EmgGraspAdapter.

Runs without ROS installed (uses ImportError-safe stubs inside the adapter).
Usage:
    python3 test_emg_adapter.py
"""
from __future__ import annotations

import time
import sys
import os

# Adapter lives next to this script in the package root
from emg_grasp_adapter import EmgGraspAdapter, EmgSignal


_PASS = 0
_FAIL = 0


def _assert_equal(got, expected, label: str) -> bool:
    global _FAIL
    if got != expected:
        print(f"  FAIL [{label}]: expected {expected!r}, got {got!r}")
        _FAIL += 1
        return False
    return True


def _assert_true(val, label: str) -> bool:
    global _FAIL
    if not val:
        print(f"  FAIL [{label}]: expected True")
        _FAIL += 1
        return False
    return True


def _assert_false(val, label: str) -> bool:
    global _FAIL
    if val:
        print(f"  FAIL [{label}]: expected False")
        _FAIL += 1
        return False
    return True


def _print_pass(label: str) -> None:
    global _PASS
    _PASS += 1
    print(f"  PASS: {label}")


def test_default_aliases() -> bool:
    print("--- test_default_aliases ---")
    ok = True
    adapter = EmgGraspAdapter()

    sig = adapter.update_raw(gesture_name="POWER", confidence=0.9, proportional=0.5)
    ok &= _assert_equal(sig.normalized_gesture, "POWER", "POWER canonical")
    ok &= _assert_true(sig.is_valid, "POWER valid")
    _print_pass("POWER canonical") if ok else None

    # OPEN with strong control but not yet held long enough → neutral by default
    sig = adapter.update_raw(gesture_name="OPEN", confidence=0.9, proportional=0.8)
    ok &= _assert_equal(sig.normalized_gesture, "NEUTRAL", "OPEN not held → neutral")
    ok &= _assert_false(sig.is_valid, "OPEN not held invalid")
    _print_pass("OPEN safety default not-held → neutral")

    # POINT has no alias by default → NEUTRAL
    sig = adapter.update_raw(gesture_name="POINT", confidence=0.9, proportional=0.5)
    ok &= _assert_equal(sig.normalized_gesture, "NEUTRAL", "POINT no alias default")
    ok &= _assert_false(sig.is_valid, "POINT no alias invalid")
    _print_pass("POINT default no-alias") if ok else None

    return ok


def test_configured_aliases() -> bool:
    print("\n--- test_configured_aliases ---")
    ok = True
    adapter = EmgGraspAdapter(
        aliases={"PINCH": "FLEXION", "POINT": "EXTENSION"},
        stale_timeout_s=2.0,  # keep data alive across sleeps in this test
    )

    sig = adapter.update_raw(gesture_name="PINCH", confidence=0.9, proportional=0.5)
    ok &= _assert_equal(sig.normalized_gesture, "FLEXION", "PINCH→FLEXION")
    ok &= _assert_true(sig.is_valid, "PINCH→FLEXION valid")
    _print_pass("PINCH→FLEXION") if ok else None

    sig = adapter.update_raw(gesture_name="POINT", confidence=0.9, proportional=0.5)
    ok &= _assert_equal(sig.normalized_gesture, "EXTENSION", "POINT→EXTENSION")
    ok &= _assert_true(sig.is_valid, "POINT→EXTENSION valid")
    _print_pass("POINT→EXTENSION") if ok else None

    # Unconfigured alias falls back to raw name; OPEN stays OPEN once held
    sig = adapter.update_raw(gesture_name="OPEN", confidence=0.9, proportional=0.8)
    ok &= _assert_equal(sig.normalized_gesture, "NEUTRAL", "OPEN not held → neutral")
    ok &= _assert_false(sig.is_valid, "OPEN not held invalid with aliases")
    time.sleep(0.55)
    sig = adapter.get_signal()
    ok &= _assert_equal(sig.normalized_gesture, "OPEN", "OPEN held with aliases")
    ok &= _assert_true(sig.is_valid, "OPEN held valid with aliases")
    _print_pass("OPEN with aliases")

    return ok


def test_low_confidence() -> bool:
    print("\n--- test_low_confidence ---")
    ok = True
    adapter = EmgGraspAdapter(confidence_threshold=0.6)

    sig = adapter.update_raw(gesture_name="POWER", confidence=0.3, proportional=0.5)
    ok &= _assert_equal(sig.normalized_gesture, "NEUTRAL", "low conf → neutral")
    ok &= _assert_false(sig.is_valid, "low conf invalid")
    _print_pass("low confidence → neutral")

    # Confidence exactly at threshold → valid
    sig = adapter.update_raw(gesture_name="POWER", confidence=0.6, proportional=0.5)
    ok &= _assert_equal(sig.normalized_gesture, "POWER", "at-threshold valid")
    ok &= _assert_true(sig.is_valid, "at-threshold valid flag")
    _print_pass("at-threshold confidence") if ok else None

    return ok


def test_stale_data() -> bool:
    print("\n--- test_stale_data ---")
    ok = True
    adapter = EmgGraspAdapter(stale_timeout_s=0.1)

    adapter.update_raw(gesture_name="POWER", confidence=0.9, proportional=0.5)
    time.sleep(0.15)
    sig = adapter.get_signal()
    ok &= _assert_equal(sig.normalized_gesture, "NEUTRAL", "stale → neutral")
    ok &= _assert_false(sig.is_valid, "stale invalid")
    _print_pass("stale data → neutral")

    return ok


def test_open_safety() -> bool:
    print("\n--- test_open_safety ---")
    ok = True
    adapter = EmgGraspAdapter(
        open_min_control=0.7,
        open_min_hold_s=0.5,
        stale_timeout_s=2.0,  # keep data alive across the sleep below
    )

    # Not enough control → neutral
    sig = adapter.update_raw(gesture_name="OPEN", confidence=0.9, proportional=0.5)
    ok &= _assert_equal(sig.normalized_gesture, "NEUTRAL", "OPEN low control → neutral")
    ok &= _assert_false(sig.is_valid, "OPEN low control invalid")
    _print_pass("OPEN low control → neutral")

    # Enough control but not held long enough → neutral
    sig = adapter.update_raw(gesture_name="OPEN", confidence=0.9, proportional=0.8)
    ok &= _assert_equal(sig.normalized_gesture, "NEUTRAL", "OPEN not held → neutral")
    ok &= _assert_false(sig.is_valid, "OPEN not held invalid")
    _print_pass("OPEN not held → neutral")

    # After hold duration passes → OPEN valid
    time.sleep(0.55)
    sig = adapter.get_signal()
    ok &= _assert_equal(sig.normalized_gesture, "OPEN", "OPEN held+control → OPEN")
    ok &= _assert_true(sig.is_valid, "OPEN held valid")
    ok &= _assert_true(sig.hold_duration >= 0.5, "OPEN hold duration >= 0.5")
    _print_pass("OPEN held+control → valid")

    return ok


def test_rest_is_neutral() -> bool:
    print("\n--- test_rest_is_neutral ---")
    ok = True
    adapter = EmgGraspAdapter()

    sig = adapter.update_raw(gesture_name="REST", confidence=0.9, proportional=0.0)
    ok &= _assert_equal(sig.normalized_gesture, "NEUTRAL", "REST → NEUTRAL")
    ok &= _assert_false(sig.is_valid, "REST invalid")
    _print_pass("REST → neutral")

    return ok


def test_hold_duration_tracking() -> bool:
    print("\n--- test_hold_duration_tracking ---")
    ok = True
    adapter = EmgGraspAdapter()

    t0 = time.time()
    sig = adapter.update_raw(gesture_name="POWER", confidence=0.9, proportional=0.5)
    time.sleep(0.2)
    sig = adapter.get_signal()
    ok &= _assert_true(sig.hold_duration >= 0.15, "hold duration increases")
    ok &= _assert_equal(sig.normalized_gesture, "POWER", "gesture unchanged during hold")
    _print_pass("hold duration tracking")

    # Gesture change resets hold (OPEN not yet held long enough → neutral)
    time.sleep(0.05)
    sig = adapter.update_raw(gesture_name="OPEN", confidence=0.9, proportional=0.8)
    ok &= _assert_true(sig.hold_duration < 0.3, "hold resets on gesture change")
    ok &= _assert_equal(sig.normalized_gesture, "NEUTRAL", "OPEN not held → neutral")
    ok &= _assert_false(sig.is_valid, "OPEN not held invalid after change")
    _print_pass("hold reset on gesture change")

    return ok


def test_wrist_gesture_safety() -> bool:
    print("\n--- test_wrist_gesture_safety ---")
    ok = True
    adapter = EmgGraspAdapter(
        aliases={"PINCH": "FLEXION", "POINT": "EXTENSION"},
        confidence_threshold=0.7,
    )

    # FLEXION (aliased from PINCH) with low confidence → neutral
    sig = adapter.update_raw(gesture_name="PINCH", confidence=0.5, proportional=0.6)
    ok &= _assert_equal(sig.normalized_gesture, "NEUTRAL", "low-conf FLEXION → neutral")
    ok &= _assert_false(sig.is_valid, "low-conf FLEXION invalid")
    _print_pass("low-confidence wrist → neutral")

    # EXTENSION (aliased from POINT) with good confidence → valid
    sig = adapter.update_raw(gesture_name="POINT", confidence=0.8, proportional=0.6)
    ok &= _assert_equal(sig.normalized_gesture, "EXTENSION", "good-conf EXTENSION valid")
    ok &= _assert_true(sig.is_valid, "good-conf EXTENSION valid flag")
    _print_pass("good-confidence wrist → valid")

    return ok


def run_all() -> bool:
    tests = [
        test_default_aliases,
        test_configured_aliases,
        test_low_confidence,
        test_stale_data,
        test_open_safety,
        test_rest_is_neutral,
        test_hold_duration_tracking,
        test_wrist_gesture_safety,
    ]
    all_ok = True
    for t in tests:
        passed = t()
        if not passed:
            all_ok = False
    return all_ok


if __name__ == "__main__":
    print("=" * 60)
    print("EMG Grasp Adapter validation")
    print("=" * 60 + "\n")

    ok = run_all()

    print("\n" + "=" * 60)
    print(f"Results: {_PASS} passed, {_FAIL} failed")
    print("=" * 60)

    sys.exit(0 if ok else 1)
