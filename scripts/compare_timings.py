#!/usr/bin/env python3
"""Compare JUnit XML test timings against a baseline JSON file.

Reads the current run's JUnit XML files and compares per-test durations
against a committed baseline. Flags regressions (tests that slowed down
by more than --threshold percent).

Usage:
    # Compare current run (in logs/test-results/) against baseline
    python3 scripts/compare_timings.py \
        --baseline tests/baselines/timings.json \
        --current-dir logs/test-results \
        --threshold 50

    # Capture a new baseline from the latest run
    python3 scripts/compare_timings.py \
        --capture tests/baselines/timings.json \
        --current-dir logs/test-results

Exit codes:
    0 — no regressions (or capture mode)
    1 — one or more tests regressed beyond threshold
    2 — usage error
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_junit_dir(current_dir: str) -> dict[str, float]:
    """Parse all JUnit XML files in a directory.

    Returns a dict mapping ``classname::name`` → duration in seconds.
    """
    timings: dict[str, float] = {}
    xml_dir = Path(current_dir)

    if not xml_dir.is_dir():
        print(f"WARNING: {current_dir} is not a directory", file=sys.stderr)
        return timings

    for xml_file in sorted(xml_dir.glob("*.xml")):
        try:
            tree = ET.parse(xml_file)
        except ET.ParseError as exc:
            print(f"WARNING: could not parse {xml_file}: {exc}", file=sys.stderr)
            continue

        for testcase in tree.iter("testcase"):
            classname = testcase.get("classname", "")
            name = testcase.get("name", "")
            # JUnit XML stores time in seconds as a string
            time_str = testcase.get("time", "0")
            try:
                duration = float(time_str)
            except ValueError:
                duration = 0.0
            key = f"{classname}::{name}" if classname else name
            timings[key] = duration

    return timings


def capture_baseline(output_path: str, current_dir: str) -> int:
    """Capture current timings as a new baseline file."""
    timings = parse_junit_dir(current_dir)
    if not timings:
        print(f"ERROR: no test timings found in {current_dir}", file=sys.stderr)
        return 1

    # Ensure the output directory exists
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    baseline = {
        "description": "Golden timing baseline for unit tests. "
        "Regenerate with: make test-baseline",
        "test_count": len(timings),
        "timings": dict(sorted(timings.items())),
    }
    out.write_text(json.dumps(baseline, indent=2) + "\n")
    print(f"Captured {len(timings)} test timings → {output_path}")
    return 0


def compare(
    baseline_path: str,
    current_dir: str,
    threshold_pct: float,
) -> int:
    """Compare current timings against baseline. Return 1 if regressions found."""
    try:
        baseline_data = json.loads(Path(baseline_path).read_text())
    except FileNotFoundError:
        print(f"ERROR: baseline file not found: {baseline_path}", file=sys.stderr)
        return 1
    except json.JSONDecodeError as exc:
        print(f"ERROR: invalid baseline JSON: {exc}", file=sys.stderr)
        return 1

    baseline_timings = baseline_data.get("timings", {})
    current_timings = parse_junit_dir(current_dir)

    if not current_timings:
        print("WARNING: no current timings found — skipping comparison")
        return 0

    regressions: list[tuple[str, float, float, float]] = []
    improvements: list[tuple[str, float, float, float]] = []
    new_tests: list[str] = []
    missing: list[str] = []

    for key, current_time in sorted(current_timings.items()):
        if key not in baseline_timings:
            new_tests.append(key)
            continue
        baseline_time = baseline_timings[key]
        if baseline_time <= 0:
            continue
        pct_change = ((current_time - baseline_time) / baseline_time) * 100.0
        if pct_change > threshold_pct:
            regressions.append((key, baseline_time, current_time, pct_change))
        elif pct_change < -threshold_pct:
            improvements.append((key, baseline_time, current_time, pct_change))

    for key in sorted(baseline_timings):
        if key not in current_timings:
            missing.append(key)

    # ── Report ────────────────────────────────────────────────────────────────
    if regressions:
        print(f"  REGRESSIONS (> {threshold_pct:.0f}% slower):")
        for key, old, new, pct in regressions:
            short = key.replace("::", ".")[:70]
            print(f"    \033[31m{pct:+6.0f}%\033[0m  {short}")
            print(f"           {old*1000:7.1f}ms → {new*1000:7.1f}ms")
        print()

    if improvements:
        print(f"  Improvements (> {threshold_pct:.0f}% faster):")
        for key, old, new, pct in improvements[:10]:
            short = key.replace("::", ".")[:70]
            print(f"    \033[32m{pct:+6.0f}%\033[0m  {short}")
            print(f"           {old*1000:7.1f}ms → {new*1000:7.1f}ms")
        if len(improvements) > 10:
            print(f"    ... and {len(improvements) - 10} more")
        print()

    if new_tests:
        print(f"  New tests (not in baseline): {len(new_tests)}")
        for key in new_tests[:5]:
            print(f"    + {key.replace('::', '.')[:70]}")
        if len(new_tests) > 5:
            print(f"    ... and {len(new_tests) - 5} more")
        print()

    if missing:
        print(f"  Missing tests (in baseline, not run): {len(missing)}")
        for key in missing[:5]:
            print(f"    - {key.replace('::', '.')[:70]}")
        if len(missing) > 5:
            print(f"    ... and {len(missing) - 5} more")
        print()

    # Summary line
    total = len(current_timings)
    stable = total - len(regressions) - len(improvements) - len(new_tests)
    print(f"  {total} tests: {stable} stable, "
          f"{len(regressions)} regressed, {len(improvements)} improved, "
          f"{len(new_tests)} new")

    if regressions:
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare or capture pytest timing baselines."
    )
    parser.add_argument(
        "--baseline",
        help="Path to the baseline JSON file (for comparison mode).",
    )
    parser.add_argument(
        "--capture",
        help="Capture current timings to this file (capture mode).",
    )
    parser.add_argument(
        "--current-dir",
        default="logs/test-results",
        help="Directory containing JUnit XML files from the current run.",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=50.0,
        help="Percentage threshold for flagging regressions (default: 50).",
    )
    args = parser.parse_args()

    if args.capture:
        return capture_baseline(args.capture, args.current_dir)

    if args.baseline:
        return compare(args.baseline, args.current_dir, args.threshold)

    parser.print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())
