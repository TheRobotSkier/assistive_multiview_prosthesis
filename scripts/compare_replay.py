#!/usr/bin/env python3
"""compare_replay.py — compare replay-test metrics against a baseline.

Reads two JSON files produced by analyze_bag.py --json / analyze_log.py --json
(or a combined dict containing both), and reports regressions/improvements
with colored output and a tolerance-based pass/fail.

Usage:
  # Compare a single combined metrics file against the baseline
  python3 scripts/compare_replay.py \
      --baseline tests/baselines/replay.json \
      --current  logs/replay-results/<run>/metrics.json

  # Compare separate bag + log metrics files
  python3 scripts/compare_replay.py \
      --baseline tests/baselines/replay.json \
      --current-bag  logs/.../bag_metrics.json \
      --current-log  logs/.../log_metrics.json

Exit codes:
  0 = no regressions (or no baseline found)
  1 = one or more metrics regressed beyond tolerance
  2 = usage error
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any

# ANSI colors (disabled if not a TTY)
if sys.stderr.isatty():
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    CYAN = "\033[36m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    RESET = "\033[0m"
else:
    RED = GREEN = YELLOW = CYAN = BOLD = DIM = RESET = ""

# Metrics where "higher is worse" (regression = increase beyond tolerance).
# Everything else defaults to "lower is worse" (regression = decrease).
HIGHER_IS_WORSE = {
    # Bag metrics
    "max_trans_jump_m", "max_rot_jump_deg", "max_velocity_mps",
    "n_trans_jumps", "n_rot_jumps", "n_velocity_warns",
    "trans_delta_mean_m", "trans_delta_max_m",
    "rot_delta_mean_deg", "rot_delta_max_deg",
    "trans_divergence_rate_mmps",
    "tf_jumps_total",
    # Sysmon
    "host_cpu_pct", "host_mem_pct", "host_gpu_util_pct", "host_gpu_mem_mb",
    # Log metrics
    "clock_offset_peak_s", "tf_jump_events_total",
    "n_errors", "n_crashes",
}

# Metrics where "higher is better" (regression = decrease beyond tolerance).
LOWER_IS_WORSE = {
    # Topic throughput — fewer messages = pipeline starved
    "count",
    # Pose path length — shorter trajectory = less motion processed (starvation)
    "path_length_m",
    "n", "n_pairs",
}

# Relative tolerance: a metric must change by more than this fraction to count
# as a regression. Generous by default because GTSAM/SIFT/TSDF are not bit-
# reproducible across runs.
DEFAULT_TOLERANCE = 0.25  # 25%

# Per-metric tolerance overrides (fraction). Tighter for counts (they're
# integers and less noisy), looser for floating-point pose stats.
TOLERANCE_OVERRIDES: dict[str, float] = {
    "count": 0.10,           # message counts should be stable
    "effective_hz": 0.15,    # rates
    "min_hz": 0.20,
    "max_hz": 0.20,
    "n": 0.15,
    "n_pairs": 0.15,
    "path_length_m": 0.15,
    "max_trans_jump_m": 0.50,  # jump magnitudes are noisy
    "max_rot_jump_deg": 0.50,
    "max_velocity_mps": 0.50,
    "trans_delta_mean_m": 0.40,
    "trans_delta_max_m": 0.50,
    "rot_delta_mean_deg": 0.40,
    "trans_divergence_rate_mmps": 0.60,  # regression slopes are very noisy
}


def _flatten(d: dict, prefix: str = "") -> dict[str, Any]:
    """Flatten a nested dict into dotted-path keys, skipping dicts/lists of dicts."""
    out: dict[str, Any] = {}
    for k, v in d.items():
        key = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            # If all values are scalars, keep them as flattened keys
            sub = _flatten(v, key)
            out.update(sub)
        elif isinstance(v, (int, float)):
            out[key] = v
        # skip strings, None, lists
    return out


def _tolerance_for(metric: str) -> float:
    """Look up tolerance for a metric (checks suffix matches)."""
    # Exact match first
    if metric in TOLERANCE_OVERRIDES:
        return TOLERANCE_OVERRIDES[metric]
    # Suffix match (e.g. "topics./jetson/head/points.count" → "count")
    parts = metric.split(".")
    for part in reversed(parts):
        if part in TOLERANCE_OVERRIDES:
            return TOLERANCE_OVERRIDES[part]
    return DEFAULT_TOLERANCE


def _is_regression(metric: str, old: float, new: float, tol: float) -> tuple[bool, float]:
    """Return (is_regression, pct_change). Handles higher/lower-is-worse semantics."""
    if old == 0:
        # If baseline is zero, any nonzero new value is a regression for
        # higher-is-worse metrics; for lower-is-worse, zero→nonzero is an improvement.
        if metric in HIGHER_IS_WORSE:
            return (new > 0, float("inf") if new > 0 else 0.0)
        return (False, 0.0)
    pct = (new - old) / abs(old)
    if metric in HIGHER_IS_WORSE:
        return (pct > tol, pct)
    if metric in LOWER_IS_WORSE:
        return (pct < -tol, pct)
    # Default: treat as higher-is-worse (safer — flags unexpected increases)
    return (pct > tol, pct)


def compare(baseline: dict, current: dict, tolerance: float | None = None) -> tuple[list, list, list]:
    """Compare two flattened metric dicts. Returns (regressions, improvements, stable)."""
    bl = _flatten(baseline)
    cur = _flatten(current)
    regressions: list[tuple[str, float, float, float]] = []
    improvements: list[tuple[str, float, float, float]] = []
    stable: list[tuple[str, float, float, float]] = []

    all_keys = sorted(set(bl) | set(cur))
    for key in all_keys:
        old = bl.get(key)
        new = cur.get(key)
        if old is None or new is None:
            continue
        tol = tolerance if tolerance is not None else _tolerance_for(key)
        is_reg, pct = _is_regression(key, old, new, tol)
        if is_reg:
            regressions.append((key, old, new, pct))
        elif abs(pct) > tol and pct < 0 and key not in HIGHER_IS_WORSE:
            improvements.append((key, old, new, pct))
        elif abs(pct) > tol and pct > 0 and key in LOWER_IS_WORSE:
            improvements.append((key, old, new, pct))
        else:
            stable.append((key, old, new, pct))
    return regressions, improvements, stable


def _fmt_val(v: float) -> str:
    if isinstance(v, float):
        if abs(v) < 0.01 and v != 0:
            return f"{v:.5f}"
        return f"{v:.3f}"
    return str(v)


def _fmt_pct(pct: float) -> str:
    if pct == float("inf"):
        return "+inf%"
    if pct == float("-inf"):
        return "-inf%"
    return f"{pct*100:+.1f}%"


def print_report(regressions, improvements, stable, out=sys.stdout):
    p = lambda *a, **kw: print(*a, file=out, **kw)
    p("")
    p(f"{BOLD}REPLAY METRIC COMPARISON{RESET}")
    p(f"  {len(regressions)} regression(s)  {GREEN}{len(improvements)} improvement(s){RESET}  "
      f"{DIM}{len(stable)} stable{RESET}")
    p("")

    if regressions:
        p(f"{RED}{BOLD}REGRESSIONS (exceeded tolerance):{RESET}")
        for key, old, new, pct in sorted(regressions, key=lambda x: -abs(x[3])):
            arrow = "↑" if new > old else "↓"
            p(f"  {RED}{arrow} {key:<55s} {_fmt_val(old):>12s} → {_fmt_val(new):>12s}  "
              f"{_fmt_pct(pct):>8s}{RESET}")
        p("")

    if improvements:
        p(f"{GREEN}{BOLD}IMPROVEMENTS:{RESET}")
        for key, old, new, pct in sorted(improvements, key=lambda x: -abs(x[3]))[:10]:
            arrow = "↓" if new < old else "↑"
            p(f"  {GREEN}{arrow} {key:<55s} {_fmt_val(old):>12s} → {_fmt_val(new):>12s}  "
              f"{_fmt_pct(pct):>8s}{RESET}")
        if len(improvements) > 10:
            p(f"  {DIM}... and {len(improvements)-10} more{RESET}")
        p("")

    if stable and (regressions or improvements):
        p(f"{DIM}Stable metrics ({len(stable)} within tolerance):{RESET}")
        for key, old, new, pct in stable[:5]:
            p(f"  {DIM}  {key:<55s} {_fmt_val(old):>12s} → {_fmt_val(new):>12s}  "
              f"{_fmt_pct(pct):>8s}{RESET}")
        if len(stable) > 5:
            p(f"  {DIM}... and {len(stable)-5} more{RESET}")
        p("")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--baseline", required=True, help="Path to baseline metrics JSON")
    grp = ap.add_mutually_exclusive_group(required=True)
    grp.add_argument("--current", help="Path to current metrics JSON (combined)")
    grp.add_argument("--current-bag", help="Path to current bag metrics JSON")
    ap.add_argument("--current-log", help="Path to current log metrics JSON")
    ap.add_argument("--threshold", type=float, default=None,
                    help=f"Override tolerance for ALL metrics (default: {DEFAULT_TOLERANCE*100:.0f}%)")
    args = ap.parse_args()

    if not os.path.exists(args.baseline):
        print(f"{YELLOW}No baseline found at {args.baseline}.{RESET}", file=sys.stderr)
        print(f"{YELLOW}Run with --capture first to create one.{RESET}", file=sys.stderr)
        sys.exit(0)

    with open(args.baseline) as f:
        baseline = json.load(f)

    if args.current:
        with open(args.current) as f:
            current = json.load(f)
    else:
        current: dict = {}
        if args.current_bag:
            with open(args.current_bag) as f:
                current["bag"] = json.load(f)
        if args.current_log:
            with open(args.current_log) as f:
                current["log"] = json.load(f)

    regressions, improvements, stable = compare(baseline, current, args.threshold)
    print_report(regressions, improvements, stable)

    if regressions:
        print(f"{RED}{BOLD}FAIL: {len(regressions)} metric(s) regressed.{RESET}", file=sys.stderr)
        sys.exit(1)
    else:
        print(f"{GREEN}PASS: no regressions detected.{RESET}", file=sys.stderr)
        sys.exit(0)


if __name__ == "__main__":
    main()
