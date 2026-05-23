#!/usr/bin/env python3
"""Analyze latency per-stage results."""
import csv
import math
import sys

SCRIPT_DIR = "/prosthesis_ws/tests/test1_software_verification"
RESULTS_DIR = f"{SCRIPT_DIR}/results"

def main():
    path = f"{RESULTS_DIR}/latency_per_stage_results.csv"
    try:
        with open(path) as f:
            reader = csv.DictReader(f)
            rows = list(reader)
    except FileNotFoundError:
        print(f"File not found: {path}")
        sys.exit(1)

    ok_rows = [r for r in rows if r["status"] == "ok"]

    print("=" * 90)
    print("PER-STAGE LATENCY RESULTS — All Successful Trials")
    print("=" * 90)

    # Per-object summary
    from collections import defaultdict
    groups = defaultdict(list)
    for r in ok_rows:
        groups[r["object"]].append(r)

    stage_fields = [
        ("pipeline_manager_ms", "PM"),
        ("twist_propagation_ms", "Twist"),
        ("segmentation_ms", "Seg"),
        ("pm_cloud_handling_ms", "PM_Cloud"),
        ("preshaping_ms", "Preshape"),
        ("total_ms", "Total"),
    ]

    print(f"\n{'Object':20s} {'n':>3s} |", end="")
    for _, label in stage_fields:
        print(f" {label:>8s}", end="")
    print()
    print("-" * 90)

    for obj in sorted(groups):
        obj_rows = groups[obj]
        print(f"{obj:20s} {len(obj_rows):>3d} |", end="")
        for field, _ in stage_fields:
            vals = [float(r[field]) for r in obj_rows if not math.isnan(float(r.get(field, "nan")))]
            if vals:
                mean = sum(vals) / len(vals)
                print(f" {mean:>8.1f}", end="")
            else:
                print(f" {'N/A':>8s}", end="")
        print()

    # Timeouts
    timeouts = [r for r in rows if r["status"] != "ok"]
    if timeouts:
        print(f"\nTIMEOUTS: {len(timeouts)}")
        for t in timeouts:
            print(f"  {t['object']:20s} rep {t['repetition']:>2s}: {t['n_events']} events")

    # Negative times
    neg = [r for r in ok_rows if float(r["pipeline_manager_ms"]) < 0]
    if neg:
        print(f"\nNEGATIVE PM TIMES: {len(neg)}")
        for n in neg:
            print(f"  {n['object']:20s} rep {n['repetition']:>2s}: pm={n['pipeline_manager_ms']}")

    # P95 check
    total_vals = [float(r["total_ms"]) for r in ok_rows if not math.isnan(float(r.get("total_ms", "nan")))]
    if total_vals:
        total_vals.sort()
        p95_idx = int(len(total_vals) * 0.95)
        p95 = total_vals[p95_idx]
        print(f"\nMAR CHECK (P95 <= 400 ms): {'PASS' if p95 <= 400 else 'FAIL'} (P95 = {p95:.1f})")
        print(f"IDE CHECK (P95 <= 100 ms): {'PASS' if p95 <= 100 else 'FAIL'} (P95 = {p95:.1f})")
        print(f"Total trials: {len(ok_rows)}/{len(rows)} successful")
        print(f"Min: {min(total_vals):.1f}, Max: {max(total_vals):.1f}")
        print(f"Mean: {sum(total_vals)/len(total_vals):.1f}")
        print(f"Median: {total_vals[len(total_vals)//2]:.1f}")

    # Stage percentage breakdown
    total_mean = sum(total_vals) / len(total_vals)
    print(f"\nPER-STAGE BREAKDOWN (% of total):")
    for field, label in stage_fields:
        if field == "total_ms":
            continue
        vals = [float(r[field]) for r in ok_rows if not math.isnan(float(r.get(field, "nan")))]
        if vals:
            stage_mean = sum(vals) / len(vals)
            pct = (stage_mean / total_mean) * 100
            print(f"  {label:>12s}: {stage_mean:>8.1f} ms ({pct:>5.1f}%)")
    print(f"  {'Total':>12s}: {total_mean:>8.1f} ms (100%)")


if __name__ == "__main__":
    main()