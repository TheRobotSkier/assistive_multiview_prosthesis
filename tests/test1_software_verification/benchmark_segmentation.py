#!/usr/bin/env python3
"""Benchmark segmentation inference latency (CPU or CUDA backend).

Sends N requests to the local inference server and reports latency stats.
Uses a synthetic point cloud with a positive click to trigger real inference.

Usage:
    python benchmark_segmentation.py [--trials 100] [--points 5000]
"""

import base64
import csv
import json
import os
import statistics
import sys
import time

import numpy as np
import requests

URL = "http://127.0.0.1:5678/segment"
DEFAULT_TRIALS = 100
DEFAULT_POINTS = 5000


def build_payload(n_points=5000, n_clicks=1, seed=42):
    """Build a realistic segmentation payload with random points and clicks."""
    rng = np.random.default_rng(seed)
    xyz = rng.normal(loc=0.0, scale=0.05, size=(n_points, 3)).astype(np.float32)
    rgb = rng.uniform(0, 1, size=(n_points, 3)).astype(np.float32)
    centroid = xyz.mean(axis=0).tolist()
    pos_clicks = [centroid] * n_clicks
    return {
        "xyz": base64.b64encode(xyz.tobytes()).decode(),
        "rgb": base64.b64encode(rgb.tobytes()).decode(),
        "positive_clicks": pos_clicks,
        "negative_clicks": [],
        "cubeedge": 0.05,
    }


def main():
    trials = int(sys.argv[sys.argv.index("--trials") + 1]) if "--trials" in sys.argv else DEFAULT_TRIALS
    n_points = int(sys.argv[sys.argv.index("--points") + 1]) if "--points" in sys.argv else DEFAULT_POINTS

    # Health check
    try:
        resp = requests.get("http://127.0.0.1:5678/health", timeout=10)
        info = resp.json()
    except Exception as e:
        print(f"ERROR: Cannot reach inference server: {e}")
        sys.exit(1)

    device = info.get("device", "unknown")
    me_cuda = info.get("minkowski_engine_cuda", False)
    me_ver = info.get("minkowski_engine_version", "?")
    model_ready = info.get("model_ready", False)
    print(f"Server: device={device}, ME_cuda={me_cuda}, ME_version={me_ver}, model_ready={model_ready}")

    if not model_ready:
        print("ERROR: Model not loaded")
        sys.exit(1)

    # Warm-up
    payload = build_payload(n_points)
    resp = requests.post(URL, json=payload, timeout=120)
    if resp.status_code != 200:
        print(f"ERROR: Warm-up failed: {resp.status_code} {resp.text[:200]}")
        sys.exit(1)
    mask = resp.json()["mask"]
    print(f"Warm-up OK: {len(mask)} points, {sum(mask)} foreground\n")

    # Benchmark
    latencies = []
    fg_counts = []
    errors = 0
    rows = []

    for i in range(trials):
        payload = build_payload(n_points, seed=i)
        t0 = time.perf_counter()
        try:
            resp = requests.post(URL, json=payload, timeout=120)
            elapsed = (time.perf_counter() - t0) * 1000
            if resp.status_code != 200:
                errors += 1
                print(f"  Trial {i+1}: HTTP {resp.status_code}")
                rows.append({"trial": i+1, "latency_ms": round(elapsed, 2),
                             "n_points": n_points, "fg_points": -1, "status": f"http_{resp.status_code}"})
                continue
            result = resp.json()
            mask = result["mask"]
            fg = sum(mask)
            latencies.append(elapsed)
            fg_counts.append(fg)
            rows.append({"trial": i+1, "latency_ms": round(elapsed, 2),
                         "n_points": n_points, "fg_points": fg, "status": "ok"})
            if (i + 1) % 10 == 0:
                print(f"  {i+1}/{trials}: {elapsed:.1f} ms  (fg={fg})")
        except Exception as e:
            errors += 1
            elapsed = (time.perf_counter() - t0) * 1000
            rows.append({"trial": i+1, "latency_ms": round(elapsed, 2),
                         "n_points": n_points, "fg_points": -1, "status": f"error: {e}"})
            print(f"  Trial {i+1}: ERROR {e} ({elapsed:.1f} ms)")

    if not latencies:
        print(f"\nNo successful trials ({errors} errors)")
        sys.exit(1)

    latencies_sorted = sorted(latencies)
    n = len(latencies)

    summary = {
        "backend": device,
        "minkowski_engine_cuda": me_cuda,
        "minkowski_engine_version": me_ver,
        "n_trials": trials,
        "n_success": n,
        "n_errors": errors,
        "n_points": n_points,
        "mean_ms": round(statistics.mean(latencies), 2),
        "median_ms": round(statistics.median(latencies), 2),
        "stdev_ms": round(statistics.stdev(latencies), 2),
        "min_ms": round(min(latencies), 2),
        "max_ms": round(max(latencies), 2),
        "p90_ms": round(latencies_sorted[int(n * 0.90)], 2),
        "p95_ms": round(latencies_sorted[int(n * 0.95)], 2),
        "p99_ms": round(latencies_sorted[int(n * 0.99)], 2),
        "fg_mean": round(statistics.mean(fg_counts), 1),
        "fg_min": min(fg_counts),
        "fg_max": max(fg_counts),
    }

    # Save per-trial CSV (to /tmp inside container; host copies out)
    backend_tag = "cpu" if not me_cuda else "cuda"
    out_dir = "/tmp"

    trial_path = os.path.join(out_dir, f"segmentation_{backend_tag}_trials.csv")
    with open(trial_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["trial", "latency_ms", "n_points", "fg_points", "status"])
        w.writeheader()
        w.writerows(rows)
    print(f"\nPer-trial results saved to {trial_path}")

    # Save summary CSV
    summary_path = os.path.join(out_dir, f"segmentation_{backend_tag}_summary.csv")
    with open(summary_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary.keys()))
        w.writeheader()
        w.writerow(summary)
    print(f"Summary saved to {summary_path}")

    # Print summary
    print(f"\n{'='*60}")
    print(f"{backend_tag.upper()} Segmentation Benchmark ({n} trials, {errors} errors)")
    print(f"{'='*60}")
    print(f"  Mean:       {summary['mean_ms']:.1f} ms")
    print(f"  Median:     {summary['median_ms']:.1f} ms")
    print(f"  Stdev:      {summary['stdev_ms']:.1f} ms")
    print(f"  Min:        {summary['min_ms']:.1f} ms")
    print(f"  Max:        {summary['max_ms']:.1f} ms")
    print(f"  P90:        {summary['p90_ms']:.1f} ms")
    print(f"  P95:        {summary['p95_ms']:.1f} ms")
    print(f"  P99:        {summary['p99_ms']:.1f} ms")
    print(f"  FG points:  mean={summary['fg_mean']}, min={summary['fg_min']}, max={summary['fg_max']}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
