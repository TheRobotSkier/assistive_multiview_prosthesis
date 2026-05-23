#!/usr/bin/env python3
"""Score-vs-samples sweep for Test 1.

Runs the grasp planner with different prediction_samples counts (1K–100K)
and records scores, latencies, and grasp types.

Each sweep config runs in a SEPARATE subprocess so the Rust OnceLock reloads fresh.
One subprocess per (object, sample_count) pair handles all reps in batch.
"""

import argparse
import csv
import os
import subprocess
import sys
import tempfile
import re
import time


SWEEP_COUNTS = [1000, 2000, 5000, 10000, 20000, 50000, 100000]
SWEEP_REPS = 30
OUTPUT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "results", "score_sweep_results.csv")


def _build_batch_script(object_name: str, config_path: str, n_reps: int) -> str:
    """Return a Python script string that runs n_reps compute calls in a batch."""
    return f"""import sys, os, json, time, numpy as np
sys.path.insert(0, "{os.path.dirname(os.path.abspath(__file__))}")
os.environ["GRASP_CONFIG_PATH"] = "{config_path}"
from ffi_bridge import GraspLibrary, make_pose, make_twist, make_request, response_to_dict
from object_registry import load_object
from hand_approaches import get_approach

lib = GraspLibrary()
obj = load_object("{object_name}")
approach = get_approach("{object_name}")
full_cloud = obj["points"]
pose = make_pose(**approach["pose"])
twist = make_twist(0.10, 0, 0, 0, 0, 0)
cameras = [(0.0, 0.0, 0.0)]

for rep in range({n_reps}):
    rng = np.random.default_rng(rep * 99991 + 42)
    idx = rng.choice(len(full_cloud), min(30000, len(full_cloud)), replace=False)
    cloud = full_cloud[idx]
    req = make_request(pose, twist, cloud, cameras)
    msg = ""
    t0 = time.perf_counter()
    status, resp, msg = lib.compute(req)
    t1 = time.perf_counter()
    latency_ms = (t1 - t0) * 1000
    if status == 0:
        d = response_to_dict(resp)
        row = dict(object="{object_name}", condition="single_view",
                   combined_score=d.get("combined_score", 0),
                   contact_score=d.get("contact_score", 0),
                   grasp_type_name=d.get("grasp_type_name", "unknown"),
                   latency_ms=round(latency_ms, 2))
    else:
        row = dict(object="{object_name}", condition="single_view",
                   combined_score=0, contact_score=0,
                   grasp_type_name=f"error_{{msg}}", latency_ms=round(latency_ms, 2))
    print(json.dumps(row), flush=True)
"""


def create_temp_config(base_config_path: str, prediction_samples: int) -> str:
    """Create a temp config with modified prediction_samples + scaled iterations."""
    with open(base_config_path) as f:
        config = f.read()
    config = re.sub(r"prediction_samples:\s*\d+", f"prediction_samples: {prediction_samples}", config)
    base_samples = 20000
    base_iters = 5
    scaled_iters = max(2, round(base_iters * (prediction_samples / base_samples) ** 0.3))
    config = re.sub(r"em_iterations:\s*\d+", f"em_iterations: {scaled_iters}", config)
    fd, path = tempfile.mkstemp(suffix=".yaml", prefix=f"sweep_{prediction_samples}_")
    with os.fdopen(fd, "w") as f:
        f.write(config)
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--counts", default=",".join(str(c) for c in SWEEP_COUNTS))
    parser.add_argument("--reps", type=int, default=SWEEP_REPS)
    parser.add_argument("--objects", nargs="+", default=None)
    parser.add_argument("--base-config",
                        default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                             "config", "grasp_preshaping.yaml"))
    args = parser.parse_args()

    sample_counts = [int(c) for c in args.counts.split(",")]
    base_config = os.path.abspath(args.base_config)

    os.makedirs(os.path.dirname(OUTPUT_CSV), exist_ok=True)

    # Write CSV header
    with open(OUTPUT_CSV, "w") as f:
        f.write("object,prediction_samples,condition,combined_score,contact_score,grasp_type_name,latency_ms\n")

    # Discover objects
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from object_registry import list_objects
    objects = args.objects or sorted(list_objects())

    total = len(objects) * len(sample_counts) * args.reps
    done = 0
    t_start = time.perf_counter()

    print(f"Sweep: {len(objects)} objects x {len(sample_counts)} sample counts x {args.reps} reps = {total} trials\n")

    for obj_name in objects:
        for n_samples in sample_counts:
            config_path = create_temp_config(base_config, n_samples)
            script = _build_batch_script(obj_name, config_path, args.reps)

            label = f"{obj_name}@{n_samples:>6d}"
            t0_batch = time.perf_counter()

            result = subprocess.run(
                ["python3", "-c", script],
                capture_output=True, text=True, timeout=300,
            )

            elapsed_batch = time.perf_counter() - t0_batch
            done += args.reps

            if result.returncode != 0:
                print(f"  [FAIL] {label} — stderr: {result.stderr[:200]}")
                print(f"  [FAIL] {label} — stdout: {result.stdout[:200]}")
                continue

            # Parse JSON lines from stdout
            rows = []
            for line in result.stdout.strip().split("\n"):
                if not line.strip():
                    continue
                try:
                    row = eval(line)
                    if isinstance(row, dict):
                        rows.append(row)
                except Exception:
                    pass

            # Write to CSV
            with open(OUTPUT_CSV, "a") as f:
                writer = csv.writer(f)
                for row in rows[:args.reps]:
                    writer.writerow([
                        row.get("object", obj_name),
                        n_samples,
                        row.get("condition", "single_view"),
                        row.get("combined_score", 0),
                        row.get("contact_score", 0),
                        row.get("grasp_type_name", "unknown"),
                        row.get("latency_ms", 0),
                    ])

            elapsed = time.perf_counter() - t_start
            rate = done / elapsed if elapsed > 0 else 0
            eta = (total - done) / rate if rate > 0 else 0
            pct = done / total * 100
            print(f"  [{'OK' if result.returncode == 0 else 'ERR':4s}] {label:30s} {elapsed_batch:.1f}s  ({pct:.0f}% done, ETA {eta:.0f}s)")

            # Clean up
            try:
                os.remove(config_path)
            except OSError:
                pass

    elapsed_total = time.perf_counter() - t_start
    print(f"\nDone in {elapsed_total:.0f}s. Results at {OUTPUT_CSV}")


if __name__ == "__main__":
    main()