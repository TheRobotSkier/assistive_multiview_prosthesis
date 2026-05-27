#!/usr/bin/env python3
"""Test 1: Software Verification — Tier A orchestrator.

Runs two sub-tests via the Rust .so (no ROS):
  1. Latency Test (Req 2.4, 1.6): Measures pipeline computation time per object.
  2. Occlusion Proxy Test (Req 2.7): Compares single-view vs. multi-view grasp quality.

Results are written to results/ as CSV files. Run plot_results.py afterwards
to generate figures.

Usage:
    python run.py                        # run all tests
    python run.py --latency-only         # only latency test
    python run.py --occlusion-only       # only occlusion test
    python run.py --objects cylinder_upright small_cube  # specific objects
    python run.py --repetitions 20       # more repetitions (default 10)
    python run.py --debug                # enable debug dumps
    python run.py --debug-dumps           # generate SQ debug dumps only (no latency)

Inside Docker:
    docker compose run --rm prosthesis python /prosthesis_ws/tests/test1_software_verification/run.py
"""

import argparse
import csv
import json
import os
import subprocess
import sys
import tempfile
import textwrap
import time

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, "results")
CONFIG_DIR = os.path.join(SCRIPT_DIR, "config")

# Ensure test modules are importable
sys.path.insert(0, SCRIPT_DIR)

from ffi_bridge import (
    GraspLibrary,
    GRASP_COMPUTE_OK,
    GRASP_TYPE_NAMES,
    make_pose,
    make_twist,
    make_request,
    response_to_dict,
)

# Reverse mapping: grasp type name -> ID
GRASP_TYPE_IDS = {v: k for k, v in GRASP_TYPE_NAMES.items()}
from object_registry import load_object, list_objects
from hand_approaches import get_approach
from view_geometry import get_camera_world_positions, get_camera_world_frames
from occlusion import generate_view_cloud


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _ensure_dirs():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    os.makedirs(os.path.join(RESULTS_DIR, "debug_dumps"), exist_ok=True)


def _write_csv(path: str, rows: list[dict], fieldnames: list[str] | None = None):
    """Write a list of dicts to a CSV file."""
    if not rows:
        print(f"  WARNING: No data to write to {path}")
        return
    if fieldnames is None:
        fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"  Wrote {len(rows)} rows to {path}")


# ---------------------------------------------------------------------------
# Baseline computation (runs in a fresh subprocess with different config)
# ---------------------------------------------------------------------------

# Inline script template for the baseline subprocess.
# This runs in a FRESH Python process so that the Rust .so's OnceLock
# picks up GRASP_CONFIG_PATH before any Rust code initializes.
_BASELINE_SCRIPT = textwrap.dedent("""\
    import sys, os, json
    import numpy as np

    # Set config BEFORE importing ffi_bridge (which loads the Rust .so)
    os.environ["GRASP_CONFIG_PATH"] = config_path
    sys.path.insert(0, script_dir)

    from ffi_bridge import (
        GraspLibrary, make_pose, make_twist, make_request, response_to_dict,
    )
    from object_registry import load_object
    from hand_approaches import get_approach

    approach = json.loads(approach_json)
    obj = load_object(obj_name)
    lib = GraspLibrary()

    pose = make_pose(**approach["pose"])
    twist = make_twist(**approach["twist"])
    cloud = obj["points"]
    cameras = [(0.0, 0.0, 0.0)]  # placeholder -- baseline uses full cloud

    runs = []
    for i in range(n_reps):
        req = make_request(pose, twist, cloud, cameras)
        status, resp, msg = lib.compute(req)
        result = response_to_dict(resp)
        result["status"] = status
        result["message"] = msg
        result["rep"] = i
        runs.append(result)

    successful = [r for r in runs if r["success"]]
    if not successful:
        agg = runs[0]
        agg["object"] = obj_name
        agg["condition"] = "baseline"
        agg["n_baseline_reps"] = n_reps
        agg["n_successful"] = 0
        all_scores = [0.0] * n_reps
    else:
        from collections import Counter
        grasp_counts = Counter(r["grasp_type_name"] for r in successful)
        best_grasp = grasp_counts.most_common(1)[0][0]
        best_grasp_type = next(
            r["grasp_type"] for r in successful if r["grasp_type_name"] == best_grasp
        )
        targets = np.array(
            [[r["target_px"], r["target_py"], r["target_pz"]] for r in successful]
        )
        quats = np.array(
            [[r["wrist_qx"], r["wrist_qy"], r["wrist_qz"], r["wrist_qw"]] for r in successful]
        )
        median_target = np.median(targets, axis=0)
        median_quat = quats[np.argmax(np.abs(quats @ np.median(quats, axis=0)))]
        best_score = max(r["combined_score"] for r in successful)
        all_scores = [r.get("combined_score", 0.0) for r in runs]
        agg = {
            "object": obj_name,
            "condition": "baseline",
            "n_baseline_reps": n_reps,
            "n_successful": len(successful),
            "grasp_type": best_grasp_type,
            "grasp_type_name": best_grasp,
            "combined_score": best_score,
            "target_px": float(median_target[0]),
            "target_py": float(median_target[1]),
            "target_pz": float(median_target[2]),
            "wrist_qx": float(median_quat[0]),
            "wrist_qy": float(median_quat[1]),
            "wrist_qz": float(median_quat[2]),
            "wrist_qw": float(median_quat[3]),
            "success": True,
            "pipeline_time_ms": max(r["pipeline_time_ms"] for r in successful),
            "all_scores": all_scores,
        }

    with open(output_path, "w") as f:
        json.dump(agg, f)
""")


def compute_baseline(
    obj_name: str,
    approach: dict,
    results_dir: str,
    timeout: int = 120,
    n_reps: int = 20,
) -> dict | None:
    """Run baseline computation in a fresh subprocess with the high-fidelity config.

    Uses subprocess.Popen (not multiprocessing.Process) to avoid fork()-related
    issues with the Rust .so's OnceLock config singleton.
    """
    config_path = os.path.join(CONFIG_DIR, "grasp_preshaping_baseline.yaml")
    approach_json = json.dumps(approach)
    output_path = os.path.join(results_dir, f"_baseline_{obj_name}.json")

    # Pass variables into the inline script via a namespace dict
    script = _BASELINE_SCRIPT
    namespace = {
        "config_path": config_path,
        "script_dir": SCRIPT_DIR,
        "approach_json": approach_json,
        "obj_name": obj_name,
        "n_reps": n_reps,
        "output_path": output_path,
    }

    # Build the script with variable assignments prepended
    header_lines = [
        f"{k} = {repr(v)}" for k, v in namespace.items()
    ]
    full_script = "\n".join(header_lines) + "\n" + script

    start = time.time()
    try:
        proc = subprocess.Popen(
            [sys.executable, "-c", full_script],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        stdout, stderr = proc.communicate(timeout=timeout)
        elapsed = time.time() - start
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.communicate()
        elapsed = time.time() - start
        print(f"TIMEOUT after {elapsed:.0f}s")
        return None

    if proc.returncode != 0:
        err_msg = stderr.decode("utf-8", errors="replace")[-500:]
        print(f"FAILED (exit {proc.returncode}, {elapsed:.0f}s)")
        if err_msg.strip():
            print(f"    stderr: {err_msg}")
        return None

    if not os.path.isfile(output_path):
        print(f"FAILED (no output file, {elapsed:.0f}s)")
        return None

    with open(output_path) as f:
        result = json.load(f)
    os.remove(output_path)

    result["_elapsed_s"] = elapsed
    return result


# ---------------------------------------------------------------------------
# Test 1a: Latency Test
# ---------------------------------------------------------------------------

def run_latency_test(lib: GraspLibrary, objects: list[str],
                     repetitions: int = 10) -> list[dict]:
    """Measure pipeline latency for each object.

    Uses the full point cloud (no occlusion) so latency reflects algorithmic
    complexity, not view-dependent cloud size.
    """
    print("\n" + "=" * 60)
    print("TEST 1a: LATENCY MEASUREMENT (Req 2.4, 1.6)")
    print("=" * 60)

    rows = []

    for obj_name in objects:
        print(f"\n  Object: {obj_name}")
        try:
            obj = load_object(obj_name)
            approach = get_approach(obj_name)
        except (KeyError, FileNotFoundError) as e:
            print(f"    SKIP: {e}")
            continue

        pose = make_pose(**approach["pose"])
        twist = make_twist(**approach["twist"])
        cloud = obj["points"]

        # Use both cameras for multi-view condition
        cameras = get_camera_world_positions(approach["pose"])

        for rep in range(repetitions):
            req = make_request(pose, twist, cloud, cameras)
            status, resp, msg = lib.compute(req)

            if status != GRASP_COMPUTE_OK:
                print(f"    Rep {rep}: FAILED (status={status}, msg={msg})")
                continue

            row = response_to_dict(resp)
            row["object"] = obj_name
            row["repetition"] = rep
            row["n_cloud_points"] = len(cloud)
            row["expected_grasp"] = obj["expected_grasp"]
            rows.append(row)

            if rep == 0:
                print(f"    Rep 0: {resp.pipeline_time_ms} ms, "
                      f"grasp={GRASP_TYPE_NAMES.get(resp.grasp_type, '?')}, "
                      f"score={resp.combined_score:.3f}")

        if rows:
            obj_rows = [r for r in rows if r["object"] == obj_name]
            times = [r["pipeline_time_ms"] for r in obj_rows]
            print(f"    Summary: mean={np.mean(times):.1f} ms, "
                  f"std={np.std(times):.1f} ms, "
                  f"min={np.min(times)}, max={np.max(times)}")

    return rows


# ---------------------------------------------------------------------------
# Test 1b: Occlusion Proxy Test
# ---------------------------------------------------------------------------

def run_occlusion_test(lib: GraspLibrary, objects: list[str],
                       repetitions: int = 10,
                       compute_baselines: bool = True,
                       baseline_timeout: int = 120) -> list[dict]:
    """Compare single-view vs. multi-view grasp quality.

    For each object, three conditions:
      - baseline: full cloud, high-fidelity config (ground truth)
      - single_view: only head camera frustum, production config
      - multi_view: both camera frustums, production config
    """
    print("\n" + "=" * 60)
    print("TEST 1b: OCCLUSION PROXY TEST (Req 2.7)")
    print("=" * 60)

    rows = []

    # Step 1: Compute baselines (ground truth) — each in a subprocess
    baselines = {}
    if compute_baselines:
        print("\n  Computing high-fidelity baselines...")
        for obj_name in objects:
            print(f"    Baseline: {obj_name}...", end="", flush=True)
            try:
                approach = get_approach(obj_name)
            except KeyError:
                print(f" SKIP (no approach)")
                continue

            result = compute_baseline(
                obj_name, approach, RESULTS_DIR,
                timeout=baseline_timeout,
            )
            if result and result.get("success"):
                baselines[obj_name] = result
                elapsed = result.get("_elapsed_s", 0)
                print(f" OK ({result['pipeline_time_ms']} ms, "
                      f"grasp={result['grasp_type_name']}, "
                      f"wall={elapsed:.1f}s)")
            else:
                msg = result.get("message", "unknown") if result else "timeout"
                print(f" FAILED ({msg})")
    else:
        print("\n  Skipping baselines (loading from previous run if available)")
        # Try loading from previous CSV
        baseline_csv = os.path.join(RESULTS_DIR, "baseline_results.csv")
        if os.path.isfile(baseline_csv):
            with open(baseline_csv) as f:
                reader = csv.DictReader(f)
                for row in reader:
                    baselines[row["object"]] = row
            print(f"    Loaded {len(baselines)} baselines from {baseline_csv}")

    # Step 2: Single-view and multi-view tests
    print("\n  Running single-view and multi-view tests...")
    for obj_name in objects:
        print(f"\n  Object: {obj_name}")
        try:
            obj = load_object(obj_name)
            approach = get_approach(obj_name)
        except (KeyError, FileNotFoundError) as e:
            print(f"    SKIP: {e}")
            continue

        pose = make_pose(**approach["pose"])
        twist = make_twist(**approach["twist"])
        full_cloud = obj["points"]

        # Get camera frames for occlusion
        cam_frames = get_camera_world_frames(approach["pose"])
        head_frame = cam_frames[0]
        wrist_frame = cam_frames[1]

        # Generate occluded point clouds
        head_cloud = generate_view_cloud(full_cloud, head_frame, use_depth_buffer=True)
        wrist_cloud = generate_view_cloud(full_cloud, wrist_frame, use_depth_buffer=True)

        # Multi-view: concatenate both camera views with fine voxel dedup.
        # This simulates the point cloud fusion in the real multi-camera system.
        # Fine dedup (0.1mm) removes exact duplicates without collapsing distinct
        # surface points that are close together.
        if len(wrist_cloud) > 0 and len(head_cloud) > 0:
            merged = np.vstack([head_cloud, wrist_cloud])
            voxel_keys = np.floor(merged / 0.0001).astype(np.int64)
            _, unique_idx = np.unique(voxel_keys, axis=0, return_index=True)
            multi_cloud = merged[unique_idx]
        elif len(head_cloud) > 0:
            multi_cloud = head_cloud
        else:
            multi_cloud = wrist_cloud

        # Camera positions for the FFI
        head_cam_pos = tuple(head_frame["position"].tolist())
        wrist_cam_pos = tuple(wrist_frame["position"].tolist())

        # Single-view uses HEAD camera only (the primary viewpoint in the real
        # system — the user's head-mounted camera provides the default view).
        # Multi-view combines both cameras (wrist adds close-range detail that
        # the head camera cannot see due to its oblique overhead angle).
        conditions = [
            ("single_view", head_cloud, [head_cam_pos]),
            ("multi_view", multi_cloud, [head_cam_pos, wrist_cam_pos]),
        ]

        for condition_name, cloud, cameras in conditions:
            if len(cloud) == 0:
                print(f"    {condition_name}: empty cloud, skipping")
                continue

            for rep in range(repetitions):
                req = make_request(pose, twist, cloud, cameras)
                status, resp, msg = lib.compute(req)

                if status != GRASP_COMPUTE_OK:
                    print(f"    {condition_name} rep {rep}: FAILED ({msg})")
                    continue

                row = response_to_dict(resp)
                row["object"] = obj_name
                row["condition"] = condition_name
                row["repetition"] = rep
                row["n_cloud_points"] = len(cloud)
                row["n_full_points"] = len(full_cloud)
                row["expected_grasp"] = obj["expected_grasp"]

                # Primary correctness: match against baseline (high-fidelity reference)
                if obj_name in baselines:
                    bl = baselines[obj_name]
                    row["baseline_grasp_type"] = bl.get("grasp_type", -1)
                    row["baseline_grasp_type_name"] = bl.get("grasp_type_name", "unknown")
                    row["grasp_type_match"] = (
                        resp.grasp_type == int(bl.get("grasp_type", -1))
                    )

                    # Position error (mm)
                    dx = resp.target_px - float(bl.get("target_px", 0))
                    dy = resp.target_py - float(bl.get("target_py", 0))
                    dz = resp.target_pz - float(bl.get("target_pz", 0))
                    row["position_error_mm"] = np.sqrt(dx*dx + dy*dy + dz*dz) * 1000

                    # Orientation error (degrees)
                    bl_q = np.array([
                        float(bl.get("wrist_qx", 0)),
                        float(bl.get("wrist_qy", 0)),
                        float(bl.get("wrist_qz", 0)),
                        float(bl.get("wrist_qw", 1)),
                    ])
                    resp_q = np.array([
                        resp.wrist_qx, resp.wrist_qy, resp.wrist_qz, resp.wrist_qw
                    ])
                    bl_q /= np.linalg.norm(bl_q) + 1e-8
                    resp_q /= np.linalg.norm(resp_q) + 1e-8
                    dot = np.clip(np.abs(np.dot(bl_q, resp_q)), -1, 1)
                    row["orientation_error_deg"] = np.degrees(2 * np.arccos(dot))

                    # Score difference
                    row["score_delta"] = (
                        resp.combined_score - float(bl.get("combined_score", 0))
                    )

                # Also compare against expected grasp (secondary)
                expected_type = GRASP_TYPE_IDS.get(obj["expected_grasp"], -1)
                row["grasp_correct"] = (resp.grasp_type == expected_type)

                rows.append(row)

                if rep == 0:
                    n_vis = len(cloud)
                    pct = n_vis / len(full_cloud) * 100 if len(full_cloud) > 0 else 0
                    print(f"    {condition_name}: {n_vis} pts ({pct:.0f}%), "
                          f"{resp.pipeline_time_ms} ms, "
                          f"grasp={GRASP_TYPE_NAMES.get(resp.grasp_type, '?')}, "
                          f"score={resp.combined_score:.3f}")

    # Save baselines to CSV for benchmark comparison
    if baselines:
        baseline_rows = []
        baseline_all_scores = []
        for obj_name, bl in sorted(baselines.items()):
            row = {"object": obj_name, "condition": "baseline"}
            for k, v in bl.items():
                if k.startswith("_"):
                    continue
                row[k] = v
            baseline_rows.append(row)
            # Expand all_scores into individual rows for violin plot
            all_scores = bl.get("all_scores", [])
            for i, score in enumerate(all_scores):
                baseline_all_scores.append({
                    "object": obj_name,
                    "condition": "baseline",
                    "rep": i,
                    "combined_score": score,
                })
        _write_csv(
            os.path.join(RESULTS_DIR, "baseline_results.csv"),
            baseline_rows,
        )
        if baseline_all_scores:
            _write_csv(
                os.path.join(RESULTS_DIR, "baseline_all_scores.csv"),
                baseline_all_scores,
            )

    return rows


# ---------------------------------------------------------------------------
# Intent Precision Summary
# ---------------------------------------------------------------------------

def compute_intent_precision(rows: list[dict]) -> list[dict]:
    """Compute per-object intent precision metrics for the occlusion test.

    Primary metric (Req 2.7): grasp type correctness — does the predicted
    grasp type match the baseline consensus? This is the "fully correct"
    metric for intent precision.

    Secondary metrics (logged, not scored):
      - Wrist rotation error: mean angular distance from baseline (degrees)
      - Position error: mean Euclidean distance from baseline target (mm)
    """
    summary = []

    # Group by (object, condition)
    from collections import defaultdict
    groups = defaultdict(list)
    for row in rows:
        if "condition" not in row or row["condition"] in ("baseline",):
            continue
        key = (row["object"], row["condition"])
        groups[key].append(row)

    # Get all objects that have both conditions
    objects_with_both = set()
    condition_objects = defaultdict(set)
    for (obj, cond) in groups:
        condition_objects[cond].add(obj)
    if "single_view" in condition_objects and "multi_view" in condition_objects:
        objects_with_both = condition_objects["single_view"] & condition_objects["multi_view"]

    for obj_name in sorted(objects_with_both):
        for cond in ["single_view", "multi_view"]:
            key = (obj_name, cond)
            if key not in groups:
                continue
            cond_rows = groups[key]

            n_total = len(cond_rows)
            n_success = sum(1 for r in cond_rows if r.get("success", False))
            n_grasp_match = sum(1 for r in cond_rows if r.get("grasp_type_match", False))
            n_grasp_correct = sum(1 for r in cond_rows if r.get("grasp_correct", False))
            n_orient_ok = sum(
                1 for r in cond_rows if r.get("orientation_error_deg", 999) < 45
            )
            n_pos_ok = sum(
                1 for r in cond_rows if r.get("position_error_mm", 999) < 30
            )

            scores = [r["combined_score"] for r in cond_rows]
            mean_score = np.mean(scores)
            max_score = np.max(scores)
            mean_pos_err = np.mean([r.get("position_error_mm", float("nan")) for r in cond_rows])
            mean_orient_err = np.mean([r.get("orientation_error_deg", float("nan")) for r in cond_rows])
            median_orient_err = np.median([r.get("orientation_error_deg", float("nan")) for r in cond_rows])
            mean_time = np.mean([r["pipeline_time_ms"] for r in cond_rows])

            # Collect per-repetition wrist errors for CDF analysis
            wrist_errors = [r.get("orientation_error_deg", float("nan")) for r in cond_rows]

            summary.append({
                "object": obj_name,
                "condition": cond,
                "n_repetitions": n_total,
                "n_successful": n_success,
                "success_rate_pct": n_success / n_total * 100 if n_total > 0 else 0,
                # Primary metric: grasp type match vs baseline (high-fidelity reference)
                "grasp_accuracy_pct": n_grasp_match / n_total * 100 if n_total > 0 else 0,
                # Secondary: match vs expected grasp label
                "grasp_correct_pct": n_grasp_correct / n_total * 100 if n_total > 0 else 0,
                "orientation_accuracy_pct": n_orient_ok / n_total * 100 if n_total > 0 else 0,
                "position_accuracy_pct": n_pos_ok / n_total * 100 if n_total > 0 else 0,
                "mean_score": mean_score,
                "max_score": max_score,
                "mean_position_error_mm": mean_pos_err,
                "mean_orientation_error_deg": mean_orient_err,
                "median_orientation_error_deg": median_orient_err,
                "mean_time_ms": mean_time,
                # Per-repetition wrist errors (JSON for CSV storage)
                "wrist_errors_json": json.dumps(wrist_errors),
            })

    # Compute Delta (multi_view - single_view) per object
    delta_rows = []
    for obj_name in sorted(objects_with_both):
        sv = [s for s in summary if s["object"] == obj_name and s["condition"] == "single_view"]
        mv = [s for s in summary if s["object"] == obj_name and s["condition"] == "multi_view"]
        if sv and mv:
            sv, mv = sv[0], mv[0]
            delta_rows.append({
                "object": obj_name,
                # Primary delta: grasp accuracy vs baseline
                "delta_grasp_accuracy_pct": mv["grasp_accuracy_pct"] - sv["grasp_accuracy_pct"],
                # Secondary delta: grasp correctness vs expected
                "delta_grasp_correct_pct": mv["grasp_correct_pct"] - sv["grasp_correct_pct"],
                # Legacy field kept for backward compat
                "delta_fully_correct_pct": mv["grasp_accuracy_pct"] - sv["grasp_accuracy_pct"],
                "delta_success_rate_pct": mv["success_rate_pct"] - sv["success_rate_pct"],
                "delta_position_error_mm": sv["mean_position_error_mm"] - mv["mean_position_error_mm"],
                "delta_orientation_error_deg": sv["mean_orientation_error_deg"] - mv["mean_orientation_error_deg"],
                "delta_score": mv["mean_score"] - sv["mean_score"],
                "delta_max_score": mv["max_score"] - sv["max_score"],
                "single_view_grasp_correct_pct": sv["grasp_correct_pct"],
                "multi_view_grasp_correct_pct": mv["grasp_correct_pct"],
                "single_view_grasp_accuracy_pct": sv["grasp_accuracy_pct"],
                "multi_view_grasp_accuracy_pct": mv["grasp_accuracy_pct"],
                "single_view_success_rate_pct": sv["success_rate_pct"],
                "multi_view_success_rate_pct": mv["success_rate_pct"],
            })

    return summary, delta_rows


# ---------------------------------------------------------------------------
# Score-vs-Samples Sweep
# ---------------------------------------------------------------------------

def run_score_sweep(lib, objects, args):
    """Sweep prediction_samples across multiple values and record scores + latency."""
    import yaml

    sample_counts = [int(x.strip()) for x in args.sweep_counts.split(",")]
    n_reps = args.sweep_reps
    prod_config = os.path.join(CONFIG_DIR, "grasp_preshaping.yaml")

    with open(prod_config) as f:
        base_cfg = yaml.safe_load(f)

    sweep_rows = []

    for n_samples in sample_counts:
        print(f"\n--- Sample count: {n_samples} ---")

        # Create a temporary config with modified prediction_samples
        cfg = dict(base_cfg)
        cfg["prediction_samples"] = n_samples
        sweep_config = os.path.join(RESULTS_DIR, f"_sweep_config_{n_samples}.yaml")
        with open(sweep_config, "w") as f:
            yaml.dump(cfg, f)
        os.environ["GRASP_CONFIG_PATH"] = sweep_config

        # Force Rust library to reload config from the new path
        from ffi_bridge import reload_config
        reload_config()

        for obj_name in objects:
            try:
                obj = load_object(obj_name)
                approach = get_approach(obj_name)
            except Exception as e:
                print(f"  {obj_name}: SKIP ({e})")
                continue

            full_cloud = obj["points"]
            cam_frames = get_camera_world_frames(approach["pose"])

            # Generate view clouds
            head_cloud = generate_view_cloud(full_cloud, cam_frames[0], use_depth_buffer=True)
            wrist_cloud = generate_view_cloud(full_cloud, cam_frames[1], use_depth_buffer=True)

            # Multi-view merge (concatenation + dedup)
            if len(wrist_cloud) > 0 and len(head_cloud) > 0:
                merged = np.vstack([head_cloud, wrist_cloud])
                vk = np.floor(merged / 0.0001).astype(np.int64)
                _, ui = np.unique(vk, axis=0, return_index=True)
                multi_cloud = merged[ui]
            elif len(head_cloud) > 0:
                multi_cloud = head_cloud
            else:
                multi_cloud = wrist_cloud

            pose = approach["pose"]
            hand_pose = make_pose(pose["px"], pose["py"], pose["pz"],
                                   pose["qx"], pose["qy"], pose["qz"], pose["qw"])
            twist = make_twist(**approach["twist"])

            conditions = [
                ("single_view", head_cloud, [cam_frames[0]["position"]]),
                ("multi_view", multi_cloud, [cam_frames[0]["position"], cam_frames[1]["position"]]),
            ]

            for cond_name, cloud, cam_positions in conditions:
                if len(cloud) == 0:
                    continue

                for rep in range(n_reps):
                    req = make_request(hand_pose, twist, cloud, cam_positions)
                    t0 = time.perf_counter()
                    status, resp, msg = lib.compute(req)
                    elapsed_ms = (time.perf_counter() - t0) * 1000

                    r = response_to_dict(resp)
                    sweep_rows.append({
                        "object": obj_name,
                        "condition": cond_name,
                        "prediction_samples": n_samples,
                        "repetition": rep,
                        "combined_score": r.get("combined_score", 0.0),
                        "contact_score": r.get("contact_score", 0.0),
                        "grasp_type_name": r.get("grasp_type_name", "unknown"),
                        "latency_ms": elapsed_ms,
                    })

            print(f"  {obj_name}: done ({n_reps * 2} trials)")

        # Clean up temp config
        if os.path.isfile(sweep_config):
            os.remove(sweep_config)

    # Restore original config
    os.environ["GRASP_CONFIG_PATH"] = prod_config

    # Save sweep results
    _write_csv(os.path.join(RESULTS_DIR, "score_sweep_results.csv"), sweep_rows)
    print(f"\nSweep complete. {len(sweep_rows)} rows saved to results/score_sweep_results.csv")


# ---------------------------------------------------------------------------
# Debug dump generation (separate from latency measurement)
# ---------------------------------------------------------------------------

def generate_debug_dumps(lib: GraspLibrary, objects: list[str]):
    """Generate one debug dump per object for SQ estimation overlays.

    This runs the pipeline once per object with debug_visualization enabled,
    producing .npz files in results/debug_dumps/ that contain sq_params.
    Latency is NOT measured during this pass — debug dumps add significant
    overhead and would skew timing results.

    Should be run AFTER the normal test pass so that the production config
    is already in place. Uses a temporary config with debug enabled.
    """
    import yaml

    print("\n" + "=" * 60)
    print("GENERATING DEBUG DUMPS (SQ estimation data)")
    print("=" * 60)
    print("  Note: Latency is NOT measured in this pass.\n")

    prod_config = os.path.join(CONFIG_DIR, "grasp_preshaping.yaml")
    with open(prod_config) as f:
        cfg = yaml.safe_load(f)
    cfg["debug_visualization"] = True
    cfg["debug_output_dir"] = os.path.join(RESULTS_DIR, "debug_dumps")
    debug_config = os.path.join(RESULTS_DIR, "_debug_dumps_config.yaml")
    with open(debug_config, "w") as f:
        yaml.dump(cfg, f)
    os.environ["GRASP_CONFIG_PATH"] = debug_config

    # Force Rust library to reload config
    from ffi_bridge import reload_config
    reload_config()

    os.makedirs(os.path.join(RESULTS_DIR, "debug_dumps"), exist_ok=True)

    for obj_name in objects:
        print(f"  Generating debug dump for {obj_name}...", end="", flush=True)
        try:
            obj = load_object(obj_name)
            approach = get_approach(obj_name)
        except (KeyError, FileNotFoundError) as e:
            print(f" SKIP ({e})")
            continue

        pose = make_pose(**approach["pose"])
        twist = make_twist(**approach["twist"])
        cloud = obj["points"]
        cameras = get_camera_world_positions(approach["pose"])

        # Run one computation to generate the debug dump
        req = make_request(pose, twist, cloud, cameras)
        status, resp, msg = lib.compute(req)

        if status == GRASP_COMPUTE_OK:
            print(f" OK (score={resp.combined_score:.3f})")
        else:
            print(f" FAILED ({msg})")

    # Restore production config
    os.environ["GRASP_CONFIG_PATH"] = prod_config
    reload_config()

    # Clean up temp config
    if os.path.isfile(debug_config):
        os.remove(debug_config)

    print("\n  Debug dumps generated in: results/debug_dumps/")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Test 1: Software Verification (Tier A — ctypes direct)"
    )
    parser.add_argument("--latency-only", action="store_true",
                        help="Only run the latency test")
    parser.add_argument("--occlusion-only", action="store_true",
                        help="Only run the occlusion proxy test")
    parser.add_argument("--no-baseline", action="store_true",
                        help="Skip baseline computation (use previous results)")
    parser.add_argument("--objects", nargs="+", default=None,
                        help="Specific objects to test (default: all)")
    parser.add_argument("--repetitions", type=int, default=100,
                        help="Number of repetitions per condition (default: 100)")
    parser.add_argument("--debug", action="store_true",
                        help="Enable debug visualization dumps (adds latency overhead)")
    parser.add_argument("--debug-dumps", action="store_true",
                        help="Generate SQ debug dumps only (no latency measurement). "
                             "Run this after the normal test pass to produce SQ "
                             "estimation data for the object gallery figure.")
    parser.add_argument("--baseline-timeout", type=int, default=120,
                        help="Per-object baseline timeout in seconds (default: 120)")
    parser.add_argument("--sweep-samples", action="store_true",
                        help="Run score-vs-samples sweep instead of normal test")
    parser.add_argument("--sweep-counts", type=str, default="1000,2000,5000,10000,20000,50000,100000",
                        help="Comma-separated sample counts for sweep (default: 1K-100K)")
    parser.add_argument("--sweep-reps", type=int, default=30,
                        help="Repetitions per sample count for sweep (default: 30)")
    args = parser.parse_args()

    _ensure_dirs()

    # Set config path for production runs
    prod_config = os.path.join(CONFIG_DIR, "grasp_preshaping.yaml")
    os.environ["GRASP_CONFIG_PATH"] = prod_config

    if args.debug:
        # Patch the config to enable debug
        import yaml
        with open(prod_config) as f:
            cfg = yaml.safe_load(f)
        cfg["debug_visualization"] = True
        debug_config = os.path.join(RESULTS_DIR, "_debug_config.yaml")
        with open(debug_config, "w") as f:
            yaml.dump(cfg, f)
        os.environ["GRASP_CONFIG_PATH"] = debug_config

    # Discover objects
    available = list_objects()
    if args.objects:
        objects = [o for o in args.objects if o in available]
        missing = set(args.objects) - set(objects)
        if missing:
            print(f"WARNING: Objects not found: {missing}")
    else:
        objects = available

    if not objects:
        print("ERROR: No objects available. Run generate_objects.py first.")
        print("  python generate_objects.py")
        if not args.no_baseline:
            print("  python generate_objects.py --fallbacks  # for YCB approximations")
        sys.exit(1)

    print(f"Test objects ({len(objects)}): {objects}")
    print(f"Repetitions: {args.repetitions}")

    # Load library
    print("\nLoading grasp preshaping library...")
    lib = GraspLibrary()
    print(f"  API version: {lib.api_version}")
    print(f"  Library: {lib.so_path}")

    if args.sweep_samples:
        run_score_sweep(lib, objects, args)
        return

    if args.debug_dumps:
        generate_debug_dumps(lib, objects)
        print("\nDone. Debug dumps in:", os.path.join(RESULTS_DIR, "debug_dumps"))
        return

    run_latency = not args.occlusion_only
    run_occlusion = not args.latency_only

    # --- Latency Test ---
    if run_latency:
        latency_rows = run_latency_test(lib, objects, args.repetitions)
        _write_csv(
            os.path.join(RESULTS_DIR, "latency_results.csv"),
            latency_rows,
        )

    # --- Occlusion Proxy Test ---
    if run_occlusion:
        occlusion_rows = run_occlusion_test(
            lib, objects, args.repetitions,
            compute_baselines=not args.no_baseline,
            baseline_timeout=args.baseline_timeout,
        )
        _write_csv(
            os.path.join(RESULTS_DIR, "occlusion_results.csv"),
            occlusion_rows,
        )

        # Compute intent precision summary
        if occlusion_rows:
            summary, delta = compute_intent_precision(occlusion_rows)
            _write_csv(
                os.path.join(RESULTS_DIR, "intent_precision_summary.csv"),
                summary,
            )
            if delta:
                _write_csv(
                    os.path.join(RESULTS_DIR, "intent_precision_delta.csv"),
                    delta,
                )

                # Print summary
                print("\n" + "=" * 60)
                print("INTENT PRECISION DELTA (multi - single)")
                print("=" * 60)
                for d in delta:
                    print(f"  {d['object']:25s}: "
                          f"\u0394_accuracy={d['delta_grasp_accuracy_pct']:+.1f}%, "
                          f"\u0394_score={d['delta_score']:+.3f}, "
                          f"\u0394_success={d['delta_success_rate_pct']:+.1f}%")

                # Overall metrics
                mean_delta_accuracy = np.mean([d["delta_grasp_accuracy_pct"] for d in delta])
                mean_delta_score = np.mean([d["delta_score"] for d in delta])
                mean_delta_success = np.mean([d["delta_success_rate_pct"] for d in delta])
                print(f"\n  Mean \u0394 (grasp accuracy vs baseline): {mean_delta_accuracy:+.1f}%")
                print(f"  Mean \u0394 (score):         {mean_delta_score:+.4f}")
                print(f"  Mean \u0394 (success rate):  {mean_delta_success:+.1f}%")
                print(f"  MAR threshold: > 0%  \u2192  {'PASS' if mean_delta_accuracy > 0 else 'FAIL'}")
                print(f"  IDE threshold: \u2265 10%  \u2192  {'PASS' if mean_delta_accuracy >= 10 else 'FAIL'}")
    # --- Latency summary ---
    if run_latency and latency_rows:
        print("\n" + "=" * 60)
        print("LATENCY SUMMARY (Req 2.4)")
        print("=" * 60)
        for obj_name in objects:
            obj_times = [r["pipeline_time_ms"] for r in latency_rows if r["object"] == obj_name]
            if not obj_times:
                continue
            arr = np.array(obj_times)
            p95 = np.percentile(arr, 95)
            p99 = np.percentile(arr, 99)
            print(f"  {obj_name:25s}: mean={arr.mean():.1f}, "
                  f"P95={p95:.1f}, P99={p99:.1f} ms "
                  f"({'PASS' if p95 <= 400 else 'FAIL'} MAR)")

    print("\nDone. Results in:", RESULTS_DIR)
    print("Run plot_results.py to generate figures.")


if __name__ == "__main__":
    main()
