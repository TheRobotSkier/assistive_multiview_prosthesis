# Fix: Baseline Subprocess Hang

## Problem

`run.py` uses `multiprocessing.Process` to compute baselines with a different config. This forks the current Python process, which already has the Rust `.so` loaded. The Rust library uses `OnceLock` for config (loaded once, cached forever) and may have internal state (mutexes, thread pools) that doesn't survive `fork()`. This causes the child process to hang/deadlock.

## Root Cause

- `runtime_config.rs:199`: `static RUNTIME_CONFIG: OnceLock<RuntimeConfig> = OnceLock::new()` — config is initialized once on first access
- `ffi_bridge.py:165`: `ctypes.CDLL(so_path)` loads the `.so` at `GraspLibrary()` construction time
- `multiprocessing.Process` uses `fork()` which copies the parent's memory including the loaded `.so` with its `OnceLock` state
- Even if the child sets `GRASP_CONFIG_PATH`, the `OnceLock` is already populated, so the baseline config is never loaded
- More critically, the forked `.so` may deadlock due to internal Rust state

## Fix

Replace `multiprocessing.Process` with `subprocess.Popen` that runs a fresh Python interpreter. This ensures:
1. The Rust `.so` is loaded fresh in the child process
2. `GRASP_CONFIG_PATH` is set before the `.so` is loaded
3. `OnceLock` is empty and loads the correct baseline config

## Implementation

### Changes to `run.py`:

1. Remove `import multiprocessing`
2. Add `import textwrap`
3. Replace `_run_baseline_worker()` + `compute_baseline()` with a new `compute_baseline()` that uses `subprocess.Popen` with an inline Python script

The inline script will:
- Set `GRASP_CONFIG_PATH` to the baseline config
- Import `ffi_bridge`, `object_registry`, `hand_approaches` fresh
- Run the baseline computation
- Write results to a temp JSON file

### Inline script template:

```python
import sys, os, json
sys.path.insert(0, {script_dir})
os.environ["GRASP_CONFIG_PATH"] = {config_path}
import numpy as np
from ffi_bridge import GraspLibrary, make_pose, make_twist, make_request, response_to_dict
from object_registry import load_object
from hand_approaches import get_approach

approach = json.loads({approach_json})
obj = load_object({obj_name})
lib = GraspLibrary()
pose = make_pose(**approach["pose"])
twist = make_twist(**approach["twist"])
cloud = obj["points"]
cameras = [(0.0, 0.0, 0.0)]

runs = []
for i in range({n_reps}):
    req = make_request(pose, twist, cloud, cameras)
    status, resp, msg = lib.compute(req)
    result = response_to_dict(resp)
    result["status"] = status
    result["message"] = msg
    runs.append(result)

successful = [r for r in runs if r["success"]]
if not successful:
    agg = runs[0]
    agg["object"] = {obj_name}
    agg["condition"] = "baseline"
    agg["n_baseline_reps"] = {n_reps}
    agg["n_successful"] = 0
else:
    from collections import Counter
    grasp_counts = Counter(r["grasp_type_name"] for r in successful)
    best_grasp = grasp_counts.most_common(1)[0][0]
    best_grasp_type = next(r["grasp_type"] for r in successful if r["grasp_type_name"] == best_grasp)
    targets = np.array([[r["target_px"], r["target_py"], r["target_pz"]] for r in successful])
    quats = np.array([[r["wrist_qx"], r["wrist_qy"], r["wrist_qz"], r["wrist_qw"]] for r in successful])
    median_target = np.median(targets, axis=0)
    median_quat = quats[np.argmax(np.abs(quats @ np.median(quats, axis=0)))]
    best_score = max(r["combined_score"] for r in successful)
    agg = {{
        "object": {obj_name}, "condition": "baseline",
        "n_baseline_reps": {n_reps}, "n_successful": len(successful),
        "grasp_type": best_grasp_type, "grasp_type_name": best_grasp,
        "combined_score": best_score,
        "target_px": float(median_target[0]), "target_py": float(median_target[1]),
        "target_pz": float(median_target[2]),
        "wrist_qx": float(median_quat[0]), "wrist_qy": float(median_quat[1]),
        "wrist_qz": float(median_quat[2]), "wrist_qw": float(median_quat[3]),
        "success": True,
        "pipeline_time_ms": max(r["pipeline_time_ms"] for r in successful),
    }}

with open({output_path}, "w") as f:
    json.dump(agg, f)
```

## Verification

- [ ] `python run.py --occlusion-only --repetitions 3` completes without hanging
- [ ] Baseline results show different config (higher `pipeline_time_ms` due to more samples/iterations)
- [ ] No `multiprocessing` import remains in `run.py`
