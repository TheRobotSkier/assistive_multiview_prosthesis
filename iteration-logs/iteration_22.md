# Iteration 22 — Replace console_scripts entry points with direct scripts to fix `PackageNotFoundError`

## What I Saw in the Logs

### Latest replay (`replay_20260617_060259`, captured ~04:06 UTC)

| Metric | Value | vs iter21 baseline | Delta |
|--------|-------|--------------------|-------|
| `/gtsam/head_pose` eff Hz | **0.0** | 0.0 | unchanged (still dead) |
| `/gtsam/arm_pose` eff Hz | **0.0** | 0.0 | unchanged (still dead) |
| `/vis/head_arm_pose` | **0.0 Hz** | 0.0 Hz | unchanged |
| `/tf` eff Hz | **22.0** | 22.0 | unchanged |
| GTSAM crashes | **still crashed** | crashed | unchanged |
| SIFT node | **still crashed** | crashed | unchanged |
| `n_crashes` | 19 | 4 | **worse** (more crash events counted) |
| host CPU avg | 27 % | 27 % | unchanged |

### Root cause — both nodes dead from packaging failure

The replay log shows both `gtsam_tracker_node` and `sift_feature_node` crash at startup:

**gtsam_tracker_node** (`host-log-20260617_040301.txt:43-63`):
```
[gtsam_tracker_node-8] importlib.metadata.PackageNotFoundError:
    No package metadata was found for gtsam_tracker
```

The generated console_scripts wrapper (produced by setuptools) calls `load_entry_point()` which calls `importlib.metadata.distribution('gtsam_tracker')`. When the dist-info metadata is not discoverable by `importlib.metadata`, the node crashes before any GTSAM code runs.

**sift_feature_node** (`host-log-20260617_040301.txt:21-42`):
```
[sift_feature_node-10] File ".../sift_feature_node.py", line 25, in <module>
    from gtsam_tracker.umeyama import umeyama
ModuleNotFoundError: No module named 'gtsam_tracker'
```

The installed copy of `sift_feature_node.py` was **stale** — it still had the old module-level `from gtsam_tracker.umeyama import umeyama` that iteration 21 moved to a lazy import. This means the build step between iterations either did not run or did not propagate the file change.

### What iteration 21 attempted

Iteration 21 changed `zip_safe=True` → `zip_safe=False` in both `setup.py` files, hoping to force setuptools to generate a proper dist-info directory. This did not fix the crash. There are two possible explanations:

1. The build container was never rebuilt after the source changes (the install directories for both packages are empty on disk), so the `zip_safe` flag never took effect.
2. Even with `zip_safe=False`, the importlib.metadata lookup can fail under certain colcon install configurations (e.g., `--symlink-install`, non-standard Python path layout).

Either way, the `zip_safe` approach is fragile — it depends on the build system generating metadata in a specific location that `importlib.metadata` can find.

## Decision — Replace console_scripts entry points with direct launcher scripts

The console_scripts entry-point mechanism has proven unreliable in this workspace. The generated wrapper depends on `importlib.metadata.distribution()` which is sensitive to build system details.

**Solution**: Replace `entry_points={'console_scripts': [...]}` with `scripts=['scripts/...']` in both `setup.py` files. The `scripts=` parameter installs standalone executable Python files to the same `lib/<package_name>/` directory that the launch system expects, but **without** going through the `importlib.metadata.distribution()` lookup. The script directly imports the module and calls `main()`.

This is a well-known ROS2 pattern used by many ament_python packages. It is more robust because:

- No dist-info metadata lookup at startup.
- The script is a plain Python file with a shebang, identical to what the generated wrapper does after loading the entry point.
- The launch system (`Node(package=..., executable=...)`) resolves to the same path.

## Changes Made

### 1. `src/gtsam_tracker/setup.py:12` — Replace entry_points with scripts

**Before:**
```python
entry_points={
    "console_scripts": [
        "gtsam_tracker_node = gtsam_tracker.gtsam_tracker_node:main",
    ],
},
```

**After:**
```python
scripts=["scripts/gtsam_tracker_node"],
```

Also reverted `zip_safe` back to `True` (`src/gtsam_tracker/setup.py:18`) since it is no longer relevant.

### 2. `src/gtsam_tracker/scripts/gtsam_tracker_node:1-19` — New file

A simple executable launcher script:
```python
#!/usr/bin/env python3
from gtsam_tracker.gtsam_tracker_node import main
if __name__ == "__main__":
    sys.exit(main())
```

Installed to `lib/gtsam_tracker/gtsam_tracker_node` (same path the console_scripts wrapper used).

### 3. `src/cross_camera_features/setup.py:12` — Replace entry_points with scripts

**Before:**
```python
entry_points={
    "console_scripts": [
        "sift_feature_node = cross_camera_features.sift_feature_node:main",
    ],
},
```

**After:**
```python
scripts=["scripts/sift_feature_node"],
```

Also reverted `zip_safe` to `True` (`src/cross_camera_features/setup.py:18`).

### 4. `src/cross_camera_features/scripts/sift_feature_node:1-20` — New file

Same pattern as the gtsam launcher:
```python
#!/usr/bin/env python3
from cross_camera_features.sift_feature_node import main
if __name__ == "__main__":
    sys.exit(main())
```

Installed to `lib/cross_camera_features/sift_feature_node`.

### 5. What iteration 21 already fixed (still in place)

- `src/cross_camera_features/cross_camera_features/sift_feature_node.py:265` — lazy import of `umeyama` inside `match_and_align()` instead of module-level import.
- `src/cross_camera_features/cross_camera_features/sift_feature_node.py:267` — broadened `except` clause to catch `ImportError` and `np.linalg.LinAlgError` in addition to `ValueError`.

These changes will take effect once the build re-installs the package, since the installed copy was stale.

## Expected Impact

### Goal 1 — Localization precision (GTSAM factor-graph optimization)

1. **GTSAM node starts without error.** The `gtsam_tracker_node` script directly imports the module and calls `main()` — no `importlib.metadata.distribution()` call, no `PackageNotFoundError`. The node should begin subscribing to VIO odometry, building the factor graph, and publishing `/gtsam/head_pose` and `/gtsam/arm_pose` within seconds of launch.

2. **Expected output rates**: ~12–15 Hz average for both pose topics (consistent with earlier working iterations before the build/packaging regression). This will restore the TF broadcasting to ~45 Hz (was at 22 Hz without GTSAM).

3. **ISAM2 periodic reset** (from iteration 19) will finally run. Search the log for "Factor graph reset" — expect 2–3 occurrences over the 170 s replay.

### Goal 2 — TSDF fusion quality (indirect)

1. **Keyframe buffer receives GTSAM-smoothed poses.** Once GTSAM publishes `/gtsam/head_pose` and `/gtsam/arm_pose`, the `keyframe_buffer` node (which subscribes to these) will use smoothed poses for its spatial gating. This reduces spurious keyframe insertion from VIO drift.

2. **TF tree rebuilds.** With GTSAM publishing corrected transforms, the `/tf` topic rate should recover from 22 Hz to ~45+ Hz. The `tf_pipeline_diagnostics` node (which was getting disconnected TF chains) will report fewer errors.

3. **TSDF fusion benefits indirectly.** The `tsdf_fusion` node receives better-aligned pointclouds via the keyframe buffer, leading to fewer outlier points and a denser fused model.

### Specific diagnostic checks for the next iteration

1. **GTSAM startup**: Search the log for `"[gtsam_tracker_node] GtsamTrackerNode ready"` — should appear within 2 seconds of launch. If it does not, check for any new import errors.

2. **GTSAM output rates**: Check `/gtsam/head_pose` first/mid/last Hz in the diagnostics block. Expect first Hz > 0 (ideally > 10 Hz).

3. **SIFT node**: Search for `"[cross_camera_features] SiftFeatureNode ready"` — should appear even if GTSAM is still failing. Verify no more `ModuleNotFoundError` for `gtsam_tracker`.

4. **TF frequency**: `/tf` effective Hz should approach 45+ Hz (versus 22.0 Hz in this replay) once GTSAM is publishing.

5. **TF chain disconnections**: The 17 instances of "TF chain disconnected" should drop to near zero once GTSAM+TF bridge is publishing all frames.

### Risks

1. **Low risk — `scripts` is a standard setuptools feature.** The `setup.cfg` files already configure `install_scripts=$base/lib/<package_name>`, so the script is placed at exactly the same path as the console_scripts wrapper would be. The ROS2 launch system makes no distinction between the two.

2. **Module import path may differ.** With `scripts=`, the script must do `from gtsam_tracker.gtsam_tracker_node import main` which assumes `gtsam_tracker` is on `sys.path`. This is the same assumption the console_scripts wrapper made (it loaded the same module after finding it via entry points). If the module was importable, it will be importable from the script too.

3. **No de-duplication**. If colcon runs `setup.py install` over a previous install, the old entry-point wrapper at the same path will be overwritten by the new script. No stale files remain.
