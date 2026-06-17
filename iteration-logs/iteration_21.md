# Iteration 21 — Fix GTSAM package metadata crash + hard import dependency in SIFT node

## What I Saw in the Logs

### Latest replay results (`replay_20260617_055218` = iteration 19 run)

| Metric | Value | vs iter18 | Delta |
|--------|-------|-----------|-------|
| `/gtsam/head_pose` eff Hz | **0.0** | 11.4/9.3 | **CRASHED** |
| `/gtsam/arm_pose` eff Hz | **0.0** | 11.3/9.3 | **CRASHED** |
| `/vis/head_arm_pose` | **0.0 Hz** | **0.0 Hz** | unchanged (no images) |
| `/tf` eff Hz | 22.0 | 45.63 | **−52 %** (no GTSAM TF) |
| Host CPU avg/max | 27 % / 42 % | 63 % / 92 % | much lower (less processing) |
| TF jump events | 0 | 2 | no GTSAM → less TF activity |
| `odom pose jumped — TF suppressed` | 81 | 81 | unchanged |
| `gtsam_tracker_node` | **crashed at startup** | clean | **REGRESSION** |

### Critical finding — GTSAM node completely dead

The log at lines 21–41 shows `gtsam_tracker_node` crashed before any GTSAM code ran:

```
[gtsam_tracker_node-8] importlib.metadata.PackageNotFoundError:
    No package metadata was found for gtsam_tracker
```

This is a **Python packaging failure** — the generated entry-point script uses `importlib.metadata.distribution('gtsam_tracker')` to find the package dist-info, but the metadata was not properly installed by the colcon build.

### Secondary crash — SIFT node also dead

The `sift_feature_node` crashed at line 63 because it has a **module-level hard import**:

```
from gtsam_tracker.umeyama import umeyama
ModuleNotFoundError: No module named 'gtsam_tracker'
```

This cascading crash means both the GTSAM tracker and the cross-camera visual factor pipeline are dead. The `/vis/head_arm_pose` topic, which was already at 0.0 Hz (no images in the replay bag), would remain 0.0 Hz even if images appeared.

### Comparison with previous iteration

In iteration 18 (replay `replay_20260617_053401`), GTSAM was working:
- `/gtsam/head_pose`: 0.0 → 11.4 → 9.3 Hz (mid→last decline of −18 %, but functional)
- `/gtsam/arm_pose`: 0.0 → 11.3 → 9.3 Hz

The only change between iter18 and iter19 was the source code in `gtsam_tracker_node.py` (adding periodic ISAM2 reset). The `setup.py` and `package.xml` were **unchanged**. This strongly suggests the packaging failure is a **transient build issue** or a **setuptools/Python 3.12 compatibility problem** with `zip_safe=True`.

## Hypothesis

Both `setup.py` files have `zip_safe=True`. With Python 3.12 and setuptools ≥ 69, `zip_safe=True` can cause the dist-info directory to not be properly generated when colcon uses `--symlink-install`. The generated entry-point script calls `importlib.metadata.distribution()` which scans for `gtsam_tracker-0.1.0.dist-info`. If the dist-info is missing (because setuptools treated the package as "zip safe" and didn't generate a directory-based metadata), the lookup raises `PackageNotFoundError`.

Changing `zip_safe=False` ensures setuptools always generates a proper dist-info directory, making the package discoverable by `importlib.metadata`.

Additionally, the SIFT node's **module-level** `from gtsam_tracker.umeyama import umeyama` makes it fragile — if `gtsam_tracker` is temporarily unavailable (partial build, packaging flake), the entire `cross_camera_features` node crashes. Moving this import to be **lazy** (inside the only function that uses `umeyama`) allows the node to start, subscribe to topics, and function up to the point where Umeyama alignment is needed.

## Changes Made

### 1. `src/gtsam_tracker/setup.py:22` — `zip_safe=True` → `zip_safe=False`

Ensures the `gtsam_tracker` package always produces a dist-info directory (never a zip file), so `importlib.metadata.distribution('gtsam_tracker')` succeeds when the entry-point script runs.

### 2. `src/cross_camera_features/setup.py:22` — `zip_safe=True` → `zip_safe=False`

Same fix for the `cross_camera_features` package, which also has a hard runtime dependency on `gtsam_tracker` via the `umeyama` import.

### 3. `src/cross_camera_features/cross_camera_features/sift_feature_node.py:25` — Removed module-level hard import of `umeyama`

**Before (line 25):**
```python
from gtsam_tracker.umeyama import umeyama
```

**After:** The import is removed from module level and placed as a lazy import inside the `match_and_align()` function (line 265).

### 4. `src/cross_camera_features/cross_camera_features/sift_feature_node.py:265` — Added lazy import

Inside the `try` block of `match_and_align()`:
```python
from gtsam_tracker.umeyama import umeyama  # lazy import
```

### 5. `src/cross_camera_features/cross_camera_features/sift_feature_node.py:267` — Broaden exception handling

**Before:** `except ValueError as exc:`
**After:** `except (ValueError, ImportError, np.linalg.LinAlgError) as exc:`

This catches:
- `ValueError` — raised by `umeyama()` for invalid input (fewer than 3 correspondences, shape mismatch)
- `ImportError` — raised by the lazy import if `gtsam_tracker` is not importable (lets the node degrade gracefully instead of crashing)
- `np.linalg.LinAlgError` — raised by SVD in `umeyama()` for singular/degenerate matrices

## Expected Impact

### Goal 1 — Localization precision (GTSAM factor-graph optimization)

1. **`zip_safe=False` fixes the packaging crash.** After the next build, the dist-info metadata for `gtsam_tracker` will be properly generated. The entry-point script (`importlib.metadata.distribution()`) will find the package, and `gtsam_tracker_node` will start.

2. **GTSAM output rates restored.** With the node running, the periodic ISAM2 reset (from iteration 19) can finally be tested. Expected: ~12–15 Hz average versus the 8.7 Hz end-run rate seen before the crash.

3. **SIFT node survives GTSAM failures.** If `gtsam_tracker` is unavailable for any reason, the `sift_feature_node` now starts successfully and only fails alignment (gracefully, via the `except` clause) when Umeyama is invoked.

### Goal 2 — TSDF fusion quality (indirect)

1. **Restored TF broadcasting.** With GTSAM publishing poses again, the TF tree at `/tf` should return to 45+ Hz (was at 22.0 Hz in this run because only the OpenVINS relay was publishing TF without GTSAM corrections).

2. **Better pose interpolation for pointcloud fusion.** When the keyframe_buffer and tsdf_fusion nodes consume GTSAM-smoothed poses instead of raw VIO relay, the pointcloud alignment is tighter, reducing outliers.

### Diagnostic expectations

1. `gtsam_tracker_node` should start without errors (no more `PackageNotFoundError`).
2. `/gtsam/head_pose` and `/gtsam/arm_pose` should show non-zero Hz in the first diagnostic block.
3. `sift_feature_node` should start without errors even if `gtsam_tracker` import fails (and log "umeyama failed: ..." instead of crashing).
4. `/vis/head_arm_pose` will remain 0.0 Hz if images are absent from the replay bag — that's expected and not fixed here.

### Risks

1. **The `zip_safe=False` change is safe.** It adds ~4 KB of dist-info metadata to the install directory and has zero runtime overhead. All ROS2 packages in the ecosystem use `zip_safe=False`.

2. **The lazy import adds ~2 µs overhead** on the first call (Python's import cache makes subsequent calls O(1)). This is negligible compared to the SIFT extraction time (~50–200 ms per frame).

3. **If the packaging crash was transient** (not caused by `zip_safe=True`), the GTSAM node still won't start. In that case, the lazy import fix at least ensures the SIFT node survives independently.

### What to check in the next iteration

1. **GTSAM startup** — Search the log for "GtsamTrackerNode ready" — should appear within 2 seconds of pipeline launch. If it doesn't, check for `PackageNotFoundError` or other import errors.

2. **SIFT node** — Search for "SiftFeatureNode ready" — should appear even if GTSAM is down.

3. **GTSAM output rates** — Check `/gtsam/head_pose` first/mid/last Hz. Expected first Hz > 0 (GTSAM starts publishing within seconds).

4. **Periodic reset** — Search for "Factor graph reset" in the log — expect 2–3 occurrences over 170 s if iteration 19's reset code is working.
