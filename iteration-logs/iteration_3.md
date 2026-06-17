# Iteration 3 — 2026-06-17

## Goal

Improve GTSAM localization precision by:
1. Fixing a bug where the kinematic range factor's `sigma` parameter (rotation noise) was silently ignored, rendering the configurable `kinematic_range_sigma_m` ineffective.
2. Adding the kinematic range factor on **every graph update** (instead of every 10th) so cross-chain coupling is continuous — critical when visual factors are unavailable.
3. Adding diagnostic logging in `cross_camera_features` so silent failures are traceable.

## What was investigated

### Replay results (replay_20260617_021726 — after iteration 2)

| Topic | Msgs | Eff Hz | Nom Hz | Health |
|---|---|---|---|---|
| `/gtsam/arm_pose` | 2011 | 11.9 | 15 | OK |
| `/gtsam/head_pose` | 2011 | 11.9 | 15 | OK |
| `/vis/head_arm_pose` | **1** | **0.006** | **30** | **STARVED** |

Key observation: the fix from iteration 2 (sync_slop 50ms → 200ms) had **zero effect** — `/vis/head_arm_pose` still has exactly 1 message.

### Root cause: golden bag has very sparse images

Inspected the golden bag metadata (`data/bags/golden_replay/metadata.yaml`):

| Topic | Messages in 165s | Rate |
|---|---|---|
| `/jetson/head/image` | **20** | 0.12 Hz |
| `/jetson/arm/image` | **11** | 0.07 Hz |
| `/jetson/head/odom` | 18844 | 114 Hz |
| `/jetson/arm/odom` | 18804 | 114 Hz |
| `/jetson/head/points` | 59 | 0.36 Hz |
| `/jetson/arm/points` | 82 | 0.50 Hz |

The golden bag only contains **20 head images and 11 arm images** over 165 seconds. The `ApproximateTimeSynchronizer` needs both head + arm images within `sync_slop_s` (200ms) — with such sparse data, paired arrivals are extremely unlikely. This explains why iteration 2's sync_slop fix was ineffective.

**Implication**: Since visual factors cannot be produced from this golden bag, cross-chain coupling relies entirely on the **kinematic range factor**. Any weakness in the range factor directly hurts localization precision.

### Bug found: `sigma` parameter silently ignored in `add_range_factor`

Location: `src/gtsam_tracker/gtsam_tracker/factor_graph.py:285-287`

The `add_range_factor` method accepts a `sigma` parameter documented as "Rotation noise sigma in radians" but the noise model array was hardcoded as:

```python
np.array([float(max_distance), float(max_distance), float(max_distance),
          10.0, 10.0, 10.0])          # ← sigma parameter IGNORED
```

The `sigma` parameter (which receives `kinematic_range_sigma_m = 0.05` from config) was discarded in favor of the hardcoded `10.0` rad (~570°) — effectively making the rotation component of the range factor **completely unconstrained**.

This is the second noise-model bug in this function (iteration 1 fixed the trans/rot ordering swap). Together they meant the range factor was simultaneously:
- Too loose in translation (after iteration 1: sigma = max_distance = 1.0m — moderate)
- **Inoperative in rotation** (sigma = 10.0 rad — essentially no constraint)

Since visual factors are not available due to sparse golden bag images, this left the two GTSAM pose chains (head and arm) with **no effective cross-chain coupling** beyond odometry's own drift.

### Second finding: `kinematic_check_interval: 10` too sparse

Location: `config/prosthesis_config.yaml:488`, `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:65`

The range factor was only added every 10th graph update (~1.2 Hz). With visual factors unavailable, each update should add a range factor to maintain continuous cross-chain coupling.

### Third finding: silent returns in SIFT node

Location: `src/cross_camera_features/cross_camera_features/sift_feature_node.py:598-601`

The `_on_synced_images` callback returned silently when clouds/K/poses weren't cached yet — no log message, no diagnostic. This makes debugging extremely difficult.

## Changes made

### 1. `src/gtsam_tracker/gtsam_tracker/factor_graph.py:277-287`

**Fix**: Use the `sigma` parameter for rotation noise instead of hardcoded `10.0`:

```python
# BEFORE (sigma parameter ignored):
noise = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([float(max_distance), float(max_distance), float(max_distance),
              10.0, 10.0, 10.0]))

# AFTER (sigma used for rotation):
noise = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([float(max_distance), float(max_distance), float(max_distance),
              float(sigma), float(sigma), float(sigma)]))
```

Updated comment to document that rotation sigma now comes from the parameter (default 0.05 rad).

### 2. `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:474-479`

**Change**: Add the kinematic range factor on every graph update when new data is available, instead of every 10th update. The range factor call was moved to right after the data-availability check (line 470), ensuring it only fires when there's new odometry.

### 3. `config/prosthesis_config.yaml:488`

**Change**: Set `kinematic_check_interval: 1` (was `10`) for consistency with the unconditional range factor logic.

### 4. `src/cross_camera_features/cross_camera_features/sift_feature_node.py:598-614`

**Change**: Replace the silent return in `_on_synced_images` with explicit per-item "missing" logging at `DEBUG` level with 5-second throttle:

```python
# BEFORE:
if (self._head_cloud is None or ...):
    return

# AFTER:
missing = []
if self._head_cloud is None: missing.append("head_cloud")
...
if missing:
    self.get_logger().debug(
        f"Synced images arrived but missing: {', '.join(missing)}",
        throttle_duration_sec=5.0)
    return
```

### 5. `src/cross_camera_features/cross_camera_features/sift_feature_node.py:651-655`

**Change**: Add debug-level logging for every `match_and_align` result that doesn't meet the publishing threshold, so the run log shows why alignments are skipped (too few matches, depth lookup failures, etc.).

## Expected impact

**Localization precision (goal 1):**

- The kinematic range factor now uses `sigma=0.05` rad for rotation, providing a meaningful orientation constraint (std dev ≈ 2.9°) between head and arm chains.
- With the range factor added on **every graph update** (~12 Hz), the cross-chain coupling is continuous rather than sparse (~1.2 Hz previously).
- Together these improve the GTSAM smoother's ability to keep the head and arm trajectories consistent, reducing relative drift.
- In replay metrics: better odom→gtsam agreement visible in lower ATE/RPE between chains.

**TSDF fusion (goal 2, indirect):**

- Better head-arm relative poses from the range-constrained GTSAM smoother mean the poses fed to TSDF integration are more consistent.
- This reduces pointcloud misalignment artefacts in the fused TSDF volume.

**Diagnostics:**

- The new debug-level logging in `sift_feature_node.py` will reveal why the SIFT node isn't publishing when images are available — whether it's missing auxiliary data, too few SIFT matches, or depth lookup failures.

## What to check in the next iteration

1. Run the test suite — the pure-logic tests in `test_factor_graph` should still pass (the fix changes numeric values but not the factor structure).
2. Examine replay `bag_analysis.txt` for `/gtsam/head_pose` and `/gtsam/arm_pose` metrics — expect lower trajectory error with the stronger range constraint.
3. Check `/vis/head_arm_pose` count — still expected to be very low (sparse golden bag images), but now with debug logging to understand why.
4. If a new golden bag with denser images becomes available, the range factor + visual factor combination should produce significantly better localization than either alone.
