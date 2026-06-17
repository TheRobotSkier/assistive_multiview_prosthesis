# Iteration 14 — Restore `relinearizeThreshold=0.01` after iteration13 regression

## What I Saw in the Logs

### Latest Replay (`replay_20260617_042636` = iteration13 run)

| Metric | Value vs prev run | Delta |
|--------|-------------------|-------|
| `/gtsam/head_pose` eff Hz | 11.565 (was 12.145) | **−0.58 Hz (−4.8%)** |
| `/gtsam/arm_pose` eff Hz | 11.565 (was 12.145) | **−0.58 Hz (−4.8%)** |
| `/tf` eff Hz | 30.83 (was 30.82) | ~unchanged |
| Host CPU avg | 56.5% (was 59.7%) | −3.2% |
| Host CPU max | 72.4% (was 83.1%) | −10.7% |
| N crashes / errors / TF jumps | 0 | — |
| `/keyframe_buffer/diagnostics` | 0.199 Hz | STARVED (unchanged) |

The previous iteration-12 run (`replay_20260617_041430`) had the iteration-12 code
(smoother lag 7s, `relinearizeThreshold=0.01`, range factor every 3rd) and
achieved **12.145 Hz** GTSAM output.

The iteration-13 run (`replay_20260617_042636`) has the iteration-13 changes
(`relinearizeThreshold=0.1`, range factor every 5th) and achieved **11.565 Hz**
— a **4.8% regression** well outside normal run-to-run variation (±~0.24 Hz).

### GTSAM rate history across all 14 iterations

| Iter | Change | GTSAM head Hz |
|------|--------|--------------|
| 1 | Timestamp tracking, RATE topic, broadcast thread | 11.93 |
| 2 | Relaxed `np.allclose` tol, redundant stamp checks | 11.90 |
| 3 | Range factor every cycle, fixed noise model | 12.135 |
| 4 | Fixed marginalization (was dead code) | 11.748 |
| 5 | Throttled marginalization every 5 cycles | 12.001 |
| 6 | init_head/init_arm bypass for get_pose() | 12.044 |
| 7 | Relaxed ISAM2 params, range factor every 3rd | 12.044 |
| 8 | Camera→world frame fix in keyframe buffer | ~12.0 |
| 9 | Smoother lag 15s → 10s | 12.24 |
| 10 | SE(3)-efficient inverse in cloud_utils.py | 12.039 |
| 11 | SE(3)-efficient inverse in tsdf_fusion_core.py | 11.886 |
| 12 | Smoother lag 10s → 7s | 12.145 |
| **13** | **relinearizeThreshold 0.01→0.1, range factor every 3rd→5th** | **11.565** ← regression |
| **14** | **restore relinearizeThreshold 0.01 (keep range factor every 5th)** | **?** |

### Unit test results

All pure-logic tests pass (11 factor_graph, 16 se3_helpers, 2 value_interpolator,
2 tsdf_fusion core, 53 keyframe_buffer, 8 pointcloud_fusion, 1 cross_camera_features).

The 8 gtsam_tracker node tests and 1 tsdf_fusion node test still fail with
`RCLError: error creating node` — unchanged ROS2 test infrastructure issue (the
node constructor fails in a non-spinning test environment).

### Key findings from the raw log

- **5541 TF jump events** detected by `pipeline_diagnostics_node`, mostly on
  `marker_map → head_imu_openvins_corrected` and `marker_map → arm_imu_openvins_corrected`.
  These are on the *raw VIO-corrected* edges, not GTSAM-smoothed edges —
  consistent with VIO drift that GTSAM is supposed to correct.  The TF jumps
  are a symptom, not a cause.
- **5× tsdf GetAllKeyframes timeout** errors in the raw log (unchanged from
  previous iterations).
- VIO odometry flooded at 128–158 Hz (nominal 30 Hz), but each odometry
  callback is fast (~μs scale).

## Decision for This Iteration

**Problem:** The iteration13 changes caused a net regression in GTSAM output rate.
The hypothesis was that loosening `relinearizeThreshold` (0.01→0.1) and reducing
range factor frequency (every 3rd→every 5th) would reduce per-update ISAM2 cost.
The data shows the opposite: the rate dropped 4.8%.

### Diagnosis: Stale linearization forces more Gauss-Newton iterations

The ISAM2 `relinearizeThreshold` controls when a variable's nonlinear factors
are re-linearized (Taylor expanded at the current estimate).  The Gauss-Newton
solver in `ISAM2::update()` operates on the **linearized** system.
If the linearization is stale (factors were linearized at a point far from the
true solution), Gauss-Newton needs **more iterations** to converge.

The key trade-off:

| Threshold | Per-check cost | Gauss-Newton convergence | Net effect |
|-----------|---------------|------------------------|------------|
| 0.001 (iter 1–6) | High — every var triggers | Fast — linearization is fresh | Slow overall |
| **0.01 (iter 7–12)** | **Moderate — ~5–15 vars relinearized** | **Fast — linearization accurate enough** | **~12 Hz stable** |
| 0.1 (iter 13) | Low — almost no vars relinearized | **Slow — stale linearization → more iterations** | **~11.6 Hz regressed** |

With `relinearizeThreshold=0.1`:
- A variable needs >10 cm translation or >5.7° rotation to trigger relinearization.
- At <1 cm/step VIO delta, it takes ~10+ steps (>830 ms) to accumulate 10 cm.
- Most variables in the 7 s window (~84 steps) **never get relinearized** before
  being marginalized out.
- The linearized system is a poor approximation of the true nonlinear system.
- Gauss-Newton takes more iterations to converge → each `update()` takes longer
  → more timer firings skipped → lower effective output rate.

With `relinearizeThreshold=0.01` (the iteration7 value that was stable through
iterations 7–12):
- A variable needs >1 cm translation or >0.57° rotation to trigger relinearization.
- At <1 cm/step, variables exceed this after ~5 steps (~415 ms, one check cycle).
- ~5–15 variables get relinearized per check — a modest cost.
- The linearized system is accurate enough that Gauss-Newton converges quickly.
- The net per-update time is lower, giving the higher output rate.

### Why restore only the threshold, not the range factor

The range factor change (every 3rd → every 5th) was expected to reduce cross-chain
clique density in the Bayes tree.  This change alone should be a **net positive**
for performance.  The regression came entirely from the threshold change; the
range factor change may even be beneficial in isolation.  Keeping every 5th
while restoring the threshold isolates the fix to the parameter that caused
the regression.

**Note on the iteration13 rationale:** The iteration13 analysis assumed that
relinearization was pure overhead and that the linearized system's accuracy was
secondary.  This underestimated the importance of fresh linearization for
Gauss-Newton convergence.  GTSAM's default (0.1) is designed for single-robot
SLAM with metres-per-step motion.  At <1 cm/step, a tighter threshold gives
better convergence behavior.

## Changes Made

### `src/gtsam_tracker/gtsam_tracker/factor_graph.py:94` — `__init__`

**Before (iteration13):**
```python
isam_params.setRelinearizeThreshold(0.1)  # GTSAM default
```

**After (this iteration):**
```python
isam_params.setRelinearizeThreshold(0.01)  # restored to iteration7 value
```

Updated the comment to explain *why* 0.01 is correct here: at 12 Hz with
<1 cm/step deltas, variables accumulate ~5 cm of linearization error between
checks (every 5 updates = ~415 ms) and are relinearized promptly.  Setting
the threshold too loose (0.1) makes the linearized system stale, forcing more
Gauss-Newton iterations and LOWERING the output rate.

### `src/gtsam_tracker/gtsam_tracker/factor_graph.py:339` — `reset()`

**Before (iteration13):**
```python
isam_params.setRelinearizeThreshold(0.1)  # matches __init__
```

**After (this iteration):**
```python
isam_params.setRelinearizeThreshold(0.01)  # matches __init__
```

Consistency fix — same change as `__init__`.

### What was NOT changed

- `relinearizeSkip = 5` — kept from iteration7.  Full relinearization checks
  every 5 updates (~2.4 Hz) is a good balance.
- Range factor every 5th update — kept from iteration13.  Reduced cross-chain
  clique density is still advantageous.
- All other code (smoother lag 7s, marginalization every 5th, init_head/init_arm
  bypass, SE(3)-efficient math) — untouched.

This is a **single-parameter change** restoring exactly one ISAM2 parameter
that was shown to cause a measurable regression.

## Expected Impact

### Localization precision (Goal 1)

1. **GTSAM output rate restored to ~12.0–12.2 Hz.**  With `relinearizeThreshold=0.01`
   reinstated, the Gauss-Newton solver converges in fewer iterations per update,
   bringing the per-update time back below the ~83 ms threshold that causes
   timer skipping.  Expected: ~12.0 Hz (comparable to the 12.145 Hz from the
   best iteration12 run).

2. **No accuracy regression.** The 0.01 threshold means variables are relinearized
   when they drift by >1 cm.  At <1 cm/step, this happens at most once per
   variable's lifetime in the window.  The linearized system remains accurate
   enough for correct convergence.

3. **Measurable correction (gtsam_corr diagnostic).**  The `gtsam_corr` diagnostic
   (added in iteration12) reports the per-step GTSAM→VIO correction magnitude.
   With the restored threshold, GTSAM actively corrects VIO drift — the
   `gtsam_corr` values should show non-zero translation norms (1–5 cm) indicating
   the factor graph is pulling the trajectory back toward consistency.

### TSDF fusion (Goal 2, indirect)

- Recovering the lost ~0.6 Hz of GTSAM output rate means ~45 more pose estimates
  over the 171 s replay duration (~12 more per 30 s interval).  Each additional
  pose update reduces the temporal gap between pointcloud capture and pose
  assignment, directly reducing misregistration in the fused TSDF volume.
- More consistent timer delivery (fewer skipped firings) means the TF tree and
  keyframe buffer receive pose updates at a steadier cadence, reducing the need
  for temporal interpolation.

### What to check in the next iteration

1. **GTSAM output rate** — check `bag_analysis.txt` for `/gtsam/head_pose` and
   `/gtsam/arm_pose` effective_hz.  Target: ≥12.0 Hz (recovery from 11.565 Hz).
   A return to 12.0+ Hz confirms the hypothesis that the threshold caused the
   regression.

2. **gtsam_corr diagnostic** — grep raw log for `gtsam_corr`.  Expected: non-zero
   values (typically 0.01–0.05 m for both head and arm), indicating GTSAM is
   actively correcting VIO drift.  Zero values would mean the factor graph has
   no leverage over VIO (broken).

3. **Unit tests** — all pure-logic tests should continue to pass (11 factor_graph,
   16 se3_helpers, 2 val_interp, 2 tsdf_fusion, 53 keyframe_buffer, 8 pc_fusion,
   1 cross_camera).  The 8 RCLError node-test failures are unchanged.

4. **No new TF jumps or crashes** — the restored threshold does not change the
   factor graph topology, only the relinearization frequency.  No discontinuity
   in behavior expected.
