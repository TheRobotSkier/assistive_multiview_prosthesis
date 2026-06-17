# Iteration 12 — Reduce smoother lag to 7s + add odom→gtsam correction diagnostic

## What I Saw in the Logs

### Latest Replay (replay_20260617_040400 vs replay_20260617_035525)

| Metric | Iter 11 (035525) | This run (040400) | Delta |
|--------|-----------------|-------------------|-------|
| `/gtsam/head_pose` eff Hz | 11.886 | 11.902 | +0.016 |
| `/gtsam/arm_pose` eff Hz | 11.886 | 11.902 | +0.016 |
| `/keyframe_buffer/diagnostics` eff Hz | 0.200 | 0.204 | ~unchanged |
| `/tf` eff Hz | 30.978 | 30.762 | −0.216 |
| Host CPU avg | 58.5% | 60.3% | +1.8% |
| Host CPU max | 88.5% | **78.8%** | **−9.7%** |
| N crashes | 0 | 0 | — |
| N errors | 0 | 0 | — |
| N TF jumps | 0 | 0 | — |

**GTSAM rate trajectory across all 12 iterations:**

| Iter | Change | GTSAM head Hz |
|------|--------|--------------|
| 1 | Timestamp tracking, RATE topic, broadcast thread | 11.93 |
| 2 | Relaxed `np.allclose` tol, redundant stamp checks | 11.90 |
| 3 | Range factor every cycle, fixed noise model | 12.135 |
| 4 | Fixed marginalization (was dead code) | 11.748 |
| 5 | Throttled marginalization every 5 cycles | 12.001 |
| 6 | init_head/init_arm bypass for get_pose() | 12.044 |
| 7 | Relaxed ISAM2 params, range factor every 3rd | 12.044 |
| 8 | Camera→world frame fix in keyframe buffer | (rate unchanged) |
| 9 | Smoother lag 15s → 10s | 12.24 |
| 10 | SE(3)-efficient inverse in cloud_utils.py | 12.039 |
| 11 | SE(3)-efficient inverse in tsdf_fusion_core.py | 11.886 |
| **12** | **Smoother lag 10s→7s + odom→gtsam diagnostic** | **?** |

### Key Findings

1. **Rate plateau at ~12 Hz**: Despite 11 optimization iterations (ISAM2 params, marginalization throttling, lag reduction, SE(3)-efficient math, init bypass), the GTSAM output rate has remained stuck at 11.7–12.2 Hz. The 15 Hz timer has a 66.7 ms period, but the `_graph_update` callback takes ~83 ms, so ~1 in 4 timer firings is skipped.

2. **No way to measure Goal 1**: The bag analysis only reports topic effective Hz — there is no metric for "odom→gtsam agreement" or "correction magnitude." Without this, we cannot tell whether the GTSAM smoothing is actually improving over raw VIO, which is the core requirement of Goal 1.

3. **No crashes, errors, or TF jumps** in the latest run. 0 failures.

4. **Test results**: 8 gtsam_tracker failures + 1 tsdf_fusion failure — all `RCLError: error creating node` (ROS2 test infrastructure). All pure-logic tests pass (factor graph, SE3 helpers, keyframe buffer, pointcloud fusion, TSDF core).

## Decision for This Iteration

Two changes, both focused on the stated goals:

### Change 1: Reduce smoother lag 10s → 7s (rate improvement)

**Diagnosis:** The ISAM2 fixed-lag smoother currently holds ~240 variables (120 head + 120 arm at 12 Hz × 10 s). Each `_graph_update` cycle solves this graph in ~83 ms, exceeding the 66.7 ms timer budget. Reducing to 7 s cuts variables to ~168 (30 % fewer), which should proportionally reduce ISAM2 solve time.

The iteration-9 change (15→10 s) gave a ~0.2 Hz improvement, but the effect was partially masked by the remaining overhead (~10–15 ms of factor creation, data conversion, marginalization bookkeeping). Going from 10→7 s is a further 30 % variable reduction, and with the overhead already minimized by prior iterations (range factor every 3rd cycle, marginalization every 5th cycle, init_head/init_arm bypass, SE(3)-efficient math), a higher fraction of the per-cycle saving should translate to rate improvement.

**Why 7 s?** At 12 Hz, 7 s provides 84 poses per chain — still a large sliding window that smooths VIO drift effectively. The robot arm camera system typically has smooth motion with minimal long-term drift, so a shorter window is sufficient for the GTSAM correction to converge, while leaving more compute headroom for higher throughput.

**Why not other approaches:**
- Further ISAM2 parameter relaxation has diminishing returns — relinearizeThreshold is already 0.01 (10× looser than default GTSAM).
- Throttling the graph update rate (e.g., 12.5 Hz timer instead of 15 Hz) would not improve throughput; it would just reduce jitter.

### Change 2: Add odom→gtsam correction diagnostic (measurability)

**Diagnosis:** The replay analysis provides topic effective Hz but no trajectory-quality metrics. Without measuring the odom→GTSAM relative transform, we cannot evaluate Goal 1 ("GTSAM factor-graph optimization clearly improves localization precision over raw VIO").

**Fix:** The `_log_stats` method (runs every 10 s) now computes `gtsam_corr(head, arm)` — the translation magnitude of `relative_transform(raw_VIO_pose, GTSAM_smoothed_pose)`. This directly measures how much the factor graph is correcting VIO drift. These values appear in the log output and will be captured in `log_analysis.txt`, giving future iterations a time-series metric of GTSAM's correction effectiveness.

Expected correction magnitude: >0 indicates GTSAM is actively correcting VIO drift. Ideally small (a few cm) for a well-tuned VIO, but non-zero — zero would mean GTSAM is not improving over VIO.

## Changes Made

### `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:64`

**Reduced smoother lag from 10.0 s to 7.0 s:**

```python
# BEFORE:
"smoother_lag_s": 10.0,
# AFTER:
"smoother_lag_s": 7.0,
```

This parameter is read at line 199 and passed to `TrajectoryFactorGraph(lag_s=self._lag_s)` at line 215. The ISAM2 fixed-lag window shrinks from 10 s to 7 s, reducing the active variable count from ~240 to ~168 (30 % reduction).

### `src/gtsam_tracker/gtsam_tracker/factor_graph.py:84`

**Updated default `lag_s` to match node parameter:**

```python
# BEFORE:
def __init__(self, lag_s: float = 15.0):
# AFTER:
def __init__(self, lag_s: float = 7.0):
```

The old default (15.0) was the pre-iteration-9 value and was never actually used at runtime (the node always passes the parameter). Changing it to 7.0 keeps the graph constructor consistent with the node's active configuration.

### `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:34`

**Added `pose3_to_matrix` to imports** for the correction diagnostic:

```python
from gtsam_tracker.se3_helpers import (
    pose_to_matrix,
    matrix_to_pose,
    pose3_to_matrix,          # ← added
    inverse_se3,
    relative_transform,
)
```

### `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:726-745`

**Added odom→GTSAM correction diagnostic to `_log_stats`:**

Computes `gtsam_corr(head, arm)` — the translation magnitude of `relative_transform(VIO_pose, GTSAM_smoothed_pose)` — and includes it in the periodic stats log output. The computation:
1. Looks up the GTSAM key for the latest smoothed pose (`key_idx - 1`)
2. Converts `gtsam.Pose3` to numpy matrix via `pose3_to_matrix`
3. Computes `relative_transform(raw_VIO, GTSAM)` — the SE(3) delta from VIO to GTSAM
4. Reports `norm(delta[:3, 3])` as the correction magnitude in metres

Example log output: `gtsam_corr(head=0.0234m, arm=0.0189m)` — showing GTSAM is applying ~2 cm correction to the raw VIO head estimate.

## Expected Impact

### Localization precision (Goal 1)

1. **Higher GTSAM output rate**: With 30 % fewer variables in the ISAM2 graph, each `update()` should complete measurably faster. Expected improvement: **~12.0 Hz → 13–14 Hz** (reducing the gap to the 15 Hz timer target). More pose estimates per second = less interpolation error downstream.

2. **Measurable correction**: The new `gtsam_corr(head, arm)` diagnostic in the stats log makes the GTSAM→VIO correction visible, directly addressing Goal 1's requirement that "GTSAM factor-graph optimization clearly improves localization precision over raw VIO." Future iterations can track whether `gtsam_corr` trends in the right direction.

3. **No degradation in smoothing quality**: 84 poses per chain (at 12 Hz × 7 s) is still a large sliding window. The range factor coupling every 3rd cycle keeps head-arm chains consistent. VIO drift over 7 s is typically sub-centimetre, and the GTSAM Bayes tree marginalization preserves information from older poses through linear factors.

### TSDF fusion (Goal 2, indirect)

- Higher and more consistent GTSAM pose output rate → more pose updates available for pointcloud integration → denser, more spatially consistent fused TSDF volume.
- Less timing jitter from missed timer firings reduces temporal misalignment between pointcloud capture and its associated pose.

### What to check in the next iteration

1. **GTSAM output rate** — check `bag_analysis.txt` for `/gtsam/head_pose` and `/gtsam/arm_pose` effective_hz. Target: ≥13.0 Hz.
2. **Odom→GTSAM correction** — grep `log_analysis.txt` for `gtsam_corr`. Expected: non-zero positive values (indicating GTSAM is actively correcting VIO drift), typically 1–10 cm.
3. **Unit tests** — all pure-logic tests should continue to pass. The 8 RCLError failures are unchanged (ROS2 test infrastructure).
4. **No new crashes or TF jumps** expected — the lag reduction only changes which keys ISAM2 marginalizes, not the factor topology.
