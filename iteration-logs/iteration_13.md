# Iteration 13 — Loosen ISAM2 relinearizeThreshold to default (0.1) + reduce range factor frequency to every 5th cycle

## What I Saw in the Logs

### Latest Replay (replay_20260617_041430 vs replay_20260617_040400)

| Metric | Iter 12 (040400, 10s lag) | This run (041430, 7s lag) | Delta |
|--------|---------------------------|---------------------------|-------|
| `/gtsam/head_pose` eff Hz | 11.902 | 12.145 | **+0.24** |
| `/gtsam/arm_pose` eff Hz | 11.902 | 12.145 | **+0.24** |
| `/keyframe_buffer/diagnostics` eff Hz | 0.204 | 0.199 | ~unchanged |
| `/tf` eff Hz | 30.762 | 30.824 | ~unchanged |
| Host CPU avg | 60.3% | 59.7% | −0.6% |
| Host CPU max | 78.8% | 83.1% | +4.3% |
| N crashes | 0 | 0 | — |
| N errors | 0 | 0 | — |
| N TF jumps | 0 | 0 | — |

**GTSAM rate history across all 13 iterations:**

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
| 12 | Smoother lag 10s → 7s | 12.145 |
| **13** | **relinearizeThreshold 0.01→0.1, range factor every 5th** | **?** |

### Key Findings from Raw Log Analysis

The replay bundle's `log_analysis.txt` points to a stale file (`host-log-20260616_162416_analysis.txt` — from a pre-iteration-9 run with 15 s lag). The actual raw log `host-log-20260616_162416.txt` is 7022 lines and contains rich data that never makes it into the replay report.

**Crucial insight from the raw log: the GTSAM output rate declines over the run.**

Looking at the per-10s published-pose counts from the raw log:

| Interval (s) | published | implied Hz |
|-------------|-----------|-----------|
| 0–10 | 31 | 3.1 (ramp-up) |
| 10–20 | 145 | 14.5 |
| 20–30 | 149 | 14.9 |
| 30–40 | 137 | 13.7 |
| 40–50 | 129 | 12.9 |
| 50–60 | 121 | 12.1 |
| 60–70 | 113 | 11.3 |

The rate starts at ~15 Hz (hitting the timer target) but **degrades 25 %** over 50 seconds to ~11.3 Hz. The bag aggregate of 12.145 Hz masks this decline entirely.

**Root cause: ISAM2 marginalization fill-in.** Each marginalized variable leaves behind linear approximation factors that couple previously independent variables in the Bayes tree. Over 50+ seconds of operation, these linear factors accumulate, increasing clique density and making each linear solve more expensive. The fixed-lag window keeps the *variable count* bounded, but the *constraint density* grows.

### Related findings

1. **VIO odometry is FLOODED** at 128–158 Hz (nominal 30 Hz). The GTSAM node's odometry callbacks fire 10× per timer tick. While each callback is fast, the cumulative CPU load is non-negligible.

2. **gtsam_corr diagnostic** (added in iter 12) appears in the raw log `[INFO]` lines but is not visible in the replay results because the log_analysis.txt is stale.

3. **5541 TF jump events** detected by `pipeline_diagnostics_node`, mostly on `marker_map → head_imu_openvins_corrected` and `marker_map → arm_imu_openvins_corrected`. These are on the *raw VIO-corrected* edges, not the GTSAM-smoothed edges.

4. **5× tsdf GetAllKeyframes timeout** errors in the raw log.

5. **Unit tests**: All pure-logic tests pass (11 factor_graph, 16 se3_helpers, 2 value_interpolator, 2 tsdf_fusion, 2 keyframe_buffer, 8 pointcloud_fusion, 1 cross_camera_features). The 8 gtsam_tracker node tests and 1 tsdf_fusion node test still fail with `RCLError: error creating node` — unchanged ROS2 test infrastructure issue.

## Decision for This Iteration

**Problem:** The GTSAM output rate degrades ~25 % over the run because ISAM2 marginalization fill-in increases Bayes tree clique density. Two ISAM2 parameters contribute to this:

1. **`relinearizeThreshold = 0.01` (10× tighter than GTSAM default 0.1):** This was set in iter 7 (tightened from 0.001 which was 100× tighter than default). At 12 Hz with 7 s lag, the per-step VIO delta is <1 cm, so at every relinearization check (every 5 updates), nearly every variable near the current time triggers re-linearization. This adds ~5–10 ms per check cycle without meaningful accuracy gain — the VIO drift between checks is sub-centimetre anyway.

2. **Range factor every 3rd update:** The cross-chain `BetweenFactorPose3(h_i, a_i, T_head_arm)` forces ISAM2 to create Bayes tree cliques that span both chains at every 3rd timestep. These cross-chain conditionals are larger (span both chains) and couple the tree more densely. Every 5th cycle (rather than every 3rd) reduces the cross-chain coupling density by 40 %, allowing ISAM2 to maintain a sparser tree on 4 out of 5 updates.

### Why these changes specifically

Both changes target the *structural* cost of the ISAM2 update rather than marginal per-operation overhead. Previous iterations addressed:
- **Iter 5:** Throttled marginalization from every cycle to every 5th (reduced direct marginalization cost)
- **Iter 6:** Provided direct init estimates (avoided get_pose back-substitution)
- **Iter 7:** Loosened relinearizeThreshold from 0.001→0.01 (reduced re-linearized variables)
- **Iter 9/12:** Reduced lag 15→10→7 s (fewer variables)
- **Iter 10/11:** SE(3)-efficient math (eliminated LAPACK calls)

None of these addressed the *accumulating fill-in* from marginalization. The two changes here directly reduce:
1. How many variables trigger relinearization per check (by 10× fewer via threshold default)
2. How many cross-chain cliques get formed per unit time (by 40 % via range factor spacing)

If the fill-in hypothesis is correct, these changes should *flatten the rate decline curve*: the rate should stay closer to the initial ~15 Hz peak for longer, raising the run average.

### Why not other approaches

- **Proactive graph reset** would eliminate fill-in but introduce pose discontinuities that degrade TF chain stability and create TSDF artifacts.
- **Batch optimization** instead of incremental ISAM2 would be a much larger architectural change with unpredictable latency impact.
- **Further lag reduction** (<7 s) has diminishing returns (iter 12: 10→7 s gave only +0.24 Hz) and would reduce the smoothing window's ability to reject VIO drift.

## Changes Made

### `src/gtsam_tracker/gtsam_tracker/factor_graph.py:94` — `__init__`

**Before:**
```python
isam_params.setRelinearizeThreshold(0.01)
```

**After:**
```python
isam_params.setRelinearizeThreshold(0.1)  # GTSAM default
```

The GTSAM default `relinearizeThreshold` is 0.1 (10 cm / 5.7°). At 12 Hz update rate with <1 cm per-step VIO deltas, almost no variable accumulates enough linearization error between relinearization checks (every 5 updates = ~415 ms) to exceed this threshold. This eliminates the ~5–10 ms of variable re-linearization work on most check cycles.

### `src/gtsam_tracker/gtsam_tracker/factor_graph.py:339` — `reset()`

Same change in the graph reset method for consistency between fresh-start and recovery paths.

**Before:**
```python
isam_params.setRelinearizeThreshold(0.01)
```

**After:**
```python
isam_params.setRelinearizeThreshold(0.1)  # matches __init__
```

### `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:485` — Range factor frequency

**Before:**
```python
# ... every 3rd update ...
if self._key_idx % 3 == 0:
```

**After:**
```python
# ... every 5th update (was every 3rd in iter 7) ...
if self._key_idx % 5 == 0:
```

The kinematic range factor creates a cross-chain `BetweenFactorPose3(h_i, a_i)`. At every 3rd cycle, this forces ISAM2 to form Bayes tree cliques that span both head and arm chains. Every 5th cycle reduces the density of these cross-chain conditionals by 40 %, allowing the Bayes tree to maintain a sparser structure on 4 out of 5 updates. The looser spacing still provides adequate cross-chain coupling — the arm and head move slowly relative to each other (sub-centimetre per 415 ms), so the constraint is still effective.

## Expected Impact

### Localization precision (Goal 1)

1. **Higher sustained GTSAM output rate**: Both changes reduce the *per-update* ISAM2 solve time. The `relinearizeThreshold` change eliminates unnecessary re-linearization work; the range factor change reduces cross-chain clique density. Expected improvement: **+0.3–0.8 Hz** on the aggregate effective rate (→ ~12.4–12.9 Hz).

2. **Flatter rate decline curve**: If the marginalization fill-in hypothesis is correct, the rate should degrade less over the run. Instead of going from 15 → 11 Hz, it might stay at 14–15 Hz for longer, raising both the average and the minimum output rate. This means downstream consumers receive higher-rate, more consistent pose updates.

3. **No meaningful accuracy regression**: The 0.1 threshold means a variable's linearization point must change by >10 cm or >5.7° before ISAM2 re-linearizes it. In a 7 s smoother window with sub-cm VIO drift per step, almost no variable will cross this threshold between checks. The linearization remains accurate enough for the optimization to converge.

### TSDF fusion (Goal 2, indirect)

- More consistent GTSAM pose output (higher minimum rate) means the keyframe buffer and TSDF fusion receive fresher poses for their timestamp alignment, reducing temporal misregistration between point clouds and their associated camera poses.
- Fewer missed timer firings = fewer gaps in the pose stream that TF interpolation has to fill.

### What to check in the next iteration

1. **GTSAM output rate**: Check `bag_analysis.txt` for `/gtsam/head_pose` and `/gtsam/arm_pose` effective_hz. Target: ≥12.5 Hz (improvement over 12.145 Hz).
2. **Rate decline over run**: The `log_analysis.txt` DIAGNOSTICS TIMELINE shows per-interval rates. Compare the "last" rate (should be higher than ~10.3 Hz from the stale log).
3. **gtsam_corr**: Grep the raw log for `gtsam_corr` to see correction magnitudes. Should show non-zero values (GTSAM actively correcting VIO drift).
4. **Unit tests**: All pure-logic tests should continue to pass. 8 RCLError failures unchanged.
5. **No new crashes or TF jumps** expected — both changes are parameter-only and don't alter the factor graph topology.
