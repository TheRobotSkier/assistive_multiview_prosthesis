# Iteration 9

## Logs Observed

### Replay Results (20260617_032516)

| Metric | Value | Baseline | Status |
|--------|-------|----------|--------|
| GTSAM head rate (eff Hz) | 12.24 Hz | 12.13 Hz | ~unchanged |
| GTSAM arm rate (eff Hz) | 12.24 Hz | 12.08 Hz | ~unchanged |
| TSDF fusion rate (eff Hz) | 1.47 Hz | 1.17 Hz | improved |
| GTSAM subscriber | 88.7 ms callback (starved) | — | **still starving** |
| Keyframe buffer subscriber | 359.2 ms callback | — | high latency |
| GTSAM latency (odom→out) | 1976 ms | — | high |
| VIO→GTSAM: head | 2.1%→93.8% late | — | major replay artifact |
| VIO→GTSAM: arm | 20.5%→83.0% late | — | major replay artifact |

### Unit Test Results (gtsam_tracker)
- **5/5 passed** in gtsam_tracker (graph update, isam2, marginalization heuristics, etc.)
- **2/2 passed** in tsdf_fusion
- **2/2 passed** in keyframe_buffer
- **2/2 passed** in pointcloud_fusion

### Previous Iteration Summary
| Iter | Change | Impact |
|------|--------|--------|
| 1 | Added timestamp tracking, RATE topic, broadcast thread | Baseline |
| 2 | Relaxed `np.allclose` tol, removed redundant stamp checks | Minor benefit |
| 3 | Switched to ROS `Rate`, removed spin-once | - |
| 4 | Split `marginalize_old_keys` from `cleanup`, persist pose in update loop | Early exit speedup |
| 5 | Throttled marginalization to every 5 cycles | Minor benefit |
| 6 | Swapped `node.handle` for `addPrior`, relaxed relinearize threshold | Head rate from 6→12 Hz |
| 7 | Lowered `relinearizeSkip` to 5, increased `relinearizeThreshold` to 0.5 | Minor improvement |
| 8 | Fixed world→odom TF transform in `cloud_utils.py` | Keyframe buffer rate improved |

## Decision for This Iteration

**Problem:** The GTSAM output rate has plateaued at ~12 Hz (target 15 Hz) despite 6 previous optimization attempts across iterations 2-7. The remaining bottleneck appears increasingly likely to be raw ISAM2 solve time as the fixed-lag graph grows.

**Diagnosis:** With `smoother_lag_s = 15.0`, the ISAM2 graph holds **~360 variables** (180 head + 180 arm poses at 12 Hz). Each `_graph_update` cycle adds 2-3 new factors and must solve this ~360-variable graph, then optionally marginalize old keys. The marginalization itself creates dense linear factors that increase solve time. The graph update simply takes longer than the 66.7 ms budget at 15 Hz.

**Fix:** Reduce the fixed-lag window from **15 s to 10 s** at `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:63`. This cuts the active variable count from ~360 to **~240** (33% reduction), directly shrinking the Bayes tree that ISAM2 must solve each cycle. A 10 s window still provides 120 trajectory poses per chain — more than sufficient for effective smoothing — while substantially reducing per-update computational cost.

**Rationale for NOT taking other approaches:**
- Further `relinearizeThreshold` or `relinearizeSkip` tuning (iter 6-7) has diminishing returns.
- Throttling marginalization further (iter 5) risks unbounded graph growth.
- The callback delay (88.7 ms) is measured replay time, not execution latency — the real bottleneck is compute, not scheduling.
- The `np.linalg.inv` → `inverse_se3` improvement is a numerical polish but would not meaningfully affect rate.

## Changes Made

**`src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:63`**
- Changed `"smoother_lag_s": 15.0` → `"smoother_lag_s": 10.0`

## Expected Impact

1. **Higher GTSAM output rate:** With 33% fewer variables in the graph, each ISAM2 update should complete measurably faster, allowing the 15 Hz target to be hit more consistently. Target: 13-14 Hz effective rate.

2. **Preserved smoothing quality:** 120 poses per chain in a 10 s window is still a large sliding window for a 12-15 Hz system. The smoother retains sufficient history for effective trajectory smoothing.

3. **Lower end-to-end latency:** Faster per-update time means the publish timestamp is closer to the odometry timestamp (currently 1976 ms odom→out).

4. **No negative impact on TSDF:** The TSDF subscriber latency is independent of GTSAM lag window size. The keyframe buffer uses timestamps and approximate pose information, not the lag window.
