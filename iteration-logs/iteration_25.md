# Iteration 25 — Stop re-publishing already-smoothed keys (root cause of 3000+ TF jumps)

## What was observed in the logs

### Latest replay results (`logs/replay-results/replay_20260617_063728/`)

The pipeline is stable (all 14 nodes start, GTSAM at 14.3 Hz, TF at 50.7 Hz). Key metrics:

| Known Failure | Iter 24 (σ=0.10) | Iter 23 (σ=0.02) | Delta |
|---|---|---|---|
| TF jump events | **3302** | 2990 | **+312 (↑10%)** |
| odom pose jumped — TF suppressed | 81 | 81 | same |
| gtsam odometry rejected | 2 | 2 | same |
| tsdf GetAllKeyframes timeout | 6 | 15 | ↓9 (improved) |
| TF chain disconnected | 17 | 17 | same |

### Critical finding — The iteration-24 fix made TF jumps WORSE

Iteration 24 increased odometry noise sigmas 5× (0.02 → 0.10 m / rad) to make GTSAM track VIO less aggressively, predicting TF jumps would drop from 2990 to <400. **Instead they increased to 3302.**

This counterintuitive result reveals a deeper root cause:

#### Root cause: GTSAM re-publishes an already-published key every cycle

In `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:813-814`, `_publish_poses` publishes key `max(key_idx - 1, 0)` — the **previous** cycle's key index. Every 67 ms ISAM2 adds a new key and **re-optimizes every key in the 7-second window**, including the key published in the previous cycle. Since the trajectory changes with every new measurement (the smoother re-distributes corrections), the same key index has a *different pose* each time it's published. The TF jump detector sees this as a sudden 0.4–0.7 m jump in the TF tree.

With tighter noise (σ=0.02, iter 23), the factor graph was stiff: each between-factor constrained poses tightly, so re-optimization corrections were small (~2990 jumps). With looser noise (σ=0.10, iter 24), the graph is more elastic: each re-optimization can produce a larger change to old poses as the optimizer distributes corrections across the window, increasing jumps to ~3302.

The fix in iteration 24 (looser noise) was addressing the **symptom** (trajectory jitter) but not the **root cause** (re-publishing already-smoothed keys). Worse, looser noise amplified the root cause.

#### Supporting evidence from the logs

1. **TF jump bursts are periodic**: Logs show "peak burst: 150 jumps in 5s window". At 14.3 Hz GTSAM output, 150 jumps in 5s = a jump on nearly every publish. This is consistent with every cycle producing a re-optimized key that differs from its previous value.

2. **Jump magnitudes (0.4–0.7 m) match smoother correction**: The pipeline_diagnostics samples show 0.398–0.676 m jumps. These magnitudes are consistent with ISAM2 redistributing the trajectory within the 7 s window when a new VIO sample arrives.

3. **Looser noise → more trajectory elasticity**: With σ=0.02 (iter 23), the between-factors were stiff — each key was tightly coupled to its neighbor, so re-optimization corrections were small. With σ=0.10 (iter 24), the optimizer has more freedom to adjust old poses when new measurements arrive, making the re-published key differ more from its previous value. This is why TF jumps INCREASED with the "fix".

## What was decided to work on

**Problem**: GTSAM publishes `key_idx - 1` (an already-published key) every cycle. Each cycle's ISAM2 update re-optimizes the entire sliding window, changing the pose of every key, including the one about to be (re-)published. This causes 3000+ TF jumps with 0.4–0.7 m magnitudes, directly harming both localization precision (unstable TF tree) and TSDF fusion (inconsistent poses for voxel ray-casting).

**Fix**: Change `_publish_poses` to publish `key_idx` (the CURRENT key, just added and optimized in this cycle) instead of `key_idx - 1`. This ensures each key is published **exactly once** — immediately after its first and only ISAM2 optimization. Consecutive publishes then compare key N vs key N+1, which differ by the **real VIO delta** (3–13 cm at 15 Hz for normal prosthesis motion) plus a small optimization correction, well below the 0.5 m TF jump threshold.

**Rationale**: In a sliding-window smoother, publishing an already-smoothed key is always incorrect for real-time TF broadcasting — the pose changes every cycle as new measurements arrive. The correct approach for real-time TF is a **filtered estimate**: each timestamp's pose is computed once (when it enters the smoother) and published once. The 7-second lag window still provides the smoothing benefit (range factors, noise filtering, cross-chain coupling), but each key's published value is stable.

This is a well-known pattern in robotics:
- **Smoothed estimate** (what the code was doing): Optimal trajectory given ALL data in the window. Changes with every new measurement. Used for offline reprocessing or delayed planning.
- **Filtered estimate** (what this fix does): Optimal estimate at insertion time. Stable, publish-once. Used for real-time TF, visualization, and control.

The GTSAM 7-second window still re-optimizes internally, but the TF tree only sees each key once.

## What was changed

### 1. `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:813-820` — Publish current key instead of previous cycle's key

**Before (lines 813-814):**
```python
kh = self._graph._make_key("h", max(self._key_idx - 1, 0))
ka = self._graph._make_key("a", max(self._key_idx - 1, 0))
```

**After (lines 813-820):**
```python
# Publish the CURRENT key (the one just added this cycle) so each key
# is published exactly ONCE after its first optimization.  Publishing
# key_idx-1 (the previous cycle's key) was the root cause of 3000+ TF
# jumps: every ISAM2 re-optimization changed the already-published key,
# causing 0.4-0.7 m discontinuities in the TF tree.
# See iteration-25 analysis in iteration-logs/.
kh = self._graph._make_key("h", self._key_idx)
ka = self._graph._make_key("a", self._key_idx)
```

### No other changes

The odometry noise sigmas remain at 0.10 m / 0.10 rad (from iteration 24). With the publish-once fix, looser noise no longer amplifies TF jumps — each key is published once regardless. The loose noise still provides better range-factor coupling and VIO noise filtering.

## Expected impact

### Goal 1 — Localization precision (GTSAM factor-graph optimization)

1. **TF jump count drops from ~3300 to near-zero**: Each key is published exactly once. Consecutive publishes compare key N vs key N+1 — two different timestamps — whose poses differ by the real VIO delta (typically 3–13 cm at 15 Hz for human prosthesis motion). This is well below the 0.5 m TF jump threshold. The only TF jumps that remain would be from genuine VIO discontinuities >0.5 m (already caught by the delta gate at ~81 per run).

2. **Temporally consistent TF tree**: The `marker_map → head_imu` and `marker_map → arm_imu` edges will update monotonically (following real motion) instead of jumping back and forth as re-optimizations change old poses. This makes all downstream TF lookups (keyframe_buffer spatial gating, pointcloud fusion coordinate transforms) reliable.

3. **GTSAM → odom correction magnitude remains moderate**: Each key's optimization correction (the difference between raw VIO and optimized GTSAM pose at insertion) is ~5–10 cm with the loose noise model. This correction is applied once and never revised, so the published trajectory has consistent accuracy.

4. **Latency improves by ~67 ms**: The old code published key N−1, adding a one-cycle lag (~67 ms at 15 Hz). Publishing key N eliminates this delay, providing more responsive pose estimates to downstream consumers.

### Goal 2 — TSDF fusion quality (indirect)

1. **Keyframe buffer gets stable poses**: With a monotonic, non-jumping TF tree, the keyframe buffer's spatial gating (inserting keyframes when the camera moves >N cm) operates on consistent pose differences. Previously, a 0.5 m TF jump would either trigger a spurious keyframe insertion or cause the buffer to skip a valid one.

2. **TSDF voxel integration becomes more consistent**: Each keyframe's pointcloud is transformed using the GTSAM-published pose at insertion time. With stable, non-revised poses, consecutive TSDF integrations see consistent spatial relationships — voxels are updated in coherent patterns rather than being partially overwritten by pose-corrected cloud transforms.

3. **Fewer GetAllKeyframes timeouts**: The 6 timeouts per run (down from 15 in iteration 23) may decrease further if the keyframe buffer spends less time servicing inconsistent pose lookups triggered by TF jumps.

### Risks

1. **Loss of "future-data" smoothing**: Each key is published before future measurements arrive. This means the 7-second lag window's full smoothing benefit (using data from the next 3.5 seconds to refine past poses) is NOT reflected in the TF tree. The published trajectory is a **filtered** estimate, not a fully-smoothed one. This is acceptable because:
   - The filtered estimate still uses the range factor (every 5th cycle) to couple head and arm
   - The between-factor at insertion time already incorporates the VIO delta and previous key's optimized pose
   - For real-time TF broadcasting, filtered estimates are standard practice (the EKF-based VIO itself produces filtered estimates)
   - The alternative (re-publishing smoothed poses) causes unacceptable TF tree instability

2. **No change in output rate**: The fix does not affect the optimization rate (15 Hz) or the number of published messages. Only the key index selection changes.

3. **No change to internal GTSAM behavior**: ISAM2 still re-optimizes the full 7-second window on every cycle. The `_log_stats` diagnostics (which use `key_idx - 1` to compute correction magnitudes) still work correctly — they compare raw VIO vs the optimized pose of the latest key, which is meaningful regardless of publishing policy.

### Diagnostic checks for the next iteration

1. **TF jump count**: Check `log_metrics.json` → `tf_jump_events_total`. Expect <100 (down from 3302). Residual jumps should only come from genuine VIO discontinuities >0.5 m, already counted in `odom pose jumped — TF suppressed` (~81 per run).

2. **GTSAM output Hz**: `/gtsam/head_pose` and `/gtsam/arm_pose` should remain at ~14.3 Hz (unchanged by this fix).

3. **GTSAM correction magnitude**: Search the host log for `gtsam_corr` — expect `head_corr` and `arm_corr` to show the 5–10 cm correction per key (same as iteration 24).

4. **Keyframe buffer diagnostics**: `/keyframe_buffer/diagnostics` Hz may improve slightly (fewer dropped messages due to TF lookup errors).

5. **TSDF timeout count**: Check `tsdf GetAllKeyframes timeout` — expect ≤6 (same or better than iteration 24).
