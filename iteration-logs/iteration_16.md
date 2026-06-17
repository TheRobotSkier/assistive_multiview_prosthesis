# Iteration 16 — Enable GTSAM TF broadcasting so smoothed poses actually reach the pipeline

## What I Saw in the Logs

### Latest replay results (`replay_20260617_044406`)

| Metric | Value |
|--------|-------|
| `/gtsam/head_pose` eff Hz | 11.805 — but declining from 15.2 → 9.1 Hz over the run |
| `/gtsam/arm_pose` eff Hz | 11.805 — same decline pattern |
| `/tf` eff Hz | 30.9 (nominal 50) |
| Host CPU avg/max | 61% / 92% |
| N crashes / errors | 2 `[ERROR]`, 2 `Traceback` (tf_pipeline_diagnostics, camera_mount_tf_publisher) |
| gtsam_corr in log | **0 occurrences** — the diagnostic was supposed to appear but does not |
| tf_jump_events_total | **0** — despite 79 "odom pose jumped — TF suppressed" events |
| `/vis/head_arm_pose` | **0.0 Hz** throughout the entire run |
| `gtsam odometry rejected` | 2x (early run, recovery from VIO divergence) |
| `odom pose jumped — TF suppressed` | 79x |
| `TF chain disconnected` | 18x |

### GTSAM rate decline (15.2 → 9.1 Hz)

The diagnostic timeline shows a 40% rate decline:
- **First interval**: 15.2 Hz (peak)
- **Mid interval**: 11.4 Hz
- **Last interval**: 9.1 Hz

This is caused by the 15-second smoother lag (`smoother_lag_s: 15.0`) which keeps ~180 poses per chain in the ISAM2 graph, and the relinearization overhead as the Bayes tree grows.

### The `/vis/head_arm_pose` starvation (0.0 Hz)

The visual factor topic from `cross_camera_features` never publishes, meaning GTSAM runs without cross-chain visual constraints — only odometry factors (independent per chain) and kinematic range factors (every 5 updates). This significantly weakens the cross-chain coupling.

## Critical Root Cause — GTSAM Output Is Invisible to the Pipeline

After reading the source code and config, I discovered the fundamental blocker for Goal 1:

**`gtsam_tracker.broadcast_tf: false`** (`config/prosthesis_config.yaml:502`)

The `gtsam_tracker_node.py:855-868` has the code to broadcast smoothed poses as dynamic TF edges (`marker_map -> head_imu`, `marker_map -> arm_imu`), but it's gated by `self._broadcast_tf` which is set from the config parameter. When `false`, the function only publishes to the raw topic (`/gtsam/head_pose`, `/gtsam/arm_pose`) but does NOT call `sendTransform()`.

Meanwhile, **`openvins_odom_tf_relay.publish_dynamic_tf: true`** (confirmed via `config/prosthesis_config.yaml:323` and relay startup log) means the relay publishes raw VIO as the `marker_map -> head_imu` and `marker_map -> arm_imu` TF edges that all downstream nodes use.

The data flow is:

```
OpenVINS odom (150 Hz)  ──→  relay  ──→  TF tree (raw VIO)
                         ──→  GTSAM  ──→  /gtsam/head_pose topic (nobody uses for TF)
```

Goal 1 *requires* that GTSAM's factor-graph-smoothed poses are what downstream nodes (pointcloud_fusion, keyframe_buffer, rviz) consume through the TF tree. With `broadcast_tf: false`, GTSAM is a pure diagnostic node — its output has zero impact on the pipeline.

## Decision for This Iteration

**Problem:** GTSAM's smoothed poses are published to topics but never reach the TF tree. Downstream nodes consume raw VIO from the relay. Goal 1 cannot be demonstrated.

**Fix:** Swap the dynamic TF ownership:
1. Enable `gtsam_tracker.broadcast_tf: true` — GTSAM publishes smoothed `marker_map -> head_imu` / `marker_map -> arm_imu` TF edges
2. Disable `openvins_odom_tf_relay.publish_dynamic_tf: false` — relay stops publishing raw VIO dynamic TF edges (still publishes static `*_imu -> *_cam0` for the TF chain)

The relay code already handles this gracefully at `openvins_odom_tf_relay.py:431-443`: when `publish_dynamic_tf=False`, it returns early after validation (init guard, jump suppression) without broadcasting the dynamic edge, while still logging periodic status messages.

The GTSAM code at `gtsam_tracker_node.py:855-868` already has the TF broadcasting logic — it just needs the config flag.

### Why this over other fixes

- **Rate decline fix** (reducing `smoother_lag_s` from 15 to 7): Would keep GTSAM faster, but faster output that nobody uses doesn't help Goal 1.
- **Odom rate limiting**: Reduces wasted CPU but doesn't change the fundamental TF ownership issue.
- **Visual factor starvation**: Important but is a separate upstream pipeline issue in `cross_camera_features`.
- **TF jump / crash fixes**: Nice-to-have but not the Goal 1 blocker.

This fix is the **prerequisite** for Goal 1 to be measurable. With `broadcast_tf=true`, every subsequent iteration can evaluate GTSAM's impact by comparing:
- Raw VIO trajectory (from relay logs) vs GTSAM-smoothed trajectory (from TF tree timestamps)
- Pointcloud fusion quality with and without GTSAM in the TF chain

## Changes Made

### 1. `config/prosthesis_config.yaml:323` — Stop relay from broadcasting raw VIO TF

**Before:**
```yaml
publish_dynamic_tf: true
```

**After:**
```yaml
publish_dynamic_tf: false
```

The relay continues to run its init guard, jump suppression, and static `*_imu -> *_cam0` publishing, but does NOT broadcast `marker_map -> *_imu` dynamic edges. This ownership is transferred to GTSAM.

### 2. `config/prosthesis_config.yaml:502` — Enable GTSAM TF broadcasting

**Before:**
```yaml
broadcast_tf: false
```

**After:**
```yaml
broadcast_tf: true
```

`gtsam_tracker_node.py:855-868` now calls `self._tf_broadcaster.sendTransform(tf_msg)` after each graph update, publishing the smoothed `marker_map -> head_imu` and `marker_map -> arm_imu` edges.

## Expected Impact

### Data flow change

**Before:**
```
OpenVINS odom → relay (init guard + jump suppress) → TF tree (raw VIO)
              → GTSAM (factor graph) → /gtsam/* topics only
```

**After:**
```
OpenVINS odom → relay (init guard + jump suppress only, no TF publish)
              → GTSAM (factor graph) → /gtsam/* topics + TF tree (smoothed)
```

### Goal 1 implications

Downstream nodes (pointcloud_fusion, keyframe_buffer) will now consume GTSAM-smoothed poses from the TF tree instead of raw VIO. This means:
1. Pointcloud fusion should produce denser, more consistent clouds (fewer outliers from VIO jumps)
2. Keyframe buffer uses smoother poses → better spatial gating
3. The `gtsam_corr` diagnostic now reflects real impact because GTSAM's correction reaches the physical pipeline

### Risks

1. **GTSAM startup transient**: If GTSAM takes time to converge its initial prior, the TF edges could have a small initial discontinuity. The prior factor uses the first odometry pose, so the first published TF edge matches VIO. The smoothing effect builds over the first ~7 seconds (smoother lag).

2. **Stale TF if GTSAM stalls**: If GTSAM's rate drops to 9 Hz (the rate decline), TF edges update at 9 Hz instead of 15 Hz. The nominal TF lookup timeout is `transform_tolerance_s: 0.15` (config line 269), which tolerates up to 150ms between updates. At 9 Hz (111ms period), this is still within tolerance.

3. **Crash impact difference**: Previously if GTSAM crashed, the relay still provided raw VIO TF. Now if GTSAM crashes, TF edges stop updating until the watchdog restarts the node. The `pipeline_diagnostics_node` would detect TF staleness and log warnings.

### What to check in the next iteration

1. **Verify `broadcast_tf=true` in startup log**: `GtsamTrackerNode ready (rate=15.0Hz, lag=15.0s, head_source=odom, broadcast_tf=True)`

2. **Verify `publish_dynamic_tf=false` in relay log**: `publish_dynamic_tf=False` in the relay startup line.

3. **Check for TF conflicts**: Look for "two publishers" or "conflicting transform" warnings in the log. The relay's code at line 431 returns early when `publish_dynamic_tf=false`, so no conflict should occur.

4. **Measure GTSAM correction via TF timestamps**: Compare the raw VIO trajectory (from relay's `_last_head_pos` before init) against the GTSAM-smoothed TF edges to quantify improvement.

5. **Watch the rate decline**: The fix doesn't change the 15s lag or ISAM2 parameters, so the 15.2→9.1 Hz decline should persist. This is a separate performance concern for a future iteration.
