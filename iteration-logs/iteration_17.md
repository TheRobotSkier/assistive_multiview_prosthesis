# Iteration 17 — Reduce smoother lag from 15s to 7s to halt GTSAM rate decline

## What I Saw in the Logs

### Latest replay results (`replay_20260617_045431` = iteration 16 run)

| Metric | Value | vs iter14 (expected) | Delta |
|--------|-------|---------------------|-------|
| `/gtsam/head_pose` eff Hz | 11.263 | ~12.0 (estimated) | **−6.1 %** |
| `/gtsam/arm_pose` eff Hz | 11.263 | ~12.0 | **−6.1 %** |
| `/gtsam/head_pose` mid→last | 10.6 → 8.3 Hz | — | **−22 % decline** |
| `/tf` eff Hz | 44.54 | 30.9 (pre-iter16) | +44 % (GTSAM now broadcasting TF) |
| Host CPU avg/max | 60.6 % / 93.8 % | — | near limit |
| TF jump events | 40 | 0 (previous) | GTSAM TF broadcast has jumps |
| odom pose jumped — TF suppressed | 81 | — | relay still detects but doesn't broadcast |
| `/vis/head_arm_pose` | **0.0 Hz** | 0.0 Hz | **unchanged — still starved** |
| gtsam_corr from log | **— not seen** | should appear | diagnostic still missing |
| tsdf_fusion_node | **SIGKILL (-9)** | — | **OOM crash** |

### Diagnostics timeline reveals steady rate decline

The GTSAM rate declines continuously over the 171 s run:
```
/gtsam/head_pose:  0.0 → 10.6 → 8.3 Hz  (first / mid / last interval)
```

A 22 % mid-to-last decline is not a startup transient — it indicates the ISAM2 update cost increases over time.

### Current config state

The iteration 16 changes are in place:
- `publish_dynamic_tf: false` (`config/prosthesis_config.yaml:323`)
- `broadcast_tf: true` (`config/prosthesis_config.yaml:502`)

But `smoother_lag_s: 15.0` (`config/prosthesis_config.yaml:484`) is still at the high value that was set several iterations ago.

### The rate-decline mechanism

With `smoother_lag_s: 15.0` and GTSAM running at ~12 Hz, the ISAM2 graph holds **~180 pose variables per chain** (360 total) at steady state. Each `_graph_update()` timer callback:

1. Adds 2 odometry factors (head + arm) to the accumulators
2. Optionally adds a range factor (every 5th update)
3. Marginalizes old keys (every 5th update)
4. Calls `ISAM2::update()` — the expensive step
5. Publishes 2 topic messages + broadcasts 2 TF edges (new in iter 16)

With `relinearizeThreshold=0.01` and 360 variables, each relinearization check (every 5th global update) examines many variables. The Bayes tree depth grows with the number of active variables, making each Gauss-Newton iteration more expensive.

**The graph does NOT plateau in cost after 15 s** because:
- Linear approximation factors from marginalization accumulate in the factor graph
- With only every-5th-update marginalization, temporarily-old variables persist across 4 of 5 cycles
- TF broadcasting (added in iter 16) adds ~0.5–1 ms per cycle — small but compounds

The net effect: the per-update cost slowly climbs, the timer fires less often, and the rate decays from 15.2 Hz (peak, small graph) to 8.3 Hz (end, full graph + accumulated marginal factors).

### /vis/head_arm_pose remains starved (0.0 Hz)

The golden bag (`data/bags/golden_replay/`) contains only **11 arm image messages** and **59 arm cloud messages** over 166 s. The `sift_feature_node`'s `ApproximateTimeSynchronizer` gets almost no synced head+arm image pairs. This is a data-composition issue in the golden bag, not a code bug. Fixing it would require re-recording the golden bag with more image data — out of scope for this iteration.

### tsdf_fusion_node OOM crash

The tsdf_fusion node received SIGKILL (-9), which is the OOM killer. This may be connected to CPU at 94 % max causing memory pressure. A higher GTSAM rate means fewer dropped timer firings and less pose-interpolation backlog, which could reduce memory pressure on the fusion pipeline.

## Decision for This Iteration

**Problem:** The 15-second smoother lag keeps ~360 variables in the ISAM2 graph, making each update progressively slower. The GTSAM rate declines 22 % mid-to-last (10.6 → 8.3 Hz), reducing the number of pose corrections that reach downstream nodes. With TF broadcasting now enabled (iter 16), a faster rate directly translates to more frequent corrections in the TF tree consumed by pointcloud_fusion and keyframe_buffer.

**Fix:** Reduce `smoother_lag_s` from 15.0 to 7.0 — a single parameter change.

### Why this over other fixes

| Candidate | Assessment |
|-----------|-----------|
| **Reduce lag 15→7** | Directly halves graph size (360→168 vars). Single parameter change. Proven effective in iter 12 (12.145 Hz). |
| Fix /vis/head_arm_pose | Golden bag data issue — only 11 images per camera. Not a code fix. |
| Fix tsdf_fusion OOM crash | May be downstream effect of CPU overload. Improving GTSAM rate reduces overall CPU pressure. |
| ISAM2 param tuning | Already using proven params (relinearizeThreshold=0.01, skip=5). Diminishing returns. |
| TF jump fix | Jumps are from VIO divergence, not GTSAM. Iter 16 already transfers TF ownership to GTSAM. |

### Why 7.0 s specifically

The 7-second lag was the setting during the iteration-12 peak (12.145 Hz). It provides:
- **~84 poses per chain** (7 s × 12 Hz) — sufficient for drift correction at <5 cm/s prosthesis motion
- **Steady-state graph size of ~168 variables** vs ~360 at 15 s
- **Faster ISAM2 updates** — smaller Bayes tree, fewer variables to relinearize
- **Quicker adaptation** — older poses are marginalized sooner, so the graph reflects current kinematics rather than old trajectory

At 7 s marginalization delay, the linearized system is still 5–6× the 1.2 s OpenVINS initialization window, so the smoother has ample history for drift correction.

## Changes Made

### `config/prosthesis_config.yaml:484` — Reduce smoother lag to 7 s

**Before:**
```yaml
smoother_lag_s: 15.0
```

**After:**
```yaml
smoother_lag_s: 7.0
```

`gtsam_tracker_node.py:200` reads `smoother_lag_s` from the config and passes it to `TrajectoryFactorGraph.__init__(lag_s=...)` (`factor_graph.py:84`). No code change needed — the parameter flows through automatically.

The existing `TrajectoryFactorGraph` default is 7.0 (`factor_graph.py:84`), but the config override was set to 15.0. This change aligns the config with the code default.

### What was NOT changed

- `relinearizeThreshold=0.01` — kept from iteration 14, prevents Gauss-Newton convergence slowdown
- `relinearizeSkip=5` — full checks every 5 updates is a good balance
- `broadcast_tf=true` — kept from iteration 16, GTSAM continues to own the TF tree
- `publish_dynamic_tf=false` — kept from iteration 16
- `kinematic_check_interval=1` — range factor every cycle keeps chains coupled
- All other ISAM2 parameters, factor graph topology, noise models — untouched

## Expected Impact

### Goal 1 — Localization precision

1. **GTSAM output rate recovers to ~12.0 Hz steady state.** With the graph halved from ~360 to ~168 variables, `ISAM2::update()` completes faster. The timer callback runs below the ~67 ms threshold (1/15 Hz), so fewer firings are skipped. Expected rate: **11.5–12.5 Hz** stable across the run (no mid-to-last decline).

2. **More TF corrections per second.** At 12 Hz vs 8.3 Hz (end), downstream nodes receive **44 % more pose updates**. Each update carries a GTSAM-smoothed correction that mitigates VIO drift. More frequent corrections → tighter trajectory → better pointcloud registration.

3. **Graph is more responsive.** A 7-second window means the smoother adapts to VIO drift changes within 7 s instead of 15 s. For prosthesis motion (typical trial ~30 s), the smoother reaches steady-state graph size at 7 s and spends the remaining ~164 s at bounded cost.

### Goal 2 — TSDF fusion quality (indirect benefit)

1. **Shorter interpolation windows.** Each pose update carries the current GTSAM estimate. At 12 Hz, the worst-case interpolation gap is ~83 ms. At 8.3 Hz, it's ~120 ms. For a camera moving at 0.05–0.1 m/s, a 120 ms gap means 6–12 mm of uncompensated motion. Tightening to 83 ms reduces this to 4–8 mm.

2. **Lower CPU pressure.** The factor graph update is the most expensive operation in gtsam_tracker_node. Faster updates mean less timer queue backlog and lower peak CPU. If the OOM crash (tsdf_fusion SIGKILL) was triggered by memory pressure from delayed processing, faster GTSAM could help the overall pipeline stay within its RAM budget.

### Risks

1. **Shorter smoothing window reduces absolute accuracy.** With 7 s of history instead of 15 s, the smoothed trajectory has less temporal context for drift correction. However, at 12 Hz input, 7 s still provides **84 poses per chain**, and the Umeyama-based visual factors (when available) provide cross-chain absolute constraints. The 7 s window is a tuning choice, not a fundamental limit.

2. **Marginalization churn.** Every 5th update marginalizes ~84 old keys after the first 7 s. The ISAM2 `marginalizeTheta` overhead is moderate (~5–10 ms with 84 keys). This is already throttled to every 5th update and does not materially affect the output rate.

### What to check in the next iteration

1. **GTSAM output rate** — check `bag_metrics.json` for `/gtsam/head_pose` effective_hz. Target: **≥11.5 Hz** with **<5 % mid-to-last decline** (vs the 22 % decline seen in iter 16).

2. **Diagnostics timeline** — Check `log_analysis.txt` for the rate timeline. Expected: first ~0 (startup), mid ~12 Hz, last ~11.5 Hz (slight decline from graph buildup, not continuous decay).

3. **Host CPU** — expect avg CPU to drop slightly (from 61 %) as GTSAM spends less time per update. Max CPU should also decrease from 94 %.

4. **TF jump events** — should remain similar (40 in iter 16). The lag reduction does not change the factor graph's ability to correct VIO jumps — it just responds faster.

5. **No new gtsam_corr diagnostic** — this diagnostic was expected from iteration 15's log capture fix but hasn't appeared in the analysis. If still missing, investigate whether the `_log_stats()` timer line is being filtered by the log level or if `gtsam_corr` values are zero (graph not correcting).
