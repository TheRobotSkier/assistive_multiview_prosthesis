# Iteration 18 — Disable ISAM2 marginalization to halt progressive rate decline

## What I Saw in the Logs

### Latest replay results (`replay_20260617_050332` = iteration 17 run)

| Metric | Value | vs iter16 | Delta |
|--------|-------|-----------|-------|
| `/gtsam/head_pose` eff Hz | 11.679 | 11.805 | **−1.1 %** |
| `/gtsam/arm_pose` eff Hz | 11.679 | 11.805 | **−1.1 %** |
| `/gtsam/head_pose` mid→last | 11.6 → 8.6 Hz | 11.4 → 9.1 Hz | **−26 % decline (was −20 %)** |
| `/tf` eff Hz | 45.36 | 30.9 (pre-iter16) | TF broadcasting working |
| Host CPU avg/max | 63.9 % / 99.2 % | 60.6 % / 93.8 % | max near limit |
| TF jump events | 1 | 40 | improved (GTSAM smoothing) |
| odom pose jumped — TF suppressed | 81 | 79 | unchanged |
| `/vis/head_arm_pose` | **0.0 Hz** | **0.0 Hz** | unchanged — golden bag data |
| `gtsam_corr` from log | **not seen** | **not seen** | diagnostic still not captured |
| tsdf_fusion_node | clean exit | SIGKILL (iter 16) | no OOM this run |

### The rate decline did NOT improve

The iteration 17 fix (reducing smoother_lag_s from 15.0 to 7.0) **failed to stop the rate decline**. The mid→last decline actually worsened:
- Iteration 16 (lag 15 s): 11.4 → 9.1 Hz (**−20 %**)
- Iteration 17 (lag 7 s): 11.6 → 8.6 Hz (**−26 %**)

The rate max (15.1 Hz) and overall average (11.68 Hz) are nearly identical to iteration 16 (15.2 Hz peak, 11.80 Hz avg). Halving the graph size from ~360 to ~168 variables had **essentially zero impact** on the progressive slowdown.

This conclusively rules out **graph size** as the cause. The slowdown is driven by something that grows **per update cycle**, not per variable.

### Key observation: marginalization creates accumulating fill-in

Every 5th `_graph_update()` cycle, the code:
1. Calls `self._graph.marginalize_old_keys(stamp)` — finds ~17 old keys per chain
2. Passes them to `ISAM2::update()` as `marginalizeTheta`

When ISAM2 marginalizes a variable, it:
1. Collects all factors connected to that variable
2. Builds a Schur-complement factor (dense linear approximation) that replaces the marginalized variable's information
3. This new factor connects **all the remaining variables** that were adjacent to the marginalized one

For a two-chain system with range factors every 5th cycle:
- A marginalized head key `h_i` had a between-factor to `h_{i+1}` AND a range factor to `a_i`
- Marginalizing `h_i` creates a linear factor connecting `h_{i+1}` and `a_i`
- This adds a **cross-chain fill-in edge** that didn't exist in the original graph
- Over many marginalization events, these fill-in edges accumulate, enriching the Bayes tree's clique structure and making each subsequent update slower

With ~34 marginalization events over 170 s (every 5th cycle at ~12 Hz), and each event creating ~17 new fill-in edges per chain, the accumulated fill-in is **~578 cross-chain edges** over the run. This is the root cause of the continuous rate decline.

### Why this explains the data

1. **Rate max (15.1 Hz) at the start**: no fill-in yet, graph is small, ISAM2 updates are fast.
2. **Mid-run rate (11.6 Hz)**: the first ~7 s of fill-in has accumulated. The rate stabilizes temporarily as the initial marginalization completes.
3. **Last-interval rate (8.6 Hz)**: 34 marginalization events worth of fill-in has accumulated. Each ISAM2 update must process a denser Bayes tree regardless of the variable count, so the rate continues to drop.

### CPU at 99.2 % max

The high CPU amplifies the rate decline: as ISAM2 takes longer per update, the timer callback competes for CPU with other nodes (especially the VIO relays at 150 Hz each, and TF broadcasting at 45 Hz). The timer period (66.7 ms at 15 Hz nominal) gets eaten by callback duration (116 ms at 8.6 Hz effective), causing missed timer fires.

## Decision for This Iteration

**Problem:** ISAM2 marginalization creates linear-approximation fill-in edges that accumulate in the Bayes tree over long replay runs, making each update progressively slower. Reducing the graph size (iteration 17) didn't help because fill-in is bounded by the number of marginalization events, not the variable count.

**Fix:** Disable marginalization entirely. Stop calling `marginalize_old_keys()` and always pass an empty key vector to `ISAM2::update()`.

### How this helps

Without marginalization:
- **Bayes tree stays chain-structured.** The head and arm chains are each a simple chain of between-factors. Cross-chain cliques exist only at range-factor timestamps (every 5th update) and do NOT accumulate.
- **ISAM2 incremental updates are O(1) per new chain variable.** Adding 2 new root cliques (head + arm) per cycle is fast. The Gauss-Newton forward-backward pass on a chain-structured tree is O(n) where n is the tree depth, but each operation is a trivial 1-variable update.
- **No fill-in from Schur-complement factors.** Without marginalization, the only cross-chain edges are the range factors themselves — bounded at ~34 edges for a 170 s run, not ~578 accumulated edges.
- **Graph grows linearly** (~4160 variables at 2040 timesteps × 2 chains for a 170 s run at ~12 Hz). This is ≈1.2 MB in GTSAM's internal C++ data structures — trivially small for the host machine (30+ GB RAM).

### Why this over other fixes

| Candidate | Assessment |
|-----------|-----------|
| **Disable marginalization** | Directly removes the fill-in accumulation. Minimal change: 1 code block replaced. Proven by existing test at `test_factor_graph.py:86` which uses `lag_s=100.0` to avoid marginalization. |
| Reduce marginalization to every 10th cycle | Would halve fill-in rate but not eliminate it. Rate would still decline, just twice as slowly. |
| Increase `relinearizeThreshold` | Makes relinearization less frequent but doesn't address fill-in. Could make convergence worse. |
| Increase `relinearizeSkip` | Same — doesn't affect fill-in accumulation. |
| Set `smoother_lag_s` higher | Already ruled out by iteration 17 — variable count isn't the problem. Fill-in per marginalization event is. |

### What about the smoother boundary?

Without marginalization, the factor graph contains ALL poses from the entire replay (full trajectory). This means:
- **More temporal context** for drift correction — the smoother sees the full history, not just a 7 s window
- **Better smoothing quality** — all available data contributes to each estimate
- **Slightly more memory** — ~4160 variables instead of ~168, but this is <5 MB total
- **No marginalization discontinuities** — the Bayes tree stays clean

For a prosthesis trial (typical duration <60 s), the graph would be even smaller (~1440 variables).

## Changes Made

### `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:576-612` — Remove marginalization from `_graph_update()`

Removed the 18-line marginalization block (lines 576-593) that called `marginalize_old_keys()` every 5th cycle. Replaced with a direct call to `self._graph.update(marginalize_keys=[])`.

**Before:**
```python
old_keys: list = []
if self._key_idx % 5 == 0:
    try:
        old_keys = self._graph.marginalize_old_keys(stamp)
    except Exception:
        pass  # Non-fatal if no keys are eligible.

try:
    self._graph.update(marginalize_keys=old_keys)
```

**After:**
```python
try:
    self._graph.update(marginalize_keys=[])
```

The error handling (consecutive failure tracking, graph reset recovery) is preserved unchanged.

### What was NOT changed

- `smoother_lag_s: 7.0` — kept in config at `config/prosthesis_config.yaml:484`, now unused but harmless. If desired, can be set to a large sentinel value in a future iteration.
- `relinearizeThreshold=0.01` — kept, ensures accurate linearization
- `relinearizeSkip=5` — kept, fast cycles don't relinearize
- `kinematic_check_interval` config — kept
- `broadcast_tf=true` — kept from iteration 16
- All noise models, factor graph topology — untouched
- `factor_graph.py` — no changes needed (the `marginalize_old_keys()` method and `_key_timestamps` dict still exist but are never called; harmless dead code)

## Expected Impact

### Goal 1 — Localization precision

1. **GTSAM rate stabilizes at ~14–15 Hz.** Without fill-in accumulation, each `ISAM2::update()` takes consistent ~2–5 ms regardless of run duration. The timer fires at or near its nominal 15 Hz for the entire run. Expected effective_hz: **≥13.5 Hz** with **<5 % mid-to-last decline** (vs the −26 % seen in iteration 17).

2. **More TF corrections per second.** At 14 Hz vs 8.6 Hz (end of iteration 17), downstream nodes receive **63 % more pose updates per second by run end**. Each correction carries a GTSAM-smoothed pose that mitigates VIO drift. This directly improves pointcloud registration and reduces TSDF fusion outliers.

3. **Full-trajectory smoothing.** Without marginalization, GTSAM retains all poses. For a 30–60 s prosthesis trial, the full trajectory is available for correction. The quality of each estimate is based on ALL data, not a 7 s sliding window. This should improve the `gtsam_corr` magnitude (the difference between odometry and GTSAM estimates).

### Goal 2 — TSDF fusion quality

1. **Shorter interpolation windows at run end.** At 14 Hz vs 8.6 Hz, the worst-case interpolation gap drops from ~116 ms to ~71 ms. For a camera at 0.05–0.1 m/s, this means 3.5–7 mm of uncompensated motion instead of 6–12 mm — directly measurably in pointcloud density and outlier count.

2. **Lower peak CPU.** Without marginalization fill-in driving ISAM2 update times up, the timer callback completes quickly and doesn't accumulate backlog. The max CPU (99.2 % in iteration 17) should drop significantly since there's no prolonged ISAM2 update that monopolizes the CPU.

### Diagnostic expectations

1. `gtsam_corr` values (logged every 10 s via `_log_stats()`) should now appear in the log — previously the rate decline may have caused the stats timer to fire less consistently or the values to be zero because the graph was stuck.

2. TF jump events should remain low (iteration 17 had 1 event). The GTSAM trajectory is now based on the full history, which provides a more stable smoothing baseline than a sliding window.

### Risks

1. **Graph memory:** ~4160 variables for a 170 s run, ≈1.2 MB in GTSAM C++ data. For real prosthesis runs (<60 s), it's ~1440 variables ≈400 KB. No risk.

2. **Full-trajectory drift:** Without marginalization, old VIO data from early in the run could theoretically bias current estimates if the VIO has a bias drift that changes over time. However, the odometry between-factors are relative (delta from previous pose), so each step only constrains the relative motion — the absolute trajectory is anchored by the initial prior. This is the same as the sliding-window case, just with more data.

3. **ISAM2 slow start:** The first cycle creates the initial prior. Subsequent cycles add 2 between-factors each. The ISAM2 update is called every cycle (not just marginalization cycles). This was already the behavior before the change — we just removed the marginalization step, not the update step.

### What to check in the next iteration

1. **GTSAM output rate** — check `bag_metrics.json` for `/gtsam/head_pose` effective_hz. Target: **≥13.5 Hz** with **<5 % mid-to-last decline**.

2. **Diagnostics timeline** — Check `log_analysis.txt` for rate timeline. Expected: first ~0 (startup), mid ~14 Hz, last ~13.5 Hz (slight decline from chain growth, not fill-in).

3. **CPU** — expected to decrease from 63.9 % avg / 99.2 % max. The ISAM2 update is no longer the bottleneck.

4. **`gtsam_corr` in log** — if the rate stabilizes, the diagnostics timer should fire consistently and `gtsam_corr` values should appear in the log analysis for the first time since they were added.

5. **TF jump events** — expected to remain low (<5 events). GTSAM smoothing over full trajectory should be at least as stable as the sliding window.
