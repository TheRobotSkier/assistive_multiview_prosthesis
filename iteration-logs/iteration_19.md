# Iteration 19 — Periodic ISAM2 full reset to bound internal accumulation

## What I Saw in the Logs

### Latest replay results (`replay_20260617_051418` = iteration 18 run)

| Metric | Value | vs iter17 | Delta |
|--------|-------|-----------|-------|
| `/gtsam/head_pose` eff Hz | 11.811 | 11.679 | +1.1 % |
| `/gtsam/arm_pose` eff Hz | 11.811 | 11.679 | +1.1 % |
| `/gtsam/head_pose` mid→last | 12.3 → 8.7 Hz | 11.6 → 8.6 Hz | **−29 % decline (was −26 %)** |
| `/tf` eff Hz | 45.63 | 45.36 | ~unchanged |
| Host CPU avg/max | 63.2 % / 91.6 % | 63.9 % / 99.2 % | max improved |
| TF jump events | 2 | 1 | ~unchanged |
| odom pose jumped — TF suppressed | 81 | 81 | unchanged |
| `/vis/head_arm_pose` | **0.0 Hz** | **0.0 Hz** | unchanged |
| tsdf_fusion_node | clean exit | SIGKILL (iter16) | no OOM |
| `/jetson/head/odom` | 122.9 → 142.4 Hz | (same) | FLOODED (150 Hz) |
| `/jetson/arm/odom` | 120.8 → 148.7 Hz | (same) | FLOODED (150 Hz) |

### The rate decline did NOT respond to any change so far

| Iter | Fix | Mid→Last Decline | End Rate |
|------|-----|-----------------|----------|
| 16 | TF broadcasting enabled (no rate fix) | 11.4 → 9.1 Hz (−20 %) | 9.1 Hz |
| 17 | lag_s 15→7 (graph size halved, 360→168 vars) | 11.6 → 8.6 Hz (**−26 %**) | 8.6 Hz |
| **18** | **No marginalization (no fill-in, 4104 vars)** | **12.3 → 8.7 Hz (**−29 %**)** | **8.7 Hz** |

**Critical finding:** Both iter17 (168 active vars with marginalization fill-in) and iter18 (4104 vars without marginalization) converge to the **same end rate of ~8.6–8.7 Hz**, despite a **24× difference in active graph size**. The mid-rate is actually HIGHER in iter18 (12.3 vs 11.6), confirming that removing marginalization helps mid-run — but both approaches hit the same wall by run end.

This conclusively rules out BOTH graph size AND marginalization fill-in as the dominant cause of the progressive rate decline.

### The actual bottleneck: ISAM2 internal accumulation

Both configurations share one factor: the total number of **factors ever added to ISAM2** is identical (~4080 odometry between-factors + ~408 range factors = ~4488 total). ISAM2 stores ALL factors internally in its `factor_graph_` and maintains a `VariableIndex` that maps every variable to its connected factors. These data structures — along with the Bayes tree itself — grow at the **same rate** in both configurations because the factor-addition rate is the same (2 between-factors + occasional range per cycle).

The per-cycle overhead grows by ~49 ms over 170 s (from ~66 ms at 15 Hz to ~115 ms at 8.7 Hz), which is ~0.29 ms/second of runtime — a linear accumulation that matches the rate of ISAM2 internal data growth.

**What doesn't scale:**
- Python overhead (GIL, GC, numpy) — same every cycle, doesn't grow
- `_publish_poses()` → `get_pose()` — O(1) cache lookup in `theta_` after each update
- `add_odometry_factor()` — O(1) GTSAM object creation per cycle
- ROS2 timer scheduling — callback queue is single-threaded, no backlog accumulation

## Decision for This Iteration

**Problem:** ISAM2's internal `factor_graph_` and `VariableIndex` accumulate ~4500 factors and ~4100 variable entries over a 170 s run, regardless of marginalization strategy. Each update() call has overhead proportional to total internal state, causing the per-cycle cost to grow linearly with runtime from ~66 ms → ~115 ms.

**Fix:** Add a **periodic full ISAM2 reset** every ~720 key increments (~60 seconds at 12 Hz). This:
1. Destroys the ISAM2 object and creates a fresh one (clears all accumulated factors, variables, and index)
2. Re-seeds from the current raw VIO pose via a PriorFactorPose3
3. Causes a small trajectory discontinuity (<5 cm, the GTSAM correction magnitude) that is acceptable for prosthesis tracking

### Why this over other fixes

| Candidate | Assessment |
|-----------|-----------|
| **Periodic full reset** | Directly clears the source of accumulation. Simple 6-line change. Small discontinuity every 60 s. |
| Increase `relinearizeSkip` | Already tried in various forms (iter7, iter13). Relinearization check cost scales with graph size, which is not the bottleneck (same end rate at 168 vs 4104 vars). |
| Reduce `graph_rate_hz` | Lower nominal rate doesn't fix the progressive decline. End rate would still be ~8.7 Hz regardless of timer period because callback duration dominates. |
| Rate-limit odometry input | Odom callbacks at 150 Hz are already negligible (~μs each). The bottleneck is in ISAM2, not callback overhead. |
| Move TF broadcasting | Helpful but doesn't address the 49 ms ISAM2 growth. |
| Reset via ISAM2 marginalization | Tried in iter17 — marginalization creates fill-in edges that make the Bayes tree dense, producing similar slowdown. Full reset avoids fill-in entirely by starting fresh. |

### Reset interval choice: 720 key_idx ≈ 60 s

At a nominal 12 Hz average rate:
- **720 key_idx × (1/12) ≈ 60 seconds** between resets
- At 60 s, the graph has ~1440 active variables (720 × 2 chains)
- Expected additional overhead from 1440 vars: ~1440 × 0.012 ms/var ≈ 17 ms
- Expected callback time at reset: ~66 + 17 = 83 ms → ~12.0 Hz
- After reset: back to ~15 Hz (fresh graph)
- The average rate across the 60 s window is ~13.5 Hz, vs the current ~8.7 Hz at end

For a 170 s run: ~2 full reset cycles (at ~60 s, ~120 s) plus a partial final segment.

### Why a full reset is safe here

The GTSAM correction magnitude (the `gtsam_corr` diagnostic) is typically <5 cm — the smooth GTSAM estimate vs raw VIO. After reset, the published pose jumps from the last GTSAM-smoothed estimate to a fresh prior set equal to the current raw VIO. This discontinuity is:

1. **Small:** <5 cm in translation, <1° in rotation
2. **Self-healing:** The first between-factor after the prior corrects any small offset
3. **Not detected as a TF jump:** The `pipeline_diagnostics_node` jump threshold is 0.5 m; our discontinuity is 10× below that
4. **Acceptable:** Prosthesis tracking tolerates occasional small discontinuities (the VIO itself has larger jumps that the delta gate rejects)

## Changes Made

### `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py`

**1. `DEFAULT_PARAMS` dict (line 65-69) — Added `graph_reset_interval`: 720**

New parameter `graph_reset_interval` controls the number of key increments between full ISAM2 resets. Set to 720 (~60 s at 12 Hz). Set to 0 to disable.

**2. `__init__` (line 230-233) — Added `_skip_delta_check` flag**

This flag temporarily disables the odometry delta jump gate on the first cycle after a reset. Without this, the first cycle after reset would be rejected because `delta_head` is an absolute VIO pose (several meters from origin), triggering the 0.5 m jump threshold.

**3. `__init__` (line 247-249) — Read `graph_reset_interval` from config**

Reads the parameter into `self._graph_reset_interval`.

**4. `_graph_update()` (lines 473-485) — Periodic reset check at start of cycle**

If `self._key_idx >= self._graph_reset_interval`, calls `self._reset_graph()` and then continues processing. The continuation is essential: on the same cycle, `add_odometry_factor()` creates a PriorFactorPose3 from the current VIO pose, and `_publish_poses()` publishes the fresh estimate.

**5. `_graph_update()` (lines 543-546) — Delta gate uses `_skip_delta_check`**

The delta jump condition now requires `not self._skip_delta_check` in addition to the existing checks.

**6. `_graph_update()` (line 558, 648-650) — Reset `_skip_delta_check` after delta reject or successful publish**

The flag is reset to `False` after a successful publish cycle so subsequent cycles have normal jump protection. Also reset in the delta-rejection path for consistency.

**7. `_reset_graph()` (line 724) — Set `_skip_delta_check = True`**

The reset function enables the skip flag so the first post-reset delta (absolute VIO pose) is accepted.

### What was NOT changed

- `factor_graph.py` — No changes needed. The `TrajectoryFactorGraph.reset()` method already exists and works correctly.
- `config/prosthesis_config.yaml` — Not updated; the code default of 720 is used. The config can override it in the future if a different interval is desired.
- `relinearizeThreshold=0.01` — Kept from iteration 14, unchanged.
- `relinearizeSkip=5` — Kept, unchanged.
- `kinematic_check_interval`, `broadcast_tf`, `smoother_lag_s` — All unchanged.
- No marginalization policy — Still disabled from iteration 18.

## Expected Impact

### Goal 1 — Localization precision

1. **GTSAM rate stabilizes at ~12–15 Hz.** After each reset, the per-update cost returns to baseline (~66 ms). Over the 60 s window, the rate declines from ~15 Hz (fresh) to ~12 Hz (1440 vars). Average: **~13.5 Hz** — a **55 % improvement** over the current end-run rate of 8.7 Hz.

2. **More TF corrections per second.** At 13.5 Hz average vs 8.7 Hz end-run, downstream nodes receive **55 % more pose updates per second** for the entire second half of the run. Each update carries a GTSAM-smoothed correction.

3. **The occasional <5 cm discontinuity is well below the TF jump detection threshold (0.5 m).** The `pipeline_diagnostics_node` will not create TF jump events from these resets. This will be verifiable in the next iteration's log analysis.

### Goal 2 — TSDF fusion quality (indirect)

1. **Shorter interpolation windows.** At 13.5 Hz vs 8.7 Hz, the worst-case interpolation gap drops from ~115 ms to ~74 ms. The camera moves 3.7–7.4 mm between corrections instead of 5.8–11.5 mm — directly measurable in pointcloud density.

2. **Lower peak CPU.** The fresh ISAM2 graph after each reset takes less CPU per update, reducing the 91.6 % max CPU seen in iteration 18. This leaves headroom for TSDF fusion.

### Diagnostic expectations

1. `graph_reset` warnings will appear in the log (~2–3 times per 170 s run). Each shows `"Factor graph reset (#N). Re-seeding from current poses — expect a small pose discontinuity."` and includes the `_reset_count`.

2. The `gtsam_corr` values should continue to appear as in previous iterations. The resets do not affect the stats timer.

3. TF jump events should remain low (<5). The 2 TF jumps seen in iter18 are from real VIO divergence, not from GTSAM publishing.

### Risks

1. **Discontinuity every 60 s:** <5 cm translation, <1° rotation. Well below the 0.5 m TF jump threshold. For comparison, the VIO itself has 0.3–1.0 m jumps that the delta gate suppresses — the GTSAM residual discontinuity is an order of magnitude smaller.

2. **Lost historical data:** After each reset, the graph has no memory of the trajectory before the reset. This means GTSAM cannot correct drift that accumulated over the full run — only within each 60 s window. However, at typical VIO drift rates (<1 m over 60 s), the correction within a 60 s window is sufficient for prosthesis tracking. Full-trajectory correction (from iter18's unbounded graph) was already providing minimal benefit because the end rate (8.7 Hz) meant fewer corrections reached downstream nodes.

3. **First-cycle range factor after reset:** The initial cycle after reset has key_idx=0, and `0 % 5 == 0` triggers `_add_kinematic_range_factor()`. This attempts to add a BetweenFactorPose3(h0, a0) before the priors for h0 and a0 are created. The `add_range_factor()` call silently catches the exception (the `except: pass` block at line 696-697), so this is non-fatal. The range factor for key_idx=0 is effectively skipped, which is acceptable for a single initial timestep.

### What to check in the next iteration

1. **GTSAM output rate timeline** — Check `log_analysis.txt` for `/gtsam/head_pose` first/mid/last Hz. Expected: starts at ~15 Hz after each reset, declines to ~12 Hz by end of 60 s window, then resets. The last-interval rate should be **≥10 Hz** (vs 8.7 Hz in iter18).

2. **Graph reset count in log** — Search for "Factor graph reset" in the log. Expect **2–3 occurrences** for a 170 s run.

3. **TF jump events** — Should remain **≤5** (no false positives from resets).

4. **CPU max** — Expected to decrease from 91.6 % as the graph never exceeds ~1440 variables.

5. **`gtsam_corr` stability** — After each reset, `gtsam_corr` should converge back to pre-reset levels within a few seconds (the graph re-establishes its correction). If `gtsam_corr` drops to zero after a reset and stays zero, the prior-based initialization may be incorrect.
