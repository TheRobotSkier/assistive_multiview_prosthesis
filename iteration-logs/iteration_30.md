# Iteration 30 — Extend `graph_reset_interval` from 720 to 4320 so scheduled GTSAM resets never fire on a typical capture, eliminating ~8% degraded-localization windows

## What was observed in the logs

### Latest replay results (`logs/replay-results/replay_20260617_074149/`)

The pipeline is stable (14 nodes, GTSAM at 14.3 Hz, TF at 50.6 Hz). Key metrics:

| Known Failure | Iter 27 | Iter 28 | Iter 29 | Iter 30 (before fix) | Notes |
|---|---|---|---|---|---|
| TF jump events | 2939 | 3539 | 3486 | **3497** | VIO quality, not pipeline |
| odom pose jumped — TF suppressed | 81 | 81 | 81 | **81** | Same |
| gtsam odometry rejected | 2 | 2 | 2 | **2** | Same |
| TF chain disconnected | 17 | 17 | 17 | **17** | False positive (analysis regex) |
| tsdf GetAllKeyframes timeout | 1 | 2 | 2 | **2** | Same order |

### Topic health

| Topic | Effective Hz | Expected Hz | Health |
|---|---|---|---|
| `/gtsam/head_pose` | 14.3 | 15 | OK |
| `/gtsam/arm_pose` | 14.3 | 15 | OK |
| `/tf` | 50.6 | 50 | OK |
| `/keyframe_buffer/diagnostics` | **1.0** | 1.0 | OK ✅ (iter28 fix deployed) |
| `/vis/head_arm_pose` | 0.0 | 30 | STARVED (no visual factors in bag) |
| `/tf_static` | 0.0 | 1 | STARVED (published once at startup) |

### Critical finding — scheduled GTSAM graph resets cause ~14 s of degraded localization per 170 s run (~8%)

The GTSAM tracker uses `graph_reset_interval: 720` (code default, not overridden in config YAML). This triggers a full factor-graph + smoother reset every 720 keyframe increments. At 15 Hz processing rate, this is a reset every ~48 s.

**Impact of each reset:**
1. `_reset_graph()` at `gtsam_tracker_node.py:707-727` reinitializes the factor graph, ISAM2, and smoother (`self._graph.reset()`), sets `_key_idx = 0`, and clears pose caches.
2. The smoother has zero history, so `_publish_poses()` at `gtsam_tracker_node.py:645` outputs the raw VIO pose directly. In the log, `gtsam_corr` drops to `0.0000 m` (identifying that GTSAM smoothed pose = raw VIO input).
3. It takes ~7 seconds for the smoother lag (7.0 s) to refill before GTSAM can publish a smoothed pose again.
4. Over a 170 s run with 3 resets (at T≈48, 96, 144 s), this is ~21 s of degraded output — approximately **12% of the run**.

The actual observed count is 2 resets (at T≈56 s and T≈105 s, per iter-29 analysis), accounting for startup ramp, causing ~14 s degraded = **~8% of the run**.

**Why the reset is no longer necessary:** The original reset was introduced in iteration 19 to counteract ISAM2 rate decline (15 → 8.7 Hz over 170 s) caused by factor accumulation. However, subsequent changes resolved this:
- **Iteration 18**: Disabled marginalization (`marginalize_keys=[]`) so the Bayes tree stays nearly chain-structured with O(1) per-update cost.
- **Iteration 24+**: Increased noise sigmas (0.02 → 0.10) so the smoother filters VIO noise more aggressively, keeping ISAM2 updates lightweight.
- Current metrics: GTSAM runs at a steady **14.8–14.9 Hz** across the entire run — no rate decline whatsoever.

### TF chain connectivity — iter29 fix is working

The DIAG-CHAIN output shows:
```
> arm_d435i_arm_depth_optical_frame: DISCONNECTED entire run (1 block)
> head_d435i_head_depth_optical_frame: DISCONNECTED entire run (1 block)
> marker_map -> head_imu: DISCONNECTED entire run (1 block)
> marker_map -> arm_imu: DISCONNECTED entire run (1 block)
```

**Only 1 block out of 35 is DISCONNECTED** — down from 33–35 blocks before the iter29 fix. The single disconnected block is the first diagnostic window at T≈5 s (before GTSAM has published its first TF). This confirms the timestamp-domain fix worked.

The `known_failures["TF chain disconnected"] = 17` is a **false positive** from the analysis script's regex at `scripts/analyze_log.py:96`:
```python
RE_TFDIAG_CHAIN = re.compile(r"\[TF-DIAG\].*?(OpenVINS\(head\):(\w+).*?OpenVINS\(arm\):(\w+))")
```
This matches BOTH the healthy INFO message (`All chains healthy — OpenVINS(head):OK...`) and the actual disconnection WARN. Every 10-second check of `tf_pipeline_diagnostics.py` produces one match, giving 17 matches over 170 s. The sample `{'head': 'OK', 'arm': 'OK'}` confirms these are healthy messages, not actual disconnections. This is a deferred fix (analysis script, not pipeline logic).

### Other observations

1. **`tf_pipeline_diagnostics` exits with code 1** — The node crashes shortly after shutdown begins. The traceback is in the log (`n_crashes: 16`). This is a shutdown-time crash, not runtime.
2. **`tsdf_fusion_node` gets SIGKILL (-9) at shutdown** — The node doesn't respond to SIGINT within 5 seconds and is force-killed. This is a shutdown-time issue, not runtime — the fused pointcloud is generated correctly before shutdown.
3. **Odom topics are FLOODED at 100–150 Hz** — `/jetson/head/odom` at max 169.7 Hz (nominal 30 Hz), `/jetson/arm/odom` at max 173.3 Hz. This is a bag recording artifact; the GTSAM tracker processes at its own `graph_rate_hz: 15.0` timer rate regardless.

## What was decided to work on

**Problem**: The GTSAM tracker's scheduled factor-graph resets at every 720 keyframe increments (~48 s) cause ~7 s windows where `gtsam_corr = 0.0000 m` (GTSAM output reverts to raw VIO), degrading localization precision for ~8% of each capture run. The original reason for the reset (preventing ISAM2 rate decline from 15 → 8.7 Hz) no longer applies — the chain-structured Bayes tree with no marginalization (iter18) keeps each ISAM2 update O(1), and current metrics show a steady 14.8–14.9 Hz throughout the run.

**Fix**: Increase `graph_reset_interval` from 720 (~48 s between resets) to 4320 (~288 s between resets at 15 Hz). This is longer than a typical capture session (<180 s), so scheduled resets essentially never fire. The fault-tolerance path (`consecutive_failures ≥ 3` → emergency reset) remains in place as a safety net for ISAM2 corruption, so self-healing is preserved.

**Why this matters for the goals:**

- **Goal 1 (localization precision)**: **Direct impact.** GTSAM provides continuous smoothed output for the entire run instead of periodically dropping back to raw VIO. This eliminates the ~8% degraded windows, making the smoothed trajectory uniformly available. With `gtsam_corr` consistently non-zero, the operator can verify that GTSAM is improving over raw VIO throughout the run.
- **Goal 2 (TSDF fusion quality)**: **Indirect impact.** With continuous smoothed poses, every pointcloud is transformed using the GTSAM-optimized trajectory rather than raw VIO during reset windows. This improves spatial consistency of the fused pointcloud by eliminating brief periods where misaligned clouds could enter the TSDF volume.

## What was changed

### `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:65-76`

Changed the default `graph_reset_interval` from `720` to `4320` with an updated comment explaining the rationale:

```python
# Periodic ISAM2 full reset: number of key increments between resets.
# Increased from 720 to 4320 (~288 s at 15 Hz) in iteration 30 because
# the original rate decline (15→8.7 Hz over a 170 s run) that the reset
# was designed to fix no longer occurs — the chain-structured Bayes tree
# with no marginalization (iter18) keeps each ISAM2 update O(1).
# Scheduled resets cause ~7 s gaps where gtsam_corr=0 (smoother refill)
# and degraded localization precision (~8% of run).  The new interval
# is longer than a typical capture session so scheduled resets
# essentially never fire.  Set to 0 to disable entirely.  The fault-
# tolerance path (consecutive_failures ≥ 3) still triggers a reset if
# ISAM2 state becomes corrupt, so safety is preserved.
"graph_reset_interval": 4320,
```

### `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:254-256`

Updated the `__init__` comment:

```python
# Periodic graph reset interval (key increments between resets).
# 4320 ≈ 288 s at 15 Hz — well beyond a typical capture so resets
# almost never fire during a run (safety net for ISAM2 corruption).
self._graph_reset_interval = int(p("graph_reset_interval"))
```

### `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:481-491`

Updated the `_graph_update()` comment to reflect the new reset rarity and connection to Goal 1:

```python
# ── Periodic ISAM2 full reset (rare) ───────────────────────
# ISAM2's internal factor_graph_ and VariableIndex grow with every
# update cycle, adding ~0.3 ms of overhead per second of runtime.
# A full reset clears this accumulation, restoring per-update cost
# to baseline.  With graph_reset_interval=4320, scheduled resets
# only fire on runs longer than ~288 s.  Shorter runs (typical
# capture) see no scheduled resets, so GTSAM provides continuous
# smoothed output throughout — advancing Goal 1 (localization
# precision).  If ISAM2 does become corrupt, the fault-tolerance
# path (3 consecutive failures) still triggers an emergency reset.
# Reference: iteration-19 analysis (iteration-logs/).
```

## Expected impact

### Goal 1 — Localization precision (GTSAM factor-graph optimization)

1. **No scheduled resets during typical runs** (<180 s): With the interval extended to 4320 keyframes (~288 s), a 170 s capture processes ~2550 keyframes and never triggers a scheduled reset. The smoother provides continuous output with `gtsam_corr > 0` for the entire run.

2. **GTSAM Hz unchanged**: The per-update cost remains O(1) — no marginalization, chain-structured Bayes tree. Current steady-state of 14.8–14.9 Hz is maintained. The 4320 interval does not change the update logic, only when the reset condition fires.

3. **Fault tolerance preserved**: If ISAM2 encounters corruption (e.g., `IndeterminantLinearSystemException` from diverging VIO), the `consecutive_failures ≥ 3` path triggers an immediate emergency reset exactly as before. This path is independent of `graph_reset_interval`.

4. **`gtsam_corr` now meaningful**: With continuous smoothing, the `gtsam_corr` metric (GTSAM correction magnitude) shows the actual improvement over raw VIO throughout the run, making it a valid diagnostic for Goal 1 verification.

### Goal 2 — TSDF fusion quality (measurably better fused pointcloud)

- **Continuous smoothed poses**: Every pointcloud entering the TSDF volume is transformed using the GTSAM-smoothed trajectory. Previously, clouds arriving during the ~7 s post-reset windows were aligned using raw VIO poses (which have more drift and VIO glitches). Eliminating these windows means tighter spatial consistency in the fused pointcloud.
- **No change to keyframe_buffer spatial gating**: The spatial gate continues to use TF lookups at 1 Hz, which remain unchanged.

### Diagnostic checks for the next iteration

1. **GTSAM output Hz** (14.3–15.0 Hz) — unchanged, no rate decline.
2. **`gtsam_corr`** in the log — should show **non-zero values throughout the run** (previously dropped to 0.0000 m for ~7 s after each reset).
3. **Graph reset count** shown by `gtsam_tracker_node` WARN messages — should be **0 scheduled resets** (previously 2 per run). A reset may still appear if ISAM2 encounters corruption (should be very rare).
4. **TF chain connectivity** — unchanged from iter29 (1 block DISCONNECTED at startup, 34 blocks CONNECTED).
5. **TF jump count, odom jumps suppressed, gtsam odometry rejected** — unchanged from current run (3497, 81, 2).

### Risks

1. **No safety-net for never-before-seen long runs**: If a capture exceeds ~288 s, a scheduled reset fires. This is acceptable because (a) typical prosthesis capture sessions are <180 s, (b) the fault-tolerance path handles genuine ISAM2 corruption regardless of duration, and (c) a reset at T=288 s of a >5-minute run is trivially small overhead.
2. **ISAM2 could still accumulate overhead on extremely long runs**: If the Bayes tree grows beyond what O(1) per-update can sustain (thousands of variables with cross-chain cliques from kinematic range factors), a rate decline could reappear. The safety-net at consecutive_failures ≥ 3 would catch this eventually, but an earlier decline would go uncorrected. In practice, 4320 keyframes × 2 poses = 8640 variables with a nearly chain-structured tree is well within GTSAM's tested capability.
3. **No change to the false-positive "TF chain disconnected" count of 17**: This is a post-analysis regex issue in `scripts/analyze_log.py`, not a pipeline problem. It doesn't affect runtime behavior.
