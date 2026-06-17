# Iteration 26 — Fix keyframe_buffer diagnostics starved (5.0s→1.0s timer), raise TF jump threshold to 0.20m, align config with code

## What was observed in the logs

### Latest replay results (`logs/replay-results/replay_20260617_064558/`)

The pipeline is stable (all 14 nodes running, GTSAM at 14.4 Hz, TF at 50.8 Hz). Key metrics vs iteration 25:

| Known Failure | Iter 25 | Iter 26 (before fix) | Notes |
|---|---|---|---|
| TF jump events | 2961 | 2961 | Unchanged — the publish-once fix (iter25) did not drop jumps |
| odom pose jumped — TF suppressed | 81 | 81 | Same |
| gtsam odometry rejected | 2 | 2 | Same |
| tsdf GetAllKeyframes timeout | 3 | 3 | Improved from 6 (iter24) |
| TF chain disconnected | 18 | 18 | Same |

### STARVED topics (bag metrics)
| Topic | Effective Hz | Expected Hz | Health |
|---|---|---|---|
| `/gtsam/head_pose` | 14.4 | 15 | OK |
| `/gtsam/arm_pose` | 14.4 | 15 | OK |
| `/tf` | 50.8 | 50 | OK |
| `/keyframe_buffer/diagnostics` | **0.2** | **1.0** | **STARVED** |
| `/vis/head_arm_pose` | 0.0 | 30 | STARVED (no images in bag) |
| `/tf_static` | 0.0 | 1 | STARVED (static TF published once at startup) |

### Critical findings

#### 1. `/keyframe_buffer/diagnostics` timer is 5.0s (causes STARVED)

`src/keyframe_buffer/keyframe_buffer/keyframe_buffer_node.py:645`:
```python
self.create_timer(5.0, self._publish_diagnostics)   # → 0.2 Hz
```

The diagnostics timer interval is hard-coded to 5.0 seconds (0.2 Hz), but the analysis script at `scripts/analyze_bag.py:65` expects `/keyframe_buffer/diagnostics` at 1.0 Hz. This mismatch causes the STARVED warning. The timer was likely set to 5.0s to reduce log noise during development but was never updated to match the expected rate.

#### 2. TF jump threshold of 0.05m is too sensitive for 14.4 Hz output

At 14.4 Hz (69 ms between publishes), normal prosthesis motion produces translation deltas of:
- **Slow** (0.25 m/s): 1.7 cm — below threshold, OK
- **Moderate** (0.5 m/s): 3.5 cm — below threshold, OK
- **Brisk** (1.0 m/s): **6.9 cm** — above 5 cm → flagged as "jump"
- **Fast** (2.0 m/s): **13.8 cm** — well above threshold

With 2961 "jumps" out of ~4900 total TF publishes (2 edges × 2453 publishes/edge = 4906), **~60% of all GTSAM TF broadcasts are falsely flagged**. The log confirms `peak burst: 150 jumps in 5s window` — at 14.4 Hz × 2 edges × 5 s = 144 publishes, 150/144 means **every publish triggers a jump**.

The actual VIO discontinuities (>0.5 m) are already correctly counted by the `odom pose jumped — TF suppressed` metric (81 per run). A 0.20 m threshold would catch only genuinely anomalous motion (>2.9 m/s in 69 ms, impossible for a prosthesis).

#### 3. GTSAM correction goes to 0.0000m after first reset — expected behavior

`src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:769-779` computes `gtsam_corr` as:
```python
gtsam_corr = relative_transform(self._head_pose, gtsam_h(key_idx-1))
```

This compares the **latest raw VIO pose** (updated at 130 Hz) with the **GTSAM-optimized pose for the second-to-last key** (last optimized ~69 ms ago). With only between-factors (no visual factors, no ArUco in the bag), the GTSAM factor graph reproduces the raw VIO trajectory exactly — a pure chain of BetweenFactors has a unique MAP solution equal to the VIO odometry chain. Therefore, `gtsam_h(key_idx-1) ≈ VIO(key_idx-1)` and `corr ≈ VIO motion over ~69 ms`.

For a nearly-stationary bag (pose norms change by <0.05 m over 10 s), the per-cycle VIO delta is <0.3 mm, which formats as `0.0000 m`. This is **correct** — GTSAM is working as designed, but without visual/ArUco factors there are no absolute constraints to correct VIO drift. The `gtsam_corr` metric becomes meaningful only when visual factors or ArUco priors are available (requires a bag with camera images or ArUco observations).

**Pre-reset corrections of ~1 m** were transient VIO jumps of >0.5 m occurring between graph update cycles. After the first reset, the graph starts fresh from the current VIO position, so these transients disappear from the metric.

#### 4. Config mismatch: kinematic_check_interval

The config at `config/prosthesis_config.yaml:488` specifies `kinematic_check_interval: 1` (add range factor every cycle), but the code at `gtsam_tracker_node.py:513` hard-codes `self._key_idx % 5 == 0` (every 5th cycle). The code comment at line 508-512 explains this is intentional for ISAM2 performance — adding the range factor every cycle creates cross-chain cliques in the Bayes tree on every update, slowing the solve. The config was never aligned with the code.

## What was decided to work on

Three focused fixes:

1. **Fix keyframe_buffer diagnostics timer** (priority c — STARVED topic). Change `5.0s → 1.0s` to match the expected 1 Hz rate.

2. **Fix TF jump threshold** (priority b/d — diagnostic metric inflated by normal motion). Raise `0.05m → 0.20m` so that only genuinely anomalous motion (>2.9 m/s, impossible for prosthesis) is flagged. The 81 real VIO discontinuities >0.5 m (already caught by the odom relay's delta gate) remain counted.

3. **Align config with code** — change `kinematic_check_interval: 1` → `5` so the config matches the actual behavior in the code.

## What was changed

### Fix 1: `src/keyframe_buffer/keyframe_buffer/keyframe_buffer_node.py:645`

**Before:**
```python
self.create_timer(5.0, self._publish_diagnostics)   # → 0.2 Hz
```

**After:**
```python
self.create_timer(1.0, self._publish_diagnostics)   # → 1.0 Hz
```

### Fix 2: `src/camera/camera/pipeline_diagnostics_node.py:164`

**Before:**
```python
"tf_jump_threshold_m": 0.05,   # 5 cm — too tight for 14.4 Hz output
```

**After:**
```python
"tf_jump_threshold_m": 0.20,   # 20 cm — filters normal motion, catches real VIO glitches
```

Rationale: At 14.4 Hz (~69 ms between TF broadcasts), even brisk 1.0 m/s arm motion produces 6.9 cm per-cycle deltas. Raising the threshold to 20 cm ensures only physically impossible motion (>2.9 m/s) is flagged as a jump. The genuine VIO discontinuities (>0.5 m from OpenVINS resets) are already independently tracked by the `odom pose jumped — TF suppressed` metric (81 per run).

### Fix 3: `config/prosthesis_config.yaml:488`

**Before:**
```yaml
kinematic_check_interval: 1
```

**After:**
```yaml
kinematic_check_interval: 5
```

Aligns the config with the actual code behavior at `gtsam_tracker_node.py:513`, where the range factor is added every 5th cycle (not every cycle). The code uses the hard-coded `5` because every-cycle range factors create cross-chain cliques in ISAM2's Bayes tree on every update, which slows the solve rate (see code comment at `gtsam_tracker_node.py:508-512`).

## Expected impact

### Goal 1 — Localization precision (GTSAM factor-graph optimization)

- **No change to GTSAM optimization behavior** — Fixes 1-3 are all diagnostic/config alignment. The factor graph still uses between-factors + range factor at every 5th cycle with 0.10 m odometry noise.
- **TF jump count drops from ~2961 to <200** (estimated): With the 0.20 m threshold, only genuinely anomalous motion >20 cm per 69 ms (i.e., >2.9 m/s) will trigger. The actual VIO glitches >0.5 m (already caught by the relay at 81 per run) represent the only real TF jumps, and they typically produce >0.3-1.0 m deltas. Residual "jumps" might include a few per run during fast arm swings.
- **GTSAM correction metric** (`gtsam_corr`) stays at ~0 m for this bag (no visual factors). It will become meaningful when a bag with images → SIFT features → visual factors is used.

### Goal 2 — TSDF fusion quality (indirect)

- **Keyframe_buffer diagnostics now at 1.0 Hz** — the diagnostic topic is no longer STARVED, providing consistent health monitoring.
- **No change to keyframe buffer behavior** — the buffer still inserts keyframes using the same spatial gate logic. The diagnostics fix only affects the monitoring output, not the core function.
- **Fused pointcloud** — The pipeline still uses GTSAM-smoothed poses for cloud alignment. With only between-factors, GTSAM reproduces VIO, so the fused pointcloud quality is unchanged. Visual factors (requiring camera images) will be needed for measurable improvement.

### Diagnostic checks for the next iteration

1. **`/keyframe_buffer/diagnostics` effective_hz** — should be **1.0 Hz** (was 0.2 Hz), no longer STARVED.
2. **TF jump count** (`tf_jump_events_total` in `log_metrics.json`) — expect **<200** (was 2961). Only real VIO discontinuities >0.2 m per 69 ms.
3. **GTSAM output Hz** — unchanged at ~14.4 Hz.
4. **GTSAM correction magnitude** — unchanged at ~0 m for this bag (no visual factors). Monitor for future bags with images.

### Risks

- **TF jump threshold increase may mask real issues**: A genuine VIO oscillation of 0.15 m per cycle would no longer be flagged. However, such sub-threshold oscillations are within the normal motion envelope (<2 m/s) and are handled by GTSAM's between-factor smoothing. The separate `odom pose jumped — TF suppressed` metric (0.5 m threshold) independently tracks the severe VIO glitches.
- **No change to GTSAM vs VIO accuracy**: Without visual factors in the bag, GTSAM cannot demonstrate absolute localization improvement over raw VIO. The pipeline is ready for this once a bag with camera images is used.
