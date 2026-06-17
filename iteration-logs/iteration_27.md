# Iteration 27 — Fix config YAML `tf_jump_threshold_m` (was still 0.05, overriding code's 0.20)

## What was observed in the logs

### Latest replay results (`logs/replay-results/replay_20260617_070409/`)

The pipeline is stable (14 nodes, GTSAM at 14.5 Hz, TF at 50.9 Hz). Key metrics:

| Known Failure | Iter 25 | Iter 26 | Iter 27 (before fix) | Notes |
|---|---|---|---|---|
| TF jump events | 2961 | 2961 | **2939** | Barely changed from iter25→26 |
| odom pose jumped — TF suppressed | 81 | 81 | 81 | Same |
| gtsam odometry rejected | 2 | 2 | 2 | Same |
| tsdf GetAllKeyframes timeout | 3 | 1 | 1 | Improved (was 15 in iter24) |
| TF chain disconnected | 18 | 17 | 17 | Same |

### Critical finding — Config YAML still has `tf_jump_threshold_m: 0.05`

**This is the root cause of 2939 TF jumps persisting across iterations 25 and 26.**

The pipeline_diagnostics node receives its parameters from `config/prosthesis_config.yaml` via the launch file:

```
pipeline.launch.py:365-367:
    parameters=[_node_params(config, "pipeline_diagnostics")]
```

Where `_node_params` extracts `config["pipeline_diagnostics"]["ros__parameters"]` (line 56).

The YAML at `config/prosthesis_config.yaml:445`:
```yaml
        tf_jump_threshold_m: 0.05
```

This OVERRIDES the code default in `pipeline_diagnostics_node.py:164`:
```python
"tf_jump_threshold_m": 0.20,   # ← iteration-26 change
```

Because `declare_parameter(key, default)` at line 202-203 uses the default only when the config doesn't provide a value. Since the launch file passes the YAML value, the effective threshold has always been **5 cm**, never 20 cm.

**Why iteration 26's fix appeared ineffective:**
- Iteration 26 changed the code default from `0.05` to `0.20` (line 164)
- But the config YAML at line 445 was NOT updated, remaining at `0.05`
- The config overrides the code default, so the threshold stayed at 5 cm
- The TF jump count dropped from 2961 to 2939 (only 22) — this is just run-to-run measurement noise, NOT an effect of the threshold change

**Why 5 cm threshold causes 2939 jumps:**
At 14.5 Hz GTSAM output (~69 ms between consecutive TF publishes for the same edge), normal prosthesis motion produces:

| Motion speed | Per-cycle delta | Exceeds 5 cm? | Exceeds 20 cm? |
|---|---|---|---|
| Slow (0.25 m/s) | 1.7 cm | No | No |
| Moderate (0.5 m/s) | 3.5 cm | No | No |
| Brisk (1.0 m/s) | 6.9 cm | **YES** | No |
| Fast (2.0 m/s) | 13.8 cm | **YES** | No |
| Extreme (3.0 m/s) | 20.7 cm | **YES** | **YES** |

With a 5 cm threshold, even brisk 1.0 m/s arm motion triggers a TF jump on every cycle. This explains why ~60% (2939/4892) of all TF publishes are flagged — they're real, normal motion, not VIO glitches.

The genuine VIO discontinuities (>0.5 m) are already tracked independently by the `odom pose jumped — TF suppressed` metric (81 per run) through the relay's delta gate at `max_pose_jump_m: 0.50`.

## What was decided to work on

**Problem**: Config YAML value `tf_jump_threshold_m: 0.05` overrides the intended `0.20` from code, causing 2939 false TF jump detections from normal prosthesis motion.

**Fix**: Update `config/prosthesis_config.yaml:445` from `0.05` to `0.20` to match the code default and the intended behavior.

**Rationale**: At 20 cm threshold, only motion >2.9 m/s in 69 ms triggers a jump — physically impossible for a hand/arm prosthesis. The 81 genuine VIO glitches >0.5 m (already caught by the relay's delta gate) survive the 20 cm gate and remain visible for diagnostics.

## What was changed

### `config/prosthesis_config.yaml:445`

**Before:**
```yaml
        tf_jump_threshold_m: 0.05
```

**After:**
```yaml
        tf_jump_threshold_m: 0.20
```

Aligns the config YAML with the code default at `pipeline_diagnostics_node.py:164`. The launch file passes parameters from YAML to the node (line 367), so the YAML value was overriding the code default. Now both agree at 0.20 m.

## Expected impact

### Goal 1 — Localization precision (GTSAM factor-graph optimization)

1. **TF jump count drops from ~2939 to <200**: With a 20 cm threshold, only genuine VIO discontinuities >20 cm per 69 ms appear. The 81 VIO glitches >0.5 m (already counted by the relay) plus a smaller number of 0.2–0.5 m sub-glitches that slip past GTSAM's 0.5 m delta gate are expected to produce <200 jumps per run.

2. **TF tree diagnostic becomes meaningful**: Instead of fire-hosing the log with 2939 "jump" warnings for normal motion, the diagnostic flags rare, genuinely anomalous events. The `peak burst: 151 in 5s` drops to near-zero.

3. **No change to GTSAM optimization**: The factor graph, noise models, and output poses are untouched. Only the monitoring threshold changes. GTSAM still runs at 14.5 Hz with the same between-factor + range-factor structure.

### Goal 2 — TSDF fusion quality (indirect)

1. **No direct change**: The TSDF and keyframe_buffer continue using the same GTSAM-smoothed poses for cloud alignment. However, with fewer TF "jump" warnings flooding the log, operator attention can focus on real issues like TSDF timeout counts and VIO glitch patterns.

2. **Keyframe_buffer spatial gating unaffected**: The spatial gate uses TF lookups, not the TF jump detector. The actual TF tree stability (monotonic, non-jumping publishes) was already fixed in iteration 25 (publish key_idx instead of key_idx-1). Any residual TF inconsistency is from VIO discontinuities, not from the monitoring threshold.

### Diagnostic checks for the next iteration

1. **TF jump count** (`tf_jump_events_total` in `log_metrics.json`): Expect **<200** (was 2939).
2. **TF jump peak burst** (`peak burst: N jumps in 5s window`): Expect **<10** (was 151).
3. **All other metrics unchanged**: GTSAM Hz (~14.5), TF Hz (~50.9), odom jumps suppressed (81), GTSAM odometry rejected (2), TSDF timeouts.
4. **`/keyframe_buffer/diagnostics`** still at 0.2 Hz — the timer was changed to 1.0s in iteration 26 code, but the test results haven't reflected this. Container rebuild may be needed for that fix to take effect.

### Risks

1. **Genuine VIO oscillations of 10–19 cm per cycle are no longer flagged**: This is acceptable because (a) such sub-threshold motion is within the normal prosthesis envelope (<2.5 m/s), (b) the separate `odom pose jumped — TF suppressed` metric (threshold 0.5 m) catches the severe VIO glitches, and (c) the GTSAM 7-second smoother filters out high-frequency VIO noise regardless of the monitoring threshold.

2. **No change to GTSAM vs VIO tracking accuracy**: Without visual factors in the bag, GTSAM still reproduces the VIO trajectory exactly (chain of between-factors has unique MAP solution = chain of measurements). The TF jump fix only affects monitoring, not the actual trajectory.
