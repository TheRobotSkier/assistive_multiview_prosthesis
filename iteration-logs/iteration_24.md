# Iteration 24 — Smooth GTSAM trajectory by increasing odometry noise sigmas

## What was observed in the logs

### Latest replay results (`logs/replay-results/replay_20260617_062826/`)

The pipeline is now fully operational after iteration 23's build fix:

| Topic | Effective Hz | Health | vs Iter22 (dead) | Delta |
|-------|-------------|--------|-------------------|-------|
| `/gtsam/head_pose` | **14.4 Hz** | OK | 0.0 Hz | ✅ Restored |
| `/gtsam/arm_pose` | **14.4 Hz** | OK | 0.0 Hz | ✅ Restored |
| `/tf` | **50.7 Hz** | OK | 22.0 Hz | ✅ Restored (GTSAM TF) |
| `/keyframe_buffer/diagnostics` | **0.2 Hz** | STARVED | 0.2 Hz | same (ongoing issue) |
| `/vis/head_arm_pose` | **0.0 Hz** | STARVED | 0.0 Hz | same (no images in bag) |

### Critical finding — 2990 TF jump events

The `pipeline_diagnostics` node (enabled via `debug_monitor:=true`) detected **2990 TF jump events** — by far the most severe known-failure metric:

```
[2990x] TF jump (pipeline_diagnostics)  [04:28:43 .. 04:31:26]
    hint: VIO/odometry discontinuity — check OpenVINS stability & clock drift
    sample: {'edge': 'marker_map -> arm_imu', 'jump_m': 0.676, 'stamp': ...}
    sample: {'edge': 'marker_map -> head_imu', 'jump_m': 0.595, 'stamp': ...}
```

Breakdown: 1743 on `marker_map → head_imu`, 1247 on `marker_map → arm_imu`.

Additional findings:
- **17x** TF chain disconnected (OpenVINS→RealSense bridge incomplete)
- **81x** `odom pose jumped — TF suppressed` (>0.5 m VIO jumps caught by relay)
- **15x** `tsdf GetAllKeyframes timeout` (keyframe_buffer service hanging)
- **2x** `gtsam odometry rejected` (VIO deltas >0.5 m caught by GTSAM delta gate)
- VIO odometry topics FLOODED at 100+ Hz vs 30 Hz nominal (`/jetson/head/odom` 105-165 Hz, `/jetson/arm/odom` 107-154 Hz)

### Root cause confirmed from host log

The host log (`host-log-20260617_042828.txt:33-42`) definitively shows:

```
[openvins_odom_tf_relay] OpenVINS odometry TF relay active:
    publish_dynamic_tf=False, use_corrected_tf=True
[gtsam_tracker_node] TF broadcasting enabled (marker_map → head_imu, arm_imu)
```

The relay is correctly configured NOT to publish dynamic TF (`publish_dynamic_tf=False` from `config/prosthesis_config.yaml:323`). GTSAM is the **sole** publisher of the `marker_map → head_imu` and `marker_map → arm_imu` TF edges (via `broadcast_tf=True`).

Therefore, the 2990 TF jumps are **internal to GTSAM** — the smoothed trajectory oscillates between consecutive 15 Hz updates with magnitudes of 0.5–0.7 m, because:

1. GTSAM's odometry noise was very tight (0.02 m translation σ, 0.02 rad rotation σ)
2. With only odometry factors in the graph (no images → no visual factors at 0 Hz, no ArUco topics in bag), GTSAM closely tracks raw VIO
3. Raw VIO has frequent discontinuities (81 suppressed >0.5 m, many more smaller ones at 100+ Hz flood rate)
4. Every 67 ms GTSAM re-optimizes and broadcasts a pose that may differ from the previous broadcast by 0.5+ m when VIO jumps

This directly harms both goals: the TF tree oscillates (bad for localization precision), and the keyframe_buffer/tsdf_fusion get inconsistent poses (bad for TSDF pointcloud quality).

## What was decided to work on

**Problem**: GTSAM tracks VIO too aggressively (0.02 m odometry σ), propagating VIO discontinuities into the smoothed trajectory and causing 2990 TF jumps with 0.5–0.7 m magnitudes.

**Fix**: Increase odometry noise sigmas 5× (0.02 → 0.10 m translation, 0.02 → 0.10 rad rotation) so the 7-second ISAM2 smoother window filters VIO noise rather than tracking every discontinuity. The trajectory becomes smoother and more temporally consistent, at the cost of slightly looser VIO tracking.

**Rationale**: With the 7-second lag smoother and only odometry factors (no visual/ArUco constraints), the factor graph is a chain of BetweenFactors. Tight noise (0.02 m) means every VIO sample is treated as nearly ground truth — the optimizer has no freedom to reject outliers. Increasing noise to 0.10 m gives the optimizer room to distribute corrections across the smoothing window, producing a C0-continuous trajectory that filters high-frequency VIO jumps while maintaining ~10 cm accuracy to the mean VIO trajectory.

## What was changed

### 1. `config/prosthesis_config.yaml:489-496` — Increased GTSAM odometry noise sigmas

**Before:**
```yaml
# Noise sigmas (used when odom covariance is zero/invalid)
odom_translation_sigma_m: 0.02
odom_rotation_sigma_rad: 0.02
```

**After:**
```yaml
# Noise sigmas (used when odom covariance is zero/invalid).
# Increased from 0.02 to 0.10 (5x) so GTSAM trusts VIO less
# aggressively and produces a smoother trajectory, filtering
# VIO discontinuities (iteration-24: 2990 TF jumps from tight
# tracking of noisy VIO).  The 7-second smoother lag maintains
# adequate tracking accuracy while suppressing VIO noise.
odom_translation_sigma_m: 0.10
odom_rotation_sigma_rad: 0.10
```

### No other changes

This is the single parameter change. The `gtsam_tracker_node.py` code default (0.02 at line 75-76) is overridden at runtime by the config file passed through `_node_params(config, "gtsam_tracker")` in `pipeline.launch.py:388`.

## Expected impact

### Goal 1 — Localization precision (GTSAM factor-graph optimization)

1. **TF jump count drops dramatically**: From 2990 to an expected <200–400. Each VIO discontinuity will be absorbed by the looser noise model rather than propagated into the TF tree. The TF tree becomes temporally consistent, making downstream TF lookups (keyframe_buffer spatial gating, pointcloud fusion) more reliable.

2. **GTSAM → odom correction magnitude more stable**: The `gtsam_corr` stat (logged every 10 s in `_log_stats`) will show smaller variation between consecutive reads. Currently, each VIO jump forces a large correction; after this fix, corrections will be smoother and more gradual.

3. **Moderate increase in raw tracking error**: GTSAM's trajectory will deviate from raw VIO by up to ~10 cm (vs ~2 cm previously). This is acceptable because: (a) the 7 s lag window means the mean trajectory still tracks VIO accurately, (b) the trajectory becomes C0-continuous instead of jumping by 0.5+ m, and (c) downstream consumers (keyframe_buffer, TSDF) prefer consistent poses over precise-but-jittery ones.

### Goal 2 — TSDF fusion quality (indirect)

1. **Spatial gating in keyframe_buffer becomes more effective**: With smoother GTSAM poses, consecutive keyframes are inserted at more consistent spatial intervals rather than being gated by transient jumps. This means fewer spurious keyframes during VIO glitches and better coverage during stable motion.

2. **Fewer GetAllKeyframes timeouts**: The TSDF node's `GetAllKeyframes` service calls (15 timeouts in this run) may decrease if the keyframe_buffer produces more consistent responses. The current timeouts are likely caused by the buffer spending time on inconsistent pose lookups triggered by TF jumps.

3. **Tighter fused pointcloud**: TSDF integration aligns each keyframe's pointcloud using the GTSAM-smoothed pose. With a smoother pose trajectory, the voxel ray-casting sees less jitter between consecutive integrations, producing a denser, less noisy fused volume.

### Diagnostic checks for the next iteration

1. **TF jump count**: Check `log_metrics.json` → `tf_jump_events_total`. Expect <500 (down from 2990).
2. **GTSAM correction magnitude**: Search the host log for `gtsam_corr` — expect `head_corr` and `arm_corr` to be more consistent between 10 s intervals (not oscillating by 0.5+ m).
3. **Keyframe buffer health**: Check `/keyframe_buffer/diagnostics` Hz and `GetAllKeyframes timeout` count. Both should improve.
4. **GTSAM output Hz**: `/gtsam/head_pose` and `/gtsam/arm_pose` should remain at ~14–15 Hz (no performance regression from looser noise).

### Risks

1. **GTSAM tracking lag**: Looser noise = more smoothing = ~100 ms additional effective lag. This is well within the 7-second smoothing window and does not affect the steady-state output rate. The published poses will lag behind true motion by at most ~200 ms total (was ~100 ms previously) — acceptable for prosthesis tracking (human reaction time is ~200–300 ms).

2. **Kinematic range factor interaction**: With looser noise on both chains, the range factor (1.0 m, 0.05 m σ) becomes relatively tighter compared to the odometry factors. This may cause the head and arm trajectories to converge more aggressively. If the two cameras are physically >1.0 m apart, the range constraint will bias both trajectories. This is acceptable since the kinematic range is a known physical constraint of the prosthesis.
