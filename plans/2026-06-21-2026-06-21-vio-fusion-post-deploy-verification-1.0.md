# VIO Fusion Fix — Post-Deployment Verification & Remaining Issues Plan

## Objective

Verify the Phase 1–4 changes from plan 4.0, identify which were deployed vs not,
diagnose remaining issues blocking stable OpenVINS/TF/fused pointclouds, and
provide a prioritized action plan to stabilize the system.

## Deployment Verification Summary

### Host-side changes (DEPLOYED ✓)

| Phase | Change | Evidence |
|-------|--------|----------|
| 2.1 | spin_thread=True | Relay log shows no deadlock; corrected TF lookups succeed |
| 2.2 | tf_publish_max_hz=20Hz | `host-log:33`: "tf_publish_max_hz=20 Hz" |
| 2.3 | Lookup timeout 0.15s | Corrected TF lookups succeed within timeout |
| 2.4 | Cache max age 4.0s | Config deployed |
| 2.5 | Decoupled lookup from rate limiter | LPF/cache update runs on every message |
| 3.1 | transform_tolerance_s=1.0s | TF lookups succeed with 1.0s timeout |
| 3.2 | Skew-corrected timestamp fallback | Code deployed |
| 3.3 | Dual-view ratio metric | `host-log:423`: "Dual-view fusion degraded" warnings present |
| 4.1 | DIAG-PC parser fix | Analysis report shows correct rates (3.4Hz, 4.6Hz) |
| 4.2 | Fusion dual-view extraction | Analysis report shows dual-view ratios |

### Jetson-side changes (NOT DEPLOYED ✗)

| Phase | Change | Evidence of absence |
|-------|--------|---------------------|
| 1.1-1.3 | IPPE_SQUARE + temporal disambiguation | 241 rotation_hard rejections at ~110° (same pre-fix pattern) |
| 1.4 | Adaptive chi2 gate (recovery mode) | Zero "recovery" strings in Jetson log |
| 1.5 | calib_cam_timeoffset normalization | Cannot verify from log alone |

**Root cause**: The Jetson Docker image was not rebuilt after the code changes.
The running container still uses the old `aruco_marker_pose_node.py` with
`SOLVEPNP_ITERATIVE` and no adaptive chi2 gate.

## Remaining Issues Analysis

### Issue 1: Arm VIO trapped in solvePnP ambiguity flip (CRITICAL — blocks all arm data)

**Symptom**: 241 `rotation_hard` rejections, all arm side, tightly clustered at
~110° (range 108.5°–112.4°). Arm correction yield: 0.2% (1/462 markers).

**Root cause**: `SOLVEPNP_ITERATIVE` without extrinsic guess deterministically
returns the flipped pose solution once VIO has diverged. The 90° rotation wall
correctly blocks it, but with no escape hatch the arm is permanently trapped.

**Fix status**: Phase 1.1-1.4 code is written and verified but NOT DEPLOYED.
Rebuilding the Jetson Docker image will deploy:
- IPPE_SQUARE with corner reordering (verified zero-error in synthetic tests)
- Temporal disambiguation (picks solution closest to previous frame)
- Seeded iterative fallback
- Adaptive chi2 gate (5× wider when VIO invalid + diverged)

**Action**: Rebuild Jetson Docker image and re-run.

### Issue 2: Corrected-TF oscillation causing 2214 TF jump warnings (HIGH)

**Symptom**: `marker_map -> head_imu_openvins_corrected` jumps 0.758m
constantly. `marker_map -> arm_imu_openvins_corrected` jumps 6.754m constantly.
2214 total TF jump warnings from pipeline_diagnostics.

**Root cause**: These are UPSTREAM frames published by the Jetson-side
`aruco_marker_pose_node.py`, NOT by the host relay. The host relay's LPF
(alpha=0.3) smooths its rebroadcast (marker_map -> head_imu shows only 147
jumps vs 1043 on the corrected frame). The upstream oscillation is caused by:
1. Head: ArUco reanchor creates a step discontinuity every correction cycle
   (~1Hz), and the corrected frame alternates between pre- and post-reanchor
   positions
2. Arm: VIO is catastrophically diverged (pos_norm varies wildly), and the
   fallback reanchor path force-applies corrections that immediately get
   overwritten by diverging VIO on the next frame

**Impact on fusion**: The relay's LPF-smoothed rebroadcast (marker_map ->
*_imu) is what the fusion node actually uses. The 147 head + 145 arm jumps on
these edges are the real concern — each 0.7m jump causes a transient
misalignment in the fused pointcloud.

**Mitigation**:
- Deploying Phase 1 (IPPE fix) will eliminate the arm's 6.754m oscillation
- The head's 0.758m oscillation is the ArUco reanchor step; the LPF reduces
  this to ~0.227m on first frame, which is still above the 0.20m diagnostic
  threshold. Lowering `corrected_tf_lpf_alpha` to 0.15 would reduce first-frame
  jump to 0.114m.
- Excluding `*_openvins_corrected` edges from pipeline_diagnostics jump
  warnings would eliminate 87% of the noise (1922/2214 warnings)

### Issue 3: TF "extrapolation into the future" failures (MEDIUM)

**Symptom**: Fusion node logs "Lookup would require extrapolation into the
future. Requested time X but latest data is at time Y" where X > Y by ~0.2-1.2s.

**Root cause**: The fusion node looks up TF at the cloud's header stamp (sensor
capture time on Jetson). The relay publishes corrected TF stamped with the odom
message's header stamp. But there's a timing mismatch:
- Depth cloud header stamp = depth capture time
- Odom header stamp = VIO observation time
- These differ by up to ~1 frame interval (~200ms at 5Hz)
- When the cloud arrives BEFORE the matching odom/TF (transport jitter), the
  TF buffer doesn't yet have a transform at the cloud's timestamp

The 1.0s timeout helps but doesn't fully solve it because the relay's 20Hz rate
limiter means TF is only published every 50ms, while clouds arrive at 3.5Hz
(~285ms apart). If the cloud's stamp is between two TF publishes and the newer
one hasn't arrived yet, the lookup fails.

**Fix approach**: The fusion node should fall back to `rclpy.time.Time()` (latest
available) when the header-stamp lookup fails, instead of skipping the cloud
entirely. A slightly-stale transform is better than no transform for
pointcloud fusion.

### Issue 4: Dual-view ratio only 33.7% (MEDIUM)

**Symptom**: 86 published clouds, 32 dual-view, 63 cam1_only. First 4 intervals
had 0% dual (TF chain disconnected during startup). Later intervals: 35%, 50%,
26%, 29%.

**Root cause**: Two factors:
1. TF failures (Issue 3) cause one cloud to be skipped, resulting in
   cam1_only publication
2. The arm's corrected TF is jumping 6.754m, causing the arm cloud's transform
   to be wildly wrong even when it succeeds — the distance filter then removes
   most arm points, effectively producing a head-only cloud

**Fix**: Deploying Phase 1 will fix the arm TF stability. Fixing Issue 3 will
reduce TF lookup failures.

### Issue 5: pipeline_diagnostics noise (LOW)

**Symptom**: 2215 WARN lines from pipeline_diagnostics_node, mostly TF jump
warnings on corrected frames that the host cannot control.

**Fix**: Add an exclusion list for `*_openvins_corrected` edges in the jump
detector, or raise the jump threshold for corrected edges.

## Implementation Plan

- [ ] **Task 1**: Rebuild Jetson Docker image to deploy Phase 1 changes
  - This is the single highest-impact action — it deploys the IPPE_SQUARE fix,
    temporal disambiguation, seeded fallback, and adaptive chi2 gate
  - Without this, the arm VIO will remain trapped in the flip state

- [ ] **Task 2**: Add TF lookup fallback to latest-available in fusion node
  - When header-stamp lookup fails with "extrapolation into future", retry
    with `rclpy.time.Time()` (zero/latest) before skipping the cloud
  - This eliminates the "cam1_only" publications caused by TF timing races

- [ ] **Task 3**: Lower corrected_tf_lpf_alpha from 0.3 to 0.15 in config
  - Reduces first-frame LPF jump from 0.227m to 0.114m (below 0.20m threshold)
  - Settling time increases from ~3 frames to ~7 frames (~0.35s at 20Hz)

- [ ] **Task 4**: Exclude *_openvins_corrected edges from pipeline_diagnostics
  jump warnings
  - These are upstream Jetson-published frames; the host cannot fix them
  - Eliminates 87% of the 2214 TF jump warnings

- [ ] **Task 5**: Add rotation LPF to relay's corrected-TF rebroadcast
  - Currently only translation is filtered; rotation discontinuities pass
    through unfiltered
  - Apply SLERP-based smoothing with the same alpha as translation

## Verification Criteria

- Arm VIO correction yield > 10% (currently 0.2%)
- Zero `rotation_hard` rejections after IPPE deployment
- Dual-view ratio > 50% sustained (currently 33.7%)
- TF "extrapolation into future" failures reduced by > 80%
- TF jump warnings on relay-rebroadcast edges < 50 total (currently 292)
- Fused pointcloud published at > 2 Hz sustained

## Potential Risks and Mitigations

1. **IPPE_SQUARE may not be available on Jetson OpenCV 4.6.0**
   Mitigation: The code has a runtime check (`hasattr(cv2, 'solvePnPGeneric')`)
   and falls back to the seeded iterative solver. The fallback is still better
   than the old from-scratch iterative solve.

2. **Lower LPF alpha adds latency to corrected TF**
   Mitigation: 0.35s settling time is acceptable for 3.5Hz cloud fusion. The
   alternative (0.758m jumps) is worse for fusion quality.

3. **Latest-available TF fallback may use stale transforms**
   Mitigation: The cloud_max_age gate (1.0s) already limits how old a cloud can
   be. A transform from ~200ms ago is well within the acceptable range for
   pointcloud fusion at these distances.
