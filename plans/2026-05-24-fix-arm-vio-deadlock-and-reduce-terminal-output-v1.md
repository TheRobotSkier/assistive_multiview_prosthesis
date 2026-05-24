# Fix Arm VIO Deadlock + Reduce Jetson Terminal Output

## Objective

Fix the arm camera VIO deadlock where ArUco markers are detected but corrections never reach the EKF, and reduce the Jetson terminal output to prevent scrollback truncation.

## Background (v20 findings)

The v20 run shows:
- **Head camera (marker-6):** Rock solid at dist=0.90m, 269 ArUco corrections accepted, bg converging normally
- **Arm camera (marker-7):** Drifting 240m→811m, bg/ba **completely frozen** (never change), ArUco detects 38-48 markers per 5s window but `corrections=0/0` — the correction function is never called
- **Fusion:** Working — TF gate opens at 16s, 28-58 point clouds published per window, dual-camera fusion active
- **Bbox removal:** Still broken (bbox_removed=0 entire run)

The arm ArUco node detects markers but they all fail quality checks before reaching `try_apply_marker_correction`. The diagnostic shows `corrections=0/0` with zero rejection reasons — meaning `try_apply_marker_correction` is never called because `fixed_candidates` is empty (all markers filtered out by quality gates in `build_marker_measurement`).

## Implementation Plan

### Part 1: Reduce Jetson Terminal Output (Jetson config, 5 min)

- [ ] **1.1** Change `verbosity: "ALL"` to `verbosity: "WARNING"` in both OpenVINS estimator configs:
  - `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/estimator_config.yaml:3`
  - `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/estimator_config.yaml:3`
  
  This eliminates the ~7 lines of state output per VIO iteration (~30-60 Hz × 2 cameras = ~60-120 lines/second). The ArUco node's `[DIAG]` lines (every 5s) will still be visible for debugging.

### Part 2: Add Diagnostic Logging to Arm ArUco Quality Gate (Jetson code, 30 min)

- [ ] **2.1** In `aruco_marker_pose_node.py`, add rejection reason logging to the `_on_image` callback (around lines 945-993). Currently, when `fixed_candidates` is empty, the function returns at line 993 without logging WHY markers were rejected. Add a diagnostic counter for each rejection reason so the `[DIAG]` output includes quality gate rejections:
  - `unknown_marker_id`
  - `duplicate_marker_id`
  - `marker_area_too_small`
  - `max_marker_distance_exceeded`
  - `marker_translation_jump`
  - `marker_rotation_jump`
  - `border_margin`
  - `corner_angle`
  - `reprojection_error`
  - `geometry_score`

  The `[DIAG]` line should include these as: `detection_rejections: marker_translation_jump=35, max_marker_distance_exceeded=8`

  This is critical — without it, we're flying blind on why the arm never applies corrections.

### Part 3: Relax Arm ArUco Quality Gates (Jetson config, 10 min)

- [ ] **3.1** In `arm_aruco_map.yaml`, increase `max_marker_translation_jump_m` from `0.20` to `0.50`. The arm camera sees the marker from a different angle and distance than the head. The current 0.20m threshold may be too tight for the arm's noisier detections, especially when VIO is drifting and the marker-to-IMU transform varies.

- [ ] **3.2** In `arm_aruco_map.yaml`, increase `max_marker_rotation_jump_deg` from `15.0` to `30.0`. Same rationale — the arm camera's view angle changes more rapidly.

- [ ] **3.3** In `arm_aruco_map.yaml`, increase `max_marker_distance_m` from `2.0` to `3.0`. The arm camera may be physically further from the marker than the head camera.

### Part 4: Allow Reanchor Even When Markers Fail Quality (Jetson code, 30 min)

- [ ] **4.1** In `aruco_marker_pose_node.py`, modify the `_on_image` callback so that when `fixed_candidates` is empty BUT markers were detected (`markers_detected > 0`), the node still attempts a "best effort" correction using the least-bad candidate. Specifically:
  - After the quality-filtered `fixed_candidates` check at line 982, if empty but `markers_detected > 0`, collect the raw detections that failed quality into a `fallback_candidates` list
  - Sort by reprojection error (best first)
  - If any fallback candidate has `stable=True` (passed temporal stability check), attempt `try_apply_marker_correction` with it, but log a warning that quality gates were bypassed
  - This ensures that when VIO is drifting and quality checks are too strict, the ArUco system can still attempt a rescue correction

  Rationale: The quality gates exist to prevent bad corrections when VIO is healthy. But when VIO is drifting uncontrollably (dist=240m+), ANY correction is better than none. The `vio_invalid_hard_reanchor` path in `try_apply_marker_correction` (line 1687-1713) is specifically designed for this case — it applies a hard reset of the global frame. The quality gates are preventing this rescue mechanism from ever engaging.

### Part 5: Investigate Arm VIO EKF Freeze (Jetson investigation, 1-2 hrs)

- [ ] **5.1** The arm VIO bg/ba are completely frozen across the entire v20 run. This means the MSCKF EKF is propagating but never performing measurement updates. Investigate:
  - Check if the arm camera is providing images to the VIO pipeline (it must be, since ArUco detects markers)
  - Check if MSCKF features are being tracked (look for feature tracking logs when verbosity is still ALL)
  - Check if the EKF is rejecting all feature updates due to chi2 gating
  - Consider if the arm camera's IMU data is corrupted or delayed

  This is a deeper investigation that may require a dedicated debug run with `verbosity: "ALL"` temporarily re-enabled for the arm only.

## Verification Criteria

- [ ] Jetson terminal output is reduced to <10 lines/second (down from ~60-120)
- [ ] `[DIAG]` lines show detection rejection reasons for the arm camera
- [ ] Arm ArUco corrections are attempted (`corrections > 0/0`) even when VIO is INVALID
- [ ] If arm VIO is frozen, the reanchor mechanism engages and applies a hard correction
- [ ] Fused point cloud continues to publish with both cameras contributing

## Potential Risks and Mitigations

1. **Reduced verbosity hides VIO problems**
   Mitigation: The `[DIAG]` output from the ArUco node (every 5s) already shows `pos_norm`, `vio` health, and correction counts. The OpenVINS state output is only needed for deep debugging and can be re-enabled temporarily.

2. **Relaxed quality gates allow bad corrections**
   Mitigation: The `try_apply_marker_correction` function has its own internal checks (chi2 gate, innovation check). The quality gates are a pre-filter; relaxing them lets more candidates through, but the correction function still validates them.

3. **Fallback corrections could destabilize a healthy VIO**
   Mitigation: The fallback path only activates when `fixed_candidates` is empty (all markers failed quality) AND `vio_valid=False`. When VIO is healthy, the normal quality-gated path is used.

4. **Arm VIO EKF freeze may be a hardware issue**
   Mitigation: If the arm IMU is providing corrupted data, no software fix will help. The investigation in Part 5 will determine if this is a software or hardware issue.

## Alternative Approaches

1. **Disable arm VIO entirely and use only head VIO + static arm-to-head transform**: Would eliminate the arm drift problem but loses the arm camera's contribution to fusion. The arm camera provides a different viewpoint that improves scene coverage.

2. **Increase OpenVINS `max_clones` and `max_msckf_in_update` for the arm**: The arm may need a larger sliding window to track features from its more dynamic viewpoint. Current values (8 clones, 25 features) are conservative for Jetson CPU.

3. **Use catadioptric odometry for the arm instead of OpenVINS**: The system already has a working catadioptric pipeline. If the arm VIO is fundamentally unreliable, replacing it with the catadioptric odometry would be more robust.
