# Plan: Investigate and Fix ArUco Marker Non-Determinism

## Objective

The ArUco marker detection pipeline produces wildly inconsistent results between runs with no code changes:
- v12: 80+ EKF updates for marker-6 (arm), 0 for marker-7 (head)
- v13: 0 EKF updates for both markers
- v15: 341 EKF updates for both markers

This investigation identifies the root causes of the non-determinism and proposes fixes.

---

## Root Cause Analysis

### Finding 1: VIO Health Gate Creates a Positive Feedback Loop

**Severity: CRITICAL — this is the primary source of run-to-run variability**

The ArUco node's `try_apply_marker_correction()` at `aruco_marker_pose_node.py:1577-1732` has a VIO health check that creates a positive feedback loop:

1. OpenVINS starts with unconverged state → position drifts rapidly
2. VIO health check at `aruco_marker_pose_node.py:1544` sees `||position|| > 5.0m` → marks VIO invalid (`position_outside_workspace`)
3. When VIO is invalid, corrections go through `vio_invalid_hard_reanchor` mode (line 1642-1665)
4. But `reanchor_cooldown_s: 3.0` means after one reanchor, the system waits 3 seconds
5. During those 3 seconds, VIO continues drifting → position goes further outside workspace
6. Next correction attempt: VIO still invalid, cooldown expired, hard reanchor again
7. BUT: if the hard reanchor doesn't bring the position back within 5m (because VIO has drifted further in 3 seconds), the next health check immediately marks it invalid again
8. This creates a cycle where corrections are applied but can't keep up with drift

**The key insight:** Whether this loop converges or diverges depends on:
- How fast VIO drifts (which depends on bg convergence, which is non-deterministic)
- How far VIO has drifted before the first marker detection (which depends on startup timing)
- Whether the first correction brings the position within the 5m workspace

In v15, the ArUco corrections started early enough (before drift exceeded 5m) and the system converged. In v13, VIO drifted too far before the first marker was detected, and the system never recovered.

### Finding 2: `stable_frames_required: 8` Creates a Startup Delay

**Severity: HIGH — this amplifies the feedback loop**

The quality gate at `head_aruco_map.yaml:34` requires `stable_frames_required: 8` consecutive detections before accepting a marker measurement. At 15 Hz detection rate, this is ~0.5 seconds of stable detection before the first correction can be applied.

During those 0.5 seconds, VIO is drifting. If VIO drifts past 5m in that time, the first correction will be a VIO-invalid reanchor, which has a 3-second cooldown. During the cooldown, VIO drifts further. The system may never recover.

The stability check is at `aruco_marker_pose_node.py:1598`:
```python
if not measurement.stable:
    return False, "marker_not_stable"
```

### Finding 3: `max_position_norm_m: 5.0` Is Too Tight for a Drifting VIO

**Severity: HIGH — this is the threshold that triggers the feedback loop**

The VIO health check at `head_aruco_map.yaml:105` uses `max_position_norm_m: 5.0`. But the VIO can drift at 3-5 m/s, meaning it takes only 1-2 seconds of unconverged VIO to exceed this threshold. Once exceeded, corrections are forced into `vio_invalid_hard_reanchor` mode with its 3-second cooldown.

The logs show head VIO drifting to 588m (v12), 96m (v13), and the arm VIO drifting to 1.7m (v12). The 5m threshold is easily exceeded within the first few seconds.

### Finding 4: Odom Buffer Timeout Can Prevent First Correction

**Severity: MEDIUM — this is a timing race**

The `find_nearest_odom()` at `aruco_marker_pose_node.py:1520-1523` requires an odom message within `max_odom_match_dt: 0.05s` of the image timestamp. If the odom topic hasn't started publishing yet (VIO still initializing), or if there's a clock desync between the camera and IMU, the first marker detection won't find a matching odom message and the correction is rejected with `odom_match_timeout`.

### Finding 5: Detection Rate Limiting May Miss Early Windows

**Severity: LOW — contributory**

The `marker_detection_rate_hz: 15.0` limit at `dynamic_id2_arm_update_live.launch.py:88` means the node processes at most 15 frames per second. If the camera is at 15fps, this means every frame is processed. But if there's a startup delay where the camera starts before the ArUco node is ready, initial frames are missed.

### Finding 6: Both Cameras Use the Same Marker ID 0

**Severity: INFORMATIONAL — not a bug, but important context**

Both `head_aruco_map.yaml` and `arm_aruco_map.yaml` define fixed marker ID 0 with the same `T_map_marker`. This means both cameras are looking for the same physical marker. If the marker is only visible to one camera, only that camera's ArUco node will produce corrections.

The head config also has dynamic marker ID 2 (arm-mounted marker), which enables cross-camera arm localization.

---

## Implementation Plan

### Phase 1: Break the Positive Feedback Loop (Critical Fix)

- [ ] **Task 1.1.** In `head_aruco_map.yaml` and `arm_aruco_map.yaml`, increase `max_position_norm_m` from `5.0` to `50.0`. This prevents the VIO health gate from triggering during the initial convergence phase. The position norm check is a sanity check for detecting catastrophic VIO failure — 50m is still reasonable for that purpose while allowing the ArUco corrections to work during the early drift phase.

  **Rationale:** The 5m threshold was being hit during normal VIO startup drift, triggering the feedback loop. A 50m threshold still catches genuine VIO failures (runaway integration) while not interfering with normal startup.

- [ ] **Task 1.2.** In `head_aruco_map.yaml` and `arm_aruco_map.yaml`, reduce `stable_frames_required` from `8` to `3`. This allows the first correction to be applied faster, before VIO drifts too far.

  **Rationale:** 3 frames at 15Hz = 0.2s. This is still enough to reject spurious detections (a single false positive won't pass) but fast enough to catch VIO before it drifts past the workspace boundary. The existing quality gates (area, reprojection error, border margin, corner angles, geometry score) already filter out bad detections — the stability requirement is an additional conservative gate that's too conservative for the startup phase.

- [ ] **Task 1.3.** In `head_aruco_map.yaml` and `arm_aruco_map.yaml`, reduce `reanchor_cooldown_s` from `3.0` to `1.0`. This allows corrections to be applied more frequently during the VIO-invalid phase, giving the system more chances to converge.

  **Rationale:** A 3-second cooldown means VIO drifts ~15m between corrections (at 5 m/s drift rate). A 1-second cooldown means ~5m drift, which is much more manageable.

### Phase 2: Add Startup-Specific Behavior

- [ ] **Task 2.1.** In `aruco_marker_pose_node.py`, add a startup grace period logic: for the first N seconds after the first marker detection (configurable, default 10s), use a relaxed VIO health check. Specifically, skip the `position_outside_workspace` check during the grace period. Add a parameter `startup_grace_period_s` (default: 10.0) to the `vio_health` config section.

  **Rationale:** During startup, VIO is expected to drift. The ArUco corrections should be allowed to work without the VIO health gate interfering. After the grace period, the normal health checks apply.

- [ ] **Task 2.2.** In `aruco_marker_pose_node.py`, add a `startup_stable_frames_required` parameter (default: 3) that is used instead of `stable_frames_required` for the first correction. After the first successful correction, switch to the normal `stable_frames_required` value.

  **Rationale:** This provides a fast first correction while maintaining the conservative stability requirement for ongoing corrections.

### Phase 3: Improve Diagnostic Visibility

- [ ] **Task 3.1.** In `aruco_marker_pose_node.py`, add a periodic diagnostic log (every 5 seconds) that reports: number of markers detected, VIO health status, current position norm, number of corrections applied, number of corrections rejected (with reason breakdown). This makes it much easier to diagnose why ArUco corrections are or aren't happening.

  **Rationale:** Currently the only visibility is through the reanchor events and VIO health transitions. A periodic summary would make it immediately obvious whether the issue is "no markers detected" vs "markers detected but corrections rejected".

---

## Verification Criteria

- [ ] **V1.** Run 3 consecutive tests with no code changes. All 3 should produce ArUco EKF updates for both cameras. The current behavior is 0-341 updates with high variance; the target is consistent 50+ updates per run.
- [ ] **V2.** The first ArUco correction should occur within 1 second of the first marker detection (currently it can take 3+ seconds due to stability + cooldown delays).
- [ ] **V3.** No `position_outside_workspace` VIO health invalidations during the first 10 seconds of operation.
- [ ] **V4.** The periodic diagnostic log shows marker detection counts and correction acceptance rates.

---

## Potential Risks and Mitigations

1. **Reducing stability frames may allow false positive corrections**
   Mitigation: The existing quality gates (area, reprojection error, border margin, corner angles, geometry score) already filter out spurious detections. 3 consecutive frames of a false positive passing all quality gates is extremely unlikely. The stability gate is defense-in-depth, not the primary filter.

2. **Increasing max_position_norm_m to 50m may allow genuinely broken VIO to go uncorrected**
   Mitigation: The other VIO health checks (velocity > 2 m/s, covariance trace > 10) still catch catastrophic failures. A VIO that drifts to 50m in normal operation would also have high velocity and high covariance, triggering those checks instead.

3. **Reducing reanchor cooldown may cause oscillation**
   Mitigation: The hard reanchor still updates `T_map_global` which immediately corrects the published odometry. The cooldown prevents rapid successive reanchors, but 1 second is still enough to prevent oscillation while allowing faster convergence.

---

## Alternative Approaches

1. **Remove VIO health gate entirely for the first correction:** Instead of a grace period, simply skip all VIO health checks until the first successful correction. Simpler but less safe — if the first marker detection is a false positive, it would corrupt the state.

2. **Adaptive workspace boundary:** Instead of a fixed 5m/50m threshold, scale the workspace boundary based on time since startup (e.g., 50m for first 10s, then linearly decrease to 5m over 30s). More sophisticated but adds complexity.

3. **Separate startup config profile:** Create a separate YAML config for the startup phase with all relaxed parameters, and switch to the normal config after the first correction. Clean separation but requires maintaining two config files.

---

## Files to Modify

| File | Change |
|------|--------|
| `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml` | Increase `max_position_norm_m`, reduce `stable_frames_required`, reduce `reanchor_cooldown_s` |
| `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml` | Same changes |
| `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py` | Add startup grace period, startup stability frames, periodic diagnostics |

## Estimated Effort

- Phase 1 (Config changes): 15 minutes
- Phase 2 (Startup behavior): 1-2 hours
- Phase 3 (Diagnostics): 30 minutes
- Total: ~2-3 hours
