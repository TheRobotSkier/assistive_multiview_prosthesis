# VIO Axis Flip, Single-View Fusion & Chi2 Gate Fix — Plan v3

## Objective

Resolve three persistent issues from the v6 run (20260620_180719):
1. **Camera axis flip** — one camera's orientation flips and never recovers
2. **Single-view fusion** — fused point cloud often only includes one camera
3. **Chi2 gate too permissive** — the relaxed gate (10000.0) is ineffective and potentially harmful

## Root Cause Analysis

### Issue 1: Axis Flip — Hard Reanchor Applies Bad Rotation From Fallback Path

**Evidence:** User observed a camera axis flip that never recovered. The analysis shows arm VIO reached 39.62m divergence with 257 translation jumps.

**Root cause chain:**

1. Arm VIO diverges (see Issue 3 below for why). When `vio_valid=False`, the correction path at `aruco_marker_pose_node.py:1905-1937` takes the `hard_set_correction` path, which directly copies the measurement transform with **no rotation limit** (`aruco_marker_pose_node.py:2047`: `self.T_map_global = T_meas_map_global.copy()`).

2. The measurement comes from the **fallback path** (`aruco_marker_pose_node.py:1062-1137`), which activates when VIO is invalid and all markers failed quality gates. This path **bypasses geometry checks, reprojection error limits, and distance limits** — it only requires temporal stability (3 consecutive frames).

3. ArUco markers are planar, and solvePnP on planar targets has a known pose ambiguity: two solutions (the true pose and a ~180°-rotated pose) can both produce low reprojection error. The fallback path accepts whichever solvePnP returns, with no rotation-plausibility check.

4. When the flipped measurement reaches `hard_set_correction`, the 180° rotation is applied directly to `T_map_global`. This flipped transform is then broadcast as the corrected TF, causing the visible axis flip. Since the flipped pose is "stable" (solvePnP consistently returns the same flipped solution), subsequent reanchors reinforce it.

**Why it never recovers:** Once `T_map_global` is flipped, the innovation (difference between measurement and current state) for the *correct* pose is enormous — it exceeds the `max_periodic_translation_correction_m` (0.30m) gate at `:1967`, triggering `hold_vio_invalid` → another hard reanchor → which accepts the flipped measurement again (because the fallback path doesn't check rotation plausibility). The system is trapped in the flipped state.

### Issue 2: Single-View Fusion — TF Timestamp Domain Mismatch (Root Cause)

**Evidence:** Fusion stats show `dual=9, cam1_only=40` (out of 49 total publications). 98 TF failures across the run. The "DISCONNECTED entire run" in the analysis is a **false alarm** — the regex at `analyze_log.py:97` doesn't recognize `CONNECTED` status, only `OK|DISCONNECTED|STALE`. The chain actually connects at ~26s and stays connected.

**Root cause — TF stamped with host processing time, not observation time:**

The relay currently stamps ALL dynamic TF edges with `host_clock.now()` (`openvins_odom_tf_relay.py:480,631`). This is the time the relay *processes* the odom message on the host, NOT when the pose was actually observed. Meanwhile, clouds are stamped with the depth capture time (Jetson system clock, restamped in the relay at `jetson_relay.py:755`).

This creates a **temporal lie**: the TF claims the pose was valid at `T_host_process`, but the pose was actually observed at `T_odom` (which arrived earlier via DDS). When the fusion node does `lookup_transform` at the cloud's stamp (`T_capture ≈ T_odom`), it fails with "extrapolation into the future" because the TF at `T_capture` hasn't been published yet — the relay hasn't processed the corresponding odom message yet.

**Why the old "10s clock gap" comment is no longer relevant:**

The code comment at `openvins_odom_tf_relay.py:625-630` says:
```
# Using the Jetson odom timestamp created a ~10s clock gap
# that prevented TF2 from composing the multi-edge chain
```

This was from before chrony was properly configured. The sysmon now confirms **chrony drift of 0.0ms** — the Jetson and host clocks are synced to sub-millisecond precision. The odom timestamp and host clock are in the **same time domain**. The 10s gap was a chrony failure symptom, not a fundamental architecture issue.

**Why static edges compose correctly with odom-timestamp TF:**

All non-dynamic edges in the TF chain are published via `StaticTransformBroadcaster`:
- `*_imu → *_cam0` (relay static, `openvins_odom_tf_relay.py:356`)
- `*_cam0 → *_link` (bridge static, `openvins_realsense_tf_bridge_node.py:347`)
- `*_link → *_depth_optical_frame` etc. (camera mount static, `publish_camera_mounts.py:241`)

TF2 treats static transforms as **valid for all times**. They compose correctly with any dynamic-edge timestamp. The only time-varying edge is `marker_map → *_imu`, which is the one we're fixing.

**The fix:** Stamp the dynamic TF edge with `msg.header.stamp` (the odom timestamp) instead of `host_clock.now()`. This makes the TF timestamp temporally accurate — it says "this is the pose at time T_odom" which is TRUE. The fusion node then finds the TF at the cloud's stamp because both are in the same temporal domain.

**Chrony safety guard:** Add a sanity check — if the odom timestamp is more than 2.0s away from `host_clock.now()`, fall back to `host_clock.now()` and log a warning. This catches any future chrony failures without crashing the pipeline.

### Issue 3: Chi2 Gate — Too Permissive AND Ineffective

**Evidence:** 32 chi2 rejections (mean=204.84, max=1033.04, gate=16.81). The relaxed gate (10000.0) was supposed to help during VIO recovery.

**The dual-mode gate is dead code in the critical scenario:**

The correction decision tree in `try_apply_marker_correction()`:
```
Step 6: vio_valid == False? → hard_set_correction (bypasses chi2 entirely)
Step 9: innovation > 0.30m?  → hold_vio_invalid → reject (bypasses chi2)
Step 10: chi2 > effective_gate? → this is where dual-mode fires
```

When VIO is invalid (the scenario we're trying to fix), step 6 fires and returns before reaching the chi2 gate. The dual-mode gate only evaluates when VIO is **valid** but **recently invalid** (within 5s). In that narrow window, `soft_update_correction` clamps corrections to 0.05m/2deg per step — far too slow to counter real divergence.

**Net effect:** The relaxed gate (10000.0) lets bad measurements through the chi2 check during the recovery window, but the step clamping prevents effective correction anyway. Meanwhile, the relaxed gate provides zero protection against the flipped poses from the fallback path (those bypass chi2 via the hard reanchor route). The gate change is all cost, no benefit.

### Root Cause of Arm Divergence: IMU Intrinsic Online Calibration

**Critical config asymmetry:**
- Arm: `calib_imu_intrinsics: true` + `calib_imu_g_sensitivity: true` (21 extra state parameters)
- Head: `calib_imu_intrinsics: false` + `calib_imu_g_sensitivity: true`

The arm jointly estimates 12 IMU intrinsic parameters (Dw=6, Da=6) plus 9 gyro g-sensitivity parameters online. These are notoriously poorly observable in monocular VIO. The intrinsics absorb unmodeled errors and corrupt the accelerometer bias estimate.

**Evidence from the run log:**
- Arm accelerometer bias: `ba = 0.4012, -0.3477, 0.4584` (0.4 m/s² — physically implausible)
- Head accelerometer bias: `ba = -0.2461, -0.0952, 0.0785` (reasonable)
- Arm Da matrix has 4.6% scale error on one axis (vs head near-identity)
- Arm VIO: INVALID, PosNorm=39.62m. Head VIO: valid, PosNorm=0.73m

The corrupted bias drives runaway integration drift. The Python correction layer can snap `T_map_global` back, but cannot fix the underlying OpenVINS state (velocity, bias, covariance). The C++ full reset is the only mechanism that can — but it's blocked by a velocity-fit gate requiring 5 samples < 2.0 m/s within 0.5s, which rarely succeeds on a moving arm.

---

## Implementation Plan

### Phase 1: Fix Arm VIO Divergence (Root Cause) [JETSON-SIDE]

- [x] **1.1. Disable arm IMU intrinsic online calibration.** Set `calib_imu_intrinsics: false` in `config/openvins/arm_d435i_310622071850/estimator_config.yaml:16` to match the head config. This removes 12 poorly-observable parameters from the state vector and stabilizes the accelerometer bias estimate. The `calib_imu_g_sensitivity: true` can remain (it's less destabilizing). This is the single highest-impact change — it addresses the root cause of arm divergence rather than treating symptoms.

- [x] **1.2. Reduce `max_position_norm_m` from 50.0 to 5.0 for both arm and head.** In `config/markers/arm_aruco_map.yaml:108` and `config/markers/head_aruco_map.yaml:109`. The current 50.0m threshold means a 39m-diverged estimator is still considered "valid VIO." At 5.0m, divergence is detected early enough for the correction mechanisms to be effective. The prosthesis workspace is < 2m, so 5.0m gives generous margin for initialization transients while catching real divergence.

### Phase 2: Fix Axis Flip — Rotation Plausibility Check [JETSON-SIDE]

- [x] **2.1. Add rotation plausibility check before hard reanchor.** In `aruco_marker_pose_node.py`, before calling `hard_set_correction` in the VIO-invalid path (line 1926) and the fallback path (line 1136), check whether the measurement's rotation relative to the current `T_map_global` exceeds a threshold (e.g., 90 degrees). If it does, reject the measurement with a diagnostic log. This blocks solvePnP pose ambiguity flips from entering the state. The check should use the same `correction_innovation` mechanism already available (compute rotation innovation, check `rot_deg > max_hard_reanchor_rotation_deg`). Add a config parameter `max_hard_reanchor_rotation_deg: 90.0` under the `reanchor` section.

- [x] **2.2. Add rotation plausibility check to the fallback path.** In the fallback path (`aruco_marker_pose_node.py:1062-1137`), after solving the marker pose, check the rotation relative to the last known `T_map_global`. If `T_map_global` is None (first lock), allow any rotation. Otherwise, reject measurements where the rotation delta exceeds `max_hard_reanchor_rotation_deg`. This prevents the fallback path — which already bypasses quality gates — from injecting flipped poses.

### Phase 3: Revert the Chi2 Dual-Mode Gate [JETSON-SIDE]

- [x] **3.1. Revert the dual-mode chi2 gate to the original tight gate.** Remove the `correction_chi2_gate_relaxed` and `chi2_relax_recovery_window_s` parameters and the dual-mode logic at `aruco_marker_pose_node.py:1987-1994`. Restore the original `if chi2 > self.correction_chi2_gate:` check. The dual-mode gate is dead code in the VIO-invalid path (where hard reanchor bypasses it) and ineffective in the VIO-valid recovery window (where step clamping prevents meaningful correction). The tight gate provides protection against noisy measurements during healthy operation. Recovery from divergence is handled by the hard reanchor path (now protected by the rotation check from Phase 2).

### Phase 4: Fix TF Timestamp Domain — Use Odom Timestamp [HOST-SIDE]

This phase replaces the wait-then-fallback approach from v2 with a root-cause fix: stamp the dynamic TF edge with the odom message's timestamp so the TF and cloud share the same temporal domain.

- [x] **4.1. Change the corrected-TF rebroadcast to use odom timestamp.** In `openvins_odom_tf_relay.py:480`, replace `host_stamp = self.get_clock().now().to_msg()` with `odom_stamp = msg.header.stamp`. This applies to the corrected-TF path (line 480) and the hold-last-good cache rebroadcast (line 529). Add a chrony sanity guard: if `|odom_stamp - host_clock.now()| > 2.0s`, fall back to `host_clock.now()` and log a warning (throttled). This catches future chrony failures.

- [x] **4.2. Change the raw VIO TF broadcast to use odom timestamp.** In `openvins_odom_tf_relay.py:631`, replace `host_stamp = self.get_clock().now().to_msg()` with the same odom timestamp + chrony sanity guard from 4.1. Update the code comment to explain that the old "10s clock gap" was a chrony failure symptom, now resolved.

- [x] **4.3. Simplify the fusion node TF lookup.** In `pointcloud_fusion_node.py:521-548`, replace the `lookup_transform_full` call with a simpler `lookup_transform` at the cloud's header stamp with a 0.2s timeout. Since the TF is now stamped in the same domain as the cloud, the lookup will succeed when the matching TF has arrived (which it should have, since odom arrives at ~35Hz and clouds arrive at ~5Hz). The `_process_clouds` method runs in a background daemon thread (`pointcloud_fusion_node.py:508`), so the 0.2s timeout does NOT block the executor. Remove the `clock_skew_detected` fallback path for the header-stamp lookup — with the odom-timestamp TF and chrony at 0.1ms drift, the fallback is unnecessary. If the lookup genuinely fails (TF gap), log it and skip the cloud — do NOT fall back to latest-available, since a wrong transform produces worse results than no transform.

### Phase 5: Fix Analysis Regex Bug [HOST-SIDE]

- [x] **5.1. Fix the `RE_DIAG_CHAIN_LINE` regex to recognize `CONNECTED` status.** In `scripts/analyze_log.py:97`, change `(OK|DISCONNECTED|STALE)` to `(CONNECTED|DISCONNECTED|STALE|OK)`. This eliminates the false "DISCONNECTED entire run" reports. The actual chain status will then be correctly reported as connected after ~26s.

### Phase 6: C++ Velocity-Fit Gate Bypass for Large Divergence [JETSON-SIDE]

- [x] **6.1. Bypass the velocity-fit gate when divergence is catastrophic.** In the OpenVINS C++ code (`VioManager.cpp`), when the innovation translation exceeds a large threshold (e.g., 5.0m — 10x the normal reset threshold), bypass the `marker_velocity_fit()` check and perform the reset with zero velocity. A 5m-diverged estimator cannot be made worse by a zero-velocity reset — the current state is already catastrophically wrong. This unblocks the only recovery mechanism for large divergence. **Note:** This requires rebuilding the OpenVINS C++ node on the Jetson.

---

## Verification Criteria

- [ ] No axis flip events — rotation innovation check blocks > 90° hard reanchors
- [ ] Arm VIO stays valid (PosNorm < 5.0m) for > 60s of continuous operation
- [ ] Arm accelerometer bias stays below 0.2 m/s² (vs current 0.4 m/s²)
- [ ] Fused point cloud shows `dual > cam1_only` in the fusion node stats
- [ ] TF failures in fusion node reduced from 98 to < 10 per run
- [ ] No chrony fallback warnings during normal operation (confirms odom-timestamp TF is stable)
- [ ] Analysis correctly reports TF chain as CONNECTED after startup
- [ ] Chi2 rejections stay below 20 per run (tight gate, stable VIO)

## Potential Risks and Mitigations

1. **Disabling arm IMU intrinsic calibration reduces accuracy if the offline calibration is wrong**
   Mitigation: The head camera works well with `calib_imu_intrinsics: false`, proving the offline calibration is sufficient. If arm accuracy degrades, re-enable with a tighter process noise on the intrinsic parameters.

2. **Rotation plausibility check (90°) might reject legitimate fast rotations**
   Mitigation: 90° is very generous — real prosthesis motion rarely exceeds 45° per frame at 15 FPS. The check only applies to hard reanchors (VIO-invalid path), not soft updates. If needed, increase to 120°.

3. **Odom-timestamp TF could fail if chrony breaks**
   Mitigation: The chrony sanity guard (2.0s threshold) falls back to host clock if the odom timestamp is implausible. The sysmon independently monitors chrony drift at 0.1ms precision. The 2.0s threshold is 20000x the normal drift, giving enormous margin.

4. **Simplified fusion TF lookup (no latest-available fallback) might drop clouds during TF gaps**
   Mitigation: With odom-timestamp TF and chrony at 0.1ms, TF gaps should be extremely rare. The 0.2s timeout in the background thread gives the relay 7 odom cycles to publish the matching TF. If gaps persist, it indicates a relay or DDS problem that should be investigated, not papered over with wrong transforms.

5. **Lowering `max_position_norm_m` to 5.0m might cause false VIO-invalid during initialization**
   Mitigation: The 20s startup grace period (`startup_grace_period_s`) tolerates large positions during OpenVINS dynamic initialization. After grace, 5.0m is generous for a prosthesis workspace.

6. **C++ velocity-fit bypass might inject wrong velocity during recovery**
   Mitigation: Zero velocity is strictly better than a 39m-diverged state with runaway velocity. The marker correction layer will continue refining `T_map_global` after the reset.

## Alternative Approaches

1. **Keep chi2 dual-mode but fix the dead-code path:** Move the chi2 gate evaluation before the VIO-invalid hard reanchor check, so the relaxed gate actually applies during VIO-invalid recovery. **Rejected:** This would slow recovery (chi2 evaluation adds latency) and the hard reanchor path is the correct mechanism for VIO-invalid recovery — it just needs the rotation check from Phase 2.

2. **Increase soft update step limits during recovery window:** Scale `max_position_correction_step_m` and `max_rotation_correction_step_deg` proportionally to innovation magnitude during the relaxed-gate window. **Rejected:** This risks overshoot and oscillation. The hard reanchor with rotation check is cleaner.

3. **Remove the fallback path entirely:** Eliminate the quality-gate bypass at lines 1062-1137 so only quality-checked measurements reach the correction layer. **Considered but deferred:** The fallback path serves a purpose during deep VIO failure (no markers pass quality gates). The rotation check from Phase 2 is sufficient to make it safe.

4. **Wait-then-fallback TF lookup (v2 approach):** Keep host-clock TF stamping but add a 200ms wait + latest-available fallback in the fusion node. **Rejected:** The user correctly identified that host-clock stamping makes "time lose its meaning." Using the odom timestamp is the root-cause fix — it eliminates the race entirely rather than working around it. The wait-then-fallback approach would still produce temporally inaccurate transforms during the fallback.

5. **Use `lookup_transform_full` with a longer timeout instead of switching to `lookup_transform`:** **Rejected:** The `_full` variant's timeout semantics are unreliable for future-extrapolation cases. The simpler `lookup_transform` with a real timeout is more predictable, and since both frames are in the same connected TF tree, the direct lookup produces identical results.
