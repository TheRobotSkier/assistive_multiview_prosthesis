# VIO Axis Flip, Single-View Fusion & Chi2 Gate Fix — Plan v2

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

### Issue 2: Single-View Fusion — TF Timing Race + Arm Divergence

**Evidence:** Fusion stats show `dual=9, cam1_only=40` (out of 49 total publications). 98 TF failures across the run. The "DISCONNECTED entire run" in the analysis is a **false alarm** — the regex at `analyze_log.py:97` doesn't recognize `CONNECTED` status, only `OK|DISCONNECTED|STALE`. The chain actually connects at ~26s and stays connected.

**Root cause — TF timing race (detailed analysis):**

The fusion node's `_process_clouds` runs in a **background daemon thread** (`pointcloud_fusion_node.py:508`), so blocking TF lookups do NOT starve the executor. The issue is that the current code uses `lookup_transform_full` at the cloud's header stamp (`pointcloud_fusion_node.py:528-533`), and when the cloud arrives before its corresponding TF broadcast, the lookup fails with "extrapolation into the future."

**The timestamp domain mismatch causing the race:**
- **Cloud stamp**: The assembler copies the depth image's header (`naive_pointcloud_assembler.py:353`), which is restamped to Jetson system clock in the relay. Cloud stamp = `T_capture` (when depth was captured on Jetson).
- **TF stamp**: The relay stamps corrected TF with `host_clock.now()` (`openvins_odom_tf_relay.py:480`). TF stamp = `T_host_now` (when relay processes the odom on the host).
- **The race**: Depth and odom travel independent DDS paths. A cloud can arrive at the fusion node with stamp `T_capture` before the corresponding odom/TF at `T_capture` has been processed by the relay. The `lookup_transform_full` at `T_capture` then fails because the latest TF data is at `T_capture - 77ms`.

**Why the current `transform_tolerance_s` doesn't help:** The tolerance parameter in `lookup_transform_full` does NOT function as a future-extrapolation window — it only allows interpolation tolerance for past data. When the requested time is ahead of the latest data, TF2 raises an extrapolation error regardless of tolerance.

**Why waiting is the right fix:** Since `_process_clouds` is already in a background thread, we can afford to **wait** for the TF to arrive. Using `lookup_transform` (simpler than `_full`) with the cloud's stamp and a real timeout (e.g., 200ms) gives the relay time to publish the matching TF. The relay processes odom at ~35Hz (every ~28ms), so a 200ms timeout covers ~7 odom cycles — plenty of time. Only if the wait expires do we fall back to latest-available, ensuring we never lose a cloud entirely.

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

- [ ] **1.1. Disable arm IMU intrinsic online calibration.** Set `calib_imu_intrinsics: false` in `config/openvins/arm_d435i_310622071850/estimator_config.yaml:16` to match the head config. This removes 12 poorly-observable parameters from the state vector and stabilizes the accelerometer bias estimate. The `calib_imu_g_sensitivity: true` can remain (it's less destabilizing). This is the single highest-impact change — it addresses the root cause of arm divergence rather than treating symptoms.

- [ ] **1.2. Reduce `max_position_norm_m` from 50.0 to 5.0 for both arm and head.** In `config/markers/arm_aruco_map.yaml:108` and `config/markers/head_aruco_map.yaml:109`. The current 50.0m threshold means a 39m-diverged estimator is still considered "valid VIO." At 5.0m, divergence is detected early enough for the correction mechanisms to be effective. The prosthesis workspace is < 2m, so 5.0m gives generous margin for initialization transients while catching real divergence.

### Phase 2: Fix Axis Flip — Rotation Plausibility Check [JETSON-SIDE]

- [ ] **2.1. Add rotation plausibility check before hard reanchor.** In `aruco_marker_pose_node.py`, before calling `hard_set_correction` in the VIO-invalid path (line 1926) and the fallback path (line 1136), check whether the measurement's rotation relative to the current `T_map_global` exceeds a threshold (e.g., 90 degrees). If it does, reject the measurement with a diagnostic log. This blocks solvePnP pose ambiguity flips from entering the state. The check should use the same `correction_innovation` mechanism already available (compute rotation innovation, check `rot_deg > max_hard_reanchor_rotation_deg`). Add a config parameter `max_hard_reanchor_rotation_deg: 90.0` under the `reanchor` section.

- [ ] **2.2. Add rotation plausibility check to the fallback path.** In the fallback path (`aruco_marker_pose_node.py:1062-1137`), after solving the marker pose, check the rotation relative to the last known `T_map_global`. If `T_map_global` is None (first lock), allow any rotation. Otherwise, reject measurements where the rotation delta exceeds `max_hard_reanchor_rotation_deg`. This prevents the fallback path — which already bypasses quality gates — from injecting flipped poses.

### Phase 3: Revert the Chi2 Dual-Mode Gate [JETSON-SIDE]

- [ ] **3.1. Revert the dual-mode chi2 gate to the original tight gate.** Remove the `correction_chi2_gate_relaxed` and `chi2_relax_recovery_window_s` parameters and the dual-mode logic at `aruco_marker_pose_node.py:1987-1994`. Restore the original `if chi2 > self.correction_chi2_gate:` check. The dual-mode gate is dead code in the VIO-invalid path (where hard reanchor bypasses it) and ineffective in the VIO-valid recovery window (where step clamping prevents meaningful correction). The tight gate provides protection against noisy measurements during healthy operation. Recovery from divergence is handled by the hard reanchor path (now protected by the rotation check from Phase 2).

### Phase 4: Fix TF Timing Race — Wait First, Then Fallback [HOST-SIDE]

- [ ] **4.1. Replace `lookup_transform_full` with a two-stage wait-then-fallback strategy.** In `pointcloud_fusion_node.py:521-548`, restructure the TF lookup as follows:

  **Stage 1 — Wait for the temporally-correct transform:** Use `lookup_transform` (NOT `lookup_transform_full`) with the cloud's header stamp and a timeout of 200ms. This gives the relay time to publish the matching TF. Since `_process_clouds` runs in a background daemon thread (`pointcloud_fusion_node.py:508`), this wait does NOT block the executor — the executor continues receiving clouds and TF updates. At 35Hz relay rate, 200ms covers ~7 odom cycles, which is more than enough time for the matching TF to arrive.

  **Stage 2 — Fall back to latest-available:** If the 200ms wait expires (genuine TF gap or relay failure), retry with `lookup_transform(target, source, rclpy.time.Time())` (latest available) and a 0.5s timeout. This ensures we never lose a cloud entirely. Log the fallback at warn level (throttled) so the timing race is visible in analysis.

  **Why `lookup_transform` instead of `lookup_transform_full`:** The `_full` variant does a two-step lookup through a fixed frame (target → fixed → source), which has different timeout semantics and is more prone to edge-case failures. The simpler `lookup_transform` does a direct chain lookup and handles timeouts more reliably. Since both source and target are in the same TF tree connected by static + dynamic edges, the direct lookup is equivalent.

  **Why not increase `transform_tolerance_s`:** The tolerance parameter does not function as a future-extrapolation window in TF2 — it only allows interpolation slack for past data. Increasing it from 0.5s to 1.0s would not fix the "extrapolation into the future" error.

### Phase 5: Fix Analysis Regex Bug [HOST-SIDE]

- [ ] **5.1. Fix the `RE_DIAG_CHAIN_LINE` regex to recognize `CONNECTED` status.** In `scripts/analyze_log.py:97`, change `(OK|DISCONNECTED|STALE)` to `(CONNECTED|DISCONNECTED|STALE|OK)`. This eliminates the false "DISCONNECTED entire run" reports. The actual chain status will then be correctly reported as connected after ~26s.

### Phase 6: C++ Velocity-Fit Gate Bypass for Large Divergence [JETSON-SIDE]

- [ ] **6.1. Bypass the velocity-fit gate when divergence is catastrophic.** In the OpenVINS C++ code (`VioManager.cpp`), when the innovation translation exceeds a large threshold (e.g., 5.0m — 10x the normal reset threshold), bypass the `marker_velocity_fit()` check and perform the reset with zero velocity. A 5m-diverged estimator cannot be made worse by a zero-velocity reset — the current state is already catastrophically wrong. This unblocks the only recovery mechanism for large divergence. **Note:** This requires rebuilding the OpenVINS C++ node on the Jetson.

---

## Verification Criteria

- [ ] No axis flip events — rotation innovation check blocks > 90° hard reanchors
- [ ] Arm VIO stays valid (PosNorm < 5.0m) for > 60s of continuous operation
- [ ] Arm accelerometer bias stays below 0.2 m/s² (vs current 0.4 m/s²)
- [ ] Fused point cloud shows `dual > cam1_only` in the fusion node stats
- [ ] TF failures in fusion node reduced from 98 to < 20 per run
- [ ] TF wait-fallback logs show < 5% fallback rate (most clouds get temporally-correct TF)
- [ ] Analysis correctly reports TF chain as CONNECTED after startup
- [ ] Chi2 rejections stay below 20 per run (tight gate, stable VIO)

## Potential Risks and Mitigations

1. **Disabling arm IMU intrinsic calibration reduces accuracy if the offline calibration is wrong**
   Mitigation: The head camera works well with `calib_imu_intrinsics: false`, proving the offline calibration is sufficient. If arm accuracy degrades, re-enable with a tighter process noise on the intrinsic parameters.

2. **Rotation plausibility check (90°) might reject legitimate fast rotations**
   Mitigation: 90° is very generous — real prosthesis motion rarely exceeds 45° per frame at 15 FPS. The check only applies to hard reanchors (VIO-invalid path), not soft updates. If needed, increase to 120°.

3. **200ms TF wait adds latency to cloud processing**
   Mitigation: The wait runs in a background daemon thread — it does NOT block the executor or delay incoming cloud/TF processing. In the common case, the TF is already available and the wait returns immediately. The 200ms timeout only fires when the cloud arrives before its TF, which should be rare once arm VIO is stabilized (Phase 1). If the wait fires frequently, it indicates a persistent timing offset that should be investigated separately.

4. **Latest-available TF fallback introduces slight temporal misalignment**
   Mitigation: This fallback only fires after the 200ms wait expires — it's the last resort, not the primary path. At 35Hz corrected-TF rate, the latest transform is at most ~28ms old. The cloud_max_age_s (1.0s) window provides 35x margin. The misalignment is negligible compared to the current alternative (dropping the cloud entirely and getting single-view fusion).

5. **Lowering `max_position_norm_m` to 5.0m might cause false VIO-invalid during initialization**
   Mitigation: The 20s startup grace period (`startup_grace_period_s`) tolerates large positions during OpenVINS dynamic initialization. After grace, 5.0m is generous for a prosthesis workspace.

6. **C++ velocity-fit bypass might inject wrong velocity during recovery**
   Mitigation: Zero velocity is strictly better than a 39m-diverged state with runaway velocity. The marker correction layer will continue refining `T_map_global` after the reset.

## Alternative Approaches

1. **Keep chi2 dual-mode but fix the dead-code path:** Move the chi2 gate evaluation before the VIO-invalid hard reanchor check, so the relaxed gate actually applies during VIO-invalid recovery. **Rejected:** This would slow recovery (chi2 evaluation adds latency) and the hard reanchor path is the correct mechanism for VIO-invalid recovery — it just needs the rotation check from Phase 2.

2. **Increase soft update step limits during recovery window:** Scale `max_position_correction_step_m` and `max_rotation_correction_step_deg` proportionally to innovation magnitude during the relaxed-gate window. **Rejected:** This risks overshoot and oscillation. The hard reanchor with rotation check is cleaner.

3. **Remove the fallback path entirely:** Eliminate the quality-gate bypass at lines 1062-1137 so only quality-checked measurements reach the correction layer. **Considered but deferred:** The fallback path serves a purpose during deep VIO failure (no markers pass quality gates). The rotation check from Phase 2 is sufficient to make it safe.

4. **Stamp relay TF with odom timestamp instead of host clock:** Eliminate the timing race by using the same time domain for TF and clouds. **Rejected for now:** This was previously changed TO host clock to fix a 10s clock gap problem. The wait-then-fallback approach from Phase 4 is safer and doesn't risk reintroducing the old bug.

5. **Use `lookup_transform_full` with a longer timeout instead of switching to `lookup_transform`:** **Rejected:** The `_full` variant's timeout semantics are unreliable for future-extrapolation cases. The simpler `lookup_transform` with a real timeout is more predictable, and since both frames are in the same connected TF tree, the direct lookup produces identical results.
