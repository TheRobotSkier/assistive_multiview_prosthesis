# VIO Instability & Pointcloud Fusion Issues — Fix Plan

## Objective

Resolve two user-reported symptoms — (1) arm stabilizing at a wrong position away from the marker, and (2) fused point clouds frequently showing only one view instead of both — by addressing their root causes identified through log/bag analysis and code investigation.

---

## Root Cause Analysis

### Symptom 1: Arm stabilizing away from marker

**Root cause: Arm VIO solvePnP flip trap.** The arm camera's ArUco correction pipeline is stuck in a self-reinforcing trap state:

1. Arm VIO diverges slightly (position norm drifts to ~1m)
2. VIO is marked invalid → fallback path activates (bypasses quality gates)
3. `solvePnP` on the planar ArUco marker returns the **flipped ambiguity solution** (~122° rotation)
4. The 90° rotation plausibility wall (`max_hard_reanchor_rotation_deg`) correctly blocks this
5. `hold_vio_invalid()` is called, extending the invalid window
6. No correction gets through → VIO stays diverged → loop repeats

**Evidence from run `20260620_215252`:**
- Arm VIO yield: **0.1%** (817 markers detected, only 1 correction applied)
- 105 `rotation_hard` rejections, mean=121.72° (suspiciously consistent → systematic flip)
- 68 `chi2` rejections, mean=98.22 vs gate=16.81 (innovation covariance far too tight)
- 8 `hard_reanchor_rotation_implausible` events on arm
- Arm `pos_norm` stabilizes at 0.53m — not drifting further, but **locked at wrong position**
- Head VIO yield: 3.1% (39 corrections) — head works because solvePnP returns correct solution

**The corrected TF path (which the relay broadcasts) inherits this wrong position.** The relay's LPF and hold-last-good cache smooth the jumps but cannot fix the underlying wrong pose. The proximity controller then measures distance from this wrong TF position, causing the arm to stabilize at the wrong location.

### Symptom 2: Fused point clouds showing only one view

**Root cause: TF lookup failures at cloud timestamp (extrapolation into the future).** The fusion node looks up TF at the cloud's header stamp with a 0.2s timeout. When the cloud arrives before its corresponding TF broadcast (transport latency race under CPU pressure), the lookup fails with "extrapolation into the future" and that camera's cloud is silently dropped.

**Evidence from fusion node stats (progressive degradation):**

| Time | Published | Dual | Cam1-only | tf_fail (head/arm) |
|------|-----------|------|-----------|---------------------|
| +40s | 22 | 11 | 24 | 77/76 |
| +50s | 34 | 4 | 26 | 81/62 |
| +60s | 36 | 3 | 29 | 97/76 |
| +70s | 38 | 2 | 38 | 100/60 |
| +80s | 46 | **0** | 52 | 103/71 |

The dual-view ratio drops from ~31% to **0%** over the run. TF failures accumulate continuously (head: 77→103, arm: 23→71).

**Contributing factors:**
- TF chain "FLIPPED" status: 13 OK, 5 DISCONNECTED of 18 blocks (~28% disconnection rate)
- 4058 TF jump events — corrected TF itself is wildly unstable
- Peak transport latency: 2.340s (clouds arrive far before their TF)
- Jetson CPU at 85-97% avg — TF broadcasts delayed under load

### Symptom 3 (discovered): DIAG-PC analyzer reports all zeros

The analysis report shows `head: depth=0.0Hz rgb=0.0Hz sync=0.0Hz points=0.0Hz reason=unknown` despite the raw log clearly showing healthy pointcloud chain data. This is a **key-name mismatch bug**: the DIAG-PC node emits `depth_c=3.3Hz image_c=4.5Hz ... points=2.8Hz [OK]` but the analyzer's `print_pointcloud_chain_health_log()` looks for keys `depth_in_hz`, `rgb_in_hz`, `sync_hz`, `points_hz`, `reason` — none of which exist in the actual format.

**Impact:** The operator's primary diagnostic surface (the analysis report) is blind to pointcloud chain health, making it impossible to diagnose fusion issues from the report alone.

---

## Implementation Plan

### Phase 1: Fix Arm VIO solvePnP Flip Trap (Symptom 1)

- [ ] **1.1. Implement solvePnP ambiguity resolution in `aruco_marker_pose_node.py`**
  - File: `multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`
  - The `solve_marker_pose()` function (~line 1155-1195) uses `SOLVEPNP_ITERATIVE` which is susceptible to planar marker pose ambiguity. Add a secondary validation pass: after solvePnP, check if the solution is the "flipped" alternative by testing the alternative hypothesis (rotate 180° around the marker's surface normal) and selecting the solution with lower reprojection error AND consistent viewing direction (camera Z-axis pointing toward marker surface, not away).
  - Rationale: The ~122° consistent rotation innovation indicates a systematic flip, not random noise. Resolving the ambiguity at the source eliminates the downstream rejection cascade.

- [ ] **1.2. Implement adaptive chi2 gate in `aruco_marker_pose_node.py`**
  - File: same as above, `innovation_chi2()` (~line 2042) and `try_apply_marker_correction()` (~line 1822)
  - When VIO is invalid AND position divergence exceeds a threshold (e.g., >0.5m), widen the chi2 gate by a configurable multiplier (e.g., 5× or 10×). This allows corrections through when the estimator is catastrophically diverged, breaking the trap state.
  - Add parameter `recovery_chi2_gate_multiplier` (default 5.0) and `recovery_pos_divergence_threshold_m` (default 0.5).
  - Rationale: The 68 chi2 rejections (mean=98.22 vs gate=16.81) show the innovation covariance is far too tight relative to actual innovations when VIO is diverged. The gate should adapt to the estimator's actual uncertainty.

- [ ] **1.3. Add position-based adaptive rotation threshold in `aruco_marker_pose_node.py`**
  - File: same as above, `try_apply_marker_correction()` (~line 1888)
  - When position divergence is catastrophic (e.g., >2m), allow larger rotation corrections (e.g., up to 150°) since a severely diverged VIO may legitimately need large corrections. The solvePnP flip resolution (1.1) should make this safe.
  - Add parameter `recovery_max_rotation_deg` (default 150.0).
  - Rationale: The fixed 90° wall is correct for steady-state but blocks recovery when the system needs it most. Combined with ambiguity resolution, this provides an escape hatch.

- [ ] **1.4. Normalize `calib_cam_timeoffset` in arm OpenVINS config**
  - File: `multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/estimator_config.yaml:14`
  - Change `calib_cam_timeoffset: false` to `calib_cam_timeoffset: true` to match head config.
  - Rationale: The arm has a measured Kalibr timeshift of ~8.8ms. Freezing it (`false`) means any residual timing drift cannot be corrected online. The head enables online refinement and is more stable. This is a minor contributor but eliminates a config asymmetry that could subtly degrade arm feature tracking.

### Phase 2: Fix Single-View Fusion (Symptom 2)

- [ ] **2.1. Add TF fallback-to-latest in fusion node `_process_clouds()`**
  - File: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:521-546`
  - When the stamp-specific TF lookup fails (extrapolation/timeout), retry with `rclpy.time.Time()` (latest available) as a fallback. Log this as a distinct event (`tf_fallback`) so it's visible in stats.
  - Add parameter `allow_tf_latest_fallback` (default True).
  - Rationale: The current design philosophy ("a wrong transform produces worse results than no transform") is too conservative for this system. Under CPU pressure with 2.3s transport latency, stamp-specific lookups fail ~50% of the time. A slightly-stale TF (100-200ms old) produces far better fusion results than dropping the cloud entirely.

- [ ] **2.2. Increase TF lookup timeout from 0.2s to 0.5s in fusion node**
  - File: same as above, line 536
  - Change `timeout=rclpy.duration.Duration(seconds=0.2)` to `seconds=0.5`.
  - Rationale: The corrected TF relay has a 0.5s lookup timeout itself. With transport latency reaching 2.3s, 0.2s is insufficient for the TF to propagate through the DDS layer. This runs in a background daemon thread so it does not block the executor.

- [ ] **2.3. Add dual-view ratio metric and warning to fusion node stats**
  - File: same as above, `_log_stats()` (~line 1004)
  - Compute and log `dual_ratio = dual / (dual + cam1_only)` in each stats interval. Emit a WARN if dual_ratio drops below a configurable threshold (default 0.3) for two consecutive intervals.
  - Rationale: The progressive degradation from 31% to 0% dual-view was invisible in real-time. Making this metric prominent enables early detection and intervention.

### Phase 3: Fix DIAG-PC Analyzer (Symptom 3)

- [ ] **3.1. Fix key-name mapping in `analyze_log.py` DIAG-PC parser**
  - File: `scripts/analyze_log.py:784-789`
  - Update `print_pointcloud_chain_health_log()` to use the actual DIAG-PC key names: `depth_c` instead of `depth_in_hz`, `image_c` instead of `rgb_in_hz`, `points` instead of `points_hz`, and parse the `[stage]` bracket for the reason instead of looking for a `reason` key.
  - Also add `depth_r`, `image_r`, `ci_raw` to the output for completeness.
  - Rationale: The analyzer is the operator's primary diagnostic surface. A blind pointcloud health section makes it impossible to diagnose fusion issues from the report.

- [ ] **3.2. Add dual-view ratio extraction from fusion node stats to analyzer**
  - File: `scripts/analyze_log.py`
  - Parse the fusion node's `Stats: published=N (dual=X, cam1_only=Y)` lines and report the dual-view ratio over time in the analysis report.
  - Rationale: This metric directly answers "are we getting single-view fusion?" and should be a first-class metric in the report.

### Phase 4: Improve TF Stability (Supporting Both Symptoms)

- [ ] **4.1. Investigate and fix corrected TF instability on arm side**
  - File: `src/camera/camera/openvins_odom_tf_relay.py:477-538`
  - The analysis shows 1185 TF jumps on `marker_map -> arm_imu_openvins_corrected` — the corrected TF itself is unstable. This is because the aruco_marker_pose_node's corrected odom inherits the solvePnP flip problem (Phase 1 fixes this upstream). After Phase 1 fixes, verify that corrected TF jump count drops significantly.
  - If jumps persist after Phase 1, investigate whether the LPF alpha (0.3) is too aggressive (introduces discontinuities when the input jumps) and consider reducing to 0.15 or adding a jump detector that holds the last good value instead of filtering through a large discontinuity.
  - Rationale: The corrected TF path is the primary TF source for downstream consumers. Its stability directly determines fusion quality and proximity controller accuracy.

- [ ] **4.2. Consider increasing corrected TF cache max age from 2.0s to 4.0s**
  - File: `config/prosthesis_config.yaml` — add `corrected_tf_cache_max_age_s: 4.0` under `openvins_odom_tf_relay`
  - Rationale: Transport latency reaches 2.3s. The current 2.0s cache expires before the next valid corrected TF arrives, causing fallback to raw VIO which is then suppressed. A 4.0s cache bridges longer latency gaps.

---

## Verification Criteria

- [ ] Arm VIO correction yield > 5% (currently 0.1%) in a test run
- [ ] `rotation_hard` rejections < 10 per run (currently 105)
- [ ] `chi2` rejections mean < 30 (currently 98.22)
- [ ] Dual-view fusion ratio > 50% (currently drops to 0%)
- [ ] TF fail count per camera < 20 over a 90s run (currently 60-103)
- [ ] DIAG-PC section in analysis report shows actual rates (not all zeros)
- [ ] Arm `pos_norm` converges to physically correct value (within 0.1m of expected distance)
- [ ] Head-arm relative distance std < 0.1m (currently 0.36m)

---

## Potential Risks and Mitigations

1. **solvePnP ambiguity resolution may not fully eliminate flips**
   Mitigation: The adaptive rotation threshold (1.3) provides a secondary escape hatch. If ambiguity resolution fails, the widened threshold allows corrections through during catastrophic divergence.

2. **TF latest-fallback may introduce subtle geometric misalignment**
   Mitigation: Log all fallback events (`tf_fallback` counter). The fallback only activates when stamp-specific lookup fails — in healthy operation it never triggers. A 100-200ms stale TF is far better than dropping the cloud.

3. **Widened chi2 gate may allow bad corrections during normal operation**
   Mitigation: The widened gate only activates when VIO is invalid AND position divergence exceeds threshold. During valid VIO operation, the standard gate applies unchanged.

4. **Increasing TF lookup timeout to 0.5s may delay processing pipeline**
   Mitigation: The lookup runs in a background daemon thread (`_timer_merge` spawns threads at line 508), so the executor is never blocked. The 15Hz timer continues receiving clouds and TF updates.

---

## Alternative Approaches

1. **Use `SOLVEPNP_IPPE` for planar markers instead of `SOLVEPNP_ITERATIVE`**: OpenCV's IPPE solver is specifically designed for planar targets and returns both solutions with their reprojection errors, allowing explicit ambiguity resolution. This is cleaner than post-hoc flip detection but requires verifying Jetson OpenCV 4.6.0 compatibility.

2. **Switch fusion to `require_both_cameras=True` with synchronizer**: This would guarantee dual-view output by construction, but at the cost of output rate (drops to the slowest camera rate minus jitter). Given the 2.3s transport latency and ~3-4Hz camera rates, this would likely produce <1Hz fused output — too slow for real-time grasp control. Not recommended.

3. **Increase cloud_max_age_s from 1.0 to 2.0**: This would keep clouds valid longer, increasing the chance both are fresh simultaneously. However, it doesn't fix the TF lookup failure root cause and would increase temporal misalignment between the two views. The TF fallback (2.1) is the better fix.

4. **Dynamic arm pose measurement (ID2 head-derived path) as primary arm correction**: The `dynamic_arm_pose_measurement_node.py` provides an independent arm pose measurement via the head camera observing a marker on the arm. This bypasses the arm's own solvePnP entirely. Could be promoted to primary correction source for the arm. However, it depends on head VIO stability and head camera seeing the arm marker, adding a new failure mode.
