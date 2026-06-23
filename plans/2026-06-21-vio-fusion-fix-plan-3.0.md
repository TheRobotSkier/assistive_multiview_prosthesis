# VIO Instability & Pointcloud Fusion Issues — Fix Plan (v3.0)

## Objective

Resolve two user-reported symptoms — (1) arm stabilizing at a wrong position away from the marker, and (2) fused point clouds frequently showing only one view instead of both — by addressing their actual root causes identified through log/bag analysis and code investigation.

---

## Root Cause Analysis

### Symptom 1: Arm Stabilizing Away From Marker

**Root cause: Arm VIO solvePnP ambiguity flip trap.**

The arm camera's ArUco correction uses `cv2.solvePnP` with `SOLVEPNP_ITERATIVE` (`aruco_marker_pose_node.py:1163-1169`). For planar markers, this solver is susceptible to the well-known **twofold pose ambiguity**: two distinct 3D orientations produce nearly identical 2D corner projections. Without a good initial guess, the solver randomly converges to the "flipped" solution.

The current code has **no in-solver ambiguity resolution** — only a downstream 90° rotation plausibility wall (`aruco_marker_pose_node.py:1888-1902`) that rejects the flipped solution. This creates a trap state:

1. Arm VIO diverges slightly → marked invalid
2. Fallback path activates (bypasses quality gates)
3. `SOLVEPNP_ITERATIVE` returns the flipped pose (~122° rotation innovation)
4. The 90° wall correctly blocks it
5. `hold_vio_invalid()` extends the invalid window → no correction gets through → loop repeats

**Evidence:** Arm VIO yield 0.1% (1 correction in 817 detections), 105 `rotation_hard` rejections with mean=121.72° (systematic flip signature). Head VIO yield 3.1% — head's solvePnP happens to converge to the correct solution.

**Key code facts:**
- `useExtrinsicGuess` is **never used** — every solve starts from scratch
- Extensive temporal pose history exists (`marker_histories`, `last_marker_measurement`) but is **not fed back** into solvePnP
- `SOLVEPNP_IPPE_SQUARE` (designed for planar square targets, returns both solutions) is **available** in OpenCV 4.6.0
- `cv2.solvePnPGeneric` (returns all candidate solutions with reprojection errors) is also available

### Symptom 2: Fused Point Clouds Showing Only One View

**Root cause: Processing pipeline race — cloud arrives before its corresponding TF.**

Chrony is working correctly in this run (Jetson drift 0.0–0.2ms per sysmon). Both the cloud header stamps and the TF stamps are in the same time domain (Jetson system clock). The problem is that the cloud reaches the fusion node **before** its corresponding TF entry has been processed and broadcast.

**Why the TF pipeline is slower than the cloud pipeline:**

- **Cloud path:** depth compressed → decompress bridge → assembler → fusion node. Relatively lightweight processing.
- **TF path:** odom → relay corrected-TF lookup (0.5s timeout to look up `*_openvins_corrected` frame) → LPF smoothing → rate limiting (100Hz) → TF broadcast. The corrected-TF path has an additional TF lookup *inside* the relay before it can publish, adding latency.

The actual TF failure messages confirm this — all are "extrapolation into the future" with small, growing gaps:

| Timestamp | Requested (cloud stamp) | Latest TF | Gap |
|-----------|------------------------|-----------|-----|
| T+12s     | 1781992406.026         | 1781992405.819 | 0.207s |
| T+22s     | 1781992413.245         | 1781992413.001 | 0.244s |
| T+32s     | 1781992424.524         | 1781992423.950 | 0.574s |
| T+68s     | 1781992460.664         | 1781992459.845 | 0.819s |

The gap grows over the run as CPU pressure increases (Jetson at 85–97% avg, host at 71% avg). The fusion node's 0.2s TF lookup timeout cannot bridge gaps that reach 0.6–0.8s.

**Why existing defenses don't help:**
- The `transform_tolerance_s` parameter (declared at `pointcloud_fusion_node.py:257`, read at `:295-296`) is **dead code** — never applied to any lookup
- The clock-skew auto-fallback (`_stamp_from_msg`, `:440-470`) only corrects the **age check** stamp, not the TF lookup stamp at line 532 (which reads `cloud.header.stamp` directly). It triggered once near the end of the run when transport latency spiked to >5s, but this is a symptom of CPU saturation, not chrony failure
- The relay's `_resolve_tf_stamp` 2.0s guard (`openvins_odom_tf_relay.py:695`) also triggered once at T+73s — same CPU saturation spike, not a chrony issue

**Evidence from fusion node stats (progressive degradation):**

| Run time | Dual-view | Cam1-only | tf_fail (head/arm) |
|----------|-----------|-----------|---------------------|
| +40s     | 11        | 24        | 77/76              |
| +60s     | 3         | 29        | 97/76              |
| +80s     | **0**     | 52        | 103/71             |

### Symptom 3 (Discovered): DIAG-PC Analyzer Reports All Zeros

The analysis report shows `head: depth=0.0Hz ... reason=unknown` despite the raw log showing healthy data. This is a **key-name mismatch bug**: the analyzer looks for `depth_in_hz`, `rgb_in_hz`, `points_hz`, `reason` but the actual DIAG-PC format uses `depth_c`, `image_c`, `points`, and `[OK]` in brackets.

---

## Implementation Plan

### Phase 1: Fix solvePnP Ambiguity at the Source (Symptom 1)

- [ ] **1.1. Switch to `SOLVEPNP_IPPE_SQUARE` in `solve_marker_pose()`**
  - File: `multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py:1155-1195`
  - Replace `cv2.solvePnP(..., flags=cv2.SOLVEPNP_ITERATIVE)` with `cv2.solvePnPGeneric(..., flags=cv2.SOLVEPNP_IPPE_SQUARE)`. IPPE analytically computes both ambiguous solutions for planar targets and returns them with their reprojection errors.
  - `solvePnPGeneric` returns `(retval, rvecs, tvecs, reprojectionErrors)` — a list of solutions instead of a single one. This enables explicit disambiguation rather than hoping the iterative solver picks the right local minimum.
  - Rationale: IPPE was designed specifically for this problem. It eliminates the root cause (iterative solver falling into the wrong local minimum) rather than patching symptoms downstream. `SOLVEPNP_IPPE_SQUARE` has been in OpenCV since 3.4.2 and is confirmed available in the Jetson's OpenCV 4.6.0.
  - Note: Verify the corner ordering from `marker_object_points()` (`aruco_marker_pose_node.py:281-294`) matches IPPE_SQUARE's expectation (TL→TR→BR→BL, z=0 plane). The current ordering should be correct but must be verified at runtime.

- [ ] **1.2. Add temporal-continuity disambiguation for IPPE solutions**
  - File: same as above, in or near `solve_marker_pose()`
  - When `solvePnPGeneric` returns two solutions, select the one closest to the previous frame's pose using temporal continuity. The previous `T_cam_marker` is available in `self.last_marker_measurement.T_cam_marker` (`aruco_marker_pose_node.py:871`) or derivable from `self.marker_histories[id][-1]` (`aruco_marker_pose_node.py:869`).
  - Disambiguation metric: compute the rotation angle delta between each candidate solution and the previous frame's pose. Select the candidate with the smaller delta.
  - For the first frame (no history), select the solution with lower reprojection error (standard IPPE behavior — IPPE's own sorting is usually correct for the first frame).
  - Rationale: Reprojection error alone cannot reliably disambiguate — both solutions have nearly identical errors. Temporal continuity is the correct disambiguator: the true pose changes smoothly between frames, while the flipped pose represents a physically impossible jump.

- [ ] **1.3. Add `useExtrinsicGuess=True` fallback path**
  - File: same as above, `solve_marker_pose()` function
  - If `SOLVEPNP_IPPE_SQUARE` fails at runtime on the Jetson (the codebase already has OpenCV 4.6.0 segfault workarounds at `aruco_marker_pose_node.py:831-833`), fall back to `SOLVEPNP_ITERATIVE` with `useExtrinsicGuess=True`, seeding with the previous frame's `rvec`/`tvec`.
  - This biases the iterative solver toward the temporally-consistent local minimum, preventing it from hopping to the flipped solution.
  - Rationale: Defense in depth. IPPE should work on 4.6.0, but the existing Jetson segfault history warrants a fallback. Seeding with the previous pose is the industry-standard approach for iterative solvers on planar targets.

- [ ] **1.4. Implement adaptive chi2 gate for recovery mode**
  - File: `aruco_marker_pose_node.py:2000-2018` and `2042-2044`
  - When VIO is invalid AND position divergence exceeds a threshold (e.g., >0.5m), widen the chi2 gate by a configurable multiplier (e.g., 5x). This allows corrections through when the estimator is catastrophically diverged.
  - Add parameters `recovery_chi2_gate_multiplier` (default 5.0) and `recovery_pos_divergence_threshold_m` (default 0.5).
  - Rationale: The 68 chi2 rejections (mean=98.22 vs gate=16.81) show the innovation covariance is far too tight relative to actual innovations when VIO is diverged. Even with ambiguity resolution (1.1-1.3), the covariance model may still reject valid corrections during recovery.

- [ ] **1.5. Normalize arm `calib_cam_timeoffset` to match head config**
  - File: `multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/estimator_config.yaml:14`
  - Change `calib_cam_timeoffset: false` to `calib_cam_timeoffset: true` to match head config.
  - Rationale: The arm has a measured Kalibr timeshift of ~8.8ms. Freezing it means residual timing drift cannot be corrected online. Minor contributor but eliminates a config asymmetry.

### Phase 2: Fix TF Pipeline Race (Symptom 2)

- [ ] **2.1. Increase TF lookup timeout and wire up `transform_tolerance_s`**
  - File: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:532-537`
  - The current 0.2s timeout is insufficient — actual gaps reach 0.6–0.8s under CPU pressure. Wire up the dead `transform_tolerance_s` parameter (currently declared and read but never used) and increase the effective timeout.
  - Change the lookup to use `timeout=rclpy.duration.Duration(seconds=self._transform_tolerance)` where `transform_tolerance_s` is set to 1.0s in config (currently 0.5s in config but unused). This runs in a background daemon thread so it does not block the executor.
  - Rationale: The cloud and TF are in the same time domain. The cloud simply arrives before the TF has been processed. Waiting longer for the TF to arrive is the correct fix — the alternative (dropping the cloud) produces worse results than a slightly-delayed transform.

- [ ] **2.2. Make TF lookup use the skew-corrected timestamp as fallback**
  - File: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:513-546`
  - Currently the TF lookup at line 532 reads `cloud.header.stamp` directly, bypassing the `_stamp_from_msg()` skew correction. While chrony is working in the current run, the skew fallback was triggered once at T+73s when transport latency spiked to >5s.
  - When `_clock_skew_detected` is True, use the arrival-time stamp for the TF lookup instead of the header stamp. This ensures the lookup searches in the correct time domain even during transient latency spikes.
  - Rationale: Defense in depth. The primary fix (2.1) handles the normal case. This fix handles the edge case where transport latency spikes cause the skew detector to trip, ensuring the TF lookup doesn't search in the wrong time domain.

- [ ] **2.3. Investigate reducing relay corrected-TF processing latency**
  - File: `src/camera/camera/openvins_odom_tf_relay.py:479-538`
  - The corrected-TF path does its own TF lookup with a 0.5s timeout (`openvins_odom_tf_relay.py:487`) before rebroadcasting. Under CPU pressure this adds significant latency to the TF pipeline, causing the cloud to arrive first at the fusion node.
  - Consider reducing the relay's corrected-TF lookup timeout from 0.5s to 0.2s (the corrected frame should be available immediately since it's published by the Jetson-side aruco node via the same DDS layer). If the lookup fails faster, the hold-last-good cache kicks in sooner, reducing overall TF latency.
  - Rationale: The TF pipeline is slower than the cloud pipeline partly because of this internal lookup. Reducing its latency helps the TF arrive before the cloud.

- [ ] **2.4. Add dual-view ratio metric and degradation warning**
  - File: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:1004-1041` (`_log_stats`)
  - Compute `dual_ratio = dual / max(dual + cam1_only, 1)` and include it in the stats log. Emit a WARN if dual_ratio drops below a configurable threshold (default 0.3) for two consecutive intervals.
  - Rationale: The progressive degradation from 31% to 0% dual-view was invisible in real-time. This metric directly answers "are we getting single-view fusion?" and enables early detection.

### Phase 3: Fix DIAG-PC Analyzer (Symptom 3)

- [ ] **3.1. Fix key-name mapping in `analyze_log.py` DIAG-PC parser**
  - File: `scripts/analyze_log.py:784-789`
  - Update `print_pointcloud_chain_health_log()` to use the actual DIAG-PC key names emitted by `pipeline_diagnostics_node.py:606-611`: `depth_c` (not `depth_in_hz`), `image_c` (not `rgb_in_hz`), `points` (not `points_hz`), and parse the `[stage]` bracket for the reason instead of looking for a `reason` key.
  - Also surface `depth_r`, `image_r`, `ci_raw` for completeness.
  - Rationale: The analyzer is the operator's primary diagnostic surface. A blind pointcloud health section makes it impossible to diagnose fusion issues from the report.

- [ ] **3.2. Add dual-view ratio extraction from fusion node stats to analyzer**
  - File: `scripts/analyze_log.py`
  - Parse the fusion node's `Stats: published=N (dual=X, cam1_only=Y)` lines and report the dual-view ratio over time.
  - Rationale: This metric directly answers "are we getting single-view fusion?" and should be a first-class metric in the report.

### Phase 4: Verify and Stabilize

- [ ] **4.1. Verify corrected TF stability after Phase 1 fixes**
  - The analysis shows 1185 TF jumps on `marker_map -> arm_imu_openvins_corrected`. These are caused by the solvePnP flip trap (Phase 1). After fixing the ambiguity resolution, verify that corrected TF jump count drops significantly.
  - If jumps persist, investigate whether the relay's LPF alpha (0.3) introduces discontinuities and consider reducing to 0.15 or adding a jump detector that holds last-good-value during large discontinuities.
  - Rationale: The corrected TF path is the primary TF source for all downstream consumers. Its stability directly determines fusion quality and proximity controller accuracy.

- [ ] **4.2. Monitor CPU pressure and transport latency trends**
  - The growing TF-cloud gap (0.2s → 0.8s) correlates with Jetson CPU at 85–97%. If CPU pressure is the underlying driver, investigate whether the Jetson relay's throttle rates or the OpenVINS processing load can be reduced.
  - The relay rate-limits TF to 100Hz (`openvins_odom_tf_relay.py:183`) — at 5Hz cloud rate, even 20Hz TF would be sufficient and would reduce CPU load.
  - Rationale: Addressing CPU pressure at the source reduces the transport latency that causes the pipeline race, complementing the timeout increase (2.1).

---

## Verification Criteria

- [ ] Arm VIO correction yield > 5% (currently 0.1%)
- [ ] `rotation_hard` rejections < 10 per run (currently 105)
- [ ] `chi2` rejections mean < 30 (currently 98.22)
- [ ] Dual-view fusion ratio > 50% (currently drops to 0%)
- [ ] TF fail count per camera < 20 over a 90s run (currently 60-103)
- [ ] DIAG-PC section in analysis report shows actual rates (not all zeros)
- [ ] Arm `pos_norm` converges to physically correct value
- [ ] Head-arm relative distance std < 0.1m (currently 0.36m)
- [ ] `transform_tolerance_s` parameter is applied to TF lookups

---

## Potential Risks and Mitigations

1. **`SOLVEPNP_IPPE_SQUARE` may segfault on Jetson OpenCV 4.6.0**
   Mitigation: The codebase already has Jetson-specific OpenCV workarounds (no `DetectorParameters` due to segfault). Runtime-verify IPPE availability before committing (`python3 -c "import cv2; print(hasattr(cv2, 'SOLVEPNP_IPPE_SQUARE'))"`). Fallback to `useExtrinsicGuess=True` with iterative solver (task 1.3) if IPPE fails.

2. **Longer TF lookup timeout may delay fusion output**
   Mitigation: The lookup runs in a background daemon thread (`pointcloud_fusion_node.py:508`), so the executor is never blocked. The 15Hz timer continues receiving clouds and TF updates. A 1.0s timeout only means the fusion output for that cloud is delayed by up to 1.0s — far better than dropping the cloud entirely.

3. **Adaptive chi2 gate may allow bad corrections during normal operation**
   Mitigation: The widened gate only activates when VIO is invalid AND position divergence exceeds threshold. During valid VIO operation, the standard gate applies unchanged.

4. **Temporal disambiguation may fail during fast motion**
   Mitigation: During fast motion, the rotation delta between frames is larger, but the flipped solution still represents a ~180° equivalent jump — far larger than any real motion. The disambiguator threshold can be tuned (e.g., always prefer the solution with <90° delta from previous).

---

## Alternative Approaches

1. **Use `useExtrinsicGuess=True` only (skip IPPE)**: Simpler change — just seed the iterative solver with the previous frame's pose. Less robust than IPPE (the solver can still escape the basin of attraction under noise) but lower risk on the Jetson platform. This is the fallback path (task 1.3) and could be promoted to primary if IPPE proves problematic.

2. **Reduce relay TF rate from 100Hz to 20Hz**: Since clouds arrive at 5Hz, a 20Hz TF rate provides 4x oversampling — sufficient for interpolation while halving the relay's CPU load. This would reduce the processing latency that contributes to the pipeline race. Lower risk than changing the timeout but doesn't fully solve the problem.

3. **Dynamic arm pose measurement (ID2) as primary arm correction**: The `dynamic_arm_pose_measurement_node.py` provides an independent arm pose measurement via the head camera observing a marker on the arm, bypassing the arm's own solvePnP entirely. Could be promoted to primary correction source. However, it depends on head VIO stability and head camera seeing the arm marker, adding new failure modes.

4. **Reorder relay processing**: Move the corrected-TF lookup *before* the rate-limit check, so the lookup starts earlier and the broadcast happens sooner. Currently the rate-limit check (`openvins_odom_tf_relay.py:458`) gates whether the corrected-TF block even runs (`:479`). If the rate limiter blocks, the corrected-TF lookup doesn't happen until the next allowed frame, adding up to 10ms delay.
