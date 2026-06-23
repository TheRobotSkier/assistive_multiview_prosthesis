# VIO Instability & Pointcloud Fusion Issues — Fix Plan (v2.0)

## Objective

Resolve two user-reported symptoms — (1) arm stabilizing at a wrong position away from the marker, and (2) fused point clouds frequently showing only one view instead of both — by addressing their actual root causes identified through deep code investigation and log/bag analysis.

---

## Root Cause Analysis (Revised)

### Symptom 1: Arm Stabilizing Away From Marker

**Root cause: Arm VIO solvePnP ambiguity flip trap.**

The arm camera's ArUco correction uses `cv2.solvePnP` with `SOLVEPNP_ITERATIVE` (`aruco_marker_pose_node.py:1163-1169`). For planar markers, this solver is susceptible to the well-known **twofold pose ambiguity**: two distinct 3D orientations produce nearly identical 2D corner projections. Without a good initial guess, the solver randomly converges to the "flipped" solution.

The current code has **no in-solver ambiguity resolution**. Instead, a 90° rotation plausibility wall (`aruco_marker_pose_node.py:1888-1902`) rejects the flipped solution downstream. This creates a trap state:

1. Arm VIO diverges slightly → marked invalid
2. Fallback path activates (bypasses quality gates)
3. `SOLVEPNP_ITERATIVE` returns the flipped pose (~122° rotation innovation)
4. The 90° wall correctly blocks it
5. `hold_vio_invalid()` extends the invalid window → no correction gets through → loop repeats

**Evidence:** Arm VIO yield 0.1% (1 correction in 817 detections), 105 `rotation_hard` rejections with mean=121.72° (systematic flip signature). Head VIO yield 3.1% — head's solvePnP happens to converge to the correct solution.

**Key code facts:**
- `useExtrinsicGuess` is **never used** anywhere in the codebase — every solve starts from scratch
- Extensive temporal pose history exists (`marker_histories`, `last_marker_measurement`) but is **not fed back** into solvePnP as an initial guess
- `SOLVEPNP_IPPE_SQUARE` (designed specifically for planar square targets, returns both solutions) is **available** in OpenCV 4.6.0 on the Jetson
- `cv2.solvePnPGeneric` (available since OpenCV 4.2) returns all candidate solutions with reprojection errors

### Symptom 2: Fused Point Clouds Showing Only One View

**Root cause: Clock-domain split between TF stamps and cloud stamps.**

This is NOT a transport latency or timing race issue. It is a **chrony failure** creating a ~56460-second constant offset between Jetson and host clocks.

The timestamp lifecycle reveals the split:

1. **Jetson side:** Depth images and odom are stamped with Jetson system clock (T_jetson)
2. **Host relay** (`openvins_odom_tf_relay.py:676-702`): `_resolve_tf_stamp()` detects `|T_jetson - T_host| > 2.0s`, falls back to **host clock** for TF stamps
3. **Host decompress bridge** (`decompress_bridge.py:175-176`): Passes Jetson stamp through **unchanged** to the assembler
4. **Host assembler** (`naive_pointcloud_assembler.py:350-354`): Copies depth header (T_jetson) to the output cloud
5. **Host fusion node** (`pointcloud_fusion_node.py:532-537`): Looks up TF at `cloud.header.stamp` (T_jetson) — but the TF buffer only contains entries at T_host

Result: The fusion node searches for a TF entry ~56460 seconds away from any cached entry. The lookup always fails, the cloud is silently dropped, and `tf_fail[frame]` increments.

**Why existing defenses don't catch it:**
- The fusion node's clock-skew detector (`_stamp_from_msg`, `pointcloud_fusion_node.py:440-470`) correctly detects the skew and falls back to arrival-time for the **age check** — but the **TF lookup** at line 532 reads `cloud.header.stamp` directly, bypassing the skew correction entirely
- The `transform_tolerance_s` parameter is declared and read but **never applied** anywhere — it's dead code
- The relay "fixes" the TF side while leaving the cloud side untouched, guaranteeing the mismatch

**Evidence from fusion node stats (progressive degradation):**

| Run time | Dual-view | Cam1-only | tf_fail (head/arm) |
|----------|-----------|-----------|---------------------|
| +40s     | 11        | 24        | 77/76              |
| +60s     | 3         | 29        | 97/76              |
| +80s     | **0**     | 52        | 103/71             |

The first successful transforms only happen because the relay occasionally publishes a TF close enough to a cloud stamp by coincidence during the startup convergence window. Once the clock offset stabilizes at ~56460s, the gap becomes consistent and TF lookups fail systematically.

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
  - Rationale: As the user's context correctly notes, reprojection error alone cannot reliably disambiguate — both solutions have nearly identical errors. Temporal continuity is the correct disambiguator: the true pose changes smoothly between frames, while the flipped pose represents a physically impossible jump.

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

### Phase 2: Fix Clock-Domain Split in Fusion TF Lookup (Symptom 2)

- [ ] **2.1. Make TF lookup use the skew-corrected timestamp**
  - File: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:513-546`
  - The `_stamp_from_msg()` method (`pointcloud_fusion_node.py:440-470`) already computes a skew-corrected timestamp for the age check. When `_clock_skew_detected` is True, it returns the host arrival time. But the TF lookup at line 532 reads `cloud.header.stamp` directly, bypassing this correction.
  - Fix: Store the skew-corrected stamp on the cloud object (or pass it alongside) so the TF lookup uses the same corrected timestamp. When clock skew is detected, use the arrival-time stamp for TF lookup instead of the header stamp. When clocks are synced (chrony working), behavior is unchanged.
  - Rationale: This is the **actual root cause fix**. The TF buffer contains entries at T_host (because the relay falls back to host clock). The cloud header is at T_jetson. Using the corrected stamp aligns the lookup with the TF buffer's time domain.

- [ ] **2.2. Fix the relay's `_resolve_tf_stamp` fallback policy**
  - File: `src/camera/camera/openvins_odom_tf_relay.py:676-702`
  - The current 2.0s sanity guard falls back to host clock when chrony is broken, creating the clock-domain split. Two options:
    - **Option A (preferred):** Keep the odom stamp (T_jetson) so TF shares the cloud's time domain. The fusion node's skew correction (2.1) then handles both consistently. Risk: downstream consumers that assume TF is in host-clock domain may break.
    - **Option B:** Also restamp depth/cloud headers downstream when the fallback triggers. More invasive, touches the decompress bridge.
  - Evaluate both options. Option A is preferred because it keeps the time domain consistent end-to-end and the fusion node fix (2.1) already handles the skew.
  - Rationale: The relay's fallback was designed to prevent "implausible" TF stamps, but it inadvertently creates a worse problem: a clock-domain split that makes TF lookups fail entirely. The right fix is to keep all data in one time domain and handle the offset at the consumer.

- [ ] **2.3. Wire up the dead `transform_tolerance_s` parameter**
  - File: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:252-257, 295-296, 536`
  - The parameter is declared, read, and documented but never used. Either wire it into the `lookup_transform` timeout (add it to the existing `Duration(seconds=0.2)`) or remove it to avoid confusion.
  - If kept: change line 536 to `timeout=rclpy.duration.Duration(seconds=0.2 + self._transform_tolerance)`. This gives the TF buffer extra time to interpolate when the cloud stamp falls between two TF entries.
  - Rationale: Dead parameters are misleading. This one was intended to solve exactly the "cloud arrives before TF" race but was never connected.

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

- [ ] **4.2. Verify chrony status and clock offset in next run**
  - Check `clock_offset_peak_s` in the latest log metrics to confirm whether the ~56460s offset persists. If chrony can be fixed, the relay's 2.0s guard works correctly and the clock-domain split disappears naturally.
  - However, the fusion node fix (2.1) should be implemented regardless as a defense against future chrony failures.
  - Rationale: Chrony is the intended time synchronization mechanism. If it's broken, fixing it eliminates the root cause. But defense-in-depth requires the code to handle chrony failure gracefully.

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
- [ ] `transform_tolerance_s` parameter is either applied or removed

---

## Potential Risks and Mitigations

1. **`SOLVEPNP_IPPE_SQUARE` may segfault on Jetson OpenCV 4.6.0**
   Mitigation: The codebase already has Jetson-specific OpenCV workarounds (no `DetectorParameters` due to segfault). Runtime-verify IPPE availability before committing. Fallback to `useExtrinsicGuess=True` with iterative solver (task 1.3) if IPPE fails.

2. **Keeping odom stamp in relay TF (Option A in 2.2) may break downstream consumers**
   Mitigation: Audit all TF consumers (proximity controller, twist propagation, diagnostics) for clock-domain assumptions. The proximity controller uses `rclpy.time.Time()` (latest) so it's unaffected. If issues arise, Option B (restamp clouds downstream) is the alternative.

3. **Adaptive chi2 gate may allow bad corrections during normal operation**
   Mitigation: The widened gate only activates when VIO is invalid AND position divergence exceeds threshold. During valid VIO operation, the standard gate applies unchanged.

4. **Temporal disambiguation may fail during fast motion**
   Mitigation: During fast motion, the rotation delta between frames is larger, but the flipped solution still represents a ~180° equivalent jump — far larger than any real motion. The disambiguator threshold can be tuned (e.g., always prefer the solution with <90° delta from previous).

---

## Alternative Approaches

1. **Fix chrony instead of code changes**: If the ~56460s offset is purely a chrony configuration issue, fixing chrony eliminates the clock-domain split naturally. However, the code should still handle chrony failure gracefully (Phase 2 fixes), and the solvePnP issue (Phase 1) is independent of chrony.

2. **Use `useExtrinsicGuess=True` only (skip IPPE)**: Simpler change — just seed the iterative solver with the previous frame's pose. Less robust than IPPE (the solver can still escape the basin of attraction under noise) but lower risk on the Jetson platform. This is the fallback path (task 1.3) and could be promoted to primary if IPPE proves problematic.

3. **Dynamic arm pose measurement (ID2) as primary arm correction**: The `dynamic_arm_pose_measurement_node.py` provides an independent arm pose measurement via the head camera observing a marker on the arm, bypassing the arm's own solvePnP entirely. Could be promoted to primary correction source. However, it depends on head VIO stability and head camera seeing the arm marker, adding new failure modes.

4. **Restamp cloud headers in decompress bridge**: Instead of fixing the fusion node's TF lookup (2.1), restamp all depth/image headers with host clock in the decompress bridge. This is more invasive (changes the timestamp domain for all downstream consumers) but ensures consistency. Less surgical than the fusion node fix.
