# VIO Instability & Pointcloud Fusion Issues — Fix Plan (v4.0)

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

**Root cause: Relay corrected-TF processing latency causes a pipeline race — the cloud arrives before its corresponding TF.**

Chrony is working correctly (Jetson drift 0.0–0.2ms). Both cloud stamps and TF stamps are in the same time domain. The problem is that the corrected-TF pipeline is structurally slower than the cloud pipeline due to three compounding issues:

**Issue A — Single-threaded executor deadlock (primary latency source):**

The relay uses a default `SingleThreadedExecutor` (`openvins_odom_tf_relay.py:1000`). The corrected-TF path does a `lookup_transform` with a 0.5s timeout (`openvins_odom_tf_relay.py:485-487`) to read the `*_imu_openvins_corrected` frame from its own TF buffer. But the `TransformListener` that populates that buffer runs on the **same single thread** (`openvins_odom_tf_relay.py:251`, no `spin_thread=True`).

This means: if the corrected frame is not already in the buffer when the lookup starts, the `/tf` subscription callback that would deliver it **cannot fire** during the blocking wait. The 0.5s timeout is a **guaranteed stall**, not a "wait for arrival." Every such stall drops ~100 odom messages per camera (at 200Hz combined stream).

**Issue B — Internal TF2 round-trip:**

The corrected transform makes an unnecessary round-trip through the relay's own TF buffer:
1. `aruco_marker_pose_node` (Jetson) → publishes on `/tf` via DDS
2. Host `TransformListener` `/tf` callback → `buffer.set_transform`
3. Relay `lookup_transform` from same buffer → read back out

Steps 2→3 are a self-inflicted indirection. The data is already in the process but takes a structurally broken path to get back out.

**Issue C — Rate-limiter gates the corrected-TF lookup:**

The corrected-TF block is gated by `tf_allowed` (`openvins_odom_tf_relay.py:479`). At 200Hz odom / 100Hz TF cap, ~50% of messages skip the corrected-TF lookup entirely. When the corrected frame arrives during a rate-limited window, it waits up to 10ms for the next allowed message before being processed.

**Observed impact — the cloud-TF gap grows with CPU pressure:**

| Time in run | Gap (cloud stamp ahead of latest TF) |
|-------------|--------------------------------------|
| T+12s       | 0.207s                               |
| T+22s       | 0.244s                               |
| T+32s       | 0.574s                               |
| T+68s       | 0.819s                               |

The fusion node's 0.2s TF lookup timeout cannot bridge these gaps. Dual-view ratio degrades from 31% to 0% over the run.

**Why existing defenses don't help:**
- The `transform_tolerance_s` parameter (`pointcloud_fusion_node.py:257`) is **dead code** — declared, read, never applied
- The clock-skew auto-fallback only corrects the age-check stamp, not the TF lookup stamp
- The hold-last-good cache in the relay masks stale data but doesn't reduce latency

### Symptom 3 (Discovered): DIAG-PC Analyzer Reports All Zeros

Key-name mismatch: the analyzer looks for `depth_in_hz`, `rgb_in_hz`, `points_hz`, `reason` but the actual DIAG-PC format uses `depth_c`, `image_c`, `points`, and `[OK]` in brackets.

---

## Implementation Plan

### Phase 1: Fix solvePnP Ambiguity at the Source (Symptom 1)

- [ ] **1.1. Switch to `SOLVEPNP_IPPE_SQUARE` in `solve_marker_pose()`**
  - File: `multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py:1155-1195`
  - Replace `cv2.solvePnP(..., flags=cv2.SOLVEPNP_ITERATIVE)` with `cv2.solvePnPGeneric(..., flags=cv2.SOLVEPNP_IPPE_SQUARE)`. IPPE analytically computes both ambiguous solutions for planar targets and returns them with their reprojection errors.
  - `solvePnPGeneric` returns `(retval, rvecs, tvecs, reprojectionErrors)` — a list of solutions instead of a single one. This enables explicit disambiguation rather than hoping the iterative solver picks the right local minimum.
  - Rationale: IPPE was designed specifically for this problem. It eliminates the root cause (iterative solver falling into the wrong local minimum) rather than patching symptoms downstream.
  - Note: Verify corner ordering from `marker_object_points()` (`aruco_marker_pose_node.py:281-294`) matches IPPE_SQUARE's expectation (TL→TR→BR→BL, z=0 plane).

- [ ] **1.2. Add temporal-continuity disambiguation for IPPE solutions**
  - File: same as above, in or near `solve_marker_pose()`
  - When `solvePnPGeneric` returns two solutions, select the one closest to the previous frame's pose. The previous `T_cam_marker` is available in `self.last_marker_measurement.T_cam_marker` (`aruco_marker_pose_node.py:871`) or derivable from `self.marker_histories[id][-1]` (`aruco_marker_pose_node.py:869`).
  - Disambiguation metric: compute the rotation angle delta between each candidate and the previous frame's pose. Select the candidate with the smaller delta.
  - For the first frame (no history), select the solution with lower reprojection error.
  - Rationale: Reprojection error alone cannot reliably disambiguate — both solutions have nearly identical errors. Temporal continuity is the correct disambiguator: the true pose changes smoothly between frames, while the flipped pose represents a physically impossible jump.

- [ ] **1.3. Add `useExtrinsicGuess=True` fallback path**
  - File: same as above, `solve_marker_pose()` function
  - If `SOLVEPNP_IPPE_SQUARE` fails at runtime on the Jetson (the codebase already has OpenCV 4.6.0 segfault workarounds at `aruco_marker_pose_node.py:831-833`), fall back to `SOLVEPNP_ITERATIVE` with `useExtrinsicGuess=True`, seeding with the previous frame's `rvec`/`tvec`.
  - Rationale: Defense in depth. IPPE should work on 4.6.0, but the existing Jetson segfault history warrants a fallback.

- [ ] **1.4. Implement adaptive chi2 gate for recovery mode**
  - File: `aruco_marker_pose_node.py:2000-2018` and `2042-2044`
  - When VIO is invalid AND position divergence exceeds a threshold (e.g., >0.5m), widen the chi2 gate by a configurable multiplier (e.g., 5x).
  - Add parameters `recovery_chi2_gate_multiplier` (default 5.0) and `recovery_pos_divergence_threshold_m` (default 0.5).
  - Rationale: The 68 chi2 rejections (mean=98.22 vs gate=16.81) show the innovation covariance is far too tight when VIO is diverged.

- [ ] **1.5. Normalize arm `calib_cam_timeoffset` to match head config**
  - File: `multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/estimator_config.yaml:14`
  - Change `calib_cam_timeoffset: false` to `calib_cam_timeoffset: true`.
  - Rationale: Eliminates a config asymmetry. The arm has a measured Kalibr timeshift of ~8.8ms that should be refinable online.

### Phase 2: Reduce Corrected-TF Processing Latency (Symptom 2 — Root Cause)

This phase attacks the TF pipeline race at its source by reducing the latency of the relay's corrected-TF path. The goal is to ensure the TF arrives at the fusion node before or simultaneously with the cloud, eliminating the extrapolation-into-future failures.

- [ ] **2.1. Fix single-threaded executor deadlock with `spin_thread=True`**
  - File: `src/camera/camera/openvins_odom_tf_relay.py:251`
  - Change `TransformListener(self._tf_buffer, self)` to `TransformListener(self._tf_buffer, self, spin_thread=True)`.
  - This gives the `/tf` subscription its own background thread and executor, so `set_transform` can populate the buffer and notify the condition variable even while the main executor is blocked inside `lookup_transform`. The 0.5s timeout becomes genuinely functional — it can now catch newly-arrived corrected frames instead of being a guaranteed stall.
  - Rationale: This is the **single most impactful fix** for the TF latency. Currently, if the corrected frame is not already in the buffer when the lookup starts, the 0.5s timeout is a guaranteed stall because the `/tf` callback cannot fire on the same thread. With `spin_thread=True`, the timeout actually waits for and catches the frame, typically returning in <10ms. TF2's `Buffer` is documented as thread-safe for this pattern.
  - Risk: Introduces multi-threaded access to the TF buffer. TF2's `Buffer` is designed for this (it uses internal mutexes), but verify no other code in the relay accesses the buffer without the TF2 API.

- [ ] **2.2. Reduce relay TF publish rate from 100Hz to 20Hz**
  - File: `config/prosthesis_config.yaml` — change `tf_publish_max_hz` to 20.0 (currently not set in config, defaults to 100.0 at `openvins_odom_tf_relay.py:183`)
  - The fusion node consumes clouds at ~5Hz. A 20Hz TF rate provides 4x oversampling — more than sufficient for tf2's linear interpolation between entries. The relay processes odom at ~200Hz per camera; reducing the TF cap from 100Hz to 20Hz means 90% fewer `sendTransform` calls and DDS messages, significantly reducing CPU load on both host and Jetson (via reduced `/tf` topic volume).
  - Rationale: At 100Hz TF and 5Hz clouds, the fusion node's TF lookup interpolates between entries 10ms apart — far more precision than needed. The CPU savings from 20Hz are substantial: ~80 fewer `sendTransform` calls per second per camera, reduced DDS serialization/deserialization, and less `/tf` topic contention. This directly reduces the CPU pressure that causes the growing cloud-TF gap.
  - Side effect: The LPF effective update rate drops from 100Hz to 20Hz. With alpha=0.3, settling time is ~5 frames = 0.25s instead of 0.05s. This is acceptable — reanchor discontinuities are smoothed over 0.25s instead of 0.05s, which is still fast enough for real-time operation. If faster settling is needed, increase alpha to 0.5 (settling in ~3 frames = 0.15s).

- [ ] **2.3. Reduce corrected-TF lookup timeout from 0.5s to 0.15s**
  - File: `src/camera/camera/openvins_odom_tf_relay.py:487`
  - Change `timeout=rclpy.duration.Duration(seconds=0.5)` to `seconds=0.15`.
  - Rationale: With the `spin_thread=True` fix (2.1), the timeout now genuinely waits for the corrected frame. In practice, the corrected frame arrives within a few ms of the odom message (both come from the Jetson via DDS). A 0.15s timeout is generous for this case. If the frame hasn't arrived in 0.15s, the hold-last-good cache (2.4) kicks in immediately instead of stalling for another 0.35s. This reduces worst-case latency from 0.5s to 0.15s.

- [ ] **2.4. Increase hold-last-good cache max age from 2.0s to 4.0s**
  - File: `config/prosthesis_config.yaml` — add `corrected_tf_cache_max_age_s: 4.0` under `openvins_odom_tf_relay`
  - Rationale: Transport latency can spike to 2.3s under CPU pressure. The current 2.0s cache may expire before the next valid corrected TF arrives, causing fallback to raw VIO (which is then suppressed by outlier gates, leaving the TF edge stale). A 4.0s cache bridges longer latency spikes while still being fresh enough for real-time operation.

- [ ] **2.5. Decouple corrected-TF lookup from rate limiter**
  - File: `src/camera/camera/openvins_odom_tf_relay.py:479`
  - Currently the corrected-TF lookup only happens when `tf_allowed` is True (i.e., not rate-limited). At 20Hz TF cap (after 2.2), this means the lookup happens every 50ms — and corrected frames arriving between those intervals are not processed until the next allowed slot.
  - Change: Always perform the corrected-TF lookup and cache update (even when rate-limited), but only broadcast when `tf_allowed`. This ensures the LPF and cache always have the latest data, and the broadcast (when it happens) uses the most recent corrected transform.
  - Rationale: Decoupling ingestion from broadcasting ensures the internal state (LPF, cache) is always current, reducing latency when a broadcast does happen. The rate limiter should gate the DDS broadcast, not the data ingestion.

### Phase 3: Fix Fusion Node TF Lookup (Symptom 2 — Consumer Side)

- [ ] **3.1. Wire up `transform_tolerance_s` and increase effective timeout**
  - File: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:532-537`
  - The parameter is declared (`:257`), read (`:295-296`), but never used. Wire it into the lookup: change the timeout to `rclpy.duration.Duration(seconds=self._transform_tolerance)`. Set `transform_tolerance_s` to 1.0s in config.
  - This runs in a background daemon thread (`:508`), so it does not block the executor.
  - Rationale: Even with Phase 2 latency reductions, residual transport jitter means the cloud may still arrive slightly before TF. A 1.0s timeout gives the TF time to arrive. The alternative (dropping the cloud) produces worse fusion results than a slightly-delayed transform.

- [ ] **3.2. Make TF lookup use skew-corrected timestamp as fallback**
  - File: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:513-546`
  - When `_clock_skew_detected` is True, use the arrival-time stamp for the TF lookup instead of `cloud.header.stamp`. Currently the skew correction only applies to the age check.
  - Rationale: Defense in depth for transient latency spikes that trip the skew detector.

- [ ] **3.3. Add dual-view ratio metric and degradation warning**
  - File: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:1004-1041`
  - Compute `dual_ratio = dual / max(dual + cam1_only, 1)`. Emit WARN if below 0.3 for two consecutive intervals.
  - Rationale: Makes the progressive degradation visible in real-time.

### Phase 4: Fix DIAG-PC Analyzer (Symptom 3)

- [ ] **4.1. Fix key-name mapping in `analyze_log.py` DIAG-PC parser**
  - File: `scripts/analyze_log.py:784-789`
  - Update to use actual DIAG-PC key names: `depth_c`, `image_c`, `points`, parse `[stage]` bracket for reason.
  - Also surface `depth_r`, `image_r`, `ci_raw`.

- [ ] **4.2. Add dual-view ratio extraction to analyzer**
  - File: `scripts/analyze_log.py`
  - Parse fusion node `Stats: published=N (dual=X, cam1_only=Y)` lines and report dual-view ratio.

### Phase 5: Verify and Stabilize

- [ ] **5.1. Verify corrected TF stability after Phase 1 fixes**
  - The 1185 TF jumps on `marker_map -> arm_imu_openvins_corrected` are caused by the solvePnP flip trap. After fixing ambiguity resolution, verify jump count drops significantly.

- [ ] **5.2. Monitor CPU pressure and cloud-TF gap trends**
  - After Phase 2 changes (20Hz TF, `spin_thread=True`, reduced timeout), verify the cloud-TF gap stays below 0.2s and dual-view ratio stays above 50%.

---

## Verification Criteria

- [ ] Arm VIO correction yield > 5% (currently 0.1%)
- [ ] `rotation_hard` rejections < 10 per run (currently 105)
- [ ] `chi2` rejections mean < 30 (currently 98.22)
- [ ] Dual-view fusion ratio > 50% (currently drops to 0%)
- [ ] TF fail count per camera < 20 over a 90s run (currently 60-103)
- [ ] Cloud-TF gap stays below 0.3s throughout run (currently grows to 0.8s)
- [ ] DIAG-PC section in analysis report shows actual rates (not all zeros)
- [ ] Arm `pos_norm` converges to physically correct value
- [ ] Head-arm relative distance std < 0.1m (currently 0.36m)
- [ ] `transform_tolerance_s` parameter is applied to TF lookups
- [ ] Relay TF rate at 20Hz with no DDS storms

---

## Potential Risks and Mitigations

1. **`SOLVEPNP_IPPE_SQUARE` may segfault on Jetson OpenCV 4.6.0**
   Mitigation: Runtime-verify before committing. Fallback to `useExtrinsicGuess=True` with iterative solver (task 1.3).

2. **`spin_thread=True` introduces multi-threaded buffer access**
   Mitigation: TF2's `Buffer` is designed for concurrent access (internal mutexes). The relay only accesses the buffer via the TF2 API (`lookup_transform`, `can_transform`). No raw internal state is touched.

3. **20Hz TF rate may cause visible RViz jitter**
   Mitigation: At 20Hz, TF entries are 50ms apart. tf2 interpolation between entries is seamless for visualization. If RViz jitter is noticeable, increase to 30Hz (still 70% reduction from 100Hz).

4. **Longer TF lookup timeout may delay fusion output**
   Mitigation: The lookup runs in a background daemon thread. The 15Hz timer continues receiving clouds and TF. A 1.0s timeout only delays the fusion output for that cloud — far better than dropping it.

5. **Adaptive chi2 gate may allow bad corrections during normal operation**
   Mitigation: Only activates when VIO is invalid AND position divergence exceeds threshold.

---

## Alternative Approaches

1. **Topic-based corrected pose (medium-term, requires Jetson-side change):** Have `aruco_marker_pose_node.py` publish the corrected pose on a dedicated topic (e.g., `/jetson/head/corrected_odom`). The relay subscribes directly, eliminating the internal TF2 round-trip and blocking lookup entirely. This is the cleanest architecture but requires cross-repo coordination. The `spin_thread=True` fix (2.1) addresses the immediate blocking issue; this alternative can be pursued as a follow-up.

2. **Use `useExtrinsicGuess=True` only (skip IPPE):** Simpler, lower risk on Jetson. Less robust than IPPE but avoids any OpenCV version concerns. This is the fallback path (task 1.3).

3. **Dynamic arm pose measurement (ID2) as primary arm correction:** Bypasses arm's own solvePnP entirely via head camera observing arm-mounted marker. Depends on head VIO stability and head camera seeing the arm marker.

4. **MultiThreadedExecutor for the relay:** Alternative to `spin_thread=True` — use a `MultiThreadedExecutor` with `ReentrantCallbackGroup` so all callbacks can run concurrently. More invasive than `spin_thread=True` but provides broader parallelism. Not recommended for now — `spin_thread=True` solves the specific deadlock with minimal change.
