# Pointcloud Fusion & VIO Stability — Issue Resolution Plan (v2)

## Objective

Resolve three interconnected issues observed in the v6 run (20260620_165849):
1. **Single-view fusion** — fused point cloud often only includes one camera view
2. **Slow point clouds** — clouds arrive slowly / at low rate
3. **Jumping** — visible discontinuities in the point cloud / RViz, possibly from OpenVINS or timestamp issues

## Root Cause Analysis

### Issue 1: Single-View Fusion — TF Chain Intermittently Broken

**Evidence:** All four TF chain targets (`*_imu`, `*_depth_optical_frame` for both head and arm) were `DISCONNECTED entire run`. The fusion node stats show `cam1_only` publications with `dual=0`. 16 WARN messages from TF transform failures.

**Root cause chain:**

1. The relay runs with `use_corrected_tf: true` (`config/prosthesis_config.yaml:325`), looking up `marker_map -> *_imu_openvins_corrected` from the TF buffer and rebroadcasting it as `marker_map -> *_imu` (`openvins_odom_tf_relay.py:444-461`).

2. The corrected frames come from the Jetson-side `aruco_marker_pose_node.py` via DDS `/tf`. They arrive but are **jumping massively** (0.7–0.96m per sample, 6118 total TF jump events).

3. The relay's corrected-TF lookup has a **0.1s timeout** (`openvins_odom_tf_relay.py:450`). When the corrected frame is absent from the TF buffer (DDS delivery jitter, or the aruco node's cycle gap), the lookup fails silently (`except Exception: pass` at line 459).

4. When the corrected lookup fails, the code falls through to the raw VIO path, which is then **suppressed by the outlier gates** (`max_pose_norm_m: 1.5`, `max_pose_jump_m: 0.30`). Raw VIO jumps 0.82–0.83m, exceeding the 0.30m threshold.

5. Result: **neither corrected nor raw TF gets published** for `marker_map -> *_imu`, the TF chain goes stale, and the fusion node silently drops that camera's cloud.

**Important nuance on the timeout:** The `lookup_transform(parent, corrected_frame, Time(), timeout=0.1s)` call uses `Time()` (latest available). Once the corrected frame is in the TF buffer, subsequent lookups return instantly regardless of timeout — the timeout only matters when the buffer is empty (startup or DDS gap). The real problem is that when the lookup fails, there is **no fallback** — the code just falls through to the suppressed raw VIO path. A hold-last-good cache is the critical fix; the timeout value is secondary.

### Issue 2: Slow Point Clouds — Mostly a False Alarm, Partly CPU Saturation

**Evidence:** The "46% Drop Rate" warning for both sides. Fused cloud at 2.5Hz effective.

**Root cause:** The "46% drop" is a **false positive from stale analysis code**. The calculation at `scripts/analyze_bag.py:1418` computes `(1 - 4.7/8.6) * 100 = 45.9%`. But the `naive_pointcloud_assembler` has **no synchronizer** — it caches depth/RGB/CameraInfo independently and runs a timer at `max_rate_hz: 5.0` (`pipeline.launch.py:443,458`). The 4.7Hz output is simply the 5Hz cap slightly reduced by processing overhead.

The Jetson CPU is at **92–98%**, causing real jitter and rate degradation (points drop to 2.9–3.0Hz at end of run). The relay rate gates (image=15Hz, depth=15Hz) are the right place to reduce load — this limits network traffic and host processing without affecting OpenVINS, which needs full camera FPS for tracking.

### Issue 3: Jumping — VIO Divergence + ArUco Reanchoring

**Evidence:**
- Head odom: 12.6m max frame jump, 19.22 m/s max velocity, 72 trans jumps >0.05m
- 6118 TF jump events, 3 drift incidents
- Marker correction yield: head 3.2%, arm 6.3% (very low)
- 48 chi2 rejections (mean=118.72, gate=10.83)

**Root cause:** OpenVINS diverges; ArUco reanchoring snaps it back (12.6m discontinuity). The chi2 gate (10.83) rejects corrections because the diverged estimator covariance is tight — marker measurements look statistically impossible (chi2 values of 118–733 vs gate of 10.83). This creates a positive-feedback trap: divergence blocks its own correction.

The 2.622s "clock offset" is **transport latency**, not chrony drift (sysmon confirms 0.1ms drift). But it triggers the fusion node's clock-skew detector (threshold 1.0s), causing a fallback to latest-TF lookups that amplifies visual jumping.

---

## Implementation Plan

### Phase 1: Reduce Jetson CPU Load via Relay Rate Limiting (Issue 2 + Issue 3 mitigation)

**Design decision:** Limit only the relay output rate, NOT the camera FPS. OpenVINS needs full-rate visual data (15 FPS depth, 30 FPS RGB) for stable tracking. The relay rate gate controls how much crosses the network and reaches the host — limiting it to 5 Hz reduces CPU pressure (less compression, less DDS traffic) and host load (less decompression, less assembly) without starving OpenVINS.

- [x] **1.1. Set the relay image/depth rate to 5 Hz via the Jetson Makefile.** The user has already started this. The relay rate is controlled by the `relay_img_hz` launch argument (or `relay_hz` master override) in the Jetson-side launch file. Setting this to 5.0 limits image and depth relay output to 5 Hz each. The RealSense cameras continue running at their native 15/30 FPS — OpenVINS consumes the full-rate streams locally on the Jetson. Only the relay output (network traffic to host) is throttled. This is the single most impactful change for CPU load.

- [x] **1.2. Set the relay camera_info rate to 10 Hz.** Currently `camera_info.hz` is 30.0. With 5 Hz image/depth output, 10 Hz camera_info is more than sufficient (intrinsics are constant; 2x the image rate ensures fresh data is always available). This reduces wire load and DDS processing.

- [x] **1.3. Verify the host assembler matches.** The host-side `naive_pointcloud_assembler` is already at `max_rate_hz: 5.0` (`pipeline.launch.py:443,458`). With 5 FPS relayed depth input, this is perfectly matched. Verify no processing skips occur in the `[DIAG-ASM]` telemetry (`skips(proc/no_ci/no_new)` should show `proc=0`).

- [x] **1.4. Keep the relay odometry rate at 50 Hz.** Odometry is tiny (~0.7KB/msg) and the relay already rate-limits TF publishing to 100 Hz internally (`openvins_odom_tf_relay.py:183`). Reducing odom rate would degrade TF update frequency and harm fusion. Leave as-is.

### Phase 2: Fix TF Chain Reliability (Issue 1 — Single-View Fusion)

- [x] **2.1. Add a hold-last-good cache for the corrected TF transform.** This is the critical fix. When the corrected-TF lookup fails (`except Exception: pass` at `openvins_odom_tf_relay.py:459`), instead of falling through to the suppressed raw VIO path, rebroadcast the last successful corrected transform (stamped with the current host clock). Add a max cache age of 2.0s — if the cache is older than that, fall through to the raw VIO path (which may also be suppressed, but at that point both sources are genuinely unavailable). This keeps the `marker_map -> *_imu` edge alive during corrected-frame delivery gaps, which are the primary cause of single-view fusion.

- [x] **2.2. Increase the corrected-TF lookup timeout from 0.1s to 0.5s.** The current 0.1s timeout at `openvins_odom_tf_relay.py:450` is too short for DDS delivery under CPU pressure. Once the corrected frame is in the TF buffer, lookups are instant — the timeout only matters for the initial lookup or after a buffer gap. A 0.5s timeout gives DDS more time to deliver without significantly blocking the executor (the call only blocks when the buffer is truly empty, which is rare after startup). Note: the user suggested 1.0s, which would also be acceptable, but 0.5s is a safer compromise since this is a blocking call on the executor thread — at 28Hz odom rate, a 1.0s block would miss ~28 messages. With the hold-last-good cache from 2.1, the exact timeout matters less because failures fall back to cached data, not to the suppressed raw path.

- [x] **2.3. Increase the fusion node's transform_tolerance_s from 0.15s to 0.5s.** In `config/prosthesis_config.yaml:269`. With corrected TF updates arriving at irregular intervals, 0.15s is too tight for TF2 to find the nearest available transform. Increasing to 0.5s allows TF2 to bridge gaps in the corrected-TF stream. The tolerance only lets TF2 look further in its cache — it doesn't create new transforms. On a 100Hz TF stream, a 0.5s tolerance means at most a 0.5s-old transform.

- [x] **2.4. Increase the fusion node's cloud_max_age_s from 0.5s to 1.0s.** In `config/prosthesis_config.yaml:257`. With 5 FPS relayed input and occasional TF lookup delays, a 0.5s window is tight. Increasing to 1.0s gives the fusion node more opportunity to find a valid TF for each cloud. The tradeoff is slightly more motion blur, but at 5 FPS the inter-frame motion is small.

### Phase 3: Reduce Jumping (Issue 3 — VIO Stability)

#### 3A. Chi2 Gate — Dual-Mode on Jetson Side

**Design decision:** The user's instinct ("why not just integrate all measurements?") is directionally correct — the current gate is far too restrictive during divergence, blocking exactly the corrections that could arrest it. However, completely removing the gate risks accepting ArUco pose ambiguities (180° rotational flips on certain marker IDs) and spurious detections, which could *cause* divergence rather than fix it. The geometric hard walls (translation 0.30m, rotation 5° per update) provide some protection but are not sufficient alone.

The recommended approach is a **dual-mode gate** that matches the operator's stated preference (`docs/CURRENT_ISSUES.md:328-330`): "aggressive position recovery when VIO is invalid (wide gate, fast reanchor), conservative when valid (tight gate, protect smoothness)." This is essentially "disable the gate when VIO is diverging" — close to the user's instinct — while keeping protection during healthy operation.

**Important constraint:** The chi2 gate lives in the OpenVINS C++ estimator config on the Jetson side (`docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_*/` and `arm_d435i_*/`). This is in the sibling `multiview_prosthesis-jetson_docker` repo, not the host repo. The host-side plan can only describe the desired behavior; the actual implementation must be done on the Jetson side.

- [x] **3.1. [JETSON-SIDE] Implement dual-mode chi2 gate in the OpenVINS estimator or aruco_marker_pose_node.** When VIO health is VALID (pose norm < 0.5m, velocity < 2 m/s, no recent drift incident): keep the tight chi2 gate (10.83) to protect smoothness and reject spurious detections. When VIO health is INVALID (pose norm > 0.5m, or velocity > 2 m/s, or `marker_innovation_jump` / `pose_covariance_too_large` flag): widen the gate to effectively disable it (e.g., 10000.0) so all marker corrections are accepted. The VIO health signals (`marker_innovation_jump`, `pose_covariance_too_large`) are already computed by the aruco node and emitted as `[DRIFT-INCIDENT]` events. This lets corrections pull the estimate back gradually instead of relying on binary reanchor snaps. The geometric hard walls (translation 0.30m, rotation 5° per update) remain as the final defense against truly bad measurements.

- [x] **3.2. [JETSON-SIDE] Fix the `self.side` AttributeError in aruco_marker_pose_node.py.** Already fixed in commit 7db670c. The crash traceback at line 1901 (`logs/jetson-run-jetson-debug-20260619_174609.txt:2197`) shows `AttributeError: 'ArucoMarkerPoseNode' object has no attribute 'side'`. This blocks the translation/rotation rejection path in some runs. Fix this before tuning the gate logic, as the current crash behavior may mask the actual rejection patterns.

#### 3B. Velocity Suppression — Cautious Investigation

**Design decision:** The user is interested but cautious. The research reveals a critical risk: adding velocity suppression as a bare `return` in the raw VIO path would reproduce the exact stale-TF problem we're fixing in Phase 2. However, the current code structure provides a natural safety net: when `use_corrected_tf: true`, the corrected-TF path runs **before** the raw VIO suppression block, and the raw path is skipped entirely if the corrected path succeeded (`if not published_corrected:` at `openvins_odom_tf_relay.py:474`). This means velocity suppression in the raw path only bites when the corrected path is already unavailable — and with the hold-last-good cache from Phase 2.1, the corrected path's output persists even during gaps.

The evidence strongly supports velocity as the most reliable failure detector: real motion is < 2 m/s (`docs/CURRENT_ISSUES.md:253`), while failure mode is > 19 m/s. A 3.0 m/s threshold gives 50% margin above real motion while being 6x below the smallest observed failure.

- [x] **3.3. Add velocity-based suppression to the raw VIO path ONLY, with hold-last-good protection.** In `openvins_odom_tf_relay.py`, add a velocity check (`jump / dt > max_velocity_mps`) alongside the existing `pos_norm` and `jump` checks in the raw VIO outlier block (around line 475). Use a threshold of 3.0 m/s. **Critical:** this check must be in the `if not published_corrected:` block (line 474) so it never fires when the corrected path is active. With the Phase 2.1 hold-last-good cache, even when velocity suppression fires on the raw path, the corrected cache keeps the TF edge alive. Add a config parameter `max_velocity_mps: 3.0` to `config/prosthesis_config.yaml` under `openvins_odom_tf_relay`.

- [x] **3.4. Fix the `_last_*_pos` update skip on suppression.** Currently, when the jump or norm gate fires (`return` at `openvins_odom_tf_relay.py:482,496`), the `_last_head_pos` / `_last_arm_pos` update at lines 498-502 is skipped. This means the next message's `jump` is measured from the pre-suppression position, potentially inflating it and causing cascading false suppressions. Move the position update to before the suppression checks, or update it unconditionally at the top of `_on_odom`. This is a pre-existing bug that would be amplified by adding velocity suppression.

- [x] **3.5. Add a velocity-suppression counter to the relay status logging.** Add a counter for velocity-suppression events (matching the existing "odom pose jumped — TF suppressed" pattern). Include it in the `_status_tick` output so the analysis tooling can track the trigger rate. If the count is non-zero during known-good motion, the threshold needs adjustment. This provides the instrumentation to validate the 3.0 m/s threshold before relying on it.

#### 3C. Smooth Corrected TF (reanchor discontinuities)

- [x] **3.6. Apply a low-pass filter to the corrected-TF rebroadcast translation.** When the relay rebroadcasts the corrected frame (`openvins_odom_tf_relay.py:444-461`), apply an exponential moving average to the translation (alpha=0.3). This smooths the reanchor discontinuities (0.7–0.96m jumps) into gradual transitions over 3–5 frames. Only filter translation, not rotation (rotation discontinuities are less visually disruptive and filtering them could cause orientation smear during fast reanchors). The ~0.2s lag is well within the cloud_max_age window. Skip the filter for the first few frames after startup (when the filter state is uninitialized).

### Phase 4: Fix Stale Analysis Warnings (Diagnostic Accuracy)

- [x] **4.1. Update `analyze_bag.py` backprojection sync warning to reflect the naive assembler.** The `print_backprojection_sync` function (`scripts/analyze_bag.py:1369-1460`) still references "Message Filter Synchronizer" and "depth_image_proc node." Update the warning text to explain that the output rate is set by the assembler's `max_rate_hz` cap, not a sync failure. Change the warning threshold: only flag when `points_hz < 1.0` (actual failure) rather than `drop_pct > 10` (expected rate-capping behavior). If the assembler's `max_rate_hz` can be read from the config, compare against that instead of the input rate.

- [x] **4.2. Update the `pipeline_diagnostics_node` SYNC_DROP label.** The diagnostic label at `src/camera/camera/pipeline_diagnostics_node.py:599-600` references the old C++ synchronizer. Update it to reflect the naive assembler's skip logic (`_skip_processing`, `_skip_no_new`, `_skip_no_ci`).

### Phase 5: Increase Clock-Skew Resilience (Issue 1 + Issue 3)

- [x] **5.1. Increase the clock_skew_fallback_s threshold from 1.0s to 5.0s.** In `config/prosthesis_config.yaml:267`. The fusion node falls back to arrival-time stamping when header-to-arrival offset exceeds 1.0s. The transport latency peak of 2.622s triggers this falsely — chrony drift is confirmed at 0.1ms. Since header stamps are valid (just delayed in transit), increasing to 5.0s prevents false clock-skew detection while still catching real chrony failures. The sysmon independently monitors chrony drift (0.1ms threshold), so real failures are still caught.

- [x] **5.2. Add a timeout to the clock-skew fallback TF lookup.** When clock-skew fallback IS active, the code at `pointcloud_fusion_node.py:531-535` uses `lookup_transform(target, source, Time(0))` with no explicit timeout. Add a 0.5s timeout to avoid blocking the processing thread when the TF chain is truly broken.

---

## Verification Criteria

- [ ] Fused point cloud shows `dual > 0` in the fusion node stats (both camera views merged)
- [ ] Fused point cloud rate is stable at ~5Hz with CV < 0.3 (low jitter)
- [ ] TF jump events reduced from 6118 to < 500 over a 90s run
- [ ] Head odom max frame jump < 0.5m (down from 12.6m)
- [ ] Jetson CPU drops below 85% average (down from 92%)
- [ ] No false "46% drop" warning from `analyze_bag.py`
- [ ] Clock-skew detector does not falsely trigger (transport latency is not chrony drift)
- [ ] `[DIAG-ASM]` telemetry shows `skips(proc/no_ci/no_new)` with `proc=0` (no processing backlog)
- [ ] Velocity suppression counter is zero during known-good slow motion
- [ ] OpenVINS camera FPS remains at native 15/30 (relay-limited only, not camera-limited)

## Potential Risks and Mitigations

1. **Relay rate limiting to 5 Hz reduces point cloud density**
   Mitigation: 5 Hz fused clouds at 640x480 resolution still provide ~150K points per cloud. For the prosthesis grasp pipeline (twist propagation, segmentation), 5 Hz is sufficient — the pipeline has a ~1.4s latency budget. If density is insufficient, increase to 10 Hz as a compromise.

2. **Dual-mode chi2 gate may accept bad marker corrections during divergence**
   Mitigation: The geometric hard walls (translation 0.30m, rotation 5° per update) still reject physically impossible corrections. The dual-mode gate only relaxes the statistical plausibility check, not the geometric sanity check. ArUco markers in a controlled environment rarely produce false positives.

3. **Velocity suppression could cause stale TF if corrected path also fails**
   Mitigation: The hold-last-good cache (Phase 2.1) keeps the TF edge alive for up to 2.0s during corrected-path gaps. Velocity suppression only fires on the raw VIO path, which is already bypassed when the corrected path succeeds. The double-failure scenario (corrected path unavailable AND raw VIO velocity-suppressed) is rare and results in cached TF being used rather than no TF at all.

4. **Low-pass filter on corrected TF adds latency and may smear fast reanchors**
   Mitigation: Use alpha=0.3 (fast response, ~3-frame settling) and only filter translation. The ~0.2s lag is within the cloud_max_age window. Skip filtering during the first few frames after startup.

5. **Clock-skew threshold increase to 5.0s masks real chrony failures**
   Mitigation: The sysmon independently monitors chrony drift at 0.1ms precision. The fusion node's detector is a secondary safety net. Real chrony failures would show >5s offset and still be caught.

6. **`_last_*_pos` bug fix may change suppression patterns**
   Mitigation: The fix (updating position before suppression checks) makes jump measurement more accurate. Current behavior inflates jumps after suppression, causing cascading false positives — fixing this should reduce total suppression events, not increase them.

## Alternative Approaches

1. **Completely remove the chi2 gate (user's initial instinct):** Simpler, ensures corrections always flow. Risk: ArUco pose ambiguities (180° flips) and spurious detections would be accepted into the EKF, potentially causing divergence rather than fixing it. The dual-mode approach achieves nearly the same effect (gate effectively disabled during divergence) while keeping protection during healthy operation. If dual-mode proves too complex to implement on the Jetson side, a simple wide gate (e.g., 1000.0) is a reasonable fallback — it would accept almost everything while still rejecting truly catastrophic outliers (chi2 > 1000 from marker mis-detections with geometric hard walls as backstop).

2. **Disable `use_corrected_tf` and rely solely on raw VIO with looser suppression:** Simpler but loses ArUco anchoring. Raw VIO drifts without marker corrections. Only viable if OpenVINS is retuned for better intrinsic stability.

3. **Switch to `gtsam_only` fusion mode:** Runs the GTSAM tracker which smooths VIO with a 7-second lag. Eliminates most jumping but adds latency. Worth trying after Phase 1–3 stabilizes the VIO input.

4. **Increase relay rate to 10 Hz instead of 5:** Compromise between CPU load and cloud density. If 5 Hz causes issues with twist propagation (not enough update rate for hit detection), 10 Hz is the fallback.
