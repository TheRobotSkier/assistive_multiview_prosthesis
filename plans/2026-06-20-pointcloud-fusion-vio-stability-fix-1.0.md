# Pointcloud Fusion & VIO Stability — Issue Resolution Plan

## Objective

Resolve three interconnected issues observed in the v6 run (20260620_165849):
1. **Single-view fusion** — fused point cloud often only includes one camera view
2. **Slow point clouds** — clouds arrive slowly / at low rate
3. **Jumping** — visible discontinuities in the point cloud / RViz, possibly from OpenVINS or timestamp issues

## Root Cause Analysis

### Issue 1: Single-View Fusion — TF Chain Intermittently Broken

**Evidence:** The analysis report shows all four TF chain targets (`*_imu`, `*_depth_optical_frame` for both head and arm) as `DISCONNECTED entire run (4 blocks)`. The fusion node's stats show `cam1_only` publications (single-view) with `dual=0`. The `pointcloud_fusion_node` logged 16 WARN messages (TF transform failures).

**Root cause chain:**

1. The relay runs with `use_corrected_tf: true` (`config/prosthesis_config.yaml:325`), which looks up `marker_map -> *_imu_openvins_corrected` from the TF buffer and rebroadcasts it as `marker_map -> *_imu` (`openvins_odom_tf_relay.py:444-461`).

2. The corrected frames are published by the Jetson-side `aruco_marker_pose_node.py` via DDS `/tf`. They arrive but are **jumping massively** (0.7–0.96m per sample, 6118 total TF jump events).

3. The relay's corrected-TF lookup has a **0.1s timeout** (`openvins_odom_tf_relay.py:450`). When the corrected frame is momentarily absent from the TF buffer (due to DDS delivery jitter or the Jetson-side aruco node's ~1Hz reanchor cycle), the lookup fails silently (`except Exception: pass` at line 459).

4. When the corrected lookup fails, the code falls through to the raw VIO path, which is then **suppressed by the outlier gates** (`max_pose_norm_m: 1.5`, `max_pose_jump_m: 0.30` at `config/prosthesis_config.yaml:317-318`). The raw VIO jumps 0.82–0.83m, exceeding the 0.30m threshold.

5. Result: **neither corrected nor raw TF gets published** for `marker_map -> *_imu`, the TF chain goes stale, and the fusion node's `lookup_transform_full` at the cloud's header stamp fails. The cloud is silently dropped.

6. The fusion node's `transform_tolerance_s: 0.15` (`config/prosthesis_config.yaml:269`) is too tight — the corrected TF updates arrive at irregular intervals (the aruco node runs at ~1Hz with reanchor events), so a 0.15s tolerance cannot bridge the gaps.

**Additionally:** The clock offset peak of 2.622s (line 93 of the analysis) triggers the fusion node's clock-skew detector (`clock_skew_fallback_s: 1.0` at `config/prosthesis_config.yaml:267`), causing a permanent fallback to arrival-time stamping with `lookup_transform(Time(0))`. This gets whatever TF happens to be latest — which may be a frame from the opposite camera's jumping corrected trajectory.

### Issue 2: Slow Point Clouds — Mostly a False Alarm, Partly CPU Saturation

**Evidence:** The analysis reports "46% Drop Rate" for both head and arm backprojection. The fused cloud rate is 2.5Hz effective (207 messages over 83.5s).

**Root cause:** The "46% drop" is a **false positive from stale analysis code**. The calculation at `scripts/analyze_bag.py:1418` computes `drop_pct = (1 - points_hz / min_input_hz) * 100` = `(1 - 4.7/8.6) * 100 = 45.9%`. But the `naive_pointcloud_assembler` has **no synchronizer** — it caches depth/RGB/CameraInfo independently and runs a timer at `max_rate_hz: 5.0` (`pipeline.launch.py:443,458`). The 4.7Hz output is simply the 5Hz cap slightly reduced by processing overhead. No frames are being dropped due to timestamp mismatch.

The warning text at `analyze_bag.py:1451-1453` references "Message Filter Synchronizer" and "Jetson Relay timestamp restamping" — neither is relevant to the current architecture.

**However, the Jetson CPU is at 92–98%** (avg 92%, max 98%), causing:
- High jitter on all topics (CV 0.23–0.47)
- Depth effective rate of 8.6Hz instead of the configured 15Hz gate
- Odom rate of 22.8Hz instead of the 50Hz gate
- The last diagnostic block shows points dropping to 2.9–3.0Hz (down from 5.0–5.1Hz mid-run)

The user's instinct to lower FPS is correct — the Jetson is CPU-saturated. The current RealSense config runs depth at 15 FPS and RGB at 30 FPS natively, with the relay gating both to 15Hz. Reducing to 5 FPS would dramatically cut CPU load.

### Issue 3: Jumping — VIO Divergence + ArUco Reanchoring

**Evidence:**
- Head odom: 12.6m max frame jump, 19.22 m/s max velocity (implausible), 72 trans jumps >0.05m, path_length=30.6m
- Arm odom: 0.83m max frame jump, 2 trans jumps >0.05m (more stable)
- 6118 TF jump events detected by `pipeline_diagnostics_node`
- 3 drift incidents: 2 marker_innovation_jump, 1 pose_covariance_too_large
- Marker correction yield: head 3.2%, arm 6.3% (very low — diverged estimator rejects corrections)

**Root cause:** OpenVINS VIO diverges over time (head reaches 12.6m position norm). The ArUco marker system detects the divergence and fires a reanchor event (~1Hz), snapping the estimate back to the marker. This creates a massive discontinuity (12.6m in one frame). The `*_imu_openvins_corrected` frames inherit these snaps.

The chi2 gate rejects 48 marker corrections (mean chi2=118.72 vs gate=10.83) because the diverged estimator state makes marker measurements look statistically impossible. This prevents the corrections from pulling the estimate back gradually, forcing the binary reanchor behavior.

The 2.622s "clock offset" is **transport latency**, not chrony drift — the sysmon shows chrony drift of only 0.1ms. The analysis correlation callout confirms: "the 207ms delta is Message Transport / Serialization Latency, NOT clock skew."

**The jumping is NOT primarily a timestamp issue** — it is VIO estimator divergence. However, the transport latency does affect the fusion node's clock-skew detector, causing the fallback to latest-TF lookups which amplifies the visual jumping.

---

## Implementation Plan

### Phase 1: Reduce Jetson CPU Load (Issue 2 + Issue 3 mitigation)

- [ ] **1.1. Lower RealSense camera FPS to 5 on the Jetson side.** The Jetson CPU is at 92–98% with depth@15FPS + RGB@30FPS. Reducing both to 5 FPS cuts the camera pipeline load by ~67%. This is the single most impactful change — it reduces CPU pressure, stabilizes odom rates, reduces jitter, and gives OpenVINS more CPU headroom. The user has already identified this and set it in the Jetson Makefile (`../multiview_prosthesis-jetson_docker`). Verify the setting took effect by checking the Jetson log for the RealSense node's reported FPS.

- [ ] **1.2. Lower relay rate gates to match 5 FPS.** In the Jetson-side launch file (`dynamic_id2_arm_update_live.launch.py`), set `relay_img_hz` to 5.0 (currently 15.0). This ensures the relay doesn't try to push more frames than the camera produces, eliminating wasted CPU on rate-gate checks for frames that will be rejected. Also set `camera_info.hz` from 30.0 to 10.0 (2x the camera rate is sufficient for fresh intrinsics).

- [ ] **1.3. Verify the assembler max_rate_hz matches.** The host-side `naive_pointcloud_assembler` is already at `max_rate_hz: 5.0` (`pipeline.launch.py:443,458`). With 5 FPS camera input, this is perfectly matched — no change needed, but verify no processing skips occur in the `[DIAG-ASM]` telemetry.

### Phase 2: Fix TF Chain Reliability (Issue 1 — Single-View Fusion)

- [ ] **2.1. Increase the relay's corrected-TF lookup timeout from 0.1s to 0.5s.** In `openvins_odom_tf_relay.py:450`, the corrected frame lookup uses `timeout=rclpy.duration.Duration(seconds=0.1)`. The aruco node publishes at ~1Hz, so the corrected frame may be up to 1s old in the buffer. A 0.1s timeout causes the lookup to fail frequently when DDS delivery is jittery. Increase to 0.5s to bridge delivery gaps. This is a blocking call on the executor thread, but at 28Hz odom rate with 100Hz TF rate limiting, the impact is negligible.

- [ ] **2.2. Add a fallback: when corrected-TF lookup fails, publish the last known corrected transform.** Currently, when the corrected lookup fails (`except Exception: pass` at `openvins_odom_tf_relay.py:459`), the code falls through to the raw VIO path which gets suppressed. Instead, cache the last successful corrected transform and rebroadcast it (stamped with host clock) when the lookup fails. This keeps the `marker_map -> *_imu` edge alive during corrected-frame gaps. Add a max age for the cached transform (e.g., 2.0s) to avoid broadcasting very stale data.

- [ ] **2.3. Increase the fusion node's transform_tolerance_s from 0.15s to 0.5s.** In `config/prosthesis_config.yaml:269`, the `transform_tolerance_s` controls how much temporal slack TF2 gets when looking up a transform at a specific time. With corrected TF updates arriving at irregular intervals, 0.15s is too tight. Increasing to 0.5s allows TF2 to find the nearest available transform even when there are gaps in the corrected-TF stream.

- [ ] **2.4. Increase the fusion node's cloud_max_age_s from 0.5s to 1.0s.** In `config/prosthesis_config.yaml:257`, clouds older than 0.5s are rejected. With 5 FPS camera input and occasional TF lookup delays, a 0.5s window is tight. Increasing to 1.0s gives the fusion node more opportunity to find a valid TF for each cloud. The tradeoff is slightly more motion blur in the fused cloud, but at 5 FPS the inter-frame motion is small.

### Phase 3: Reduce Jumping (Issue 3 — VIO Stability)

- [ ] **3.1. Widen the chi2 gate on the Jetson side to accept more marker corrections.** The Jetson analysis shows 48 chi2 rejections (mean=118.72, max=733.16, gate=10.83). When VIO diverges, marker corrections look statistically impossible and are rejected, preventing gradual correction and forcing binary reanchor snaps. Widen the chi2 gate (e.g., to 50.0 or 100.0) or implement a dual-mode gate: tight (10.83) when VIO is valid, wide (100.0) when VIO pose norm exceeds 0.5m. This allows corrections to pull the estimate back gradually instead of snapping. This change is in the Jetson-side OpenVINS config or the `aruco_marker_pose_node.py` rejection logic.

- [ ] **3.2. Add velocity-based suppression to the relay.** Currently the relay suppresses based on position norm (1.5m) and frame-to-frame jump (0.30m). Add a third check: velocity = jump / dt. If velocity > 3.0 m/s (well above any physical motion), suppress the TF. This catches divergence faster than waiting for the position norm to exceed 1.5m. The head VIO reached 19.22 m/s — a velocity gate would have caught this within one frame instead of letting it diverge to 12.6m. Add this check in `openvins_odom_tf_relay.py` in the raw VIO outlier suppression block (around line 475).

- [ ] **3.3. Smooth the corrected-TF rebroadcast with a simple low-pass filter.** When the relay rebroadcasts the corrected frame, apply a low-pass filter to the translation (e.g., exponential moving average with alpha=0.3). This smooths the reanchor discontinuities (0.7–0.96m jumps) into gradual transitions over 3–5 frames. The tradeoff is a ~0.2s lag in the corrected trajectory, but this is acceptable for pointcloud visualization and fusion. Implement in the corrected-TF rebroadcast block (`openvins_odom_tf_relay.py:444-461`).

### Phase 4: Fix Stale Analysis Warnings (Diagnostic Accuracy)

- [ ] **4.1. Update `analyze_bag.py` backprojection sync warning to reflect the naive assembler architecture.** The `print_backprojection_sync` function (`scripts/analyze_bag.py:1369-1460`) still references "Message Filter Synchronizer" and "depth_image_proc node." Update the warning text to explain that the drop is the intentional `max_rate_hz` cap, not a sync failure. Change the warning threshold to only flag when `points_hz < 1.0` (actual failure) rather than `drop_pct > 10` (expected rate-capping behavior).

- [ ] **4.2. Update the `pipeline_diagnostics_node` SYNC_DROP label.** The diagnostic label at `src/camera/camera/pipeline_diagnostics_node.py:599-600` references the old C++ synchronizer. Update it to reflect the naive assembler's skip logic (`_skip_processing`, `_skip_no_new`, `_skip_no_ci`).

### Phase 5: Increase Clock-Skew Resilience (Issue 1 + Issue 3)

- [ ] **5.1. Increase the clock_skew_fallback_s threshold from 1.0s to 5.0s.** In `config/prosthesis_config.yaml:267`, the fusion node falls back to arrival-time stamping when header-to-arrival offset exceeds 1.0s. The transport latency peak of 2.622s triggers this fallback, which then uses `lookup_transform(Time(0))` — getting whatever TF is latest, which may be a jumping frame. Since chrony drift is confirmed at 0.1ms (not the problem), the 2.622s offset is pure transport latency and the header stamps are still valid. Increasing the threshold to 5.0s prevents the false clock-skew detection while still catching real chrony failures.

- [ ] **5.2. When clock-skew fallback IS active, use `lookup_transform(Time(0))` with a shorter tolerance instead of no tolerance.** Currently the fallback path at `pointcloud_fusion_node.py:531-535` uses `lookup_transform(target, source, Time(0))` with no explicit timeout. Add a 0.5s timeout to avoid blocking the processing thread when the TF chain is truly broken.

## Verification Criteria

- [ ] Fused point cloud shows `dual > 0` in the fusion node stats (both camera views merged)
- [ ] Fused point cloud rate is stable at ~5Hz with CV < 0.3 (low jitter)
- [ ] TF jump events reduced from 6118 to < 500 over a 90s run
- [ ] Head odom max frame jump < 0.5m (down from 12.6m)
- [ ] Jetson CPU drops below 85% average (down from 92%)
- [ ] No false "46% drop" warning from `analyze_bag.py`
- [ ] Clock-skew detector does not falsely trigger (transport latency is not chrony drift)
- [ ] `[DIAG-ASM]` telemetry shows `skips(proc/no_ci/no_new)` with `proc=0` (no processing backlog)

## Potential Risks and Mitigations

1. **Lowering FPS to 5 reduces tracking quality for OpenVINS**
   Mitigation: OpenVINS uses the IMU (200Hz) as its primary motion model; visual updates at 5Hz are sufficient for slow-motion prosthesis use cases. Monitor VIO yield (currently 3.2–6.3%) — if it drops further, increase to 10 FPS as a compromise.

2. **Widening chi2 gate may accept bad marker corrections**
   Mitigation: The translation (0.30m) and rotation (5.0°) hard walls still reject physically impossible corrections. The chi2 gate only relaxes the statistical plausibility check, not the geometric sanity check.

3. **Low-pass filter on corrected TF adds latency**
   Mitigation: Use alpha=0.3 (fast response) and only filter translation, not rotation. The ~0.2s lag is well within the cloud_max_age window and doesn't affect grasp timing (the pipeline has ~1.4s latency budget).

4. **Increasing transform_tolerance_s may use stale TFs**
   Mitigation: The tolerance only allows TF2 to look further in the cache; it doesn't create new transforms. A 0.5s tolerance on a 100Hz TF stream means at most a 0.5s-old transform, which is acceptable for 5Hz clouds.

5. **Clock-skew threshold increase to 5.0s masks real chrony failures**
   Mitigation: The sysmon already monitors chrony drift independently (0.1ms threshold). The fusion node's clock-skew detector is a secondary safety net, not the primary diagnostic. Real chrony failures would show >5s offset and still be caught.

## Alternative Approaches

1. **Disable `use_corrected_tf` and rely solely on raw VIO with looser suppression thresholds**: Simpler but loses the ArUco anchoring benefit. The raw VIO drifts without marker corrections, so the fused cloud would slowly slide in RViz. Only viable if OpenVINS is retuned for better intrinsic stability.

2. **Switch to `gtsam_only` fusion mode**: Runs the GTSAM tracker which smooths VIO with a 7-second lag. This would eliminate most jumping but adds latency and requires the GTSAM pipeline to be stable. Worth trying after Phase 1–3 stabilizes the VIO input.

3. **Run both cameras at 10 FPS instead of 5**: Compromise between CPU load and tracking quality. If 5 FPS causes VIO instability, 10 FPS is the fallback. The relay and assembler rates would also be set to 10.

4. **Increase `cloud_max_age_s` to 2.0s and use only the latest TF**: Maximizes fusion availability but risks publishing clouds with very stale transforms. Not recommended for production but useful for debugging.
