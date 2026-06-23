# V6 Remaining Issues — Revised Diagnosis and Resolution Plan (v2)

## Objective

Identify and resolve all remaining issues blocking the multiview prosthesis pipeline from producing fused point clouds and stable localization. This revised plan incorporates operator feedback, actual source code analysis from both the host repo and the Jetson-side `jetson-docker` repo, and a corrected root-cause chain.

---

## Issue Summary (Priority-Ranked)

| # | Issue | Severity | Evidence |
|---|-------|----------|----------|
| 1 | **aruco_marker_pose_node crashes on first marker detection** | CRITICAL | `UnboundLocalError: cannot access local variable 'marker_ids'` at line 997 — variable used before assignment |
| 2 | **100% backprojection frame drop (sync slop too tight)** | CRITICAL | depth=3.1Hz, rgb=4.6Hz → points=0.0Hz; `slop=0.06` too tight for the ~200ms natural timestamp gap |
| 3 | **camera_info topic bursting (238–538 Hz peaks)** | HIGH | RELIABLE QoS + token-gating without RateGate cap; 830% DDS overhead |
| 4 | **Arm TF chain breaks after VIO divergence** | CRITICAL | Arm initialized at 56.7s, pose norm exceeds 1.0m at 60s → relay suppresses all subsequent poses → TF DISCONNECTED for rest of run |
| 5 | **Arm VIO catastrophic divergence** | CRITICAL | X=[-0.334, 310]m, velocity 1094 m/s; positive-feedback loop after OpenVINS init |

---

## Corrected Root-Cause Chain

```
[Issue 3: camera_info RELIABLE QoS bursts]
    ↓ (830% DDS overhead → CPU pressure on both machines)
[Issue 1: ArUco node crashes on first marker detection]
    ↓ (no ArUco corrections → VIO has no absolute reference)
[Issue 5: VIO diverges within seconds of initialization]
    ↓ (relay correctly suppresses poses >1.0m norm — defense working as designed)
[Issue 4: Arm TF chain goes DISCONNECTED after suppression]
    ↓ (fusion TF wait gate opens briefly at 56.7s, then arm disconnects)
    +
[Issue 2: sync slop=0.06 too tight → 100% backprojection drops]
    ↓
[Result: zero point clouds, zero fused output, pipeline non-functional]
```

**Key insight from log analysis**: The relay init guard is NOT the problem. It correctly waited for OpenVINS covariance to become non-zero (event-driven, checked on every odom message). Head initialized at 47.3s, arm at 56.7s — both via covariance detection, not the 30s fallback timer. The TF chain actually connected briefly at 56.7s (`host-log-20260620_114259.txt:984`: "All chains healthy"). But within 3.6 seconds, the arm VIO pose norm exceeded 1.0m, the relay suppressed it, and the arm TF went DISCONNECTED for the rest of the run. The root cause is the VIO divergence itself, not the relay's initialization behavior.

---

## Implementation Plan

### Phase 1: Fix ArUco Marker Pose Node Crash (Issue 1) — Jetson Side

**Rationale**: This is the single most impactful fix. The ArUco node crashes immediately on the first marker detection due to a trivial Python bug, leaving VIO without any absolute position corrections. Without ArUco corrections, the arm VIO diverges uncontrollably within seconds. This is the root enabler of Issues 4 and 5.

**Root cause** (confirmed from source code at `../multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py:988-1002`):

At line 997, the f-string references `marker_ids`:
```python
f"marker_ids={marker_ids}"
```
But `marker_ids` is not assigned until line 1002:
```python
marker_ids = [int(marker_id_arr[0]) for marker_id_arr in ids]
```
The log line at 997 is inside the `if self._first_marker_detection_wall_sec is None:` block (line 988), which executes on the very first marker detection. The node crashes before it can process any markers. Both crash tracebacks (lines 4038 and 5016 of the Jetson log) show the identical error.

- [ ] **1.1. Fix the `marker_ids` reference in `aruco_marker_pose_node.py:997`**. Move the `marker_ids` assignment (line 1002) to before the first-marker-detection log block (before line 988), or change the f-string at line 997 to use `ids` directly (e.g., `f"marker_ids={[int(i[0]) for i in ids]}"`). The latter is a one-line change with no side effects since `marker_ids` is recomputed from `ids` anyway at line 1002.

- [ ] **1.2. Add `respawn=True` to the ArUco node in the Jetson launch files** (`head_marker_pose_phase2.launch.py`, `arm_marker_pose_phase2.launch.py`) as a safety net so the node automatically restarts if it crashes again for any other reason.

- [ ] **1.3. Verify the fix**: After deploying to the Jetson, check that the Jetson log shows `[STARTUP] phase=first_marker side=head` and `side=arm` without any traceback, and that the VIO state estimation health section shows non-zero Markers and Corrections counts.

### Phase 2: Fix Backprojection Sync Slop (Issue 2) — Host Side

**Rationale**: The Jetson relay already has sophisticated token-gating that locks depth+RGB+camera_info as timestamped triplets (`jetson_relay.py:730-763`). The relay preserves original hardware timestamps from the RealSense ASIC clock. The problem is purely on the host side: the `point_cloud_xyzrgb_node`'s `ApproximateTimeSynchronizer` with `slop=0.06` (60ms) is too tight. At the observed rates (depth ~3 Hz, RGB ~4.7 Hz), the natural timestamp gap between depth and RGB frames from the same exposure can be up to ~200ms. The synchronizer discards 100% of frames.

Restamping is NOT the right approach — the original capture timestamps are important for TF-based cloud transformation and should be preserved. Increasing the slop is the correct fix.

- [ ] **2.1. Increase the ApproximateTimeSynchronizer slop** from `0.06` to `0.15` in `pipeline.launch.py:403` and `pipeline.launch.py:421`. At depth=3Hz and RGB=4.7Hz, the maximum natural timestamp gap between the nearest depth and RGB frames is ~167ms (1/3Hz + 1/4.7Hz ≈ 0.54s period, but the actual gap depends on phase alignment). A slop of 150ms accommodates this while still rejecting frames from different exposure cycles. Also increase `queue_size` from 20 to 25 to give the synchronizer more candidates.

- [ ] **2.2. Verify the fix**: Run the pipeline with `debug_monitor:=true` for 60 seconds. Check the `[DIAG-PC]` blocks in the host log — `points` should show non-zero Hz for both head and arm. The `pipeline_diagnostics_node` already labels the issue as `[SYNC_DROP]` when depth and RGB arrive but points don't (`host-log-20260620_114259.txt:975-976`).

### Phase 3: Cap camera_info Rate (Issue 3) — Jetson Side

**Rationale**: The `camera_info` topic is showing 238–538 Hz bursts despite a `camera_info.hz` parameter of 1.0. The problem: `_ci_hz` is declared and read (`jetson_relay.py:448,480`) but **never applied to a RateGate**. The `_on_ci` callback (line 730) only uses `_stamp_matches_token` for gating, which forwards one camera_info per approved image frame (~5 Hz). However, camera_info uses `_CAMINFO_QOS` with `RELIABLE` durability (`jetson_relay.py:83-87`), while image/depth use `_SENSOR_QOS` (BEST_EFFORT). This QoS asymmetry means camera_info messages require DDS acknowledgment and retransmission. Under network pressure, unacknowledged RELIABLE messages pile up and burst-retransmit, producing the 238–538 Hz peaks.

The operator notes that outdated camera_info has been a past concern, so a middle ground of ~30 Hz is appropriate — high enough to always have fresh intrinsics, low enough to prevent flooding.

- [ ] **3.1. Add a RateGate to camera_info forwarding** in `jetson_relay.py`. In `_setup_camera_info` (line 638), create a RateGate per camera: `self._gates[f"ci_{cam}"] = RateGate(30.0)`. In `_on_ci` (line 730), add a rate check before the token check: `if not self._gates[f"ci_{camera}"].should_publish(): return`. This caps camera_info at 30 Hz regardless of token-gating behavior.

- [ ] **3.2. Change camera_info QoS from RELIABLE to BEST_EFFORT** in `jetson_relay.py:83-87`. Camera intrinsics are constant for a given resolution — a dropped frame is inconsequential since the next frame carries identical data. RELIABLE QoS on a high-rate topic across a Cat5e link is the primary source of DDS retransmission overhead. BEST_EFFORT eliminates the acknowledgment/retransmission cycle entirely. The host-side `point_cloud_xyzrgb_node` subscribes with `SystemDefaultsQoS()` which is RELIABLE in C++, but the decompress bridge pattern (BEST_EFFORT input → RELIABLE output) can be applied if needed. Alternatively, verify that `depth_image_proc` accepts BEST_EFFORT camera_info — it typically does for sensor data.

- [ ] **3.3. Update the `camera_info.hz` parameter default** from 1.0 to 30.0 in `jetson_relay.py:448` to match the new RateGate, ensuring the logged configuration (`_log_config` at line 811) accurately reflects the actual behavior.

### Phase 4: Address Arm VIO Divergence (Issue 5) — Algorithmic / Longer Term

**Rationale**: The arm VIO diverges catastrophically within seconds of OpenVINS initialization. The relay's outlier suppression (max_pose_norm_m=1.0, max_pose_jump_m=0.20) correctly blocks diverging poses, but this leaves the arm TF chain disconnected. The root cause is in the OpenVINS estimator on the Jetson — a positive-feedback loop where a corrupted velocity/bias estimate feeds bad predictions into the next update.

With the ArUco node fix (Phase 1), marker corrections should keep VIO bounded. But the divergence may still occur if markers are not visible or if the chi2 gate rejects corrections (as documented in `docs/CURRENT_ISSUES.md:169-173`).

- [ ] **4.1. Verify whether the ArUco fix alone stabilizes VIO.** After deploying Phase 1, run a capture session and check the bag analysis for arm odom stability. If the head-arm relative distance stays within 0.2–1.5m with low variance, the ArUco corrections were the missing piece and no further VIO work is needed.

- [ ] **4.2. If VIO still diverges with ArUco corrections, investigate the chi2 gate.** Per `docs/CURRENT_ISSUES.md:169-173`, 1,462 marker corrections were rejected (mean chi2=960 vs gate=10.83) because the estimator state was so far off that marker measurements looked statistically impossible. The fix is a dual-mode correction policy: when VIO health is INVALID, use a wide chi2 gate to let corrections pull the estimate back; when valid, use the tight gate to protect smoothness. This requires modifying `aruco_marker_pose_node.py` on the Jetson side.

- [ ] **4.3. Analyze the `[DRIFT-INCIDENT]` telemetry** in the next run's logs to determine whether divergence starts gradually (IMU bias drift) or suddenly (visual tracking failure). The operator noted in `docs/CURRENT_ISSUES.md:78-81` that the IMU was Kalibr-calibrated but excessive drift persists.

- [ ] **4.4. Consider relaxing the relay's outlier suppression thresholds** as a stopgap. Currently `max_pose_norm_m=1.0` and `max_pose_jump_m=0.20` (`config/prosthesis_config.yaml:317-318`). The arm pose at initialization was 0.355m, and it exceeded 1.0m within 3.6 seconds. If the thresholds are slightly too tight for legitimate movement, relaxing to `max_pose_norm_m=1.5` and `max_pose_jump_m=0.30` would allow more poses through while still blocking catastrophic divergence. This is a tradeoff: more TF availability but potentially noisier transforms.

---

## Verification Criteria

- [ ] `aruco_marker_pose_node` runs without crashing for the full capture duration (currently crashes on first marker detection)
- [ ] `/jetson/head/points` and `/jetson/arm/points` show >0 Hz in diagnostic output (currently 0.0 Hz with `[SYNC_DROP]`)
- [ ] `/fused_pointcloud` shows >0 Hz (currently ABSENT — 0 messages)
- [ ] TF chain `marker_map → arm_d435i_arm_depth_optical_frame` stays CONNECTED for the majority of the run after initialization (currently DISCONNECTED after ~4s)
- [ ] camera_info topic rate stabilizes at ~30 Hz with no bursts >60 Hz (currently peaks at 538 Hz)
- [ ] DDS network overhead drops below 200% (currently 830%)
- [ ] Head-arm relative distance stays within 0.2–1.5m range (currently mean=807m)

## Potential Risks and Mitigations

1. **ArUco fix may not fully prevent VIO divergence**
   Mitigation: Phase 4.1 verifies this empirically. If divergence persists, Phase 4.2 (chi2 gate fix) and Phase 4.4 (threshold relaxation) provide additional defenses. The relay's outlier suppression is the last line of defense and is working correctly.

2. **Increasing sync slop may match wrong frame pairs**
   Mitigation: At 3–5 Hz with slop=0.15s, the synchronizer matches the nearest pair within 150ms. Since depth and RGB are captured near-simultaneously by the RealSense (within ~15-23ms per the Jetson relay comments at `jetson_relay.py:754-755`), the correct pair is always the closest match. The risk of mismatching non-corresponding frames is negligible at these low rates.

3. **camera_info QoS change may break depth_image_proc**
   Mitigation: The C++ `point_cloud_xyzrgb_node` uses `SystemDefaultsQoS()` which defaults to RELIABLE. If it rejects BEST_EFFORT camera_info, add a host-side relay similar to `decompress_bridge` that subscribes BEST_EFFORT and republishes RELIABLE locally. Test first — many ROS2 sensor nodes accept BEST_EFFORT via QoS auto-negotiation.

4. **camera_info RateGate at 30 Hz may still be too high for the wire link**
   Mitigation: Camera intrinsics are constant — even at 30 Hz, each message is only 0.4KB, totaling 12KB/s. This is negligible compared to the depth+RGB payload (~287KB/s). The 30 Hz rate ensures the host always has fresh intrinsics without flooding.

## Alternative Approaches

1. **Use `fusion_mode='tsdf_preview'` instead of legacy fusion**: The TSDF path is more tolerant of individual frame drops since it integrates keyframes over time. However, it requires additional nodes (keyframe buffer, GTSAM poses) that add complexity. Recommended only if the legacy fusion sync issues prove intractable — which they should not be after the slop fix.

2. **Host-side camera_info throttle**: If the Jetson-side QoS change is delayed, add a lightweight throttle node on the host that subscribes to `/jetson/*/camera_info` (BEST_EFFORT) and republishes at 30 Hz (RELIABLE) for local consumption. This is a temporary workaround.

---

## Dependency Graph

```
Phase 1 (ArUco crash fix) ────► VIO corrections flow ────► VIO stays bounded ──┐
                                                                                  │
Phase 3 (camera_info cap) ────► Network stable ──► All topics reliable ──────────┤
                                                                                  │
Phase 2 (sync slop fix) ──────► Point clouds produced ──────────────────────────►│
                                                                                  │
Phase 4 (VIO divergence) ─────► Stable localization ────────────────────────────►│
                                                                                  ▼
                                                                    Fused pointcloud working
```

**Recommended execution order**: Phase 1 → Phase 2 → Phase 3 → Phase 4

Phase 1 (ArUco fix) is highest priority because it's the root enabler of VIO divergence and is a trivial one-line fix. Phase 2 (sync slop) is next because it directly unblocks point cloud generation on the host side. Phase 3 (camera_info) reduces network pressure. Phase 4 is conditional — only needed if VIO still diverges after the ArUco fix.

---

## What Was Removed from v1 and Why

- **Restamping depth/RGB** (v1 Phase 1.2–1.3): Removed. The operator correctly notes that original capture timestamps are important for TF-based cloud transformation. The Jetson relay already has token-gating that locks triplets. The correct fix is increasing the sync slop, not changing timestamps.

- **GTSAM tracker enablement** (v1 Phase 5.1): Removed. The operator does not want to enable GTSAM at this time.

- **Init guard timeout reduction** (v1 Phase 4.2): Removed. Log analysis shows the init guard is working correctly — it's event-driven (checks on every odom message), and the 30s fallback timer is just a safety net that wasn't even needed in this run (both cameras initialized via covariance detection). The actual TF chain breakage is caused by post-init VIO divergence triggering outlier suppression, not by the init guard being too slow.

- **Velocity-based VIO health gate** (v1 Phase 5.4): Removed from immediate scope. The existing position-norm and jump-magnitude checks are sufficient for the current divergence patterns.
