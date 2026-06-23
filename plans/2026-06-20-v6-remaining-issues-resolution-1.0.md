# V6 Remaining Issues — Diagnosis and Resolution Plan

## Objective

Identify and resolve all remaining issues blocking the multiview prosthesis pipeline from producing fused point clouds and stable localization. The latest analysis report (`host-log-20260620_114259`, bag `v6_20260620_114341`, Jetson log `jetson-run-jetson-debug-20260620_134300`) reveals **five interconnected critical issues** that together prevent the system from functioning. This plan addresses each issue in priority order, tracing the root-cause chain and proposing concrete fixes.

---

## Issue Summary (Priority-Ranked)

| # | Issue | Severity | Evidence | Blocks |
|---|-------|----------|----------|--------|
| 1 | **aruco_marker_pose_node crashing on Jetson** | CRITICAL | 2 crashes (exit code 1), VIO=INVALID, 0 markers detected | ArUco corrections, VIO stabilization, corrected TF frames |
| 2 | **100% backprojection frame drop (timestamp mismatch)** | CRITICAL | depth=3.1Hz, rgb=4.6Hz → points=0.0Hz, 100% drop rate | All point clouds, fusion, segmentation, twist propagation |
| 3 | **camera_info topic flooding (238–538 Hz)** | HIGH | Peak 538 Hz on head, 238 Hz on arm; 830% DDS overhead | Network stability, potential packet loss |
| 4 | **TF chain disconnected entire run** | CRITICAL | 18/18 blocks DISCONNECTED for arm, 9/9 for head | Pointcloud fusion TF wait gate never opens |
| 5 | **Arm VIO catastrophic divergence** | CRITICAL | X=[-0.334, 310]m, max velocity 1094 m/s, path 3332m in 45s | Stable localization, pose estimation |

---

## Root-Cause Chain Analysis

The five issues are **not independent** — they form a cascading failure chain:

```
[Issue 3: camera_info flooding]
    ↓ (830% DDS overhead → packet loss / CPU exhaustion)
[Issue 1: aruco_marker_pose_node crash]
    ↓ (no ArUco corrections → VIO has no absolute reference)
[Issue 5: VIO runaway divergence]
    ↓ (relay outlier suppression blocks poses >1.0m norm)
[Issue 4: TF chain never connects]
    ↓ (fusion TF wait gate stays closed)
    +
[Issue 2: timestamp mismatch in synchronizer]
    ↓ (100% frame drop in point_cloud_xyzrgb_node)
    ↓
[Result: zero point clouds, zero fused output, pipeline non-functional]
```

**Key insight**: Issues 1 and 2 are the most immediately fixable on the host side. Issue 1 requires Jetson-side investigation. Issue 5 is the deepest algorithmic problem but is partially mitigated once Issue 1 is resolved (ArUco corrections keep VIO bounded).

---

## Implementation Plan

### Phase 1: Fix Backprojection Timestamp Synchronization (Issue 2)

**Rationale**: This is the most impactful host-side fix. Depth and RGB arrive at healthy rates (2.6–4.7 Hz) but the `point_cloud_xyzrgb_node`'s `ApproximateTimeSynchronizer` with `slop=0.06` discards 100% of frames because the Jetson sensor timestamps on depth and RGB don't match closely enough. The 879ms clock skew compounds this. This fix alone would restore point cloud generation.

- [ ] **1.1. Increase the ApproximateTimeSynchronizer slop tolerance** in `pipeline.launch.py:400-404` and `pipeline.launch.py:418-422`. The current `slop=0.06` (60ms) is too tight given that depth arrives at ~3 Hz and RGB at ~4.7 Hz — their timestamps can differ by up to ~200ms naturally. Increase to `slop=0.15` (150ms) to accommodate the rate difference while still rejecting truly stale frames. Also increase `queue_size` from 20 to 30 to give the synchronizer more candidates to match.

- [ ] **1.2. Add a restamp option to `decompress_bridge.py`** — When `restamp:=true`, the decompress bridge overwrites the output Image header timestamp with the host clock (`self.get_clock().now()`), eliminating clock-skew-induced sync failures entirely. This is a parameter-controlled fallback: the default remains `restamp:=false` (preserve Jetson timestamps for chrony-synced setups), but it can be enabled when chrony is broken. Add the parameter declaration at `decompress_bridge.py:60-63` and apply it at `decompress_bridge.py:175-176` (where `out.header = msg.header` is set).

- [ ] **1.3. Enable restamp by default for depth and RGB decompress bridges** in `pipeline.launch.py:346-385`. Add `"restamp": true` to each of the four decompress bridge Node parameter blocks. This ensures all four images share the host clock domain, making the synchronizer trivially correct. Document that this sacrifices header-stamp-based TF lookup accuracy (the fusion node's clock-skew auto-fallback at `pointcloud_fusion_node.py:452-467` will activate and use arrival-time stamping).

- [ ] **1.4. Verify the fix with a targeted test**: Run the pipeline for 60 seconds with `debug_monitor:=true` and confirm `/jetson/head/points` and `/jetson/arm/points` show non-zero Hz in the `[DIAG-PC]` diagnostic blocks. The `pipeline_diagnostics_node` already monitors these topics (`pipeline_diagnostics_node.py:22-24`).

### Phase 2: Mitigate camera_info Topic Flooding (Issue 3)

**Rationale**: The `camera_info` topic is spiking to 238–538 Hz (should be ~5–10 Hz). At 0.4KB per message, 538 Hz = ~215 KB/s of pure overhead. This is the primary cause of the 830% DDS overhead (expected 0.35 MB/s, actual 3.3 MB/s). The flooding likely causes CPU exhaustion on both Jetson (60% avg, 100% peak) and host, and may contribute to packet loss affecting all topics.

- [ ] **2.1. Investigate the camera_info publishing rate on the Jetson side.** The `realsense2_camera_node` (pid=40) publishes camera_info natively. The `jetson_relay` (pid=44) relays it to the host. Check whether the relay is re-publishing at the native camera frame rate (typically 15–30 Hz for D435) or whether a QoS mismatch is causing retransmission bursts. The 538 Hz peak suggests either a burst on startup or a RELIABLE QoS retransmission storm.

- [ ] **2.2. Throttle camera_info on the Jetson relay.** Camera intrinsics are constant for a given resolution — there is no need to relay them at more than 1–2 Hz. Add a throttle to the `jetson_relay` node (Jetson-side code, in the `jetson-docker` repo) that limits camera_info output to 2 Hz with latched semantics. This alone should reduce DDS overhead by ~90%.

- [ ] **2.3. Add a host-side camera_info throttle as a fallback.** If the Jetson-side fix is delayed, add a lightweight throttle node on the host that subscribes to `/jetson/*/camera_info` and republishes at 2 Hz on a throttled topic. Update the `point_cloud_xyzrgb_node` remappings in `pipeline.launch.py:395-396,413-414` to consume from the throttled topic. This is a temporary measure until the Jetson relay is fixed.

- [ ] **2.4. Verify DDS overhead reduction**: After the fix, re-run `make analyze` and confirm the NIC rx average drops from 3.3 MB/s to under 0.6 MB/s, and the "Massive DDS/RTPS Network Overhead" warning disappears.

### Phase 3: Diagnose and Fix aruco_marker_pose_node Crash (Issue 1)

**Rationale**: The ArUco marker pose node crashed twice (exit code 1) on the Jetson, leaving both head and arm VIO without marker corrections. This directly enables the VIO divergence in Issue 5. The crash tracebacks are at lines 4038 and 5016 of the Jetson log — these need to be examined.

- [ ] **3.1. Retrieve and analyze the crash tracebacks** from the Jetson log file `logs/jetson-run-jetson-debug-20260620_134300.txt` at lines 4038 and 5016. The tracebacks will identify the exact exception (likely an import error, camera frame format mismatch, or OpenCV ArUco detection failure). The node path is `/miahand_ws/src/install_overlay/sensor_fusion_bringup/lib/...` indicating it lives in the Jetson-side `sensor_fusion_bringup` package.

- [ ] **3.2. Fix the identified crash cause** in the `aruco_marker_pose_node.py` source (Jetson-side `jetson-docker` repo, `sensor_fusion_bringup` package). Common causes for exit code 1 in ArUco detection nodes: (a) OpenCV ArUco module not available (`cv2.aruco` import fails), (b) dictionary ID mismatch, (c) image encoding mismatch between RealSense and the detector, (d) parameter file missing.

- [ ] **3.3. Add a crash-recovery wrapper** — If the node crashes again, the system should restart it automatically. Add `respawn=True` and `respawn_delay=2.0` to the ArUco node entry in the Jetson launch file so it automatically restarts on crash. This is a safety net, not a fix — the root cause from 3.2 must still be addressed.

- [ ] **3.4. Verify ArUco corrections are flowing**: After the fix, check that `/head/marker_pose/vio_valid` and `/arm/marker_pose/vio_valid` publish `True` within 30 seconds of startup, and that the Jetson log shows non-zero "Markers" and "Corrections" in the VIO state estimation health section.

### Phase 4: Fix TF Chain Connectivity (Issue 4)

**Rationale**: The TF chain `marker_map → *_imu → *_cam0 → *_d435i_*_link → *_depth_optical_frame` was DISCONNECTED for the entire run. The fusion node's TF wait gate (`pointcloud_fusion_node.py:826-910`) never opened. This is partly a consequence of Issues 1+5 (relay suppresses diverging poses → no `marker_map → *_imu` edge), but also has independent causes in the bridge node startup sequence.

- [ ] **4.1. Verify the relay initialization guard behavior.** The relay at `openvins_odom_tf_relay.py:367-409` blocks TF publishing until OpenVINS covariance trace > 1e-12 or the 30-second fallback timeout. In this run, head odom only became ready at 13:43:50 (47s after start), and arm odom never showed an init marker. Check whether the relay's 30-second fallback timer actually fired — the status log shows "WAITING" for both sides at 13:43:53 (50s in), suggesting the fallback may not be working as expected.

- [ ] **4.2. Reduce the init guard fallback timeout** from 30s to 15s (`openvins_odom_tf_relay.py:57`). In a 105-second run, waiting 30+ seconds for the fallback means nearly a third of the run is lost. The fallback exists specifically for cases where covariance never becomes non-zero — 15 seconds is sufficient to distinguish "still initializing" from "will never converge."

- [ ] **4.3. Verify the bridge node (`openvins_realsense_tf_bridge_node`) resolves the cam0→link extrinsic.** The bridge needs the RealSense static chain to be present to resolve `T(color_optical → link)`. Since the bridge publishes the nominal static chain itself (`openvins_realsense_tf_bridge_node.py:325-351`), this should be self-contained. But if the bridge's `_startup_tick` never resolves the extrinsic (because the `marker_map → head_cam0` edge is missing due to relay suppression), the bridge stays in startup phase indefinitely. Add a fallback: if the extrinsic hasn't resolved after 10 seconds, use the nominal `T(color_optical → link)` from the RealSense URDF values (which are already published as part of the nominal static chain).

- [ ] **4.4. Add diagnostic logging to the TF wait gate** — The gate at `pointcloud_fusion_node.py:826-910` already logs which specific frame is disconnected. Enhance the `pipeline_diagnostics_node` to also report which **edge** in the chain is missing (e.g., "marker_map→head_imu missing" vs "head_cam0→head_d435i_head_link missing") so the root cause is immediately visible without manual TF inspection.

### Phase 5: Address VIO Divergence (Issue 5 — Algorithmic, Longer Term)

**Rationale**: The arm VIO divergence (X reaching 310m, velocity 1094 m/s) is the deepest algorithmic problem. It originates in the OpenVINS estimator on the Jetson and is the #1 problem documented in `docs/CURRENT_ISSUES.md:23-27`. While Issues 1–4 are necessary preconditions (without point clouds and TF, nothing works), Issue 5 determines whether the system produces *accurate* results once it produces *any* results.

- [ ] **5.1. Enable the GTSAM tracker in legacy fusion mode.** Currently `fusion_mode='legacy'` does not launch the GTSAM tracker (`/gtsam/head_pose` and `/gtsam/arm_pose` both at 0.0 Hz). The GTSAM tracker provides the kinematic range factor (1.0m soft bound between head and arm) and the delta gate (rejects >0.5m jumps) that are specifically designed to suppress VIO divergence. Modify `pipeline.launch.py` to optionally launch the GTSAM tracker alongside legacy fusion, or switch to `fusion_mode='gtsam_only'` which already does this. The config already has `gtsam_tracker.broadcast_tf: true` and `openvins_odom_tf_relay.publish_dynamic_tf: true` — these need to be coordinated (set relay's `publish_dynamic_tf: false` when GTSAM is active to avoid TF conflicts).

- [ ] **5.2. Implement the dual-mode chi2 gate for ArUco corrections.** Per `docs/CURRENT_ISSUES.md:169-173, 224-234`, the current chi2 gate rejects 1,462 marker corrections (mean chi2=960 vs gate=10.83) because the estimator state is so far off that marker measurements look statistically impossible. The fix (documented in `docs/CURRENT_ISSUES.md:328-330`) is a dual-mode policy: when VIO health is INVALID, use a wide gate (e.g., chi2 threshold = 1000) to let corrections pull the estimate back; when VIO is valid, use the tight gate (10.83) to protect smoothness. This requires modifying the `aruco_marker_pose_node` on the Jetson side.

- [ ] **5.3. Investigate IMU bias drift as a root cause.** The operator noted in `docs/CURRENT_ISSUES.md:78-81` that the IMU was calibrated with Kalibr but excessive drift persists. The `[DRIFT-INCIDENT]` telemetry (with `pos_slope`) was added to distinguish gradual IMU bias drift from sudden visual-initiated jumps. Analyze the next run's `[DRIFT-INCIDENT]` logs to determine whether the divergence starts gradually (IMU bias) or suddenly (visual tracking failure). If IMU bias is confirmed, investigate whether the OpenVINS IMU initialization period is sufficient and whether the GY-91 IMU's bias stability meets the estimator's assumptions.

- [ ] **5.4. Add a velocity-based VIO health gate to the relay.** The relay at `openvins_odom_tf_relay.py:411-438` currently suppresses poses based on position norm (>1.0m) and jump magnitude (>0.20m). Add a third check: velocity plausibility. If the implied velocity between consecutive accepted poses exceeds `max_velocity_mps` (e.g., 3.0 m/s — well above any real human movement), suppress the TF and flag the VIO as diverging. This provides faster divergence detection than waiting for the position norm to exceed 1.0m.

### Phase 6: Clock Synchronization Hardening

**Rationale**: The 879ms peak clock offset (20/20 blocks exceeded 50ms) directly causes the timestamp mismatch in Issue 2 and degrades TF lookup accuracy throughout the system. While Phase 1's restamp fix works around this for backprojection, the underlying chrony sync should be fixed for correct operation of the GTSAM tracker, keyframe buffer, and cross-camera features.

- [ ] **6.1. Verify chrony is running and synchronized** on both machines before each capture session. Add a pre-flight check to the Makefile (`make timesync-check`) that verifies `chronyc tracking` shows offset < 50ms on the Jetson. If chrony is not running, run `make timesync` to set it up.

- [ ] **6.2. Investigate the 879ms offset source.** The sysmon overlay shows `Drift: min=0.0ms avg=0.0ms max=0.0ms` for both host and Jetson, which contradicts the 879ms clock offset measured from message timestamps. This discrepancy suggests either (a) the sysmon drift measurement is not capturing the full picture (it may measure RTT, not absolute offset), or (b) the offset is in the message header stamping on the Jetson (sensor clock vs system clock), not in the system clock sync. Investigate whether the Jetson relay uses `sensor_msgs::msg::Header::stamp` from the RealSense driver (which may use a separate hardware clock) or `rclcpp::Clock::now()` (which uses the system clock synchronized by chrony).

---

## Verification Criteria

- [ ] `/jetson/head/points` and `/jetson/arm/points` show >0 Hz in diagnostic output (currently 0.0 Hz)
- [ ] `/fused_pointcloud` shows >0 Hz (currently ABSENT — 0 messages)
- [ ] TF chain `marker_map → head_d435i_head_depth_optical_frame` connects within 20 seconds of startup (currently DISCONNECTED entire run)
- [ ] `aruco_marker_pose_node` runs without crashing for the full capture duration (currently crashes twice)
- [ ] DDS network overhead drops below 100% (expected/actual ratio; currently 830%)
- [ ] camera_info topic rate stabilizes at <10 Hz (currently peaks at 538 Hz)
- [ ] Head-arm relative distance stays within 0.2–1.5m range with std/mean <0.1 (currently mean=807m, std=936m)
- [ ] No velocity warnings >2.0 m/s in steady-state operation (currently 1673 warnings on arm)

## Potential Risks and Mitigations

1. **aruco_marker_pose_node fix requires Jetson-side changes**
   Mitigation: The crash tracebacks must be read from the Jetson log to identify the root cause. If the fix requires changes to the `jetson-docker` repo (separate from this repo), coordinate with the Jetson-side developer. In the interim, the GTSAM tracker (Phase 5.1) provides partial localization stabilization without ArUco corrections.

2. **Restamping depth/RGB loses TF-time consistency**
   Mitigation: The fusion node's clock-skew auto-fallback (`pointcloud_fusion_node.py:452-467`) already handles this gracefully by switching to arrival-time stamping. The tradeoff is slightly less precise TF-to-cloud temporal alignment, but this is far better than 100% frame drops. Once chrony is verified working (Phase 6), restamping can be disabled.

3. **Increasing sync slop may match wrong frame pairs**
   Mitigation: At 3–5 Hz with slop=0.15s, the synchronizer will match the nearest pair within 150ms. Since depth and RGB are captured near-simultaneously by the RealSense hardware (within ~30ms), the correct pair will always be the closest match. The risk of mismatching non-corresponding frames is negligible at these low rates.

4. **GTSAM tracker may not converge with severely diverging arm VIO**
   Mitigation: The delta gate (`gtsam_tracker_node.py:543-570`) rejects odometry deltas >0.5m, preventing poisoned data from entering the graph. The kinematic range factor provides a soft 1.0m leash. If the arm VIO is completely unusable, the GTSAM tracker will effectively run head-only with the arm pose derived from the kinematic constraint — degraded but functional.

5. **camera_info throttle may break point_cloud_xyzrgb_node if intrinsics change**
   Mitigation: Camera intrinsics are constant for a given resolution and do not change at runtime. A 2 Hz throttle with latched QoS ensures the host always has the latest (unchanging) intrinsics. The throttle only reduces redundant retransmission, not information loss.

## Alternative Approaches

1. **Switch to `fusion_mode='gtsam_only'` instead of fixing legacy mode**: This would launch the GTSAM tracker, keyframe buffer, and cross-camera features alongside the legacy segmentation pipeline. The GTSAM tracker's delta gate and kinematic range factor directly address VIO divergence. However, this requires the GTSAM tracker to be stable and the cross-camera features (SIFT backend) to be functional — both need verification. **Trade-off**: Faster path to stable localization, but introduces more moving parts.

2. **Switch to `fusion_mode='tsdf_preview'`**: This uses the TSDF fusion path instead of legacy pointcloud fusion. The TSDF node integrates keyframes over time and is more tolerant of individual frame drops. However, it requires the keyframe buffer and GTSAM poses to be functional, and it does not produce `/fused_pointcloud` in the same format. **Trade-off**: More robust to frame drops, but different output format and additional dependencies.

3. **Move point cloud reconstruction to the Jetson**: Instead of sending compressed depth+RGB and reconstructing on the host, generate point clouds on the Jetson and send them directly. This eliminates the timestamp sync problem entirely. **Trade-off**: Significantly higher network bandwidth (PointCloud2 at 320×240 = ~460KB per frame vs ~43KB for compressed depth+RGB). This was the original architecture and was abandoned due to congestion. Not recommended.

---

## Dependency Graph

```
Phase 1 (timestamp sync) ──────┐
                                ├──► Point clouds produced ──┐
Phase 4 (TF chain) ────────────┘                             │
                                                              ├──► Fused pointcloud
Phase 3 (ArUco crash) ─────► VIO stabilized ─────► TF stable ─┘
                                │
Phase 5 (VIO divergence) ──────┘
                                │
Phase 2 (camera_info flood) ──► Network stable ──► All topics reliable
                                │
Phase 6 (chrony sync) ────────┘
```

**Recommended execution order**: Phase 2 → Phase 1 → Phase 4 → Phase 3 → Phase 6 → Phase 5

Phase 2 (camera_info throttle) should be done first because it's the simplest fix with the broadest impact (network stability affects everything). Phase 1 (timestamp sync) is next because it directly unblocks point cloud generation. Phase 4 (TF chain) depends on having non-diverging VIO, so Phase 3 (ArUco crash fix) should follow closely. Phase 5 (VIO divergence) is the longest-term fix and can proceed in parallel once the pipeline is producing output.
