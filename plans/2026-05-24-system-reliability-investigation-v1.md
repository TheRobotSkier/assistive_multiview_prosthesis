# System Reliability Investigation — V1/V11, V12, V13 Cross-Run Analysis

**Date:** 2026-05-24
**Status:** Findings only — no code changes
**Scope:** Root cause analysis of non-deterministic behavior across three test runs with identical code

---

## Executive Summary

The multiview prosthesis camera pipeline exhibits severe non-determinism across runs. The same code, same hardware, and same test scenario produce radically different outcomes. This document traces the root causes through source code analysis and cross-run log comparison.

Three test runs were analyzed:
- **v1/v11** (identical logs): jetson-log-v1.txt + camera-test-log-v11.txt
- **v12**: jetson-log-v12.txt + host-log-v12.txt
- **v13**: jetson-log-v13.txt + host-log-v13.txt

---

## Finding 1: OpenVINS Online Calibration Drift (ROOT CAUSE)

**Severity: CRITICAL — the single largest source of non-determinism**

OpenVINS re-estimates the IMU-to-camera extrinsic (`T_cam_imu`) online during each run. These extrinsics are **not fixed** — they vary dramatically between runs for the same physical camera:

### marker-6 (head camera) — T_cam_imu translation:

| Run | t_x | t_y | t_z |
|-----|------|------|-------|
| v1 | 0.023 | -0.002 | -0.004 |
| v12 | 0.015 | 0.011 | **-0.066** |
| v13 | **0.055** | -0.018 | -0.005 |

The z-component alone ranges from -0.004m to -0.066m — a **6.2cm variation** for a rigid mount.

### marker-7 (arm camera) — T_cam_imu translation:

| Run | t_x | t_y | t_z |
|-----|------|------|-------|
| v1 | 0.058 | 0.037 | 0.028 |
| v12 | 0.002 | 0.003 | 0.007 |
| v13 | 0.045 | 0.009 | -0.022 |

The x-component ranges from 0.002m to 0.058m — a **5.6cm variation**.

**Evidence:**
- `jetson-log-v12.txt:5` — marker-7: `cam0 extrinsics = 0.007,-0.007,0.006,1.000 | 0.002,0.003,0.007`
- `jetson-log-v12.txt:33` — marker-6: `cam0 extrinsics = -0.001,-0.006,0.009,1.000 | 0.015,0.011,-0.066`
- `jetson-log-v13.txt` — marker-6: `cam0 extrinsics = 0.001,-0.000,0.000,1.000 | 0.055,-0.018,-0.005`
- `jetson-log-v13.txt` — marker-7: `cam0 extrinsics = -0.000,0.003,0.007,1.000 | 0.045,0.009,-0.022`

**Impact:** The online calibration directly affects:
1. VIO position estimation accuracy — wrong extrinsics = wrong pose
2. The hardcoded extrinsics in `openvins_odom_tf_relay` become stale after one run
3. Point cloud registration misalignment when the TF chain uses outdated values

**Source:** OpenVINS `run_subscribe_msckf` binary on the Jetson (not in this repo — installed from overlay workspace at `/miahand_ws/src/install_overlay/`).

---

## Finding 2: Hardcoded Extrinsics in Host Config Are Wrong After First Run

**Severity: HIGH — compounds Finding 1**

The `openvins_odom_tf_relay` node publishes a static `*_imu -> *_cam0` TF edge using hardcoded values from `config/prosthesis_config.yaml:174-185`:

```yaml
# These values are taken from OpenVINS' live output (cam0 extrinsics)
# observed on the Jetson during the v12 test:
#   head (marker-6): T_cam_imu t=[ 0.015,  0.011, -0.066]
#   arm  (marker-7): T_cam_imu t=[ 0.002,  0.003,  0.007]
imu_to_cam_x_head: -0.015
imu_to_cam_y_head: -0.011
imu_to_cam_z_head: 0.066
imu_to_cam_x_arm: -0.002
imu_to_cam_y_arm: -0.003
imu_to_cam_z_arm: -0.007
```

These values were captured from the v12 run but are **already wrong** for v13. The actual v13 extrinsics differ by up to 4cm (head z: 0.066 vs 0.005; arm x: -0.002 vs -0.045).

**Evidence:**
- `config/prosthesis_config.yaml:172-185` — hardcoded values
- `src/camera/camera/openvins_odom_tf_relay.py:106-125` — parameter declarations
- `src/camera/camera/openvins_odom_tf_relay.py:178-201` — static TF publishing

**Impact:** The `imu -> cam0` static TF edge is wrong, which cascades into the `cam0 -> link` bridge edge and ultimately misaligns the entire point cloud registration chain. The error is up to 6cm in a single direction.

---

## Finding 3: No Initialization Guard in Odom TF Relay

**Severity: HIGH — garbage TFs published during startup**

The `openvins_odom_tf_relay` node has **no check** for whether OpenVINS has actually initialized before publishing TFs. It blindly republishes whatever odometry it receives, including:

- Identity poses (before VIO convergence)
- Zero-covariance messages (uninitialized state)
- Wildly drifting positions (during early VIO convergence)

The `docs/agents/chat_export_2026-05-15.md:56-60` mentions an initialization guard was planned:
> "Uninitialized OpenVINS publishes odometry with identity pose and zero covariance. Before running prediction, the node checks: 1. All pose/orientation/twist values are finite (not NaN, not Inf) 2. Pose covariance trace > 0 (meaning VIO has converged)"

But **this guard was never implemented** in `openvins_odom_tf_relay.py`. The `_on_odom` method at line 228-252 publishes every message without any validation.

**Evidence:**
- `src/camera/camera/openvins_odom_tf_relay.py:228-252` — no covariance/finite checks
- `docs/agents/chat_export_2026-05-15.md:56-60` — planned but not implemented

**Impact:** During the first 40-80 seconds of each run, garbage TFs flood the system. The TF buffer accumulates wildly wrong transforms that downstream nodes (fusion, bbox removal) attempt to use, causing:
- Point clouds placed at absurd positions
- TF extrapolation errors
- Fusion stalls

---

## Finding 4: ArUco Marker Detection is Completely Non-Deterministic

**Severity: CRITICAL — the second largest source of non-determinism**

The ArUco marker EKF update count varies from 0 to 80+ across runs with no code changes:

| Run | marker-6 (head) ArUco updates | marker-7 (arm) ArUco updates | Total |
|-----|------|------|-------|
| v1 | ~1-2 (then rejected) | 0 | ~1-2 |
| v12 | **80+** (all accepted, chi2 ≤ 0.006) | 0 | 80+ |
| v13 | **0** | **0** | 0 |

The ArUco node source (`aruco_marker_pose_node.py`) is **not in this repo** — it's installed from the Jetson overlay workspace. Its runtime behavior can only be inferred from logs.

In v13, the ArUco node produces **zero runtime output** — no VIO health checks, no marker detections, nothing. It only shows shutdown errors. This suggests it either:
- Never received camera images
- Never detected any markers
- Failed to subscribe to the correct topics

**Evidence:**
- `jetson-log-v12.txt:38,59,70,101,102,143` — 80+ accepted updates for marker-6 only
- `jetson-log-v13.txt` — zero MARKER lines in the entire log
- `jetson-log-v13.txt:8155-8205` — only shutdown tracebacks from aruco nodes
- `jetson-log-v1.txt:104,620,1579,2227,2883,5518,5519,5586` — intermittent VIO health checks

**Impact:** Without ArUco corrections, VIO drifts without bound. This is why marker-7 (arm in v1, head in v12/v13) drifts hundreds of meters. When ArUco works (v12 marker-6), the position locks to within 2mm. When it doesn't (v13), the position drifts freely.

---

## Finding 5: Marker-to-Camera Mapping Confusion

**Severity: MEDIUM — documentation/config inconsistency, not a runtime bug**

The config comments in `prosthesis_config.yaml:172-173` say:
```
#   head (marker-6): T_cam_imu t=[ 0.015,  0.011, -0.066]
#   arm  (marker-7): T_cam_imu t=[ 0.002,  0.003,  0.007]
```

This mapping is confirmed by matching the `cam0 extrinsics` in the logs to these values. However, the naming is confusing because:
- `run_subscribe_msckf_marker-6` corresponds to the **head** camera (small T_cam_imu z-translation when calibrated)
- `run_subscribe_msckf_marker-7` corresponds to the **arm** camera (small T_cam_imu overall)

The ArUco nodes are named:
- `aruco_marker_pose_node_phase2` (pid 3) — monitors marker-7 (arm)
- `aruco_marker_pose_node_arm_phase2` (pid 4) — monitors marker-6 (head)

The `_phase2` node monitors marker-7 (arm) and the `_arm_phase2` node monitors marker-6 (head) — the naming is inverted relative to what you'd expect.

**Evidence:**
- `jetson-log-v1.txt:104` — `aruco_marker_pose_node_phase2` interleaved with marker-7 output
- `jetson-log-v1.txt:1579` — `aruco_marker_pose_node_arm_phase2` reports `position_outside_workspace` for marker-6
- `jetson-log-v12.txt:7952-7953` — shutdown shows both node names

---

## Finding 6: VIO Gyroscope Bias Converges to Different Values Each Run

**Severity: HIGH — directly causes position drift variability**

The gyroscope bias (`bg`) converges to different values in every run:

### marker-6 (head camera) bg:

| Run | bg_x | bg_y | bg_z |
|-----|-------|-------|-------|
| v1 | -0.0001 | 0.0038 | 0.0031 |
| v12 | 0.0043 | -0.0202 | -0.0009 |
| v13 | 0.0127 | 0.1012 | -0.0022 |

The bg_y component ranges from -0.020 to **0.101** — a 5x variation. In v13, the bg_y value of 0.1012 is an order of magnitude larger than typical, suggesting the estimator converged to a wrong local minimum.

### marker-7 (arm camera) bg:

| Run | bg_x | bg_y | bg_z |
|-----|-------|-------|-------|
| v1 | 0.0096 | -0.0004 | 0.0036 |
| v12 | 0.0057 | 0.0032 | -0.0167 |
| v13 | 0.0168 | -0.0039 | 0.0023 |

**Evidence:**
- `jetson-log-v12.txt:670` — marker-6 bg = 0.0043,-0.0202,-0.0009
- `jetson-log-v13.txt` — marker-6 bg = 0.0127,0.1012,-0.0022
- `jetson-log-v1.txt:5520` — marker-7 bg = 0.0096,-0.0004,0.0036

**Impact:** Wrong gyroscope bias causes orientation drift, which causes position drift through integration. The v13 marker-6 bg_y of 0.1012 is pathological — it would cause ~5.8 deg/sec rotation error, explaining the rapid position drift.

---

## Finding 7: VIO Static Initialization is Fragile

**Severity: MEDIUM — causes startup delays and bad initial state**

The arm camera (marker-6 in v1) repeatedly fails static initialization with "no accel jerk detected" (`jetson-log-v1.txt:107-137`). The initialization requires a specific motion pattern ("accel jerk") to seed the filter. If the camera happens to be stationary or moving smoothly at startup, initialization fails repeatedly.

In v12 and v13, this specific failure doesn't occur, but the fact that it can happen means the initialization path is fragile and depends on the physical motion at the exact moment of startup.

**Evidence:**
- `jetson-log-v1.txt:107-140` — 15+ failed init attempts before success
- `jetson-log-v12.txt` — no init failures (lucky startup motion)
- `jetson-log-v13.txt` — no init failures (lucky startup motion)

---

## Finding 8: TF Bridge Startup Race Condition

**Severity: MEDIUM — 40-80 second delay before system is functional**

The `openvins_realsense_tf_bridge_node` needs to resolve the `cam0 -> link` extrinsic by looking up the RealSense static chain. The `openvins_odom_tf_relay` needs to receive valid odometry from the Jetson. These two nodes start simultaneously, creating a race:

| Run | Time until both OpenVINS chains connected |
|-----|------|
| v11 | **Never** (permanently MISSING) |
| v12 | ~40 seconds |
| v13 | ~80 seconds |

During this period, the fusion node receives no valid TF chain and publishes nothing.

**Evidence:**
- `host-log-v12.txt:62,68,76,81` — MISSING at t+16, t+26, t+36, OK at t+45
- `host-log-v13.txt` — MISSING until ~t+80

The root cause is that the relay publishes garbage TFs during VIO initialization (Finding 3), and the bridge needs valid TFs to resolve the extrinsic. The bridge's `_startup_tick` at `openvins_realsense_tf_bridge_node.py:365-417` keeps trying until it succeeds, but the time to success depends on when VIO converges.

---

## Finding 9: Bbox Arm Removal is Chronically Unreliable

**Severity: MEDIUM — persistent across all runs, not run-dependent**

The bounding box arm removal success rate oscillates between 0% and 83% in every run, regardless of other system health. The root cause is TF transforms for pruning box frames (`palm_frame`, `d435i_arm_bottom_screw_frame_8_cm_cam_mount`) going stale beyond the 2.0s cache limit.

The TF chain for bbox removal is: `marker_map -> arm_d435i_arm_link -> ... -> palm_frame / d435i_arm_bottom_screw_frame_8_cm_cam_mount`. This chain has 6+ edges, and if any single edge is delayed or dropped, the entire lookup fails.

**Evidence:**
- `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:504-541` — bbox removal with TF lookup
- `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:570-592` — `_lookup_bbox_transform` with 500ms timeout
- All host logs — `Bbox removal success rate (0%-85%) below acceptable threshold`

---

## Root Cause Chain

```
OpenVINS online calibration drift (Finding 1)
    ├── Wrong T_cam_imu → wrong VIO pose → position drift
    ├── Hardcoded host extrinsics become stale (Finding 2)
    │   └── Static imu->cam0 TF wrong → point cloud misalignment
    └── Different bg convergence each run (Finding 6)
        └── Orientation drift → position drift via integration

ArUco marker detection non-determinism (Finding 4)
    ├── v12: 80+ corrections → position locks (marker-6 dist=1.70m)
    ├── v13: 0 corrections → position drifts freely (marker-7 dist=1024m)
    └── v1: intermittent corrections → partially constrained

No initialization guard in relay (Finding 3)
    └── Garbage TFs published during startup
        ├── TF bridge can't resolve extrinsics → 40-80s delay (Finding 8)
        └── Fusion node gets garbage transforms → stalls
```

---

## Recommended Fixes (Priority Order)

### 1. Disable Online Extrinsic Estimation in OpenVINS
**Impact: Eliminates Finding 1 (root cause)**

OpenVINS should use fixed, pre-calibrated extrinsics rather than re-estimating them online. The `run_subscribe_msckf` binary on the Jetson needs its config changed to lock the `T_cam_imu` extrinsic. This is the single most impactful change.

If online estimation must be kept, the relay should extract the live extrinsic from the OpenVINS odometry message (which includes `cam0 extrinsics` in its output) rather than using hardcoded values.

### 2. Add Initialization Guard to Odom TF Relay
**Impact: Eliminates Finding 3, mitigates Finding 8**

In `src/camera/camera/openvins_odom_tf_relay.py:228-252`, add checks before publishing:
```python
def _on_odom(self, msg, parent, child, name):
    # Skip uninitialized OpenVINS output
    cov = msg.pose.covariance
    if all(c == 0.0 for c in cov):
        return  # Zero covariance = uninitialized
    x, y, z = _extract_translation_from_odom(msg)
    if not all(math.isfinite(v) for v in [x, y, z]):
        return  # NaN/Inf = garbage
    # ... rest of publishing
```

### 3. Investigate ArUco Marker Detection Source
**Impact: Eliminates Finding 4**

The `aruco_marker_pose_node.py` source needs to be examined on the Jetson. Key questions:
- Why does it produce 80+ detections in v12 and zero in v13?
- Is it subscribing to the correct image topics?
- Are there any DDS QoS mismatches that cause image delivery to be intermittent?
- Is there a VIO health gate that prevents marker publication when VIO is unhealthy?

### 4. Extract Live Extrinsics from OpenVINS Output
**Impact: Eliminates Finding 2**

Instead of hardcoding `imu_to_cam_*` values, subscribe to the OpenVINS odometry and extract the `cam0 extrinsics` from its published calibration. Alternatively, have the Jetson publish the extrinsic as a separate topic that the relay can subscribe to.

### 5. Increase VIO Initialization Robustness
**Impact: Mitigates Findings 6 and 7**

Consider:
- Using a longer static initialization window
- Accepting multiple initialization strategies (static + motion-based)
- Seeding the initial bias from a previous run's converged values
- Adding a pre-flight calibration step that ensures the IMU is stationary

---

## Files Examined

| File | Purpose |
|------|---------|
| `config/prosthesis_config.yaml:150-185` | Odom relay config with hardcoded extrinsics |
| `src/camera/camera/openvins_odom_tf_relay.py` | Host-side odom-to-TF relay (no init guard) |
| `src/camera/camera/openvins_realsense_tf_bridge_node.py` | Bridge node with startup race |
| `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:504-592` | Bbox removal with TF lookup |
| `src/sensor_fusion_bringup/scripts/tf_pipeline_diagnostics.py` | TF chain health monitoring |
| `jetson-log-v1.txt`, `jetson-log-v2.txt` | v1/v11 Jetson logs (identical) |
| `jetson-log-v12.txt`, `host-log-v12.txt` | v12 logs |
| `jetson-log-v13.txt`, `host-log-v13.txt` | v13 logs |
| `camera-test-log-v11.txt` | v11 host-side log |
