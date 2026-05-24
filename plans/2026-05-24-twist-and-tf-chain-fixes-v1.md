# Twist Propagation Axis Alignment + TF Chain Fixes

## Session Date: 2026-05-24

---

## Part 1: Twist Propagation Axis Alignment Bug

### Root Cause

In `src/twist_propagation/twist_propagation/twist_propagation_node.py`, the **twist (velocity) and orientation were never transformed to the cloud frame** before being used for propagation. Only the position was transformed. When `pose_frame` (e.g. `marker_map`) differs from `cloud_frame` (e.g. an optical frame), velocity axes get swapped — causing "forward becomes up" behavior.

### The exact problem (in `_run_idle_cycle`)

1. **Position** was correctly transformed from `pose_frame` to `cloud_frame`
2. **Twist** was estimated from pose differences in `pose_frame` coordinates, but was passed directly to propagation **without any frame transformation**
3. **Orientation** was never transformed — the raw quaternion from `pose_frame` was used directly in cloud-frame propagation
4. **External twist** from OpenVINS odometry was in the camera body frame, not in `pose_frame`, and was never converted

### Fix summary

| Addition | Location | Purpose |
|---|---|---|
| `_quat_rotate_vector()` | `twist_propagation_node.py` | Rotate a 3D vector by a quaternion |
| `_quat_to_rotation_matrix_cols()` | `twist_propagation_node.py` | Extract 9-element rotation matrix from quaternion |
| `_transform_twist_by_rotation()` | `twist_propagation_node.py` | Rotate a 6-DOF twist (linear + angular) by a rotation matrix |
| `_transform_full_pose_to_cloud_frame()` | `twist_propagation_node.py` | Replaces old position-only transform — now transforms **both position and orientation** to cloud frame |
| `_transform_twist_to_cloud_frame()` | `twist_propagation_node.py` | Transforms twist from `pose_frame` to `cloud_frame` using TF2 rotation |
| `_run_idle_cycle` updated | `twist_propagation_node.py` | Now transforms full pose + twist before propagation |
| `_estimate_twist` updated | `twist_propagation_node.py` | Converts external twist from body frame to pose frame using current orientation |
| `_frame_mismatch_logged` flag | `twist_propagation_node.py` | One-time log when `pose_frame != cloud_frame` is detected |

### Verification

12 standalone pure-function tests pass (quaternion rotation, twist transformation, propagation with rotated twists, round-trip transforms). Full pytest suite requires ROS2 environment (Docker).

### New unit tests

| Test Class | Tests |
|---|---|
| `TestQuatRotateVector` | Identity, 90-deg Z, 180-deg Y rotations |
| `TestTransformTwistByRotation` | Identity, 90-deg Z, quaternion integration, optical frame scenario |
| `TestPropagationWithFrameTransform` | End-to-end propagation after 90-deg Z and Y rotations |

---

## Part 2: Arm Point Cloud Alignment Bug (imu->cam0 Static Transform)

### Root Cause

The `openvins_odom_tf_relay` was publishing an incorrect `imu -> cam0` static TF using old Kalibr calibration values instead of the Jetson's live OpenVINS calibration. This introduced a systematic offset in the TF chain from `marker_map` to each camera's depth optical frame.

### Evidence from Jetson v12 Log

The Jetson's OpenVINS prints `cam0 extrinsics = q_cam_imu | T_cam_imu`:

| Camera | Marker | OpenVINS T_cam_imu (live) |
|---|---|---|
| Head | marker-6 | `q≈(0,0,0,1), t=[0.015, 0.011, -0.066]` |
| Arm | marker-7 | `q≈(0,0,0,1), t=[0.002, 0.003, 0.007]` |

The relay publishes `T_imu_cam = inv(T_cam_imu) ≈ -T_cam_imu` (since rotation is near-identity):

| Camera | Correct T_imu_cam | Old (stale Kalibr) | Error |
|---|---|---|---|
| Head | `[-0.015, -0.011, 0.066]` | `[0.023, -0.002, -0.004]` | **62mm Z, wrong signs** |
| Arm | `[-0.002, -0.003, -0.007]` | `[0.018, -0.001, -0.001]` | **20mm XY** |

### Why the old values were wrong

The Kalibr calibration files on the Jetson (`kalibr_imucam_chain.yaml`) loaded by OpenVINS contain updated values that differ from the host-side `prosthesis_config.yaml`. The relay config was never updated to match what OpenVINS actually uses.

### Also: Relay missing /tf liveness

The relay published `imu->cam0` only on `/tf_static` ONCE at startup. If the fusion node's TF2 buffer misses the `TRANSIENT_LOCAL` latch (known DDS issue), the chain `marker_map -> arm_imu -> arm_cam0 -> ...` is broken. The bridge solved this same problem with a 2Hz liveness timer on `/tf`.

### Fix

| File | Change |
|---|---|
| `config/prosthesis_config.yaml` | Correct imu->cam0 values from Jetson v12 (signed correctly: T_imu_cam = -T_cam_imu) |
| `src/camera/camera/openvins_odom_tf_relay.py` | Updated code defaults to match config + added 2Hz liveness timer on `/tf` for imu->cam0 edges |

---

## Part 3: Bbox Removal TF Lookup Timeout (v12 Log Analysis)

### Symptoms in v12 Host Log

1. **0-20s**: Arm OpenVINS data doesn't reach host over DDS — arm cloud can't be fused (`dual=0`, `OpenVINS(arm):MISSING`)
2. **20-40s**: Arm chain connects, clouds ARE being dual-fused (`dual=24`), but bbox removal at **0% success rate** — "no cached transform available" for both `palm_frame` and `d435i_arm_bottom_screw_frame_8_cm_cam_mount`
3. **40-90s**: Bbox removal partially recovers via cache mechanism — first successes at ~54s
4. **90s+**: Bbox removal working at ~80% success rate with occasional cache expiration ("cached transform too old")

### Root Cause: Bbox lookup timeout too short

The fusion node's `_lookup_bbox_transform` used a **50ms timeout** for TF chain lookups with 6+ edges:
```
marker_map -> arm_imu -> arm_cam0 -> arm_d435i_arm_link -> screw_frame -> palm_frame
```

The diagnostics node uses a **1s timeout** with `can_transform` and succeeds. The 50ms was insufficient for multi-edge chain composition, causing all fresh lookups to time out. The cache mechanism eventually accumulated transforms from occasional successes, but the initial 40+ seconds of 0% bbox removal meant all fused clouds included arm/camera points that should have been pruned.

### Clock skew issue

Jetson timestamps on point clouds were 2-3 seconds ahead of host TF data, causing "extrapolation into the past" errors for cloud transforms. The fusion node's re-stamping (line 562) mitigates this for downstream consumers but cloud transforms still fail intermittently.

### Fix

| File | Change |
|---|---|
| `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` | Increased `_lookup_bbox_transform` timeout: 50ms -> 500ms. Added `debug`-level logging of the actual TF exception for debugging |

---

## Files Changed (Aggregate)

| File | Change Summary |
|---|---|
| `src/twist_propagation/twist_propagation/twist_propagation_node.py` | 3 new pure helpers + 2 new methods + updated `_run_idle_cycle` + `_estimate_twist` for frame-aware twist propagation |
| `src/twist_propagation/test/test_twist_propagation.py` | 3 new test classes for quaternion rotation, twist transformation, and frame-aware propagation |
| `config/prosthesis_config.yaml` | Correct imu->cam0 values from Jetson v12 OpenVINS calibration |
| `src/camera/camera/openvins_odom_tf_relay.py` | Updated defaults + 2Hz liveness timer for imu->cam0 static transforms |
| `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` | 50ms -> 500ms bbox lookup timeout + TF error debug logging |

---

## Testing Recommendations

1. **Twist propagation**: Run `python3 -m pytest src/twist_propagation/test/test_twist_propagation.py -v` inside the Docker container to verify all frame transformation tests
2. **TF chain**: Run `make camera-test` and verify:
   - `All chains healthy` appears within ~30 seconds (not 60+)
   - `bbox_removed > 0` appears within 5-10 seconds (not 50+)
   - Both clouds overlap in RViz with correct alignment
3. **imu->cam0 values**: Run `ros2 topic echo /tf_static` on the host and verify `arm_imu -> arm_cam0` and `head_imu -> head_cam0` have translations matching the Jetson's live calibration output

---

## Remaining Known Issues (Not Fixed in This Session)

1. **DDS unreliability**: Arm OpenVINS data takes ~20s to start reaching the host. This is a FastRTPS/CycloneDDS issue across the Ethernet link. Workaround: if both machines use the same DDS vendor (e.g., both CycloneDDS) and configure reliable QoS, delivery might improve.
2. **Clock skew**: Jetson clock is 2-3 seconds ahead of host clock. NTP/chrony should be configured on both machines. The fusion node's output re-stamping helps downstream but cloud transforms within the fusion node still fail due to extrapolation.
3. **Bbox cache max age**: Currently 2.0 seconds. With the 500ms timeout fix this should be less critical, but could be increased to 5.0s for robustness during transient TF failures.
