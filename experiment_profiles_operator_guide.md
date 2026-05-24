# OpenVINS Experiment Profiles — Operator Guide

## Quickstart

All profiles are toggled with one launch argument: `openvins_experiment_profile:=<name>`.
Omit it (or pass `baseline`) for unchanged original behavior.

```bash
# Head-only
ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py \
    openvins_experiment_profile:=marker_strong_ekf

# Arm-only
ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py \
    openvins_experiment_profile:=marker_strong_ekf

# Combined head+arm+dynamic ID2
ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py \
    openvins_experiment_profile:=all_changes
```

## Profile Catalog

| Profile | What it toggles | When to use |
|---------|-----------------|-------------|
| `baseline` | Nothing — original defaults | Sanity check, comparing against |
| `marker_strong_ekf` | Trust markers 4× more, relax rejection gates | Marker visible but OV keeps drifting |
| `marker_easier_initial_lock` | Lock to marker_map sooner, zero-vel fallback | Marker visible but `marker_map_locked` stays false |
| `reset_bias_policy` | Zero IMU biases on first marker lock | Post-reset drift suggests bad bias carryover |
| `calib_extrinsics` | Enable online camera-IMU calibration | Camera mount changed, want OV to adapt |
| `zupt` | Enable zero-velocity updates | Stationary bench tests only |
| `imu_frame_variant` | **Resolved — no mismatch found** | No action needed |
| `native_aruco_landmarks` | ArUco corners as SLAM landmarks (OV native) | Compare landmark-only vs marker-pose path |
| `marker_graph_correction` | Head→arm sync via marker chains | Cameras see different markers, need cross-camera sync |
| `all_changes` | Combines strong_ekf + easier_lock + bias_policy | Quick test of maximum marker influence |

## Diagnosing What Happens

Every marker decision is now published as JSON. Monitor in real time:

```bash
# Head OpenVINS — shows every accept/reject/reset reason
ros2 topic echo /ov_msckf/marker_update/status

# Arm OpenVINS
ros2 topic echo /ov_msckf_arm/marker_update/status

# Marker graph (Epic 2) — shows chain quality, hops, head→arm transform
ros2 topic echo /marker_graph_estimator/status

# Graph odom correction status — shows whether correction is active
ros2 topic echo /marker_graph/correction_status

# Compare raw vs corrected arm odom
ros2 topic echo /ov_msckf_arm/odomimu           # raw
ros2 topic echo /ov_msckf_arm/odomimu_corrected  # corrected
```

### Key diagnostic fields in `marker_update/status`

| Field | Meaning |
|-------|---------|
| `event_type` | `ekf_accepted`, `ekf_rejected`, `reset_performed`, `reset_skipped_velocity_fit`, `first_marker_map_lock`, `queue_drop` |
| `chi2` | Chi-squared statistic (< gate = accepted) |
| `noise_multiplier` | Active noise scale (1.0=baseline, 0.25=4× trust under strong_ekf) |
| `active_bias_policy` | `preserve`, `zero`, `zero_on_initial_lock`, `inflate_only` |
| `is_first_lock_attempt` | True when marker_map not yet locked |
| `initial_lock_zero_velocity_fallback` | True when easier_lock profile kicked in |
| `bias_gyro_norm_before` / `bias_gyro_norm_after` | IMU bias magnitudes pre/post reset |
| `marker_map_initialized` | True once marker_map gauge is locked |

## Testing Strategy

### Step 1 — Confirm baseline works
```bash
ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py \
    openvins_experiment_profile:=baseline
```
Verify all topics publish, no launch errors.

### Step 2 — Isolated profile tests
Run each profile one at a time on either the head or arm alone:

```bash
# Test stronger marker trust
ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py \
    openvins_experiment_profile:=marker_strong_ekf

# Test easier initial lock
ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py \
    openvins_experiment_profile:=marker_easier_initial_lock

# Test bias zeroing
ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py \
    openvins_experiment_profile:=reset_bias_policy

# Test online calibration
ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py \
    openvins_experiment_profile:=calib_extrinsics

# Test ZUPT (stationary bench only!)
ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py \
    openvins_experiment_profile:=zupt
```

Monitor `marker_update/status` during each run. Note which profiles increase `accepted` counts and which cause `reset_performed`.

### Step 3 — Combined stress test
```bash
ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py \
    openvins_experiment_profile:=all_changes
```

### Step 4 — Marker graph pipeline (Epic 2)
```bash
# Launch with ArUco landmarks for comparison
ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py \
    openvins_experiment_profile:=native_aruco_landmarks \
    start_marker_graph:=true \
    start_marker_graph_odom_relay:=true

# Launch with marker graph correction only
ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py \
    openvins_experiment_profile:=marker_graph_correction \
    start_marker_graph:=true \
    start_marker_graph_odom_relay:=true
```

## Switching Marker Detector (Python vs C++)

By default the Python `aruco_marker_pose_node.py` runs. Opt into the new C++ frontend:

```bash
# Head with C++ detector
ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py \
    use_cpp_marker_pose:=true

# Arm with C++ detector
ros2 launch sensor_fusion_bringup arm_marker_pose_phase2.launch.py \
    use_cpp_marker_pose:=true
```

The C++ node publishes on the same `/head/marker_pose/observation` topic — OpenVINS doesn't know which detector is running.

## What Did NOT Change

These host-facing interfaces remain stable across all profiles:

```
/ov_msckf/odomimu          /ov_msckf_arm/odomimu
/head/marker_pose/observation  /arm/marker_pose/observation
/ov_msckf/marker_map_locked    /ov_msckf_arm/marker_map_locked
marker_map, head_imu, arm_imu, head_cam0, arm_cam0
```

The host `pc-fusion-hourly` side should need zero changes.

## Running Tests (no Jetson needed)

```bash
# Python: profile parsing + YAML validity (12 tests)
python3 -m pytest docker_ws/multi_cam_localization/sensor_fusion_bringup/test/test_openvins_profiles.py -v

# Python: marker graph edge/path logic (31 tests)
python3 -m pytest docker_ws/multi_cam_localization/sensor_fusion_bringup/test/test_marker_graph.py -v

# C++: marker updater params + validation (18 tests)
# (requires colcon build with BUILD_TESTING=ON)
colcon build --packages-select ov_msckf --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select ov_msckf --ctest-args -R test_marker_pose_updater
```
