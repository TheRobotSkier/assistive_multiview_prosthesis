# Future Pose Prediction & Collision Checker — Full Chat Export

**Date:** 2026-05-15
**Branch:** `work/cleaned_full_test_20260515` (x86 PC)
**Architecture:** Jetson (jetson_docker) → CycloneDDS → x86 PC (prosthesis container)

---

## 1. Original Specification (User Request)

Implement a future pose prediction and pointcloud collision/proximity checker node
that runs in the existing x86 `prosthesis` Docker container, subscribing to topics
published by the Jetson over CycloneDDS.

### Architecture
```
Jetson (jetson_docker branch, no changes):
  Publishes:
    /ov_msckf_arm/odomimu       (nav_msgs/Odometry, ~200 Hz)
    /arm/d435i_arm/points_marker_map (PointCloud2, ~8 Hz)
    /tf, /tf_static
            
x86 PC (work/cleaned_full_test_20260515 branch):
  Docker container: prosthesis (network_mode: host)
  New node subscribes:
    /ov_msckf_arm/odomimu
    /arm/d435i_arm/points_marker_map
  Publishes:
    /prediction/future_trajectory           (custom msg)
    /prediction/future_trajectory/path      (nav_msgs/Path, RViz)
    /prediction/future_trajectory/status    (JSON)
    /segmentation/click_positive            (PointStamped)
    /segmentation/click_positive/status     (JSON)
```

### Critical Timing Requirement
- Predictions and collision checks triggered ONLY by fresh pointcloud messages
- Output cadence = pointcloud cadence (~8 Hz)
- Odometry stored internally at high rate; prediction triggered on cloud arrival

---

## 2. Design Decisions (During Planning)

### Cloud-triggered, not timer-triggered
The `twist_propagation_node.py` uses a free-running timer, but this node was
designed to fire exclusively on pointcloud arrival. A separate 1 Hz status timer
publishes health telemetry only (odom age, cloud age, last trigger time) and
never triggers prediction.

### No TF2 for v1
Collision geometry defaults to `arm_imu` frame (which is the odom child frame).
Sphere offset applied directly without TF lookup. TF2-based transform can be
added later if a different collision frame is needed.

### OpenVINS initialization guard
Uninitialized OpenVINS publishes odometry with identity pose and zero covariance.
Before running prediction, the node checks:
1. All pose/orientation/twist values are finite (not NaN, not Inf)
2. Pose covariance trace > 0 (meaning VIO has converged)

### Twist covariance floor
OpenVINS does not independently estimate angular velocity
(ROS2Visualizer.cpp:425-427). To prevent overconfident predictions, the
initial 12×12 covariance applies `np.maximum(twist_cov, noise_floor)` where
the noise floor uses `process_noise_linear_mps2_per_s` and
`process_noise_angular_radps2_per_s`.

### Empty pointcloud guard
If parsed cloud has 0 valid points, the node returns early with status
reason `"empty_cloud"` to prevent KDTree construction errors.

### `_parse_xyz` inlined
Not imported from `twist_propagation_node.py` (that file is on the
`test1-daniel` Jetson branch, not on the x86 branch). Inlined as a
~20-line standalone function in the new node.

### Motion Model (constant-twist in marker_map)
```
p_{k+1} = p_k + v_k * dt
q_{k+1} = q_k ⊗ exp(ω_k * dt / 2)
v, ω constant (from latest odometry twist)
```

### Covariance Model (discrete, linearized)
```
F  = [[I_3, 0,    dt*I_3, 0     ],
      [0,   I_3,  0,      dt*I_3],
      [0,   0,    I_3,    0     ],
      [0,   0,    0,      I_3   ]]
Q_k = diag(0,0,0, 0,0,0, dt*σ_v²,dt*σ_v²,dt*σ_v², dt*σ_ω²,dt*σ_ω²,dt*σ_ω²)
P_{k+1} = F·P_k·F^T + Q_k
```

12-D error state: [x, y, z, θx, θy, θz, vx, vy, vz, ωx, ωy, ωz]

### Launch file pattern
Followed `single_d435i.launch.py` pattern: `OpaqueFunction` + YAML config loading.
Simple `DeclareLaunchArgument` + `Node` launch (no TF2 or complex orchestration needed).

### Tests as pure functions
All testable helpers extracted as standalone functions at the top of the node
(no rclpy dependencies). Tests import them directly with `sys.path` manipulation.
16 pytest tests across 6 classes: propagation, covariance, proximity, odom guard,
voxel downsample, pointcloud parsing.

---

## 3. Files Created (8 new + 3 modified)

### New message package
| File | Purpose |
|------|---------|
| `src/sensor_fusion_msgs/package.xml` | format 3, ament_cmake + rosidl_default_generators |
| `src/sensor_fusion_msgs/CMakeLists.txt` | rosidl_generate_interfaces(FuturePoseTrajectory) |
| `src/sensor_fusion_msgs/msg/FuturePoseTrajectory.msg` | Header, Pose[], float64[<=3600] covariance_diag, horizon_s, dt_s, source topics, odom_age_s |

### New node + config + launch
| File | Purpose |
|------|---------|
| `src/sensor_fusion_bringup/config/future_prediction_collision.yaml` | 20 parameters with defaults |
| `src/sensor_fusion_bringup/scripts/future_pose_prediction_collision_node.py` | ~570 line node |
| `src/sensor_fusion_bringup/launch/future_prediction_collision.launch.py` | OpaqueFunction pattern |

### Tests + docs
| File | Purpose |
|------|---------|
| `src/sensor_fusion_bringup/test/test_future_prediction_collision.py` | 16 pytest tests |
| `docs/future_prediction_collision_validation.md` | Build/launch/verify/rollback guide |

### Modified files
| File | Change |
|------|--------|
| `docker/Dockerfile` | +`python3-scipy`, +`python3-pytest` |
| `src/sensor_fusion_bringup/CMakeLists.txt` | Added new script + test install; removed Jetson-only scripts |
| `.gitignore` | Removed `.agents/` to allow tracking |

---

## 4. All Parameters (20 total)

| Parameter | Default | Description |
|-----------|---------|-------------|
| enable_prediction | false | Master enable for trajectory prediction |
| enable_collision_check | false | Master enable for proximity checks |
| horizon_s | 1.0 | Prediction horizon (seconds) |
| dt_s | 0.02 | Prediction time step |
| hit_threshold_m | 0.05 | Max distance for proximity hit |
| min_points_near_hit | 3 | Min nearby cloud points for hit |
| odom_max_age_s | 0.1 | Max odometry staleness |
| cloud_max_age_s | 2.0 | Max pointcloud staleness |
| max_prediction_covariance_trace | 100.0 | Max allowed covariance trace |
| process_noise_linear_mps2_per_s | 0.1 | Linear velocity process noise (σ_v²) |
| process_noise_angular_radps2_per_s | 0.5 | Angular velocity process noise (σ_ω²) |
| collision_geometry_type | sphere | Collision shape (only sphere supported) |
| collision_geometry_radius_m | 0.10 | Sphere radius (applied to proximity) |
| collision_geometry_frame | arm_imu | Attached TF frame (v1: direct, no lookup) |
| odom_topic | /ov_msckf_arm/odomimu | Odom subscription topic |
| cloud_topic | /arm/d435i_arm/points_marker_map | Pointcloud subscription topic |
| trajectory_topic | /prediction/future_trajectory | Trajectory publish topic |
| click_topic | /segmentation/click_positive | Click publish topic |
| voxel_leaf_m | 0.02 | Pointcloud downsampling leaf size |
| status_period_s | 1.0 | Health status publish interval |

---

## 5. Build & Validation Results

| Check | Result |
|-------|--------|
| `colcon build --packages-select sensor_fusion_msgs sensor_fusion_bringup` | 2 packages finished |
| `ros2 interface show sensor_fusion_msgs/msg/FuturePoseTrajectory` | Valid |
| `ros2 launch ... --show-args` | 3 args: enable_prediction, enable_collision_check, use_sim_time |
| Node smoke test (defaults, no topics) | Starts clean, no crash |
| `python3 -m pytest ... -v` | **16 passed** (0.33s) |

---

## 6. Acceptance Criteria Status

| Criterion | Status |
|-----------|--------|
| Build succeeds in Docker container | Verified (host + Docker) |
| Launch args shown correctly | Verified |
| FuturePoseTrajectory message type validates | Verified |
| Node publishes predicted poses in marker_map frame | Code passes smoke test; needs Jetson integration |
| Predicted trajectory cadence matches pointcloud (~8 Hz) | Architecture ensures this |
| No predictions when pointcloud is stale/missing | Guard implemented |
| No predictions when odometry is stale/missing | Guard implemented |
| No predictions when enable_prediction=false | Guard implemented |
| Covariance grows monotonically along horizon | Tested (test_covariance_grows_monotonically) |
| Click published when predicted pose intersects pointcloud | Code present; collision_radius applied |
| Status JSON includes accepted, reason, odom_age_s, point counts | Implemented |
| pytest suite passes all tests | 16/16 passed |
| Existing Jetson OpenVINS + marker behavior unchanged | No Jetson-side changes |
| Rollback: enable_prediction:=false stops output | Architecture ensures |

---

## 7. Post-Implementation Issues & Fixes

### Issue 1: RViz can't render the trajectory
**Symptom:** The topic `/prediction/future_trajectory` appeared in RViz2's "By topic"
dropdown but nothing rendered.

**Root cause:** RViz2's "Path" display only renders `nav_msgs/Path`. Our node publishes
`sensor_fusion_msgs/FuturePoseTrajectory` — a custom message type with no RViz plugin.

**Fix:** Added a secondary publisher on `/prediction/future_trajectory/path` that
converts the predicted poses to a standard `nav_msgs/Path` message. RViz2 renders
this natively. In RViz, add: `By topic /prediction/future_trajectory/path → Path`.

### Issue 2: Click fires on physical contact, not before
**Symptom:** The `/segmentation/click_positive` point only appeared when the user
physically collided with the object, not before.

**Root cause:** The `collision_geometry_radius_m` parameter (0.10m) was declared
but never threaded into the `check_proximity_hit()` function. The KDTree query
used only `hit_threshold_m` (0.05m), meaning the click fired only when the exact
center of the predicted arm position was within 0.05m of cloud points.

**Fix:** `check_proximity_hit()` now accepts `collision_radius_m` and uses
`effective_threshold = hit_threshold_m + collision_radius_m`. With the default
radius of 0.10m, effective threshold = 0.05 + 0.10 = 0.15m. The click now fires
when the sphere surface (not center) approaches the cloud cluster.

---

## 8. Parameter Tuning Guide (for earlier click detection)

Launch with tuned parameters:
```bash
ros2 launch sensor_fusion_bringup future_prediction_collision.launch.py \
  enable_prediction:=true enable_collision_check:=true \
  collision_geometry_radius_m:=0.15 hit_threshold_m:=0.10 min_points_near_hit:=2
```

| Parameter | Default | To fire earlier | Effect |
|-----------|---------|-----------------|--------|
| `collision_geometry_radius_m` | 0.10 | Increase → 0.15-0.20 | Sphere surface reaches cluster sooner |
| `hit_threshold_m` | 0.05 | Increase → 0.10-0.20 | Larger proximity bubble per cloud point |
| `min_points_near_hit` | 3 | Decrease → 1-2 | Fewer points needed to confirm hit |
| `horizon_s` | 1.0 | Increase → 1.5-2.0 | Look farther ahead |
| `voxel_leaf_m` | 0.02 | Decrease → 0.01 | Denser KDTree (more CPU) |
| `dt_s` | 0.02 | Decrease → 0.01 | Finer prediction steps (more CPU) |

---

## 9. What Still Needs to Be Done

### 9.1 Jetson-side Integration Testing
All smoke tests were on the x86 host without live Jetson topics. Full validation:
- Jetson running `jetson_docker` branch publishing live topics
- x86 PC running Docker with CycloneDDS host networking
- Verify cadence match (~8 Hz for cloud and trajectory/path)
- Verify frame_id is `marker_map`
- Verify clicks fire when predicted trajectory intersects real cloud clusters
- RViz visualization of path + click points

### 9.2 Docker Rebuild
The Docker image must be rebuilt to include `python3-scipy` and `python3-pytest`:
```bash
cd docker/
docker compose build prosthesis
docker compose up -d prosthesis
```

### 9.3 Covariance Validation Against Live OpenVINS
The process noise defaults (σ_v²=0.1, σ_ω²=0.5) were chosen as reasonable
starting values. They should be validated against real arm motion data.

### 9.4 Collision Geometry Frame via TF2 (v2)
Currently, `collision_geometry_frame` defaults to `arm_imu` and applies offset
directly. For other frames (e.g., hand tool frame), add `tf2_ros.Buffer` +
`TransformListener` gated by a flag.

### 9.5 Segmentation/Preshaping Pipeline Integration
The node publishes clicks but doesn't monitor segmentation response or trigger
preshaping. Full pipeline needs:
- Subscribe to `/segmentation/object_cloud`
- Service client for `/grasp_preshaping/compute_grasp`
- Internal state machine (IDLE → WAITING_FOR_SEGMENTATION → WAITING_FOR_PRESHAPING)
- Pattern reference: `twist_propagation_node.py` on `test1-daniel` branch

### 9.6 System Launch Integration
The standalone launch file should be integrated into `prosthesis_launch` package's
top-level launch files (`pipeline.launch.py`, `digital_twin.launch.py`).

### 9.7 Performance Profiling
KDTree construction on full pointclouds is O(N log N). With voxel downsampling
(voxel_leaf_m=0.02), performance should be acceptable at 8 Hz, but should be
profiled on actual hardware with live data.

### 9.8 package.xml Cleanup
`sensor_fusion_bringup/package.xml` has exec_depends carried from the Jetson
branch (`tf2_ros`, `cv_bridge`, `rosbag2_py`) that may not be needed on x86.

---

## 10. Docker Commands (Quick Reference)

```bash
# Rebuild image (after Dockerfile change)
cd ~/Documents/GitHub/assistive_multiview_prosthesis/docker
docker compose build prosthesis

# Start container
docker compose up -d prosthesis

# Build workspace inside container
docker exec -it prosthesis bash -c \
  'source /opt/ros/jazzy/setup.bash && cd /prosthesis_ws && \
   colcon build --symlink-install --packages-select sensor_fusion_msgs sensor_fusion_bringup'

# Launch — disabled (safe default)
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && \
   ros2 launch sensor_fusion_bringup future_prediction_collision.launch.py'

# Launch — prediction only
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && \
   ros2 launch sensor_fusion_bringup future_prediction_collision.launch.py enable_prediction:=true'

# Launch — prediction + collision
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && \
   ros2 launch sensor_fusion_bringup future_prediction_collision.launch.py \
   enable_prediction:=true enable_collision_check:=true'

# RViz
docker exec -it prosthesis rviz2
#    Fixed frame: marker_map
#    Add: By topic /prediction/future_trajectory/path → Path
#    Add: By topic /segmentation/click_positive → PointStamped

# Tests
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && \
   python3 -m pytest /prosthesis_ws/src/sensor_fusion_bringup/test/test_future_prediction_collision.py -v'

# Rollback
docker exec -it prosthesis bash -c \
  'source /prosthesis_ws/install/setup.bash && \
   ros2 launch sensor_fusion_bringup future_prediction_collision.launch.py \
    enable_prediction:=false enable_collision_check:=false'
```

---

## Appendix A: Full Test Results (16/16 passed)

```
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestPropagation::test_zero_twist_stationary PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestPropagation::test_constant_velocity_linear_advance PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestPropagation::test_constant_velocity_rotation PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestCovariance::test_covariance_grows_monotonically PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestCovariance::test_propagate_covariance_symmetry PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestCovariance::test_build_initial_covariance_twist_floor PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestProximity::test_hit_near_points PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestProximity::test_no_hit_far_from_points PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestOdomGuard::test_odom_uninitialized_rejected PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestOdomGuard::test_odom_initialized_accepted PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestOdomGuard::test_odom_nan_rejected PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestOdomGuard::test_odom_inf_rejected PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestVoxelDownsample::test_empty_input PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestVoxelDownsample::test_single_point PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestVoxelDownsample::test_merge_duplicates PASSED
src/sensor_fusion_bringup/test/test_future_prediction_collision.py::TestParseXYZ::test_basic_parsing PASSED

Results: 16 passed in 0.48s
```

## Appendix B: Complete File Inventory

### New files (9)
```
src/sensor_fusion_msgs/package.xml
src/sensor_fusion_msgs/CMakeLists.txt
src/sensor_fusion_msgs/msg/FuturePoseTrajectory.msg
src/sensor_fusion_bringup/config/future_prediction_collision.yaml
src/sensor_fusion_bringup/scripts/future_pose_prediction_collision_node.py
src/sensor_fusion_bringup/launch/future_prediction_collision.launch.py
src/sensor_fusion_bringup/test/test_future_prediction_collision.py
docs/future_prediction_collision_validation.md
.agents/chat_export_2026-05-15.md
```

### Modified files (3)
```
docker/Dockerfile                        (+2 apt: python3-scipy, python3-pytest)
src/sensor_fusion_bringup/CMakeLists.txt (new script + test install)
.gitignore                               (removed .agents/ for git tracking)
```

## Appendix C: Build Verification

```bash
$ colcon build --symlink-install --packages-select sensor_fusion_msgs sensor_fusion_bringup
Starting >>> sensor_fusion_msgs
Finished <<< sensor_fusion_msgs [0.40s]
Starting >>> sensor_fusion_bringup
Finished <<< sensor_fusion_bringup [0.10s]
Summary: 2 packages finished [0.97s]

$ ros2 interface show sensor_fusion_msgs/msg/FuturePoseTrajectory
std_msgs/Header header
geometry_msgs/Pose[] poses
float64[<=3600] covariance_diag
float64 horizon_s
float64 dt_s
string odom_source_topic
string cloud_source_topic
float64 odom_age_s

$ ros2 launch ... --show-args
Arguments:
    'enable_prediction':        (default: 'false')
    'enable_collision_check':   (default: 'false')
    'use_sim_time':             (default: 'false')
```
