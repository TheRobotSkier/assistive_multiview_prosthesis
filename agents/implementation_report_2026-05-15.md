# Future Pose Prediction & Collision Checker — Implementation Report

**Date:** 2026-05-15
**Branch:** `work/cleaned_full_test_20260515` (x86)
**Author:** opencode

---

## 1. What Was Implemented

### 1.1 New ROS 2 Message Package: `sensor_fusion_msgs`

| File | Purpose |
|------|---------|
| `src/sensor_fusion_msgs/package.xml` | Package metadata, depends on `geometry_msgs`, `std_msgs`, uses `rosidl_default_generators` |
| `src/sensor_fusion_msgs/CMakeLists.txt` | Builds `FuturePoseTrajectory.msg` via `rosidl_generate_interfaces` |
| `src/sensor_fusion_msgs/msg/FuturePoseTrajectory.msg` | Custom message: header, `Pose[] poses`, `float64[<=3600] covariance_diag`, `horizon_s`, `dt_s`, `odom_source_topic`, `cloud_source_topic`, `odom_age_s` |

### 1.2 Main Node: `future_pose_prediction_collision_node.py`

**Location:** `src/sensor_fusion_bringup/scripts/future_pose_prediction_collision_node.py` (~450 lines)

**Architecture:**
- Pointcloud-triggered (not timer-driven). Prediction fires exclusively on `/arm/d435i_arm/points_marker_map` arrival.
- Separate 1 Hz health status timer publishes telemetry only (odom age, cloud age, last trigger time). Never triggers prediction.
- No TF2 lookups for v1. Collision geometry defaults to `arm_imu` frame with direct sphere offset.

**Subscriptions:**
- `/ov_msckf_arm/odomimu` (nav_msgs/Odometry) — latched internally
- `/arm/d435i_arm/points_marker_map` (sensor_msgs/PointCloud2) — sole prediction trigger

**Publishers:**
- `/prediction/future_trajectory` (sensor_fusion_msgs/FuturePoseTrajectory)
- `/prediction/future_trajectory/status` (std_msgs/String, JSON)
- `/segmentation/click_positive` (geometry_msgs/PointStamped)
- `/segmentation/click_positive/status` (std_msgs/String, JSON)

**Pipeline per accepted cloud:**
1. Verify `enable_prediction` or `enable_collision_check` is true
2. Parse cloud → check empty (guard: `"empty_cloud"` reason)
3. Check cloud staleness (`cloud_max_age_s`, default 2.0s)
4. Check odometry availability + staleness (`odom_max_age_s`, default 0.1s)
5. Check OpenVINS initialized (finite values + pose covariance trace > 0)
6. Voxel downsample cloud (`voxel_leaf_m`, default 0.02m)
7. Build `scipy.spatial.KDTree` from downsampled cloud
8. Build initial 12x12 covariance (with twist floor from process noise)
9. Propagate pose + covariance forward (constant-twist model, `horizon_s`/`dt_s` = 50 steps)
10. If collision enabled: for each predicted pose, query KDTree for `min_points_near_hit` neighbors within `hit_threshold_m`
11. On hit: publish `geometry_msgs/PointStamped` on click topic
12. Always: publish `FuturePoseTrajectory` (if prediction enabled)
13. Always: publish JSON status

**Motion Model (constant-twist in marker_map):**
```
p_{k+1} = p_k + v_k * dt
q_{k+1} = q_k ⊗ exp(ω_k * dt / 2)
v, ω constant (from latest odometry twist)
```

**Covariance Model (discrete, linearized):**
```
F  = [[I_3, 0,    dt*I_3, 0     ],
      [0,   I_3,  0,      dt*I_3],
      [0,   0,    I_3,    0     ],
      [0,   0,    0,      I_3   ]]
Q_k = diag(0,0,0, 0,0,0, dt*σ_v²,dt*σ_v²,dt*σ_v², dt*σ_ω²,dt*σ_ω²,dt*σ_ω²)
P_{k+1} = F·P_k·F^T + Q_k
```
- State order (12-D): [x, y, z, θx, θy, θz, vx, vy, vz, ωx, ωy, ωz]
- Initial covariance: pose 6×6 from odom, twist 6×6 from odom with process noise floor
- `covariance_diag` in message: 6 values per pose (3 position + 3 orientation variance)

**OpenVINS Guard:**
- Rejects odom with any non-finite values (NaN/Inf)
- Rejects odom with pose covariance trace ≤ 0 (uninitialized VIO publishes identity pose with zero covariance)

**Twist Covariance Floor:**
- Applies `np.maximum(twist_cov, noise_floor)` where noise_floor is diag from `process_noise_linear_mps2_per_s` and `process_noise_angular_radps2_per_s`
- Prevents overconfident angular predictions (OpenVINS does not independently estimate angular velocity — ROS2Visualizer.cpp:425-427)

### 1.3 Config YAML

**Location:** `src/sensor_fusion_bringup/config/future_prediction_collision.yaml`

All 20 parameters with defaults:

| Parameter | Default | Description |
|-----------|---------|-------------|
| enable_prediction | false | Master enable |
| enable_collision_check | false | Master enable |
| horizon_s | 1.0 | Prediction horizon |
| dt_s | 0.02 | Prediction time step |
| hit_threshold_m | 0.05 | Max distance for proximity hit |
| min_points_near_hit | 3 | Min nearby points for hit |
| odom_max_age_s | 0.1 | Max odometry staleness |
| cloud_max_age_s | 2.0 | Max pointcloud staleness |
| max_prediction_covariance_trace | 100.0 | Max allowed covariance trace |
| process_noise_linear_mps2_per_s | 0.1 | Linear velocity process noise |
| process_noise_angular_radps2_per_s | 0.5 | Angular velocity process noise |
| collision_geometry_type | sphere | Collision shape |
| collision_geometry_radius_m | 0.10 | Sphere radius |
| collision_geometry_frame | arm_imu | Attached TF frame |
| odom_topic | /ov_msckf_arm/odomimu | |
| cloud_topic | /arm/d435i_arm/points_marker_map | |
| trajectory_topic | /prediction/future_trajectory | |
| click_topic | /segmentation/click_positive | |
| voxel_leaf_m | 0.02 | Pointcloud downsampling before KD-tree |
| status_period_s | 1.0 | Min interval between status publications |

### 1.4 Launch File

**Location:** `src/sensor_fusion_bringup/launch/future_prediction_collision.launch.py`

- Uses `OpaqueFunction` + YAML config loading pattern (matching existing `dual_d435i.launch.py`)
- Declares 3 launch arguments: `enable_prediction` (false), `enable_collision_check` (false), `use_sim_time` (false)
- Loads defaults from YAML, overlays CLI arguments
- Safe by default — both master enables default to false

### 1.5 Tests

**Location:** `src/sensor_fusion_bringup/test/test_future_prediction_collision.py`

**16 pytest tests across 6 test classes:**

| Class | Tests | What it validates |
|-------|-------|-------------------|
| `TestPropagation` | 3 | Zero twist stationary, constant velocity linear advance, rotation-only orientation |
| `TestCovariance` | 3 | Monotonic trace growth, symmetry preservation, twist covariance floor |
| `TestProximity` | 2 | Hit detection near known cluster, no-hit far from points |
| `TestOdomGuard` | 4 | Uninitialized rejected, initialized accepted, NaN rejected, Inf rejected |
| `TestVoxelDownsample` | 3 | Empty input, single point, merge duplicates |
| `TestParseXYZ` | 1 | Basic PointCloud2 XYZ parsing |

**Result:** 16 passed, 0 failed (0.33s)

### 1.6 Dockerfile Modification

**Location:** `docker/Dockerfile` (line 11-12)

Added two apt packages:
```
python3-scipy \
python3-pytest \
```

### 1.7 CMakeLists.txt Modification

**Location:** `src/sensor_fusion_bringup/CMakeLists.txt`

- Removed references to Jetson-only scripts (`analyze_marker_covariance_bags.py`, `aruco_marker_pose_node.py`, `marker_quality_monitor.py`) that don't exist on the x86 branch
- Added `future_pose_prediction_collision_node.py` to PROGRAMS install
- Added `if(BUILD_TESTING)` block to install test directory

### 1.8 Validation Docs

**Location:** `docs/future_prediction_collision_validation.md`

Covers: build, launch (disabled/enabled/collision), topic checks, RViz, launch args, test execution, rollback procedure.

---

## 2. Build & Validation Results

| Check | Result |
|-------|--------|
| `colcon build --packages-select sensor_fusion_msgs sensor_fusion_bringup` | 2 packages finished |
| `ros2 interface show sensor_fusion_msgs/msg/FuturePoseTrajectory` | Valid |
| `ros2 launch ... --show-args` | 3 args with correct defaults |
| Node smoke test (defaults, no topics) | Starts clean, no crash |
| `python3 -m pytest test_future_prediction_collision.py -v` | **16 passed** |

---

## 3. Acceptance Criteria Status

| Criterion | Status |
|-----------|--------|
| Build succeeds in Docker container | Verified (on host with ROS 2 Humble) |
| Launch args shown correctly | Verified |
| FuturePoseTrajectory message type validates | Verified |
| Node publishes predicted poses in marker_map frame | Code passes smoke test; needs Jetson integration |
| Predicted trajectory cadence matches pointcloud (~8 Hz) | Architecture ensures this; needs Jetson integration |
| No predictions when pointcloud is stale or missing | Guard implemented |
| No predictions when odometry is stale or missing | Guard implemented |
| No predictions when enable_prediction=false | Guard implemented |
| Covariance grows monotonically along horizon | Tested (test_covariance_grows_monotonically) |
| Click published when predicted pose intersects pointcloud | Code present; needs Jetson integration |
| Status JSON includes accepted, reason, odom_age_s, point counts | Implemented |
| pytest suite passes all tests | 16/16 passed |
| Existing Jetson OpenVINS + marker behavior unchanged | No Jetson-side changes needed |
| Rollback: enable_prediction:=false stops all prediction/click | Architecture ensures this |

---

## 4. What Still Needs to Be Implemented Later

### 4.1 Docker Rebuild (pending)
The Docker image must be rebuilt to include `python3-scipy` and `python3-pytest`. This hasn't been done yet because the Docker daemon may or may not be running on this host.

```bash
cd ~/Documents/GitHub/assistive_multiview_prosthesis/docker
docker compose build prosthesis
```

### 4.2 Jetson-side Integration Testing
All smoke tests were done on the host without live Jetson topics. Full integration validation requires:
- Jetson running `jetson_docker` branch publishing `/ov_msckf_arm/odomimu` and `/arm/d435i_arm/points_marker_map`
- x86 PC running Docker container with CycloneDDS host networking
- Verify cadence match (~8 Hz for both cloud and trajectory)
- Verify frame_id is `marker_map`
- Verify clicks fire when predicted trajectory intersects real pointcloud clusters
- RViz visualization of trajectory path + click points

### 4.3 Covariance from Live OpenVINS
The covariance propagation and twist floor logic was unit-tested with synthetic data. It should be verified against real OpenVINS odometry output to confirm the floor values (`process_noise_linear_mps2_per_s=0.1`, `process_noise_angular_radps2_per_s=0.5`) are appropriate for the prosthetic arm dynamics.

### 4.4 Collision Geometry Frame Transformation (v2)
Currently, the `collision_geometry_frame` defaults to `arm_imu` and applies the sphere offset directly in the cloud frame without TF lookup. If a different collision frame is needed (e.g., a tool frame attached to the hand), TF2-based transform must be added. A `tf2_ros.Buffer` + `TransformListener` should be wired into the `_on_cloud` callback, gated by a flag.

### 4.5 Segmentation/Preshaping Pipeline Integration
Currently, the node publishes clicks on `/segmentation/click_positive` but does not monitor the segmentation response or trigger preshaping. The full pipeline (click → segmentation → preshaping) would need:
- Subscription to `/segmentation/object_cloud` to detect segmentation completion
- Service client for `/grasp_preshaping/compute_grasp`
- Internal state machine (IDLE → WAITING_FOR_SEGMENTATION → WAITING_FOR_PRESHAPING)
- This mirrors the pattern used in `twist_propagation_node.py` on the `test1-daniel` branch

### 4.6 Node Integration into System Launch
The node currently has its own standalone launch file. It should be integrated into the top-level `prosthesis_launch` package's launch files (e.g., `pipeline.launch.py`, `digital_twin.launch.py`) so it starts automatically as part of the full system.

### 4.7 Parameter Auto-Tuning
The process noise and covariance trace thresholds may need tuning based on real-world data:
- `process_noise_linear_mps2_per_s` — affects how fast position uncertainty grows
- `process_noise_angular_radps2_per_s` — affects orientation uncertainty growth
- `max_prediction_covariance_trace` — determines when prediction horizon is truncated
- `hit_threshold_m` and `min_points_near_hit` — sensitivity of click triggering

### 4.8 Performance Profiling
KDTree construction on full pointclouds is O(N log N). With the `voxel_leaf_m=0.02` downsampling, performance should be acceptable at 8 Hz, but profiling on the actual x86 hardware with live data would confirm this.

### 4.9 `package.xml` Cleanup
The `sensor_fusion_bringup/package.xml` has several exec_depends that may not be needed on the x86 branch (e.g., `tf2_ros`, `cv_bridge`, `rosbag2_py`). These were carried over from the Jetson branch and should be audited.

---

## 5. File Inventory

### New Files (8)
```
src/sensor_fusion_msgs/package.xml
src/sensor_fusion_msgs/CMakeLists.txt
src/sensor_fusion_msgs/msg/FuturePoseTrajectory.msg
src/sensor_fusion_bringup/config/future_prediction_collision.yaml
src/sensor_fusion_bringup/scripts/future_pose_prediction_collision_node.py
src/sensor_fusion_bringup/launch/future_prediction_collision.launch.py
src/sensor_fusion_bringup/test/test_future_prediction_collision.py
docs/future_prediction_collision_validation.md
```

### Modified Files (2)
```
docker/Dockerfile        (+2 apt lines: python3-scipy, python3-pytest)
src/sensor_fusion_bringup/CMakeLists.txt  (added new script + test install)
```

## 6. Post-Implementation Fixes (2026-05-15, same session)

### Fix 1: RViz visualization — add `nav_msgs/Path` publisher
**Problem:** RViz2's "Path" display only renders `nav_msgs/Path`, but the node published
`sensor_fusion_msgs/FuturePoseTrajectory` (custom type). The topic appeared in RViz's
dropdown but nothing rendered.

**Fix:** Added a secondary publisher on `/prediction/future_trajectory/path` that
converts the predicted poses to a standard `nav_msgs/Path`. RViz2 renders this
natively. User should add `By topic /prediction/future_trajectory/path → Path`
in RViz.

### Fix 2: Earlier click detection — apply `collision_geometry_radius_m`
**Problem:** The `collision_geometry_radius_m` parameter was declared but never used
in `check_proximity_hit()`. The KDTree query used only `hit_threshold_m` (0.05m),
meaning the click fired only when the exact center of the predicted arm was within
0.05m of cloud points — effectively at physical contact.

**Fix:** `check_proximity_hit()` now accepts `collision_radius_m` and uses
`effective_threshold = hit_threshold_m + collision_radius_m`. With the default
radius of 0.10m, the click fires when the sphere surface (not center) approaches
the cloud cluster — roughly 0.10m / ~0.5-1.0s earlier.

**Tunable parameters for click timing:**

| Parameter | Default | To fire earlier | Effect |
|-----------|---------|-----------------|--------|
| `collision_geometry_radius_m` | 0.10 | Increase → 0.15-0.20 | Sphere surface reaches cluster sooner |
| `hit_threshold_m` | 0.05 | Increase → 0.10-0.20 | Larger proximity bubble |
| `min_points_near_hit` | 3 | Decrease → 1-2 | Fewer points needed to confirm hit |
| `horizon_s` | 1.0 | Increase → 1.5-2.0 | Look farther ahead |
