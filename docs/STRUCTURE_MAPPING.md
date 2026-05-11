# Structure Mapping: jetson-docker → full_test_implementation (asger-jetson)

> Created: 2026-05-11 for bead `jetson-docker-dtk` (P4.2)

## Target Workspace Layout

The asger-jetson branch should follow the same `src/` structure as `full_test_implementation`
on the x86 host, but only packages that actually need to run on the Jetson Orin Nano are
included. Everything else stays on the x86 host and communicates over ROS 2 DDS (CycloneDDS).

```
src/
├── camera/                  # ← jetson_docker realsense launch + D435 config + ChArUco TF
├── imu_driver/              # ← jetson_docker multi_cam_localization/imu_driver/
├── sensor_fusion_bringup/   # ← jetson_docker multi_cam_localization/sensor_fusion_bringup/
├── sensor_fusion_msgs/      # ← jetson_docker multi_cam_localization/sensor_fusion_msgs/
├── mia_hand_description/    # ← jetson_docker (already present)
├── mia_hand_driver/         # ← jetson_docker (already present)
├── mia_hand_msgs/           # ← jetson_docker (already present)
├── mia_hand_ros2_control/   # ← jetson_docker (already present)
├── pipeline_manager/        # ← full_test_implementation (Python — can run on Jetson)
├── twist_propagation/       # ← full_test_implementation (Python — can run on Jetson)
├── force_controller/        # ← full_test_implementation (Python — can run on Jetson)
├── wrist_driver/            # ← full_test_implementation (Python — can run on Jetson)
└── prosthesis_launch/       # ← full_test_implementation (Jetson-specific launch files only)
```

## 1. Full Test Implementation Package Classification

### Packages that CAN run on Jetson

| Package | Reason | Action |
|---------|--------|--------|
| `camera` | Minimal D435 launch; will be **replaced** by Jetson version | Replace with jetson_docker version |
| `pipeline_manager` | Pure Python ROS 2 state machine; no heavy deps | Copy from full_test_implementation |
| `twist_propagation` | Python + scipy/numpy; hand twist estimation | Copy from full_test_implementation |
| `force_controller` | Python force-based grasp regulation | Copy from full_test_implementation |
| `wrist_driver` | Python Dynamixel wrist driver (needs USB/serial on Jetson) | Copy from full_test_implementation |
| `mia_hand_description` | URDF/meshes/launch (already on Jetson) | Already present |
| `mia_hand_driver` | Hand motor driver (already on Jetson) | Already present |
| `mia_hand_msgs` | Message definitions (already on Jetson) | Already present |
| `mia_hand_ros2_control` | ROS2 control config (already on Jetson) | Already present |
| `prosthesis_launch` | Top-level launch files (Jetson-specific subset) | Port Jetson-specific launcher |
| `emg_bridge` | MindRove EMG; theoretically possible but requires BT | **Defer** — keep x86 for now |

### Packages that MUST stay on x86 host

| Package | Reason |
|---------|--------|
| `segmentation` | MinkowskiEngine + InterObject3D — requires CUDA GPU, heavy compute |
| `grasp_preshaping` | Rust/C++ FFI build chain + heavy compute |
| `haptic_band` | Bluetooth hardware only on x86 host (Vibro8 armband) |
| `mia_hand_mujoco` | MuJoCo simulation — visualization only, needs display |
| `mia_hand_moveit_config` | MoveIt planning — x86 only (display + heavy planning) |

## 2. Jetson Docker Component → Target Mapping

### 2.1 Camera (RealSense D435 Launch)

**Source:** `jetson_docker/docker_ws/realsense-ros/` (realsense2_camera wrapper)
**Target:** `src/camera/`

The full_test_implementation `camera` package contains:
- `launch/single_d435.launch.py` — minimal single-camera launch
- `launch/two_d435.launch.py` — dual-camera launch with serial_no workaround
- `launch/multiview_full_launch.py` — wraps two_d435 + TF

The jetson_docker equivalent lives in `sensor_fusion_bringup/launch/`:
- `dual_d435.launch.py` — **dual D435 (no IMU)** with Jetson NEON pointcloud fix
- `dual_d435i.launch.py` — dual D435i (with IMU for OpenVINS)

**Mapping:**
```
jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dual_d435.launch.py
    → src/camera/launch/dual_d435.launch.py

jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/config/d435_cameras.yaml
    → src/camera/config/d435_cameras.yaml
```

The Jetson version adds:
- NEON pointcloud fix (`pointcloud__neon_.enable` delayed set)
- Serial numbers: head=823313022234, arm=830213023028
- I2C bus mapping for GY-91 IMUs (bus 7=head/cam0, bus 1=arm/cam1)
- Namespace conventions: `/head/d435_head`, `/arm/d435_arm`

### 2.2 IMU Driver

**Source:** `jetson_docker/docker_ws/multi_cam_localization/imu_driver/`
**Target:** `src/imu_driver/`

Complete package, maps 1:1:
```
imu_driver/
├── CMakeLists.txt              → src/imu_driver/CMakeLists.txt
├── package.xml                 → src/imu_driver/package.xml
├── config/
│   ├── imu.yaml                → src/imu_driver/config/imu.yaml        (template)
│   ├── imu_cam0.yaml           → src/imu_driver/config/imu_cam0.yaml   (bus 7, head)
│   └── imu_cam1.yaml           → src/imu_driver/config/imu_cam1.yaml   (bus 1, arm)
├── launch/
│   └── imu_driver.launch.py    → src/imu_driver/launch/imu_driver.launch.py
└── scripts/
    └── read_gy91_simple.py     → src/imu_driver/scripts/read_gy91_simple.py
```

Key calibration values (from P3.1) to embed/apply:
- Axis remap (MPU9250→ROS): ros_x=-mpu_y, ros_y=mpu_x, ros_z=mpu_z
- Accel bias: [0.0607, -0.0800, 0.1369] m/s²
- Gyro noise std: 0.00096 rps
- IMU-to-camera offset: tx=0.020m, ty=-0.005m, tz=-0.010m

### 2.3 Sensor Fusion Bringup

**Source:** `jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/`
**Target:** `src/sensor_fusion_bringup/`

Launch files needed on Jetson:
```
sensor_fusion_bringup/
├── package.xml
├── CMakeLists.txt
├── config/
│   ├── d435_cameras.yaml                  → camera config (head+arm serials)
│   ├── d435i_cameras.yaml                 → D435i config (for OpenVINS mode)
│   ├── ekf_imu_only.yaml                  → EKF config for IMU-only fusion
│   ├── markers/                           → ArUco marker map configs
│   │   ├── head_aruco_map.yaml
│   │   ├── arm_aruco_map.yaml
│   │   ├── head_aruco_map_large_160mm.yaml
│   │   └── head_aruco_map_replay_1533mm.yaml
│   ├── aruco_markers/                     → printable ArUco marker images
│   └── openvins/                          → OpenVINS calibrations (reference)
├── launch/
│   ├── dual_d435.launch.py                → dual D435 (Jetson NEON fix)
│   ├── dual_d435i.launch.py               → dual D435i + OpenVINS
│   ├── two_imus.launch.py                 → 2x GY-91 IMU nodes
│   ├── two_imus_ekf.launch.py             → 2x IMU + robot_localization EKF
│   ├── head_marker_pose_phase2.launch.py  → ChArUco pose estimation (head)
│   ├── arm_marker_pose_phase2.launch.py   → ChArUco pose estimation (arm)
│   ├── head_d435i_openvins.launch.py      → OpenVINS head
│   ├── head_d435i_openvins_phase2.launch.py
│   ├── arm_d435i_openvins_phase2.launch.py
│   └── single_d435i.launch.py             → single D435i fallback
└── scripts/
    ├── aruco_marker_pose_node.py           → ArUco pose detection node
    ├── marker_quality_monitor.py           → marker quality monitoring
    ├── generate_aruco_a4.py               → A4 ArUco generator
    └── analyze_marker_covariance_bags.py  → analysis tool
```

### 2.4 Sensor Fusion Messages

**Source:** `jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_msgs/`
**Target:** `src/sensor_fusion_msgs/`

Maps 1:1. Custom ROS 2 message definitions for marker and sensor-fusion data.

### 2.5 Mia Hand Packages (Already Present)

These packages exist in both repos with the same content:

| Package | jetson_docker path | full_test_implementation path |
|---------|-------------------|------------------------------|
| `mia_hand_description` | `docker_ws/mia_hand_description/` | `src/mia_hand_description/` |
| `mia_hand_driver` | `docker_ws/mia_hand_driver/` | `src/mia_hand_driver/` |
| `mia_hand_msgs` | `docker_ws/mia_hand_msgs/` | `src/mia_hand_msgs/` |
| `mia_hand_ros2_control` | `docker_ws/mia_hand_ros2_control/` | `src/mia_hand_ros2_control/` |

**Action:** Keep the jetson_docker versions (they are identical and already tested on Jetson).

### 2.6 Calibration Data

**Source:** `jetson_docker/docker_ws/calibration/`
**Target:** Reference only — extract key values into config files.

```
calibration/
├── head/d435i_336222071386/
│   ├── camera_intrinsics/          → extract into camera configs
│   ├── imu_noise/                  → extract into IMU/EKF configs
│   └── camera_imu_extrinsics_inflated10x/  → OpenVINS calibrations
└── arm/d435i_310622071850/
    ├── camera_intrinsics/
    ├── imu_noise/
    └── camera_imu_extrinsics_inflated10x/
```

## 3. What Should Be Stripped / Removed from Jetson

### 3.1 Remove from jetson_docker (NOT needed on Jetson)

| Component | Reason |
|-----------|--------|
| `mia_hand_mujoco/` | MuJoCo simulation — x86 only |
| `mia_hand_moveit_config/` | MoveIt planning — x86 only |
| `mia_hand_ros2_pkgs/` | Meta-package only — not needed |
| `src/open_vins/` | OpenVINS — heavy C++ build, optional. Keep configs, remove source. |
| `build_overlay/` | Build artifacts — regenerate |
| `install_overlay/` | Install artifacts — regenerate |
| `log_overlay/` | Log artifacts — regenerate |
| `calibration/` (full directory) | ~100MB of calibration raw data. Extract key values → config, archive rest. |

### 3.2 Remove from full_test_implementation (when used as reference)

These packages stay on x86 and should NOT be included on Jetson:
- `segmentation/`
- `grasp_preshaping/`
- `haptic_band/`
- `emg_bridge/` (deferred — possibly move later)

### 3.3 Docker Adjustments

The Docker compose currently mounts the entire workspace. After refactoring:
- Docker image should NOT install MuJoCo (save ~500MB)
- Remove OpenVINS build dependencies (libceres-dev, libboost-all-dev, libeigen3-dev) if OpenVINS is dropped
- Keep only: realsense2_camera, ros-jazzy-ros-base, libserial-dev, python3-pip

## 4. Launch Hierarchy (Post-Refactor)

### Jetson-side launch (all-in-one)
```bash
# On Jetson:
ros2 launch prosthesis_launch jetson_bringup.launch.py
```
This would include:
1. dual_d435.launch.py (cameras + NEON fix)
2. two_imus.launch.py (GY-91 IMUs)
3. imu_ekf.launch.py (robot_localization EKF)
4. marker_pose.launch.py (ChArUco detection)
5. mia_hand_driver (hand motor control)
6. wrist_driver (wrist motor)
7. twist_propagation (hand twist estimation)
8. force_controller (grasp force regulation)
9. pipeline_manager (state machine)

### x86-side launch
```bash
# On x86 host:
ros2 launch prosthesis_launch digital_twin.launch.py  # unchanged
```
This keeps:
1. segmentation_bridge (talks to Jetson via DDS for pointclouds)
2. grasp_preshaping
3. haptic_bridge
4. emg_bridge
5. RViz visualization
6. MuJoCo/URDF display

## 5. Disk Space Considerations

Jetson Orin Nano: 28GB total, ~5GB free after current Docker setup.

Estimated savings from stripping:
| Item | Savings |
|------|---------|
| Remove MuJoCo from Docker image | ~500MB |
| Remove OpenVINS source/build | ~200MB |
| Remove calibration raw data | ~100MB |
| Remove build_overlay/install_overlay | ~2-5GB (rebuild only needed packages) |
| **Total estimated savings** | **~3-6GB** |

## 6. Summary Table

| full_test_implementation package | On Jetson? | Jetson source |
|----------------------------------|------------|---------------|
| `camera` | ✅ Yes | jetson_docker (dual_d435 + D435 config) |
| `imu_driver` | ✅ Yes (new) | jetson_docker/multi_cam_localization/imu_driver |
| `sensor_fusion_bringup` | ✅ Yes (new) | jetson_docker/multi_cam_localization/sensor_fusion_bringup |
| `sensor_fusion_msgs` | ✅ Yes (new) | jetson_docker/multi_cam_localization/sensor_fusion_msgs |
| `mia_hand_description` | ✅ Yes | jetson_docker (same as x86) |
| `mia_hand_driver` | ✅ Yes | jetson_docker (same as x86) |
| `mia_hand_msgs` | ✅ Yes | jetson_docker (same as x86) |
| `mia_hand_ros2_control` | ✅ Yes | jetson_docker (same as x86) |
| `pipeline_manager` | ✅ Yes | full_test_implementation (copy) |
| `twist_propagation` | ✅ Yes | full_test_implementation (copy) |
| `force_controller` | ✅ Yes | full_test_implementation (copy) |
| `wrist_driver` | ✅ Yes | full_test_implementation (copy) |
| `prosthesis_launch` | ✅ Yes (subset) | new Jetson-specific launch file |
| `emg_bridge` | ⬜ Deferred | full_test_implementation (later) |
| `segmentation` | ❌ No | stays on x86 |
| `grasp_preshaping` | ❌ No | stays on x86 |
| `haptic_band` | ❌ No | stays on x86 |
| `mia_hand_mujoco` | ❌ No | stays on x86 |
| `mia_hand_moveit_config` | ❌ No | stays on x86 |
