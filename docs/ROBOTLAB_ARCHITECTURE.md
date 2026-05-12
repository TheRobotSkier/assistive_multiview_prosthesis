# Robotlab Jetson — P8 Grasping Project

## System Characteristics

| Characteristic | Value |
|---|---|
| **Host PC** | x86 laptop (CachyOS/Arch), podman for containers |
| **Peripheral** | NVIDIA Jetson Orin Nano, ARM64, 500GB NVMe, 8GB RAM |
| **OS (Jetson)** | NVIDIA custom Ubuntu 22.04, kernel 5.15.148-tegra |
| **ROS Distro** | Jazzy (both machines) |
| **Network — WiFi** | Host AP (Linksys AE3000): SSID `robotlab-wifi`, password `labrobot123` |
| **Network — Ethernet** | Direct cable: host `192.168.100.1/24` ↔ Jetson `192.168.100.2/24` |
| **ROS DDS** | Default FastRTPS over Ethernet (multicast works), NO CycloneDDS/config needed |
| **SSH** | Host → Jetson via WiFi (`ssh robotlab` → 10.42.0.2) or Ethernet (`192.168.100.2`) |
| **VNC** | Jetson display at `10.42.0.2:5900` (x11vnc, no password) |

## Repositories

### 1. jetson-docker (Jetson side)
- **Path:** `~/jetson_ws/jetson-docker` on robotlab
- **Branch:** `asger-jetson`
- **Remote:** `github.com/TheRobotSkier/assistive_multiview_prosthesis`
- **Docker image:** `docker-deployment-miahand_ros2` (built from `docker_ws/docker-deployment/Dockerfile`, based on `ros:jazzy-ros-base`)

### 2. full_test_implementation (Host side)
- **Path:** `~/Drive/AAU/P8/grasping/multiview_prosthesis` on host
- **Branch:** `full_test_implementation`
- **Remote:** `github.com/TheRobotSkier/assistive_multiview_prosthesis`

## Hardware Connected to Jetson

| Device | Model | Connection | Details |
|---|---|---|---|
| **Head camera** | Intel RealSense D435 | USB 2-1.1 | Serial: `827112072033`, no internal IMU |
| **Arm camera** | Intel RealSense D435 | USB 2-1.2 | Serial: `829212072207`, no internal IMU |
| **Head IMU** | GY-91 (MPU-9250+BMP280) | I2C bus 7, addr 0x68 | Frame: `cam0_imu_link`, WHOAMI=0x70 |
| **Arm IMU** | GY-91 (MPU-9250+BMP280) | I2C bus 1, addr 0x68 | Frame: `cam1_imu_link`, WHOAMI=0x70 |

## What's Working (Built So Far)

### Bringup (`make up` on robotlab)
Launches everything in one Docker container:
- ✅ Both D435 cameras: depth 424x240@15fps, color 640x480@15fps, pointclouds enabled
- ✅ Both GY-91 IMUs: 15Hz (matching camera FPS)
- ✅ EKF odometry fusion (`/odometry/filtered`)
- ✅ NEON pointcloud fix for Jetson
- ✅ Initial reset on camera startup
- ✅ USB bandwidth stable (lowered depth resolution fixed contention)

### ROS Topics Published (verified from both robotlab and host)
```
/head/d435_head/depth/color/points    # Head pointcloud
/arm/d435_arm/depth/color/points      # Arm pointcloud
/head/d435_head/color/image_raw       # Head RGB
/arm/d435_arm/color/image_raw         # Arm RGB
/cam0/data_raw                        # Head IMU (15Hz)
/cam1/data_raw                        # Arm IMU (15Hz)
/odometry/filtered                    # EKF fused odometry
/tf, /tf_static                       # Transforms
```

### Cross-Network ROS
- ✅ Host podman containers see ALL robotlab topics
- ✅ Default FastRTPS over Ethernet — NO special RMW config needed
- ❌ Discovery does NOT work over WiFi-only link (multicast broken on WiFi adapter)

### Host RViz
- ✅ `make rviz` launches podman container with default RMW
- ✅ RViz config at `rviz/robotlab_cameras.rviz`

### Config Consolidation (E1)
- ✅ Camera profiles, pointcloud, align_depth → `d435_cameras.yaml`
- ✅ IMU Hz documented in `imu_cam0.yaml` / `imu_cam1.yaml`
- ✅ Bringup launch reads from configs

### World Marker (E2)
- ✅ Marker ID changed 0→1 in `head_aruco_map.yaml` and `arm_aruco_map.yaml`
- ✅ Frame IDs updated: `marker_0`→`marker_1`
- ✅ All hardcoded references cleaned

### OpenVINS Adaptation (E3)
- ✅ New configs created: `openvins/head_d435_827112072033/` and `arm_d435_829212072207/`
- ✅ Calibration values from P3.1 archaeology (IMU noise, offsets) embedded
- ✅ New launch files: `head_d435_openvins_phase2.launch.py`, `arm_d435_openvins_phase2.launch.py`
- ✅ Marker configs updated to use D435 frame IDs and external IMU frames

## What Still Needs Doing

### OpenVINS Testing (E3.5 — P1)
- Actually launch OpenVINS and verify it publishes `/ov_msckf/odomimu`
- May need stationary initialization (OpenVINS typical requirement)
- Verify marker pose correction works with ID 1

### TF Tree Verification (E4.3 — P1)
- Verify full TF chain: `marker_map` ← `cam0_imu_link` / `cam1_imu_link` ← camera optical frames
- Static TF publishers for camera→IMU extrinsics (E4.1 pending)
- Marker pose → world TF chain (E4.2 pending)

### Git Push
- Robotlab has no internet access — commits are local
- Need to push `asger-jetson` branch from a machine with internet

### Calibration
- Camera intrinsics: using factory defaults (not calibrated)
- IMU→camera extrinsics: using placeholder from P3.1 (tx=0.020, ty=-0.005, tz=-0.010)
- No proper Kalibr calibration for D435+GY-91 setup

## Key Directories (jetson-docker repo)

```
~/jetson_ws/jetson-docker/
├── Makefile                          # make build, make up, make kill, make shell
├── docker_ws/
│   ├── docker-deployment/
│   │   ├── Dockerfile                # Image: ros:jazzy-ros-base + realsense + ros2_control
│   │   ├── docker-compose.yml        # miahand_ros2 service (host net, privileged, /dev mounted)
│   │   └── cyclonedds_robotlab.xml   # (deprecated — not used, default FastRTPS works)
│   └── multi_cam_localization/
│       ├── imu_driver/               # C++ I2C IMU driver (reads GY-91 from /dev/i2c-X)
│       │   ├── config/imu_cam0.yaml  # bus=7, addr=104, 15Hz, frame=cam0_imu_link
│       │   └── config/imu_cam1.yaml  # bus=1, addr=104, 15Hz, frame=cam1_imu_link
│       └── sensor_fusion_bringup/
│           ├── launch/
│           │   ├── robotlab_bringup.launch.py           # Main bringup (cameras+IMU+EKF)
│           │   ├── two_imus_ekf.launch.py               # EKF node only
│           │   ├── head_d435_openvins_phase2.launch.py  # OpenVINS for head
│           │   ├── arm_d435_openvins_phase2.launch.py   # OpenVINS for arm
│           │   └── dual_d435_native.launch.py           # (legacy — ExecuteProcess approach)
│           ├── config/
│           │   ├── d435_cameras.yaml        # Camera profiles, pointcloud, serials
│           │   ├── ekf_dual_imu.yaml        # Dual IMU EKF config
│           │   ├── markers/
│           │   │   ├── head_aruco_map.yaml  # Marker ID 1, frame cam0_imu_link
│           │   │   └── arm_aruco_map.yaml   # Marker ID 1, frame cam1_imu_link
│           │   └── openvins/
│           │       ├── head_d435_827112072033/  # estimator_config, kalibr chains
│           │       └── arm_d435_829212072207/
│           └── scripts/
│               ├── aruco_marker_pose_node.py
│               └── marker_quality_monitor.py
```

## How to Operate

### On robotlab (SSH in):
```bash
cd ~/jetson_ws/jetson-docker
make up       # build + launch cameras, IMUs, EKF
make kill     # stop everything
make shell    # interactive bash in container
```

### On host:
```bash
cd ~/Drive/AAU/P8/grasping/multiview_prosthesis
make rviz     # launch RViz viewing robotlab topics
```

### Encountered Pitfalls
1. **DDS discovery over WiFi:** Does NOT work on this WiFi adapter. Always use Ethernet.
2. **USB bandwidth:** Two D435 at 640x480 depth exceeds Jetson USB 3.0 bandwidth. Use 424x240 depth.
3. **Video device re-enumeration:** Jetson kernel renumbers /dev/video* on each container restart. Serial-based camera discovery handles this.
4. **Robotlab internet:** Jetson has no internet when on robotlab-wifi hotspot. `git push` fails. Push from host.
5. **--packages-select needed:** Full colcon build exceeds Jetson storage. Only build imu_driver, sensor_fusion_bringup, sensor_fusion_msgs.
6. **Disk space:** Expanded NVMe partition from 28GB→457GB. 430GB free.
7. **I2C permissions:** Container user must be in i2c group (gid 116). Added via group_add in compose.
8. **Docker TTY:** `make up` from non-TTY needs `docker compose run -T` (not `-it`).
