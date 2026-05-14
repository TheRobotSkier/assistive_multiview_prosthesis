# Goal: Port full_test_implementation into x86_openvins

## Purpose

Bring the `x86_openvins` branch up to date with everything in
`full_test_implementation` so the complete prosthesis pipeline — cameras, IMU,
OpenVINS, grasp detection, and hand control — can run on an x86 PC in Docker,
with the Jetson Nano acting only as a remote camera/IMU streaming server.

## Hardware

- **Cameras:** Intel RealSense **D435** (non-i, no built-in IMU). Two units
  connected to the Jetson over USB.
- **IMUs:** External ESP32-based dual GY-91 IMU units, connected to the Jetson
  over serial USB, bridged to ROS2 via the serial bridge node.
- **Jetson Nano** (`robotlab@robotlab.local`): Streams all sensor data (camera
  images, pointclouds, IMU) over direct Ethernet to the host x86 PC via the
  ROS2 DDS network (CycloneDDS, `config/cyclonedds_peer.xml`).
- **Host x86 PC**: Runs OpenVINS, grasp detection, and the full processing
  pipeline in Docker (podman). Receives all sensor data from the Jetson.

## Architecture

```
Jetson Nano (robotlab@robotlab.local)
  ├── cameras_test container (D435 cameras → ROS2 topics)
  └── openvins container (OpenVINS VIO)
        │  DDS over Ethernet
        ▼
x86 Host PC (this machine)
  ├── docker (openvins) ─────── receives /tf, /odom, /pointcloud
  ├── docker (prosthesis) ────── grasp detection, twist propagation
  └── docker (segmentation) ─── pointcloud segmentation
```

Wait — the goal here is actually to MOVE OpenVINS to the x86 side. Currently
`x86_openvins` has OpenVINS in a Docker container on x86 (receiving raw camera
images from Jetson). `full_test_implementation` has the Jetson running OpenVINS.
The port should make the x86 side run OpenVINS receiving raw images over Ethernet.

## What This Branch Already Has (x86_openvins)

- `src/open_vins/` — OpenVINS as git submodule
- `src/realsense-ros/` — RealSense ROS2 driver submodule (for x86 calibration)
- `docker/Dockerfile.openvins` — OpenVINS Docker image for x86
- Camera calibration data in `config/calibration/head/d435i_336222071386/`
- ESP32 dual GY-91 IMU firmware in `esp32_imu/`
- Basic dual-camera launch in `src/sensor_fusion_bringup/`

## What Is Missing (needs porting from full_test_implementation)

Compare the two branches with:
```bash
git diff x86_openvins full_test_implementation --stat
```

Key things to port:

### 1. Infrastructure & Config

- `config/cyclonedds_peer.xml` — DDS peer config for Jetson ↔ host communication
- `jetson/` directory — Jetson Makefile and config for managing Jetson containers
- `jetson/config/cyclonedds_robotlab.xml` — Jetson-side DDS config

### 2. Makefile Targets

The `full_test_implementation` Makefile has extensive Jetson management targets
(`jetson-sync`, `jetson-cameras`, `jetson-openvins`, `jetson-imu-test-*`, etc.)
and RViz launch targets. Port all of these, adapting the OpenVINS target to
launch the **x86-side** OpenVINS container (not the Jetson one).

Also port `scripts/jetson_run.sh` — the queue wrapper script used for all Jetson
access.

### 3. RViz Configurations

- `rviz/phase2_dual_openvins.rviz`
- `rviz/imu_test_single.rviz` / `rviz/imu_test_dual.rviz`
- `rviz/grasp_test.rviz`

### 4. Documentation

- `docs/ROBOTLAB_ARCHITECTURE.md`
- `docs/ROBOTLAB_SETUP.md`
- `docs/ethernet_ros2_setup.md`
- `docs/jetson_setup.md`
- `docs/tf_tree.md`

### 5. ROS2 Launch Files

- `src/sensor_fusion_bringup/launch/dual_openvins_phase2.launch.py`
- `src/sensor_fusion_bringup/launch/imu_test.launch.py`
- IMU dead reckoning node: `src/sensor_fusion_bringup/` (added in full_test_impl)

### 6. OpenVINS Configuration for x86

Adapt the OpenVINS Docker setup so it:
- Receives `/cam0/color/image_raw`, `/cam1/color/image_raw`, IMU topics from the
  Jetson over DDS Ethernet
- Uses `config/cyclonedds_peer.xml` for DDS discovery
- Does NOT require cameras to be physically connected to the x86 machine

This requires changes to `docker/docker-compose.yml` to pass the CycloneDDS
config into the openvins container, and to the OpenVINS launch file to subscribe
to remote topics.

### 7. AGENTS.md

Add `AGENTS.md` to this worktree. ✅ (Already done — you are reading it.)

## Definition of Done

- [ ] `make jetson-sync && make jetson-cameras` works via the queue (Jetson streams cameras)
- [ ] x86-side `make openvins` (or equivalent) starts OpenVINS in Docker, subscribing to Jetson topics
- [ ] OpenVINS receives images and IMU data from Jetson over Ethernet
- [ ] `/tf` tree is correct (world → odom → cam0_base_link)
- [ ] RViz shows point clouds and OpenVINS pose from Jetson data
- [ ] All `jetson-*` Makefile targets work from this worktree
- [ ] `scripts/jetson_run.sh` exists and is used for all Jetson queue calls

## Where to Start

1. Read the diff:
   ```bash
   git diff x86_openvins full_test_implementation -- Makefile
   git diff x86_openvins full_test_implementation -- jetson/
   git diff x86_openvins full_test_implementation -- config/
   git diff x86_openvins full_test_implementation -- docs/
   ```

2. Port infrastructure first (cyclonedds config, Makefile targets, scripts/)

3. Then port launch files and RViz configs

4. Finally, adapt the OpenVINS Docker setup for remote image subscription

## Jetson Access

All Jetson interaction goes through the agent-task-queue MCP tool.
See `AGENTS.md` for the full queue protocol.

Quick reference for the most common task (this worktree):
```python
run_task(
    command="scripts/jetson_run.sh make jetson-cameras",
    working_directory="/home/asger/Drive/AAU/P8/grasping/multiview_prosthesis_wt",
    queue_name="jetson",
    timeout_seconds=300
)
```
