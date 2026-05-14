# Goal: ArUco Marker Position Estimation with IMU-EKF Fusion

## Purpose

Implement a simpler, more robust alternative to OpenVINS for hand/camera position
estimation using ArUco marker detection fused with IMU data through an Extended
Kalman Filter (EKF).

OpenVINS is fragile, requires careful calibration, and is overkill for a structured
environment with known marker placements. The goal here is a deterministic, reliable
system that an agent can implement and test incrementally.

## Hardware

- **Cameras:** Intel RealSense **D435** (non-i, no built-in IMU). Two units:
  - Head-mounted (stationary reference camera)
  - Arm/wrist-mounted (hand-pose tracking camera)
- **IMUs:** External ESP32-based dual GY-91 IMU units, connected over serial USB,
  bridged into ROS2 via the serial bridge node in `src/sensor_fusion_bringup/`.
  Topics: `/cam0/data_raw` (head IMU), `/cam1/data_raw` (arm IMU).
- **Jetson Nano:** Runs the Docker containers that stream camera + IMU data over
  the direct Ethernet link (`robotlab@robotlab.local`). The host x86 PC receives
  data over the ROS2 DDS network.

## Architecture

```
D435 camera frames (30 Hz) ──→  ArUco detector node  ──→  marker pose estimate
                                                              │
ESP32 IMUs (100-200 Hz) ─────────────────────────────────→  EKF (robot_localization)
                                                              │
                                                         /hand_pose (world → wrist_link)
```

**Key design principle:** IMU integration provides continuous high-rate position
updates between camera frames. Each new ArUco observation "resets" accumulated IMU
drift by providing an absolute pose correction into the EKF. This is a loose-coupling
approach — the EKF state is not reset, but the marker observation is fed as an
absolute pose measurement.

## What Needs to Be Built

### 1. ArUco Detector Node (`src/sensor_fusion_bringup/`)

A ROS2 node (Python or C++) that:
- Subscribes to `/cam0/color/image_raw` (and optionally `/cam1/color/image_raw`)
- Detects ArUco markers using OpenCV (`cv2.aruco`)
- Uses camera intrinsics from `/cam0/color/camera_info` for pose estimation
- Publishes detected marker poses as `geometry_msgs/PoseStamped` on
  `/aruco/head/pose` and `/aruco/arm/pose`
- Publishes as `sensor_msgs/PoseWithCovarianceStamped` for EKF input
  (covariance should reflect reprojection error quality)

Reference: There are existing marker-related files in
`src/sensor_fusion_bringup/config/markers/` and launch files
`arm_marker_pose_phase2.launch.py` / `head_marker_pose_phase2.launch.py` —
study these before implementing.

Also check `sensor_fusion_msgs/msg/MarkerPoseObservation.msg` for the existing
message type.

### 2. EKF Configuration

Extend or replace `ekf_dual_imu.yaml` to fuse:
- IMU orientation + angular velocity (existing)
- ArUco pose observations as absolute position measurements

Use `robot_localization` package's `ekf_node`. The ArUco pose feeds the
`pose0` input. Configure covariance appropriately — marker detection at
~0.5m range has ~5-10mm position accuracy.

### 3. Launch Integration

Modify or create a launch file that:
- Starts the ArUco detector node
- Starts the EKF node with the new config
- Publishes `/hand_pose` on the same topic expected by `twist_propagation`
- Replaces the OpenVINS launch path in `dual_openvins_phase2.launch.py`

### 4. TF Integration

The EKF output should maintain the same TF tree as OpenVINS:
```
world → odom → cam0_base_link  (from EKF)
world → wrist_link             (from hand pose)
```
See `docs/tf_tree.md` for the full tree.

## Definition of Done

- [ ] ArUco detector node publishes stable poses when markers are visible
- [ ] EKF fuses IMU + ArUco, publishes continuous `/odom` and `/hand_pose`
- [ ] IMU-only mode works (drifts, but doesn't crash) when no markers visible
- [ ] Position estimate "snaps" to correct pose when marker comes back into view
- [ ] Launch file starts the full pipeline cleanly
- [ ] Tested on Jetson via `make jetson-cameras` → observe topics from host

## Key Files to Read First

```
ekf_dual_imu.yaml                                    # existing EKF config
src/sensor_fusion_bringup/launch/                   # existing launch files
src/sensor_fusion_bringup/config/markers/           # marker configs
src/sensor_fusion_bringup/config/d435i_cameras.yaml # camera config (note: D435 non-i here)
sensor_fusion_msgs/msg/MarkerPoseObservation.msg    # existing message type
docs/tf_tree.md                                     # TF frame layout
docs/ROBOTLAB_ARCHITECTURE.md                       # system overview
```

## Jetson Access

All Jetson interaction goes through the agent-task-queue MCP tool.
See `AGENTS.md` for the full queue protocol.

Quick reference for the most common task:
```python
# Sync code + start cameras on Jetson
run_task(
    command="scripts/jetson_run.sh make jetson-cameras",
    working_directory="/home/asger/Drive/AAU/P8/grasping/wt-aruco-ekf",
    queue_name="jetson",
    timeout_seconds=300
)
```

After the Jetson containers are running, you can observe ArUco topics from the
host without queuing (the data comes over the DDS network automatically).
