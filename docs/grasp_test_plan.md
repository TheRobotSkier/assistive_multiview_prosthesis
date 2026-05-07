# Grasp Test Pipeline — Integration Test Plan

## Purpose

This integration test validates the end-to-end grasp pipeline: from point cloud capture through segmentation, twist propagation, grasp planning, and proximity-controlled execution. It exercises all ROS 2 nodes defined in `grasp_test.launch.py` and confirms that the pipeline state machine completes a full grasp cycle.

## Architecture Overview

The grasp test pipeline consists of the following components, chained in order:

| # | Component | Package | Executable | Role |
|---|-----------|---------|------------|------|
| 1 | **Cloud Source** | `pipeline_manager` (mock) or `camera` (D435) | `mock_cloud_publisher` or `two_d435_launch.py` | Provides point cloud input. Mock mode publishes a synthetic cloud; camera mode uses dual RealSense D435. |
| 2 | **Segmentation Bridge** | `segmentation_bridge` | `segmentation_ros2_node` | Forwards the scene cloud to an external HTTP inference server (`/segmentation/input_cloud` → inference → `/segmentation/object_cloud`). |
| 3 | **Cloud Snapshot Node** | `camera` | `cloud_snapshot_node` | Subscribes to `/segmentation/object_cloud` and republishes it as `/segmentation/object_cloud_snapshot`. Freezes the cloud during PLANNING state for RViz visualization. |
| 4 | **Hand Pose Publisher** | `camera` | `hand_pose_publisher` | Reads `wrist_link` TF transform and publishes `/hand_pose` (geometry_msgs/PoseStamped). |
| 5 | **Twist Propagation** | `twist_propagation` | `twist_propagation_node` | Detects hand→object collision by propagating hand twist forward in time; publishes collision events and refined twist. |
| 6 | **Grasp Preshaping Service** | `grasp_preshaping` | `preshaping_service_bridge_node` | C++/Rust FFI bridge that solves the grasp optimization problem: TSDF construction, SMC sampling, grasp scoring. |
| 7 | **Grasp Proximity Controller** | `grasp_preshaping` | `grasp_proximity_controller_node.py` | Approach & contact phase: publishes wrist rotation and fractional finger closures when far, full closures when near the target. |
| 8 | **Pipeline Manager** | `pipeline_manager` | `pipeline_manager_node` | State machine orchestrator that sequences the pipeline through states: IDLE → SEGMENTING → PLANNING → APPROACHING → GRASPING → HOLDING → RELEASING → IDLE. |
| 9 | **Wrist Driver** | `wrist_driver` | `wrist_driver_node` | Low-level Dynamixel driver for wrist rotation. |
| 10 | **Robot State Publisher** | `robot_state_publisher` | `robot_state_publisher` | Publishes hand URDF and TF transforms. |
| 11 | **RViz** | `rviz2` | `rviz2` | Visualization with `grasp_test.rviz` config. |

### Data Flow

```
Cloud Source → Segmentation Bridge → Cloud Snapshot Node → RViz
                                          ↓
Hand Pose Publisher → Twist Propagation → Preshaping Service → Proximity Controller → Hand
                        ↓                      ↓                      ↓
                    Pipeline Manager (state machine orchestrator)
                        ↓
                    /pipeline/state_name
```

## Pre-requisites

### Hardware

- **Mia Hand** connected via USB serial port (default: `/dev/ttyUSB1`)
- **Wrist Dynamixel** connected via USB (default: `/dev/ttyUSB0`)
- (Optional) **2× Intel RealSense D435** mounted for stereo view — required for `--camera` mode

### Software

- **Docker/Podman** with compose plugin (`podman-compose` or `docker compose`)
- **ROS 2 Jazzy** workspace built at `/prosthesis_ws`
- **Segmentation inference server** running on `http://127.0.0.1:5678`
- **Rust grasp-preshaping library** compiled (`libgrasp_preshaping.so` via `rust_build` one-shot service)

### Verification

```bash
# Check podman-compose
podman-compose --version

# Check ROS 2
ros2 node list

# Verify workspace is sourced
echo $ROS_DISTRO  # should print "jazzy"

# Check Rust library exists
ls docker_ws/dev/grasp_preshaping/target/release/libgrasp_preshaping.so
```

## Setup Steps

### 1. Build containers

```bash
cd docker
podman-compose build
```

### 2. (One-time) Build Rust grasp-preshaping library

```bash
cd docker_ws/docker-deployment
docker compose --profile standalone up rust_build
```

### 3. Start the segmentation inference server

```bash
cd docker
podman-compose up -d segmentation
```

### 4. Run the integration test

```bash
# Default: mock cloud, no camera
./scripts/grasp_test.sh

# With real cameras
./scripts/grasp_test.sh --camera

# Monitor-only mode (attach to a running pipeline)
./scripts/grasp_test.sh --monitor
```

## Test Scenarios

### Scenario 1: Basic — Mock Cloud, Manual Hand Movement

**Objective:** Verify the pipeline runs end-to-end with synthetic data and manual hand control.

**Setup:**
```bash
./scripts/grasp_test.sh
```

**Steps:**
1. Pipeline starts with mock cloud publisher providing synthetic point cloud data.
2. Segmentation bridge forwards mock cloud to inference server.
3. Pipeline manager transitions through states.
4. Manually move the hand (via joint_state_publisher_gui if available) to trigger twist propagation.
5. Observe states via `--monitor` or `ros2 topic echo /pipeline/state_name`.

**Success Criteria:**
- All nodes start without errors.
- Pipeline reaches HOLDING state within 60 seconds.
- Twist propagation detects hand→object interaction.
- Preshaping service computes a valid grasp.

---

### Scenario 2: Camera — Real D435, Manual Hand Movement

**Objective:** Verify the pipeline with real depth camera input.

**Setup:**
```bash
./scripts/grasp_test.sh --camera
```

**Steps:**
1. Two RealSense D435 cameras start and publish depth point clouds.
2. Point cloud fusion merges both camera streams.
3. Pipeline runs as in Scenario 1 but with live camera data.
4. Manually move the hand in front of the cameras.

**Success Criteria:**
- Both cameras publish clouds (verify via `ros2 topic hz /cam1/depth/color/points`).
- Segmentation produces valid object cloud.
- Pipeline reaches HOLDING state.
- No TF lookup failures in logs.

---

### Scenario 3: Automated — Hand Trajectory Publisher

**Objective:** Full cycle without manual hand movement using `mujoco_hand_trajectory_node`.

**Setup:**
```bash
# Start the trajectory publisher alongside the pipeline
# (Requires MuJoCo simulation running)
./scripts/grasp_test.sh
ros2 run mujoco_hand_trajectory_node --ros-args -p auto_start:=true
```

**Steps:**
1. Hand trajectory publisher moves the simulated hand along a predefined path toward the target object.
2. Pipeline automatically detects the approaching hand, computes grasp, and executes.
3. No manual intervention required.

**Success Criteria:**
- Pipeline transitions through all states autonomously.
- HOLDING state reached within the trajectory duration + pipeline latency.
- All log output shows clean state transitions with no errors.

---

### Scenario 4: Snapshot Test — Cloud Freeze at PLANNING

**Objective:** Verify that the cloud snapshot node correctly freezes the segmented point cloud during PLANNING state.

**Setup:**
```bash
# Run with mock cloud for deterministic behavior
./scripts/grasp_test.sh

# In another terminal, monitor the snapshot topic
ros2 topic echo /segmentation/object_cloud_snapshot
```

**Steps:**
1. Observe that `/segmentation/object_cloud_snapshot` updates freely during IDLE/SEGMENTING states.
2. When pipeline enters PLANNING, the snapshot should freeze (same point cloud published repeatedly).
3. Snapshot should persist during APPROACHING and GRASPING states.
4. On transition to RELEASING, snapshot should clear.

**Success Criteria:**
- Snapshot stamps are identical (frozen) during PLANNING/APPROACHING/GRASPING.
- Snapshot clears on RELEASING.
- No stale cloud persists after RELEASING → IDLE transition.

## Success Criteria Summary

| # | Scenario | Primary Criterion | Secondary Criterion |
|---|----------|-------------------|---------------------|
| 1 | Basic | Reaches HOLDING in ≤60s | All nodes publish on expected topics |
| 2 | Camera | Reaches HOLDING | Camera clouds appear in RViz |
| 3 | Automated | No manual input needed | Full state cycle completes |
| 4 | Snapshot | Cloud freezes at PLANNING | Cloud clears at RELEASING |

## Troubleshooting Guide

### Nodes fail to start

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `container_name "grasp_test" already in use` | Previous run not cleaned up | `podman-compose --profile grasp_test down` |
| `mock_cloud_publisher` not found | Package not built | `colcon build --packages-select pipeline_manager` |
| `libgrasp_preshaping.so` not found | Rust library not compiled | Run `docker compose --profile standalone up rust_build` |
| Segmentation bridge errors | Inference server not running | `podman-compose up -d segmentation` and check `curl http://127.0.0.1:5678/health` |

### Pipeline stuck in a state

| State | Symptom | Fix |
|-------|---------|-----|
| IDLE | No state transition | Ensure hand_pose and cloud topics are being published. Check `ros2 topic list`. |
| SEGMENTING | Stays in SEGMENTING | Check inference server is reachable and segmentation returns valid clouds. |
| PLANNING | Stays in PLANNING | Check preshaping service log — TSDF or optimization may be failing. |
| APPROACHING | Stuck approaching | Check proximity controller parameters. Hand may be too far from target. |

### Diagnostic commands

```bash
# Check which topics are active
ros2 topic list

# Check node list
ros2 node list

# View pipeline state
ros2 topic echo /pipeline/state_name

# View node logs
docker logs grasp_test

# Check point cloud rate
ros2 topic hz /segmentation/object_cloud

# Verify TF tree
ros2 run tf2_tools view_frames.py
```

## How to Extend the Test

### Add a new test scenario

1. Add a new function in `scripts/grasp_test.sh` following the pattern of `run_test_sequence()`.
2. Define success criteria as return codes (0 = pass, 1 = fail).
3. Wire the new scenario into the `main()` case statement.

### Add a new pipeline node

1. Add the node to `grasp_test.launch.py` following the existing pattern.
2. Add a `wait_for_node` call in `grasp_test.sh`.
3. Document the node in the Architecture Overview table above.
4. Add any new test scenarios that exercise the new node's behavior.

### Parameterize test behavior

The script accepts `--camera` and `--monitor` flags. To add new flags:

1. Add a new boolean variable (e.g., `SIMULATION_MODE`).
2. Parse it in the `while [[ $# -gt 0 ]]` loop.
3. Use it to conditionally modify pipeline startup or test flow.
