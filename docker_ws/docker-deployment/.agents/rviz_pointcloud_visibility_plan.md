# High-Level Plan: RViz Pointcloud Visibility + Digital Twin Service

**Branch:** `rviz_pointcloud_visibility`  
**Goal:** Create a Docker service that visualizes real camera pointclouds, segmented objects, and a simulated Mia Hand "digital twin" that executes grasp plans computed from the real-world scene. This allows end-to-end verification of the perception-segmentation-grasping pipeline without the physical Mia Hand hardware.

---

## Context

The project already has:
- **Real cameras**: `multiview_full` publishes `/fused_pointcloud` (ROS2 Humble, bridged via DDS to Jazzy).
- **Segmentation**: `segmentation_ros2_node` + `segmentation_inference` server consume pointclouds and clicks, publish `/segmentation/object_cloud`.
- **Grasp planning**: `preshaping_service_bridge_node` (C++ / Rust FFI) exposes `/grasp_preshaping/compute_grasp`.
- **Grasp execution**: `grasp_proximity_controller_node` implements far-away partial-preshape + near-zone full-closure logic.
- **Mia Hand simulation**: `mia_hand_mujoco` with `InteractiveSystemInterface` publishes `/hand_pose`, `/hand_twist`, and accepts joint commands via ROS2 Control.
- **RViz configs**: Existing configs for `full_system_test` and standalone `mia_hand_description`.

**What is missing:** A single launch file and Docker service that wires the *real* camera stream into the *simulated* hand pipeline, with an RViz view showing everything together.

---

## High-Level Steps

| Step | Title | Description | Beads Issue Prefix |
|------|-------|-------------|-------------------|
| 1 | [RViz Configuration](#step-1-rviz-configuration) | Create a dedicated RViz config that displays the fused pointcloud, segmented pointcloud, Mia Hand robot model, TF tree, and the interactive PublishPoint tool. | `rviz-config` |
| 2 | [Digital Twin Launch File](#step-2-digital-twin-launch-file) | Create a ROS2 launch file that brings up the Mia Hand MuJoCo simulation (digital twin), segmentation node, click relay, grasp preshaping bridge, grasp proximity controller, and RViz. | `launch-digital-twin` |
| 3 | [Topic & TF Bridges](#step-3-topic--tf-bridges) | Add lightweight nodes to remap `/fused_pointcloud` → `/segmentation/input_cloud` and publish a static TF aligning the real camera frame with the simulation `world` frame. | `bridge-nodes` |
| 4 | [Docker Service Definition](#step-4-docker-service-definition) | Add a new `digital_twin` service to `docker-compose.linux-podman.yml` (and `.windows.yml`) extending `miahand_ros2` with X11, the correct command, and a `standalone` profile. | `docker-service` |
| 5 | [Integration, Test & Documentation](#step-5-integration-test--documentation) | Validate the full pipeline: pointcloud visible → segmentation with clicks → grasp service call → simulated hand preshapes and closes as "approach" is simulated. Write runbook. | `integration-test` |

---

## Step 1: RViz Configuration

Create `dev/mujoco/config/digital_twin.rviz`.

**Displays to include:**
- `TF` — show the full transform tree.
- `RobotModel` — subscribe to `/robot_description` (published by the Mia Hand simulation), show the Mia Hand URDF.
- `PointCloud2` **"FusedCloud"** — topic `/fused_pointcloud`, axis-color by Z, small point size.
- `PointCloud2` **"SegmentedCloud"** — topic `/segmentation/object_cloud`, flat orange color, slightly larger points.
- `MarkerArray` **"SeedMarkers"** — topic `/segmentation/seed_markers` (if available).
- `PublishPoint` tool configured to publish to `/clicked_point`.

**Global options:**
- Fixed Frame: `world`
- Background: dark grey

---

## Step 2: Digital Twin Launch File

Create `dev/mujoco/launch/digital_twin_launch.py`.

**Nodes to launch (in order):**

1. **Mia Hand MuJoCo simulation**
   - `ros2 launch mia_hand_mujoco mia_hand_system_interface_launch.py`
   - Args: `hardware_plugin:=mia_hand_mujoco/InteractiveSystemInterface`, `enable_depth_publisher:=false`, `enable_preshaping_service:=false`, `scene:=static`, `include_wrist:=true`
   - *Why `InteractiveSystemInterface`*: it publishes `/hand_pose` and `/hand_twist`, required by the grasp planner and proximity controller.

2. **Topic relay**
   - `ros2 run topic_tools relay /fused_pointcloud /segmentation/input_cloud`
   - Or a Python relay node if `topic_tools` is not in the image.

3. **Segmentation node**
   - `python3 /miahand_ws/src/dev/pc_segmentation/nodes/segmentation_ros2_node.py`
   - Parameter: `inference_url:=http://127.0.0.1:5678`

4. **Click relay**
   - `python3 /miahand_ws/src/dev/pc_segmentation/nodes/demo_click_relay_node.py`
   - Converts RViz `/clicked_point` → `/segmentation/click_positive`/`negative`.

5. **Grasp preshaping service bridge**
   - `preshaping_service_bridge_node`
   - Parameters: `camera_frames:=['cam1_d435_1_color_optical_frame', 'cam2_d435_2_color_optical_frame']`, `preshaping_closure_fraction:=0.3`
   - *Note:* the service must be called manually (or via a trigger node) to compute the grasp.

6. **Grasp proximity controller**
   - `python3 /miahand_ws/src/dev/grasp_preshaping/nodes/grasp_proximity_controller_node.py`
   - Parameters: `proximity_enter_threshold_m:=0.08`, `partial_closure_factor:=0.3`

7. **Static TF publisher (camera → world)**
   - `static_transform_publisher` from `world` to `cam1_d435_1_color_optical_frame`.
   - Translation/rotation should be launch arguments so users can set them to their physical camera mounting pose.

8. **RViz**
   - `rviz2 -d /miahand_ws/src/dev/mujoco/config/digital_twin.rviz`
   - Delayed start (TimerAction, 5 s) to let other nodes initialise.

**Optional / for testing:**
- **Hand trajectory node** — `mujoco_hand_trajectory_node.py` can be included (commented out by default) to automatically move the simulated hand from a far pose to a near pose, exercising the proximity controller.

---

## Step 3: Topic & TF Bridges

### 3.1 Pointcloud Relay
The real cameras publish on `/fused_pointcloud`. The segmentation node expects `/segmentation/input_cloud`. A `topic_tools relay` or small Python relay node closes this gap.

### 3.2 Camera Static TF
The grasp planner looks up camera frames relative to `world`. The RealSense driver does not know about `world`. We publish a static TF:
```
world → cam1_d435_1_color_optical_frame
```
with configurable translation and rotation (launch args). Users set these to match their physical camera rig.

---

## Step 4: Docker Service Definition

Add to `docker-compose.linux-podman.yml` (and `.windows.yml`):

```yaml
digital_twin:
  extends: miahand_ros2
  container_name: miahand_digital_twin
  profiles: [standalone]
  environment:
    - CAMERA_FRAME_WORLD_X=${CAMERA_FRAME_WORLD_X:-0.0}
    - CAMERA_FRAME_WORLD_Y=${CAMERA_FRAME_WORLD_Y:-0.0}
    - CAMERA_FRAME_WORLD_Z=${CAMERA_FRAME_WORLD_Z:-1.0}
    - CAMERA_FRAME_WORLD_QX=${CAMERA_FRAME_WORLD_QX:-0.0}
    - CAMERA_FRAME_WORLD_QY=${CAMERA_FRAME_WORLD_QY:-0.0}
    - CAMERA_FRAME_WORLD_QZ=${CAMERA_FRAME_WORLD_QZ:-0.0}
    - CAMERA_FRAME_WORLD_QW=${CAMERA_FRAME_WORLD_QW:-1.0}
  command: >
    bash -c "
      source install/setup.bash &&
      ros2 launch dev/mujoco/launch/digital_twin_launch.py
        camera_world_x:=${CAMERA_FRAME_WORLD_X:-0.0}
        camera_world_y:=${CAMERA_FRAME_WORLD_Y:-0.0}
        camera_world_z:=${CAMERA_FRAME_WORLD_Z:-1.0}
        camera_world_qx:=${CAMERA_FRAME_WORLD_QX:-0.0}
        camera_world_qy:=${CAMERA_FRAME_WORLD_QY:-0.0}
        camera_world_qz:=${CAMERA_FRAME_WORLD_QZ:-0.0}
        camera_world_qw:=${CAMERA_FRAME_WORLD_QW:-1.0};
      /bin/bash
    "
```

**Prerequisites documented:**
- `rust_build` service must have been run once so `libgrasp_preshaping.so` exists.
- `segmentation_inference` service must be running (or started alongside).
- `multiview_full` must be running to produce `/fused_pointcloud`.

---

## Step 5: Integration, Test & Documentation

**Test sequence:**
1. Start `multiview_full` (real cameras).
2. Start `segmentation_inference`.
3. Run `rust_build` (one-time).
4. Start `digital_twin` service.
5. In RViz, verify `/fused_pointcloud` is visible.
6. Use PublishPoint tool to click on the object → verify `/segmentation/object_cloud` appears.
7. Call the grasp service:
   ```bash
   ros2 service call /grasp_preshaping/compute_grasp std_srvs/srv/Trigger
   ```
8. Verify simulated hand wrist rotates and fingers partially close (preshape).
9. (Optional) Publish a `/mujoco/move_hand` pose closer to the object → verify fingers fully close when within `proximity_enter_threshold_m`.

**Deliverables:**
- Working Docker service.
- Updated README or inline docs showing the startup sequence.
- Beads issues closed for each step.

---

## Beads Task Mapping

Each high-level step maps to one or more beads issues. See the corresponding `impl_step_*.md` files in this folder for detailed acceptance criteria and sub-tasks.

| Beads Issue (proposed) | Title | Blocks |
|------------------------|-------|--------|
| `rviz-config` | Create digital twin RViz configuration | `launch-digital-twin` |
| `launch-digital-twin` | Create digital twin ROS2 launch file | `docker-service` |
| `bridge-nodes` | Add pointcloud relay and camera static TF | `launch-digital-twin` |
| `docker-service` | Add digital_twin Docker compose service | `integration-test` |
| `integration-test` | Integrate and validate full digital twin pipeline | — |

**Dependencies:**
```
rviz-config  →  launch-digital-twin  →  docker-service  →  integration-test
bridge-nodes  ↗
```
