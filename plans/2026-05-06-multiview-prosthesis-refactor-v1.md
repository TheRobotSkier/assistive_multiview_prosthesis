# Comprehensive Refactor: Multiview Prosthesis Architecture

## Objective

Refactor the multiview prosthesis project from its current fragmented state into a lean, well-structured ROS 2 Jazzy system that implements a clear grasp pipeline: EMG gesture activation → point cloud segmentation → grasp preshaping → proximity-based execution → force control → gesture-based release. The new architecture eliminates MuJoCo simulation, MoveIt overhead, and bloated Docker images in favor of a minimal ROS Jazzy base with RViz-only visualization, centralized configuration, and clean node boundaries.

---

## Current State Assessment

### What Exists (Inventory)

| Component | Location | State | Keep? |
|---|---|---|---|
| **Grasp Preshaping (Rust)** | `dev/grasp_preshaping/` | Core complete: planner, predictor, LUT, TSDF, C API, config.rs | **YES — core asset** |
| **Preshaping Service Bridge (C++)** | `dev/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp` | Functional: loads Rust .so via dlopen, subscribes to pose/twist/cloud, calls FFI, publishes results | **YES — keep, adapt** |
| **Proximity Controller (Python)** | `dev/grasp_preshaping/nodes/grasp_proximity_controller_node.py` | Partially done: hysteresis near/far, partial closure logic | **YES — keep, extend** |
| **Mia Hand Driver (C++)** | `mia_hand_driver/` | Complete: serial protocol, motor/joint control, force reading | **YES — core hardware driver** |
| **Mia Hand Description** | `mia_hand_description/` | URDF/xacro, meshes (.obj), joint state publisher | **YES — needed for RViz** |
| **Mia Hand MuJoCo** | `mia_hand_mujoco/` | Full MuJoCo sim: 3 plugins (base, planner GUI, interactive), scenes, depth publisher | **NO — drop entirely** |
| **Mia Hand ros2_control** | `mia_hand_ros2_control/` | Hardware interface wrapping Mia driver | **YES — needed for hardware** |
| **Mia Hand MoveIt Config** | `mia_hand_moveit_config/` | Full MoveIt2 setup | **NO — overkill, drop** |
| **MindRove EMG Classifier** | `dev/mindrove/` | Complete pipeline: collect → train → live inference → ROS bridge | **YES — core input** |
| **MindRove NaviFlame** | `dev/mindrove_naviflame/` | Alternative gesture system with TCP bridge | **MAYBE — secondary, not in critical path** |
| **Point Cloud Segmentation** | `dev/pc_segmentation/` | InterObject3D with MinkowskiEngine, Flask server + ROS bridge | **YES — but heavy, needs lightening** |
| **Multiview Cameras** | `dev/multiview/` | ROS Humble dual-D435 + fusion | **YES — but needs Jazzy migration** |
| **Haptic Band** | `dev/haptic_band/` | Vibro8 BT bridge + haptic controller | **YES — keep for force feedback** |
| **Dynamixel SDK** | `DynamixelSDK/` | Full SDK (C, C++, Python, ROS wrappers) | **PARTIAL — only pip SDK needed** |
| **Dynamixel Hardware Interface** | `dynamixel_hardware_interface/` | ros2_control for wrist motor | **YES — needed for wrist** |
| **Mia Hand Msgs** | `mia_hand_msgs/` | Custom message types (ForceData, etc.) | **YES — dependency** |

### Key Problems Identified

1. **Bloated Docker**: Single monolithic `ros:jazzy-ros-base` + MuJoCo + MoveIt + everything = massive image, slow builds
2. **Fragmented Configuration**: No central config; thresholds and topics scattered across `config.rs`, Python params, YAML files, and hardcoded strings
3. **No Clear Pipeline Flow**: Nodes exist independently without a coherent orchestration layer
4. **Humble/Jazzy Mismatch**: Multiview cameras run on Humble, everything else on Jazzy — DDS bridging is fragile
5. **Missing Components**: Force controller doesn't exist; localization doesn't exist; gesture-based release doesn't exist
6. **MuJoCo Dependency**: Heavy simulation stack used even for simple visualization — RViz would suffice
7. **Dynamixel SDK Bloat**: Full C/C++/ROS SDK vendored when only `dynamixel-sdk` pip package is needed

---

## Target Architecture

### Pipeline Flow

```
[1] EMG Gesture (MindRove)
     → /emg/gesture_label
        │
[2] Segmentation (InterObject3D)
     ← /fused_pointcloud or /depth/points
     → /segmented_object_cloud
        │
[3] Grasp Preshaping (Rust + C++ bridge)
     ← /hand_pose, /hand_twist, /segmented_object_cloud
     → /grasp/target_hand_pose, /grasp/target_finger_closures, /grasp/wrist_rotation
        │
[4] Proximity Executor
     ← /hand_pose, /grasp/target_*
     → /thumb_pos_ff_controller/commands, /index_..., /mrl_..., /wrist/set_position
     (far: wrist + light preshape → near: full preshape)
        │
[5] Force Controller (NEW)
     ← finger force readings from Mia Hand
     → adjusted closure commands
        │
[6] Release gesture (EMG "OPEN" or "REST")
     → open hand
```

### Package Structure (New)

```
prosthesis_ws/
├── config/
│   └── prosthesis_config.yaml          # Central YAML config
├── docker/
│   ├── Dockerfile.base                  # ros:jazzy + minimal deps
│   ├── Dockerfile.preshaping            # base + Rust toolchain
│   ├── Dockerfile.segmentation          # Python 3.8 + MinkowskiEngine
│   ├── Dockerfile.cameras               # ros:jazzy + realsense
│   ├── Dockerfile.mindrove              # Python + mindrove SDK
│   ├── docker-compose.yml               # All services
│   └── .env                             # UID/GID, BT addresses, ports
├── launch/
│   └── full_pipeline.launch.py          # Single orchestration launch
├── rviz/
│   └── prosthesis.rviz                  # RViz config
└── src/
    ├── mia_hand_description/            # URDF + meshes (keep as-is)
    ├── mia_hand_driver/                 # Serial driver (keep as-is)
    ├── mia_hand_msgs/                   # Custom messages (keep as-is)
    ├── mia_hand_ros2_control/           # Hardware interface (keep as-is)
    ├── dynamixel_hw/                    # Lightweight wrist motor interface
    ├── grasp_preshaping/                # Rust lib + C++ bridge + proximity ctrl
    ├── emg_bridge/                      # MindRove classifier + ROS bridge
    ├── segmentation/                    # Segmentation node (lighter wrapper)
    ├── force_controller/                # NEW: force regulation node
    ├── haptic_band/                     # BT bridge + haptic controller
    └── pipeline_manager/               # NEW: state machine orchestrator
```

---

## Implementation Plan

### Phase 1: Foundation — Central Config + Minimal Docker

- [ ] **1.1** Create `config/prosthesis_config.yaml` with all tunable parameters organized by subsystem:
  - `emg:` gesture names, confidence threshold, sampling rate
  - `segmentation:` inference URL, cube edge, input/output topics
  - `preshaping:` all current `config.rs` values (TSDF resolution, truncation, collision tolerance, prediction horizon, scoring weights, closure fraction)
  - `execution:` proximity enter/exit thresholds, partial closure factor, control rate
  - `force:` target force range, max force, integral gain, update rate
  - `topics:` all ROS topic names (input_cloud, hand_pose, hand_twist, segmented_cloud, etc.)
  - `hardware:` serial ports, dynamixel IDs, BT addresses
  - `frames:` world frame, hand frame, camera frames

  *Rationale*: A single YAML file loaded by every node via ROS parameters eliminates the scattered config problem and makes the system tunable without code changes.

- [ ] **1.2** Create `docker/Dockerfile.base` starting from `ros:jazzy-ros-base` with only:
  - `python3-pip`, `libserial-dev`, `python3-colcon-common-extensions`
  - `ros-jazzy-rviz2`, `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`
  - `ros-jazzy-xacro`, `ros-jazzy-joint-state-publisher-gui`
  - `ros-jazzy-tf2-ros`, `ros-jazzy-rmw-cyclonedds-cpp`
  - `dynamixel-sdk` (pip), `requests` (pip)
  - NO MuJoCo, NO MoveIt, NO glfw, NO libegl

  *Rationale*: This should produce an image ~2-3 GB instead of the current ~8-10 GB. Build times drop from 20+ minutes to ~5 minutes.

- [ ] **1.3** Create `docker/Dockerfile.preshaping` extending base with Rust toolchain (or use a multi-stage build / separate `rust_build` service as current).

- [ ] **1.4** Create `docker/docker-compose.yml` with these services:
  - `prosthesis_core` (base image): runs the pipeline manager + hand driver + RViz
  - `preshaping` (preshaping image): bridge node
  - `segmentation_inference` (Python 3.8 MinkowskiEngine): Flask server
  - `segmentation_ros2` (base image): ROS bridge to inference
  - `cameras` (base image + realsense): single or dual D435
  - `mindrove` (mindrove image): EMG classifier + ROS bridge
  - `haptic` (base image + BT): haptic bridge
  - `rust_build` (rust:slim): one-shot .so compilation

- [ ] **1.5** Create `docker/.env` template with all configurable environment variables.

### Phase 2: Core Package Migration

- [ ] **2.1** Migrate `mia_hand_description/` as-is — URDF xacros, .obj meshes, joint state publisher. This is a clean package with no changes needed.

- [ ] **2.2** Migrate `mia_hand_driver/` as-is — the C++ serial driver (`cpp_driver.cpp`, `ros_driver.cpp`) is complete and functional. No changes needed.

- [ ] **2.3** Migrate `mia_hand_msgs/` as-is — custom message definitions (ForceData, etc.).

- [ ] **2.4** Migrate `mia_hand_ros2_control/` — keep the hardware interface that wraps the Mia driver for ros2_control. Strip out the `test_joint_trajectory_controller` launch if unused.

- [ ] **2.5** Create lightweight `dynamixel_hw/` package:
  - Use only `dynamixel-sdk` pip package (not the vendored DynamixelSDK C/C++/ROS tree)
  - Implement a minimal ros2_control hardware interface for the wrist Dynamixel motor
  - Reference: current `dynamixel_hardware_interface/` logic, but rewritten cleanly

  *Rationale*: The current `DynamixelSDK/` directory is ~300+ files of C, C++, Python, ROS wrappers. Only the Python pip package is needed.

- [ ] **2.6** Migrate `grasp_preshaping/` package:
  - Rust library: keep all of `src/` unchanged (config.rs, planner.rs, predictor.rs, lut_helper.rs, pointcloud_helper.rs, c_api.rs, debug_export.rs)
  - C++ bridge node: keep `preshaping_service_bridge_node.cpp` but update topic names to read from parameters
  - Python proximity controller: keep `grasp_proximity_controller_node.py` but update topic names to read from parameters
  - FFI types header: keep `include/grasp_preshaping/ffi_types.hpp`
  - CMakeLists.txt and package.xml: keep, adapt install rules
  - Data files: ensure `data/finger_contact_lut.npz` is tracked (currently appears missing from data/ — verify .gitignore or build artifact)

- [ ] **2.7** Create `emg_bridge/` package from `dev/mindrove/`:
  - Keep: `emg_classifier/` (board_reader, features, classifier, preprocessing, proportional, config, ros_bridge_node)
  - Keep: `scripts/` (collect_data, train, run_classifier)
  - Keep: Dockerfile and Dockerfile.ros
  - Update: topic names configurable via parameters matching central config

- [ ] **2.8** Create `segmentation/` package from `dev/pc_segmentation/nodes/`:
  - Keep: `segmentation_ros2_node.py` (the ROS bridge)
  - Keep: `inference_server.py` (Flask server)
  - Drop: the entire MinkowskiEngine source tree from the build context — it should only be in the inference Docker image
  - The inference Dockerfile (`docker/Dockerfile.cpu`) stays as-is since it needs Python 3.8 + MinkowskiEngine

- [ ] **2.9** Migrate `haptic_band/`:
  - Keep: `haptic_bridge/` package (bridge_node.py, haptic_controller_node.py)
  - Keep: Dockerfile
  - Simplify: remove the second BT device if not used, or make it configurable

### Phase 3: New Components

- [ ] **3.1** Create `force_controller/` package — a Python ROS 2 node that:
  - Subscribes to Mia Hand finger force readings (`/mia_hand/forces` or equivalent)
  - Subscribes to current closure commands (from proximity controller)
  - Publishes adjusted closure commands that regulate force to a target range
  - Implements a simple PI controller per finger:
    - If force < target_min: increase closure
    - If force > target_max: decrease closure
    - If no contact detected (force = 0) and in "near" mode: continue closing
  - Parameters from central config: target_force_range, max_force, integral_gain, update_rate
  - State machine: IDLE → CLOSING (ramp up) → HOLDING (regulate) → RELEASING (open)

  *Rationale*: This is the missing piece between "preshape" and "stable grasp." Without force control the hand either drops the object or crushes it.

- [ ] **3.2** Create `pipeline_manager/` package — a Python ROS 2 node that acts as the system state machine:
  - States: `IDLE`, `SEGMENTING`, `PLANNING`, `APPROACHING`, `GRASPING`, `HOLDING`, `RELEASING`
  - Subscribes to `/emg/gesture_label` to trigger transitions:
    - `POWER` or `PINCH` → triggers segmentation → planning → approach → grasp
    - `OPEN` → triggers release
    - `REST` → IDLE
  - Manages the overall pipeline lifecycle:
    - Activates segmentation when gesture detected
    - Calls preshaping service when segmentation returns cloud
    - Monitors proximity controller state
    - Manages force controller activation
  - Publishes system state to `/pipeline/state` for RViz visualization
  - Health monitoring: watchdog on each subsystem, logs failures clearly

  *Rationale*: Currently there is no orchestration — the full_system_test just launches everything and hopes they connect. A state machine makes the flow explicit and debuggable.

- [ ] **3.3** Create RViz visualization config (`rviz/prosthesis.rviz`):
  - Show Mia Hand URDF model with live joint states
  - Show point cloud (segmented and raw)
  - Show grasp target pose (axes marker)
  - Show pipeline state (text overlay)
  - Show force readings (colored markers on fingertips)
  - Show EMG gesture (text)
  - Camera views: world view + (optionally) wrist camera view

  *Rationale*: Replace MuJoCo simulation entirely with RViz visualization. The hand model, point clouds, and state are all viewable in RViz without the massive MuJoCo dependency.

### Phase 4: Integration and Launch

- [ ] **4.1** Create `launch/full_pipeline.launch.py` that:
  - Loads `config/prosthesis_config.yaml` as ROS parameters
  - Launches all nodes in correct dependency order:
    1. Mia hand driver (hardware)
    2. Dynamixel wrist driver (hardware)
    3. EMG bridge
    4. Camera nodes + segmentation
    5. Preshaping bridge
    6. Proximity controller
    7. Force controller
    8. Pipeline manager
    9. Haptic controller
    10. RViz
  - Uses composable nodes where possible for efficiency
  - Supports `use_mock_hardware:=true` for testing without physical hardware

- [ ] **4.2** Create mock/simulated data sources for testing:
  - Mock EMG publisher (cycles through gestures)
  - Mock point cloud publisher (publishes a simple object cloud)
  - Mock hand pose/twist publisher
  - These allow testing the full pipeline without any hardware

- [ ] **4.3** Update the `Dockerfile.base` to include any missing ROS packages discovered during integration.

### Phase 5: Cleanup and Documentation

- [ ] **5.1** Remove from the new workspace:
  - `mia_hand_mujoco/` — entire MuJoCo simulation package
  - `mia_hand_moveit_config/` — MoveIt configuration
  - `DynamixelSDK/` — full vendored SDK (pip package replaces it)
  - `dev/mujoco/` — MuJoCo scenes, interactive simulator, GUI simulator
  - `dev/pc_segmentation/MinkowskiEngine/` — source tree (only needed in inference Docker)
  - `dev/pc_segmentation/tests/` — MinkowskiEngine tests
  - `dev/pc_segmentation/examples/` — MinkowskiEngine examples
  - `dev/pc_segmentation/docs/` — MinkowskiEngine docs
  - `dev/mindrove_cpp/` — C++ MindRove experiment
  - `dev/mindrove_naviflame/` — secondary gesture system (keep as archive)
  - `dev/multiview/` — old Humble-based camera setup (replaced by Jazzy-native)
  - `dev/hardware/` — standalone test scripts

- [ ] **5.2** Migrate multiview cameras to Jazzy:
  - Rewrite `Dockerfile.humble_cameras` to use `ros:jazzy-ros-base`
  - Replace `ros-humble-realsense2-camera` with `ros-jazzy-realsense2-camera`
  - Update `pointcloud_fusion_node.py` for Jazzy API compatibility
  - This eliminates the Humble/Jazzy DDS bridging problem

- [ ] **5.3** Verify `finger_contact_lut.npz` is properly tracked in the new repo structure. Currently `data/` appears empty — this file is critical for the Rust preshaping pipeline (loaded at `c_api.rs:290`). It may need to be regenerated using `scripts/model.py` or tracked via Git LFS.

---

## Verification Criteria

- [ ] `docker compose build` completes in under 10 minutes for base image
- [ ] Total Docker image storage < 5 GB (base + preshaping + segmentation)
- [ ] `docker compose up` launches full pipeline with mock hardware and all nodes reach active state
- [ ] RViz shows hand model, point cloud, grasp target, and pipeline state
- [ ] Central config YAML can change any threshold/topic without code changes
- [ ] EMG gesture triggers segmentation → preshaping → proximity execution flow
- [ ] Force controller regulates finger closure based on force readings
- [ ] "OPEN" gesture triggers hand release
- [ ] Pipeline manager state machine transitions are logged and visible
- [ ] No MuJoCo, MoveIt, or Humble dependencies remain

---

## Potential Risks and Mitigations

1. **finger_contact_lut.npz missing from version control**
   - The Rust preshaping pipeline requires this LUT file at runtime
   - Mitigation: Verify it exists in the current build artifacts, add to Git LFS if large, or document regeneration via `scripts/model.py`

2. **Segmentation inference server requires Python 3.8 + MinkowskiEngine**
   - This is fundamentally incompatible with ROS Jazzy (Python 3.12)
   - Mitigation: Keep the current architecture of separate containers (Python 3.8 inference server + Jazzy ROS bridge). This is already the working pattern.

3. **Realsense2 camera drivers may not have Jazzy packages yet**
   - `ros-jazzy-realsense2-camera` may not exist in apt
   - Mitigation: Check package availability; if missing, build from source in Dockerfile or keep Humble container for cameras only with CycloneDDS bridging

4. **Dynamixel hardware interface rewrite may introduce bugs**
   - Current C++ interface is battle-tested
   - Mitigation: Keep the current `dynamixel_hardware_interface/` package as-is initially, only replace the vendored DynamixelSDK with the pip package for the Python path. The C++ hardware interface can continue using the SDK's C++ library.

5. **Force controller has no reference implementation**
   - This is entirely new code with no existing test data
   - Mitigation: Start with a simple bang-bang controller (close if below threshold, open if above), then iterate to PI control. Test extensively with mock force data first.

6. **Pipeline manager state machine complexity**
   - Real-world grasping has many edge cases (dropped objects, failed grasps, timeouts)
   - Mitigation: Start with minimal states (IDLE → GRASPING → HOLDING → RELEASING), add error states incrementally. Use ROS 2 lifecycle nodes for clean startup/shutdown.

7. **Mia hand driver serial protocol is fragile**
   - The C++ driver uses raw serial with manual byte parsing
   - Mitigation: Keep the existing driver unchanged — it works. Wrap it in ros2_control for clean integration.

---

## Alternative Approaches

1. **Keep MuJoCo for development testing only**: Instead of removing MuJoCo entirely, keep it as a separate `docker compose --profile sim` profile. This allows testing without hardware but doesn't burden the production pipeline. Trade-off: larger repo, more maintenance, but safer transition.

2. **Use ROS 2 components (composable nodes)**: Instead of separate processes for each node, load multiple nodes into a single process using `rclcpp_components`. Trade-off: better performance and lower latency, but harder to debug and less fault-isolated.

3. **Replace InterObject3D with a lighter segmentation**: The current MinkowskiEngine-based segmentation requires a separate Python 3.8 container. Alternatives include a simple depth-based clustering (Euclidean clustering) or a lighter neural network. Trade-off: less accurate segmentation but much simpler deployment.

4. **Use existing force control from ros2_control**: Instead of a custom force controller, leverage ros2_control's `joint_trajectory_controller` with force feedback. Trade-off: more standard but less flexible for the specific Mia Hand force sensing protocol.

5. **Keep MoveIt for wrist path planning**: If the wrist needs to follow complex trajectories (not just rotation), MoveIt could be valuable. Trade-off: significant Docker/image overhead for potentially simple wrist motion.
