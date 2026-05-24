# Comprehensive Refactor: Multiview Prosthesis Architecture (v2)

## Objective

Refactor the multiview prosthesis project from its current fragmented state into a lean, flat ROS 2 Jazzy workspace with a single Docker Compose file, a single base image (`osrf/ros:jazzy-desktop`), a central YAML config, and RViz-only visualization. The grasp pipeline flows: EMG gesture → segmentation → preshaping → proximity execution → force control → gesture release.

---

## Design Decisions (Based on Feedback)

1. **Base image**: `osrf/ros:jazzy-desktop` — includes RViz, rqt, and desktop tools out of the box
2. **Docker**: Delete `docker-deployment/` entirely. One new `docker/` folder with a single `docker-compose.yml` (unified Linux + WSL), one `Dockerfile`, one `.env`
3. **Workspace layout**: Flat `src/` workspace — no `dev/` nesting. Every package sits directly under `src/`
4. **Rust `.so`**: Pre-compiled `libgrasp_preshaping.so` checked into the repo (or built locally). No Rust toolchain in Docker
5. **Multiview**: Minimal Jazzy-native camera node for now (single D435). The fancy upgrade is coming later, so don't over-invest
6. **No MuJoCo, no MoveIt**: RViz for visualization only

---

## Current State Summary (Updated After Code Review)

### Grasp Preshaping — Evolved Significantly

The Rust pipeline has grown substantially since the initial assessment:

- **SMC optimization loop** (`c_api.rs:416-498`): Multi-iteration particle filter with convergence-based early termination, elite resampling with decaying proposal variance, grasp type mutation, and score-weighted parent selection
- **Superquadric backside estimation** (`lib.rs:8`, `config.rs:58-65`): New `superquadric` module that fits a superquadric to the point cloud and uses it to estimate the backside of objects in the TSDF
- **API version 5** (`c_api.rs:640`): Response struct now includes `alignment_score`, `force_closure_score`, `contact_count_score`, `contact_score`, `second_best_combined_score`, `second_best_grasp_type`, `pipeline_time_ms`, `smc_iterations_used`
- **Config expanded** (`config.rs`): Now 73 lines with SMC constants (iterations, decay, elite ratio, convergence tol, proposal stds), superquadric params, and increased `PREDICTION_SAMPLES` to 20000
- **FFI types in sync**: `ffi_types.hpp` matches the Rust response struct exactly

### What's Mature vs What's Missing

| Component | Maturity | Notes |
|---|---|---|
| Rust grasp preshaping | **Production** | SMC, superquadric, full scoring pipeline |
| C++ preshaping bridge | **Production** | Loads .so, subscribes, publishes |
| Python proximity controller | **Functional** | Near/far hysteresis, partial closure |
| Mia Hand C++ driver | **Production** | Full serial protocol, force reading |
| Mia Hand URDF + meshes | **Production** | Ready for RViz |
| MindRove EMG classifier | **Production** | Collect/train/infer + ROS bridge |
| Segmentation (InterObject3D) | **Functional** | Two-container pattern works |
| Haptic band bridge | **Functional** | BT + force feedback |
| Force controller | **Missing** | No force regulation exists |
| Pipeline manager | **Missing** | No state machine orchestration |
| Central config | **Missing** | Parameters scattered everywhere |
| Jazzy camera setup | **Missing** | Currently Humble-based |

---

## Target Workspace Layout

```
prosthesis/                              # repo root
├── config/
│   └── prosthesis_config.yaml           # single central config
├── docker/
│   ├── Dockerfile                       # osrf/ros:jazzy-desktop + light deps
│   ├── Dockerfile.segmentation          # Python 3.8 + MinkowskiEngine (separate)
│   ├── docker-compose.yml               # one file, all platforms
│   ├── .env.example                     # template for user-specific vars
│   └── ros_entrypoint.sh                # USB device setup
├── launch/
│   ├── full_pipeline.launch.py          # main orchestration
│   └── mock_hardware.launch.py          # testing without hardware
├── rviz/
│   └── prosthesis.rviz
└── src/
    ├── mia_hand_description/            # URDF + meshes (migrated as-is)
    ├── mia_hand_driver/                 # C++ serial driver (migrated as-is)
    ├── mia_hand_msgs/                   # Custom messages (migrated as-is)
    ├── mia_hand_ros2_control/           # Hardware interface (migrated as-is)
    ├── grasp_preshaping/                # Rust lib (.so) + C++ bridge + proximity ctrl
    │   ├── data/
    │   │   └── finger_contact_lut.npz   # pre-built, checked into repo
    │   ├── include/grasp_preshaping/
    │   │   └── ffi_types.hpp
    │   ├── nodes/
    │   │   ├── preshaping_service_bridge_node.cpp
    │   │   └── grasp_proximity_controller_node.py
    │   ├── src/                         # Rust source (for local dev)
    │   ├── Cargo.toml
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── lib/
    │       └── libgrasp_preshaping.so   # pre-built, checked into repo
    ├── emg_bridge/                      # MindRove classifier + ROS bridge
    ├── segmentation/                    # ROS2 bridge node (inference server separate)
    ├── force_controller/                # NEW: force regulation
    ├── pipeline_manager/               # NEW: state machine
    ├── haptic_band/                     # BT bridge + haptic controller
    └── wrist_driver/                    # Lightweight Dynamixel wrist motor interface
```

---

## Implementation Plan

### Phase 1: Clean Slate — Docker Foundation

- [ ] **1.1** Delete `docker_ws/docker-deployment/` entirely
- [ ] **1.2** Create `docker/Dockerfile` based on `osrf/ros:jazzy-desktop`:
  - Minimal additional apt packages: `libserial-dev`, `python3-pip`, `usbutils`, `setserial`
  - Minimal pip packages: `dynamixel-sdk`, `requests`
  - ROS packages already in desktop image: rviz2, rqt, ros2-control, ros2-controllers, xacro, tf2-ros, cyclonedds
  - User setup (UID/GID from args), dialout group, colcon build
  - NO MuJoCo, NO MoveIt, NO Rust toolchain, NO GLFW, NO libegl
  - Copy workspace source, run `rosdep install` + `colcon build`

- [ ] **1.3** Create `docker/Dockerfile.segmentation` for the Python 3.8 inference server (keep existing `dev/pc_segmentation/docker/Dockerfile.cpu` pattern — this is the one exception that needs its own Python version)

- [ ] **1.4** Create `docker/docker-compose.yml` — single unified file:
  ```
  services:
    prosthesis:        # main container (all ROS nodes + RViz)
    segmentation:      # Python 3.8 inference server
    segmentation_ros2: # ROS bridge (extends prosthesis image)
    mindrove:          # EMG classifier (custom Dockerfile or extends prosthesis)
    haptic:            # BT bridge (extends prosthesis)
  ```
  - Use `network_mode: host` for all (simplest DDS discovery)
  - X11 forwarding for RViz (Linux: /tmp/.X11-unix mount; WSL: detect via env)
  - USB device passthrough via `devices:` or `privileged: true`
  - No platform-specific compose files — handle differences via `.env` variables and entrypoint logic

- [ ] **1.5** Create `docker/.env.example` with:
  - `USER_UID`, `USER_GID`
  - `DISPLAY` (auto-set on Linux, manual on WSL)
  - `MIA_SERIAL_PORT`, `WRIST_PORT`
  - `HAPTIC_BT_ADDR1`
  - `XAUTHORITY` (Linux Wayland)

- [ ] **1.6** Create `docker/ros_entrypoint.sh` — simplified version of current entrypoint (USB device setup, bash exec)

  *Rationale*: Starting fresh with Docker eliminates years of accumulated platform-specific workarounds. A single compose file with `.env` is cleaner than the current `compose.yaml` → `docker-compose.linux-podman.yml` → `docker-compose.windows.yml` chain.

### Phase 2: Flat Workspace Migration

- [ ] **2.1** Create the new `src/` directory structure at repo root (or in a new `ws/` directory — decide based on preference)

- [ ] **2.2** Migrate `mia_hand_description/` — copy as-is from `docker_ws/mia_hand_description/`. Contains URDF xacros, .obj meshes, joint state publisher, RViz config. No changes needed.

- [ ] **2.3** Migrate `mia_hand_driver/` — copy as-is from `docker_ws/mia_hand_driver/`. The C++ serial driver (~1340 lines `cpp_driver.cpp`) is complete and battle-tested. No changes needed.

- [ ] **2.4** Migrate `mia_hand_msgs/` — copy as-is. Custom message definitions (ForceData, etc.) that other packages depend on.

- [ ] **2.5** Migrate `mia_hand_ros2_control/` — copy from `docker_ws/mia_hand_ros2_control/`. Hardware interface wrapping the Mia driver for ros2_control. Remove the test launch file if unused.

- [ ] **2.6** Migrate `grasp_preshaping/` — the most important package:
  - **Rust source** (`src/`): Copy all `.rs` files including the new `superquadric.rs` module. This is for local development only — not built in Docker
  - **Pre-built `.so`**: Build locally with `cargo build --release --lib`, place `libgrasp_preshaping.so` into `grasp_preshaping/lib/` (checked into repo)
  - **Data**: Ensure `finger_contact_lut.npz` exists in `grasp_preshaping/data/` — if missing, generate using `scripts/model.py` or copy from existing build artifacts
  - **C++ bridge**: Copy `preshaping_service_bridge_node.cpp` — update to read topic names from ROS parameters instead of hardcoded strings
  - **FFI types**: Copy `include/grasp_preshaping/ffi_types.hpp` — already in sync with API v5
  - **Python proximity controller**: Copy `grasp_proximity_controller_node.py` — update topic names to parameterized
  - **CMakeLists.txt**: Simplify — no `find_program cargo`, just install the pre-built `.so` from `lib/`
  - **package.xml**: Keep as-is

- [ ] **2.7** Create `emg_bridge/` from `docker_ws/dev/mindrove/`:
  - Copy `emg_classifier/` (board_reader, features, classifier, preprocessing, proportional, config, ros_bridge_node)
  - Copy `scripts/` (collect_data, train, run_classifier)
  - Create a simple `package.xml` and `setup.py` (pure Python package)
  - Update topic names to parameterized

- [ ] **2.8** Create `segmentation/` from `docker_ws/dev/pc_segmentation/nodes/`:
  - Copy `segmentation_ros2_node.py` and `inference_server.py`
  - Create `package.xml` and `setup.py`
  - The heavy MinkowskiEngine source tree stays out of the workspace — only in `Dockerfile.segmentation`
  - Copy weights directory reference (weights downloaded at Docker build time)

- [ ] **2.9** Migrate `haptic_band/` from `docker_ws/dev/haptic_band/`:
  - Copy `haptic_bridge/` (bridge_node.py, haptic_controller_node.py, setup.py)
  - Create `package.xml`
  - Simplify to single BT device (make second optional via config)

- [ ] **2.10** Create `wrist_driver/` — lightweight Dynamixel wrist motor interface:
  - Use `dynamixel-sdk` pip package only
  - Simple Python ros2_control hardware interface or direct topic-based control
  - No vendored DynamixelSDK C/C++ source needed
  - If the current C++ `dynamixel_hardware_interface/` works well, migrate it as-is instead

  *Rationale*: Flat workspace means every package is a first-class citizen. No hidden gems in `dev/` subdirectories. `colcon build` builds everything uniformly.

### Phase 3: Central Configuration

- [ ] **3.1** Create `config/prosthesis_config.yaml` with sections:
  ```yaml
  # Topics - all ROS topic names in one place
  topics:
    emg_gesture: /emg/gesture_label
    emg_confidence: /emg/confidence
    input_cloud: /fused_pointcloud
    segmented_cloud: /segmented_object_cloud
    hand_pose: /hand_pose
    hand_twist: /hand_twist
    target_hand_pose: /grasp/target_hand_pose
    target_finger_closures: /grasp/target_finger_closures
    wrist_rotation: /grasp/wrist_rotation
    grasp_type: /grasp/grasp_type
    thumb_command: /thumb_pos_ff_controller/commands
    index_command: /index_pos_ff_controller/commands
    mrl_command: /mrl_pos_ff_controller/commands
    wrist_command: /wrist/set_position
    haptic_motors: /haptic_band/motors
    finger_forces: /mia_hand/forces
    pipeline_state: /pipeline/state

  # EMG
  emg:
    confidence_threshold: 0.55
    gesture_activate: [1, 2]  # POWER, PINCH
    gesture_release: [3]       # OPEN

  # Segmentation
  segmentation:
    inference_url: http://127.0.0.1:5678
    cubeedge: 0.05

  # Preshaping (mirrors config.rs defaults, overridable here)
  preshaping:
    tsdf_resolution_m: 0.005
    truncation_cells: 4
    collision_tol_m: 0.01
    prediction_horizon_s: 5.0
    prediction_samples: 20000
    iterations: 5
    # ... all other config.rs values

  # Execution
  execution:
    proximity_enter_threshold_m: 0.08
    proximity_exit_threshold_m: 0.10
    partial_closure_factor: 0.3
    min_closure_amount: 0.1
    control_rate_hz: 10.0

  # Force control
  force:
    target_force_min: 100
    target_force_max: 400
    max_closure_delta: 0.05
    integral_gain: 0.001
    update_rate_hz: 20.0

  # Hardware
  hardware:
    mia_serial_port: /dev/ttyUSB1
    wrist_port: /dev/ttyUSB0
    haptic_bt_addr: "84:2E:14:09:E1:4E"

  # Frames
  frames:
    world: world
    hand: mia_hand_link
    camera_front: camera_front_depth_optical_frame
    camera_wrist: camera_wrist_depth_optical_frame
  ```

- [ ] **3.2** Update each node to load its parameters from this file via ROS 2 parameter file loading in the launch file. The launch file passes `parameters: [$(find prosthesis)/config/prosthesis_config.yaml]` and each node reads its namespace.

  *Rationale*: One YAML file, one place to tune everything. No more hunting through `config.rs`, Python `declare_parameter()` defaults, and hardcoded C++ strings. The preshaping Rust `config.rs` stays as compile-time defaults, but the C++ bridge can override via ROS params if needed in the future.

### Phase 4: New Components

- [ ] **4.1** Create `src/pipeline_manager/` — Python ROS 2 node, state machine:
  ```
  States: IDLE → SEGMENTING → PLANNING → APPROACHING → GRASPING → HOLDING → RELEASING → IDLE

  IDLE:
    - Wait for EMG activate gesture (POWER/PINCH)
    - Transition → SEGMENTING

  SEGMENTING:
    - Trigger segmentation (or wait for fresh segmented cloud)
    - On cloud received → transition → PLANNING
    - Timeout (5s) → back to IDLE, log error

  PLANNING:
    - Call /grasp_preshaping/compute_grasp service
    - On success → transition → APPROACHING
    - On failure → back to IDLE, log error

  APPROACHING:
    - Proximity controller handles near/far logic
    - Monitor distance to target
    - When near → transition → GRASPING
    - Timeout (10s) → back to IDLE

  GRASPING:
    - Activate force controller
    - When forces stable in target range → transition → HOLDING
    - Timeout (5s) → HOLDING anyway (best effort)

  HOLDING:
    - Force controller maintains grip
    - Wait for EMG release gesture (OPEN)
    - On release → transition → RELEASING

  RELEASING:
    - Open hand (zero closures)
    - After 1s → transition → IDLE
  ```
  - Publish state to `/pipeline/state` (std_msgs/String)
  - Health monitoring: watchdog on each input topic
  - All state transitions logged with timestamps

  *Rationale*: This is the missing brain. Currently nothing orchestrates the flow — nodes just run independently and hope. A simple state machine makes the pipeline deterministic and debuggable.

- [ ] **4.2** Create `src/force_controller/` — Python ROS 2 node:
  - Subscribes to finger forces from Mia Hand (`/mia_hand/forces`)
  - Subscribes to current target closures (from proximity controller)
  - Publishes adjusted closures
  - Simple regulation logic:
    - Track per-finger force over a short window
    - If force below target_min and not timed out: increment closure by small delta
    - If force above target_max: decrement closure by small delta
    - If no contact detected (zero force) after timeout: log warning, continue closing
  - Parameters from central config: target range, delta, rate
  - Activated/deactivated by pipeline manager via a service or topic

  *Rationale*: The Mia Hand driver already reads finger forces (`get_finger_forces()` at `cpp_driver.cpp:466`). Nothing closes the loop. This is essential for stable grasping.

- [ ] **4.3** Create `rviz/prosthesis.rviz`:
  - Robot model (Mia Hand URDF + joint states)
  - Point cloud display (segmented cloud)
  - Grasp target (Axes marker at target pose)
  - Pipeline state (Text display)
  - Force readings (colored markers)
  - EMG gesture (Text display)
  - Camera view panels

### Phase 5: Camera Setup (Minimal)

- [ ] **5.1** Create a minimal Jazzy-native camera node:
  - Simple launch file that starts a single Intel RealSense D435
  - Uses `ros-jazzy-realsense2-camera` (if available) or builds from source
  - Publishes `/camera/depth/color/points` (PointCloud2)
  - This feeds directly into segmentation
  - No fusion, no dual-camera complexity — that's for the future upgrade

  *Rationale*: You said the multiview upgrade is coming. So we just need a working single-camera path now. If `ros-jazzy-realsense2-camera` doesn't exist as an apt package, we can either: (a) build it from source in the Dockerfile, or (b) keep a separate Humble container just for cameras with CycloneDDS bridging (current working pattern).

### Phase 6: Launch and Integration

- [ ] **6.1** Create `launch/full_pipeline.launch.py`:
  - Loads `config/prosthesis_config.yaml`
  - Starts nodes in dependency order with appropriate delays
  - Supports `use_mock_hardware:=true` for testing
  - Supports `cameras:=false` for testing without RealSense

- [ ] **6.2** Create `launch/mock_hardware.launch.py`:
  - Publishes fake EMG gestures (cycles through POWER → REST)
  - Publishes fake point cloud (simple sphere/cylinder)
  - Publishes fake hand pose/twist
  - Allows testing the full pipeline without any hardware

- [ ] **6.3** Wire up the C++ preshaping bridge node topic names to match central config parameters. Currently hardcoded:
  - `/hand_pose` → parameterize
  - `/hand_twist` → parameterize
  - `/segmented_object_cloud` → parameterize
  - Controller command topics → parameterize

### Phase 7: Cleanup

- [ ] **7.1** Delete from old workspace (after migration verified):
  - `docker_ws/docker-deployment/` (already replaced)
  - `docker_ws/mia_hand_mujoco/` (MuJoCo simulation)
  - `docker_ws/mia_hand_moveit_config/` (MoveIt)
  - `docker_ws/DynamixelSDK/` (vendored SDK — pip replaces it)
  - `docker_ws/dev/mujoco/` (MuJoCo scenes, simulators)
  - `docker_ws/dev/pc_segmentation/MinkowskiEngine/` (source tree)
  - `docker_ws/dev/pc_segmentation/tests/`, `examples/`, `docs/`
  - `docker_ws/dev/mindrove_cpp/` (C++ experiment)
  - `docker_ws/dev/mindrove_naviflame/` (secondary gesture system)
  - `docker_ws/dev/multiview/` (old Humble setup)
  - `docker_ws/dev/hardware/` (standalone test scripts)

- [ ] **7.2** Verify the `finger_contact_lut.npz` situation. The Rust code loads it from `CARGO_MANIFEST_DIR/data/finger_contact_lut.npz` (`c_api.rs:333`). The `data/` directory appears empty in the repo. Options:
  - If it's in `.gitignore`: document how to generate it, or check in the pre-built version
  - If it's generated by `scripts/model.py`: run it and commit the output
  - This file is **critical** — the pipeline cannot run without it

---

## Verification Criteria

- [ ] `docker compose build` completes in under 8 minutes for the main image
- [ ] Single `docker-compose.yml` works on both Linux and WSL
- [ ] `docker compose up` with mock hardware launches all nodes and RViz shows the hand model
- [ ] Pipeline manager transitions through all states when receiving mock EMG + mock cloud
- [ ] Central config YAML can change any topic name or threshold without touching source code
- [ ] No MuJoCo, MoveIt, vendored DynamixelSDK, or Humble dependencies remain
- [ ] `colcon build` in the workspace succeeds with only the packages that are actually used
- [ ] Pre-built `libgrasp_preshaping.so` loads correctly via dlopen in the C++ bridge

---

## Potential Risks and Mitigations

1. **`ros-jazzy-realsense2-camera` may not exist as apt package**
   - Mitigation: Check availability first. Fallback: (a) build from source in Dockerfile, (b) keep minimal Humble container for cameras only, or (c) use `pyrealsense2` with a simple Python publisher node

2. **`finger_contact_lut.npz` missing from version control**
   - Mitigation: Generate it before migration, commit to repo. The file is likely a few MB — acceptable for Git

3. **Pre-built `.so` may not be portable across glibc versions**
   - The `.so` is built in the Rust Docker container (Debian bookworm). `osrf/ros:jazzy-desktop` is also Debian-based. Should be compatible. If not, build locally on the target machine.

4. **Force controller has no reference data**
   - Mitigation: Start with a simple bang-bang controller, test with mock force data, iterate

5. **Pipeline manager state machine edge cases**
   - Mitigation: Start with minimal states, add error handling incrementally. Use generous timeouts.

6. **Windows/WSL X11 forwarding differences**
   - Mitigation: The entrypoint script detects WSL vs Linux and sets DISPLAY accordingly. Test on both platforms early.

---

## Alternative Approaches

1. **Keep a separate `rust_build` service in compose** instead of pre-built `.so`: If you prefer not to commit binary files, keep a lightweight Rust build stage. Trade-off: slower CI, but always in sync with source.

2. **Use ROS 2 lifecycle nodes for pipeline manager**: Instead of a custom state machine, use the lifecycle node API for managed startup/shutdown. Trade-off: more standard ROS pattern, but steeper learning curve for the state machine logic.

3. **Single-container everything**: Run all nodes (including segmentation inference) in one container. Trade-off: simpler compose file, but can't use Python 3.8 for MinkowskiEngine alongside Jazzy's Python 3.12. Only viable if segmentation is replaced with a lighter alternative.

4. **Keep `dynamixel_hardware_interface/` C++ package as-is**: Instead of rewriting in Python, just migrate the existing C++ hardware interface. Trade-off: still needs the Dynamixel C++ SDK, but it's proven code. The vendored SDK tree gets replaced by `apt install ros-jazzy-dynamixel-sdk` if available, or building just the C++ SDK from source.
