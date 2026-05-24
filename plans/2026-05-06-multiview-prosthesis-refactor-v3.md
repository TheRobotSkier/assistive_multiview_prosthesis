# Comprehensive Refactor: Multiview Prosthesis Architecture (v3)

## Objective

Refactor the multiview prosthesis project into a lean, flat ROS 2 Jazzy workspace with a single Docker Compose file, a single base image (`osrf/ros:jazzy-desktop`), a central YAML config, and RViz-only visualization. The grasp pipeline flows: EMG gesture → segmentation → preshaping → proximity execution → force control → gesture release.

---

## Design Decisions

1. **Base image**: `osrf/ros:jazzy-desktop` — includes RViz, rqt, ros2-control, xacro, tf2-ros, desktop tools
2. **Docker**: Delete `docker-deployment/` entirely. One new `docker/` folder with a single `docker-compose.yml` (unified Linux + WSL), one `Dockerfile`, one `.env`
3. **Workspace layout**: Flat `src/` workspace — no `dev/` nesting
4. **Rust `.so`**: Pre-compiled, checked into repo. No Rust toolchain in Docker
5. **Multiview**: Minimal single-camera setup for now (upgrade coming later)
6. **No MuJoCo, no MoveIt**: RViz for visualization only
7. **Entrypoint**: No fake serial devices. Real device mapping in compose, or mock hardware mode
8. **Services**: Minimal — main container + segmentation inference + optional camera container

---

## MinkowskiEngine Clarification

MinkowskiEngine is a **sparse tensor library from NVIDIA** used by the InterObject3D segmentation network. It requires Python 3.8 + PyTorch 1.12 + C++ compilation. This is fundamentally incompatible with ROS Jazzy (Python 3.12).

**We do NOT remove MinkowskiEngine.** The current two-container architecture is correct:

1. `segmentation` container: Python 3.8 + MinkowskiEngine + Flask inference server (`inference_server.py`)
2. Main container: ROS Jazzy bridge node (`segmentation_ros2_node.py`) that talks to Flask over HTTP

What we DO remove from the workspace is the **MinkowskiEngine source tree** (`dev/pc_segmentation/MinkowskiEngine/`, `tests/`, `examples/`, `docs/`, `setup.py`). This source code only exists inside the segmentation Docker image — it's built during `docker build` and never needed in the ROS workspace. The ROS side only needs `segmentation_ros2_node.py` and `inference_server.py`.

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
│   └── .env.example                     # template for user-specific vars
├── launch/
│   ├── pipeline.launch.py               # main orchestration
│   └── mock.launch.py                   # testing without hardware
├── rviz/
│   └── prosthesis.rviz
└── src/
    ├── mia_hand_description/            # URDF + meshes (as-is)
    ├── mia_hand_driver/                 # C++ serial driver (as-is)
    ├── mia_hand_msgs/                   # Custom messages (as-is)
    ├── mia_hand_ros2_control/           # Hardware interface (as-is)
    ├── grasp_preshaping/                # Rust lib (.so) + C++ bridge + proximity ctrl
    ├── emg_bridge/                      # MindRove classifier + ROS bridge
    ├── segmentation/                    # ROS2 bridge node + inference server script
    ├── force_controller/                # NEW: force regulation
    ├── pipeline_manager/               # NEW: state machine
    ├── haptic_band/                     # BT bridge + haptic controller
    └── wrist_driver/                   # Dynamixel wrist motor interface
```

---

## Implementation Plan

### Phase 1: Docker — Clean Slate

- [ ] **1.1** Delete `docker_ws/docker-deployment/` entirely

- [ ] **1.2** Create `docker/Dockerfile` based on `osrf/ros:jazzy-desktop`:
  - Additional apt: `libserial-dev`, `python3-pip`, `usbutils`
  - Additional pip: `dynamixel-sdk`, `requests`
  - Everything else (rviz2, ros2-control, xacro, tf2-ros, cyclonedds) already in desktop image
  - User setup (UID/GID args), dialout group membership
  - Copy workspace source → `rosdep install` → `colcon build`
  - NO MuJoCo, NO MoveIt, NO Rust toolchain, NO GLFW, NO fake serial entrypoint

- [ ] **1.3** Create `docker/Dockerfile.segmentation` — essentially the existing `dev/pc_segmentation/docker/Dockerfile.cpu`:
  - `python:3.8-slim-bullseye` base
  - PyTorch 1.12 + MinkowskiEngine CPU build
  - InterObject3D clone + Flask server
  - Weights download on first run
  - This is the only container that ever touches MinkowskiEngine

- [ ] **1.4** Create `docker/docker-compose.yml` — minimal services:
  ```yaml
  services:
    prosthesis:
      build: { context: .., dockerfile: docker/Dockerfile }
      # Main container: all ROS nodes + RViz
      # Profiles: default = mock hardware, [hardware] = real devices

    segmentation:
      build: { context: ../src/segmentation, dockerfile: ../../docker/Dockerfile.segmentation }
      # Python 3.8 inference server only
  ```
  - **No fake serial entrypoint**. Instead:
    - Mock mode (default): `use_mock_hardware:=true` in launch, no device mapping
    - Hardware mode (`--profile hardware`): `devices: ["/dev/ttyUSB0:/dev/ttyUSB0", ...]`
  - X11 forwarding: single mount `/tmp/.X11-unix` + DISPLAY env var (works on both Linux and WSL2)
  - All containers: `network_mode: host` + `RMW_IMPLEMENTATION: rmw_cyclonedds_cpp`
  - No platform-specific compose files

- [ ] **1.5** Create `docker/.env.example`:
  ```
  USER_UID=1000
  USER_GID=1000
  DISPLAY=:0
  MIA_SERIAL_PORT=/dev/ttyUSB1
  WRIST_PORT=/dev/ttyUSB0
  HAPTIC_BT_ADDR1=84:2E:14:09:E1:4E
  ```

  *Rationale*: The current setup has 22 services, most of which are variants of the same image. We need 2: the main ROS container and the segmentation inference server. Mock vs hardware is handled by compose profiles, not by different services.

### Phase 2: Flat Workspace Migration

- [ ] **2.1** Create `src/` directory structure at repo root

- [ ] **2.2** Migrate `mia_hand_description/` from `docker_ws/mia_hand_description/` — copy as-is. URDF xacros, .obj meshes, joint state publisher, RViz config.

- [ ] **2.3** Migrate `mia_hand_driver/` from `docker_ws/mia_hand_driver/` — copy as-is. C++ serial driver (~1340 lines), complete Mia Hand protocol.

- [ ] **2.4** Migrate `mia_hand_msgs/` from `docker_ws/mia_hand_msgs/` — copy as-is. Custom message definitions.

- [ ] **2.5** Migrate `mia_hand_ros2_control/` from `docker_ws/mia_hand_ros2_control/` — copy as-is. Hardware interface wrapping Mia driver.

- [ ] **2.6** Migrate `grasp_preshaping/`:
  - **Rust source** (`src/*.rs`): Copy all including `superquadric.rs`. For local dev only
  - **Pre-built `.so`**: Build locally (`cargo build --release --lib`), place in `grasp_preshaping/lib/libgrasp_preshaping.so`, commit to repo
  - **Data**: Ensure `finger_contact_lut.npz` is in `grasp_preshaping/data/`. If missing from current branch, check other branches or regenerate via `scripts/model.py`
  - **C++ bridge**: Copy `nodes/preshaping_service_bridge_node.cpp` — update hardcoded topic strings to ROS parameters
  - **FFI types**: Copy `include/grasp_preshaping/ffi_types.hpp` (already in sync with API v5)
  - **Python proximity controller**: Copy `nodes/grasp_proximity_controller_node.py` — update topic names to parameterized
  - **CMakeLists.txt**: Simplify — no cargo detection, just install pre-built `.so` from `lib/`
  - **Cargo.toml**: Keep for local development reference

- [ ] **2.7** Create `emg_bridge/` from `docker_ws/dev/mindrove/`:
  - Copy `emg_classifier/` (board_reader, features, classifier, preprocessing, proportional, config, ros_bridge_node)
  - Copy `scripts/` (collect_data, train, run_classifier)
  - Create `package.xml` + `setup.py` (pure Python package)
  - Update topic names to parameterized

- [ ] **2.8** Create `segmentation/` from `docker_ws/dev/pc_segmentation/`:
  - Copy `nodes/segmentation_ros2_node.py` — the ROS bridge
  - Copy `nodes/inference_server.py` — the Flask server (referenced by Dockerfile.segmentation)
  - Copy `docker/Dockerfile.cpu` → referenced as `../../docker/Dockerfile.segmentation`
  - Copy `docker/entrypoint.inference.sh`
  - Copy `weights/` directory reference (weights downloaded at runtime)
  - **Do NOT copy**: `MinkowskiEngine/`, `tests/`, `examples/`, `docs/`, `setup.py` (MinkowskiEngine build), `src/` (C++ source)
  - Create `package.xml` for the ROS bridge node
  - The MinkowskiEngine source is only needed inside the Docker image build context — it gets COPY'd in during `docker build` and compiled there

- [ ] **2.9** Migrate `haptic_band/` from `docker_ws/dev/haptic_band/`:
  - Copy `haptic_bridge/` (bridge_node.py, haptic_controller_node.py, setup.py)
  - Create `package.xml`

- [ ] **2.10** Create `wrist_driver/`:
  - If current `dynamixel_hardware_interface/` works: migrate as-is
  - Otherwise: lightweight Python node using `dynamixel-sdk` pip package
  - Do NOT bring the vendored `DynamixelSDK/` tree (~300+ files) — only the pip package

### Phase 3: Central Configuration

- [ ] **3.1** Create `config/prosthesis_config.yaml` with sections:
  - `topics:` — all ROS topic names
  - `emg:` — gesture mapping, confidence threshold
  - `segmentation:` — inference URL, cubeedge
  - `preshaping:` — mirrors config.rs defaults (overridable)
  - `execution:` — proximity thresholds, partial closure factor, control rate
  - `force:` — target force range, gains, update rate
  - `hardware:` — serial ports, BT addresses
  - `frames:` — TF frame names

- [ ] **3.2** Update each node to load parameters from this file via the launch file's `parameters:` argument

### Phase 4: New Components

- [ ] **4.1** Create `src/pipeline_manager/` — Python ROS 2 state machine:
  ```
  IDLE → SEGMENTING → PLANNING → APPROACHING → GRASPING → HOLDING → RELEASING → IDLE
  ```
  - EMG gesture triggers transitions (POWER/PINCH → start, OPEN → release)
  - Publishes state to `/pipeline/state`
  - Health monitoring on each input topic
  - All transitions logged with timestamps

- [ ] **4.2** Create `src/force_controller/` — Python ROS 2 node:
  - Subscribes to Mia Hand finger forces
  - Subscribes to target closures from proximity controller
  - Publishes adjusted closures (simple PI regulation)
  - Activated/deactivated by pipeline manager

- [ ] **4.3** Create `rviz/prosthesis.rviz` — hand model + point cloud + grasp target + state display

### Phase 5: Camera (Minimal)

- [ ] **5.1** Create minimal single-camera setup:
  - Check if `ros-jazzy-realsense2-camera` exists as apt package
  - If yes: install in Dockerfile, launch in main container
  - If no: either build from source in Dockerfile, or use `pyrealsense2` with a simple Python publisher
  - No fusion, no dual-camera — just get a point cloud to segmentation

### Phase 6: Launch and Integration

- [ ] **6.1** Create `launch/pipeline.launch.py`:
  - Loads `config/prosthesis_config.yaml`
  - Starts all nodes in dependency order
  - Supports `use_mock_hardware:=true`
  - Supports `cameras:=false`

- [ ] **6.2** Create `launch/mock.launch.py`:
  - Fake EMG, fake point cloud, fake hand pose
  - Full pipeline test without hardware

- [ ] **6.3** Wire up C++ bridge node topic names to parameters

### Phase 7: Cleanup

- [ ] **7.1** Delete from old workspace after migration verified:
  - `docker_ws/docker-deployment/`
  - `docker_ws/mia_hand_mujoco/`
  - `docker_ws/mia_hand_moveit_config/`
  - `docker_ws/DynamixelSDK/`
  - `docker_ws/dev/mujoco/`
  - `docker_ws/dev/pc_segmentation/MinkowskiEngine/`, `tests/`, `examples/`, `docs/`
  - `docker_ws/dev/mindrove_cpp/`
  - `docker_ws/dev/mindrove_naviflame/`
  - `docker_ws/dev/multiview/`
  - `docker_ws/dev/hardware/`

- [ ] **7.2** Resolve `finger_contact_lut.npz` — must exist in `grasp_preshaping/data/` before pipeline can run

---

## Verification Criteria

- [ ] `docker compose build` completes in under 8 minutes
- [ ] Single `docker-compose.yml` works on Linux and WSL
- [ ] `docker compose up` with mock hardware: all nodes start, RViz shows hand model
- [ ] Pipeline manager transitions through states with mock data
- [ ] Central config changes take effect without source code changes
- [ ] No MuJoCo, MoveIt, vendored DynamixelSDK, or Humble dependencies
- [ ] `colcon build` succeeds with only needed packages
- [ ] Pre-built `libgrasp_preshaping.so` loads in C++ bridge

---

## Potential Risks and Mitigations

1. **`ros-jazzy-realsense2-camera` may not exist as apt package**
   - Mitigation: Check first. Fallback: build from source, or use pyrealsense2 Python publisher

2. **`finger_contact_lut.npz` missing from repo**
   - Mitigation: Check other branches, regenerate if needed, commit to repo

3. **Pre-built `.so` glibc compatibility**
   - Mitigation: Build on same Debian version as Docker base image

4. **Force controller has no reference data**
   - Mitigation: Start with simple bang-bang, test with mock forces first

5. **WSL X11 differences**
   - Mitigation: Test early. WSL2 with WSLg should work with standard X11 mount

---

## Alternative Approaches

1. **Keep `rust_build` compose service** instead of pre-built `.so` if you prefer not to commit binaries

2. **Keep `dynamixel_hardware_interface/` C++ as-is** instead of Python rewrite — proven code

3. **Single-container everything** — only viable if segmentation is replaced with a lighter alternative (not recommended now)


## User input

Alright I changed my mind a bit about the rust component. The thing is i dont mind building it, if it doesnt need to cahgne, but we might change the config, so i kinda want to hear your opinion, do you think it would be viable to rewite, so we can read from a text file like yaml? then we can still cahnge things when the library is built. I am curious if you see that as a viable path.

Also another idea i have is to make a bit of some helper scripts to fx test that things are working, a bit like a CI/CD pipline, but for local testing, and making sure things are okay, to quickly catch issues. It is not something comprehensive, mose like something minimal, that ensures basic functionality is there, while running withouth human intervention.

Also do we need the multiview folder? like it seems to just be an example, but i might be wrong.

Also i think it is high time for a rewrite of the central Readme file, to reflect the new architecture, and also to be more clear about how to get started, and how to use the project. I think it would be good to have a clear and concise README that explains the project, the architecture, the components, and how to get started with it. It should also include some troubleshooting tips, and maybe a FAQ section.