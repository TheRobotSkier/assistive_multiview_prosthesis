# Multiview Prosthesis

A ROS 2 Jazzy system for EMG-controlled robotic hand grasping with real-time point cloud segmentation and grasp preshaping.

## Architecture

```
                          ┌─────────────┐
                          │  MindRove   │
                          │  EMG Band   │
                          └──────┬──────┘
                                 │ gesture trigger
                                 ▼
┌──────────┐    cloud     ┌──────────────┐    segmented     ┌──────────────────┐
│  Camera  │─────────────▶│ Segmentation │────────────────▶│ Grasp Preshaping │
│  (D435)  │              │  (Minkowski) │                  │  (Rust pipeline) │
└──────────┘              └──────┬───────┘                  └────────┬─────────┘
     │                           │                                   │
     │ cloud                     │ click_positive                    │ preshape + wrist + hand pose
     ▼                           ▼                                   ▼
┌──────────────────┐  ┌──────────────────┐                 ┌──────────────────┐
│ Twist Propagation │  │ Pipeline Manager │                 │ Pipeline Manager │
│  (collision det.) │  │  (state machine) │                 │  (state machine) │
└──────────────────┘  └────────┬─────────┘                 └────────┬─────────┘
                               │                                    │
                                          ┌────────────────────┼────────────────────┐
                                          ▼                   ▼                    ▼
                                  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
                                  │ Wrist Driver │   │  Mia Hand    │   │   Force      │
                                  │ (Dynamixel)  │   │  Driver      │   │  Controller  │
                                  └──────────────┘   └──────────────┘   └──────────────┘
```

**Pipeline flow:**
1. User activates a grasp pattern via MindRove EMG band
2. Point cloud segmentation isolates the grasping objective
3. Grasp preshaping pipeline computes preshape, wrist pose, and hand grasp position
4. Pipeline manager orchestrates approach → grasp → hold states
5. Force controller regulates contact force during grasp
6. User releases via EMG gesture

## Quick Start

### Prerequisites

- Docker + Docker Compose
- X11 display (Linux or WSL2 with WSLg)

### Setup

```bash
# Clone and configure
git clone <repo-url>
cd multiview_prosthesis

# Set user IDs for Docker
echo -e "USER_UID=$(id -u)\nUSER_GID=$(id -g)" > docker/.env
```

### Build

```bash
# Build the main ROS image
docker compose build prosthesis

# Build the segmentation image (separate, Python 3.8 + MinkowskiEngine)
docker compose build segmentation
```

### Run

```bash
# Mock mode (no hardware, publishes fake point clouds)
docker compose up

# Hardware mode (maps USB devices)
docker compose --profile hardware up

# Run segmentation alongside
docker compose up segmentation
```

### Inside the Container

```bash
# Launch the full pipeline
ros2 launch prosthesis_launch pipeline.launch.py

# Or launch with mock data (no camera/hardware needed)
ros2 launch prosthesis_launch mock.launch.py

# Open RViz manually
rviz2 -d /prosthesis_ws/rviz/prosthesis.rviz
```

## Configuration

### Central Config

All parameters live in `config/prosthesis_config.yaml` — topic names, thresholds, state machine transitions, and force controller tuning. Edit this file and restart the container.

### Grasp Preshaping Config

The Rust grasp preshaping pipeline reads from `config/grasp_preshaping.yaml` at runtime. This file contains all TSDF, SMC, scoring, and superquadric parameters. You can tune these without rebuilding the `.so` — just edit the YAML and restart.

Config resolution order:
1. `GRASP_CONFIG_PATH` environment variable
2. `config/grasp_preshaping.yaml` next to the workspace
3. Compile-time defaults (in `src/grasp_preshaping/src/runtime_config.rs`)

## Packages

| Package | Language | Description |
|---------|----------|-------------|
| `mia_hand_description` | URDF/xacro | Hand model + meshes for RViz |
| `mia_hand_driver` | C++ | Serial communication with Mia Hand hardware |
| `mia_hand_msgs` | ROS msgs | Custom message/service/action definitions |
| `mia_hand_ros2_control` | C++ | ros2-control hardware interface for Mia Hand |
| `grasp_preshaping` | Rust/C++ | Grasp planning pipeline (TSDF + SMC optimization) |
| `pipeline_manager` | Python | State machine: IDLE -> SEGMENTING -> APPROACHING -> GRASPING -> HOLDING -> RELEASING |
| `twist_propagation` | Python | Predicts hand trajectory from pose stream, detects collisions with point cloud |
| `force_controller` | Python | Force regulation for stable grasping |
| `segmentation_bridge` | Python | ROS node bridging to segmentation inference server |
| `emg_bridge` | Python | MindRove EMG classifier + ROS bridge |
| `haptic_band` | Python | Bluetooth haptic armband driver |
| `wrist_driver` | Python | Dynamixel wrist motor driver |
| `camera` | Python/launch | Minimal RealSense D435 launch |
| `prosthesis_launch` | Python/launch | Top-level launch files |

## Docker Services

| Service | Image | Purpose |
|---------|-------|---------|
| `prosthesis` | `osrf/ros:jazzy-desktop` | Main ROS container (all nodes + RViz) |
| `prosthesis-hw` | same | Hardware variant with USB device mapping |
| `segmentation` | `python:3.8-slim` | MinkowskiEngine inference server (isolated) |
| `test` | same as prosthesis | Runs smoke tests and exits |

## Network Setup (Jetson Orin Nano)

The robotlab Jetson connects to a dedicated WiFi hotspot (`robotlab-wifi`) with a static IP
of `10.42.0.2`. Run the appropriate script for your OS to create the hotspot:

| OS | Script | How to run |
|----|--------|------------|
| **Linux** (native) | `scripts/setup_robotlab_wifi_linux.sh` | `sudo ./scripts/setup_robotlab_wifi_linux.sh <wifi_iface>` |
| **Windows** (native, any WiFi adapter) | `scripts/setup_robotlab_wifi.ps1` | `powershell -ExecutionPolicy Bypass -File setup_robotlab_wifi.ps1` (as Admin) |

**Common settings:**
- SSID: `robotlab-wifi`
- Password: `labrobot123`
- Hotspot IP: `10.42.0.1/24`
- Robotlab IP: `10.42.0.2/24` (static)
- Internet sharing via NAT (robotlab reaches internet through your machine)

Once the hotspot is active, SSH into robotlab:
```bash
ssh robotlab@10.42.0.2
```

## Testing

Automated smoke tests run inside Docker (no host ROS installation needed):

```bash
docker compose run --rm test
```

This runs:
- **Build check** — `colcon build` from scratch
- **Launch syntax** — all `.launch.py` files parse without errors
- **Preshaping .so** — library loads via ctypes, API version check
- **Node startup** — key nodes start and register with ROS

## Rebuilding the Rust Library

The grasp preshaping `.so` is pre-built and committed to `src/grasp_preshaping/lib/`. If you modify the Rust source:

```bash
cd src/grasp_preshaping
cargo build --release --lib
cp target/release/libgrasp_preshaping.so lib/
```

## Project Structure

```
multiview_prosthesis/
├── config/
│   ├── prosthesis_config.yaml       # Central config (topics, thresholds)
│   └── grasp_preshaping.yaml        # Rust pipeline parameters
├── docker/
│   ├── Dockerfile                    # Main ROS Jazzy image
│   ├── Dockerfile.segmentation       # Python 3.8 + MinkowskiEngine
│   ├── docker-compose.yml            # All services
│   └── .env.example                  # User-local overrides
├── scripts/
│   ├── run_tests.sh                  # Test orchestrator
│   ├── test_build.sh
│   ├── test_launch_syntax.sh
│   ├── test_preshaping_so.sh
│   └── test_nodes_start.sh
├── rviz/
│   └── prosthesis.rviz               # RViz visualization config
├── src/
│   ├── camera/                       # RealSense D435 launch
│   ├── emg_bridge/                   # MindRove EMG classifier
│   ├── force_controller/             # Force regulation
│   ├── grasp_preshaping/             # Rust pipeline + C++ bridge
│   ├── haptic_band/                  # BT haptic armband
│   ├── mia_hand_description/         # URDF + meshes
│   ├── mia_hand_driver/              # Hardware driver
│   ├── mia_hand_msgs/                # ROS message definitions
│   ├── mia_hand_ros2_control/        # ros2-control interface
│   ├── pipeline_manager/             # State machine orchestrator
│   ├── prosthesis_launch/            # Top-level launch files
│   ├── segmentation/                 # Segmentation + MinkowskiEngine
│   ├── twist_propagation/            # Hand trajectory prediction + collision detection
│   └── wrist_driver/                 # Dynamixel wrist driver
└── plans/                            # Architecture and planning docs
```

## Troubleshooting

**Docker build fails on `rosdep install`:**
Make sure all `package.xml` files have correct dependencies. Check `scripts/test_build.sh` output for the specific package that failed.

**RViz shows nothing:**
- Check that the container has X11 access: `echo $DISPLAY` inside the container should show `:0`
- On WSL2, ensure WSLg is enabled
- Try `xhost +local:docker` on the host

**Mia Hand driver can't find serial port:**
- Check the device exists: `ls /dev/ttyUSB*`
- Verify the device mapping in `docker/.env` matches your system
- Ensure the user is in the `dialout` group

**Segmentation inference server won't start:**
- Build the segmentation image separately: `docker compose build segmentation`
- First run downloads model weights (~200 MB)

**Grasp preshaping crashes with "finger_contact_lut.npz not found":**
- The file should be at `src/grasp_preshaping/data/finger_contact_lut.npz`
- If missing, regenerate from `src/grasp_preshaping/scripts/model.py`

## Development

### Adding a New Node

1. Create a package under `src/your_package/`
2. Add a `package.xml` and build system (`setup.py` for Python, `CMakeLists.txt` for C++)
3. Rebuild: `docker compose build prosthesis`

### Modifying Grasp Parameters

Edit `config/grasp_preshaping.yaml` — no rebuild needed. The Rust library reads this file at runtime on first pipeline invocation.

### Modifying Pipeline Parameters

Edit `config/prosthesis_config.yaml` — topic names, thresholds, and state machine parameters. Restart the container to apply.

## EMG Gesture Pipeline

The MindRove EMG classifier supports 5 gestures: REST, POWER, PINCH, OPEN, POINT.

```bash
# Inside the container — record training data
ros2 run emg_bridge collect_data

# Train the classifier
ros2 run emg_bridge train

# Run live inference
ros2 run emg_bridge run_classifier
```

Connect to the MindRove WiFi network first. The classifier publishes to `/emg/gesture_label`, `/emg/gesture_name`, `/emg/confidence`, and `/emg/proportional`.

## Twist Propagation

The twist propagation node predicts the hand's future trajectory from a stream of hand poses and detects collisions with the point cloud. When a collision is predicted, it publishes a click point and triggers grasp preshaping.

### How it works

1. Subscribes to hand poses (`/hand_pose`) and the live point cloud (`/camera/depth/color/points`)
2. Estimates the hand's twist (linear + angular velocity) from recent poses using an EMA-smoothed least-squares fit
3. Propagates the hand pose forward in time (up to `propagation_time_horizon_s` seconds) in small steps (`propagation_dt_s`)
4. At each step, checks if the propagated hand position is within the collision threshold of any point in the cloud
5. On collision: publishes a click point (`/segmentation/click_positive`), waits for a segmented cloud, then calls the grasp preshaping service
6. Publishes the estimated twist, predicted path, collision spheres, and hit markers for RViz visualization

### Launch commands

```bash
# Standalone (with mock data sources)
ros2 launch prosthesis_launch twist_propagation_test.launch.py

# Standalone (no mock data, expects external pose/cloud publishers)
ros2 launch twist_propagation twist_propagation.launch.py

# As part of the full pipeline
ros2 launch prosthesis_launch pipeline.launch.py
```

### Start/Stop control

The node starts **inactive** by default. Activate it at runtime via ROS services:

```bash
# Activate
ros2 service call /twist_propagation/activate std_srvs/srv/Trigger

# Deactivate
ros2 service call /twist_propagation/deactivate std_srvs/srv/Trigger

# Check current state
ros2 topic echo /twist_propagation/status --once
```

Or launch with `active:=true` to start active immediately:

```bash
ros2 launch prosthesis_launch twist_propagation_test.launch.py active:=true
```

### Key parameters

Parameters are configured in `src/twist_propagation/config/twist_propagation.yaml` and `config/prosthesis_config.yaml`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `propagation_time_horizon_s` | 2.0 | How far into the future to predict (seconds) |
| `propagation_dt_s` | 0.02 | Time step for each propagation step |
| `hit_threshold_m` | 0.05 | Base distance threshold for collision detection |
| `collision_geometry_radius_m` | 0.05 | Added to `hit_threshold_m` for effective collision radius |
| `cycle_delay_s` | 0.1 | Time between propagation cycles |
| `twist_estimation_window` | 5 | Number of recent poses used to estimate velocity |
| `cloud_max_age_s` | 2.0 | Cloud is rejected if older than this |
| `voxel_leaf_m` | 0.02 | Voxel size for cloud downsampling (0 to disable) |
| `enable_covariance_propagation` | true | Truncate horizon when uncertainty exceeds threshold |

### Topics

**Inputs:**
- `/hand_pose` (geometry_msgs/PoseStamped) -- tracked hand position
- `/camera/depth/color/points` (sensor_msgs/PointCloud2) -- live depth cloud
- `/segmentation/object_cloud` (sensor_msgs/PointCloud2) -- segmented object cloud

**Outputs:**
- `/hand_twist` (geometry_msgs/TwistStamped) -- estimated hand velocity
- `/segmentation/click_positive` (geometry_msgs/PointStamped) -- predicted collision point
- `/twist_propagation/status` (std_msgs/String) -- JSON status (active, state, reason)

**Visualization (RViz):**
- `/twist_propagation/predicted_path` (nav_msgs/Path)
- `/twist_propagation/collision_spheres` (visualization_msgs/MarkerArray)
- `/twist_propagation/hit_marker` (visualization_msgs/MarkerArray)
- `/twist_propagation/trajectory_line` (visualization_msgs/MarkerArray)

### Testing

```bash
# Unit tests (pure functions, no ROS needed)
python3 -m pytest src/twist_propagation/test/test_twist_propagation.py -v

# Integration test (requires node running in a separate terminal)
ros2 launch prosthesis_launch twist_propagation_test.launch.py  # terminal 1
python3 scripts/test_twist_propagation_integration.py            # terminal 2
```

## Daniel Scratchpad

### How to fix GPU rendering in WSL2

The real fix is at the Windows/WSL2 level, not in your container config:

1. Update your Windows GPU driver -- Download the latest NVIDIA driver for Windows from
   nvidia.com (https://www.nvidia.com/drivers). Your current driver (580.97) may not be properly exposing D3D12/Vulkan to
   WSL2.

2. Update WSL2 -- Run this in PowerShell:
   wsl --update
   Then restart WSL: wsl --shutdown

1. After the update, verify on the WSL2 host that ls /dev/dri/ shows renderD128 (or similar). If it does, GPU-accelerated
   OpenGL will work automatically and the container will inherit it.

2. If /dev/dri/ still doesn't appear after driver/WSL updates, you may need to add this to C:\Users\jonat\.wslconfig:
   [wsl2]
   networkingMode=mirrored
   firewall=false
   Then wsl --shutdown and relaunch. Some WSL2 builds require explicit GPU support to be enabled.

Once /dev/dri/ exists on the host, I can add back --device /dev/dri to the Makefile targets and RViz will use GPU rendering through Mesa's d3d12 driver.
