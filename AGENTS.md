# Agent Instructions for multiview_prosthesis Workspace

## Golden Rules

1. **Always run code inside Docker containers.** Never run ROS 2, MuJoCo, Rust, or build commands on the host. Use the appropriate Docker Compose service for every execution.
2. **Minimise changes to `mia_hand_*` packages.** These are upstream/shared packages. If you must modify them, preserve the original default behaviour by adding command-line flags, environment variables, or launch arguments. Do not alter default behaviour without an explicit opt-in mechanism.
3. **Keep new code in the `dev/` folder.** New scripts, nodes, scenes, and utilities belong under `docker-ws/dev/`. New MuJoCo simulation scenes/assets go in `docker-ws/dev/mujoco/`.
4. **Separate new behaviour from defaults.** When adding features that touch existing packages, gate them behind flags (e.g. launch argument, env var, CLI flag) so that running the service without extra arguments produces the original behaviour.

---

## Repository Layout

```
multiview_prosthesis/
├── docker-ws/                  # Everything mounted into the container at /miahand_ws/src
│   ├── dev/                    # Custom / new code lives here
│   │   ├── grasp_preshaping/   # Rust grasp-planning crate
│   │   └── mujoco/             # Custom MuJoCo scenes & assets
│   ├── docker-deployment/      # Dockerfile, docker-compose.yml, .env, entrypoint
│   ├── mia_hand_description/   # URDF, meshes, RViz config (upstream — avoid edits)
│   ├── mia_hand_driver/        # ROS 2 Mia Hand driver (upstream — avoid edits)
│   ├── mia_hand_moveit_config/ # MoveIt 2 config (upstream — avoid edits)
│   ├── mia_hand_msgs/          # Custom messages/services (upstream — avoid edits)
│   ├── mia_hand_mujoco/        # MuJoCo ros2_control plugin (upstream — avoid edits)
│   ├── mia_hand_ros2_control/  # ros2_control hardware interface (upstream — avoid edits)
│   └── docker_ws/     # Meta-package
└── multiview/                  # Multi-camera launch scripts (runs on Jetson, not Docker)
```

---

## Docker Setup

### Prerequisites (one-time, on host)

```bash
# From the repo root:
cd docker-ws/docker-deployment

# 1. Create .env with host UID/GID (required for volume mount permissions)
echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > .env
```

### Working Directory

All `docker compose` commands must be run from:
```
docker-ws/docker-deployment/
```

---

## Docker Compose Services

| Service | Container Name | Purpose |
|---|---|---|
| `miahand_ros2` | `miahand_ros2` | **Default.** Interactive ROS 2 Jazzy shell with all dependencies. Use for general development, building, and running custom code. |
| `miahand_mujoco` | `miahand_ros2_mujoco` | MuJoCo simulation with ros2_control. Use for any MuJoCo-related work. |
| `miahand_description` | `miahand_ros2_rviz` | RViz2 URDF visualisation. |
| `miahand_driver` | `miahand_ros2_driver` | Mia Hand hardware driver. |
| `miahand_moveit` | `miahand_ros2_moveit` | MoveIt 2 planning & execution. |
| `miahand_ros2_control` | `miahand_ros2_control` | ros2_control without MuJoCo. |

All services extend `miahand_ros2` and inherit its volumes, network, and environment. The workspace source tree (`docker-ws/`) is bind-mounted at `/miahand_ws/src` inside the container.

### Choosing the Right Service

- **General ROS 2 work, building packages, running scripts** → `miahand_ros2`
- **MuJoCo simulation, depth-publisher, sim-related testing** → `miahand_mujoco`
- **Visualising URDF / meshes** → `miahand_description`
- **Motion planning with MoveIt 2** → `miahand_moveit`
- **When unsure** → `miahand_ros2` (it is the base service with all deps)

---

## Running Containers

### Start a service (interactive shell)

```bash
docker compose run --rm <service_name>
```

### Build and start (first time or after Dockerfile changes)

```bash
docker compose run --build --rm <service_name>
```

### Start MuJoCo simulation

```bash
# Default custom scene with depth publisher:
docker compose run --rm miahand_mujoco

# Override scene or depth publisher via env vars:
MUJOCO_SCENE=custom ENABLE_DEPTH_PUBLISHER=true docker compose run --rm miahand_mujoco
```

### Open additional terminals into a running container

```bash
docker exec -it <container_name> bash
# Example:
docker exec -it miahand_ros2_mujoco bash
```

Inside the container, always source the workspace before running ROS commands:
```bash
source install/setup.bash
```

---

## Building Code Inside Containers

### Rebuild all ROS 2 packages

```bash
# Inside the container (working directory is /miahand_ws):
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### Build a specific package

```bash
colcon build --packages-select <package_name>
source install/setup.bash
```

### Build Rust code (grasp_preshaping)

```bash
# Inside the container:
cd src/dev/grasp_preshaping
cargo build --release
```

Run the Rust grasp planner:
```bash
cargo run -r -- --mode ros --publish-commands --command-backend pos_ff
```

---

## Environment Details

- **ROS 2 distro:** Jazzy
- **DDS implementation:** CycloneDDS (`rmw_cyclonedds_cpp`)
- **MuJoCo version:** 3.3.4 (installed at `/opt/mujoco/mujoco-3.3.4`)
- **Python:** 3.12 (system Python inside the container)
- **Rust:** installed system-wide via rustup
- **Container OS:** Ubuntu (ros:jazzy-ros-base)
- **Network mode:** host (ROS 2 topics are visible across all containers on the same host)
- **Discovery:** `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`

---

## Code Organisation Rules

### Where to put new code

| What | Where |
|---|---|
| New ROS 2 nodes, scripts, launch files | `docker-ws/dev/` |
| New MuJoCo scenes, assets, XMLs | `docker-ws/dev/mujoco/` |
| Rust grasp-planning code | `docker-ws/dev/grasp_preshaping/` |
| Modifications to existing `mia_hand_*` packages | **Avoid.** If unavoidable, gate behind a flag. |

### Modifying upstream packages (`mia_hand_*`)

If a change to a `mia_hand_*` package is truly necessary:

1. **Add a launch argument / env var / CLI flag** that defaults to the original behaviour.
2. **Guard the new code path** so running the service without the flag produces identical results to before.
3. **Document the flag** in the relevant launch file or README.
4. Prefer adding a new launch file in `dev/` that includes/wraps the original launch, rather than editing the original.

### Example: adding a custom MuJoCo scene

```
docker-ws/dev/mujoco/scenes/my_new_scene.xml   # Scene file
docker-ws/dev/mujoco/assets/my_object.stl       # Assets
```
Reference it via the `MUJOCO_SCENE` env var or a new launch argument.


