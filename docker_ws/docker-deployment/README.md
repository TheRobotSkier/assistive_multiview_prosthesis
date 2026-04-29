# Docker container installation
The **mia_hand_ros2_pkgs** packages use `Docker Compose` to simplify the use of the Mia Hand packages in **ROS 2**. 
Each component is containerized, to ensure environment consistency and easy setup.

## Docker Compose services
All the services defined in the compose files can be retrieved by running:
```bash
cd docker-deployment
docker compose config --services
```
An overview of the services is reported below:

* `miahand_ros2`: the core `ROS2` workspace, with all the necessary dependencies.
* `miahand_description`: visualize the Mia Hand URDF with meshes in `RViz2`.
* `miahand_driver`: launch the generic Mia Hand `ROS2` driver.
* `miahand_moveit`: planning and execution of motions with `Moveit2` by using a simulated, mock, or real hardware interface.
* `miahand_mujoco`: simulate Mia Hand in `MuJoCo` and control the simulated hand through the `ROS2 Control` framework.
* `miahand_ros2_control`: control Mia Hand through the `ROS2 Control` framework.
* `rust_build`: **one-shot** — compiles `libgrasp_preshaping.so` from the Rust grasp-preshaping crate. Run before any simulation service that uses the grasp planner.

### Using Docker Compose

**Saving the current user id into a file:**
```bash
cd docker-deployment
echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > .env
```
It is needed to mount the folder from the Docker container.

**Running the services:**
To launch a specific service, the `docker compose run` command should be used. 
This will start an interactive session associated with that service.
```bash
docker compose run --rm <name_of_service>
```
#### Examples
Launch the `miahand_ros2` service:
```bash
docker compose run --rm miahand_ros2
```
Launch the `miahand_description` service:
```bash
docker compose run --rm miahand_description
```
Launch the `miahand_driver` service:
```bash
docker compose run --rm miahand_driver
```
Launch the `miahand_moveit` service:
```bash
USE_MUJOCO_SIM=true USE_MOCK_HARDWARE=false docker compose run --rm miahand_moveit
```
Launch the `miahand_mujoco` service:
```bash
docker compose run --rm miahand_mujoco 
```
Launch the `miahand_ros2_control` service:
```bash
USE_MOCK_HARDWARE=false docker compose run --rm miahand_ros2_control
```

#### Rust grasp-preshaping build (one-time / on Rust source change)

The grasp-preshaping Rust library (`libgrasp_preshaping.so`) is built in its own container and
**must be built before** starting any simulation service that uses the planner
(`mujoco_dynamic`, `mujoco_interactive`, `mujoco_trajectory`).

```bash
# First time, or after changing docker_ws/dev/grasp_preshaping/src/**
docker compose run --rm rust_build
```

The compiled `.so` is written to `docker_ws/dev/grasp_preshaping/target/release/` on the host and
loaded at runtime by the bridge node via `dlopen`. The main ROS 2 image does not need Rust
installed.

### Graphic User Interface (GUI) Support
For the services that require a GUI (`RViz2` or `MuJoCo`), the Docker access to the X server should be granted.
The easiest way to allow the current user to connect to the X server is through the following command:
```bash
xhost +si:localuser:$(whoami)
```
This command can eventually be automated, by adding it to `.bashrc`.
By this way, the command will run automatically whenever a new terminal session is started. This can be done through the following command:
```bash
echo "xhost +si:localuser:$(whoami) > /dev/null 2>&1" >> ~/.bashrc
```

---

## Classical EMG Gesture Classifier

Three services for the MindRove armband EMG pipeline. Connect to the armband's WiFi before running any of them. Data and trained models are persisted in `../dev/mindrove/data/` and `../dev/mindrove/models/` via bind mounts.

| Service | Purpose |
|---|---|
| `mindrove_emg_collect` | Interactive gesture recording (saves `.npz` to `data/`) |
| `mindrove_emg_train` | Offline training — loads all `.npz` files, trains LDA classifier, saves `.pkl` models |
| `mindrove_emg_run` | Real-time inference — streams from board, prints gesture + confidence + proportional control |

**Typical workflow:**

```bash
# 1. Record gestures (guided, interactive)
docker compose run --rm mindrove_emg_collect

# 2. Train the classifier
docker compose run --rm mindrove_emg_train

# 3. Run live
docker compose run --rm mindrove_emg_run
```

See `../dev/mindrove/README.md` for the full pipeline description and tuning options.
