To set up docker:

FIRST:
Check the docker-compose.yml, and comment out lines that have comments "#added LINUX" if you are not on a linux system. For reference, the compose should for Arch linux x86-64 with Wayland. I'm not sure if there are differences with ubuntu. The changes made are related to wayland/x11 functionality, so ubuntu Wayland should work the same, but no clue if WSL even includes x11/wayland.

From the main directory (docker_miniproject):
THEN RUN:

```bash
echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > docker_ws/docker-deployment/.env
```

For Linux Wayland, also run:

```bash
echo "XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}" >> docker_ws/docker-deployment/.env
```

If you are using Podman Desktop on WSL/WSLg, also run:

```bash
echo "X11_SOCKET_DIR=/mnt/wslg/.X11-unix" >> docker_ws/docker-deployment/.env
```

Linux note: if not using bash shell, use

```bash
bash -c 'COMMAND'
```

# Select your compose file (system specific)

`compose.yaml` auto-discovers the correct platform compose file via the
`PLATFORM_COMPOSE` variable in `.env`. The default is `docker-compose.linux-podman.yml`.
For Windows/WSL, set:

```bash
echo "PLATFORM_COMPOSE=docker-compose.windows.yml" >> docker_ws/docker-deployment/.env
```

THEN cd to the docker-deployment directory:

```bash
cd docker_ws/docker-deployment
```

### Build the Rust grasp-preshaping library (once, or on Rust source change)

Before starting the simulation for the first time, build the Rust `.so`:

```bash
docker compose run --rm rust_build
```

This compiles `libgrasp_preshaping.so` and places it in
`docker_ws/dev/grasp_preshaping/target/release/` on the host. Re-run this step
whenever you change files under `docker_ws/dev/grasp_preshaping/src/`.

### For the interactive simulation (recommended):

Run

```bash
docker compose run --build --rm mujoco_interactive
```

#### Environment Variables

| Variable | Values | Default | Description |
|---|---|---|---|
| `MUJOCO_OBJECT` | `sphere`, `cylinder` | `sphere` | Target object type. Selects the scene XML and point-cloud target geom. |
| `MUJOCO_PC_MODE` | `object`, `full` | `object` | Point cloud mode. `object` publishes only points from the target geom; `full` publishes the full scene. |

**Examples:**
```bash
MUJOCO_OBJECT=cylinder MUJOCO_PC_MODE=full docker compose run --rm mujoco_interactive
```

This launches an interactive MuJoCo simulation based on the upstream MuJoCo `simulate` viewer. It includes:
- A full MuJoCo GUI with physics controls, rendering options, and joint/actuator sliders
- A **Grasp Planner** panel for triggering the grasping pipeline and selecting grasp modes
- A **Scene Control** panel for instantly repositioning the hand base, object (sphere), and depth camera by typing position/orientation values; changes are also available via ROS topics (`/mujoco/set_hand_pose`, `/mujoco/set_object_pose`, `/mujoco/set_camera_pose`)
- A **Motion Control** panel for smoothly interpolating the hand, object, or camera to a target pose over a set duration (in seconds); each entity has its own target position/RPY fields and a *Move* button — also available via ROS topics (see below)
- The built-in **Rendering** panel (left sidebar) contains a *Camera* dropdown listing all cameras in the scene — select the depth camera entry to switch the viewport to the depth camera's point of view
- A realistic **Intel RealSense D435** mesh is shown as the camera body (decimated to ~180 k faces for MuJoCo compatibility)

Logs for the grasp planner can be found under `docker_ws/dev/mujoco/log/`

#### ROS topics for scripted motion control

The simulation exposes the following ROS2 topics for external control from Python scripts or the terminal.

**Instant teleport** (`geometry_msgs/Pose`, orientation as quaternion xyzw):
- `/mujoco/set_hand_pose` — immediately move the hand base to the given pose
- `/mujoco/set_object_pose` — immediately move the object (sphere)
- `/mujoco/set_camera_pose` — immediately move the depth camera body

**Smooth interpolated motion** (`geometry_msgs/PoseStamped`, orientation as quaternion xyzw):
- `/mujoco/move_hand` — smoothly move the hand to the given pose
- `/mujoco/move_object` — smoothly move the object
- `/mujoco/move_camera` — smoothly move the camera

For the motion topics the **duration in seconds** is encoded in `header.stamp` (i.e. `stamp.sec + stamp.nanosec / 1e9`). If the stamp is zero, a default of 1.0 s is used.

**Current poses** are published at ~10 Hz on:
- `/mujoco/hand_pose`, `/mujoco/object_pose`, `/mujoco/camera_pose`

**Simulation time** (`std_msgs/Float64`, ~10 Hz):
- `/mujoco/sim_time` — MuJoCo simulation time in seconds. Use this with the pose topics to compute velocities (`Δpos / Δt`).

**IMU — front camera** (`depth_cam_body`, clean/noiseless):
- `/mujoco/front_cam/imu` (`sensor_msgs/Imu`, ~100 Hz) — angular velocity, linear acceleration, and orientation in front camera frame. Linear acceleration includes gravity correction (at rest reads ≈ +9.81 m/s² upward in camera frame).
- `/mujoco/front_cam/imu/magnetic_field` (`sensor_msgs/MagneticField`, ~100 Hz) — simulated magnetometer. World X+ projected into front camera frame (arbitrary "north").

**IMU — wrist camera** (`wrist_cam_body`, fixed to back of hand, clean/noiseless):
- `/mujoco/wrist_cam/imu` (`sensor_msgs/Imu`, ~100 Hz) — same convention as front camera IMU, but in wrist camera frame.
- `/mujoco/wrist_cam/imu/magnetic_field` (`sensor_msgs/MagneticField`, ~100 Hz) — magnetometer in wrist camera frame.

**Point cloud / depth** (published by the depth pipeline, not ros2_control):
- `/mujoco/depth/image` — raw depth image
- `/mujoco/depth/camera_info` — camera intrinsics
- `/segmented_object_cloud` — segmented object point cloud (`sensor_msgs/PointCloud2`)

All topics use `frame_id = "mujoco_front_depth_cam"` for front camera data and `frame_id = "mujoco_camera_wrist_cam"` for wrist camera data.

**Terminal example** — read the current front camera IMU data once:

```bash
ros2 topic echo --once /mujoco/front_cam/imu
```

**Python example** — read sim_time and hand pose to compute hand velocity:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64

class VelocityEstimator(Node):
    def __init__(self):
        super().__init__('vel_estimator')
        self.last_pos  = None
        self.last_time = None
        self.create_subscription(Pose,    '/mujoco/hand_pose', self.on_pose, 10)
        self.create_subscription(Float64, '/mujoco/sim_time',  self.on_time, 10)

    def on_time(self, msg):
        self.last_time = msg.data

    def on_pose(self, msg):
        pos = (msg.position.x, msg.position.y, msg.position.z)
        if self.last_pos and self.last_time:
            dt = self.last_time - getattr(self, '_prev_t', self.last_time)
            if dt > 0:
                vel = tuple((pos[i] - self.last_pos[i]) / dt for i in range(3))
                self.get_logger().info(f'hand velocity: {vel}')
        self._prev_t  = self.last_time
        self.last_pos = pos

rclpy.init()
node = VelocityEstimator()
rclpy.spin(node)
```

### ASGER: I have not actually tested these examples! I did not have time. But, the motion control UI is tested and works perfectly.

**Terminal example** — move the hand to (x=0.0, y=0.1, z=0.3) over 2 seconds:

```bash
ros2 topic pub --once /mujoco/move_hand geometry_msgs/msg/PoseStamped \
  "{header: {stamp: {sec: 2, nanosec: 0}}, pose: {position: {x: 0.0, y: 0.1, z: 0.3}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}"
```

**Python example** — move the hand along a short trajectory:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

rclpy.init()
node = Node('motion_sender')
pub = node.create_publisher(PoseStamped, '/mujoco/move_hand', 10)

waypoints = [
    (0.0,  0.0, 0.2),
    (0.0,  0.1, 0.3),
    (0.05, 0.0, 0.25),
]

for x, y, z in waypoints:
    msg = PoseStamped()
    msg.header.stamp.sec = 2      # 2-second move duration
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0  # identity rotation
    pub.publish(msg)
    time.sleep(2.5)               # wait for motion to complete before next waypoint

node.destroy_node()
rclpy.shutdown()
```

A cylinder variant of the dynamic scene is available at
`docker_ws/dev/mujoco/scenes/scene_right_cylinder.xml`. It is identical
to the dynamic scene except the target object is a red upright cylinder
(radius 0.03 m, height 0.12 m) rather than a sphere.

### For the dynamic simulation:

Run

```bash
docker compose run --build --rm mujoco_dynamic
```

This will launch the dynamic simulation, and you should have a GUI for selecting the grasp planning parameters (and executing the grasp planning). Logs for the grasp planner can be found under docker_ws/dev/mujoco/log/

### For the old simulation:

The legacy standalone CLI (`cargo run -- --mode ros`) has been removed. The grasp preshaping pipeline is now integrated as a ROS 2 service node that is launched automatically with the interactive or dynamic simulation. Use the interactive simulation (recommended) or the dynamic simulation described above.

**Multiview**:
The multiview system presumes launch on the Nvidia Jetson, and is not containerized-- This will be harder to set up to run on your own systems.
For using the launch script in the multiview folder, change the directory path in the .sh file as: RVIZ_CONFIG.

---

## Classical EMG Gesture Classifier (MindRove armband)

A three-step pipeline in `docker_ws/dev/mindrove/` that recognises 4 hand gestures + REST from the MindRove WiFi armband and outputs a proportional control value. All steps run inside Docker (connect to the armband's WiFi first, then run from `docker_ws/docker-deployment/`).

### Step 1 — Record training data

```bash
docker compose run --rm mindrove_emg_collect
```

Guides you through recording each gesture interactively (default: 3 reps × 5 s each). Saves raw EMG to `docker_ws/dev/mindrove/data/`.

### Step 2 — Train the classifier

```bash
docker compose run --rm mindrove_emg_train
```

Filters + windows the recordings, extracts features (MAV, RMS, WL, ZC, SSC, VAR × 8 channels), trains an LDA classifier with 5-fold cross-validation, and saves models to `docker_ws/dev/mindrove/models/`.

### Step 3 — Run live inference

```bash
docker compose run --rm mindrove_emg_run
```

Streams from the armband at ~10 Hz and prints the recognised gesture, confidence, and proportional control value to the terminal.

See `docker_ws/dev/mindrove/README.md` for the full pipeline overview, file layout, and tuning options.

---

## Haptic Band (Vibro8 BT bridge)

A ROS 2 Jazzy Bluetooth bridge for a **Vibro8** 8-motor haptic armband, running in its own Docker container.

### Start the bridge

```bash
docker compose run --build --rm haptic_band
```

Connects to the Vibro8 over Bluetooth Classic (RFCOMM), sends a brief buzz-buzz on first connect, then listens on:

```
/haptic_band/motors   std_msgs/Float32MultiArray   [8 values, 0.0–100.0]
```

Publish a message to that topic from any other ROS 2 node or container to drive the 8 motors.

### Run the motor sweep test

```bash
docker compose run --rm haptic_band_test
```

Activates each of the 8 motors one at a time, ramping 0 → 100 % in 10 % steps (~1 s), then turns off before moving to the next motor.

See `docker_ws/dev/haptic_band/README.md` for the full protocol details, address configuration, and file layout.

---

## Pointcloud Segmentation (InterObject3D)

Interactive 3D object segmentation using the [InterObject3D](https://github.com/theodorakontogianni/InterObject3D) network. Click on an object in RViz2 to segment it from the scene. Runs in two separate containers: a Python 3.8 inference server (MinkowskiEngine) + a ROS 2 Jazzy bridge node.

### Quick start

From `docker_ws/docker-deployment/`:

```bash
# Build images (once, or after code changes):
docker compose build segmentation_inference
docker compose build miahand_ros2

# Start inference server + ROS2 node:
./run_segmentation.sh
```

Press `Ctrl+C` to stop both containers.

### Interactive demo (with RViz2)

In a second terminal while segmentation is running:

```bash
docker compose run --rm segmentation_demo
```

In the terminal that opens: `p` = positive mode | `n` = negative mode | `r` = reset | `q` = quit.
Use the **Publish Point** tool in RViz2 to click on an object — segmented points appear as `/segmentation/object_cloud`.

### One-shot inference test (no ROS)

```bash
docker compose run --rm segmentation_direct
```

Downloads weights on first run, then runs a single inference to verify the MinkowskiEngine image.

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/segmentation/input_cloud` | `PointCloud2` | Input scene cloud |
| `/segmentation/click_positive` | `PointStamped` | Positive (foreground) click |
| `/segmentation/click_negative` | `PointStamped` | Negative (background) click |
| `/segmentation/reset` | `Empty` | Clear all clicks and output |
| `/segmentation/object_cloud` | `PointCloud2` | Segmented foreground points |

---

## Full System Test

Launch the complete system (EMG + haptics + cameras + segmentation + MuJoCo mirror) with Docker profiles:

```bash
# From docker_ws/docker-deployment/
docker compose --profile full_system up
```

This starts: `mindrove_emg_ros`, `multiview_full`, `segmentation_inference`, `segmentation_ros2`, and `full_system_test` (RViz + MuJoCo mirror + haptic controller). Hardware must be connected (Mia Hand on USB, haptic band via BT, D435 cameras on USB). See `docker_ws/dev/mujoco/launch/full_system_test_launch.py` for the full launch configuration.
