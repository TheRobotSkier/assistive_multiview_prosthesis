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

and replace COMMAND with the command to run (keep the quotes in). This will run the command as a bash command regardless of your shell.

THEN cd to the docker-deployment directory:

```bash
cd docker_ws/docker-deployment
```

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

**Grasp start signal** (`std_msgs/Bool`):
- `/mujoco/grasp_start` — reserved topic for triggering the autonomous grasping algorithm. Published when the simulation starts; no messages are sent yet.

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

All topics use `frame_id = "mujoco_front_depth_cam"` for front camera data and `frame_id = "mujoco_wrist_cam"` for wrist camera data.

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

THEN to build the simulation:

```bash
scene=custom docker compose run --build --rm miahand_mujoco
```

It will show a wrong simulation for some reason. So, use ctrl+c to stop the sim, type "exit" to leave the container shell, and then run the container again (without "--build"):

```bash
scene=custom docker compose run --rm miahand_mujoco
```

Now it should show the simulation with the hand and a red ball in front of it.

THEN in a different terminal (also in docker-deployment folder), to start grasp script run:

```bash
docker compose run --build --rm miahand_ros2
```

and in the shell run:

```bash
cd src/dev/grasp_preshaping && cargo run -r -- --mode ros --pointcloud-topic /segmented_object_cloud --pointcloud-scale 1.0 --iterations 1 --publish-commands --command-backend pos_ff
```

```bash
cd src/dev/grasp_preshaping && cargo run -r -- --mode ros --publish-commands --command-backend pos_ff
```

If there are any issues, try exiting the docker container, then stop all containers with

```bash
docker stop -a
```

and start your container again.


Multiview:
The multiview system presumes launch on the Nvidia Jetson, and is not containerized-- This will be harder to set up to run on your own systems.
For using the launch script in the multiview folder, change the directory path in the .sh file as: RVIZ_CONFIG.



## Daniel Notes

### Launch Usage
ros2 launch mia_hand_mujoco mia_hand_system_interface_launch.py \
  scene:=approach \
  enable_depth_publisher:=true \
  enable_approach_controller:=true

Then trigger: ros2 service call /approach_controller/start std_srvs/srv/Trigger

### Files Modified

┌───────────────────────────────────┬───────────────────────────────────────────────────┐
│ File                              │ Change                                            │
├───────────────────────────────────┼───────────────────────────────────────────────────┤
│ docker_ws/dev/mujoco/nodes/mujoco │ Added base pose tracking (subscriptions to        │
│ _scene_state_publisher_node.py:24 │ hand/object/camera pose topics, body ID lookup,   │
│ 6-428                             │ _apply_base_poses_unlocked())                     │
├───────────────────────────────────┼───────────────────────────────────────────────────┤
│ docker_ws/dev/mujoco/interactive_ │ Added TwistCovarianceMode enum and member         │
│ simulator/interactive_system_inte │ variables                                         │
│ rface.hpp:171-180                 │                                                   │
├───────────────────────────────────┼───────────────────────────────────────────────────┤
│ docker_ws/dev/mujoco/interactive_ │ Parameter reading for twist covariance in         │
│ simulator/interactive_system_inte │ on_activate                                       │
│ rface.cpp:182-202                 │                                                   │
├───────────────────────────────────┼───────────────────────────────────────────────────┤
│ docker_ws/dev/mujoco/interactive_ │ Velocity-scaled covariance computation replacing  │
│ simulator/interactive_system_inte │ hardcoded 1e-6                                    │
│ rface.cpp:450-477                 │                                                   │
├───────────────────────────────────┼───────────────────────────────────────────────────┤
│ docker_ws/mia_hand_mujoco/CMakeLi │ Added approach_controller_node.py to install      │
│ sts.txt:242                       │                                                   │
├───────────────────────────────────┼───────────────────────────────────────────────────┤
│ docker_ws/mia_hand_mujoco/launch/ │ Added approach scene choice,                      │
│ mia_hand_system_interface_launch. │ enable_approach_controller launch arg, approach   │
│ py                                │ controller node                                   │
└───────────────────────────────────┴───────────────────────────────────────────────────┘

### Files Created

┌─────────────────────────────┬─────────────────────────────────────────────────────────┐
│ File                        │ Purpose                                                 │
├─────────────────────────────┼─────────────────────────────────────────────────────────┤
│ docker_ws/dev/mujoco/scenes │ Approach scene with head-mounted camera at (-0.1,       │
│ /scene_right_approach.xml   │ 0.25, 0.55) looking down                                │
├─────────────────────────────┼─────────────────────────────────────────────────────────┤
│ docker_ws/dev/mujoco/nodes/ │ Approach-and-grasp workflow node (state machine: IDLE   │
│ approach_controller_node.py │ -> APPROACHING -> PRESHAPING_TRIGGERED -> GRASP_READY)  │
└─────────────────────────────┴─────────────────────────────────────────────────────────┘
