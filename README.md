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