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
- A **Scene Control** panel for adjusting the hand base pose, object (sphere) position, and camera position/orientation in real time, via ROS topics (`/mujoco/scene/hand_pose`, `/mujoco/scene/object_pose`, `/mujoco/scene/camera_pose`)

Logs for the grasp planner can be found under `docker_ws/dev/mujoco/log/`

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