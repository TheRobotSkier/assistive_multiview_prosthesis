# Mia Hand Base Frame — Visualization & Location Report

## Overview

This document describes how to visualize the Mia Hand URDF model in RViz2 using a Docker container (ROS 2 Jazzy on an Ubuntu 22.04 Humble host), and documents the location of the Mia Hand base frame relative to the physical hand.

## Quick Start — View the Hand in RViz

### Prerequisites

- Docker installed and running
- X11 display available (`echo $DISPLAY` should return something like `:1`)

### Build the image

```bash
cd /path/to/assistive_multiview_prosthesis
docker build -f docker/Dockerfile.mia-hand-rviz -t mia-hand-rviz .
```

### Launch

```bash
# Allow local X11 connections
xhost +local:

# Run in detached mode
docker run -d --name mia-hand-rviz --network host \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=/tmp/.xauth \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $XAUTHORITY:/tmp/.xauth:ro \
  mia-hand-rviz
```

Two windows will appear:
1. **RViz2** — 3D view of the Mia Hand with TF axes on every frame
2. **joint_state_publisher_gui** — slider panel to move each joint

### Re-launch after closing

```bash
docker start -i mia-hand-rviz
```

### Stop

```bash
docker kill mia-hand-rviz
```

### Clean up

```bash
docker rm mia-hand-rviz
docker rmi mia-hand-rviz
```

## What the Container Does

The image is defined in `docker/Dockerfile.mia-hand-rviz`. It:

1. Starts from `osrf/ros:jazzy-desktop`
2. Installs the required ROS 2 packages: `joint_limits`, `xacro`, `joint_state_publisher_gui`, `cyclonedds`
3. Copies `src/mia_hand_description/` (URDF files, meshes, launch files, RViz config)
4. Builds the `mia_hand_description` package with `colcon`
5. Launches `mia_hand_description mia_hand_rviz_launch.py` which starts:
   - `robot_state_publisher` — processes the URDF and publishes TF frames
   - `joint_state_publisher_gui` — interactive joint angle sliders (namespace `mia_hand`)
   - `rviz2_joint_state_publisher_node` — links joint limits to the GUI
   - `rviz2` — the 3D visualizer

## Understanding the RViz Display

The RViz config (`src/mia_hand_description/rviz/mia_hand_config.rviz`) is set up with:

- **Fixed Frame:** `world`
- **TF Display** — shows the full transform tree with coordinate axes on every link:
  - `world` → `palm` → `hand_ee_link`
  - `world` → `palm` → `index_fle` → `index_sensor`
  - `world` → `palm` → `middle_fle` → `middle_sensor`
  - `world` → `palm` → `ring_fle`
  - `world` → `palm` → `little_fle`
  - `world` → `palm` → `thumb_opp` → `thumb_sensor` → `thumb_fle`
- **RobotModel Display** — reads from topic `/mia_hand/robot_description`, shows the STL meshes

TF axes colors: **X = red, Y = green, Z = blue**.

## Mia Hand Base Frame Location

The base frame of the Mia Hand is the **`palm`** link. It is the root of the hand's kinematic tree (connected to `world` via a fixed joint `j_palm`).

### Physical location of the `palm` frame origin

The following table lists the offsets of physical landmarks relative to the `palm` frame origin, extracted from the URDF (`src/mia_hand_description/urdf/mia_hand_right.urdf.xacro`):

| Landmark | X (m) | Y (m) | Z (m) | Source |
|---|---|---|---|---|
| Wrist / mounting interface | -0.040 | +0.015 | -0.024 | `wrist_r.stl` mesh origin |
| Palm surface | +0.004 | +0.066 | -0.012 | `palm_simple_r.stl` mesh origin |
| Back of hand (dorsum) | +0.003 | +0.075 | -0.025 | `dorsum_simple_r.stl` mesh origin |
| Hand end-effector | 0 | +0.130 | +0.030 | `palm_to_hand_ee_link` joint |
| UR flange attachment | 0 | -0.023 | 0 | `j_ur_flange` joint origin |

### Coordinate frame orientation

- **X axis** → points to the right (toward the thumb side for a right hand)
- **Y axis** → points forward along the hand toward the fingertips
- **Z axis** → points upward, out of the back of the hand

### Summary

The `palm` frame origin is located approximately **in the center of the palm volume**, slightly offset toward the back of the hand. The physical mounting interface (wrist) is approximately **4 cm** in the **-X direction** (toward the radius/outer side) from the frame origin.

The `world` → `palm` transform is set by the fixed joint `j_palm`. In the standalone description, the palm is placed at `(0, 0, 0.2)` meters from world with a 90° roll (`rpy="1.57 0 0"` for the right hand).

## Key Files

| File | Purpose |
|---|---|
| `docker/Dockerfile.mia-hand-rviz` | Minimal Docker image for Mia Hand RViz visualization |
| `src/mia_hand_description/urdf/mia_hand_right.urdf.xacro` | Right-hand URDF with all links, joints, visuals, collisions |
| `src/mia_hand_description/urdf/mia_hand_left.urdf.xacro` | Left-hand URDF (mirror of right) |
| `src/mia_hand_description/urdf/mia_hand_description.urdf.xacro` | Standalone URDF (world → palm) for RViz viewing |
| `src/mia_hand_description/launch/mia_hand_rviz_launch.py` | Launch file for standalone RViz with joint GUI |
| `src/mia_hand_description/rviz/mia_hand_config.rviz` | RViz2 config with TF axes and RobotModel |
| `src/mia_hand_description/meshes/stl/` | STL mesh files for the hand |
| `src/mia_hand_description/calibration/joint_limits_default.yaml` | Default joint limits |

## Choosing Left vs Right Hand

The launch file defaults to the right hand. To view the left hand, launch from inside the container:

```bash
docker exec mia-hand-rviz bash -c \
  "source /mia_ws/install/setup.bash && ros2 launch mia_hand_description mia_hand_rviz_launch.py laterality:=left"
```

This will open a second set of RViz + GUI windows for the left hand.

## Troubleshooting

**RViz window doesn't appear:**
```bash
# Check X11 forwarding
echo $DISPLAY                 # should show something like :0 or :1
ls /tmp/.X11-unix             # should show a socket file like X0 or X1
xhost +local:                 # permit local X connections

# Check container logs
docker logs mia-hand-rviz
```

**OpenGL errors (`glx: failed to create dri3 screen`):**
These are NVIDIA-specific warnings. The container uses software rendering (MESA) via the `osrf/ros:jazzy-desktop` image, which works correctly despite the warnings. OpenGL 4.5 is confirmed working in the logs.

**Rebuilding after source changes:**
```bash
docker build -f docker/Dockerfile.mia-hand-rviz -t mia-hand-rviz .
```
