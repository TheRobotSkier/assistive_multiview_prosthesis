# TF Tree — Grasp Test Scenario

This document describes the TF tree structure used in the grasp test
pipeline (`grasp_test.launch.py`) and how the various coordinate frames
relate to each other.

## Frame Hierarchy

```
world (fixed frame)
├── d435_1_depth_optical_frame   (stationary camera)
│       From: two_d435_launch.py — published by realsense2_camera_node
│       for the stationary D435 camera (cam1).
│
└── wrist_link                    (hand root link)
    │   From: robot_state_publisher loading mia_hand_description URDF.
    │   FK is computed from joint states (or GUI sliders when using
    │   joint_state_publisher_gui).
    │
    └── d435_2_depth_optical_frame
            From: static_transform_publisher (node: wrist_to_camera2_tf
            in grasp_test.launch.py).  This is the hand-mounted D435 #2.
            Translation & rotation must be calibrated to the physical
            mount.
```

## Frame Details

### `world`

Fixed reference frame. All transforms in the grasp pipeline are ultimately
expressed relative to `world`. RViz uses this as the fixed frame.

### `d435_1_depth_optical_frame`

Depth camera optical frame for the stationary RealSense D435 (cam1).
Published by the realsense2_camera_node under namespace `/cam1`.
The transform from `world` to this frame is published by the camera driver
(typically using `camera_color_optical_frame` as the base, with the
depth-to-color extrinsics applied internally).

### `wrist_link`

Root link of the prosthetic hand, as defined in the MIA hand URDF
(`mia_hand_description/urdf/mia_hand_description.urdf.xacro`).
The `robot_state_publisher` computes `wrist_link → world` via forward
kinematics from joint states.

**Pose published on `/hand_pose`**: The `hand_pose_publisher` node reads the
`world → wrist_link` transform from the TF tree and publishes it as a
`geometry_msgs/PoseStamped` on the `/hand_pose` topic. This is the primary
input for the twist propagation node (collision detection).

### `d435_2_depth_optical_frame`

Depth camera optical frame for the hand-mounted RealSense D435 (cam2).
This camera is rigidly attached to the wrist via a physical mount.

A `static_transform_publisher` (named `wrist_to_camera2_tf`) publishes:
```
wrist_link → d435_2_depth_optical_frame
```
with identity rotation and zero translation **by default**. These values
**must** be updated after calibrating the physical camera mount:

| Parameter | Default | Description            |
|-----------|---------|------------------------|
| x         | 0.0     | Lateral offset (m)     |
| y         | 0.0     | Vertical offset (m)    |
| z         | 0.0     | Forward offset (m)     |
| qx, qy, qz, qw | 0,0,0,1 | Identity rotation |

## Complete TF Chain

From the hand-mounted camera back to the world frame:

```
d435_2_depth_optical_frame
        ↑  (static TF, identity by default)
    wrist_link
        ↑  (robot_state_publisher, FK from joint states)
    world
        ↑  (cam1 realsense driver)
d435_1_depth_optical_frame
```

## Topic Relationships

| Topic                    | Type                        | Publisher                    | Consumer(s)            |
|--------------------------|-----------------------------|------------------------------|------------------------|
| `/hand_pose`             | `geometry_msgs/PoseStamped` | `hand_pose_publisher` or     | `twist_propagation`,   |
|                          |                             | `hand_trajectory_publisher`  | `preshaping_service`   |
| `/grasp_preshaping/      | `geometry_msgs/PoseStamped` | `preshaping_service`         | `hand_trajectory_      |
|   target_hand_pose`      |                             |                              |   publisher`           |

**Note**: Either `hand_pose_publisher` (live TF → pose) or
`hand_trajectory_publisher` (synthetic testing) publishes to `/hand_pose`.
They should not run simultaneously — the launch file selects one based on
the test scenario.
