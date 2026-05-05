# Implementation Plan: Step 2 — Digital Twin Launch File

**Beads Issue:** `launch-digital-twin`  
**Type:** task  
**Priority:** 1 (blocked by `rviz-config`; blocks `docker-service`)  
**Estimated Effort:** Medium

---

## Objective

Create `dev/mujoco/launch/digital_twin_launch.py` — a single ROS2 launch file that orchestrates the entire digital twin pipeline: simulated Mia Hand, segmentation, click relay, grasp planning, grasp proximity control, and RViz.

---

## Acceptance Criteria

- [ ] Launch file exists at `dev/mujoco/launch/digital_twin_launch.py`.
- [ ] Launch file is valid Python and passes `ros2 launch --show-args` syntax checks.
- [ ] All required nodes are declared with appropriate `TimerAction` delays where needed.
- [ ] Launch arguments exist for:
  - `camera_world_{x,y,z,qx,qy,qz,qw}` — static TF from `world` to camera optical frame.
  - `use_trajectory` — optional flag to include the hand trajectory test node.
  - `inference_url` — URL of the segmentation inference server.
  - `segmentation_cubeedge` — click cube half-width.
- [ ] No hard-coded absolute paths that break when the workspace is moved.
- [ ] Launch file can be invoked standalone:
  ```bash
  ros2 launch dev/mujoco/launch/digital_twin_launch.py
  ```

---

## Node Inventory

| # | Node / Launch | Package / Source | Purpose | Start Delay |
|---|---------------|------------------|---------|-------------|
| 1 | `mia_hand_system_interface_launch.py` | `mia_hand_mujoco` | MuJoCo simulation + ROS2 Control (digital twin) | 0 s |
| 2 | `static_transform_publisher` | `tf2_ros` | `world → cam1_d435_1_color_optical_frame` | 0 s |
| 3 | `topic relay` or `relay_node.py` | `topic_tools` or custom | `/fused_pointcloud` → `/segmentation/input_cloud` | 2 s |
| 4 | `segmentation_ros2_node.py` | `pc_segmentation` | Segmentation ROS bridge | 3 s |
| 5 | `demo_click_relay_node.py` | `pc_segmentation` | RViz click → segmentation clicks | 3 s |
| 6 | `preshaping_service_bridge_node` | `grasp_preshaping` | Grasp planner service | 5 s |
| 7 | `grasp_proximity_controller_node.py` | `grasp_preshaping` | Proximity-based grasp execution | 5 s |
| 8 | `rviz2` | `rviz2` | Visualization | 8 s |
| 9 | *(optional)* `mujoco_hand_trajectory_node.py` | `mia_hand_mujoco` | Automated hand approach test | 15 s |

---

## Implementation Details

### 2.1 Mia Hand Simulation Launch

Include `mia_hand_mujoco` launch with these arguments:
```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('mia_hand_mujoco'), 'launch',
            'mia_hand_system_interface_launch.py'
        ])
    ]),
    launch_arguments={
        'hardware_plugin': 'mia_hand_mujoco/InteractiveSystemInterface',
        'enable_depth_publisher': 'false',
        'enable_preshaping_service': 'false',
        'scene': 'static',
        'include_wrist': 'true',
        'depth_publish_tf': 'false',
    }.items()
)
```

**Rationale:**
- `InteractiveSystemInterface` is required because it publishes `/hand_pose` and `/hand_twist`, which the grasp planner and proximity controller consume.
- `enable_depth_publisher:=false` because we use the real camera pointcloud, not MuJoCo's simulated depth camera.
- `enable_preshaping_service:=false` because we launch our own `preshaping_service_bridge_node` explicitly (gives us control over parameters like `camera_frames`).

### 2.2 Static TF Publisher

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        LaunchConfiguration('camera_world_x'),
        LaunchConfiguration('camera_world_y'),
        LaunchConfiguration('camera_world_z'),
        LaunchConfiguration('camera_world_qx'),
        LaunchConfiguration('camera_world_qy'),
        LaunchConfiguration('camera_world_qz'),
        LaunchConfiguration('camera_world_qw'),
        'world',
        'cam1_d435_1_color_optical_frame',
    ],
    name='camera_world_tf',
)
```

**Note:** If the multiview system also publishes a `cam1_d435_1_link` → `cam1_d435_1_color_optical_frame` TF (RealSense driver does), then publishing `world → cam1_d435_1_color_optical_frame` is sufficient. If the user has two cameras, add a second static TF argument set for `cam2_d435_2_color_optical_frame`.

### 2.3 Pointcloud Relay

Option A — `topic_tools relay` (preferred, minimal code):
```python
Node(
    package='topic_tools',
    executable='relay',
    arguments=['/fused_pointcloud', '/segmentation/input_cloud'],
    name='pc_relay',
    condition=IfCondition(use_relay),  # optional
)
```

Option B — custom Python relay node (fallback if `topic_tools` not available):
A 20-line `rclpy` node that subscribes to `/fused_pointcloud` and republishes on `/segmentation/input_cloud`.

**Decision:** Try Option A first. If `topic_tools` is not in the base image (`ros:jazzy-ros-base`), install it or fall back to Option B.

### 2.4 Segmentation Node

```python
Node(
    executable='python3',
    arguments=['/miahand_ws/src/dev/pc_segmentation/nodes/segmentation_ros2_node.py'],
    name='segmentation_node',
    parameters=[{
        'cubeedge': LaunchConfiguration('segmentation_cubeedge'),
        'inference_url': LaunchConfiguration('inference_url'),
    }],
    output='screen',
)
```

### 2.5 Click Relay

```python
Node(
    executable='python3',
    arguments=['/miahand_ws/src/dev/pc_segmentation/nodes/demo_click_relay_node.py'],
    name='click_relay',
    output='screen',
)
```

### 2.6 Grasp Preshaping Service Bridge

```python
Node(
    package='grasp_preshaping',
    executable='preshaping_service_bridge_node',
    name='preshaping_service_bridge',
    parameters=[{
        'camera_frames': ['cam1_d435_1_color_optical_frame', 'cam2_d435_2_color_optical_frame'],
        'preshaping_closure_fraction': 0.3,
        'min_closure_amount': 0.1,
    }],
    output='screen',
)
```

**Important:** The bridge subscribes to `/hand_pose`, `/hand_twist`, and `/segmented_object_cloud`. It will only compute a grasp when the service `/grasp_preshaping/compute_grasp` is called.

### 2.7 Grasp Proximity Controller

```python
Node(
    executable='python3',
    arguments=['/miahand_ws/src/dev/grasp_preshaping/nodes/grasp_proximity_controller_node.py'],
    name='grasp_proximity_controller',
    parameters=[{
        'proximity_enter_threshold_m': 0.08,
        'proximity_exit_threshold_m': 0.10,
        'partial_closure_factor': 0.3,
        'min_closure_amount': 0.1,
        'control_rate_hz': 10.0,
    }],
    output='screen',
)
```

### 2.8 RViz

```python
TimerAction(period=8.0, actions=[
    Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', '/miahand_ws/src/dev/mujoco/config/digital_twin.rviz'],
        output='screen',
    )
])
```

### 2.9 Optional Hand Trajectory Test Node

```python
Node(
    executable='python3',
    arguments=['/miahand_ws/src/dev/mujoco/nodes/mujoco_hand_trajectory_node.py'],
    name='hand_trajectory_test',
    output='screen',
    condition=IfCondition(LaunchConfiguration('use_trajectory')),
)
```

This node moves the simulated hand from far to near, automatically exercising the proximity controller. Disabled by default.

---

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `camera_world_x` | `0.0` | Camera optical frame X in world |
| `camera_world_y` | `0.0` | Camera optical frame Y in world |
| `camera_world_z` | `1.0` | Camera optical frame Z in world |
| `camera_world_qx` | `0.0` | Camera optical frame quaternion x |
| `camera_world_qy` | `0.0` | Camera optical frame quaternion y |
| `camera_world_qz` | `0.0` | Camera optical frame quaternion z |
| `camera_world_qw` | `1.0` | Camera optical frame quaternion w |
| `use_trajectory` | `false` | Include automated hand approach test |
| `inference_url` | `http://127.0.0.1:5678` | Segmentation inference server URL |
| `segmentation_cubeedge` | `0.05` | Click cube half-width (m) |

---

## Sub-tasks (beads tracking)

1. Scaffold `digital_twin_launch.py` with imports and `generate_launch_description()`.
2. Add Mia Hand simulation include with correct arguments.
3. Add static TF publisher with launch arguments.
4. Add pointcloud relay (topic_tools or custom).
5. Add segmentation, click relay, preshaping bridge, proximity controller nodes.
6. Add RViz node with delayed start.
7. Add optional trajectory node with condition.
8. Declare all launch arguments.
9. Syntax-check with `ros2 launch --show-args`.
10. Commit.

---

## Notes / Risks

- **Risk:** `topic_tools` may not be installed in the base image.
  - **Mitigation:** Check if available; if not, install `ros-jazzy-topic-tools` in Dockerfile or use a custom 20-line relay node.
- **Risk:** `InteractiveSystemInterface` may conflict with ros2_control joint commands if not designed to run alongside them.
  - **Mitigation:** Test that joint commands from the proximity controller still reach the simulated fingers when using `InteractiveSystemInterface`. If not, we may need to use `SystemInterface` and add a small node that publishes `/hand_pose` from TF or MuJoCo body state.
- **Risk:** The preshaping bridge publishes immediate preshape commands to `/*/pos_ff_controller/commands`, which may race with the proximity controller publishing to the same topics.
  - **Mitigation:** This is acceptable for the first version — the bridge sends one-shot preshape on service call, then the proximity controller takes over at 10 Hz. If conflicts cause jitter, we can later parameterise the bridge to skip immediate publishing.
