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
  - `use_trajectory` — optional flag to include the hand trajectory test node.
  - `inference_url` — URL of the segmentation inference server.
  - `segmentation_cubeedge` — click cube half-width.
  - `publish_initial_commands` — whether the preshaping bridge publishes immediate joint commands (default `true`; digital twin sets `false`).
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
| 2 | `topic relay` or `relay_node.py` | `topic_tools` or custom | `/fused_pointcloud` → `/segmentation/input_cloud` | 2 s |
| 3 | `segmentation_ros2_node.py` | `pc_segmentation` | Segmentation ROS bridge | 3 s |
| 4 | `demo_click_relay_node.py` | `pc_segmentation` | RViz click → segmentation clicks | 3 s |
| 5 | `preshaping_service_bridge_node` | `grasp_preshaping` | Grasp planner service | 5 s |
| 6 | `grasp_proximity_controller_node.py` | `grasp_preshaping` | Proximity-based grasp execution (sends initial preshape + runs control loop) | 5 s |
| 7 | `rviz2` | `rviz2` | Visualization | 8 s |
| 8 | *(optional)* `mujoco_hand_trajectory_node.py` | `mia_hand_mujoco` | Automated hand approach test | 15 s |

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

### 2.2 Pointcloud Relay

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

### 2.3 Segmentation Node

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
        'publish_initial_commands': LaunchConfiguration('publish_initial_commands'),
    }],
    output='screen',
)
```

**Important:** 
- The bridge subscribes to `/hand_pose`, `/hand_twist`, and `/segmented_object_cloud`. It will only compute a grasp when the service `/grasp_preshaping/compute_grasp` is called.
- When `publish_initial_commands:=false`, the bridge skips sending the immediate preshape to `*_pos_ff_controller/commands`. Instead, the proximity controller sends the initial preshape when it commits the plan.

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

**Key behavior:** When the proximity controller commits a new plan (all three planner topics received), it **immediately** publishes:
1. The wrist rotation command (`/wrist/set_position`).
2. The partial finger closure (preshape) to `*_pos_ff_controller/commands`.

Then, on each control loop tick, it evaluates distance and transitions between far (partial) and near (full) modes.

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
| `use_trajectory` | `false` | Include automated hand approach test |
| `inference_url` | `http://127.0.0.1:5678` | Segmentation inference server URL |
| `segmentation_cubeedge` | `0.05` | Click cube half-width (m) |
| `publish_initial_commands` | `true` | If `true`, the preshaping bridge sends immediate joint commands on service call. Digital twin sets this to `false` so the proximity controller owns all controller commands. |

---

## Sub-tasks (beads tracking)

1. Scaffold `digital_twin_launch.py` with imports and `generate_launch_description()`.
2. Add Mia Hand simulation include with correct arguments.
3. Add pointcloud relay (topic_tools or custom).
4. Add segmentation, click relay, preshaping bridge, proximity controller nodes.
5. Add RViz node with delayed start.
6. Add optional trajectory node with condition.
7. Declare all launch arguments.
8. Syntax-check with `ros2 launch --show-args`.
9. Commit.

---

## Notes / Risks

- **Risk:** `topic_tools` may not be installed in the base image.
  - **Mitigation:** Check if available; if not, install `ros-jazzy-topic-tools` in Dockerfile or use a custom 20-line relay node.
- **Risk:** `InteractiveSystemInterface` may conflict with ros2_control joint commands if not designed to run alongside them.
  - **Mitigation:** Test that joint commands from the proximity controller still reach the simulated fingers when using `InteractiveSystemInterface`. If not, we may need to use `SystemInterface` and add a small node that publishes `/hand_pose` from TF or MuJoCo body state.
- **Risk:** The preshaping bridge and proximity controller both publish to `/*/pos_ff_controller/commands`.
  - **Mitigation:** The bridge is parameterised with `publish_initial_commands:=false` in the digital twin launch. The proximity controller owns all controller command publishing. The bridge still publishes planner topics (`/grasp_preshaping/*`) consumed by the controller.
