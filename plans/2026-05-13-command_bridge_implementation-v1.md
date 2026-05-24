# Command Bridge Node — Forwarding `*_pos_ff_controller/commands` to Raw Driver

## Objective

Create a lightweight Python bridge node that subscribes to the ros2_control-style position command topics (`/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands`) and forwards them to the raw Mia Hand driver's `joints/*/set_trajectory` services. This keeps the raw driver as the sole hardware interface (preserving force data streaming) while making the proximity controller and force controller work without changes.

Additionally, the bridge will publish `/joint_states` by subscribing to the raw driver's `data_streams/joints/positions/data` topic, so the force controller can track current joint positions.

## Background

### Why a bridge?

The pipeline has two incompatible halves:
- **Proximity controller + Force controller** publish to `*_pos_ff_controller/commands` (ros2_control convention)
- **Raw Mia Hand driver** exposes `joints/*/set_trajectory` services + force data streams

They cannot both open the serial port. The bridge lets the raw driver own the hardware while translating the command topics into service calls.

### Data flow after the bridge

```
Proximity Controller ──► /thumb_pos_ff_controller/commands ──► Bridge ──► joints/thumb/set_trajectory
Force Controller     ──► /index_pos_ff_controller/commands  ──► Bridge ──► joints/index/set_trajectory
                     ──► /mrl_pos_ff_controller/commands    ──► Bridge ──► joints/mrl/set_trajectory

Raw Driver ──► data_streams/joints/positions/data ──► Bridge ──► /joint_states
Raw Driver ──► data_streams/fingers/forces/data   ──► Force Controller (unchanged)
```

## Implementation Plan

### Step 1: Create the bridge node Python file

- [ ] **1.1** Create `src/command_bridge/command_bridge/command_bridge_node.py`
  - Subscribes to three `std_msgs/Float64MultiArray` topics: `/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands`
  - For each message received, calls the corresponding `mia_hand_msgs/srv/SetJointTraj` service:
    - `joints/thumb/set_trajectory`, `joints/index/set_trajectory`, `joints/mrl/set_trajectory`
  - Maps `Float64MultiArray.data[0]` → `SetJointTraj.Request.target_angle` (radians)
  - Uses a configurable `spe_for_percent` parameter (default: 80) for trajectory speed
  - Uses `call_async()` to avoid blocking — the bridge should not stall if a service call is slow
  - Logs warnings on service call failure but does not crash

- [ ] **1.2** Add `/joint_states` publishing to the bridge
  - Subscribes to `data_streams/joints/positions/data` (`mia_hand_msgs/JointData`)
  - Converts to `sensor_msgs/JointState` with joint names `["j_thumb_fle", "j_index_fle", "j_mrl_fle"]`
  - Publishes on `/joint_states` at whatever rate the raw driver provides
  - This is needed by the force controller (`src/force_controller/force_controller/force_controller_node.py:298-306`) which reads joint positions to compute incremental adjustments

- [ ] **1.3** Make all topic/service names configurable via parameters
  - `thumb_cmd_topic` (default: `/thumb_pos_ff_controller/commands`)
  - `index_cmd_topic` (default: `/index_pos_ff_controller/commands`)
  - `mrl_cmd_topic` (default: `/mrl_pos_ff_controller/commands`)
  - `thumb_traj_service` (default: `joints/thumb/set_trajectory`)
  - `index_traj_service` (default: `joints/index/set_trajectory`)
  - `mrl_traj_service` (default: `joints/mrl/set_trajectory`)
  - `joint_positions_topic` (default: `data_streams/joints/positions/data`)
  - `joint_states_topic` (default: `/joint_states`)
  - `default_speed_percent` (default: 80)
  - Also activate the joint positions data stream on startup by calling `data_streams/joints/positions/switch` with `True`

### Step 2: Package the bridge as a ROS2 ament_python package

- [ ] **2.1** Create `src/command_bridge/package.xml`
  - Dependencies: `rclpy`, `std_msgs`, `sensor_msgs`, `mia_hand_msgs`, `std_srvs`

- [ ] **2.2** Create `src/command_bridge/setup.py`
  - Entry point: `command_bridge_node = command_bridge.command_bridge_node:main`
  - Standard ament_python layout

- [ ] **2.3** Create `src/command_bridge/setup.cfg`
  - Standard install directories

- [ ] **2.4** Create `src/command_bridge/resource/command_bridge`
  - Empty marker file for ament index

### Step 3: Update the pipeline launch

- [ ] **3.1** Add the bridge node to `src/prosthesis_launch/launch/pipeline.launch.py`
  - Add a `Node` entry for `command_bridge` between the `mia_hand_driver` and the proximity controller
  - Pass config file parameters

- [ ] **3.2** Add bridge parameters to `config/prosthesis_config.yaml`
  - Add a `command_bridge:` section under the ROS2 node parameter sections with the configurable topic/service names and `default_speed_percent`

### Step 4: Update Dockerfile if needed

- [ ] **4.1** Verify the Dockerfile picks up the new package
  - The `colcon build` in the Dockerfile should auto-discover `src/command_bridge` — no changes needed unless the Dockerfile has an explicit package list

## Verification Criteria

- [ ] `colcon build` succeeds with the new package
- [ ] `ros2 run command_bridge command_bridge_node` starts without error when the raw driver is running
- [ ] Publishing to `/thumb_pos_ff_controller/commands` with `{data: [0.5]}` results in a service call to `joints/thumb/set_trajectory` and the thumb moves
- [ ] `/joint_states` is published when the raw driver's joint position stream is active
- [ ] The force controller receives force data AND can command position changes through the bridge
- [ ] The proximity controller's partial/full closure commands reach the hand through the bridge

## Potential Risks and Mitigations

1. **Service call latency**: The bridge uses `call_async()` so it won't block the callback. However, if the raw driver's service queue backs up, commands may be delayed. Mitigation: add a throttle/debounce — only send the latest command if the previous service call hasn't completed yet.

2. **Position vs. trajectory semantics**: The `*_pos_ff_controller/commands` topics carry a single position value (like a setpoint), but `SetJointTraj` is a trajectory request with speed. The bridge maps position → target_angle and uses a fixed speed. This is fine for the proximity controller (gradual approach) and force controller (small incremental adjustments).

3. **Joint state stream activation**: The bridge needs to call `data_streams/joints/positions/switch` to start the stream. If the raw driver hasn't connected yet, this call will fail. Mitigation: use a timer to retry until successful, and log a warning.

4. **Command ordering**: If both the proximity controller and force controller publish simultaneously (shouldn't happen due to pipeline state gating), the bridge could send conflicting commands. Mitigation: this is already handled by the pipeline state machine — proximity controller stops during GRASPING/HOLDING.

## Alternative Approaches

1. **Modify ros2_control plugin to include force sensors**: Extend `MiaHandSystemInterface::read()` in C++ to also query force sensors and expose them as state interfaces. This is the "proper" ros2_control solution but requires C++ changes and rebuilding the plugin. Trade-off: cleaner architecture but more effort and harder to iterate on.

2. **Change proximity/force controllers to call services directly**: Instead of publishing to `*_pos_ff_controller/commands`, have them call `joints/*/set_trajectory` services. Trade-off: requires changing two existing working nodes and couples them to the raw driver API. The bridge approach keeps them decoupled.

3. **Use ros2_control with a separate force bridge**: Run `mia_hand_ros2_control` for position control, and create a separate node that opens a second connection to the hand just for force data. Trade-off: the `CppDriver` opens the serial port exclusively — this won't work without modifying the driver to support shared access.
