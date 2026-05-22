# Pipeline Launch Fix: Use ros2_control for Joint Commands

## Objective

Update `pipeline.launch.py` so that the proximity controller and force controller can actually command the Mia Hand hardware. Currently the pipeline launches the raw `mia_hand_driver`, which does NOT provide the `*_pos_ff_controller/commands` topics that these nodes publish to. The fix must also preserve access to force sensor data, which only the raw driver provides.

## Problem Analysis

### Current State

`src/prosthesis_launch/launch/pipeline.launch.py:55-61` launches `mia_hand_driver_node` (the raw driver). This driver:
- Provides services: `motors/thumb/set_trajectory`, `data_streams/fingers/forces/switch`, etc.
- Publishes: `data_streams/fingers/forces/data` (ForceData), `data_streams/motors/positions/data`, etc.
- Does **NOT** provide: `/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands`, or `/joint_states`

The proximity controller publishes to `*_pos_ff_controller/commands` (ros2_control topics).
The force controller publishes to `*_pos_ff_controller/commands` and subscribes to `/joint_states`.
The force controller subscribes to `data_streams/fingers/forces/data` and calls `data_streams/fingers/forces/switch` (raw driver topics/services).

### The Conflict

- `mia_hand_ros2_control` provides: `*_pos_ff_controller/commands`, `/joint_states` — but has **zero force sensor support**
- `mia_hand_driver` provides: force sensor data, streaming, services — but has **no ros2_control topics**
- Both use `CppDriver::create()` which opens the serial port exclusively — **they cannot run simultaneously**

### What the Force Controller Needs

Looking at `src/force_controller/force_controller/force_controller_node.py`:
- Subscribes to `data_streams/fingers/forces/data` (ForceData) — from raw driver
- Calls `data_streams/fingers/forces/switch` (SetBool) — raw driver service
- Subscribes to `/joint_states` (JointState) — from ros2_control's joint_state_broadcaster
- Publishes to `*_pos_ff_controller/commands` — ros2_control ForwardCommandController topics

The force controller needs **both** interfaces simultaneously. This is the core design tension.

## Recommended Approach: ros2_control + Force Data Bridge

Replace the raw `mia_hand_driver` with `mia_hand_ros2_control` in the pipeline launch. This gives us the `*_pos_ff_controller/commands` topics and `/joint_states` that the proximity and force controllers need. For force sensor data, spawn a minimal "force bridge" node that talks to the Mia Hand via a separate mechanism.

However, since both the ros2_control interface and the raw driver open the same serial port exclusively, we need a different strategy. The cleanest solution:

### Strategy: ros2_control as primary + force data via ros2_control sensor interface

The `MiaHandSystemInterface` already reads joint positions/velocities in its `read()` method (`mia_hand_system_interface.cpp:311-342`). We need to extend it to also read force sensor data and expose it as a ros2_control state interface. Then a lightweight bridge node can read from the ros2_control state interface and publish the `ForceData` message that the force controller expects.

But this requires modifying C++ code in the ros2_control plugin, which is a significant change.

### Simpler Strategy (RECOMMENDED): ros2_control + direct serial force reader

1. Use `mia_hand_ros2_control` as the sole driver (it owns the serial port)
2. Extend `MiaHandSystemInterface::read()` to also query force sensors and expose them as custom state interfaces
3. Create a thin Python bridge node that reads force data from the ros2_control state interfaces and republishes as `ForceData` + provides the `forces/switch` service

Actually, even simpler: the ros2_control `read()` already calls `get_joint_positions()` and `get_joint_speeds()` every cycle. The CppDriver also has `get_force_data()` which reads forces over serial. We can add force reading to the system interface's `read()` method and expose force values as state interfaces.

### Simplest Possible Strategy (QUICK FIX): Run ros2_control only, skip force controller for now

For immediate hardware testing:
1. Replace `mia_hand_driver_node` with `mia_hand_ros2_control` launch in `pipeline.launch.py`
2. Spawn all three individual position controllers (`thumb_pos_ff_controller`, `index_pos_ff_controller`, `mrl_pos_ff_controller`)
3. The proximity controller will work immediately
4. The force controller won't get force data, but it handles this gracefully (logs warning, operates in passive mode)
5. Add force sensor support to ros2_control as a follow-up task

## Implementation Plan (Quick Fix + Follow-up)

### Phase 1: Quick Fix — Make Proximity Controller Work with Hardware

- [ ] **Task 1.1** Replace `mia_hand_driver_node` in `pipeline.launch.py` with an `IncludeLaunchDescription` that imports `mia_hand_ros2_control/launch/mia_hand_system_interface_launch.py`. Pass `serial_port` from the config. Set `rviz2_gui:=false` (we have our own RViz). Set `controller:=thumb_pos_ff_controller` (we'll spawn the others separately).

- [ ] **Task 1.2** Add spawner nodes for `index_pos_ff_controller` and `mrl_pos_ff_controller` in `pipeline.launch.py`. The default launch only spawns one controller (the `controller` arg). We need all three individual position controllers active for the proximity and force controllers to publish to. Add them as `Node(package='controller_manager', executable='spawner', arguments=['thumb_pos_ff_controller', '-c', '/controller_manager'])` etc., sequenced after the `joint_state_broadcaster`.

- [ ] **Task 1.3** Remove the old `mia_hand_driver` node entry from `pipeline.launch.py` since ros2_control replaces it entirely.

- [ ] **Task 1.4** Verify that `/joint_states` is published (from `joint_state_broadcaster`) and that `ros2 topic echo /thumb_pos_ff_controller/commands` shows messages when the proximity controller publishes.

### Phase 2: Make Force Controller Work — Add Force Data to ros2_control

- [ ] **Task 2.1** In `mia_hand_system_interface.cpp`, add force sensor reading to the `read()` method. After reading joint positions/velocities, also call `mia_hand_->get_force_data()` (or equivalent) to read the 6 force values (3 normal + 3 tangential). Store them in member arrays `fin_normal_forces_[3]` and `fin_tangential_forces_[3]`.

- [ ] **Task 2.2** In `mia_hand.ros2_control.xacro`, add 6 custom state interfaces for force data. Use GPIO-style sensors in the ros2_control xacro: `fin_for_thumb_normal`, `fin_for_thumb_tangential`, `fin_for_index_normal`, etc. These map to the force values stored in the system interface.

- [ ] **Task 2.3** In `export_state_interfaces()`, expose the 6 force values as state interfaces so they can be read by controllers or bridges.

- [ ] **Task 2.4** Create a new lightweight Python node `force_data_bridge_node.py` in the `force_controller` package. This node:
  - Reads the 6 force state interfaces from ros2_control (via `/dynamic_joint_states` or a custom controller)
  - Publishes `ForceData` messages on `data_streams/fingers/forces/data`
  - Provides the `data_streams/fingers/forces/switch` service (since streaming is now always-on when the ros2_control read cycle includes force queries, the service is just a no-op that returns success)

  Actually, the simpler approach: since the force data is now available as ros2_control state interfaces, the force controller can read them directly via a custom controller or via the `/joint_states` topic if we extend the joint_state_broadcaster. But joint_state_broadcaster only broadcasts joint states, not GPIO/sensor states.

  **Simplest approach for Task 2.4:** Instead of a bridge, modify the force controller to read force data from a different source. Options:
  - (a) Use `ros2_control` GPIO state interfaces via a custom broadcaster
  - (b) Use the `mia_hand_driver` CppDriver directly from Python via a thin wrapper
  - (c) Add a force-reading thread in the system interface that publishes `ForceData` as a ROS topic directly from the `read()` cycle

  Option (c) is the most practical: the system interface already runs a read cycle at 5 Hz. Add a publisher that emits `ForceData` after each force read. This way the force controller's existing subscription to `data_streams/fingers/forces/data` works unchanged.

- [ ] **Task 2.5** Rebuild and verify force data flows through the ros2_control path.

### Phase 3: Mock Mode Compatibility

- [ ] **Task 3.1** Update `mock.launch.py` similarly — use `mia_hand_ros2_control` with `use_mock_hardware:=true` instead of no driver at all. This gives mock mode the `*_pos_ff_controller/commands` topics and `/joint_states` that the proximity and force controllers expect.

- [ ] **Task 3.2** Verify mock mode still works: `ros2 launch prosthesis_launch mock.launch.py` starts without errors, and the proximity controller can publish to the command topics.

## Verification Criteria

- [ ] `make build` succeeds
- [ ] `make up-hw` followed by `make shell` → `ros2 launch prosthesis_launch pipeline.launch.py` starts without errors
- [ ] `ros2 topic list` shows `/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands`
- [ ] `ros2 topic echo /joint_states` shows position/velocity for `j_thumb_fle`, `j_index_fle`, `j_mrl_fle`
- [ ] Publishing to `/grasp_preshaping/target_finger_closures` + moving hand near target causes the physical hand to close
- [ ] Force data flows on `data_streams/fingers/forces/data` (after Phase 2)
- [ ] `make up` (mock mode) also works correctly

## Potential Risks and Mitigations

1. **Serial port contention between ros2_control and force bridge**
   The ros2_control system interface owns the serial port. No other process can open it. Force data must be read from within the system interface's `read()` cycle.
   Mitigation: Read forces inside `MiaHandSystemInterface::read()` and publish via an embedded ROS publisher or expose as state interfaces.

2. **Controller spawner ordering**
   The position controllers must be spawned after the `joint_state_broadcaster`. The existing launch file handles this sequencing via `OnProcessExit` events.
   Mitigation: Follow the same pattern for the additional controller spawners.

3. **Multiple controllers commanding the same joints**
   Only one position controller per joint can be active at a time. If `group_pos_ff_controller` and `thumb_pos_ff_controller` are both active for `j_thumb_fle`, ros2_control will reject the mode switch.
   Mitigation: Only spawn the three individual controllers (`thumb`, `index`, `mrl`), NOT the group controller. Or spawn the group controller and use it instead of the three individuals. The proximity/force controllers publish to individual topics, so we need the individual controllers.

4. **ros2_control update rate mismatch**
   The controller manager runs at 5 Hz (`mia_hand_controllers.yaml:7`). The proximity controller publishes at 10 Hz and the force controller at 10 Hz. Commands published between ros2_control update cycles will be queued.
   Mitigation: Increase the ros2_control update rate to 10 Hz to match the controllers, or accept the slight latency.

5. **Build dependency: mia_hand_ros2_control must be built before prosthesis_launch**
   If `mia_hand_ros2_control` is not in the workspace, the `IncludeLaunchDescription` will fail.
   Mitigation: It's already in the workspace at `src/mia_hand_ros2_control/`. Verify `colcon build` builds it.

## Alternative Approaches

1. **Keep raw driver + add ForwardCommandController-style subscribers**: Instead of switching to ros2_control, modify the proximity/force controllers to publish to the raw driver's services (`motors/thumb/set_trajectory` etc.) instead of ros2_control topics. This avoids the serial port conflict but means we lose `/joint_states` and all ros2_control benefits.
   Trade-off: Less refactoring of the launch file, but more refactoring of the controller nodes, and we lose the ros2_control ecosystem.

2. **Keep raw driver + add a bridge that creates the command topics**: Create a new node that subscribes to `*_pos_ff_controller/commands` and forwards them to the raw driver's `set_trajectory` services. This way the proximity/force controllers publish to their expected topics, and the bridge translates to the raw driver interface.
   Trade-off: Simple to implement, no C++ changes, but adds latency and a single point of failure. The bridge would also need to publish `/joint_states` from the raw driver's joint data streams.

3. **Run both drivers on different serial ports**: Not possible — there's only one Mia Hand and one serial port.
   Trade-off: Not viable.
