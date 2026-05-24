# Force Controller Implementation Plan

## Objective

Implement a working force controller for the Mia Hand that regulates grasp force during the GRASPING and HOLDING pipeline states. The controller reads fingertip force sensor data and adjusts motor positions to maintain a target force range. This replaces the existing non-functional `force_controller` node.

---

## Analysis of Available Control Modes

### Mode 1: Position-Based Force Control (RECOMMENDED — Primary Approach)

**Mechanism:** Read force sensors → compute position adjustments → send new target positions via `set_joint_trajectory()`.

**Interfaces used:**
- Input: `data_streams/fingers/forces/data` topic (`ForceData` msg with `int32` normal + tangential forces per finger)
- Output: `/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands` (`Float64MultiArray` — joint position in radians)
- Force stream activation: `data_streams/fingers/forces/switch` service (`std_srvs/SetBool`)

**Why this is the primary approach:**
- The proximity controller already uses this exact path (`grasp_proximity_controller_node.py:99-104`)
- The ros2_control system interface supports position commands (`mia_hand_system_interface.cpp:351-363`)
- No driver modifications needed
- Well-understood control problem: incremental position adjustments to regulate force

**Limitations:**
- Position-to-force mapping is object-dependent (stiff objects respond faster than soft ones)
- Requires a PI controller with conservative gains to avoid oscillation
- Serial bus contention at 115200 baud (mitigated by streaming + 10 Hz control rate)

### Mode 2: Current-Limited Motor Speed Control (SUPPLEMENTARY — Safety Layer)

**Mechanism:** Use `set_motor_speed(motor, target_speed, max_current_percent)` to limit motor current to a safe percentage, preventing excessive grip force.

**Interfaces used:**
- `motors/thumb/set_speed`, `motors/index/set_speed`, `motors/mrl/set_speed` services (`SetMotorSpeed.srv`)
- `max_current_percent` parameter: range [0, 80]

**Why this is supplementary:**
- Current limiting is not the same as force control — it's a coarse safety mechanism
- The Mia Hand firmware maps `spe_for_percent` in `execute_grasp()` to speed/force, but the exact semantics are firmware-dependent and poorly documented
- `set_motor_speed` with `max_cur` parameter directly caps motor torque, which indirectly limits maximum force
- Best used as an upper-bound safety layer during grasping, not as the primary force regulation mechanism

**Limitations:**
- Current limit is not force feedback — it doesn't adapt to object properties
- The relationship between current percentage and fingertip force is nonlinear and temperature-dependent
- Switching between position mode and speed mode requires careful mode management (see `prepare_command_mode_switch` in `mia_hand_system_interface.cpp:230-309`)

### Mode 3: Grasp Execute with Speed/Force Parameter (NOT RECOMMENDED for Fine Control)

**Mechanism:** Use `execute_grasp(grasp_id, close_percent, spe_for_percent)` — the `spe_for_percent` parameter nominally controls speed/force.

**Why not recommended:**
- This is a coarse, open-loop parameter — no feedback
- It controls all 3 motors simultaneously with a single parameter
- No per-finger granularity
- The exact firmware behavior of `spe_for_percent` is unclear (speed vs force is ambiguous)

### Decision: Use Mode 1 (Position-Based) as primary, Mode 2 (Current Limit) as safety

---

## Architecture

```
Pipeline Manager publishes /pipeline/state
         │
         ▼
  ┌─────────────────────┐
  │  Force Controller   │◄──── data_streams/fingers/forces/data (ForceData)
  │  (Python node)      │
  │                     │──── /thumb_pos_ff_controller/commands
  │  PI control loop    │──── /index_pos_ff_controller/commands
  │  + slip detection   │──── /mrl_pos_ff_controller/commands
  │  + safety limits    │
  └─────────────────────┘
         │
         ▼
  publishes /force_controller/status (ForceControllerStatus)
         │
         ▼
  Pipeline Manager (GRASPING → HOLDING transition trigger)
```

**Key design decisions:**
1. Force controller **takes over** from proximity controller when pipeline enters GRASPING state
2. During APPROACHING, proximity controller runs (as now); force controller is dormant
3. Force controller publishes a status topic so the pipeline manager can transition GRASPING → HOLDING when forces stabilize
4. Slip detection uses tangential force rate-of-change to trigger re-grip

---

## Files to Modify

### 1. `src/force_controller/force_controller/force_controller_node.py` — REWRITE (existing file)
### 2. `src/force_controller/package.xml` — EDIT (add `mia_hand_msgs`, `std_srvs` dependencies)
### 3. `src/force_controller/setup.py` — EDIT (no changes needed, entry point is correct)
### 4. `src/force_controller/resource/force_controller` — CREATE (empty file for ament index)
### 5. `config/prosthesis_config.yaml` — EDIT (update force section with new parameters)
### 6. `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py` — EDIT (subscribe to force controller status, add GRASPING→HOLDING transition)
### 7. `src/mia_hand_msgs/CMakeLists.txt` — EDIT (add new message files)
### 8. NEW: `src/mia_hand_msgs/msg/ForceControllerStatus.msg` — CREATE

---

## Implementation Plan (STATUS: ALL COMPLETE)

### Phase 1: Build Infrastructure

- [x] **Task 1.1** DONE — `src/force_controller/resource/force_controller` created (empty file)
- [x] **Task 1.2** DONE — `mia_hand_msgs`, `std_srvs`, `sensor_msgs` deps added to `src/force_controller/package.xml`
- [x] **Task 1.3** DONE — `src/mia_hand_msgs/msg/ForceControllerStatus.msg` created with all fields
- [x] **Task 1.4** DONE — `src/mia_hand_msgs/CMakeLists.txt` updated with new msg file

### Phase 2: Rewrite the Force Controller Node

- [x] **Task 2.1** DONE — `src/force_controller/force_controller/force_controller_node.py` fully rewritten (494 lines)
- [x] **Task 2.2** DONE — Streaming activation lifecycle implemented in `_activate_controller()` / `_deactivate_controller()`
- [x] **Task 2.3** DONE — Position command interface publishes to `*_pos_ff_controller/commands`
- [x] **Task 2.4** DONE — `/joint_states` subscription tracks `j_thumb_fle`, `j_index_fle`, `j_mrl_fle`

### Phase 3: Pipeline Manager Integration

- [x] **Task 3.1** DONE — Pipeline manager subscribes to `/force_controller/status`
- [x] **Task 3.2** DONE — `_on_force_status` callback triggers GRASPING → HOLDING on `force_stable=true`
- [x] **Task 3.3** DONE — GRASPING → HOLDING already in valid transitions map
- [x] **Task 3.4** DONE — `mia_hand_msgs` dep added to `src/pipeline_manager/package.xml`

### Phase 4: Configuration

- [x] **Task 4.1** DONE — `config/prosthesis_config.yaml` updated with all new force parameters
- [x] **Task 4.2** DONE — Launch files already pass `config_file` parameter to force controller node

### Phase 5: Handoff Between Proximity Controller and Force Controller

- [x] **Task 5.1** DONE — Handoff protocol documented in force controller node docstring
- [x] **Task 5.2** DONE — Proximity controller subscribes to `/pipeline/state` and skips control loop during GRASPING/HOLDING

### Phase 6: Safety and Edge Cases

- [x] **Task 6.1** DONE — Emergency force release backs off by `2 * max_position_step` when force > 500
- [x] **Task 6.2** DONE — Integral anti-windup clamps to `±integral_limit`
- [x] **Task 6.3** DONE — Stale force data handling (2s timeout, skips commands)
- [x] **Task 6.4** DONE — Service availability handled with `wait_for_service(timeout_sec=2.0)` only on activation

---

## Verification Criteria

1. **Force streaming activates on GRASPING entry**: When pipeline transitions to GRASPING, the force controller successfully calls `data_streams/fingers/forces/switch` with `true`, and `ForceData` messages start arriving on `data_streams/fingers/forces/data`.

2. **Position commands are published to correct topics**: During GRASPING/HOLDING, the force controller publishes `Float64MultiArray` messages to `/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, and `/mrl_pos_ff_controller/commands`.

3. **Force regulation converges**: Given a target force range of [50, 200] raw units and an initial contact force, the controller adjusts positions until all finger forces are within `stability_tolerance` of the target range within 5 seconds.

4. **Stability detection triggers HOLDING**: When forces stabilize, the `/force_controller/status` message has `force_stable=true`, and the pipeline manager transitions from GRASPING to HOLDING.

5. **Slip detection works**: If tangential force rate exceeds `slip_threshold`, the controller increases grip force and `slip_detected=true` appears in the status.

6. **Emergency release works**: If any finger force exceeds `max_force_emergency`, all fingers back off immediately.

7. **No conflicts with proximity controller**: During APPROACHING, only the proximity controller publishes position commands. During GRASPING/HOLDING, only the force controller publishes.

8. **Clean state transitions**: Entering and leaving GRASPING/HOLDING properly activates/deactivates force streaming and resets controller state.

---

## Potential Risks and Mitigations

1. **Serial bus contention at 115200 baud**
   - The Mia Hand uses a single serial port for all communication. Force streaming + position commands share this bus.
   - Mitigation: The streaming timer adapts its period based on number of active streams (`10ms * n_streams_` — `ros_driver.cpp:430`). With only force streaming active, this is 10ms per read. At 10 Hz control rate, the force controller sends 3 position commands per second. Total serial load is manageable.

2. **Nonlinear position-to-force relationship**
   - Motor position to fingertip force depends heavily on object stiffness, geometry, and contact location.
   - Mitigation: Use conservative PI gains (`kp=0.01`, `ki=0.001`) and a small `max_position_step` (0.05 rad ≈ 2.9 degrees per tick). This ensures slow, stable convergence regardless of object properties.

3. **Sensor noise and quantization**
   - Force sensors return raw `int32` values that may be noisy.
   - Mitigation: Apply a simple moving average filter (window of 5 samples) to force readings before using them in the control loop. This is a standard approach for noisy sensor feedback.

4. **Two nodes commanding same joints**
   - Proximity controller and force controller both publish to `*_pos_ff_controller/commands`.
   - Mitigation: Pipeline state gating. Proximity controller skips its loop during GRASPING/HOLDING; force controller only runs during GRASPING/HOLDING. This is a clean time-division multiplexing.

5. **Unknown force sensor units**
   - The raw `int32` values from `ForceData` are not documented as Newtons or any specific unit. The config values `target_force_min: 50` and `target_force_max: 200` suggest the useful range is in the hundreds.
   - Mitigation: Treat sensor values as arbitrary units. Tune `target_force_min/max` empirically with the actual hardware. The controller design is unit-agnostic.

6. **ros2_control position command timing**
   - The ros2_control `write()` method calls `set_joint_trajectory()` with a hardcoded speed of 50% (`mia_hand_system_interface.cpp:353-354`). This means position changes are not instantaneous.
   - Mitigation: The 10 Hz control rate is slow enough that the motor will reach its target before the next command. If needed, the speed parameter could be exposed via the controller config.

---

## Alternative Approaches

1. **Current-limiting as primary force control (rejected)**: Use `set_motor_speed()` with a low `max_cur` to indirectly limit force. Rejected because current limiting is open-loop — it doesn't use force sensor feedback and can't adapt to different objects. Better as a safety layer.

2. **Embedded force control in Mia Hand firmware (not available)**: Some robotic hands implement force control in firmware. The Mia Hand firmware does not expose a closed-loop force control mode. The `spe_for_percent` parameter in `execute_grasp()` is the closest, but it's a single scalar for all motors and its exact behavior is undocumented.

3. **ros2_control force controller plugin (over-engineered)**: Implement a custom `ros2_control` controller plugin (like `force_controllers/JointGroupForceController`). This would be more "ROS2-idiomatic" but requires C++ development, custom controller registration, and doesn't provide significant advantages over a Python node for this use case. The sensor data isn't even exposed through ros2_control state interfaces — it comes from a separate topic.

4. **Hybrid position + current-limit approach (recommended addition)**: After the primary position-based controller is working, add a secondary safety layer that calls `set_motor_speed()` with a conservative `max_cur` (e.g., 30%) during GRASPING. This provides a hard ceiling on force even if the position controller misbehaves. This can be added as a future enhancement without changing the architecture.
