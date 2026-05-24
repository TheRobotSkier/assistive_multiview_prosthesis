# Force Controller Feasibility Analysis & Implementation Plan

## Objective

Assess the feasibility of implementing a working force controller for the Mia Hand that maintains correct grasp force based on fingertip sensor feedback, and create a plan to make it functional within the existing pipeline architecture.

---

## Current State Assessment

### What Already Exists

A `force_controller` package already exists at `src/force_controller/` with a skeleton implementation in `src/force_controller/force_controller/force_controller_node.py:1-202`. It is included in both the real pipeline launch (`src/prosthesis_launch/launch/pipeline.launch.py:107-114`) and the mock launch (`src/prosthesis_launch/launch/mock.launch.py:88-95`).

### Critical Gap Analysis — Why It Does NOT Work

The existing force controller has **several fundamental disconnects** from the actual Mia Hand driver infrastructure. It was written as a standalone concept without aligning to the real sensor/command interfaces:

#### Gap 1: Wrong Force Data Source (CRITICAL)

The force controller subscribes to `/hand/forces` (a `Float32MultiArray`) at `force_controller_node.py:69-74`, but this topic **does not exist anywhere in the system**. The Mia Hand driver publishes force data on `data_streams/fingers/forces/data` using the `mia_hand_msgs/msg/ForceData` message type (see `ros_driver.cpp:115-116`). Furthermore, force streaming is **off by default** — it must be explicitly enabled via the `data_streams/fingers/forces/switch` service (`ros_driver.cpp:300-302`).

The real force data provides **6 values per reading** (3 normal forces + 3 tangential forces as `int32`), not 3 `float32` values as the current controller expects.

#### Gap 2: Wrong Command Output Interface (CRITICAL)

The force controller publishes to `/hand/motor_commands` (`Float32MultiArray`) at `force_controller_node.py:89-93`, but **no node subscribes to this topic**. The actual motor command interfaces are:
- **ros2_control path**: Joint position commands via `ForwardCommandController` topics (`/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands`) — as used by the proximity controller (`grasp_proximity_controller_node.py:99-104`).
- **Direct driver path**: Motor trajectory services (`motors/thumb/set_trajectory`, etc.) or actions (`motors/thumb/trajectory`, etc.) — see `ros_driver.cpp:180-188`.

The force controller's output goes to a dead-end topic.

#### Gap 3: Wrong Force Units/Scaling (HIGH)

The controller uses parameters like `target_force: 2.0` (Newtons) and `max_force: 8.0` (Newtons) at `force_controller_node.py:17-19`. However, the Mia Hand force sensors return **raw integer values** (see `ForceData.msg:1-7` and `cpp_driver.cpp:466-553`), not Newtons. The `prosthesis_config.yaml:89-94` has a more realistic `target_force_min: 50.0` and `target_force_max: 200.0`, suggesting the raw sensor range is in the hundreds. The controller does not read from the config file at all — it uses its own hardcoded defaults.

#### Gap 4: No Integration with Pipeline Manager (MEDIUM)

The pipeline manager (`pipeline_manager_node.py:9`) states "Force controller active" during GRASPING and "monitoring forces" during HOLDING, but there is **no actual signal** from the force controller back to the pipeline manager. The GRASPING → HOLDING transition has no trigger — the pipeline manager never transitions to HOLDING because nothing publishes a "forces stable" event. The force controller only reads `/pipeline/state` but never publishes back to it.

#### Gap 5: No Force Stream Activation (MEDIUM)

The force controller never calls the `data_streams/fingers/forces/switch` service to enable force streaming. Without this, the Mia Hand driver will never publish force data, and the controller will sit at zero forces forever.

#### Gap 6: Missing `resource/` Directory (LOW)

The `setup.py:12` references `resource/force_controller` but no `resource/` directory exists in the package, which will cause `colcon build` warnings (though it won't fail the build).

### Prioritized Issues

| Priority | Issue | Impact |
|----------|-------|--------|
| 1 | Wrong force topic name and message type | Controller receives no data |
| 2 | Wrong command output topic | Commands go nowhere |
| 3 | Wrong force units/scaling | Would over/under-grip if data arrived |
| 4 | No force stream activation | No data flows from hardware |
| 5 | No pipeline integration | State machine stuck in GRASPING |
| 6 | Missing resource directory | Build warning |

---

## Feasibility Verdict

**Yes, this is fully feasible.** The hardware supports it — the Mia Hand has 6-axis force sensors on all three fingertips (thumb, index, middle-ring-little), the driver can stream this data, and motor trajectory commands allow fine-grained position adjustments. The existing code is a reasonable starting skeleton that just needs to be rewired to the actual interfaces.

The Mia Hand's serial protocol supports:
- **Reading forces**: `@ADAo...........*\r` command returns 76 bytes with 6 force values (`cpp_driver.cpp:466-553`)
- **Streaming forces**: Periodic publishing via `stream_tmr_fun()` at `ros_driver.cpp:3673-3681`
- **Motor trajectory control**: `set_motor_trajectory(motor, target_pos, speed_percent)` for fine position adjustments (`cpp_driver.cpp:555-661`)
- **Motor speed control**: `set_motor_speed(motor, speed, max_current_percent)` — useful for current-limited force control (`cpp_driver.cpp:663-755`)

---

## Implementation Plan

### Phase 1: Fix the Force Data Pipeline

- [ ] **1.1** Change the force subscription in `force_controller_node.py` from `/hand/forces` (`Float32MultiArray`) to `data_streams/fingers/forces/data` using `mia_hand_msgs/msg/ForceData` message type. Import `mia_hand_msgs` as a dependency in `package.xml` and `setup.py`.
- [ ] **1.2** Add a service client for `data_streams/fingers/forces/switch` (`std_srvs/srv/SetBool`) in the force controller's `__init__`. On startup, call this service with `data=True` to enable force streaming from the Mia Hand driver. Add `std_srvs` as a dependency.
- [ ] **1.3** Update the force callback to extract all 6 force values (3 normal + 3 tangential) from the `ForceData` message. Store them in instance variables: `self.current_forces_normal = [thumb_nfor, index_nfor, mrl_nfor]` and `self.current_forces_tangential = [thumb_tfor, index_tfor, mrl_tfor]`.
- [ ] **1.4** Add a startup sequence: wait for the force stream service to become available (with timeout), enable streaming, and log success/failure. If the service never becomes available (e.g., mock mode), log a warning and continue in a passive mode.

### Phase 2: Fix the Command Output Interface

- [ ] **2.1** Replace the single `/hand/motor_commands` publisher with three publishers matching the proximity controller's interface: `/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, and `/mrl_pos_ff_controller/commands` — all using `Float64MultiArray`. This ensures compatibility with the ros2_control `ForwardCommandController` path.
- [ ] **2.2** Alternatively (or additionally), add service clients for the direct motor trajectory services (`motors/thumb/set_trajectory`, `motors/index/set_trajectory`, `motors/mrl/set_trajectory`) using `mia_hand_msgs/srv/SetMotorTraj`. This gives direct serial control with speed/force percentage, which may be more appropriate for fine-grained force regulation.
- [ ] **2.3** Decide on the command approach: the `ForwardCommandController` path works in joint-space (radians), while the direct motor trajectory path works in motor-space (0-255 integer positions). The force controller should use whichever path the rest of the pipeline uses. Based on the proximity controller using `ForwardCommandController`, use the same path for consistency.

### Phase 3: Fix Force Control Logic

- [ ] **3.1** Update parameters to use raw sensor units instead of Newtons. Read from `prosthesis_config.yaml` via the `config_file` parameter (already passed in the launch file). Map to the config section `force.target_force_min` (50) and `force.target_force_max` (200) as the target force range.
- [ ] **3.2** Implement a proper PI controller instead of the current proportional-only approach. The config already defines `kp: 0.01` and `ki: 0.001` at `prosthesis_config.yaml:92-93`. Use the normal force component for regulation (normal force = grasp pressure, tangential = slip detection).
- [ ] **3.3** Add integral windup protection: clamp the integral term to prevent runaway accumulation when the hand is not in contact (forces are zero during approach).
- [ ] **3.4** Add slip detection using tangential forces: if tangential force exceeds a threshold relative to normal force (friction cone violation), increase the closure command to tighten the grip.
- [ ] **3.5** The control output should be a **delta adjustment** on top of the desired position from the proximity controller, not an absolute position. The force controller should subscribe to the same closure commands the proximity controller publishes and apply small corrections.

### Phase 4: Integrate with Pipeline Manager

- [ ] **4.1** Add a publisher on a feedback topic (e.g., `/force_controller/status`) that publishes the current force regulation state: `{forces_stable: bool, max_force: float, avg_force: float, slip_detected: bool}`.
- [ ] **4.2** In the pipeline manager, subscribe to this status topic. Use `forces_stable == True` as the trigger to transition from GRASPING → HOLDING (currently this transition has no trigger in the pipeline manager).
- [ ] **4.3** During HOLDING, the force controller continues regulating. If `slip_detected` becomes True or forces drop below minimum, the pipeline manager could log a warning or trigger a re-grasp.
- [ ] **4.4** During RELEASING, the force controller should stop regulating and allow the hand to open fully (pass-through mode).

### Phase 5: Activation Lifecycle

- [ ] **5.1** The force controller should only be active during GRASPING and HOLDING states (already partially implemented at `force_controller_node.py:140`). In all other states, it should pass through commands unchanged.
- [ ] **5.2** On transition TO GRASPING: reset the integral term, record the initial closure positions as the baseline, begin force regulation.
- [ ] **5.3** On transition FROM HOLDING (to RELEASING): stop publishing force-adjusted commands, allow the proximity controller or pipeline manager to command the hand open.
- [ ] **5.4** Add a latched subscriber to `/pipeline/state` (already exists) and track state transitions with previous/current state comparison.

### Phase 6: Build & Package Fixes

- [ ] **6.1** Add `mia_hand_msgs` and `std_srvs` to `package.xml` dependencies.
- [ ] **6.2** Create the missing `resource/force_controller` empty file so `setup.py` doesn't warn.
- [ ] **6.3** Update `setup.py` to include the new dependencies in `install_requires` if needed.
- [ ] **6.4** Verify the package builds with `colcon build --packages-select force_controller`.

---

## Verification Criteria

- [ ] Force controller node starts without errors and logs successful force stream activation
- [ ] When force data is published on `data_streams/fingers/forces/data`, the controller receives and processes it correctly
- [ ] During GRASPING/HOLDING states, the controller publishes adjusted position commands on the three `ForwardCommandController` topics
- [ ] During IDLE/APPROACHING/RELEASING states, the controller passes through commands unchanged
- [ ] The pipeline manager transitions from GRASPING → HOLDING when forces stabilize
- [ ] Emergency force release works when forces exceed the maximum threshold
- [ ] `colcon build --packages-select force_controller` succeeds without errors

---

## Potential Risks and Mitigations

1. **Serial bus contention**: Both the force controller (reading forces) and the position controller (writing positions) communicate through the same serial port at 115200 baud. The driver already serializes access with a mutex (`cpp_driver.cpp:409`), but high-frequency force reading + position writing could cause latency.
   - **Mitigation**: Use the streaming mechanism rather than on-demand force reads. The streaming timer (`ros_driver.cpp:3621-3684`) already handles this efficiently. Keep the force control loop rate modest (10 Hz per config).

2. **Sensor noise**: Raw force sensor values may be noisy, leading to oscillation in the control loop.
   - **Mitigation**: Apply a low-pass filter (exponential moving average) to force readings before using them in the PI controller. The `force_tolerance` parameter already provides a deadband.

3. **Nonlinear force-position relationship**: The relationship between motor position and fingertip force is highly nonlinear (depends on object stiffness, contact geometry, etc.). A simple PI controller on position may not converge.
   - **Mitigation**: Start with conservative gains (`kp=0.01` from config). The current approach of incremental position adjustments is safer than trying to compute exact positions. Consider adding a current-limiting mode (`set_motor_speed` with `max_current_percent`) as an alternative control strategy.

4. **Two nodes commanding the same joints**: Both the proximity controller and the force controller may publish to the same `ForwardCommandController` topics simultaneously.
   - **Mitigation**: The force controller should only publish during GRASPING/HOLDING, while the proximity controller publishes during APPROACHING. The pipeline state machine ensures these are mutually exclusive. Add a guard in both nodes to check pipeline state before publishing.

5. **Force stream not available in mock mode**: In mock mode there is no Mia Hand driver, so the force stream service won't exist.
   - **Mitigation**: The force controller should handle the service-not-available case gracefully (log a warning, operate in passive mode). The mock launch already includes the force controller, so it should not crash.

---

## Alternative Approaches

1. **Current-limited force control**: Instead of adjusting position, use `set_motor_speed(motor, speed, max_current_percent)` to limit motor current, which indirectly limits grip force. This is simpler and more robust but gives less precise control. The Mia Hand supports current limiting from 0-80% (`cpp_driver.hpp:276-283`).

2. **ros2_control force controller plugin**: Instead of a standalone Python node, implement a ros2_control `ControllerInterface` plugin in C++ that sits in the control loop alongside the position controllers. This gives lower latency and tighter integration but requires C++ development and more complex configuration.

3. **Hybrid approach**: Use the proximity controller for approach/preshape, then hand off to a force controller that uses `set_motor_trajectory` with carefully chosen speed/force percentages to close until target force is reached. This avoids the need for continuous position adjustment and leverages the Mia Hand's built-in force-limiting during trajectory execution.

---

## Recommended Approach

**Go with the main plan (Phases 1-6)** but keep the architecture simple:
- Use the `ForwardCommandController` topic interface (consistent with proximity controller)
- Use the force streaming interface (already implemented in driver)
- Apply a PI controller with conservative gains and deadband
- Add slip detection as a bonus feature

This is the lowest-risk path to a working force controller. The existing skeleton is about 60% structurally correct — it just needs to be rewired to the actual ROS topics and message types.
