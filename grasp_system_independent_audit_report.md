# Independent MIA Hand Grasp System Audit

Date: 2026-05-14

Repository: `wt-hand-interface`

Scope: review of the previous `grasp_system_audit_report.md`, independent inspection of the codebase, hardware deployment readiness, and test sufficiency for force-aware MIA hand grasping.

## Executive Verdict

I agree with the previous report's main conclusion: the repository contains useful pieces for a force-aware MIA hand grasp system, but the deployed architecture is not currently wired well enough to run reliably on the actual hand.

The strongest blockers are not only high-level architecture drift. There are concrete code-level incompatibilities that would stop the system before reaching force-aware closure:

| Severity | Finding | Effect |
|---|---|---|
| Critical | `pipeline_manager` transition logging references undefined `new` | First state transition can throw before completing cleanly |
| Critical | Preshaping publishes `target_hand_pose` as `Pose`, consumers subscribe as `PoseStamped` | Proximity controller and hand trajectory publisher will not receive the target pose |
| Critical | Main hardware launch does not start `ros2_control`, controllers, wrist driver, camera, or hand pose publisher | Hardware command topics and force feedback path are absent |
| Critical | `grasp_proximity_controller_node` infers active controller mode from a non-existent `SwitchController` response field | Successful velocity switch is likely recorded as `position`, so final velocity closure never runs |
| Critical | The standalone `force_controller_node` is disconnected from every real hardware path | It reads and publishes orphan topics |
| Critical | No automated test validates real MIA serial connection, `/joint_states.effort`, controller switching, release/open behavior, or force contact stopping | Hardware readiness is not verified |

The previous report was broadly accurate. I would refine a few details: the standalone MIA driver does publish a `ForceData` topic in addition to services, but it is still not compatible with `/hand/forces`; the mock ros2_control description declares effort state interfaces for the actuated joints, but it does not generate meaningful force data; and the ARM64 `.so` concern is deployment-relevant mainly if the full grasp planner runs on Jetson rather than the x86 host.

## Intended Architecture

The intended architecture appears to be:

```text
EMG gesture
  -> pipeline_manager
  -> segmentation / object cloud
  -> grasp_preshaping C++ bridge / Rust planner
  -> target hand pose + wrist pose + finger closures
  -> grasp_proximity_controller
  -> ros2_control position controllers for approach
  -> ros2_control velocity controllers for final force-aware closure
  -> MiaHandSystemInterface
  -> CppDriver serial commands to MIA hand
  -> raw fingertip force values returned as /joint_states.effort
```

There is also an older or parallel architecture:

```text
mia_hand_driver_node
  -> RosDriver
  -> CppDriver
  -> custom topics/services/actions under data_streams/, joints/, motors/, fingers/
```

Those two paths are not interchangeable. The force-aware proximity controller expects the `ros2_control` path, but the main hardware launch starts the standalone driver path.

## Review Of Previous Report Findings

### Findings I Agree With

I agree with these previous findings:

| Prior finding | My evaluation |
|---|---|
| Duplicate force-control paths exist | Correct. `force_controller_node.py` and `grasp_proximity_controller_node.py` implement incompatible force loops. |
| `force_controller_node.py` is orphaned | Correct. Its `/hand/forces`, `/hand/grasp_commands`, and `/hand/motor_commands` topics are not used elsewhere. |
| `pipeline.launch.py` misses the `ros2_control` stack | Correct. It launches `mia_hand_driver_node`, not `controller_manager` and not the MIA ros2_control hardware interface. |
| Raw force values are used as effort | Correct. Normal fingertip force integers are copied directly into `jnt_eff_state_`. |
| No release/open command is sent on `RELEASING` | Correct. The state changes, but no hand-open actuator command is wired to that transition. |
| MIA and wrist serial defaults conflict | Correct. Config says MIA `/dev/ttyUSB1`, wrist `/dev/ttyUSB0`; launch defaults often use the opposite or hardcode `/dev/ttyUSB0`. |
| Emergency stop is incomplete in the active architecture | Correct. The standalone driver exposes stop/play, but the ros2_control hardware interface does not expose a ROS emergency stop service. |
| Integrated grasp tests are missing | Correct. Current tests are mostly smoke/unit tests and do not prove real hardware behavior. |

### Findings I Would Refine

The previous report said the standalone driver force path is service-only. It is not only service-only: `RosDriver::init_topics()` publishes `data_streams/fingers/forces/data` as `mia_hand_msgs/ForceData` (`src/mia_hand_driver/src/mia_hand_driver/ros_driver.cpp:115-116`), and it also provides `fingers/get_forces` (`ros_driver.cpp:159-161`). The conclusion is still correct because neither path publishes `/hand/forces` as `Float32MultiArray`.

The previous report said mock hardware does not support force state interfaces. The xacro does declare `effort` state interfaces for `j_thumb_fle`, `j_index_fle`, and `j_mrl_fle` (`src/mia_hand_ros2_control/description/ros2_control/mia_hand.ros2_control.xacro:35-57`). The practical issue is more precise: mock hardware does not simulate changing contact forces. It may expose effort fields, but they are not meaningful force measurements.

The previous report's Rust `.so` warning is valid if the planner runs on Jetson/ARM64. The repository goal says the MIA hand connects to the x86 host, so this is not the first blocker for the hand-control loop. It remains a packaging risk because the Docker image does not build Rust from source.

## Additional Critical Findings

### 1. `pipeline_manager` Has A Transition-Time NameError

File: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py:109-124`

The transition method logs:

```python
f'State: {old.name} -> {new.name} ({reason})'
```

There is no local variable named `new`; the parameter is `new_state`. On the first state transition, this can throw `NameError` after `_state` is updated but before `_publish_state()` is called.

Impact: the pipeline manager can fail on the first EMG-triggered transition, before segmentation or preshaping is reached.

Fix: change `new.name` to `new_state.name`, and add a unit test that sends a fake gesture and asserts `/pipeline/state_name` changes.

### 2. `create_timer(..., one_shot=True)` Is Not A Valid rclpy API

File: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py:148-151`

The release path calls:

```python
self.create_timer(1.0, lambda: self._transition(State.IDLE, 'Release complete'), one_shot=True)
```

ROS 2 Python `Node.create_timer()` does not accept `one_shot` in standard rclpy. This will fail when an OPEN/release gesture is processed.

Impact: release behavior is broken before considering the missing hand-open command.

Fix: create a timer and cancel it inside the callback, or use a small release state timer stored on the node.

### 3. Preshaping Target Pose Type Does Not Match Subscribers

Producer: `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:138-139, 447-456`

Consumers: `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:170-175`, `src/pipeline_manager/pipeline_manager/hand_trajectory_publisher.py:124-129`

The C++ preshaping bridge publishes `/grasp_preshaping/target_hand_pose` as `geometry_msgs/msg/Pose`.

The Python proximity controller and trajectory publisher subscribe to `/grasp_preshaping/target_hand_pose` as `geometry_msgs/msg/PoseStamped`.

ROS 2 endpoints with the same topic name but different message types do not connect.

Impact: `_buf_hand_frame` never gets set in the proximity controller, `_try_commit_plan()` never commits a full plan, and no proximity-based approach/final closure runs.

Fix: standardize the topic on `PoseStamped` or `Pose`. `PoseStamped` is preferable because the frame and timestamp matter for a moving hand/camera scene.

### 4. Controller Switch Result Handling Is Wrong

File: `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:454-467`

The node tries to infer active controllers from `result.active_controllers`:

```python
any_vel_active = any(
    c in self._finger_vel_controllers for c in getattr(result, 'active_controllers', [])
)
self._controllers_active = "velocity" if any_vel_active else "position"
```

`controller_manager_msgs/srv/SwitchController` responses normally contain `ok`, not `active_controllers`. `getattr(result, 'active_controllers', [])` therefore returns an empty list, so even a successful switch to velocity controllers is recorded as `position`.

Impact: `_control_force_closure()` returns early at `if self._controllers_active != "velocity"`, so no final velocity commands are sent after the switch.

Fix: track requested target mode locally when issuing the switch, or call `list_controllers` after switching. Do not infer from a field that the service does not return.

### 5. Main Hardware Launch Is Not A Hardware Launch

File: `src/prosthesis_launch/launch/pipeline.launch.py`

The docstring claims the launch starts MIA hand, wrist, camera, force controller, pipeline manager, and RViz. The actual launch has several gaps:

| Missing or wrong element | Evidence | Effect |
|---|---|---|
| No `ros2_control_node` | `pipeline.launch.py` only launches `mia_hand_driver_node` | Controller topics used by proximity controller do not exist |
| No controller spawners | No spawner nodes in `pipeline.launch.py` | `/thumb_pos_ff_controller/commands` and velocity command topics are absent |
| No wrist driver | No `wrist_driver` node in `pipeline.launch.py` | `/wrist/set_position` has no hardware consumer |
| No camera launch | `camera` launch arg exists but is unused | Segmentation input has no default source |
| No `hand_pose_publisher` | Not launched | Preshaping has no pose input unless an external source exists |
| No conditionals | `rviz`, `camera`, and `mia_hand` args are declared but not applied | Launch behavior cannot be toggled as advertised |
| Hardcoded MIA port | `serial_port` is `/dev/ttyUSB0` | Conflicts with config and wrist default |

Impact: `pipeline.launch.py` does not bring up a coherent real-hardware grasp system.

Fix: make a single authoritative hardware launch that includes `robot_state_publisher`, `ros2_control_node`, joint state broadcaster, individual position and velocity controllers, wrist driver, hand pose source, segmentation/camera source, preshaping, proximity controller, and pipeline manager.

### 6. Controller Spawning Does Not Match The Proximity Controller

Files: `src/mia_hand_ros2_control/launch/mia_hand_system_interface_launch.py:128-137, 202-205`, `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:113-128`

The MIA system interface launch spawns only one controller by default: `group_pos_vel_controller`.

The proximity controller publishes to individual feedforward position and velocity controller topics and switches individual controllers:

```text
thumb_pos_ff_controller, index_pos_ff_controller, mrl_pos_ff_controller
thumb_vel_ff_controller, index_vel_ff_controller, mrl_vel_ff_controller
```

Impact: even if the ros2_control launch is started manually, the controllers needed by the proximity controller are not necessarily loaded/active.

Fix: spawn the exact individual controllers used by the proximity controller, or change the proximity controller to publish to a group controller and switch group controllers consistently.

### 7. Pipeline Manager Calls Preshaping Before Required Inputs Exist

Files: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py:154-160`, `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:236-257`

On grasp gesture, the pipeline manager immediately transitions to `SEGMENTING` and calls `/grasp_preshaping/compute_grasp`. The preshaping bridge returns failure unless it has pose, twist, and point cloud.

Impact: with the current main launch, preshaping likely fails with `No pose data received yet`, `No twist data received yet`, or `No point cloud data received yet`.

Fix: the manager should trigger segmentation first, wait for a valid object cloud, wait for hand pose/twist availability, then call preshaping.

### 8. `pipeline_manager` Requires Grasp Type Before Planning, But Grasp Type Comes From Planning

File: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py:168-177`

When an object cloud arrives during `SEGMENTING`, the manager refuses to transition to `PLANNING` unless `_grasp_type != 0`. `_grasp_type` is set from `/grasp_preshaping/grasp_type`, which the preshaping bridge publishes only after a successful compute.

Impact: this creates a sequencing inconsistency. The manager can get stuck in `SEGMENTING` waiting for data that should only exist after `PLANNING`.

Fix: remove the grasp-type gate from segmentation completion, or make grasp type an input gesture/intent rather than a planner output.

### 9. Missing Effort Data Is Mistaken For Fresh Force Data

File: `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:253-264`

The joint-state callback updates `_last_force_time` whenever the required joint names exist, even if `msg.effort` is empty or too short. In that case efforts remain at their previous values, often zeros.

Impact: in simulation or GUI joint-state mode, force staleness can be falsely considered fresh. The final closure can continue using zero force until timeout or over-closure rather than immediately stopping because force feedback is unavailable.

Fix: only update `_last_force_time` after all required effort entries are present. Add a diagnostic warning when joint positions arrive without efforts.

### 10. Release State Does Not Command Opening

Files: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py:145-151`, `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`

The release transition is currently only a state transition. No node responds by commanding `j_thumb_fle`, `j_index_fle`, and `j_mrl_fle` to open.

Impact: a real hand can remain closed on the object after the user performs the OPEN gesture.

Fix: add a single release owner. The proximity controller is the natural place because it already owns hand command topics. It should subscribe to `/pipeline/state`, switch to position controllers if needed, and publish open positions.

## Hardware Interface Assessment

### What Is Present

The low-level MIA hardware interface does contain useful pieces:

| Capability | Evidence |
|---|---|
| Serial connection through `CppDriver` | `mia_hand_system_interface.cpp:104-128` |
| Position command interface | `mia_hand_system_interface.cpp:220-225` |
| Velocity command interface | `mia_hand_system_interface.cpp:227-232` |
| Effort state interface | `mia_hand_system_interface.cpp:191-196` |
| Normal fingertip force readback | `mia_hand_system_interface.cpp:340-353` |
| Controller mode switching hooks | `mia_hand_system_interface.cpp:238-317` |

This is enough for a simple experimental contact-stop controller if launched manually with the correct controllers.

### What Is Missing Or Risky

| Concern | Evidence | Deployment risk |
|---|---|---|
| No effort command interface | `export_command_interfaces()` exports only position/velocity | This is not true force/impedance control |
| Force values are raw | `jnt_eff_state_` receives raw `thumb_nfor`, `index_nfor`, `mrl_nfor` | Contact thresholds are uncalibrated |
| Tangential forces discarded | `thumb_tfor`, `index_tfor`, `mrl_tfor` read but unused | Slip detection is unavailable |
| Driver errors suppressed | read/write contain commented-out `return_type::ERROR` | Failures may not stop controllers |
| Serial I/O is blocking | `CppDriver::send_command()` waits for ACK with timeouts | Control loop can miss cycles on serial jitter |
| Emergency stop not exposed on ros2_control path | stop/play services are in standalone `RosDriver`, not `MiaHandSystemInterface` | Active architecture lacks a direct ROS safety stop |
| Serial ports not stable | `/dev/ttyUSB0` and `/dev/ttyUSB1` are hardcoded in several places | Hardware may swap after reboot |

## Test Coverage Assessment

### Tests That Exist

| Test or script | What it checks | Hardware relevance |
|---|---|---|
| `tests/test_force_aware_closure.py` | Pure Python force-closure math | Useful unit test, no ROS/hardware coverage |
| `tests/test_imu_dead_reckoning.py` | IMU math unrelated to hand grasp force | Not relevant to MIA hand closure |
| `scripts/run_tests.sh` | Build, launch parse, `.so` load, selected node startup, twist propagation integration | Smoke coverage only |
| `scripts/test_nodes_start.sh` | Starts `pipeline_manager`, `force_controller`, `twist_propagation` | Does not start MIA hardware, proximity controller, or ros2_control |
| `scripts/test_launch_syntax.sh` | Imports launch files | Does not instantiate launch descriptions or validate topic wiring |
| `scripts/grasp_test.sh` | Waits for pipeline state `HOLDING` | Does not verify finger commands, MIA forces, hardware motion, or release |
| `docs/grasp_test_plan.md` | Manual scenarios | Useful plan, not automated validation |

### Tests I Tried To Run

I attempted to run the Python tests from the repository root:

```text
pytest -q
python3 -m pytest -q
```

Both failed because `pytest` is not installed in the host environment:

```text
pytest: command not found
No module named pytest
```

This does not prove the tests fail in the Docker image, but it does mean the host workspace does not currently have the basic test runner available.

### Test Sufficiency Verdict

The tests are not sufficient to verify intended real hardware behavior.

Missing coverage includes:

| Missing test | What it should prove |
|---|---|
| MIA serial bringup test | Correct device opens, firmware responds, failure messages are clear |
| ros2_control launch test | Controller manager starts, joint state broadcaster starts, required controllers spawn |
| Force feedback test | `/joint_states.effort` contains non-empty values for thumb/index/MRL |
| Force calibration test | Idle noise floor and contact thresholds are measured on the real hand |
| Controller-switch test | Position controllers stop, velocity controllers start, velocity commands are accepted |
| Final closure integration test | Near-zone entry causes velocity closure, force threshold stops each finger |
| Release test | OPEN gesture commands fingers to fully open and verifies position feedback moves open |
| Safety test | Stale force, serial loss, over-closure, and emergency stop all halt motion safely |
| Topic-contract test | Preshaping producer and proximity/trajectory consumers use identical message types |
| Hardware-in-the-loop smoke test | A short open/close cycle verifies position, velocity, effort, and stop behavior |

## Recommended Fix Order

### P0: Make The System Launch A Coherent Hardware Graph

1. Replace the standalone MIA driver in `pipeline.launch.py` with the `ros2_control` hardware interface.
2. Spawn the individual position and velocity controllers that the proximity controller uses.
3. Launch the wrist driver or remove wrist commands from the default pipeline.
4. Launch a valid hand pose source and camera/segmentation source, or require them explicitly as external dependencies.
5. Apply real launch conditions for `rviz`, `camera`, and `mia_hand` arguments.

### P0: Fix Message And State-Machine Breakages

1. Fix `new.name` to `new_state.name` in `pipeline_manager`.
2. Replace invalid `one_shot=True` timer usage.
3. Standardize `/grasp_preshaping/target_hand_pose` on `PoseStamped`.
4. Remove the grasp-type-before-planning gate or make grasp type an input.
5. Sequence segmentation, pose/twist readiness, and preshaping in the correct order.

### P0: Make Force-Aware Closure Actually Run

1. Fix controller switch mode tracking.
2. Require real effort data before treating force feedback as fresh.
3. Add warnings and a safe stop when efforts are missing.
4. Calibrate raw force thresholds.
5. Add release/open handling.

### P1: Remove Or Consolidate Dead Paths

1. Remove `force_controller_node.py` from hardware launches.
2. Keep either standalone `mia_hand_driver_node` or `MiaHandSystemInterface` as the active path, not both.
3. If standalone driver remains useful for diagnostics, document it as a diagnostic-only launch.

### P1: Add Hardware Verification

1. Add a hardware smoke test that checks serial connection and firmware response.
2. Add a ros2_control graph test that checks controllers and `/joint_states` fields.
3. Add an open/close/force-contact hardware-in-the-loop test with conservative current limits.
4. Add a release test that asserts fingers open after the OPEN gesture.
5. Add a force-noise calibration script and store measured thresholds per hand.

## Conclusion

The repository has the pieces of a plausible force-aware grasping system, especially the MIA `ros2_control` interface and the pure force-aware closure state machine. However, the current deployed graph is inconsistent: the main launch uses the wrong hand driver path, key hardware nodes are missing, the planner output type does not match consumers, controller switching is tracked incorrectly, and the pipeline manager has transition-time bugs.

I would not deploy this to the actual MIA hand as-is. The first deployment milestone should be much narrower: bring up only `ros2_control`, the MIA hand, the joint state broadcaster, one safe position controller, and a hardware smoke test. After force readings are proven in `/joint_states.effort` and calibrated, add the proximity controller and final closure. The full EMG/segmentation/planning stack should come last, after the hand-control loop is demonstrably safe and test-covered.
