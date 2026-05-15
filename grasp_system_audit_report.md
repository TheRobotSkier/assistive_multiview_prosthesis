# Grasp System Architecture Audit Report

**Date:** 2026-05-14
**Repository:** `wt-hand-interface` (Multiview Prosthesis)
**Scope:** MIA Hand grasp pipeline -- architecture, control flow, deployment readiness

---

## 1. System Architecture Overview

### 1.1 High-Level Pipeline

```
EMG Band (MindRove)
      │
      ▼
┌──────────────────┐    gesture    ┌──────────────────┐
│  emg_bridge       │─────────────▶│ pipeline_manager  │  (state machine)
│  (classifier)     │              │  IDLE→SEGMENTING  │
└──────────────────┘              │  →PLANNING→APPRO..│
                                  └────────┬─────────┘
                                           │ trigger
                                           ▼
┌──────────────────┐   object    ┌──────────────────┐
│  segmentation      │──────────▶│  grasp_preshaping │  (Rust FFI .so)
│  (MinkowskiEngine) │   cloud    │  (C++ bridge)     │
└──────────────────┘              └────────┬─────────┘
                                           │ closures + wrist + hand pose
                                           ▼
                                  ┌──────────────────┐
                                  │ grasp_proximity   │
                                  │ controller        │  (approach + force-aware closure)
                                  └────────┬─────────┘
                                           │ pos/vel commands
                                           ▼
                                  ┌──────────────────┐
                                  │ ros2_control      │  (controller_manager)
                                  │ hardware interface│
                                  └────────┬─────────┘
                                           │ serial USB
                                           ▼
                                  ┌──────────────────┐
                                  │ MIA Hand (Prensilia)│
                                  │ strain gauges → force│
                                  └──────────────────┘
```

### 1.2 Package Map

| Package | Language | Role |
|---------|----------|------|
| `mia_hand_driver` | C++ | Serial protocol to MIA hand hardware |
| `mia_hand_ros2_control` | C++ | ros2_control `SystemInterface` wrapping driver |
| `mia_hand_msgs` | ROS msgs | `.msg`, `.srv`, `.action` definitions |
| `mia_hand_description` | URDF/xacro/C++ | Hand model, meshes, TF joints |
| `grasp_preshaping` | Rust + C++ bridge + Python | Grasp planning (TSDF + SMC + LUT) + force-aware closure |
| `pipeline_manager` | Python | State machine orchestrator |
| `force_controller` | Python | Independent force regulation (see issues) |
| `wrist_driver` | Python | Dynamixel wrist motor driver |
| `camera` | Python | RealSense launch, pointcloud fusion/relay, ChArUco tracking, hand pose publisher |
| `segmentation_bridge` | Python | HTTP client to MinkowskiEngine inference server |
| `emg_bridge` | Python | MindRove EMG classifier + ROS bridge |
| `twist_propagation` | Python | Hand twist estimation + collision prediction |
| `prosthesis_launch` | Python launch | Top-level bringup launch files |

### 1.3 Two Parallel Driver Paths

The system has two architectures for driving the hand, which run in conflict:

**Path A -- ros2_control (intended for force-aware control):**
```
MiaHandSystemInterface (hardware_interface::SystemInterface)
    └── embeds CppDriver
    └── exports: position, velocity, effort STATE interfaces
    └── exports: position, velocity COMMAND interfaces
    └── controllers: thumb/index/mrl_pos_ff_controller, *_vel_ff_controller
    └── joint_state_broadcaster → /joint_states (with effort)
```

**Path B -- Standalone ROS2 Driver:**
```
mia_hand_driver_node (RosDriver)
    └── embeds CppDriver
    └── provides services: GetForceData, GetJointData, SetJointTraj, etc.
    └── publishes ForceData.msg on topics
    └── does NOT use ros2_control controller_manager
```

**The launch files mix these in incompatible ways** (see Issue #1 below).

---

## 2. Critical Issues

### 2.1 [CRITICAL] Duplicate Force Controllers with No Integration

**Location:** `src/force_controller/force_controller_node.py` and `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`

Both `pipeline.launch.py` and `mock.launch.py` launch both nodes simultaneously. They operate independently, potentially fighting for the same actuators:

- **force_controller_node** subscribes to `/hand/forces` (which no node publishes), publishes to `/hand/motor_commands` (which no node consumes). It is a dead code path.
- **grasp_proximity_controller_node** subscribes to `/joint_states` for effort, publishes to ros2_control per-finger command topics. It is the *actual* force-aware path.

**Impact:** The force_controller_node is completely orphaned -- it produces no observable effect and wastes CPU cycles. If someone accidentally makes `/hand/forces` work and subscribes to `/hand/motor_commands`, it would conflict with the proximity controller.

**Recommendation:** Remove `force_controller_node` entirely, or consolidate all force regulation into the proximity controller.

---

### 2.2 [CRITICAL] pipeline.launch.py Missing ros2_control Stack

**Location:** `src/prosthesis_launch/launch/pipeline.launch.py`

The pipeline launch file launches `mia_hand_driver_node` (standalone driver, Path B) but does NOT launch:
- `controller_manager` / `ros2_control_node`
- `robot_state_publisher`
- `joint_state_broadcaster`
- Any controller spawners

However, the `grasp_proximity_controller_node` publishes to topics like `/thumb_pos_ff_controller/commands`, `/thumb_vel_ff_controller/commands`, etc. These topics only exist when `ros2_control` is running with the controller_manager spawned.

**Impact:** On deployment, the proximity controller's position/velocity commands will publish to topics that don't exist. The hand will never move through this path. You would need to launch `mia_hand_system_interface_launch.py` instead.

**Recommendation:** Either:
- (A) Replace `mia_hand_driver_node` with the full ros2_control launch (`mia_hand_system_interface_launch.py`) in `pipeline.launch.py`, or
- (B) Create a unified launch that brings up the full ros2_control stack with the pipeline nodes.

---

### 2.3 [CRITICAL] /hand/forces Topic Has No Publisher

**Location:** `src/force_controller/force_controller_node.py:70`

The force controller subscribes to `/hand/forces` (Float32MultiArray). This topic is declared in `prosthesis_config.yaml` under `finger_forces: "data_streams/fingers/forces/data"` but the actual force data path is:

- Via ros2_control: effort values appear in `/joint_states` (published by `joint_state_broadcaster`)
- Via standalone driver: `GetForceData.srv` (service call, not a topic)

No node in the codebase publishes to `/hand/forces`. The force controller will always see zero forces.

**Recommendation:** Delete this topic reference and remove the force_controller_node. Use the effort field on `/joint_states` via the proximity controller instead.

---

### 2.4 [CRITICAL] Joint Effort Units Not Calibrated

**Location:** `src/mia_hand_ros2_control/src/mia_hand_ros2_control/mia_hand_system_interface.cpp:347-353`

The force values read from the hardware and stored in `jnt_eff_state_` are raw integer values from the MIA hand firmware ADC:

```cpp
jnt_eff_state_[0] = static_cast<double>(thumb_nfor);
jnt_eff_state_[1] = static_cast<double>(index_nfor);
jnt_eff_state_[2] = static_cast<double>(mrl_nfor);
```

However, the force-aware closure profiles in `prosthesis_config.yaml` use thresholds like:
```yaml
contact_force_threshold: 50.0   # what unit? Newtons? Raw ADC?
contact_force_spike_threshold: 15.0
```

**Impact:** The force contact detection will trigger at wrong levels because the raw ADC values are compared against thresholds that may have been intended as Newtons or normalized values. The MIA hand strain gauges produce integer readings in some proprietary range (the firmware message format parses 4-digit integers at `cpp_driver.cpp:495`). These raw values could be 0-9999 or similar. A threshold of 50 raw units might trigger on noise.

**Recommendation:** Add a calibration step that either:
- (A) Maps raw ADC to Newtons via a known conversion factor, or
- (B) Tunes thresholds empirically on real hardware by recording idle noise levels and setting thresholds above noise floor.

---

### 2.5 [MAJOR] No Release/Grasp-Open Command on State Transition

**Location:** `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py:148-151` and `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`

When the pipeline_manager transitions to `RELEASING`, it sets a 1-second auto-transition to `IDLE`:
```python
self.create_timer(1.0, lambda: self._transition(State.IDLE, 'Release complete'), one_shot=True)
```

But no joint movement command is ever sent to open the hand. The `grasp_proximity_controller_node` only acts on subscription callbacks from `/grasp_preshaping/target_finger_closures` and `/grasp_preshaping/wrist_pose` -- it has no awareness of pipeline state. The `force_controller_node` ignores `RELEASING` state (it only acts in `GRASPING`/`HOLDING`).

**Impact:** Triggering an EMG "OPEN" gesture will transition the state machine to RELEASING and then IDLE, but the physical hand will remain at its last closed position.

**Recommendation:** Add a release action in the pipeline_manager that sends zero/negative position commands to the joint controllers, or subscribe the proximity controller to `/pipeline/state` and respond to `RELEASING` by commanding full-open positions.

---

### 2.6 [MAJOR] Wrist Driver and MIA Hand Serial Port Conflict

**Location:** `pipeline.launch.py:59` and `wrist_driver_node.py:64`

- The MIA hand driver defaults to `/dev/ttyUSB0` (pipeline.launch.py)
- The wrist Dynamixel driver defaults to `/dev/ttyUSB0` (wrist_driver_node.py)
- The config file maps MIA to `/dev/ttyUSB1` and wrist to `/dev/ttyUSB0`

But `pipeline.launch.py` hardcodes `serial_port: "/dev/ttyUSB0"` for the MIA hand, which conflicts with the wrist driver's default.

**Impact:** On real hardware with both devices plugged in, one of them will fail to open the port. USB device enumeration order may cause `/dev/ttyUSB0` and `/dev/ttyUSB1` to swap between boots.

**Recommendation:** Always use `config/prosthesis_config.yaml` values, add udev rules for persistent device naming (e.g., `/dev/mia_hand`, `/dev/wrist_motor`).

---

### 2.7 [MAJOR] Mock Hardware Does Not Support Force State Interfaces

**Location:** `src/mia_hand_ros2_control/description/ros2_control/mia_hand.ros2_control.xacro:16-33`

When `use_mock_hardware:=true`, the xacro uses `mock_components/GenericSystem`. The mock GenericSystem does not produce `effort` state interfaces, so `/joint_states` will never contain effort values. This means the force-aware closure logic (`grasp_proximity_controller_node`) cannot be tested in mock mode.

**Impact:** The entire force-aware closure path is untestable without real hardware attached.

**Recommendation:** Add a mock force publisher or extend the GenericSystem mock to emit effort state interfaces, or create a dedicated force mock node that publishes fake effort values to a separate topic that the proximity controller can use in mock mode.

---

### 2.8 [MAJOR] Competing Driver Interfaces -- Only One Can Own the Serial Port

**Location:** `src/mia_hand_driver/src/mia_hand_driver_node.cpp` vs `src/mia_hand_ros2_control/`

Both `mia_hand_driver_node` (RosDriver) and `MiaHandSystemInterface` instantiate `CppDriver` and attempt to open the same serial port. CppDriver uses LibSerial which can't share ports.

**Impact:** If both are launched (as in pipeline.launch.py which launches the driver node and any omitted ros2_control stack), only one succeeds. The other fails silently or blocks.

**Recommendation:** Choose one driver architecture and use it consistently. The ros2_control SystemInterface is the more modern ROS2 approach and should be the single path.

---

### 2.9 [MAJOR] No Hand Open Before Initiating Grasp (Stale Position)

**Location:** `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:266-288`

When a new grasp plan is committed (`_try_commit_plan`), the hand immediately starts closing to `partial_closure_factor * planned_closure`. If the hand is already partially closed from a previous grasp, the new grasp will start from an arbitrary starting position.

**Impact:** The predicted closure amounts from the planner assume the hand starts from fully open. The actual finger closure will be off by the residual position.

**Recommendation:** Before committing a new plan, command all joints to 0.0 (fully open) and either wait for a position-done signal or add a fixed delay before starting the approach.

---

### 2.10 [MINOR] Pipeline Launch Includes Twist Propagation When Inactive

**Location:** `pipeline.launch.py:100-105` and `config/prosthesis_config.yaml:155`

`twist_propagation` is configured with `active: false` by default and the pipeline_manager directly calls `/grasp_preshaping/compute_grasp` on gesture trigger, bypassing twist propagation entirely. But the node is still launched consuming resources.

**Impact:** No functional problem, but the holistic pipeline (twist → collision prediction → click point → segmentation → preshaping) is not wired. The `grasp_test.launch.py` actually activates twist propagation but uses it independently.

**Recommendation:** Either wire the full integration path (manager waits for twist_propagation trigger) or remove the node from the default pipeline launch.

---

## 3. Design-Level Concerns

### 3.1 State Machine Coupling

The pipeline state machine is implemented in `pipeline_manager_node.py` but three other nodes (`force_controller_node`, `grasp_proximity_controller_node`, `twist_propagation_node`) independently maintain state tracking. The state constants are duplicated as module-level integers rather than imported from a shared enum. If the pipeline_manager adds a state, other nodes silently break.

**Recommendation:** Centralize state definitions in a shared ROS2 message or a Python module imported by all nodes.

### 3.2 Serial Command Blocking in read() Loop

`CppDriver::send_command()` (`cpp_driver.cpp:1283-1339`) performs blocking serial I/O inside the ros2_control `read()` method. Each command sends a serial message, then blocks for up to 20ms waiting for ACK. With 6 commands in `read()` (joint positions + joint speeds + finger forces), that's up to 120ms of blocking time. The ros2_control loop runs at 5 Hz (200ms period), so this may fit, but any retry or timeout leaves zero margin.

**Impact:** On a noisy serial line, one timeout cascades, causing the control loop to miss cycles. Error returns are [intentionally suppressed with TODO comments](`mia_hand_system_interface.cpp:262-263`: "TODO: uncomment after fixing driver timeouts issue").

**Recommendation:** Convert the serial protocol to use async I/O or a dedicated reader thread with double-buffered state, so `read()` always returns immediately with the latest known values.

### 3.3 Rust `.so` Binary in Git

`src/grasp_preshaping/lib/libgrasp_preshaping.so` is a pre-built x86_64 shared library committed to the repository. It:

- **Will not work on ARM64 (Jetson Orin Nano)** if the pipeline is ever deployed there
- Cannot be rebuilt inside the Docker container (no Rust toolchain in Dockerfile)
- Has no CI pipeline to ensure it's current with source changes

**Recommendation:** Either add Rust compilation to the Dockerfile (install rustup, `cargo build --release`), or build separate `.so` artifacts per architecture, or use the pure-Python `force_aware_closure.py` path for the contact detection and skip the Rust pipeline when not available.

### 3.4 Force Data Staleness Handling

`grasp_proximity_controller_node.py:366-380` has a force data staleness check:

```python
if force_stale:
    if elapsed * 1000.0 > self._force_reading_timeout_ms:
        self._stop_all_fingers()
        return
```

If force data hasn't arrived within 500ms of entering final closure, all fingers are stopped with zero velocity (SAFETY_STOPPED). But if force data arrives late, the state machine has no path to recover -- it's stuck in SAFETY_STOPPED until a new plan is committed (`_try_commit_plan`).

**Recommendation:** Add a manual retry mechanism or automatically transition back to OPEN_LOOP/APPROACHING if force data resumes within a grace period.

### 3.5 Segmentation Inference Server Dependency

The `segmentation_bridge` node requires a separate Docker container running a MinkowskiEngine inference server on Python 3.8. If this server is not running, the entire pipeline stalls at the SEGMENTING state forever. There's no timeout or fallback.

**Recommendation:** Add a timeout for the segmentation phase. If the segmented object cloud is not received within N seconds, transition back to IDLE and log an error.

---

## 4. Deployment & Operational Concerns

### 4.1 USB Serial Enumeration

The system relies on `/dev/ttyUSB0` and `/dev/ttyUSB1` for MIA hand and wrist motor respectively. On Linux, these names are assigned in USB enumeration order and can change between boots. No udev rules exist.

**Recommendation:** Create udev rules matching device serial numbers or USB port paths to create persistent symlinks (`/dev/mia_hand`, `/dev/wrist_motor`).

### 4.2 No Systemd/Daemon Management

The launch process requires manual SSH + docker-compose commands. No systemd unit, no auto-restart on crash, no health checks.

**Recommendation:** Consider systemd service units for the docker-compose services with restart policies.

### 4.3 Emergency Stop Architecture

There are three emergency stop mechanisms:
1. CppDriver::emergency_stop() -- serial command to stop all motors, software flag prevents further movement until play() is called
2. force_controller_node -- force exceeded max_force, backs off by 0.1
3. force_aware_closure.py -- SAFETY_STOPPED state with velocity=0

But #2 (force_controller_node) is on the dead code path (see Issue 2.3), and #1 is never exposed through ROS2 (no service, no topic). Only #3 actually works, and it's per-finger only.

**Recommendation:** Expose `emergency_stop()` and `play()` as ROS2 services. Wire them to a hardware stop button or a keyboard trigger.

### 4.4 Container Lifetime

The Makefile hardcodes a 30-minute container lifetime for RViz containers. Useful for preventing abandoned long-running processes, but if the pipeline test exceeds 30 minutes, RViz will silently die.

### 4.5 No smoke test validates the integrated grasp path

`scripts/run_tests.sh` tests only build, launch syntax, .so loading, and individual node startup. There's no end-to-end integration test that exercises the pipeline from EMG gesture to joint commands.

---

## 5. Summary of Recommendations (Priority Order)

| # | Priority | Problem | Recommendation |
|---|----------|---------|----------------|
| 1 | **P0** | Two incompatible driver paths | Choose ros2_control as the single architecture; remove `mia_hand_driver_node` from pipeline launch |
| 2 | **P0** | Missing ros2_control stack in pipeline launch | Add controller_manager, robot_state_publisher, controller spawners to `pipeline.launch.py` |
| 3 | **P0** | Force controller is orphaned (dead code) | Remove `force_controller_node.py` or consolidate into proximity controller |
| 4 | **P0** | No release/open on RELEASING state | Send full-open joint commands when pipeline enters RELEASING |
| 5 | **P0** | Force units not calibrated | Record idle noise floor on real hardware; map raw ADC to Newtons via calibration |
| 6 | **P1** | Serial command blocking in control loop | Async I/O or dedicated reader thread for CppDriver |
| 7 | **P1** | No hand open before grasp | Command zero position and wait before starting approach |
| 8 | **P1** | Simulation cannot test force-aware closure | Add force mock or extend GenericSystem mock |
| 9 | **P1** | Serial port naming fragile | udev rules for persistent device names |
| 10 | **P2** | State machine constants duplicated | Centralize state enum in shared Python module |
| 11 | **P2** | Segmentation timeout missing | Add timeout guard for segmentation phase |
| 12 | **P2** | Emergency stop not exposed to ROS2 | Expose stop/play as ROS2 services |
| 13 | **P3** | No end-to-end integration test | Create pipeline integration test with mock hardware |
| 14 | **P3** | Rust .so platform-dependent | Add Rust toolchain to Dockerfile or add runtime feature gating |

---

## 6. Alternative Approaches to Consider

### 6.1 Direct Force Control via Velocity Ramping

Instead of the current position-based approach (plan a closure, close to it, then switch to velocity for force-aware contact), use pure velocity control from the start:

```
EMG trigger → compute hand pose target → velocity ramp toward closure
→ continuously read force → decelerate when contact detected → hold
```

This eliminates the position/velocity controller switch (which has race-condition potential) and avoids the "past predicted closure" edge case in the state machine.

### 6.2 Impedance/Admittance Control

The MIA hand's motor current control could be used to implement a virtual spring-damper (impedance control). Setting the motor current limit proportional to the desired grasp force would make the hand naturally compliant, avoiding the need for explicit force feedback loops. The ros2_control `pid_controller` with effort command interface could be used for this if the hardware supports effort commands.

### 6.3 Pre-Shape Only, No Force Feedback

If force feedback proves unreliable (strain gauge noise, calibration drift), the system could fall back to position-only control with conservative closure amounts. The planner already computes predicted closure positions from the TSDF/contact LUT. Closing to exactly the predicted position (with a small safety margin) may be sufficient for many objects without force sensing.

### 6.4 Event-Driven Instead of Timer-Driven Control

The current architecture uses fixed-rate timers (10 Hz control loop, 5 Hz state publisher). A more responsive system could use event-driven callbacks: position reached → next stage, force contact → stop immediately, etc. This would reduce latency and remove the risk of over-closing between timer ticks.
