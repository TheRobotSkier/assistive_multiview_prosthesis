# Goal: MIA Hand Hardware Interface — Force/Effort Controller

## Purpose

Complete the MIA hand hardware interface in `src/mia_hand_ros2_control/` so it
exposes force/effort state interfaces and can accept effort commands through the
ROS2 Control framework. This enables a complete force-feedback grasp controller.

The `test1-daniel` branch (remote: `assistive-multiview-prosthesis/test1-daniel`)
contains software verification tests (`tests/test1_software_verification/`) that
verify the grasp preshaping pipeline. The work here must ensure the hardware
interface is complete enough to run those tests on real hardware.

## Hardware

- **MIA Hand** — prosthetic hand by Prensilia, connected via USB serial
  (`/dev/ttyUSB0` by default). Has 4 joints: `j_thumb_fle`, `j_index_fle`,
  `j_mrl_fle`, `j_thumb_opp`. Strain gauges on each finger provide force feedback.
- **Host computer** — x86 PC running the ROS2 controller stack in Docker (podman).

## Current State

The `mia_hand_ros2_control` package (`src/mia_hand_ros2_control/`) already has:
- ✅ Hardware interface (`mia_hand_system_interface.cpp`) with `position` and
  `velocity` state/command interfaces
- ✅ Position controllers (feedforward, PID, trajectory) — working
- ✅ Velocity controllers — working
- ✅ Driver layer (`mia_hand_driver`) reading force data from hardware via
  `GetForceData` service and publishing on `ForceData.msg`
- ❌ **Missing:** `effort` hardware interface state (force reading exposed to
  ros2_control framework)
- ❌ **Missing:** Force/effort command interface (sending effort setpoints to hardware)
- ❌ **Missing:** Effort controller configuration in `mia_hand_controllers.yaml`
- ❌ **Missing:** Any launch/bringup for the complete force-feedback grasp loop

## Architecture

```
External controller (grasp preshaper)
        │
        │  /force_controller/command  (effort setpoints)
        ▼
ros2_control controller manager
        │
        │  effort command interface
        ▼
MiaHandSystemInterface (hardware_interface::SystemInterface)
        │
        │  mia_hand_driver::CppDriver  (serial USB)
        ▼
MIA Hand hardware (strain gauges → force readings)
        │
        │  effort state interface (force feedback)
        ▼
ros2_control → joint_state_broadcaster → /joint_states
```

## What Needs to Be Implemented

### 1. Effort State Interface in Hardware Interface
**File:** `src/mia_hand_ros2_control/src/mia_hand_system_interface.cpp`

In `export_state_interfaces()`, add `hardware_interface::HW_IF_EFFORT` for each
finger joint alongside the existing `position` and `velocity` exports. In `read()`,
populate the effort state values by calling the driver's force reading API.

Check `mia_hand_driver/include/mia_hand_driver/cpp_driver.hpp` for the force-reading
method — the driver already supports it (see `mia_hand_msgs/msg/ForceData.msg` and
`mia_hand_msgs/srv/GetForceData.srv`).

### 2. Effort Command Interface (optional, for direct effort control)
If direct effort (torque) control is supported by the hardware, also export
`hardware_interface::HW_IF_EFFORT` in `export_command_interfaces()` and wire it
to the appropriate driver call in `write()`. If not supported, document why and
skip.

### 3. Controller Configuration
**File:** `src/mia_hand_ros2_control/config/mia_hand_controllers.yaml`

Add force/effort controllers:
```yaml
# Force state broadcaster (reads effort state interface)
force_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster  # with effort interface

# OR a dedicated effort controller if direct effort commands are used:
group_effort_controller:
  type: effort_controllers/JointGroupEffortController
```

### 4. Topic Interface Check

The grasp preshaping pipeline (`src/grasp_preshaping/`) sends grasp commands.
Verify the expected topic names match what the controllers advertise. The
ros2_control framework publishes controller inputs on:
```
/mia_hand/<controller_name>/commands   (or similar namespace)
```

Check `src/prosthesis_launch/launch/` and `src/grasp_preshaping/` for how the
upper layers send commands, and ensure the controller names/namespaces match.

### 5. Integration with test1-daniel Verification Tests

The `test1-daniel` branch (check out or cherry-pick from
`assistive-multiview-prosthesis/test1-daniel`) contains:
```
tests/test1_software_verification/
  run.py              # main test runner
  hand_approaches.py  # hand grasp approach sequences
  ffi_bridge.py       # FFI bridge to the hand controller
```

Ensure the hardware interface, when running in mock mode (`use_mock_hardware:=true`),
passes the test1 software verification suite without failures.

## Definition of Done

- [ ] `effort` state interfaces exported and populated with real force values
- [ ] Force values visible on `/joint_states` (effort field)
- [ ] Effort controller(s) configured and spawnable without errors
- [ ] Can receive grasp commands from `/grasp_preshaping` or equivalent topics
- [ ] Mock hardware mode passes `tests/test1_software_verification/run.py`
- [ ] Real hardware mode: force values update during a simple open/close cycle

## Key Files to Read First

```
src/mia_hand_ros2_control/src/mia_hand_system_interface.cpp  # hardware interface impl
src/mia_hand_ros2_control/include/                           # interface headers
src/mia_hand_ros2_control/config/mia_hand_controllers.yaml   # controller config
src/mia_hand_driver/include/mia_hand_driver/cpp_driver.hpp   # driver API
src/mia_hand_msgs/msg/ForceData.msg                          # force message type
src/mia_hand_msgs/srv/GetForceData.srv                       # force service
src/grasp_preshaping/                                        # command source
src/prosthesis_launch/launch/                                # bringup launch files
```

Also fetch and study the test1-daniel branch work:
```bash
git fetch assistive-multiview-prosthesis
git show assistive-multiview-prosthesis/test1-daniel:tests/test1_software_verification/hand_approaches.py
git show assistive-multiview-prosthesis/test1-daniel:tests/test1_software_verification/ffi_bridge.py
```

## No Jetson Needed for This Task

The MIA hand connects directly to the host x86 PC via USB. The Jetson is not
involved. You may still use the queue for any tasks that happen to require Jetson
access, but primary development and testing happens locally.

If you do need Jetson access, see `AGENTS.md` for the queue protocol.
