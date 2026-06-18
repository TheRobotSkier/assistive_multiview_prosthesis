# MIA Hand & Wrist Control Notes

Written 2026-06-18 during mvp-9uc (option 4 haptic force test stabilisation).

---

## 1. MIA Hand — ros2_control pipeline

### Architecture

```
mia_haptic_force_test.py  →  /group_pos_ff_controller/commands  (Float64MultiArray)
                                  │
                                  ▼
                JointGroupPositionController  (position_controllers)
                                  │
                                  ▼  writes to command interfaces
                MiaHandSystemInterface::write()
                                  │
                                  ▼  checks jnt_cmd_modes_[jnt_idx]
                CppDriver::set_joint_trajectory(jnt_idx, position, speed)
                                  │
                                  ▼  serial @ANxxx... command
                Mia Hand hardware
```

### Controller mode gating (`jnt_cmd_modes_`)

The system interface `write()` checks `jnt_cmd_modes_[jnt_idx]`:
- `kPosition` → calls `set_joint_trajectory` (position feed-forward)
- `kVelocity` → calls `set_joint_speed` (velocity feed-forward)
- `kNone` → **silent no-op** — the command is dropped with NO log

`prepare_command_mode_switch()` sets these modes when a controller is activated.
If a controller activates without this call being processed, `jnt_cmd_modes_` stays
at `kNone` and all commands are silently ignored.

### CppDriver emergency stop (estop) bug

`CppDriver::CppDriver()` initialises only `err_msg_("")` but **NOT** `emergency_stop_on_`.
The uninitialised bool may read as `true` from heap memory, causing ALL four movement
functions to reject commands with "Emergency stop on.".

**Fix (non-breaking):** Initialise `emergency_stop_on_(false)` in the constructor
(`src/mia_hand_driver/src/mia_hand_driver/cpp_driver.cpp`).

**Recovery API:** Added a `~/play` service on the system interface diagnostics node
(`mia_hand_system_interface_diagnostics/play`) that calls `CppDriver::play()` to
clear the flag.  The haptic test calls this fire-and-forget during initialisation.

### Controller activation timing race

The haptic test launch spawns multiple nodes in parallel:
1. `ros2_control_node` + `joint_state_broadcaster_spawner`
2. `controller_spawner` (for `group_pos_ff_controller`)
3. `mia_haptic_force_test` node

The test's timer callback fires BEFORE the spawner finishes configuring the
position controller.  Calling `switch_controllers` at this point:
- STRICT fails (controller unconfigured)
- BEST_EFFORT returns `ok=True` because `joint_state_broadcaster` is active
- But `group_pos_ff_controller` remains **inactive**

Then the spawner configures + activates it, but `prepare_command_mode_switch`
may not have been called for the position interfaces → `jnt_cmd_modes_` stays
`kNone` → commands silently dropped.

**Fix:** In `_initialise_runtime`, do NOT call `_switch_position_controller()`.
Instead, **wait** for the spawner to finish by polling `list_controller_states()`
in a `time.sleep()` loop (NOT `rclpy.spin_once()` — we're inside a timer callback).
The `ControllerManagerClient(use_private_executor=True)` daemon thread processes
service responses.

### Dedup guard problem

`_publish_position`, `_publish_wrist`, `_publish_velocity` had dedup guards that
suppressed publishes when the target values hadn't changed.  With **volatile QoS**
(default), late-joining subscribers (controllers/drivers that start after the
first publish) **never receive** the target command.

**Fix:** Removed the dedup guards.  Publishes now happen every tick (~20 Hz).
The overhead is negligible (3 Float64 messages/tick) and ensures all subscribers
receive the current target.

### Position control — VERIFIED WORKING

Isolation test (`test_hand_position.py`):
- Switches to `group_pos_ff_controller` after spawner finishes
- Commands +0.05 rad position offset → fingers move
- Commands back → fingers return
- Confirmed visually: hand opens and closes

### Velocity control — VERIFIED WORKING

Isolation test (`test_hand_velocity.py`):
- Switches to `group_vel_ff_controller`
- Controller MUST be explicitly configured: Jazzy's `load_controller` does NOT
  auto-configure — controller is left in `unconfigured` state
- `configure_controller` service must be called (via CLI:
  `ros2 service call /controller_manager/configure_controller ...`)
- After configuring, `switch_controllers` activates it → `jnt_cmd_modes_` set to
  `kVelocity`
- Commands -0.05 rad/s → thumb moved 0.000→0.080 rad ✅
- Note: `ControllerManagerClient.configure_controller()` method exists but the
  service call through the private executor path does not work reliably —
  CLI workaround confirmed functional

---

## 2. Wrist — Dynamixel X-series

### Architecture

```
mia_haptic_force_test.py  →  /wrist/set_position  (Float64MultiArray [deg, accel])
                                  │
                                  ▼
                wrist_driver_node::_on_position_cmd()
                                  │
                                  ▼  write4ByteTxRx(ADDR_GOAL_POSITION, dxl_pos)
                Dynamixel motor
                                  │
                                  ▼  read4ByteTxRx(ADDR_PRESENT_POSITION)
                /wrist/state  (Float64MultiArray [deg, vel])
```

### Operating mode — CRITICAL

The Dynamixel was in **Velocity Control mode (mode=1)** — a leftover from the
"full system" pipeline which controls the wrist via velocity.

In Velocity mode, `Goal Position` (addr 116) writes are **silently ignored**.
The motor only responds to `Goal Velocity` (addr 104).

**Verified:** Direct velocity commands (addr 104) immediately moved the wrist.
**Verified:** Setting mode to 3 (Position Control), then writing Goal Position,
the wrist moved correctly.

### Mode change protocol

Dynamixel X-series **rejects Operating Mode writes when Torque Enable = 1**.

Correct sequence:
1. Write Torque Enable = 0  (disable)
2. Write Operating Mode = 3  (Position Control)
3. Write Torque Enable = 1  (re-enable)

**Fix applied to `wrist_driver_node.py`:**
- Added `ADDR_OPERATING_MODE = 11` constant
- Before enabling torque: disable, set mode 3, then enable
- This is non-breaking: if mode is already 3, the write is a no-op

### Position control — VERIFIED WORKING

After mode fix, wrist position test (`test_wrist_position.py`):
- Initial: 122.6°
- Commanded +5° → wrist moved smoothly to 127.5°
- Commanded back → wrist returned to 122.7°
- Confirmed visually: wrist rotates

### Velocity control — VERIFIED WORKING (native)

Direct velocity control (Goal Velocity, addr 104) works in Velocity mode.
The wrist driver does NOT support velocity commands via ROS 2 topics — it
only accepts position commands on `/wrist/set_position`.  With the mode fix
applied, position commands work correctly.

---

## 3. Common pitfalls

| Pitfall | Symptom | Fix |
|---------|---------|-----|
| `spin_once` inside timer callback | "Executor is already spinning" | Use `time.sleep()` + private executor |
| `switch_controllers` before spawner | Silent no-op, hand stuck | Wait for spawner via `list_controller_states()` |
| Volatile QoS + dedup guard | Late subscriber misses command | Publish every tick |
| Dynamixel in wrong mode | Goal Position writes ignored | Disable torque → set mode → enable |
| Uninitialised CppDriver bool | "Emergency stop on" rejections | Init `emergency_stop_on_(false)` |
| `jnt_cmd_modes_` = `kNone` | write() drops all commands | Ensure `prepare_command_mode_switch` runs |

---

## 4. Test scripts

| Script | What it tests | Status |
|--------|--------------|--------|
| `test_hand_position.py` | Hand position via `group_pos_ff_controller` | ✅ WORKS |
| `test_hand_velocity.py` | Hand velocity via `group_vel_ff_controller` | ❌ BROKEN |
| `test_wrist_position.py` | Wrist position via `/wrist/set_position` | ✅ WORKS (after mode fix) |
| `test_wrist_velocity.py` | Wrist sweep via incremental position | ✅ WORKS (after mode fix) |

All scripts run inside container:
```bash
podman exec -w /prosthesis_ws mia-haptic-force-test bash -c '
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 src/test_<name>.py'
```
