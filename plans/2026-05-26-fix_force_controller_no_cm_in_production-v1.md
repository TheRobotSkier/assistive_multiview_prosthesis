# Fix Force Controller: No controller_manager in Production Pipeline

## Objective

Fix the force controller's fundamental incompatibility with the production pipeline. The production pipeline (`pipeline.launch.py`) uses `mia_hand_driver` + `command_bridge` instead of `ros2_control`'s `controller_manager`. The force controller tries to call `controller_manager` services that don't exist, causing all controller switches to fail.

## Root Cause

**The `controller_manager` node is never launched in the production pipeline.** It only exists in `emg_grasp_test.launch.py` which includes `mia_hand_system_interface_launch.py`.

In production:
- `command_bridge_node.py` subscribes only to per-finger **position** topics (`/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands`)
- It translates those to raw driver service calls (`joints/*/set_trajectory`)
- There is NO velocity command forwarding — the bridge only handles position
- There is NO `controller_manager` to switch between velocity and position mode

The force controller:
- Publishes velocity commands to `/group_vel_ff_controller/commands` (line 157) — **nobody subscribes to this**
- Publishes position commands to `/group_pos_ff_controller/commands` (line 158) — **nobody subscribes to this either** (command_bridge subscribes to per-finger topics, not group topics)
- Tries to call `/controller_manager/switch_controller` — **service doesn't exist**

## Implementation Plan

- [ ] **Task 1. Add `use_controller_manager` parameter to `force_controller_node.py`**
  - Add `self.declare_parameter("use_controller_manager", True)` in `__init__`
  - Store as `self._use_cm`
  - When `False`, skip all `ControllerManagerClient` calls (no service discovery, no switching)
  - Rationale: The force controller needs to work in both architectures. A parameter lets the launch file control the behavior.

- [ ] **Task 2. Make `_finish_activate()` skip controller switching when `use_controller_manager=False`**
  - When `self._use_cm` is `False`, skip the `switch_controllers()` call entirely
  - Still set `_ramp_idx`, `_hand_phase = "CLOSING"`, etc. so the control loop runs
  - The velocity commands will be published to `/group_vel_ff_controller/commands` — but since nobody subscribes to that in production, we also need Task 3
  - Rationale: The control logic (ramp, force hold, etc.) should work regardless of whether controller_manager exists.

- [ ] **Task 3. Add per-finger velocity-to-position conversion when `use_controller_manager=False`**
  - When there's no controller_manager, the force controller must publish per-finger **position** commands to the individual topics that `command_bridge` subscribes to: `/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands`
  - Convert velocity commands to position deltas: `new_pos = current_pos + velocity * dt`
  - Publish position commands to per-finger topics instead of the group velocity topic
  - Add per-finger position publishers for the individual topics
  - Rationale: The `command_bridge` only subscribes to per-finger position topics. Without ros2_control, velocity commands go nowhere. We need to integrate velocity into position and publish on the topics the bridge actually listens to.

- [ ] **Task 4. Make `_reset_controller()` skip controller switching when `use_controller_manager=False`**
  - When `self._use_cm` is `False`, skip the `switch_controllers()` call
  - Still publish open positions and reset internal state
  - Rationale: Symmetric with Task 2.

- [ ] **Task 5. Make initialization skip `wait_for_services()` when `use_controller_manager=False`**
  - Don't create `ControllerManagerClient` at all when `self._use_cm` is `False`
  - Don't call `wait_for_services()`
  - Rationale: Avoids the 10-second timeout warning at startup when we know controller_manager won't exist.

- [ ] **Task 6. Set `use_controller_manager=False` in `pipeline.launch.py`**
  - In the force_controller node declaration (around line 362-370), add `"use_controller_manager": False` to the parameters
  - Rationale: The production pipeline uses command_bridge, not ros2_control.

- [ ] **Task 7. Update tests if needed**
  - Verify existing tests still pass
  - The mock-based tests mock all ROS imports so they should be unaffected
  - Rationale: Ensure no regressions.

## Verification Criteria

- [ ] Force controller starts without the 10-second `wait_for_services` timeout warning in production pipeline
- [ ] No "controller_manager services not available" errors in production pipeline
- [ ] When GRASPING is entered, velocity ramp translates to per-finger position commands published to `/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands`
- [ ] `command_bridge` receives and forwards these position commands to the Mia Hand driver
- [ ] Force hold and release phases work correctly with position-based control
- [ ] `emg_grasp_test.launch.py` still works with `use_controller_manager=True` (the default)

## Potential Risks and Mitigations

1. **Velocity-to-position integration drift**: Converting velocity to position deltas can accumulate error over time
   Mitigation: The force controller already has force-based feedback loops (deadzone hold, force thresholds) that will correct for drift. The position is bounded by `joint_position_limits` and `joint_position_min`.

2. **Per-finger vs group command semantics**: The current code publishes group commands (3 values in one message). Per-finger topics expect single values.
   Mitigation: Create 3 separate publishers, one per finger topic, each publishing a single-value `Float64MultiArray`.

3. **Timing mismatch**: The `_control_tick()` runs at a fixed rate. Position deltas depend on `dt` being consistent.
   Mitigation: Use `self._dt` (already computed from `update_rate_hz`) for the velocity-to-position conversion.

4. **The `command_bridge` has coalescing logic**: It drops commands that haven't changed by more than `command_epsilon` within `min_command_interval_s`.
   Mitigation: The force controller's velocity ramp produces changing positions every tick, so commands will be forwarded. During hold phase with zero velocity, positions stabilize and coalescing is appropriate.

## Alternative Approaches

1. **Add velocity topic support to `command_bridge`**: Modify command_bridge to also subscribe to velocity topics and convert to position internally. This is cleaner architecturally but changes the bridge's responsibility.

2. **Replace `command_bridge` with `ros2_control` in production**: Include `mia_hand_system_interface_launch.py` in `pipeline.launch.py`. This would make the full ros2_control stack available but requires removing the raw driver and ensuring the hardware interface is production-ready.

3. **Make force controller publish directly to driver services**: Bypass command_bridge entirely and call `joints/*/set_trajectory` services directly. This duplicates the bridge's logic and tightens coupling to the driver.

**Recommended approach**: Tasks 1-6 (parameter-gated mode with velocity-to-position conversion in the force controller). It's the least invasive change — the force controller already has all the state it needs (current positions from `/joint_states`), and the conversion is straightforward.
