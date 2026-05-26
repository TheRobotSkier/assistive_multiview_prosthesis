# Fix Force Controller for Production Pipeline (No controller_manager)

## Objective

Make the force controller work in the production pipeline where there is no `controller_manager`. The production pipeline uses `command_bridge` which only subscribes to per-finger **position** topics. The force controller currently publishes to group velocity/position topics that nobody reads, and tries to call services that don't exist.

## Strategy

Add a `use_controller_manager` parameter. When `False` (production mode):
- Skip all `ControllerManagerClient` calls (no service discovery, no switching)
- Convert velocity commands to position deltas and publish to per-finger position topics
- The `command_bridge` will forward these to the hardware, same as the proximity controller does

When `True` (emg_grasp_test mode), behavior is unchanged.

## Implementation Plan

- [x] **Task 1. Add `use_controller_manager` parameter and per-finger topic parameters**
  In `force_controller_node.py` `__init__`:
  - Add `self.declare_parameter("use_controller_manager", True)` 
  - Add per-finger position topic parameters:
    - `finger_cmd_topics` default `["/thumb_pos_ff_controller/commands", "/index_pos_ff_controller/commands", "/mrl_pos_ff_controller/commands"]`
  - Store `self._use_cm = self.get_parameter("use_controller_manager").value`
  - Store the per-finger topic list as `self._finger_cmd_topics`
  - Rationale: Parameter-gated mode lets the same node work in both architectures. The per-finger topics match exactly what `command_bridge` subscribes to (confirmed in `command_bridge_node.py:65-67`).

- [x] **Task 2. Conditionally create `ControllerManagerClient`**
  In `__init__`, wrap lines 249-254:
  - Only create `ControllerManagerClient` and call `wait_for_services()` when `self._use_cm` is `True`
  - Set `self._cm_client = None` when `False`
  - Rationale: Avoids the 10-second timeout warning at startup when we know controller_manager won't exist.

- [x] **Task 3. Add per-finger position publishers**
  In `__init__`, after the existing publishers (line 288):
  - Create 3 publishers, one per finger topic from `self._finger_cmd_topics`
  - Store as `self._finger_pubs: list[Publisher]`
  - Rationale: These are the topics `command_bridge` subscribes to. The proximity controller uses the same topics and works.

- [x] **Task 4. Modify `_publish_velocity()` to do velocity→position integration when no CM**
  In `_publish_velocity()` (line 672):
  - When `self._use_cm` is `True`: existing behavior (publish to group velocity topic)
  - When `self._use_cm` is `False`: for each finger, compute `new_pos = self._positions[i] + velocities[i] * self._dt`, clamp to `[joint_pos_min, joint_pos_max]`, publish as single-value `Float64MultiArray` to the per-finger topic
  - Rationale: The `command_bridge` only forwards position commands. We integrate velocity into position using the known joint positions from `/joint_states` and the control loop's `dt`. The force feedback loop corrects any drift.

- [x] **Task 5. Modify `_publish_position()` to publish per-finger when no CM**
  In `_publish_position()` (line 683):
  - When `self._use_cm` is `True`: existing behavior (publish to group position topic)
  - When `self._use_cm` is `False`: publish each position value to its per-finger topic
  - Rationale: Same as Task 4 — command_bridge needs per-finger topics.

- [x] **Task 6. Skip controller switching in `_finish_activate()` when no CM**
  In `_finish_activate()` (line 416):
  - When `self._use_cm` is `False`: skip the entire `switch_controllers()` block, go straight to setting `_ramp_idx`, `_hand_phase = "CLOSING"`, etc.
  - Rationale: No controller_manager means no switching needed. The control loop will publish position deltas via per-finger topics.

- [x] **Task 7. Skip controller switching in `_reset_controller()` when no CM**
  In `_reset_controller()` (line 460):
  - When `self._use_cm` is `False`: skip the `switch_controllers()` block
  - Rationale: Symmetric with Task 6.

- [x] **Task 8. Skip controller switching in `destroy_node()` when no CM**
  In `destroy_node()` (line 710):
  - Guard the `self._cm_client.services_ready()` check with `self._use_cm and self._cm_client is not None`
  - Rationale: Prevents AttributeError when `_cm_client` is None.

- [x] **Task 9. Set `use_controller_manager: false` in production config**
  In `config/prosthesis_config.yaml`, add to the `force_controller` section:
  ```yaml
  use_controller_manager: false
  finger_cmd_topics:
      - "/thumb_pos_ff_controller/commands"
      - "/index_pos_ff_controller/commands"
      - "/mrl_pos_ff_controller/commands"
  ```
  Rationale: The production pipeline uses `command_bridge`, not `ros2_control`.

- [x] **Task 10. Run existing tests to verify no regressions**
  - Run `test/test_controller_manager_client.py` — should still pass (API unchanged)
  - Rationale: Ensure the CM path still works for `emg_grasp_test`.

## Verification Criteria

- [ ] Force controller starts without the 10-second `wait_for_services` warning
- [ ] No "controller_manager services not available" errors during GRASPING/RELEASING transitions
- [ ] When GRASPING: velocity ramp produces per-finger position commands on `/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands`
- [ ] `command_bridge` forwards these to the Mia Hand driver (same path as proximity controller)
- [ ] Force hold and release phases work
- [ ] Existing unit tests pass

## Potential Risks and Mitigations

1. **Velocity→position integration drift**: Accumulated error over time
   Mitigation: Force feedback loop corrects drift. Positions are clamped to `[min, max]`. The control rate is 20 Hz with typical closing velocities of 0.1-0.3 rad/s, so position deltas per tick are 0.005-0.015 rad — well within hardware precision.

2. **Stale position data**: If `/joint_states` stops updating, integrated positions diverge from reality
   Mitigation: Already handled — the control loop checks `_joint_pos_received` and stale force data (line 520-537). Add a similar check for stale positions if needed.

3. **Per-finger topic format mismatch**: `command_bridge` expects `Float64MultiArray` with a single value per message
   Mitigation: The proximity controller already publishes this format successfully. We'll match it exactly.
