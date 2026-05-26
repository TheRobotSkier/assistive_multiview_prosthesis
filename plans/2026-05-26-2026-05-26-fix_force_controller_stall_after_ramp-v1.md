# Fix Force Controller Stalling After Ramp Completion

## Objective

The force controller stops closing the hand after the velocity ramp completes (step 5/5). The root cause is that `_publish_velocity()` de-duplicates identical velocity commands, but in command_bridge mode each tick needs to re-publish the integrated position. After the first post-ramp publish, the de-duplication suppresses all subsequent commands, so the hand stops moving.

## Root Cause Analysis

The flow after ramp completion:
1. `_run_closing()` enters the `else` branch (steady-state), calls `_publish_velocity([0.25, 0.25, 0.25])`
2. `_publish_velocity()` computes `new_pos = self._positions[i] + 0.25 * 0.05` and publishes per-finger
3. Next tick (0.05s later): `_publish_velocity([0.25, 0.25, 0.25])` is called again
4. **De-duplication check** (line 695-697): `velocities == self._last_vel_cmd` → `True` → **returns without publishing**
5. The hand never moves further because no new position commands are sent

In ros2_control mode, this de-duplication is fine because the velocity controller continuously applies the last velocity command. But in command_bridge mode, position commands are one-shot — each position command moves to a target and stops.

## Implementation Plan

- [x] **Task 1. Track integrated position internally instead of relying on `/joint_states` for velocity integration**
  In `force_controller_node.py`:
  - Add `self._integrated_positions: list[float] = list(self._open_positions)` to internal state
  - When `_hand_phase` transitions to "CLOSING" (in `_finish_activate()`), initialize `_integrated_positions` from `self._positions` (current joint positions)
  - In `_publish_velocity()` command_bridge mode: use `self._integrated_positions[i]` instead of `self._positions[i]` for the integration base, then update `self._integrated_positions[i] = new_pos` after computing it
  - Rationale: This makes the integration independent of `/joint_states` feedback latency. Each tick advances the position by `vel * dt` from the last commanded position.

- [x] **Task 2. Remove velocity de-duplication in command_bridge mode**
  In `_publish_velocity()`:
  - Skip the de-duplication check (`if not force and self._last_vel_cmd...`) when `self._use_cm` is `False`
  - Rationale: In command_bridge mode, each tick must publish a position command because the command_bridge doesn't continuously apply velocity — it's a one-shot position target. De-duplication prevents the hand from continuing to close.

- [x] **Task 3. Reset integrated positions on contact and on reset**
  - In `_run_closing()` when contact is detected (transition to FORCE_HOLD): snap `_integrated_positions` to `self._positions` (actual hardware position)
  - In `_reset_controller()`: reset `_integrated_positions` to `self._open_positions`
  - In `_activate_controller()`: reset `_integrated_positions` to current `self._positions`
  - Rationale: Keeps integrated position synchronized with reality at phase transitions.

- [x] **Task 4. Verify with existing tests**
  - Run `test/test_controller_manager_client.py` — should still pass
  - Rationale: No API changes to `ControllerManagerClient`.

## Verification Criteria

- [ ] After ramp step 5/5, force controller continues publishing position commands every tick
- [ ] Hand continues closing until contact is detected (force >= 300 or position >= 1.5)
- [ ] FORCE_HOLD phase activates and adjusts velocity to reach target forces
- [ ] Existing unit tests pass

## Potential Risks and Mitigations

1. **Position drift between integrated and actual positions**
   Mitigation: Integrated positions are snapped to actual positions at every phase transition (CLOSING→FORCE_HOLD, reset, activate). During CLOSING, drift is acceptable because the force feedback loop in FORCE_HOLD corrects it.

2. **Increased topic traffic** (no de-duplication)
   Mitigation: At 20 Hz with 3 fingers, that's 60 messages/sec — negligible. The proximity controller publishes at the same rate without issues.
