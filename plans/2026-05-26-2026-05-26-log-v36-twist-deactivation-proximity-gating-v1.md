# Fix Plan: Twist Deactivation on Preshaping Failure + Proximity Controller Logging Cleanup

## Objective

Fix two issues from log v36:
1. **Issue 4 (from v36 diagnostic)**: Twist propagation is not deactivated when preshaping fails (PLANNING → IDLE). It continues running, finding hits, and triggering spurious segmentation runs.
2. **Proximity controller logging outside APPROACHING**: The proximity controller's control loop continues running (logging FAR/NEAR mode, computing distances) even when the pipeline has left APPROACHING. It should only be active during APPROACHING.

## Implementation Plan

### Fix 1: Deactivate twist propagation on preshaping failure

**File**: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py:639-644`

The `_on_preshaping_response` method currently only deactivates twist on success (line 638). On failure (lines 640-641) and error (lines 643-644), it transitions to IDLE but never deactivates twist propagation.

- [ ] Add `self._deactivate_twist_propagation()` to the failure branch (line 641, after the `_transition` call) and the error branch (line 644, after the `_transition` call). This ensures twist is always stopped when preshaping completes (success or failure).

### Fix 2: Gate proximity controller control loop on APPROACHING state

**File**: `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:231-254`

The control loop at line 231 currently only gates on states 5/6/7 (GRASPING/HOLDING/VOLITIONAL). It runs in ALL other states, including IDLE, TWISTING, SEGMENTING, PLANNING, and RELEASING. This causes:
- FAR/NEAR logging spam when the pipeline is in IDLE/TWISTING
- Stale plan execution after RELEASING → IDLE
- Near zone signals published when the pipeline isn't in APPROACHING

- [ ] Change the control loop guard to **only run during APPROACHING** (state 4). Replace the current guard `if self._pipeline_state in (5, 6, 7): return` with `if self._pipeline_state != 4: return`. This means:
  - IDLE (0): loop returns early — no logging, no commands
  - TWISTING (1): loop returns early
  - SEGMENTING (2): loop returns early
  - PLANNING (3): loop returns early
  - APPROACHING (4): loop runs — computes distance, publishes commands
  - GRASPING (5): loop returns early (force controller owns joints)
  - HOLDING (6): loop returns early
  - VOLITIONAL (7): loop returns early
  - RELEASING (8): loop returns early

- [ ] When the control loop returns early due to state != APPROACHING, also reset `_is_near` to `False` and publish `Bool(False)` on `/proximity/near_zone_entered` if `_is_near` was `True`. This ensures the near zone signal is always cleared when leaving APPROACHING.

## Verification Criteria

- [ ] When preshaping fails, twist propagation is deactivated (log should show "Twist propagation deactivated")
- [ ] When preshaping succeeds, twist propagation is deactivated (existing behavior, unchanged)
- [ ] Proximity controller does NOT log FAR/NEAR mode when pipeline is in IDLE/TWISTING/SEGMENTING/PLANNING/RELEASING
- [ ] Proximity controller DOES log FAR/NEAR mode when pipeline is in APPROACHING
- [ ] Near zone signal is cleared (Bool(False)) when pipeline leaves APPROACHING

## Potential Risks and Mitigations

1. **Risk**: If the pipeline transitions APPROACHING → GRASPING very quickly, the near zone `Bool(False)` might be published after `Bool(True)`, confusing the pipeline manager.
   **Mitigation**: Only publish `Bool(False)` when the state is NOT APPROACHING AND NOT GRASPING (i.e., don't clear near zone when transitioning to GRASPING — only clear on IDLE/RELEASING/etc.). This can be achieved by checking `self._pipeline_state not in (4, 5)`.

2. **Risk**: If the pipeline manager sends a preshaping request while twist propagation is already deactivated (e.g., from a previous failure), the deactivation call on failure is a no-op. This is harmless.

## Alternative Approaches

1. **Clear the proximity controller's plan on state exit** (instead of gating the control loop): This would also work but requires more careful coordination — the plan might arrive before the pipeline enters APPROACHING, and we'd need to keep it buffered. Gating the control loop is simpler and more robust.
2. **Subscribe to pipeline state in twist propagation** (instead of deactivating via service call): More complex, requires twist propagation to understand pipeline semantics. The service-based deactivation is cleaner.
