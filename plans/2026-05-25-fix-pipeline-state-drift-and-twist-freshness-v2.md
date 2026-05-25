# Fix Remaining Pipeline State Drift and Twist Freshness Bug

## Objective

Fix the remaining unfixed instances of two confirmed bugs:

1. **Pipeline state integer mismatch** — Three downstream consumers still use the old 7-state numbering (0–6). The pipeline manager uses a 9-state enum (0–8). Four files have already been fixed (force_controller, haptic_controller, pipeline_manager itself, emg_grasp_test.yaml); three files plus one secondary issue in the force controller remain.
2. **External twist freshness broken** — `_on_hand_twist()` in twist_propagation uses `/ 1e-9` instead of `/ 1e9`, storing a timestamp ~10¹⁸× too large, so stale external twist data never ages out.

## Current Status of Each Affected File

| File | Bug | Status |
|---|---|---|
| `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py` | State numbering (source of truth) | OK — no change needed |
| `src/force_controller/force_controller/force_controller_node.py:76-95` | State constants | FIXED |
| `src/force_controller/force_controller/force_controller_node.py:311-316,425` | VOLITIONAL not in deactivation/control guards | STILL BROKEN |
| `src/haptic_band/haptic_bridge/haptic_controller_node.py:24-32` | State constants | FIXED |
| `src/camera/camera/cloud_snapshot_node.py:36-43` | State constants | STILL BROKEN |
| `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:189` | Hard-coded `(4, 5)` | STILL BROKEN |
| `src/twist_propagation/twist_propagation/twist_propagation_node.py:826` | `/ 1e-9` timestamp | STILL BROKEN |
| `tests/test1_software_verification/run_latency_stages.py:197,204,208` | State mapping | STILL BROKEN |
| `tests/emg_grasp/emg_grasp_test.yaml:68` | `wrist_permissive_states` | FIXED |

## Implementation Plan

- [ ] **Task 1. Fix cloud_snapshot_node state constants** (`src/camera/camera/cloud_snapshot_node.py:36-43`)

  Update to match pipeline manager numbering:
  ```python
  IDLE = 0
  TWISTING = 1
  SEGMENTING = 2
  PLANNING = 3
  APPROACHING = 4
  GRASPING = 5
  HOLDING = 6
  VOLITIONAL = 7
  RELEASING = 8
  ```
  Update `_STATE_NAMES` dict (lines 45–53) to include TWISTING and VOLITIONAL.
  Update the module docstring at lines 23–25 to reflect the current numbering.
  
  The `_on_state` callback at line 157 compares against `APPROACHING` and `RELEASING` by name — once the constants are corrected, these comparisons will work correctly.

- [ ] **Task 2. Fix grasp_proximity_controller state comparison** (`src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:189`)

  Change `if self._pipeline_state in (4, 5):` to `if self._pipeline_state in (5, 6, 7):` to correctly match GRASPING=5, HOLDING=6, and VOLITIONAL=7. The force controller remains active during VOLITIONAL, so the proximity controller must also suppress its joint commands during that state to avoid fighting the force controller.

- [ ] **Task 3. Fix force_controller VOLITIONAL guard** (`src/force_controller/force_controller/force_controller_node.py:311-316,425`)

  Two locations need updating:
  
  **Line 311-316** — Deactivation guard: The pipeline manager transitions HOLDING→VOLITIONAL, so when the state goes VOLITIONAL→RELEASING, `old_state` is VOLITIONAL (7). The current guard only checks `old_state in (STATE_GRASPING, STATE_HOLDING)`, so it misses the VOLITIONAL→RELEASING transition. Add `STATE_VOLITIONAL` to the old_state tuple:
  ```python
  if old_state in (STATE_GRASPING, STATE_HOLDING, STATE_VOLITIONAL) and new_state not in (
      STATE_GRASPING,
      STATE_HOLDING,
      STATE_VOLITIONAL,
  ):
  ```
  
  **Line 425** — Control loop: The pipeline manager keeps the force controller active during VOLITIONAL (the user can adjust force). Change:
  ```python
  if self._pipeline_state not in (STATE_GRASPING, STATE_HOLDING):
  ```
  to:
  ```python
  if self._pipeline_state not in (STATE_GRASPING, STATE_HOLDING, STATE_VOLITIONAL):
  ```

- [ ] **Task 4. Fix twist_propagation timestamp conversion** (`src/twist_propagation/twist_propagation/twist_propagation_node.py:826`)

  Change `self.get_clock().now().nanoseconds / 1e-9` to `self.get_clock().now().nanoseconds / 1e9`. This is a single-character fix: `1e-9` → `1e9`. The rest of the file (lines 890, 1543) already uses the correct `/ 1e9`.

- [ ] **Task 5. Fix run_latency_stages test state mapping** (`tests/test1_software_verification/run_latency_stages.py:197,204,208`)

  Update `state_names` at line 197 to match pipeline manager numbering:
  ```python
  state_names = {0: "IDLE", 1: "TWISTING", 2: "SEGMENTING", 3: "PLANNING", 4: "APPROACHING", 5: "GRASPING", 6: "HOLDING", 7: "VOLITIONAL", 8: "RELEASING"}
  ```
  Update the state comparisons:
  - Line 204: `if state == 2:` (SEGMENTING, was `1`)
  - Line 208: `elif state == 3:` (PLANNING, was `2`)

## Verification Criteria

- [ ] Cloud snapshot freezes on PLANNING (state=2) or APPROACHING (state=4) and clears on IDLE (0) or RELEASING (8)
- [ ] Grasp proximity controller suppresses joint commands during GRASPING (5), HOLDING (6), and VOLITIONAL (7)
- [ ] Force controller remains active during VOLITIONAL and correctly deactivates on VOLITIONAL→RELEASING transition
- [ ] External twist ages out correctly after `external_twist_max_age_s` seconds (default 0.5s)
- [ ] Latency test correctly identifies SEGMENTING (state=2) and PLANNING (state=3) transitions

## Potential Risks and Mitigations

1. **Risk: Adding VOLITIONAL to the force controller control loop changes behavior during volitional mode**
   Mitigation: This is the correct behavior — the pipeline manager's state machine explicitly supports force adjustment during VOLITIONAL. The `_on_manual_adjust` callback (line 328) already adjusts the target, and the control loop should be running to apply those adjustments.

2. **Risk: External twist was always "fresh" before, so fixing the timestamp may cause unexpected fallback to estimated twist**
   Mitigation: This is the correct behavior. The `external_twist_max_age_s` parameter (default 0.5s) provides a reasonable window. If the external twist source publishes at a healthy rate (>2 Hz), it will remain fresh. Only truly stale data will be rejected.

3. **Risk: Changing state constants in cloud_snapshot_node breaks any config overrides of `snapshot_on_state` or `clear_on_state`**
   Mitigation: No YAML config overrides were found for these parameters (searched all `.yaml` files). The defaults are `snapshot_on_state=2` (PLANNING) and `clear_on_state=0` (IDLE), which already match the pipeline manager numbering.

## Alternative Approaches

1. **Define states in a shared Python module**: Create `src/pipeline_manager/pipeline_manager/states.py` with the State IntEnum and have all downstream nodes import from it. This prevents future drift but requires package dependency changes.

2. **Use state name strings instead of integers**: Have consumers subscribe to `/pipeline/state_name` and compare strings. Eliminates numbering issues but requires more changes and string comparison overhead.
