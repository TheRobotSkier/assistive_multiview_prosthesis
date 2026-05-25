# Fix Pipeline State Numbering Drift and Twist Freshness Bug

## Objective

Fix two confirmed bugs:
1. **Pipeline state integer mismatch** — `pipeline_manager` uses a 9-state enum (0–8), but four downstream consumers hard-code an older 7-state numbering (0–6). This makes the force controller, haptic band, cloud snapshot node, and grasp proximity controller completely non-functional for state-dependent behavior.
2. **External twist freshness broken** — `_on_hand_twist()` in twist_propagation uses `/ 1e-9` instead of `/ 1e9`, storing a timestamp ~10¹⁸× too large. The age check never invalidates stale external twist data.

## Detailed Analysis

### Bug 1: State Numbering Mismatch

The pipeline manager (`pipeline_manager_node.py:62-71`) defines:

```
IDLE=0, TWISTING=1, SEGMENTING=2, PLANNING=3, APPROACHING=4, GRASPING=5, HOLDING=6, VOLITIONAL=7, RELEASING=8
```

The downstream nodes were written against an older 7-state scheme that omitted TWISTING and VOLITIONAL:

| State | Pipeline Manager | Affected Nodes (old) |
|---|:---:|:---:|
| IDLE | 0 | 0 |
| TWISTING | 1 | — |
| SEGMENTING | 2 | 1 |
| PLANNING | 3 | 2 |
| APPROACHING | 4 | 3 |
| GRASPING | 5 | 4 |
| HOLDING | 6 | 5 |
| VOLITIONAL | 7 | — |
| RELEASING | 8 | 6 |

**Impact per node:**

- **force_controller** (`force_controller_node.py:76-82`) — `STATE_GRASPING=4` vs published `5`: the controller never activates on entering GRASPING. `STATE_HOLDING=5` vs published `6`: control loop skips HOLDING. `STATE_RELEASING=6` vs published `8`: full reset never fires. The force controller is entirely inert.

- **haptic_controller** (`haptic_controller_node.py:24-27`) — `_GRASPING=4`/`_HOLDING=5` never match published `5`/`6`, so force-feedback motors stay zeroed. `_RELEASING=6` never matches published `8`, so the release buzz never triggers.

- **cloud_snapshot_node** (`cloud_snapshot_node.py:37-43`) — `APPROACHING=3` vs published `4`: freeze doesn't trigger on APPROACHING. `RELEASING=6` vs published `8`: snapshot never clears on release.

- **grasp_proximity_controller** (`grasp_proximity_controller_node.py:189`) — hard-codes `(4, 5)` for GRASPING/HOLDING, but the pipeline publishes `5`/`6`. The guard that prevents joint commands during force control never activates, causing the proximity controller and force controller to fight over joint topics.

- **run_latency_stages.py** (`run_latency_stages.py:197`) — state_names map is wrong: `3→"APPROACH"`, `4→"RELEASING"`. The actual values are `3→PLANNING`, `4→APPROACHING`. State transitions at lines 204 and 208 compare against `1` and `2` which happen to match SEGMENTING and PLANNING by coincidence (both are `2` and `3` in the pipeline manager — wait, no: pipeline publishes SEGMENTING=2, the test checks `state == 1` for SEGMENTING). The test is also broken.

- **emg_grasp_test.yaml** (`emg_grasp_test.yaml:68`) — `wrist_permissive_states: [0, 6]` with comment `0=IDLE, 6=RELEASING`. RELEASING is actually `8`, so wrist motion would be blocked during release.

### Bug 2: Twist Freshness Timestamp

`twist_propagation_node.py:826`:
```python
now_s = self.get_clock().now().nanoseconds / 1e-9
```

`1e-9 = 0.000000001`, so this computes `nanoseconds × 10⁹` instead of `nanoseconds / 10⁹`. The stored `_external_twist_time` is ~10¹⁸× too large. At line 890–891, the comparison uses the correct `/ 1e9`, so the age is always massively negative, and the external twist never ages out.

## Implementation Plan

- [ ] **Task 1. Fix force_controller state constants** (`src/force_controller/force_controller/force_controller_node.py:76-92`)

  Update the state constants and STATE_NAMES dict to match the pipeline manager's 9-state enum. Add the missing TWISTING=1 and VOLITIONAL=7 entries. Shift all subsequent values to match:
  ```
  STATE_IDLE = 0
  STATE_TWISTING = 1
  STATE_SEGMENTING = 2
  STATE_PLANNING = 3
  STATE_APPROACHING = 4
  STATE_GRASPING = 5
  STATE_HOLDING = 6
  STATE_VOLITIONAL = 7
  STATE_RELEASING = 8
  ```
  Update STATE_NAMES accordingly. Also update the `_on_pipeline_state` activation logic (line 301): the activation condition currently checks `old_state in (STATE_APPROACHING, STATE_PLANNING)` — with corrected values this should also include `STATE_VOLITIONAL` as a valid predecessor to GRASPING (though the pipeline manager doesn't currently transition VOLITIONAL→GRASPING, the VOLITIONAL state should be recognized in the deactivation guard at line 308). Add STATE_VOLITIONAL to the deactivation guard so the controller deactivates when transitioning from VOLITIONAL to RELEASING.

- [ ] **Task 2. Fix haptic_controller state constants** (`src/haptic_band/haptic_bridge/haptic_controller_node.py:23-27`)

  Update to match pipeline manager numbering:
  ```python
  _IDLE = 0
  _GRASPING = 5
  _HOLDING = 6
  _RELEASING = 8
  ```
  This ensures the release buzz triggers correctly and force-feedback motors activate during GRASPING/HOLDING.

- [ ] **Task 3. Fix cloud_snapshot_node state constants** (`src/camera/camera/cloud_snapshot_node.py:36-43`)

  Update to match pipeline manager numbering:
  ```python
  IDLE = 0
  SEGMENTING = 1
  PLANNING = 2
  APPROACHING = 3
  GRASPING = 4
  HOLDING = 5
  RELEASING = 6
  ```
  Wait — these actually DO match the pipeline manager for the states they define (0–6). The issue is only APPROACHING and RELEASING. Let me re-check... Pipeline manager: APPROACHING=4, cloud_snapshot: APPROACHING=3. Pipeline: RELEASING=8, cloud_snapshot: RELEASING=6. So yes, these need updating. Also add TWISTING=1 and VOLITIONAL=7 for completeness, and update the docstring at lines 23–25. Update _STATE_NAMES dict (lines 45–52) to include all states.

- [ ] **Task 4. Fix grasp_proximity_controller state comparison** (`src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:189`)

  Change `if self._pipeline_state in (4, 5):` to `if self._pipeline_state in (5, 6):` to correctly match GRASPING=5 and HOLDING=6. Also consider adding VOLITIONAL=7 to the guard, since the force controller is also active during VOLITIONAL.

- [ ] **Task 5. Fix twist_propagation timestamp conversion** (`src/twist_propagation/twist_propagation/twist_propagation_node.py:826`)

  Change `self.get_clock().now().nanoseconds / 1e-9` to `self.get_clock().now().nanoseconds / 1e9`. This is a single-character fix (`1e-9` → `1e9`). The rest of the file (lines 890, 1543) already uses the correct `/ 1e9`.

- [ ] **Task 6. Fix run_latency_stages test state mapping** (`tests/test1_software_verification/run_latency_stages.py:197, 204, 208`)

  Update `state_names` to match pipeline manager numbering. The test currently maps `{0: "IDLE", 1: "SEGMENTING", 2: "PLANNING", 3: "APPROACH", 4: "RELEASING"}` but should be `{0: "IDLE", 1: "TWISTING", 2: "SEGMENTING", 3: "PLANNING", 4: "APPROACHING", 5: "GRASPING", 6: "HOLDING", 7: "VOLITIONAL", 8: "RELEASING"}`. Update the state comparisons at lines 204 and 208: SEGMENTING is now `state == 2` (was `1`), PLANNING is now `state == 3` (was `2`).

- [ ] **Task 7. Fix emg_grasp_test.yaml wrist_permissive_states** (`tests/emg_grasp/emg_grasp_test.yaml:67-68`)

  Change `wrist_permissive_states: [0, 6]` to `wrist_permissive_states: [0, 8]` and update the comment from `0=IDLE, 6=RELEASING` to `0=IDLE, 8=RELEASING`.

- [ ] **Task 8. Add a shared state constants module (optional but recommended)**

  Create a small Python module (e.g., `src/pipeline_manager/pipeline_manager/states.py`) that defines the State IntEnum, and have all downstream nodes import from it instead of re-declaring constants. This prevents future drift. This is a follow-up task — the immediate fixes above are sufficient to resolve the bugs.

## Verification Criteria

- [ ] Force controller activates when pipeline enters GRASPING (state=5), deactivates when leaving GRASPING/HOLDING/VOLITIONAL, and resets on RELEASING (state=8)
- [ ] Haptic controller triggers release buzz when pipeline enters RELEASING (state=8) and provides force feedback during GRASPING (5) and HOLDING (6)
- [ ] Cloud snapshot freezes on PLANNING (2) or APPROACHING (3) and clears on IDLE (0) or RELEASING (8)
- [ ] Grasp proximity controller suppresses joint commands during GRASPING (5), HOLDING (6), and VOLITIONAL (7)
- [ ] External twist ages out correctly after `external_twist_max_age_s` seconds (default 0.5s) — stale twist is rejected and falls back to pose estimation
- [ ] Latency test correctly identifies SEGMENTING (state=2) and PLANNING (state=3) transitions
- [ ] EMG grasp test config allows wrist motion during IDLE (0) and RELEASING (8)

## Potential Risks and Mitigations

1. **Risk: Changing state constants breaks running systems that depend on the old numbering**
   Mitigation: All changes must be deployed atomically — pipeline_manager and all consumers must be updated together. The pipeline_manager itself does not need changes (its numbering is the source of truth).

2. **Risk: The VOLITIONAL state (7) is not handled by force_controller or haptic_controller**
   Mitigation: The force controller's control loop at line 421 checks `self._pipeline_state not in (STATE_GRASPING, STATE_HOLDING)` — this should be extended to include `STATE_VOLITIONAL` since the force controller remains active during volitional mode. Similarly, the haptic controller should provide force feedback during VOLITIONAL.

3. **Risk: External twist was always "fresh" before, so fixing the timestamp may cause unexpected fallback to estimated twist**
   Mitigation: This is the correct behavior. The `external_twist_max_age_s` parameter (default 0.5s) provides a reasonable window. If the external twist source publishes at a healthy rate (>2 Hz), it will remain fresh. Only truly stale data will be rejected.

4. **Risk: Test file changes may affect existing test results/baselines**
   Mitigation: The test was already producing incorrect state labels and timing measurements. The fix corrects the measurement to reflect actual pipeline behavior.

## Alternative Approaches

1. **Define states in a shared ROS 2 message package**: Create a `PipelineState.msg` or use a shared Python package. This is the most robust long-term solution but requires build system changes. The constant-fix approach is simpler and sufficient for now.

2. **Use state name strings instead of integers**: Have the pipeline_manager publish state names (it already does on `/pipeline/state_name`), and have consumers compare against strings. This eliminates numbering issues entirely but requires more subscriber changes and string comparison overhead.

3. **Pin the pipeline_manager to the old 7-state numbering**: Remove TWISTING and VOLITIONAL, reverting to 0–6. This is not viable because those states are actively used in the state machine logic.
