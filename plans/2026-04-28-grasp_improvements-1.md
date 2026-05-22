# Three Improvements to Grasp Preshaping

## Objective

Three related improvements:
1. Show the raw number of active collision/contact points in the debug visualizer
2. Make grasp score weights configurable from `config.rs`
3. Add a multi-finger diversity check to prevent single-finger grasps from passing validation

## Implementation Plan

### Part 1: Export raw contact count in debug dump

- [ ] **Task 1.1. Add `active_contact_count` field to `GraspScoreResult`** in `planner.rs`
  Store the raw number of active contacts (before normalizing to a score ratio) so it can be exported.

- [ ] **Task 1.2. Add `active_contact_count` field to `ScoredGraspExport`** in `debug_export.rs`
  Pass the raw count through the debug export pipeline.

- [ ] **Task 1.3. Expand scored_grasps from 26 to 27 columns** in `debug_export.rs`
  Add the raw contact count as a new column in the npz serialization.

- [ ] **Task 1.4. Wire `active_contact_count` through `c_api.rs`** export path

- [ ] **Task 1.5. Update Python visualizer** to read the 27-column format and display raw contact count in the summary and info text.

### Part 2: Configurable grasp weights

- [ ] **Task 2.1. Add weight constants to `config.rs`**
  Add `GRASP_WEIGHT_PROBABILITY`, `GRASP_WEIGHT_ALIGNMENT`, `GRASP_WEIGHT_FORCE_CLOSURE`, `GRASP_WEIGHT_CONTACT_COUNT`.

- [ ] **Task 2.2. Update `GraspWeights::default()` in `planner.rs`**
  Derive defaults from `config.rs` constants.

### Part 3: Multi-finger diversity check

- [ ] **Task 3.1. Add a `finger_group` function to `Contact` in `lut_helper.rs`**
  Map each Contact variant to a finger group enum (Thumb, Index, Middle, Ring, Little, Palm).

- [ ] **Task 3.2. Add `min_fingers` field to `GraspSpec`** in `planner.rs`
  Specify the minimum number of distinct fingers that must have active contacts.

- [ ] **Task 3.3. Add finger diversity check in `score_grasp`**
  After finding active contacts, verify they span at least `min_fingers` distinct finger groups. If not, treat the grasp as invalid (return None or zero score).

## Verification Criteria

- [ ] Debug visualizer shows raw contact count for each grasp in the summary
- [ ] Changing weight values in `config.rs` changes the combined scores
- [ ] A grasp with all contacts on a single finger is rejected (scored as invalid)
- [ ] Existing tests continue to pass
- [ ] Backward compatibility with old 24/25/26-column npz formats is maintained

## Potential Risks and Mitigations

1. **NPZ format change (26 -> 27 columns)**
   Risk: Old visualizer scripts can't read new dumps.
   Mitigation: The Python loader already has backward-compatible format detection (24/25/26 cols). Add 27 as a new supported format.

2. **Min-fingers check too aggressive**
   Risk: Valid pinch/lateral grasps get rejected because they only use 2 fingers.
   Mitigation: Set `min_fingers` per grasp type: cylindrical=2, pinch=2, lateral=2. Only cylindrical benefits from the check since pinch/lateral are inherently 2-finger.

3. **Weight change affects existing tuning**
   Risk: Changing where defaults come from may subtly change behavior.
   Mitigation: Use the exact same default values currently hardcoded, just move them to config.rs.
