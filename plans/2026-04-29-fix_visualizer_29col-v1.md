# Fix Visualizer for 29-Column SMC Format

## Objective

The Python visualizer (`scripts/visualize_grasp_debug.py`) does not recognize the new 29-column format introduced by the SMC implementation. This causes complete data misalignment when loading new debug dumps, producing garbage scores and rendering failures.

## Root Cause

`scripts/visualize_grasp_debug.py:118-134` — the column detection logic checks for 28, 27, 26, 25, or 24 columns, but the Rust side now writes **29 columns** (adding `smc_iteration` at column [12]). When the total value count happens to divide evenly by a smaller column count (e.g., 2,900,000 % 25 == 0), the parser picks the wrong row length and every field after column 12 is misaligned.

## Implementation Plan

- [ ] **Task 1. Add 29-column detection to `load_dump()`** — Insert `raw_len % 29 == 0` as the first check in the column detection chain at `scripts/visualize_grasp_debug.py:121`. This must come before the 28-col check since 29-col data could also divide evenly by a smaller number.
- [ ] **Task 2. Add 29-column parsing block** — Add a new `if row_len == 29:` block after the existing `if row_len == 28:` block (around line 136). The column layout matches the Rust serialization at `src/debug_export.rs:200-214`:
  - [0] sample_index (int)
  - [1] grasp_type (int)
  - [2] closure_amount
  - [3] alignment_score
  - [4] force_closure_score
  - [5] contact_count_score
  - [6] contact_score
  - [7] active_contact_count (int)
  - [8] found_collision (>0.5)
  - [9] combined_score
  - [10] sample_probability
  - [11] wrist_rotation
  - [12] smc_iteration (int) — **NEW**
  - [13:29] pose_4x4 (16 values, reshape to 4×4)
- [ ] **Task 3. Include `smc_iteration` in the grasps dict** — The 29-col parser should add `"smc_iteration": grasps_raw[:, 12].astype(int)` to the returned dict. All existing parsers (24–28 col) should set `"smc_iteration": np.zeros(len(grasps_raw), dtype=int)` for backward compatibility.

## Verification Criteria

- [ ] Loading a 29-column dump (e.g., `grasp_dump_560428_134056.npz`) correctly parses all fields: `combined` scores in [0,1] range, `probability` in [0,1], `smc_iteration` as integers
- [ ] Loading older 28-column dumps still works without errors
- [ ] The visualizer renders grasp positions and hand skeletons correctly for 29-col data
- [ ] `print_summary()` shows reasonable values (no `combined=9983` or `prob=3.0`)

## Potential Risks and Mitigations

1. **Existing 28-col dumps have raw_len divisible by 29**
   Mitigation: Unlikely in practice (28 × 100000 = 2,800,000; 2,800,000 % 29 = 26, so it would correctly fall through to 28). But if it happens, the parser would misparse. We could add a more robust format marker (e.g., store column count as metadata in the npz), but for now the divisibility heuristic is sufficient.

2. **Other scripts may also need updating**
   Mitigation: Only `visualize_grasp_debug.py` reads the `scored_grasps` array. No other Python scripts in the repo parse this format.

## Alternative Approaches

1. **Store column count as npz metadata**: Add a `scored_grasps_cols` array to the npz so the parser doesn't need to guess. More robust but requires changing both Rust and Python sides.
2. **Keep column count at 28 and drop smc_iteration from the flat format**: Store SMC iteration info separately. Simpler parser but loses per-row iteration tagging.
