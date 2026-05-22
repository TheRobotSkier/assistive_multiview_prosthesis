# Investigation: Divergent Grasp Results and Closure Fraction Bug

## Objective

Investigate and document the root causes of two observed bugs:
1. The ROS2 node reports a different best grasp (Lateral, combined=0.4042) than the debug dump visualizer (Cylindrical, combined=0.7715) for the same dump file.
2. Setting `PRESHAPING_CLOSURE_FRACTION = 1.0` in `config.rs` has no effect on the actual hand closure sent to the simulation.

---

## Root Cause Analysis

### Bug 1: Divergent "Best Grasp" Between ROS2 Log and Debug Dump

**Finding: The ROS2 node and the debug dump visualizer look at different data.**

The ROS2 node (`preshaping_service_bridge_node.cpp`) calls `grasp_preshaping_compute()` in `c_api.rs`, which runs the full SMC optimization loop. At line `c_api.rs:545`, it calls `select_best_grasps(&scored)` on the **final iteration's scored results only** — the variable `scored` is overwritten each iteration at `c_api.rs:427`.

However, the debug dump at `c_api.rs:480-541` writes **all grasps from all iterations** to the `.npz` file. The visualizer's `_find_best_grasp()` function (`visualize_grasp_debug.py:262-277`) searches across **every grasp in the file** (all 180,000 = 20,000 particles × 9 iterations).

This means:
- **ROS2 node**: selects best from the **last SMC iteration only** (20,000 grasps) → finds Lateral with combined=0.4042
- **Debug visualizer**: selects best from **all iterations combined** (180,000 grasps) → finds Cylindrical with combined=0.7715

The Cylindrical grasp with score 0.7715 was likely found in an early SMC iteration (iteration 0 or 1) but was **not carried forward** into the final iteration's particle population. The SMC resampling process (`resample_around_elites`) can lose good grasps if the elite selection or grasp-type weighting causes the population to converge away from that type/region.

**This is the primary divergence mechanism.** The visualizer's "best" is a global maximum across all iterations; the ROS2 node's "best" is the maximum of only the final iteration.

### Bug 2: `PRESHAPING_CLOSURE_FRACTION` in `config.rs` Is Never Used

**Finding: The Rust constant `PRESHAPING_CLOSURE_FRACTION` is dead code.**

- `config.rs:54` defines `pub const PRESHAPING_CLOSURE_FRACTION: f64 = 1.0;`
- This constant is **never referenced** anywhere in the Rust codebase (confirmed by search).
- The actual closure fraction is controlled by the **C++ ROS2 node** at `preshaping_service_bridge_node.cpp:63`:
  ```cpp
  preshaping_closure_fraction_(declare_parameter<double>("preshaping_closure_fraction", 0.3))
  ```
  This uses ROS2 parameter declaration with a **default value of 0.3**, not 1.0.

So even though you set the Rust constant to 1.0, the C++ node applies a 0.3 fraction, meaning only 30% of the planned closure is sent to the finger controllers (lines 409-411):
```cpp
const double preshape_thumb = std::max(full_thumb * preshaping_closure_fraction_, min_closure_amount_);
```

With `preshaping_closure_fraction_ = 0.3` and a closure amount of 0.2437 (as logged), the actual preshape sent is `max(0.2437 * 0.3, 0.1) = max(0.073, 0.1) = 0.1` — the minimum closure amount. This explains why the hand barely closes.

---

## Implementation Plan

### Bug 1 Fix: Align Debug Dump and ROS2 Node "Best Grasp" Semantics

- [ ] **Task 1.1**: In `c_api.rs`, store the best overall grasp across ALL SMC iterations, not just the last one. Add a variable (e.g., `overall_best: Option<ScoredGrasp>`) before the loop. After each iteration's `score_all_particles()`, compare and update `overall_best` if the current iteration's best exceeds it. Use `overall_best` for the final `ComputeOutput` instead of the last iteration's `scored` results.
  - **Rationale**: The SMC optimizer's final population may not contain the globally best grasp found during optimization. Tracking the overall best ensures the ROS2 node and debug dump agree on the best result.

- [ ] **Task 1.2**: Alternatively (or additionally), in `visualize_grasp_debug.py`, add a flag/option to filter grasps by SMC iteration (e.g., `--last-iteration-only`) so the visualizer can match the ROS2 node's behavior for diagnostic comparison.
  - **Rationale**: Even after fixing Task 1.1, having the visualizer show iteration-specific results is useful for debugging SMC convergence behavior.

### Bug 2 Fix: Make `PRESHAPING_CLOSURE_FRACTION` Effective

- [ ] **Task 2.1**: Remove the unused `PRESHAPING_CLOSURE_FRACTION` constant from `config.rs` since it serves no purpose and creates confusion. The C++ ROS2 node is the correct place for this parameter since it controls how the planner output maps to joint commands.
  - **Rationale**: Dead code that looks like it should affect behavior is misleading. Removing it eliminates the false expectation that changing it will change behavior.

- [ ] **Task 2.2**: Set the `preshaping_closure_fraction` ROS2 parameter to 1.0 at runtime (via launch file or command line), or change the default in `preshaping_service_bridge_node.cpp:63` from `0.3` to `1.0`.
  - **Rationale**: The user's intent is to have full closure applied immediately. The C++ default of 0.3 overrides the unused Rust constant. Changing the C++ default or passing the parameter at launch time achieves the desired behavior.

- [ ] **Task 2.3** (Optional): If the closure fraction should be a shared configuration between Rust and C++, expose it through the FFI response or request struct so the Rust planner can communicate the intended fraction, and the C++ node can apply it. Alternatively, pass it as a field in `GraspComputeRequestFFI`.
  - **Rationale**: Centralizing configuration avoids the current disconnect where Rust and C++ each have their own independent defaults.

---

## Verification Criteria

- [ ] After fix, the ROS2 node log and debug dump visualizer report the **same** best grasp type and combined score for the same input data.
- [ ] After fix, setting closure fraction to 1.0 results in the hand closing fully to the planned closure amount (not capped at 30%).
- [ ] The `config.rs` no longer contains a misleading `PRESHAPING_CLOSURE_FRACTION` constant that has no effect.
- [ ] Running the visualization script on a new debug dump shows the same best grasp as the ROS2 node logged for that run.

---

## Potential Risks and Mitigations

1. **SMC convergence regression from using overall best**: Tracking the overall best across iterations is safe for the output selection but does not change the SMC dynamics. The final population still drives resampling. No regression risk.
   - Mitigation: None needed — only the output selection changes, not the optimization.

2. **Removing the Rust constant breaks downstream code**: The constant is confirmed unused (zero references outside its definition). Safe to remove.
   - Mitigation: Verified by search; no risk.

3. **Changing C++ default from 0.3 to 1.0 changes existing deployed behavior**: If other users or launch configurations rely on the 0.3 default, this could cause unexpected full closure.
   - Mitigation: Prefer setting the parameter via the ROS2 launch file or parameter file rather than changing the C++ default, so existing deployments are unaffected unless explicitly configured.

---

## Alternative Approaches

1. **For Bug 1**: Instead of tracking overall best in Rust, store the "winning iteration" index in the debug dump metadata so the visualizer can filter to that iteration. This preserves the SMC's final-iteration semantics but makes the visualizer consistent.
   - Trade-off: Less invasive change but requires the visualizer to know which iteration to look at.

2. **For Bug 1**: Keep all elite grasps across iterations in a separate "hall of fame" and select from that at the end. This is a more robust approach for stochastic optimization.
   - Trade-off: Slightly more memory and code complexity, but standard practice in evolutionary/swarm optimization.

3. **For Bug 2**: Instead of removing the Rust constant, have the C++ node read it from the Rust library via a new FFI function (e.g., `grasp_preshaping_get_config()`). This makes Rust the single source of truth.
   - Trade-off: More engineering effort, but ensures configuration consistency across the FFI boundary.
