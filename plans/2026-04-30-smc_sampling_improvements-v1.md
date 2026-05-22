# Bug 1: SMC Losing Best Grasps — Root Cause Analysis and Improvement Plan

## Objective

Fix the SMC optimizer so it does not lose the best grasp found across iterations, and add iteration-cycling to the visualizer for debugging. Additionally, investigate whether the sampling strategy itself can be improved to produce better and more stable grasp candidates.

---

## Root Cause: Why the Last SMC Iteration Doesn't Contain the Best Grasp

The search space is **9-dimensional**: 3 for position, 3 for orientation (via twist displacement), 1 for wrist rotation, 1 for grasp type (discrete), and effectively 1 for the time horizon `t`. With 20,000 particles and a rapidly decaying proposal variance (`0.7^iteration`), the SMC converges aggressively:

**Iteration 0**: 20,000 particles spread across the full motion-model distribution. With uniform grasp type assignment (1/3 each), ~6,667 are cylindrical. One of these happens to land near the object and scores 0.7715.

**Iteration 1+**: The top 15% (3,000 particles) become elites. `compute_grasp_type_weights` (`predictor.rs:385-434`) shifts probabilities toward whichever type had the best *average* scores. If lateral grasps had more Tier-1 results (even if individually lower-scoring), lateral's weight increases. Cylindrical's weight drops to the floor of 0.15.

**Key problem**: The elite selection at `predictor.rs:368-381` selects by `particle.score` (the combined score). But the resampling at `predictor.rs:280-364` picks a **random elite** as the parent for each new particle. The best elite is no more likely to be chosen than the 3000th-best elite. So even though the best cylindrical grasp is an elite, it has only a 1/3000 chance of being any given new particle's parent.

**By iteration 9** (where your run converged): proposal std has decayed to `0.002 * 0.7^8 = 0.00016m` — extremely narrow. If the cylindrical grasp's exact position was not carried forward (very likely given the random elite selection), no amount of jittering will rediscover it. The population has converged to the lateral region.

### Specific Issues Identified

1. **No elitism preservation**: The best grasp is never explicitly carried forward. Standard SMC/CEM practice is to keep the top-k elites unchanged ("hall of fame" or "elite injection").

2. **Uniform elite sampling**: `resample_around_elites` picks parents uniformly from the elite set (`predictor.rs:297`). Weighting toward higher-scored elites would help preserve the best.

3. **Wrist rotation fully randomized on resample**: `predictor.rs:340-341` re-randomizes wrist rotation uniformly in `[-π/2, π/2]`. If the best grasp depended on a specific wrist angle, that information is completely lost during resampling.

4. **Grasp type probability collapse**: Even with the 0.15 floor, if cylindrical has 15% probability and the population is 20,000, only ~3,000 particles explore cylindrical — and they're jittered around random lateral-biased elites, not around the best cylindrical grasp found so far.

5. **sample_probability is copied from elite**: `predictor.rs:356` copies `elite.sample_probability` to the new particle. This is the motion-model probability from the *original* sampling — it doesn't reflect the new particle's actual position. This distorts the combined score because `sample_probability` contributes 0.5/5.5 = 9% of the weighted score.

---

## Implementation Plan

### Part A: Fix the Output Selection (Minimal Fix for Bug 1)

- [ ] **Task A.1**: In `c_api.rs`, add an `overall_best: Option<ScoredGrasp>` variable before the SMC loop (around line 419). After each iteration's `score_all_particles()` call, compare the iteration's best against `overall_best` and update if better. Use `overall_best` for the final `ComputeOutput` instead of the last iteration's `scored`.
  - **Rationale**: This is the simplest fix that ensures the ROS2 node and debug dump agree on the best grasp. It does not change SMC dynamics at all — only the output selection.

### Part B: Improve SMC Sampling Quality

- [ ] **Task B.1**: Add **elite injection** to `resample_around_elites` in `predictor.rs`. Reserve a small fraction (e.g., top 5% of the new population, ~1000 particles) as unchanged copies of the best elites. Only the remaining 95% are jittered resamples. This guarantees the best-so-far grasps survive into the next iteration.
  - **Rationale**: This is standard practice in CEM/SMC optimization. Without it, the optimizer relies on luck to carry forward the best solutions. With it, the best grasps are always available for further refinement.

- [ ] **Task B.2**: Change elite parent selection in `resample_around_elites` from **uniform** to **score-weighted** sampling. Instead of `elites[rng.random_range(0..n_elites)]`, use the elite's score as a weight so better elites are more likely to be chosen as parents.
  - **Rationale**: Currently the 1st-best and 3000th-best elite are equally likely parents. Score-weighted selection biases new particles toward the best-known regions, which is the whole point of SMC.

- [ ] **Task B.3**: Add **wrist rotation perturbation** instead of full re-randomization during resampling. Instead of `rng.random_range(-π/2..π/2)`, perturb the parent's wrist rotation by a small Gaussian (e.g., `σ = WRIST_ROTATION_RANGE_RAD * decay * 0.3`). This preserves good wrist angles while still exploring.
  - **Rationale**: Wrist rotation is a critical DOF for grasp success. Fully randomizing it each iteration throws away any information about good wrist angles, making convergence much harder.

- [ ] **Task B.4**: Fix the **sample_probability inheritance** in `resample_around_elites` (`predictor.rs:356`). Instead of copying the elite's probability, recompute it based on the new particle's displacement from the current pose, or simply use a fixed neutral value (e.g., 1.0) for resampled particles since they are no longer drawn from the motion model.
  - **Rationale**: The current approach gives resampled particles the probability of a random elite's original motion-model sample, which is meaningless for the new particle's actual position. This distorts the combined score and can cause the optimizer to prefer lucky initial samples over genuinely better converged ones.

### Part C: Visualizer Iteration Cycling

- [ ] **Task C.1**: Add keyboard shortcut (e.g., `+`/`-` or `]`/`[`) to the PyVista visualizer to cycle through SMC iterations. When an iteration is selected, filter the displayed grasps to only those with `smc_iteration == selected_iteration`. Display the current iteration number and its best score in the info text.
  - **Rationale**: This is a powerful debugging tool for understanding SMC convergence behavior. You can watch how the population evolves, which grasp types dominate each iteration, and where good grasps are lost.

- [ ] **Task C.2**: Add a "best across all iterations" mode (e.g., key `b`) that shows the global best regardless of iteration, matching the behavior after fix A.1.
  - **Rationale**: After implementing the hall-of-fame fix, this mode in the visualizer provides a way to verify the fix works correctly.

---

## Verification Criteria

- [ ] The ROS2 node always reports a grasp at least as good as the debug dump's global best.
- [ ] With elite injection, the best score is non-decreasing across iterations (it can stay the same but never drop).
- [ ] The visualizer can cycle through individual iterations and shows the per-iteration best.
- [ ] End-to-end grasp quality (combined score) improves or stays the same compared to the current implementation on the same test scenarios.

---

## Potential Risks and Mitigations

1. **Elite injection reduces diversity**: Keeping unchanged elites means fewer particles explore new regions.
   - Mitigation: Only inject the top ~5% (1000 of 20,000). The remaining 95% still explore. The decay schedule already handles the exploration/exploitation tradeoff.

2. **Score-weighted elite selection accelerates premature convergence**: If the initial population happens to cluster in a suboptimal region, score-weighting makes it harder to escape.
   - Mitigation: The `GRASP_TYPE_MIN_PROBABILITY` floor already helps. Combined with the wrist rotation perturbation fix (B.3), diversity is maintained in the critical DOFs.

3. **Wrist rotation perturbation needs tuning**: Too small a perturbation prevents exploration; too large is equivalent to the current randomization.
   - Mitigation: Tie the perturbation scale to the same decay schedule used for position/orientation, so it naturally narrows as the optimizer converges.

---

## Alternative Approaches

1. **Hall of Fame (separate from population)**: Instead of injecting elites into the population, maintain a separate "hall of fame" of the top-k grasps across all iterations. Select the output from the hall of fame at the end. This is cleaner than elite injection but doesn't help the SMC converge toward the best region — it only fixes the output selection.
   - Trade-off: Simpler to implement, but doesn't improve the optimization itself. Good as a minimum viable fix combined with Part A.

2. **Multi-start SMC**: Run 3 independent SMC optimizers, one per grasp type, each with 6,667 particles. This eliminates the grasp-type competition problem entirely.
   - Trade-off: More computationally expensive (3 separate TSDF scoring passes) but guarantees each grasp type gets a fair optimization budget. Could be parallelized since the three runs are independent.

3. **Weighted ensemble averaging for output**: Instead of returning the single best grasp, compute a weighted average of the top-k grasps (weighted by combined score) for the output position/orientation. This produces a more robust output that's less sensitive to noise in any single sample.
   - Trade-off: Averaging SE(3) poses requires careful interpolation (e.g., dual quaternion averaging). If the top grasps are from different modes (e.g., one cylindrical, one lateral), averaging produces a meaningless result. Should only average within the same grasp type.
