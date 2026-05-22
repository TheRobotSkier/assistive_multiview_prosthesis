# Sequential Monte Carlo (SMC) with Decaying Proposal Variance

## Objective

Replace the current single-pass random sampling strategy with an iterative SMC optimizer that:
1. Samples `PREDICTION_SAMPLES` (1000) candidate hand poses per iteration
2. Scores all candidates in parallel using rayon
3. Selects the top `ELITE_RATIO` (10%) as "elite" samples
4. Re-samples around elites with a proposal variance that geometrically decays each iteration
5. Repeats for `ITERATIONS` (8) iterations, progressively narrowing the search

The expected outcome: the optimizer converges from a broad exploration of the search space to a precise local refinement, producing higher-quality grasps with the same per-iteration sample budget.

---

## Current Architecture Analysis

### Sampling Flow (what happens today)

1. **`c_api.rs:315-357`** — `compute_from_request()` orchestrates the pipeline:
   - Converts FFI input → `DualQuaternion` pose + `Twist6` + covariance
   - Calls `predict_roi_with_samples()` which internally calls `sample_future_poses()`
   - `sample_future_poses()` (`predictor.rs:155-204`) generates `n_samples` random poses by:
     - Sampling a random time `t ∈ [0, t_max]`
     - Adding Gaussian noise to the twist (omega, v) scaled by `sqrt(t)` and the fixed covariance
     - Converting to SE(3) via `twist_to_se3()`
     - Randomly assigning grasp type (0/1/2) and wrist rotation
   - The ROI AABB is computed from these sample positions
   - Point cloud is pruned to ROI, TSDF is built
   - `score_all_samples()` (`c_api.rs:169-201`) scores all samples in parallel via `par_iter()`
   - `select_best_grasp()` picks the single best

2. **Current parallelism**: Rayon is already used in:
   - `c_api.rs:178` — `samples.par_iter().map(...)` for scoring (the main bottleneck)
   - `pointcloud_helper.rs:239` — point cloud pruning
   - `pointcloud_helper.rs:288` — morton code computation
   - `pointcloud_helper.rs:415` — TSDF sign flipping

3. **Key observation**: The TSDF is built once from the pruned point cloud. In the SMC loop, if we keep the same ROI/TSDF, we only need to re-sample poses and re-score them. The TSDF construction cost is amortized across all iterations.

### What Needs to Change

The fundamental shift: instead of `sample_future_poses()` producing a flat batch of independent random samples, we need an iterative loop where:
- **Iteration 0**: Broad sampling (same as current, using twist covariance + wide proposal jitter)
- **Iterations 1..N**: Resample around elite poses from the previous iteration, adding proposal noise that decays geometrically

---

## Implementation Plan

### File: `src/config.rs`

- [ ] **Add SMC constants to config.rs** after the existing `FIXED_COV_*` block (around line 28). These are the user-specified values:

  ```
  // SMC Optimization Constants
  pub const ITERATIONS: usize = 8;
  pub const DECAY_RATE: f64 = 0.75;       // Geometric decay factor per iteration
  pub const ELITE_RATIO: f64 = 0.1;       // Top 10% selected as elites

  // Starting Proposal Variance (The "Wide Net")
  pub const INITIAL_PROPOSAL_STD_V: f64 = 0.002;      // metres
  pub const INITIAL_PROPOSAL_STD_OMEGA: f64 = 0.005;   // radians
  ```

  **Rationale**: Single source of truth for all tunables, consistent with existing config pattern. The `FIXED_COV_*` values remain as the motion-model covariance for the initial broad sampling; the `INITIAL_PROPOSAL_STD_*` values are the SMC jitter applied on top during resampling.

### File: `src/predictor.rs`

- [ ] **Add a new `SmcParticle` struct** to hold per-particle state across iterations. Each particle represents one candidate hand pose with its associated metadata:

  ```
  pub struct SmcParticle {
      pub pose: DualQuaternion,
      pub grasp_type: usize,
      pub wrist_rotation: f64,
      pub score: f64,             // combined_score from last evaluation
      pub sample_probability: f64,
  }
  ```

  **Rationale**: Unlike `SampledPose` which is a one-shot sample from the motion model, `SmcParticle` carries forward state across SMC iterations. The `score` field caches the evaluation result so we can rank and select elites.

- [ ] **Add `sample_initial_particles()` function** that generates the initial broad particle set. This is essentially the current `sample_future_poses()` logic but outputs `SmcParticle` instead of `SampledPose`. It uses the existing twist covariance (`FIXED_COV_OMEGA`/`FIXED_COV_V`) for the motion model noise, exactly as the current code does.

  Signature: `fn sample_initial_particles(current_pose, twist, covariance, config, rng) -> Vec<SmcParticle>`

  **Rationale**: Iteration 0 is the "wide net" — identical to current behavior. This ensures backward compatibility and a good initial spread.

- [ ] **Add `resample_around_elites()` function** that takes the elite particles, the current proposal std (decayed), and generates a new full population of `PREDICTION_SAMPLES` particles:

  1. Compute `n_elite = (particles.len() as f64 * ELITE_RATIO).ceil() as usize`
  2. Sort particles by score descending, take top `n_elite`
  3. For each new particle: pick a random elite, extract its pose as SE(3), add Gaussian jitter:
     - Position jitter: `N(0, proposal_std_v)` on each of x, y, z
     - Orientation jitter: `N(0, proposal_std_omega)` on each of three rotation axis perturbations (applied as a small incremental rotation via `twist_to_se3`)
     - Re-randomize grasp type (uniform 0/1/2) and wrist rotation (uniform over range)
  4. Return the new particle set

  Signature: `fn resample_around_elites(elites: &[SmcParticle], proposal_std_v: f64, proposal_std_omega: f64, rng: &mut impl Rng) -> Vec<SmcParticle>`

  **Rationale**: This is the core SMC resampling step. The geometric decay of proposal std ensures convergence: early iterations explore broadly, later iterations refine precisely. Re-randomizing grasp type and wrist rotation ensures we don't get stuck in a local optimum for those discrete choices.

- [ ] **Keep `SampledPose` and `sample_future_poses()` untouched** — they are still used for ROI prediction (the AABB computation). The SMC loop operates on `SmcParticle` after the initial ROI/TSDF are built.

### File: `src/c_api.rs`

- [ ] **Refactor `compute_from_request()` to implement the SMC loop**. The current flow at lines 315-460 becomes:

  ```
  1. Build ROI and TSDF exactly as before (lines 324-355) — one-time cost
  2. Generate initial particles via sample_initial_particles()
  3. For iteration i in 0..ITERATIONS:
     a. Score all particles in parallel (reuse score_all_samples pattern)
     b. If last iteration: break
     c. Compute proposal_std_v = INITIAL_PROPOSAL_STD_V * DECAY_RATE^i
     d. Compute proposal_std_omega = INITIAL_PROPOSAL_STD_OMEGA * DECAY_RATE^i
     e. Select elites (top ELITE_RATIO by score)
     f. Resample full population around elites via resample_around_elites()
  4. Select best from final scored population
  5. Proceed with debug export and FFI response (unchanged)
  ```

  **Rationale**: The TSDF is built once and reused across all iterations — this is the key efficiency win. Each iteration only re-samples poses and re-scores, both of which are cheap compared to TSDF construction. The `score_all_samples` function already uses `par_iter()` for parallel scoring, so each SMC iteration inherits this parallelism.

- [ ] **Modify `score_all_samples()` to accept `&[SmcParticle]`** (or create a new `score_all_particles()` function). The current function at `c_api.rs:169-201` takes `&[SampledPose]` and produces `Vec<ScoredGrasp>`. The new version should:
  - Take `&[SmcParticle]`
  - Return `Vec<(SmcParticle, ScoredGrasp)>` or update the particles' `score` field in place
  - Use the same `par_iter().map(...)` pattern for parallelism

  **Rationale**: Minimal change to the scoring path. The scorer functions (`score_cylindrical`, `score_pinch`, `score_lateral`) and the TSDF query logic remain completely untouched.

- [ ] **Update debug export to include SMC iteration metadata**. Extend `ScoredGraspExport` (or add a new field) to record which iteration produced each grasp, so the visualizer can show convergence. Add a new column to the `scored_grasps` array (e.g., column 28 = iteration index).

  **Rationale**: Essential for debugging and tuning the SMC parameters. Without this, there's no way to verify convergence visually.

### File: `src/debug_export.rs`

- [ ] **Extend `ScoredGraspExport`** with an `smc_iteration: usize` field. Update the serialization in `export_npz()` to include this as an additional column (making each row 29 columns instead of 28).

  **Rationale**: Debug visibility into SMC convergence. The Python visualizer will need a corresponding update to read this column, but that's outside the scope of this plan.

### File: `benches/pipeline.rs`

- [ ] **Add `bench_smc_pipeline` benchmark** that mirrors `bench_full_pipeline` but runs the SMC loop. This measures the total cost of 8 iterations × 1000 samples scoring.

  **Rationale**: Critical for verifying that the SMC overhead is acceptable. The expectation is that TSDF construction (done once) dominates, and the additional scoring iterations are a moderate multiplier on total time.

- [ ] **Add `bench_resample_around_elites` micro-benchmark** to measure the resampling step in isolation.

  **Rationale**: Ensures resampling itself isn't a bottleneck. It should be negligible compared to scoring.

---

## Rayon / Parallelism Analysis

### Current State
Rayon is **already** used for the most expensive parallel operations:
- **Sample scoring** (`c_api.rs:178`): `samples.par_iter().map(...)` — this is the main bottleneck and is already parallel
- **Point cloud pruning** (`pointcloud_helper.rs:239`): `par_iter().filter()`
- **Morton code computation** (`pointcloud_helper.rs:288`): `par_iter().map()`
- **TSDF sign flipping** (`pointcloud_helper.rs:415`): `par_iter_mut().enumerate()`

### What Changes with SMC
- The scoring parallelism is preserved as-is: each SMC iteration calls `par_iter()` on the particle set
- The resampling step (`resample_around_elites`) generates new particles independently — this could also be parallelized with `par_iter()`, but at 1000 particles the overhead of rayon thread pool management may exceed the benefit. **Recommendation**: keep resampling single-threaded; it's just random number generation and small matrix operations, which is cheap compared to scoring.
- The TSDF is built once and shared (immutable borrow) across all parallel scoring calls — this is already safe.

### Potential Additional Optimizations (Not in Scope, But Noted)
- **Batch RNG**: Using `rand::rngs::SmallRng` with thread-local instances for the resampling step could reduce contention vs. the default thread-safe RNG
- **Pre-allocated particle buffers**: Ping-pong between two `Vec<SmcParticle>` buffers to avoid allocation per iteration
- **Early termination**: If the best score doesn't improve for 2+ iterations, break early. This would need a new config constant.

---

## Verification Criteria

- [ ] **Correctness**: The SMC loop produces a valid `ComputeOutput` with scores >= what the single-pass approach produces (measured on the same input data)
- [ ] **Convergence**: Debug dumps show that the best score improves (or plateaus) across iterations, and the proposal variance shrinks as expected
- [ ] **Performance**: The full SMC pipeline (8 iterations × 1000 samples) completes in < 50ms on a typical workstation (the TSDF is built once; 8000 total scoring evaluations should be fast given the existing rayon parallelism)
- [ ] **No regression**: Existing unit tests in `predictor.rs` and `planner.rs` continue to pass unchanged
- [ ] **API compatibility**: The FFI interface (`GraspComputeRequestFFI` / `GraspComputeResponseFFI`) remains unchanged — the SMC is entirely internal
- [ ] **Debug export**: The `.npz` debug files include the SMC iteration column and can be loaded by the Python visualizer (with minor reader update)

---

## Potential Risks and Mitigations

1. **TSDF may be too coarse for refined SMC iterations**
   - The TSDF is built from the initial broad ROI. In later SMC iterations, particles cluster tightly around the best region, but the TSDF resolution (5mm) may not provide enough precision to differentiate very close poses.
   - **Mitigation**: The 5mm resolution is already quite fine. If this becomes an issue, a second TSDF could be built around the elite cluster at higher resolution in later iterations, but this is a future optimization.

2. **Elite ratio too aggressive may lose diversity**
   - With 10% elite ratio and 1000 samples, only 100 elites survive. If the initial spread is poor, we could converge to a local optimum.
   - **Mitigation**: The re-randomization of grasp type and wrist rotation in `resample_around_elites` maintains diversity in those dimensions. The geometric decay (0.75^7 ≈ 0.13 after 8 iterations) still allows meaningful exploration in later iterations.

3. **Total compute time increase**
   - 8× more scoring evaluations than the current single-pass approach. With rayon parallelism and the TSDF cached, this should be manageable.
   - **Mitigation**: Benchmark first. If too slow, reduce `ITERATIONS` or `PREDICTION_SAMPLES` for the SMC iterations (e.g., use fewer samples in later iterations since the search space is narrower).

4. **Random number generator thread safety**
   - The current code uses `rand::rng()` (thread-safe). In the SMC loop, resampling happens on the main thread before parallel scoring, so there's no contention issue. If resampling is later parallelized, thread-local RNGs would be needed.
   - **Mitigation**: Keep resampling single-threaded for now.

5. **Breaking the debug visualizer**
   - Adding a column to `scored_grasps` changes the file format. The Python visualizer (`scripts/visualize_grasp_debug.py`) will need a corresponding update.
   - **Mitigation**: Add the column at the end of each row so existing readers that ignore extra columns still work. Document the change clearly.

---

## Alternative Approaches

1. **Cross-Entropy Method (CEM)**: Instead of selecting hard elites and resampling uniformly around them, CEM fits a Gaussian to the elite set and samples from the fitted distribution. This is more robust to elite selection noise but requires tracking mean/covariance of a 6D distribution (3 position + 3 rotation), which adds complexity. **Trade-off**: More principled but more code; SMC is simpler and sufficient for this use case.

2. **Bayesian Optimization**: Use a Gaussian Process surrogate model to guide sampling. **Trade-off**: Much more complex to implement, overkill for a 6D search space, and doesn't parallelize as naturally.

3. **Grid refinement**: Start with a coarse grid, score all cells, refine the best cells with a finer grid. **Trade-off**: Doesn't handle the continuous rotation space well, and doesn't naturally incorporate the motion model covariance.

4. **Keep single-pass but increase samples**: Simply increase `PREDICTION_SAMPLES` from 1000 to 8000. **Trade-off**: Same total compute as 8 SMC iterations but without the guided convergence. Wastes samples in regions that are clearly poor. This is the baseline the SMC approach should beat.
