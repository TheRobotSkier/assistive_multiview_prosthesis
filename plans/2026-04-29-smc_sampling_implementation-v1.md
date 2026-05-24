# Sequential Monte Carlo (SMC) Sampling with Decaying Proposal Variance

## Objective

Replace the current single-pass random sampling + scoring pipeline with an iterative Sequential Monte Carlo (SMC) optimizer. Instead of drawing 10,000 independent random samples once and picking the best, SMC will run multiple iterations where:
1. All samples are scored in parallel (using existing `PREDICTION_SAMPLES` count per iteration)
2. The top `ELITE_RATIO` fraction are selected as "elite" particles
3. The remaining particles are resampled around the elites with a proposal distribution whose variance decays geometrically each iteration
4. This concentrates the search progressively around the best-scoring regions

Additionally, audit and enhance parallelism via Rayon throughout the pipeline.

---

## Current Architecture Analysis

### Pipeline Flow (single-pass)

The current pipeline in `c_api.rs:315-460` (`compute_from_request`) works as:

1. **Predict ROI + sample poses** (`predictor.rs:155-204`): `sample_future_poses()` draws `PREDICTION_SAMPLES` (10,000) random samples from a twist-based motion model with fixed covariance. Each sample gets a random grasp type (0/1/2) and random wrist rotation.
2. **Prune point cloud** to the predicted ROI AABB
3. **Build TSDF** from the pruned cloud
4. **Score all samples in parallel** (`c_api.rs:169-201`): `score_all_samples()` uses `par_iter()` over samples, calling the appropriate scorer function per grasp type
5. **Select best** by max combined score
6. **Debug export** (if enabled)

### Key Files & Their Roles

| File | Role | SMC Impact |
|------|------|-----------|
| `src/config.rs` | All tunable constants | Add SMC constants here |
| `src/predictor.rs` | Pose sampling, twist math, ROI prediction | Major refactor: sampling becomes iterative, needs proposal jittering |
| `src/c_api.rs` | Main pipeline orchestration, FFI boundary | Major refactor: loop around score→select→resample |
| `src/planner.rs` | Grasp scoring (cylindrical/pinch/lateral) | No changes needed |
| `src/lut_helper.rs` | Finger LUT, DualQuaternion math | No changes needed |
| `src/pointcloud_helper.rs` | TSDF, point cloud, pruning | No changes needed |
| `src/debug_export.rs` | Debug .npz output | Minor: export iteration info |
| `benches/pipeline.rs` | Benchmarks | Update to benchmark SMC loop |

### Current Parallelism (Rayon Usage)

- `c_api.rs:177` — `score_all_samples()` uses `samples.par_iter().map(...)` — **already parallelized**
- `pointcloud_helper.rs:239` — `prune()` uses `pc.points.par_iter().filter(...)` — **already parallelized**
- `pointcloud_helper.rs:287-310` — `morton()` uses `par_iter().map()` and `par_sort_by_key()` — **already parallelized**
- `pointcloud_helper.rs:415` — TSDF sign computation uses `par_iter_mut()` — **already parallelized**
- `predictor.rs:155-204` — `sample_future_poses()` uses a **sequential loop** with a single `rng` — **NOT parallelized** (cannot be, due to `&mut impl Rng`)
- `benches/pipeline.rs:171` — Full pipeline benchmark uses a **sequential** `for sp in &samples` loop — **NOT parallelized** (unlike `c_api.rs`)

---

## Implementation Plan

### Phase 1: Add SMC Constants to config.rs

- [ ] **1.1** Add the following constants to `src/config.rs` after the existing `FIXED_COV_*` block (around line 28):

```
// SMC Optimization Constants
pub const SMC_ITERATIONS: usize = 8;
pub const SMC_DECAY_RATE: f64 = 0.75;
pub const SMC_ELITE_RATIO: f64 = 0.1;

// Starting Proposal Variance (The "Wide Net")
pub const SMC_INITIAL_PROPOSAL_STD_V: f64 = 0.002;
pub const SMC_INITIAL_PROPOSAL_STD_OMEGA: f64 = 0.005;
```

**Rationale**: These are the user-specified values. Naming them with `SMC_` prefix avoids collision with existing constants. The `PREDICTION_SAMPLES` constant (line 16, currently 10,000) remains unchanged and defines the particle count per SMC iteration.

### Phase 2: Extend the SampledPose Structure and Sampling Functions

- [ ] **2.1** Add a `noise_omega: Vector3<f64>` and `noise_v: Vector3<f64>` field to `SampledPose` in `predictor.rs:10-17`.

**Rationale**: SMC resampling needs to know the current noise state of each particle so it can re-jitter from the elite particles. Currently `sample_twist()` (line 97-133) computes `noise_omega` and `noise_v` but discards them after computing the pose. These must be preserved.

**Current `SampledPose`** (`predictor.rs:10-17`):
```rust
pub struct SampledPose {
    pub pose: DualQuaternion,
    pub sample_probability: f64,
    pub grasp_type: usize,
    pub wrist_rotation: f64,
}
```

**After change**:
```rust
pub struct SampledPose {
    pub pose: DualQuaternion,
    pub sample_probability: f64,
    pub grasp_type: usize,
    pub wrist_rotation: f64,
    /// Noise components from the twist sampling (for SMC resampling).
    pub noise_omega: Vector3<f64>,
    pub noise_v: Vector3<f64>,
}
```

- [ ] **2.2** Update `sample_future_poses()` in `predictor.rs:155-204` to populate the new `noise_omega` and `noise_v` fields from the `SampledTwist` result.

**Rationale**: The data is already computed in `sample_twist()` at line 121-122; it just needs to be threaded through to the `SampledPose`.

- [ ] **2.3** Create a new function `smc_resample_around_elites()` in `predictor.rs` that takes:
  - `elite_poses: &[SampledPose]` — the elite subset from the previous iteration
  - `n_samples: usize` — total particles to produce (`PREDICTION_SAMPLES`)
  - `proposal_std_omega: f64` — current iteration's omega proposal std
  - `proposal_std_v: f64` — current iteration's v proposal std
  - `rng: &mut impl Rng`

  The function should:
  1. For each of `n_samples` output particles, randomly select one elite (uniform random index into `elite_poses`)
  2. Copy the elite's `pose` as the new center
  3. Jitter the elite's `noise_omega` and `noise_v` by adding Gaussian noise with std = `proposal_std_omega` / `proposal_std_v` respectively
  4. Compute the new displaced pose by applying the jittered twist via `twist_to_se3()` + `DualQuaternion::multiply()`
  5. Re-randomize grasp type (uniform 0/1/2) and wrist rotation (uniform in `[-WRIST_ROTATION_RANGE_RAD, WRIST_ROTATION_RANGE_RAD]`)
  6. Recompute `sample_probability` using `compute_sample_probability()` with the new noise values
  7. Return `Vec<SampledPose>`

**Rationale**: This is the core SMC resampling step. The jitter is applied to the **noise components** (not directly to the pose), which keeps the motion model coherent. The proposal variance decays each iteration, narrowing the search.

- [ ] **2.4** Add a helper function `compute_proposal_std(iteration: usize) -> (f64, f64)` in `predictor.rs` that returns `(omega_std, v_std)` as `INITIAL_PROPOSAL_STD_OMEGA * DECAY_RATE^iteration` and `INITIAL_PROPOSAL_STD_V * DECAY_RATE^iteration`.

**Rationale**: Encapsulates the geometric decay formula. With `DECAY_RATE = 0.75` and `ITERATIONS = 8`, the final iteration's std is `0.75^7 ≈ 0.133` of the initial — a ~7.5x narrowing.

### Phase 3: Refactor the Main Pipeline (c_api.rs) to SMC Loop

This is the most significant change. The current single-pass flow in `compute_from_request()` (`c_api.rs:315-460`) must become an iterative loop.

- [ ] **3.1** Refactor `compute_from_request()` to use an SMC loop structure:

**Current flow** (single pass):
```
predict_roi_with_samples() → build TSDF → score_all_samples() → select_best
```

**New flow** (SMC iterative):
```
Iteration 0:
  predict_roi_with_samples() → build TSDF → score_all_samples() → select elite
Iteration 1..N-1:
  smc_resample_around_elites() → score_all_samples() → select elite (update if better)
```

Key design decisions:
- The **TSDF is built once** from the initial ROI prediction and reused across all SMC iterations. The ROI does not change between iterations because the object doesn't move — only the proposal distribution changes.
- The **initial sample set** (iteration 0) is identical to the current behavior: broad random sampling from the motion model.
- From iteration 1 onward, `smc_resample_around_elites()` replaces `predict_roi_with_samples()`.
- The **global best** grasp is tracked across all iterations, not just within each iteration.

- [ ] **3.2** Update `score_all_samples()` (`c_api.rs:169-201`) to also return the indices of the top `ELITE_RATIO` fraction of samples.

**Rationale**: SMC needs the elite set for resampling. The function currently returns `Vec<ScoredGrasp>`. After this change it should also identify and return the elite indices (or the elite `SampledPose`s directly). This can be done by:
1. Collecting `(index, combined_score)` pairs
2. Sorting by `combined_score` descending (partial sort for top-K efficiency)
3. Taking the top `n_elite = (samples.len() as f64 * ELITE_RATIO).ceil() as usize`
4. Returning the corresponding `SampledPose`s as the elite set

**Note on performance**: With 10,000 samples and `ELITE_RATIO = 0.1`, we need the top 1,000. A full sort is O(N log N) = ~133K comparisons. This is negligible compared to the 10,000 scoring evaluations. However, a partial sort (`select_nth_unstable_by`) could be used for marginal improvement.

- [ ] **3.3** Modify `select_best_grasp()` or create a new `select_elites()` function that returns both the best grasp and the elite particle set.

**Rationale**: The current `select_best_grasp()` (`c_api.rs:203-209`) returns only `Option<&ScoredGrasp>`. The SMC loop needs the top-K particles, not just the single best.

- [ ] **3.4** Implement the SMC loop body in `compute_from_request()`. Pseudocode:

```
let (roi, mut samples) = predict_roi_with_samples(...);
// Build TSDF once from initial ROI
let tsdf = build_tsdf(...);
let mut global_best: Option<ScoredGrasp> = None;
let mut global_best_sample: Option<SampledPose> = None;

for iteration in 0..SMC_ITERATIONS {
    let scored = score_all_samples(lut, &tsdf, &samples, collision_tol);
    let (best, elites) = select_best_and_elites(&scored, &samples);
    
    // Update global best
    if global_best is None || best.combined > global_best.combined {
        global_best = best;
        global_best_sample = samples[best_idx].clone();
    }
    
    // Don't resample on last iteration
    if iteration < SMC_ITERATIONS - 1 {
        let (omega_std, v_std) = compute_proposal_std(iteration);
        samples = smc_resample_around_elites(&elites, PREDICTION_SAMPLES, omega_std, v_std, &mut rng);
    }
}

// Use global_best and global_best_sample for output
```

**Rationale**: This structure keeps the TSDF construction cost amortized (built once), while the scoring cost is `SMC_ITERATIONS × PREDICTION_SAMPLES`. With 8 iterations × 10,000 samples = 80,000 total scoring evaluations, but the later iterations should converge much faster because they're exploring near the best regions.

### Phase 4: Parallelism Audit and Enhancement

- [ ] **4.1** Parallelize `smc_resample_around_elites()` using Rayon.

**Current issue**: The function will use `rng` which is `&mut`, making it non-parallelizable directly.

**Solution**: Use `rand::rngs::SmallRng` with per-thread seeds. Create a `Vec<SmallRng>` seeded from a master RNG, then use `par_iter()` on an index range, each thread using its own RNG. This is the standard pattern for parallel random sampling in Rust:

```rust
use rayon::prelude::*;
use rand::SeedableRng;
use rand::rngs::SmallRng;

let master_seed = rng.random::<u64>();
let thread_rngs: Vec<SmallRng> = (0..n_samples)
    .map(|i| SmallRng::seed_from_u64(master_seed.wrapping_add(i as u64)))
    .collect();

let new_samples: Vec<SampledPose> = (0..n_samples)
    .into_par_iter()
    .map(|i| {
        let mut local_rng = thread_rngs[i];
        // ... resample logic using local_rng
    })
    .collect();
```

**Rationale**: With 10,000 samples per iteration, parallel generation is worthwhile. `SmallRng` is fast and suitable for non-cryptographic use.

- [ ] **4.2** Fix the benchmark in `benches/pipeline.rs:171` to use `par_iter()` instead of sequential `for sp in &samples`.

**Current**: `for sp in &samples { ... scorer(...) ... }` — sequential.
**Fix**: `samples.par_iter().map(|sp| { ... }).collect()` — matches the actual pipeline in `c_api.rs`.

**Rationale**: The benchmark currently under-reports the pipeline's actual throughput since the real code uses Rayon. This is a pre-existing bug that should be fixed.

- [ ] **4.3** Consider parallelizing the elite selection (partial sort).

With 10,000 samples, a sequential `select_nth_unstable_by` is fast enough (~microseconds). However, if `PREDICTION_SAMPLES` grows significantly, a parallel partial sort could help. **Recommendation**: Skip for now — the bottleneck is scoring, not sorting.

- [ ] **4.4** Verify that `score_all_samples()` remains the primary parallelism hotspot and that Rayon's thread pool is well-utilized.

The scoring functions (`score_cylindrical`, `score_pinch`, `score_lateral`) each do:
- Sweep through LUT samples checking TSDF collisions
- Binary search for exact collision point
- Contact analysis

Each call is independent and stateless — ideal for `par_iter()`. The current implementation at `c_api.rs:177` is already correct. No changes needed to the scoring parallelism itself.

### Phase 5: Debug Export Updates

- [ ] **5.1** Add SMC iteration metadata to the debug export.

Add an `smc_iteration` field to `ScoredGraspExport` in `debug_export.rs:18-37` and include it in the NPZ output. This allows the visualizer to distinguish which iteration produced each grasp candidate.

**Rationale**: Without this, all 80,000 samples across 8 iterations would be indistinguishable in the debug dump. The iteration number is critical for understanding convergence behavior.

- [ ] **5.2** Decide on debug export strategy for SMC.

**Option A (Recommended)**: Export only the final iteration's samples (10,000 rows, same as current). This keeps the file size manageable and the visualizer unchanged.

**Option B**: Export all iterations (80,000 rows). More data for debugging convergence, but 8x larger files and requires visualizer updates.

**Option C**: Export only the elite particles from each iteration (8 × 1,000 = 8,000 rows). Best of both worlds — shows convergence trajectory without bloating the file.

**Recommendation**: Start with Option A for minimal disruption. The iteration metadata from 5.1 can be added later when Option C is desired.

### Phase 6: Update Tests and Benchmarks

- [ ] **6.1** Update existing predictor tests in `predictor.rs:266-408` to include the new `noise_omega` and `noise_v` fields in any manually constructed `SampledPose` instances.

**Impact**: The tests at lines 307-345 and 363-407 construct `SampledPose` indirectly through `sample_future_poses()`, so they should still compile once `sample_future_poses()` is updated. No manual `SampledPose` construction exists in tests currently.

- [ ] **6.2** Add a new test for `smc_resample_around_elites()` that verifies:
  - Output count matches `n_samples`
  - All output poses are valid (finite coordinates)
  - Proposal variance decreases when called with smaller std values
  - Elite poses appear in the output (resampling around them)

- [ ] **6.3** Add a new test for `compute_proposal_std()` that verifies the geometric decay:
  - Iteration 0 returns the initial values
  - Iteration N returns `initial * DECAY_RATE^N`
  - Values decrease monotonically

- [ ] **6.4** Update `benches/pipeline.rs` to add an SMC-specific benchmark that runs the full iterative loop, enabling comparison against the current single-pass approach.

- [ ] **6.5** Fix the existing `bench_full_pipeline` benchmark (`benches/pipeline.rs:133-182`) to use `par_iter()` for scoring instead of sequential iteration, matching the actual pipeline behavior.

### Phase 7: Update the Debug Example

- [ ] **7.1** Update `examples/debug_dump.rs` to use the SMC loop if the public API changes. If the SMC loop remains internal to `c_api.rs`, the example may need to replicate the loop or call a new public function.

**Rationale**: The example currently replicates the scoring loop manually (lines 136-164). If the SMC loop is exposed as a public function, the example should use it.

---

## Verification Criteria

- [ ] `cargo test` passes with all new and existing tests
- [ ] `cargo bench` shows the SMC pipeline benchmark results
- [ ] The debug dump NPZ file contains valid data with SMC iteration metadata
- [ ] The FFI API (`grasp_preshaping_compute`) remains backward-compatible — the C++ bridge node (`nodes/preshaping_service_bridge_node.cpp`) should not require changes
- [ ] Total scoring evaluations increase from 10,000 to ~80,000, but the best score should improve measurably (especially for hard-to-grasp objects)
- [ ] No regressions in the benchmark for individual scoring functions (`score_cylindrical`, `score_pinch`, `score_lateral`)
- [ ] Rayon utilization remains high — all 8 iterations should fully utilize the thread pool during scoring

## Potential Risks and Mitigations

1. **Performance regression from 8x more scoring evaluations**
   - **Risk**: 80,000 total evaluations vs current 10,000 could make the pipeline too slow for real-time use.
   - **Mitigation**: The scoring is already parallelized via Rayon. On an 8-core machine, the wall-clock increase is ~10x/8 = ~1.25x per iteration × 8 iterations = ~10x total. If this is too slow, `SMC_ITERATIONS` can be reduced (e.g., 4) or `PREDICTION_SAMPLES` per iteration can be reduced (e.g., 5,000). The constants are in `config.rs` for easy tuning.
   - **Additional mitigation**: Consider reducing `PREDICTION_SAMPLES` from 10,000 to something smaller (e.g., 2,000-5,000) since SMC's iterative refinement should achieve equal or better quality with fewer samples per iteration.

2. **SMC convergence to local optima**
   - **Risk**: If the initial broad sampling misses the region entirely, SMC will converge to a poor local optimum.
   - **Mitigation**: The initial proposal std values (0.002 for v, 0.005 for omega) are relatively wide, and the decay rate of 0.75 is moderate. The first iteration is identical to the current broad sampling. If convergence issues are observed, increase `SMC_INITIAL_PROPOSAL_STD_*` or decrease `SMC_DECAY_RATE`.

3. **RNG determinism/reproducibility**
   - **Risk**: Parallel RNG with per-thread seeds may produce different results across runs.
   - **Mitigation**: This is acceptable for a sampling-based optimizer. If exact reproducibility is needed for debugging, a sequential fallback can be added behind a config flag.

4. **Breaking the FFI interface**
   - **Risk**: Changes to `compute_from_request()` could break the C API.
   - **Mitigation**: The `GraspComputeResponseFFI` struct (`c_api.rs:80-100`) and the `grasp_preshaping_compute()` function signature (`c_api.rs:483-560`) remain unchanged. All SMC logic is internal to `compute_from_request()`. The API version (`grasp_preshaping_api_version()` at line 478) should be bumped from 3 to 4 to signal the algorithmic change.

5. **TSDF staleness across iterations**
   - **Risk**: The TSDF is built once but the resampled poses might drift outside the TSDF bounds.
   - **Mitigation**: The initial ROI is computed from the broad sampling (iteration 0), which covers the full prediction horizon. Resampled poses in later iterations are jittered around the elite poses from iteration 0, which are already within the TSDF. The jitter magnitudes are small (decaying from 2mm/5mrad) and unlikely to exceed the TSDF bounds. The TSDF already returns `f32::MAX` for out-of-bounds queries, which naturally penalizes such samples.

6. **Memory usage increase**
   - **Risk**: Storing 10,000 `SampledPose` structs (each containing a `DualQuaternion` + noise vectors) for 8 iterations.
   - **Mitigation**: Only one generation of samples is kept at a time (the previous elite set + the current full set). Memory usage is ~2 × 10,000 × sizeof(SampledPose) ≈ ~2 × 10,000 × ~200 bytes ≈ 4 MB. Negligible.

## Alternative Approaches

1. **Cross-Entropy Method (CEM)**: Instead of resampling around elites, fit a Gaussian to the elite set and sample from it. This is mathematically cleaner but requires computing mean/covariance of the elite distribution in SE(3) space, which is complex. SMC's simpler "copy + jitter" approach is more practical for rigid body poses.

2. **Bayesian Optimization**: Use a surrogate model (Gaussian Process) to guide the search. This is sample-efficient but adds significant implementation complexity and doesn't parallelize as naturally. Not recommended for 10,000-sample budgets.

3. **Simple multi-start with refinement**: Run the current broad sampling, then do gradient-based refinement on the top-K candidates. This requires differentiating through the scoring function, which is not straightforward given the TSDF lookups and binary search. SMC is simpler and more robust.

4. **Reduce per-iteration samples, increase iterations**: Instead of 8 × 10,000 = 80,000, use 16 × 5,000 = 80,000. More iterations with fewer samples per iteration could converge faster but reduces per-iteration parallelism efficiency. This is a tuning knob that can be adjusted via `config.rs` after initial implementation.
