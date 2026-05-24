# Grasp Scoring and Sampling Optimization — Refined Plan v2

## Objective

Optimize the grasp preshaping pipeline to achieve reliable grasp selection with significantly fewer total evaluations while addressing zero-score scenarios in low-sample conditions. The core strategy is: **make every sample count more** by expanding the information content of each evaluation, rather than just reducing sample count blindly.

## Refined Analysis

### The Dimensional Coverage Problem

The search space is effectively **8 continuous dimensions** (3 position from twist, 3 orientation from twist, 1 time, 1 wrist rotation) plus 1 discrete (grasp type). With 10,000 samples, that's roughly 2.5 samples per dimension — already sparse. Reducing to 2,500 would be ~0.6 per dimension, which risks missing the object entirely.

**Conclusion**: We should NOT dramatically reduce sample count. Instead, we should make each sample more informative and use smarter resampling.

### The TSDF Truncation Problem (Your Point 1)

Current truncation: 4 cells × 5mm = **20mm**. This is the critical bottleneck:

- The sweep in `sweep_for_collision` (planner.rs:386-426) checks `get_distance < collision_tol (5mm)` at each finger position
- If a sample's fingers are >20mm from any surface, ALL sweep points return `f32::MAX` → Tier 4 → contact_score = 0.0
- This creates a **dead zone** where ~90% of samples in early iterations get zero gradient
- The SMC optimizer gets no directional information from these samples

**Increasing truncation to 8 cells (40mm)** means:
- Samples up to 40mm from the surface get non-`f32::MAX` distance values
- This doesn't change collision detection (still checks `< 5mm`)
- But it enables a proximity-based Tier 4 score (see below)
- TSDF memory grows: ~8x more voxels (cube of 2x), but this is a one-time cost
- For a 0.3m ROI at 5mm resolution: current ~60³ × 8 = ~1.7M voxels → 8 truncation: ~66³ × 8 = ~2.3M voxels. Manageable.

### The Grasp Type Problem (Your Point 2)

Current behavior: `resample_around_elites` (predictor.rs:279-362) **re-randomizes grasp type uniformly** (line 337: `rng.random_range(0..3)`). This means:
- Iteration 0: A good position with cylindrical grasp gets a high score
- Iteration 1: That elite's position is reused, but grasp type is re-randomized → 2/3 chance of picking a different type
- If pinch at that position is bad, we waste 2/3 of resampled particles

**Weighted grasp type resampling** is the right fix. Compute per-type average scores from the current population, then sample proportionally (with a minimum probability floor to prevent collapse).

### Wrist Rotation Coverage (Your Point 2, extra note)

Current: `WRIST_ROTATION_RANGE_RAD = pi/2` (90° each way), sampled uniformly. The twist covariance adds rotational noise too, but it's small (0.001 rad std). The wrist rotation is the **dominant** orientation DOF.

With 10,000 samples × 8 iterations, the wrist coverage is actually decent per iteration. But with fewer effective samples (after elite selection), the resampled population clusters. **Recommendation**: Keep the range as-is but consider increasing the initial proposal omega slightly to maintain wrist diversity in early SMC iterations.

### Answer to Your Point 5

Yes, the idea behind "increase probability weight when max contact_score < 0.1" was to promote more probable (closer to motion model prediction) positions when we have no contact information. But on reflection, this is a band-aid. The real fix is giving Tier 4 samples a proximity-based score, so they carry directional information even without collision.

## Implementation Plan

### Phase 1: Widen the TSDF Truncation Band

**Files**: `src/config.rs`, `src/planner.rs`

- [ ] **Increase `TRUNCATION_CELLS` from 4 to 8** in `config.rs:7`
  - Rationale: Extends the "gradient zone" from 20mm to 40mm. Samples within 40mm of the surface now get non-infinite TSDF values, enabling proximity scoring.
  - Cost: ~30-40% more TSDF voxels (one-time construction cost), negligible compared to scoring.
  - Impact: Dramatically increases the fraction of samples that get non-zero information.

- [ ] **Add a proximity-based Tier 4 score** in `planner.rs:score_grasp` (currently lines 297-308)
  - Rationale: Currently Tier 4 returns contact_score=0.0, providing zero gradient. With wider truncation, we can compute a proximity score.
  - Implementation: After the `None` branch in `sweep_for_collision`, compute the minimum TSDF distance across all sweep points at closure=0.5 (midpoint). If any value is non-`f32::MAX`, convert to a small proximity score.
  - Formula: `contact_score = 0.05 * max(0, 1.0 - min_distance / (TRUNCATION_CELLS as f32 * resolution))`
  - This gives a score in [0.0, 0.05] — below Tier 3's 0.1, so tier ordering is preserved, but non-zero for samples near the object.
  - Location: `planner.rs:score_grasp`, modify the `None` arm of the match (lines 298-308)

- [ ] **Add a helper function `min_tsdf_distance`** in `planner.rs`
  - Rationale: Needed for the proximity score; queries the minimum distance across all sweep points at a given closure value.
  - Signature: `fn min_tsdf_distance(lut, tsdf, base, sweep_points, control, max_dist) -> f32`
  - Returns the minimum non-MAX distance found, or `f32::MAX` if all points are unobserved.
  - Location: New function in `planner.rs`, after `collides_at_control` (after line 466)

### Phase 2: Weighted Grasp Type Resampling

**Files**: `src/predictor.rs`, `src/c_api.rs`, `src/config.rs`

- [ ] **Add `compute_grasp_type_weights` function** in `predictor.rs`
  - Rationale: Compute per-type weights from the scored population for weighted resampling.
  - Logic: 
    1. Compute average score per grasp type (0, 1, 2)
    2. Shift to non-negative: `shifted = max(0, avg_score - min_avg_score + epsilon)`
    3. Normalize to probabilities with a minimum floor of `GRASP_TYPE_MIN_PROBABILITY` (0.15)
    4. Renormalize to sum to 1.0
  - This means: if cylindrical scores 0.8, pinch 0.3, lateral 0.1 → probabilities roughly [0.55, 0.25, 0.20] (with floor applied)
  - If all types score 0.0 → uniform [0.33, 0.33, 0.33]
  - Location: New public function in `predictor.rs`

- [ ] **Add `GRASP_TYPE_MIN_PROBABILITY` constant** in `config.rs`
  - Rationale: Prevents any grasp type from being completely eliminated, maintaining diversity.
  - Value: 0.15 (15% minimum probability for each type)
  - Location: Add after the SMC constants in `config.rs` (after line 37)

- [ ] **Modify `resample_around_elites`** in `predictor.rs:279-362`
  - Rationale: Replace uniform grasp type sampling with weighted sampling.
  - Current code (line 337): `let grasp_type: usize = rng.random_range(0..3);`
  - New: Accept a `grasp_type_weights: [f64; 3]` parameter, sample using weighted choice.
  - Add helper: `fn weighted_choice(weights: &[f64; 3], rng: &mut impl Rng) -> usize`
  - Location: Modify function signature and line 337 in `predictor.rs`

- [ ] **Update SMC loop in `c_api.rs:401-434`** to compute and pass grasp type weights
  - Rationale: After scoring each iteration, compute weights from the particle population and pass to `resample_around_elites`.
  - Location: `c_api.rs:compute_from_request`, inside the SMC loop (lines 401-434)
  - Logic: After `score_all_particles`, call `compute_grasp_type_weights(&particles)`, then pass result to `resample_around_elites`.

### Phase 3: Early Termination with Configurable Threshold

**Files**: `src/config.rs`, `src/c_api.rs`

- [ ] **Add `SMC_CONVERGENCE_TOL` constant** in `config.rs`
  - Rationale: Configurable threshold for early termination, as you requested.
  - Value: 0.01 (combined score change threshold)
  - Location: Add in `config.rs` after SMC constants

- [ ] **Add `SMC_MIN_ITERATIONS` constant** in `config.rs`
  - Rationale: Minimum iterations before early termination is allowed.
  - Value: 3
  - Location: Add in `config.rs` after SMC constants

- [ ] **Implement convergence check in SMC loop** in `c_api.rs:401-434`
  - Rationale: Stop iterating when best score stabilizes.
  - Logic: Track `prev_best_score`. After iteration >= SMC_MIN_ITERATIONS, if `abs(best_score - prev_best_score) < SMC_CONVERGENCE_TOL`, break.
  - Location: `c_api.rs:compute_from_request`, inside the `for iteration in 0..n_iterations` loop
  - Must track best score across iterations (find max of `scored[].combined` after each scoring pass)

### Phase 4: Smarter Score Combination for Low-Information Scenarios

**Files**: `src/planner.rs`

- [ ] **Improve Tier 2 scoring to preserve alignment and force closure info**
  - Rationale: Currently Tier 2 (insufficient fingers) returns `contact_score = contact_count_score * 0.5`. This discards alignment and force closure info. But these metrics ARE computed and returned — they just don't affect contact_score. The combined_score already uses them via weights, so this is actually fine as-is. The issue is only Tier 4.
  - Decision: No change needed for Tier 2. The proximity score in Phase 1 addresses Tier 4.

- [ ] **Ensure combined_score never returns exactly 0.0 when proximity info exists**
  - Rationale: With the proximity-based Tier 4 score, combined_score will be non-zero for samples near the object. This naturally fixes the "best score is 0" problem without hacky weight adjustments.
  - No additional code needed — the proximity score in Phase 1 handles this.

### Phase 5: Parameter Tuning

**Files**: `src/config.rs`

- [ ] **Adjust `PREDICTION_SAMPLES` from 10000 to 5000**
  - Rationale: With the improved information density from wider truncation + proximity scoring + weighted grasp resampling, each sample carries more information. 5,000 should be sufficient for the initial broad search.
  - Total evaluations with early termination: ~5,000 × 4-5 iterations = 20,000-25,000 (vs current 80,000)
  - Location: `config.rs:16`

- [ ] **Increase `ELITE_RATIO` from 0.1 to 0.15**
  - Rationale: With 5,000 samples, 0.15 gives 750 elites — enough diversity for 3 grasp types to each have ~250 representatives.
  - Location: `config.rs:33`

- [ ] **Increase `INITIAL_PROPOSAL_STD_OMEGA` from 0.002 to 0.005**
  - Rationale: Your concern about coverage is valid. A larger orientation proposal ensures better wrist/rotation exploration in early SMC iterations. 0.005 rad ≈ 0.3° per step, which is still small but covers more ground across the population.
  - Location: `config.rs:37`

- [ ] **Keep `INITIAL_PROPOSAL_STD_V` at 0.001** (no change)
  - Rationale: Position exploration comes primarily from the motion model (twist + time), not SMC jitter. The SMC position jitter should be small — it refines, not explores.
  - Location: `config.rs:36` (unchanged)

- [ ] **Keep `ITERATIONS` at 8** (no change)
  - Rationale: Early termination will cut this short when converged. Keeping 8 as max gives headroom for difficult cases.
  - Location: `config.rs:31` (unchanged)

## Verification Criteria

- [ ] **Proximity scoring**: Tier 4 samples within 40mm of surface get contact_score > 0.0
- [ ] **No more zero best-scores**: In scenarios where the object is within the ROI, the best combined score should be > 0.0
- [ ] **Grasp type diversity**: After convergence, check that the elite set contains at least 2 grasp types (not collapsed to 1)
- [ ] **Early termination activates**: In >50% of test cases, convergence happens before iteration 8
- [ ] **Total evaluations**: Average case < 30,000 (vs current 80,000)
- [ ] **Quality maintained**: Best grasp quality (measured by contact_score) should be within 90% of current quality on the same scenarios

## Potential Risks and Mitigations

1. **Risk: Larger truncation increases TSDF build time significantly**
   - Mitigation: The TSDF build uses BFS (pointcloud_helper.rs:384-410) which is O(voxels). Going from 4→8 truncation roughly doubles each dimension's padding, so ~4-8x voxels. At 5mm resolution with 0.3m ROI, this is still <5M voxels. Benchmark to confirm.
   - Fallback: Use 6 instead of 8 if memory/time is a concern.

2. **Risk: Proximity score creates false gradient toward wrong grasps**
   - Mitigation: Score is capped at 0.05 (well below Tier 3's 0.1), so it only acts as a tiebreaker among Tier 4 samples. It cannot outrank any sample that actually collides.

3. **Risk: Weighted grasp type resampling collapses to one type too quickly**
   - Mitigation: The 15% minimum floor ensures every type maintains representation. Even if one type dominates, 15% × 5000 = 750 particles still explore the other types.

4. **Risk: Early termination stops too early on difficult objects**
   - Mitigation: Minimum 3 iterations required. The convergence tolerance (0.01) is conservative — scores need to truly stabilize.

5. **Risk: Reduced sample count (5000) misses narrow objects**
   - Mitigation: The wider truncation band (40mm) means samples don't need to be as precise to get signal. Combined with 8 SMC iterations and early termination, difficult objects will simply use more iterations.

## Files Modified Summary

| File | Changes |
|------|---------|
| `src/config.rs` | TRUNCATION_CELLS 4→8, PREDICTION_SAMPLES 10000→5000, ELITE_RATIO 0.1→0.15, INITIAL_PROPOSAL_STD_OMEGA 0.002→0.005, add SMC_CONVERGENCE_TOL, SMC_MIN_ITERATIONS, GRASP_TYPE_MIN_PROBABILITY |
| `src/planner.rs` | Add `min_tsdf_distance` helper, modify Tier 4 scoring in `score_grasp` to use proximity |
| `src/predictor.rs` | Add `compute_grasp_type_weights`, modify `resample_around_elites` to accept and use weights, add `weighted_choice` helper |
| `src/c_api.rs` | Add convergence check in SMC loop, compute grasp type weights and pass to resampling |
| `benches/pipeline.rs` | Update benchmark to reflect new parameter defaults (no functional change) |
| `scripts/visualize_grasp_debug.py` | No changes needed — already handles all score columns |

## Implementation Order

1. **Phase 1 first** (truncation + proximity) — this is the highest-impact change and is self-contained
2. **Phase 3 second** (early termination) — simple, independent, immediate speedup
3. **Phase 2 third** (weighted grasp resampling) — improves SMC quality
4. **Phase 5 fourth** (parameter tuning) — tune after the above changes are in place
5. **Phase 4** is handled by Phase 1 (no separate work needed)
