# Grasp Preshaping Performance Regression — Verified Analysis & Fix Plan

## Objective

Identify and resolve the performance regression from ~6.5s (old, 1M samples, 1 iteration) to ~112s (new, 1M samples, up to 5 SMC iterations) in the grasp preshaping pipeline, while retaining the new SMC optimization, superquadric backside estimation, and dense scoring features.

---

## Part 1: Verified Root Cause Analysis

### Benchmark Context

| Benchmark | Old | New | Delta |
|---|---|---|---|
| `roi_prediction` | 2.90 ms | 3.21 ms | +11% (minor) |
| `pointcloud_prune` | 2.03 ms | 2.12 ms | negligible |
| `tsdf_construction` (no cameras, no SQ) | 2.63 ms | 2.59 ms | **faster** |
| `full_pipeline_predict_to_score` (10k, no cameras, no SQ) | 120 ms | 126 ms | +5% (minor) |
| `smc_pipeline` (10k, no cameras, no SQ, 5 iter) | N/A | 689 ms | new feature |
| `score_cylindrical` | 15.6 µs | 12.3 µs | **faster** |
| `score_pinch` | 9.9 µs | 9.0 µs | **faster** |
| `score_lateral` | 11.0 µs | 8.2 µs | **faster** |
| `superquadric_fitting` | N/A | 3.4 ms | new feature |
| `tsdf_construction_with_superquadric` | N/A | 7.9 ms | new feature |

**Key insight**: The micro-kernels (scoring, bare TSDF) are not the problem. The regression is in the **production orchestration path**: SMC iteration, camera-based sign correction, SQ blending, debug export, and sorting.

---

### Root Cause 1: Camera Sign Correction — O(V × C × P) Instead of O(V × C) [CRITICAL]

**Old code**: `grasp_preshaping_old/src/pointcloud_helper.rs:429-460`
```
let mp = &morton_array[nearest[flat_idx] as usize];
let pw = Vector3::new(mp.x, mp.y, mp.z);
for cam in cameras {
    let d_cv = (vw - cam.position).norm_squared();
    let d_cp = (pw - cam.position).norm_squared();
    // O(1) per camera — just compare distances to nearest surface point
}
```

**New code**: `grasp_preshaping/src/pointcloud_helper.rs:459-526`
```
for cam in cameras {
    for &sp in &surface_points {       // <-- NESTED LOOP over ALL surface points
        let to_surf = sp - cam.position;
        let proj = to_surf.dot(&ray_dir);
        // ... alignment check, perpendicular distance ...
    }
}
```

**Impact**: For a grid of ~250K voxels, ~5K surface points, and 2 cameras, this is 2.5 billion dot products + norm calculations. This alone can explain tens of seconds of wall time when cameras are enabled.

**Why it was changed**: The old code used the brushfire `nearest` array to get the single closest surface point per voxel, then compared camera distances. This was fast but incorrect for opposing cameras — the nearest surface point might be on the wrong side. The new code scans all surface points to find the best ray-aligned occluder per camera.

**Verified fix**: Restore the `nearest`-based O(1) lookup but add the opposing-camera fix on top. Specifically:
- Use the `nearest` surface point for the primary distance comparison (fast path)
- Only do the full surface scan when cameras disagree (rare case)
- Or: pre-build a per-camera depth buffer / projected index so the ray check is O(1)

**Feature tradeoff**: None if done correctly. The `nearest` array already tracks the closest surface point. The opposing-camera case can be handled by the SQ tiebreaker (which the new code already has at `pointcloud_helper.rs:535-543`).

---

### Root Cause 2: Superquadric Full-Grid Evaluation [HIGH]

**Source**: `grasp_preshaping/src/pointcloud_helper.rs:560-643`

The SQ blend pass iterates over **every voxel** in the grid and for each:
1. Calls `sq.taubin_distance(vw)` — which internally does: `evaluate` + `gradient` + projected `evaluate` + projected `gradient` = **4 SQ evaluations**
2. Calls `sq.evaluate(vw)` separately for sign check = **1 more SQ evaluation**
3. Does smoothstep blending math

**Cost per voxel**: ~5 `evaluate` calls, each involving multiple `.powf()` operations on f32. For 250K voxels, that's ~1.25M heavy transcendental function calls.

**Verified fix**: Your idea of using the point-cloud sign as a mask is sound. Specifically:
- After the brushfire + camera sign pass, every voxel already has a sign (positive or negative).
- Voxels that are `f32::MAX` (unobserved) or negative (inside) are the only ones that need SQ correction.
- Voxels that are positive and near the observed surface (low distance value) should keep the camera/brushfire authority.
- This reduces the SQ evaluation set from 100% of the grid to roughly 10-30% (the interior + unobserved backside).

**Risk**: If we skip SQ on positive voxels that are actually on the backside (camera marked them as outside incorrectly), we lose backside correction. Mitigation: only skip SQ on voxels where the camera sign is confident (multiple cameras agree, or the voxel is very close to the observed surface).

---

### Root Cause 3: SMC Iteration Multiplier [HIGH — BY DESIGN, BUT AMPLIFIED]

**Source**: `grasp_preshaping/src/c_api.rs:435-489`

The SMC loop runs up to `config::ITERATIONS = 5` iterations. Each iteration:
1. Scores all 10K particles (`score_all_particles`) — this is the expensive part
2. Clones debug data for every particle (if `DEBUG_VISUALIZATION = true`)
3. Sorts all particles for elite selection
4. Resamples the full population

At 10K particles × 5 iterations = 50K scoring calls. The old code did 10K × 1 = 10K. That's a 5× multiplier by design.

**However**, the per-iteration overhead is amplified by:
- Debug cloning (Root Cause 5)
- Full sorting (Root Cause 4)
- Dense scoring for invalid particles (Root Cause 6)

**Verified fix**: The iteration count itself is a design choice. But we should:
- Ensure early termination works well (convergence check at `c_api.rs:450-457`)
- Reduce per-iteration overhead (fixes for Root Causes 4, 5, 6)
- Consider starting with fewer samples and growing, rather than full 10K from iteration 0

---

### Root Cause 4: Full Sort for Elite Selection [MEDIUM]

**Source**: `grasp_preshaping/src/predictor.rs:427-439`

```rust
pub fn select_elite_indices(particles: &[SmcParticle], elite_ratio: f64) -> Vec<usize> {
    let mut indexed: Vec<usize> = (0..particles.len()).collect();
    indexed.sort_by(|&a, &b| { /* full sort */ });
    indexed.truncate(n_elite);
}
```

This sorts all N particles to take the top 5% (`ELITE_RATIO = 0.05`). At 10K particles, sorting is fast (~100µs). But at 1M particles, `O(N log N)` sorting every iteration becomes noticeable.

**Also**: `select_best_grasps` at `c_api.rs:187-195` sorts the full scored set to find the top 2. This is also `O(N log N)` when a single-pass max would be `O(N)`.

**Verified fix**: 
- Replace `select_elite_indices` with `select_nth_unstable` (partial sort) or a heap-based top-K selection. Rust's `select_nth_unstable_by` is O(N) average case.
- Replace `select_best_grasps` with a single-pass top-2 scan.

**Feature tradeoff**: None. Same results, different algorithm.

---

### Root Cause 5: Debug Cloning Across All Iterations [MEDIUM-HIGH]

**Source**: `grasp_preshaping/src/c_api.rs:421-554`

```rust
let mut all_debug: Option<Vec<(usize, ScoredGrasp, usize, SmcParticle)>> = ...;
// ...
for iteration in 0..n_iterations {
    // ...
    if let Some(ref mut debug) = all_debug {
        for (i, sg) in scored.iter().enumerate() {
            debug.push((i, sg.clone(), iteration, particles[i].clone()));
        }
    }
}
```

With `DEBUG_VISUALIZATION = true` (which is the current default at `config.rs:71`):
- 10K particles × 5 iterations = 50K entries
- Each entry clones a `ScoredGrasp` (which contains a `GraspScoreResult`) AND a full `SmcParticle` (which contains a `DualQuaternion` + other fields)
- Then the entire history is mapped into `ScoredGraspExport` structs and written to disk

At 1M samples, this would be 5M clone operations + a massive npz write.

**Verified fix**: 
- Set `DEBUG_VISUALIZATION = false` for production (this is a compile-time constant, so the compiler eliminates the entire path)
- If debug is needed, store only the final iteration's data, or sample every Nth particle

**Feature tradeoff**: Debug export is diagnostic only. No impact on grasp quality.

---

### Root Cause 6: Dense Scoring for Invalid Particles [MEDIUM]

**Source**: `grasp_preshaping/src/planner.rs:289-438`

The old code returned `None` immediately for:
- No collision (Tier 4) → `None`
- Start-position collision (Tier 3) → `None`
- Too few fingers (Tier 2) → `None`

The new code returns a `GraspScoreResult` for every case:
- **Tier 4 (no collision)**: calls `min_tsdf_distance()` which evaluates all 25 sweep points at mid-closure, doing 25 TSDF lookups + 25 matrix multiplications. Returns a proximity score of 0.0–0.05.
- **Tier 2 (soft rejection)**: iterates over all sweep points for each missing finger group (thumb: ~3 points, index: ~8 points), doing TSDF lookups for each. Returns a penalty-adjusted score.
- **Tier 3 (start collision)**: returns immediately with `contact_score = 0.1` — this is cheap.

The Tier 4 proximity scoring is the most expensive: for a particle that misses the object entirely, we still do 25 TSDF queries. At 10K particles, if 80% are Tier 4, that's 200K extra TSDF lookups per iteration.

**Verified fix**: Replace the full `min_tsdf_distance` scan with a simplified heuristic:
- Use only 2–3 key points (e.g., palm center, index tip, thumb tip) instead of all 25 sweep points
- Or use a precomputed point cloud centroid distance as a rough proximity signal
- The SMC optimizer mainly needs directional gradient information; a coarse signal is sufficient for Tier 4 particles

**Feature tradeoff**: The proximity score becomes less precise, but it was only 0.0–0.05 anyway (vs 0.8–1.0 for valid grasps). The SMC should still converge because the meaningful signal is in Tier 1 grasps.

---

### Root Cause 7: Config Parameter Changes [LOW-MEDIUM]

**Source**: `grasp_preshaping/src/config.rs` vs `grasp_preshaping_old/src/config.rs`

| Parameter | Old | New | Impact |
|---|---|---|---|
| `COLLISION_TOL_M` | 0.005 | 0.01 | 2× wider collision band → more Tier 2/3 hits, fewer Tier 4 |
| `BINARY_SEARCH_TOL` | 0.01 | 0.05 | 5× coarser binary search → fewer iterations in `refine_binary` (slightly faster) |
| `MAX_TSDF_DIM_M` | 0.3 | 0.5 | ~4.6× larger grid volume → more voxels in TSDF |
| `GRASP_WEIGHT_CONTACT_COUNT` | 1.5 | N/A (replaced by `GRASP_WEIGHT_CONTACT_SCORE = 3.0`) | Changes score distribution |
| `RAY_ALIGNMENT_THRESHOLD` | 0.8 | 0.9 | Stricter alignment → more surface points rejected in camera sign check |

The `MAX_TSDF_DIM_M` change from 0.3 to 0.5 is significant: grid volume scales cubically, so this is roughly a (0.5/0.3)³ ≈ 4.6× increase in voxel count. This affects both the camera sign pass and the SQ blend pass.

**Verified fix**: Revert `MAX_TSDF_DIM_M` to 0.3 unless the larger ROI is demonstrably needed. If the SMC needs a wider search space, consider using a separate (larger) ROI for sampling vs. a tighter ROI for the TSDF grid.

---

### Root Cause 8: Out-of-Bounds TSDF Query Semantics [LOW]

**Source**: 
- Old: `grasp_preshaping_old/src/pointcloud_helper.rs:131-133` — clamps to grid boundary
- New: `grasp_preshaping/src/pointcloud_helper.rs:132-137` — returns `f32::MAX` immediately

The old code clamped out-of-bounds queries to the nearest boundary voxel and interpolated. The new code returns `f32::MAX` immediately.

This is actually a **reversal** — the old code clamped, the new code rejects. The new behavior is more correct (points outside the TSDF should not produce interpolated values), but it means:
- Points far from the object always get `f32::MAX` from `get_distance`
- The sweep loop in `sweep_for_collision` cannot early-exit based on boundary proximity
- `min_tsdf_distance` returns `f32::MAX` for all out-of-bounds points

**Impact**: Low. This mainly affects Tier 4 particles that are far from the object. The `f32::MAX` return causes them to get proximity_score = 0.0, which is correct behavior.

**Recommendation**: Keep the new semantics (return `f32::MAX` for out-of-bounds). It is more correct.

---

## Part 2: Additional Investigation Items

These are smaller issues or potential optimizations worth investigating:

### A. `find_active_contacts` Deep-Penetration Filter

**Source**: `grasp_preshaping/src/planner.rs:568`
```rust
if dist >= threshold || dist < -threshold {
    continue;
}
```

The new code skips contacts with strongly negative distances (deep penetration). The old code (`grasp_preshaping_old/src/planner.rs:453`) only skipped `dist >= threshold`. This is a correctness improvement but may slightly reduce the number of active contacts found, which could affect scoring. Worth monitoring but not a performance issue.

### B. `FingerGroup` Enum and HashSet Allocation

**Source**: `grasp_preshaping/src/planner.rs:362-366`
```rust
let mut finger_set = HashSet::new();
for &(contact, _) in &active_with_contacts {
    finger_set.insert(contact.finger_group());
}
```

A `HashSet` is allocated for every scored grasp to check finger diversity. With only 3–4 possible finger groups, a fixed-size array or bitfield would be faster and avoid heap allocation. At 10K particles × 5 iterations, this is 50K HashSet allocations.

### C. `roi_prediction` Regression (+11%)

The new `roi_prediction` is slightly slower (3.21ms vs 2.90ms). This is likely due to the wrist rotation sampling added to `sample_future_poses` (the new code applies a wrist rotation DQ multiplication per sample). Not a major issue but worth noting.

### D. `compute_grasp_type_weights` Per-Iteration Overhead

**Source**: `grasp_preshaping/src/predictor.rs:444-493`

This function iterates all particles, computes per-type averages, shifts, floors, and renormalizes. At 10K particles it's trivial. At 1M it's still O(N) but with several passes. Could be fused into the scoring loop.

### E. `resample_around_elites` Allocates `Vec<SmcParticle>` Every Iteration

**Source**: `grasp_preshaping/src/predictor.rs:318`

A new `Vec<SmcParticle>` is allocated every SMC iteration. At 10K particles this is fine, but at 1M it's a 80+ MB allocation per iteration. Consider reusing the buffer.

---

## Part 3: Implementation Plan

### Phase 1: Zero-Feature-Loss Quick Wins

These changes have no impact on grasp quality or feature set:

- [ ] **1.1. Set `DEBUG_VISUALIZATION = false` in `grasp_preshaping/src/config.rs:71`.**  
  Rationale: This is the single biggest hidden tax. With `true`, every pipeline invocation clones all particles across all iterations and writes a debug npz. With `false`, the compiler eliminates the entire code path. No feature loss — debug can be re-enabled for development.

- [ ] **1.2. Replace `select_elite_indices` full sort with `select_nth_unstable_by` in `grasp_preshaping/src/predictor.rs:427-439`.**  
  Rationale: We only need the top `elite_ratio` fraction, not a full ordering. `select_nth_unstable_by` is O(N) average vs O(N log N) for full sort. Same results.

- [ ] **1.3. Replace `select_best_grasps` full sort with a single-pass top-2 scan in `grasp_preshaping/src/c_api.rs:187-195`.**  
  Rationale: Finding the best and second-best is O(N) with a single pass. No need to sort the entire array.

- [ ] **1.4. Replace `HashSet` in finger diversity check with a bitfield in `grasp_preshaping/src/planner.rs:362-366`.**  
  Rationale: There are only 3–4 finger groups. A `u8` bitfield avoids heap allocation and is faster. Same logic, different data structure.

### Phase 2: Camera Sign Correction Fix

- [ ] **2.1. Restore `nearest`-based O(1) camera sign lookup in `grasp_preshaping/src/pointcloud_helper.rs:417-544`.**  
  Rationale: The `nearest` array already tracks the closest surface point per voxel from the brushfire pass. Use it for the primary camera distance comparison (like the old code did). Keep the new per-camera voting logic (`inside_votes` / `outside_votes`) but use the `nearest` point instead of scanning all surface points.
  
  Specifically, replace the inner `for &sp in &surface_points` loop (lines 476-507) with a single lookup using `nearest[flat_idx]`, similar to old code lines 429-430.

- [ ] **2.2. Keep the SQ tiebreaker for opposing-camera disagreements.**  
  Rationale: The new code already has this at lines 535-543. When cameras disagree (`inside_votes > 0 && outside_votes > 0`), use `sq.is_inside()` as tiebreaker. This preserves the opposing-camera fix without the O(P) scan.

- [ ] **2.3. Remove the `surface_points` collection (lines 431-438) since it will no longer be needed.**  
  Rationale: Eliminates the allocation and the O(P) scan entirely.

### Phase 3: Sparse Superquadric Evaluation

- [ ] **3.1. Add a candidate mask to the SQ blend pass in `grasp_preshaping/src/pointcloud_helper.rs:560-643`.**  
  Rationale: Only evaluate `taubin_distance` for voxels that actually need SQ correction. Skip voxels where:
  - The camera sign is confident (positive, low distance, multiple cameras agree)
  - The voxel is far from the SQ surface (can check with a cheap `evaluate` first)
  
  Implementation: Before the `par_iter_mut` loop, compute the SQ bounding box (using `a`, `b`, `c`, `translation`, `rotation`). Only process voxels inside or near this bounding box.

- [ ] **3.2. Consider using `evaluate` (cheap) as a gate before `taubin_distance` (expensive).**  
  Rationale: `evaluate` is one SQ evaluation. `taubin_distance` is 4–5 SQ evaluations. If `evaluate` shows the voxel is far from the SQ surface (|F| >> 0), we can skip the expensive Taubin refinement and just use F/||grad_F|| as a rough distance.

### Phase 4: Tier 4 Proximity Score Simplification

- [ ] **4.1. Replace `min_tsdf_distance` full sweep with a sparse key-point check in `grasp_preshaping/src/planner.rs:301-314`.**  
  Rationale: For Tier 4 (no collision) particles, we currently check all 25 sweep points. Instead, check only 2–3 key points (e.g., palm center at `PalmDistRadi` or `PalmDistUlna`, and the index tip at `IndexTip`). This reduces TSDF lookups from 25 to 2–3 per Tier 4 particle.

- [ ] **4.2. Consider using point-cloud centroid distance as an even cheaper proxy.**  
  Rationale: The point cloud centroid is already computed during Morton encoding. A simple Euclidean distance from the hand position to the centroid gives a rough proximity signal without any TSDF lookups. Less precise but essentially free.

### Phase 5: Config Tuning

- [ ] **5.1. Revert `MAX_TSDF_DIM_M` from 0.5 to 0.3 in `grasp_preshaping/src/config.rs:19`.**  
  Rationale: The 0.5 value creates a ~4.6× larger grid volume. If the SMC needs a wider search space, use the larger ROI for particle sampling but clip the TSDF to 0.3m for grid construction. If 0.5 is truly needed, accept the grid cost but ensure the SQ and camera passes are sparse (Phases 2–3).

- [ ] **5.2. Review `COLLISION_TOL_M` = 0.01 vs old 0.005.**  
  Rationale: The wider tolerance means more particles hit Tier 2/3 instead of Tier 4, which triggers the expensive missing-finger penalty calculations. If the wider tolerance is needed for grasp quality, keep it. If not, reverting to 0.005 reduces Tier 2/3 hit rates.

### Phase 6: Additional Optimizations (Lower Priority)

- [ ] **6.1. Fuse `compute_grasp_type_weights` into the scoring loop.**  
  Rationale: Instead of a separate pass over all particles, accumulate per-type score sums during `score_all_particles`. Saves one full iteration over the particle array.

- [ ] **6.2. Reuse the particle `Vec` across SMC iterations.**  
  Rationale: `resample_around_elites` currently returns a new `Vec<SmcParticle>`. At 1M particles, this is an 80+ MB allocation per iteration. Pass a mutable buffer instead.

- [ ] **6.3. Consider adaptive sample counts per SMC iteration.**  
  Rationale: Start with fewer samples (e.g., 2K) in iteration 0, then increase to 10K in later iterations when the proposal distribution is narrower. This reduces the total number of scoring calls.

---

## Part 4: Verification Criteria

After implementing the fixes, verify:

- [ ] `full_pipeline_predict_to_score` (10k samples, no cameras, no SQ) stays at ~126ms or improves
- [ ] `smc_pipeline` (10k samples, no cameras, no SQ, 5 iter) drops from ~689ms to under 400ms
- [ ] `tsdf_construction_with_superquadric` (5k pts, 1 camera, SQ on) stays at ~8ms or improves
- [ ] The production path (`grasp_preshaping_compute` with cameras + SQ) processes 1M samples in under 30 seconds
- [ ] All existing unit tests pass (especially the SQ backside tests in `pointcloud_helper.rs`)
- [ ] SMC convergence rate is not degraded (valid grasps still found within 5 iterations)

---

## Part 5: Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Restoring `nearest`-based sign loses opposing-camera accuracy | Low | Medium | SQ tiebreaker already handles this case |
| Sparse SQ evaluation misses backside voxels | Medium | Medium | Use conservative bounding box inflation; validate with existing SQ tests |
| Simplified Tier 4 scoring reduces SMC gradient quality | Low | Low | Tier 4 scores are 0.0–0.05; the real signal is in Tier 1 (0.8–1.0) |
| `MAX_TSDF_DIM_M` revert clips valid grasps at ROI boundary | Low | High | Check if SMC samples actually need the full 0.5m range; if so, keep 0.5 but ensure sparse passes |
| `DEBUG_VISUALIZATION = false` removes diagnostic capability | None | None | It's a compile-time flag; re-enable for development |

---

## Part 6: Expected Outcome

If all phases are implemented:

1. **Camera sign pass**: O(V × C) instead of O(V × C × P) — estimated 10–50× speedup for this pass
2. **SQ blend pass**: 70–90% fewer voxels evaluated — estimated 3–10× speedup for this pass
3. **Debug export**: eliminated entirely in production — saves 50K+ clone operations per invocation
4. **Sorting**: O(N) instead of O(N log N) — saves ~10ms per iteration at 1M samples
5. **Tier 4 scoring**: 8–12× fewer TSDF lookups — saves ~20% of scoring time

Combined, these should bring the 1M-sample production path from ~112s down to the 15–30s range, with further SMC tuning potentially bringing it closer to the 6.5s baseline.


User:
Very good list. I have few notes:
- For the SQ full evaluation: My idea was to construct the TSDF grid using only point cloud, and then take only the negative voxels, inflate them the voxele mask using truncation band width, and then use that mask to evaluate the SQ only on those voxels. I think we should be able to make sure that mask includes all the voxels that would need to be opdated, but another approch would also be to estimate a geometric object from the SQ and inflate that. I am honestly not sure what to do, or if it is even worth it, but I thnk we need to be carefull about the whole belnding and making sure to integrate the pointcloud and the SQ in a good way.
- For SMC iterations: I will say the thing is not totally fair, since the old pipline would lieklæy require 1M samples to match 100k samples or maybe enven less on the new pipeline, so it is not a totally lost cause. But yes we do need to be faster to actually meet our performance goals.
- It is a very intersting isight that the debug export is liekly adding a lot of overhead. I did not think of that, but we should definetly try to disable it and see.
- I am not sure about the TSDF max dim, we could try to lower it. I am not sure what the right value should be for it to be honest. I do wonder however if the SQ needs to be integrated into how we calcualte the bounding box of the TSDF, perhaps we need to look at that.
- Please also take a look at the Out-of-Bounds TSDF Semantics, it is actually a thing on my list aswell.

Other than that i think you ideas are good. Can you revise the plan to include this feedback? then i will hand it of to implementation.