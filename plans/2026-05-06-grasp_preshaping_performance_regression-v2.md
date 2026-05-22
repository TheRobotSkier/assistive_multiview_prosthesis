# Grasp Preshaping Performance Regression — Verified Analysis & Fix Plan

## Objective

Identify and resolve the performance regression from ~6.5s (old, 1M samples, 1 iteration) to ~112s (new, 1M samples, up to 5 SMC iterations) in the grasp preshaping pipeline, while retaining the new SMC optimization, superquadric backside estimation, and dense scoring features.

---

## Part 1: Verified Root Cause Analysis

### Benchmark Context

| Benchmark | Old | New | Delta |
|---|---|---|---|
| `roi_prediction` | 2.90 ms | 3.21 ms | +11% (minor, from wrist rotation) |
| `pointcloud_prune` | 2.03 ms | 2.12 ms | negligible |
| `tsdf_construction` (no cameras, no SQ) | 2.63 ms | 2.59 ms | **faster** |
| `full_pipeline_predict_to_score` (10k, no cameras, no SQ) | 120 ms | 126 ms | +5% (minor) |
| `smc_pipeline` (10k, no cameras, no SQ, 5 iter) | N/A | 689 ms | new feature |
| `score_cylindrical` | 15.6 µs | 12.3 µs | **faster** |
| `score_pinch` | 9.9 µs | 9.0 µs | **faster** |
| `score_lateral` | 11.0 µs | 8.2 µs | **faster** |
| `superquadric_fitting` | N/A | 3.4 ms | new feature |
| `tsdf_construction_with_superquadric` | N/A | 7.9 ms | new feature |

**Key insight**: The micro-kernels (scoring, bare TSDF) are not the problem. The regression is in the **production orchestration path**: camera-based sign correction, SQ blending, SMC iteration overhead, debug export, and sorting.

**Important context on SMC vs. old pipeline**: The old pipeline needed ~1M samples to find good grasps in a single pass. The new SMC pipeline with iterative refinement should converge with far fewer total evaluations (e.g., 100K samples across 5 iterations might match or exceed 1M one-shot samples). So the 5× iteration multiplier is not a pure loss — it's an investment in sample efficiency. The goal is to make each iteration fast enough that the total time is competitive.

---

### Root Cause 1: Camera Sign Correction — O(V × C × P) Instead of O(V × C) [CRITICAL]

**Old code**: `grasp_preshaping_old/src/pointcloud_helper.rs:429-460`
- Uses the brushfire `nearest[flat_idx]` array to get the single closest surface point per voxel
- Compares camera-to-voxel distance vs camera-to-nearest-surface distance: O(1) per camera per voxel
- Sign logic: `behind_count > n_cams / 2 && inside_votes > behind_count / 2`

**New code**: `grasp_preshaping/src/pointcloud_helper.rs:459-526`
- Builds a `surface_points` vector from all unique Morton cells
- For each voxel and each camera, scans **every surface point** to find the best ray-aligned occluder
- Inner loop per camera per voxel: dot product, norm, alignment check, perpendicular distance

**Impact**: For ~250K voxels, ~5K surface points, and 2 cameras: ~2.5 billion geometric operations. This is the single biggest regression when cameras are enabled.

**Why it was changed**: The old `nearest`-based approach gave misleading results for opposing cameras (the nearest surface point might be on the wrong side). The new code finds the best ray-aligned occluder per camera to handle opposing views correctly.

**Fix**: Restore the `nearest`-based O(1) lookup as the primary path. The new code already has an SQ tiebreaker at `pointcloud_helper.rs:535-543` for when cameras disagree. This means:
- Use `nearest` for the primary camera distance comparison (fast, correct for single-camera and same-side multi-camera)
- When cameras disagree (`inside_votes > 0 && outside_votes > 0`), use `sq.is_inside()` as tiebreaker (already implemented)
- Remove the `surface_points` collection entirely

**Feature tradeoff**: None. The SQ tiebreaker handles the opposing-camera case that motivated the original change.

---

### Root Cause 2: Superquadric Full-Grid Evaluation [HIGH]

**Source**: `grasp_preshaping/src/pointcloud_helper.rs:560-643`

The SQ blend pass iterates over **every voxel** in the grid. For each non-surface, non-MAX voxel:
1. `sq.taubin_distance(vw)` — internally: `evaluate` + `gradient` + projected `evaluate` + projected `gradient` = 4 heavy SQ evaluations
2. `sq.evaluate(vw)` — 1 more SQ evaluation for sign check
3. Smoothstep blending math

**Cost**: ~5 `evaluate` calls per voxel, each involving multiple `.powf()` operations. For 250K voxels, that's ~1.25M heavy transcendental function calls.

**Your mask idea — analysis**:

The idea is: after the brushfire + camera sign pass, take only the voxels with negative sign (inside the object), inflate that mask by the truncation band width, and evaluate SQ only on voxels in that inflated mask.

This is a good approach because:
- Negative voxels are the ones most likely to need SQ correction (the camera sign is least reliable there)
- Inflating by the truncation band ensures we also catch the transition zone where blending happens
- The SQ exists to correct the backside — and the backside is where the point cloud produces negative voxels

**Concern**: Some voxels that are positive (outside) near the backside might also need SQ correction if the camera sign is wrong. However:
- The camera sign pass already handles the case where cameras disagree (SQ tiebreaker)
- For positive voxels where cameras agree they're outside, the camera authority is correct and SQ correction is unnecessary
- The only positive voxels that might need SQ are those in the "disagree" zone, which are handled by the camera pass tiebreaker

**Recommendation**: Build the mask as follows:
1. After the brushfire + camera sign pass, identify all voxels where `distance[flat_idx] < 0.0` (inside)
2. Inflate this set by `truncation_cells` in all directions (morphological dilation)
3. Also include any voxels where `distance[flat_idx] == f32::MAX` (unobserved) that fall within the SQ bounding box
4. Only run the SQ blend pass on voxels in this mask

This should reduce the SQ evaluation set from ~100% of the grid to roughly 10-30%.

**Alternative**: Instead of building a voxel mask from the negative region, compute the SQ's axis-aligned bounding box in world space (using `a`, `b`, `c`, `translation`, `rotation`), inflate it by the truncation band, and only process voxels inside this AABB. This is simpler and potentially more inclusive, but may process more voxels than necessary if the SQ is much larger than the actual object.

**Blending concern**: The current blending logic (lines 612-642) does smooth transitions between camera authority and SQ authority based on distance bands. When we restrict SQ evaluation to the masked region, we need to ensure:
- Voxels at the mask boundary still get proper blending (the inflation by truncation band should handle this)
- Voxels outside the mask keep their camera/brushfire values unchanged (which is correct — they don't need SQ)

---

### Root Cause 3: SMC Iteration Multiplier [HIGH — BY DESIGN, BUT AMPLIFIED]

**Source**: `grasp_preshaping/src/c_api.rs:435-489`

Up to 5 iterations × 10K particles = 50K scoring calls vs old 10K × 1 = 10K. This is a 5× multiplier by design.

**Important context**: The old pipeline needed ~1M samples for good results because it was a single-pass random search. The SMC pipeline should converge with far fewer total evaluations because it focuses the search around high-scoring regions. So the fair comparison is not 10K×5 vs 10K×1, but rather something like 100K total SMC evaluations vs 1M one-shot evaluations.

However, the per-iteration overhead is amplified by debug cloning (Root Cause 5), full sorting (Root Cause 4), and dense scoring for invalid particles (Root Cause 6). Making each iteration lean is critical.

**Fix**: Reduce per-iteration overhead through the other root cause fixes. The iteration count itself is a design choice that should be tuned based on convergence data.

---

### Root Cause 4: Full Sort for Elite Selection [MEDIUM]

**Source**: `grasp_preshaping/src/predictor.rs:427-439`

`select_elite_indices` sorts all N particles to take the top 5%. At 10K this is fast (~100µs), but at higher sample counts it becomes noticeable.

`select_best_grasps` at `c_api.rs:187-195` also sorts the full scored set to find the top 2.

**Fix**: 
- Replace `select_elite_indices` with `select_nth_unstable_by` (Rust's partial sort, O(N) average)
- Replace `select_best_grasps` with a single-pass top-2 scan

**Feature tradeoff**: None. Same results, different algorithm.

---

### Root Cause 5: Debug Cloning Across All Iterations [MEDIUM-HIGH]

**Source**: `grasp_preshaping/src/c_api.rs:421-554`

With `DEBUG_VISUALIZATION = true` (current default at `config.rs:71`):
- 10K particles × 5 iterations = 50K entries in `all_debug`
- Each entry clones a `ScoredGrasp` AND a full `SmcParticle` (containing a `DualQuaternion`)
- Then the entire history is mapped into `ScoredGraspExport` structs and written to npz

At 1M samples this would be 5M clone operations + a massive npz write. Even at 10K, the allocation and clone overhead is significant.

**Fix**: Set `DEBUG_VISUALIZATION = false` for production. This is a compile-time constant, so the compiler eliminates the entire code path at zero runtime cost. For development, keep it configurable.

**Feature tradeoff**: Debug export is diagnostic only. No impact on grasp quality.

---

### Root Cause 6: Dense Scoring for Invalid Particles [MEDIUM]

**Source**: `grasp_preshaping/src/planner.rs:289-438`

The old code returned `None` immediately for no-collision, start-collision, and too-few-fingers cases. The new code returns a `GraspScoreResult` for every case:

- **Tier 4 (no collision)**: calls `min_tsdf_distance()` which evaluates all 25 sweep points at mid-closure. Each sweep point requires a matrix multiplication + TSDF lookup. Returns proximity_score 0.0–0.05.
- **Tier 2 (soft rejection)**: iterates over sweep points for each missing finger group (thumb: ~3 points, index: ~8 points), doing TSDF lookups for each. Returns a penalty-adjusted score.
- **Tier 3 (start collision)**: returns immediately with `contact_score = 0.1` — cheap.

The Tier 4 proximity scoring is the most expensive: for particles that miss the object entirely, we do 25 TSDF queries. If 80% of particles are Tier 4, that's 200K extra TSDF lookups per iteration at 10K samples.

**Fix**: Replace the full `min_tsdf_distance` sweep with a sparse key-point check (2-3 points: palm center, index tip, thumb tip). The SMC optimizer mainly needs coarse directional information for Tier 4 particles; the real signal is in Tier 1 (0.8–1.0) vs Tier 4 (0.0–0.05).

**Feature tradeoff**: Less precise proximity gradient for far-from-object particles. Acceptable because the SMC resampling focuses on high-scoring particles anyway.

---

### Root Cause 7: Config Parameter Changes [LOW-MEDIUM]

**Source**: `grasp_preshaping/src/config.rs` vs `grasp_preshaping_old/src/config.rs`

| Parameter | Old | New | Impact |
|---|---|---|---|
| `COLLISION_TOL_M` | 0.005 | 0.01 | 2× wider band → more Tier 2/3 hits |
| `BINARY_SEARCH_TOL` | 0.01 | 0.05 | 5× coarser → fewer refine iterations (slightly faster) |
| `MAX_TSDF_DIM_M` | 0.3 | 0.5 | ~4.6× larger grid volume |
| `RAY_ALIGNMENT_THRESHOLD` | 0.8 | 0.9 | Stricter alignment in camera sign check |

The `MAX_TSDF_DIM_M` change from 0.3 to 0.5 creates a (0.5/0.3)³ ≈ 4.6× larger grid volume. This affects both the camera sign pass and the SQ blend pass.

**Regarding SQ and bounding box integration**: Currently the TSDF grid size is determined solely by the pruned point cloud's Morton extents + truncation padding (`pointcloud_helper.rs:342-356`). The SQ is fitted to the same point cloud, so its bounding box is generally within the point cloud's AABB. However, the SQ might extend slightly beyond the observed points (it estimates the full object shape). If the SQ extends beyond the grid, those backside voxels won't exist in the TSDF and can't be corrected.

**Fix**: Consider expanding the TSDF grid to encompass the SQ bounding box (inflated by truncation band) when SQ backside estimation is enabled. This ensures the grid covers the full SQ-predicted shape. However, this conflicts with the goal of reducing grid size. The better approach is:
1. Keep the grid sized to the point cloud (current behavior)
2. Only expand if the SQ bounding box significantly exceeds the point cloud AABB
3. The SQ blend pass already handles `f32::MAX` voxels (lines 574-589), filling them with SQ distance values

---

### Root Cause 8: Out-of-Bounds TSDF Query Semantics [NEEDS REVISION]

**Source**: 
- Old: `grasp_preshaping_old/src/pointcloud_helper.rs:131-133` — **clamps** to grid boundary, then interpolates
- New: `grasp_preshaping/src/pointcloud_helper.rs:132-137` — returns `f32::MAX` immediately for out-of-bounds

The old code:
```rust
let gx = gx.max(0.0).min((self.width - 1) as f32);  // clamp
```

The new code:
```rust
if gx < 0.0 || gx >= (self.width - 1) as f32 {       // reject
    return f32::MAX;
}
```

**Analysis**: Both approaches have problems:

1. **Old (clamp)**: A point slightly outside the grid gets the boundary voxel's distance value. This can produce incorrect collision readings — a finger just outside the grid might read a small positive distance (near surface) or even a negative distance (inside), leading to false collisions or missed collisions. However, it does provide "smooth" behavior at the boundary.

2. **New (reject)**: A point outside the grid always gets `f32::MAX`. This means:
   - `sweep_for_collision` sees `f32::MAX > collision_tol` → no collision detected for out-of-bounds points
   - `min_tsdf_distance` returns `f32::MAX` → proximity_score = 0.0
   - This is semantically correct (we don't know what's outside the grid) but it means grasps near the grid boundary are treated as "no object nearby"

**The real issue**: Neither approach is fully satisfactory. The correct behavior depends on context:
- For **collision detection**: out-of-bounds should mean "no collision" (the new behavior is correct)
- For **contact scoring**: out-of-bounds should mean "no surface contact possible" (the new behavior is correct)
- For **proximity scoring (Tier 4)**: out-of-bounds means "we can't tell how close the object is" — returning `f32::MAX` gives proximity_score = 0.0, which provides no gradient toward the object

**Recommendation**: Keep the new behavior (return `f32::MAX` for out-of-bounds). It is more correct. The Tier 4 proximity scoring limitation can be addressed separately by using the point cloud centroid distance as a fallback heuristic for particles that are entirely out-of-bounds.

**Additional consideration**: With the SQ backside estimation filling in `f32::MAX` voxels (lines 574-589 of the SQ blend pass), the effective TSDF coverage extends beyond the brushfire reach. This means fewer queries will hit out-of-bounds in practice when SQ is enabled.

---

## Part 2: Additional Investigation Items

### A. `FingerGroup` HashSet Allocation

**Source**: `grasp_preshaping/src/planner.rs:362-366`

A `HashSet` is allocated for every scored grasp to check finger diversity (3-4 possible groups). A `u8` bitfield would avoid heap allocation. At 10K particles × 5 iterations = 50K HashSet allocations.

### B. `roi_prediction` Minor Regression (+11%)

The new code adds wrist rotation sampling per pose (`predictor.rs:174-188`), which adds a DQ multiplication per sample. Minor but worth noting.

### C. `compute_grasp_type_weights` Per-Iteration Overhead

**Source**: `grasp_preshaping/src/predictor.rs:444-493`

Iterates all particles with several passes. Could be fused into the scoring loop.

### D. `resample_around_elites` Allocates New Vec Every Iteration

**Source**: `grasp_preshaping/src/predictor.rs:318`

At 1M particles this is 80+ MB per iteration. Consider reusing the buffer.

### E. SQ Bounding Box vs. TSDF Grid Coverage

The SQ is fitted to the pruned point cloud, so its AABB is generally within the point cloud's extents. But the SQ predicts the full object shape, which may extend beyond the observed points. If the SQ extends beyond the grid, backside voxels won't exist. The SQ blend pass handles this for `f32::MAX` voxels (lines 574-589), but only if those voxels exist in the grid. If the grid is too small, the SQ-predicted backside may be cut off.

---

## Part 3: Implementation Plan

### Phase 1: Zero-Feature-Loss Quick Wins

These changes have no impact on grasp quality or feature set:

- [ ] **1.1. Set `DEBUG_VISUALIZATION = false` in `grasp_preshaping/src/config.rs:71`.**  
  File: `grasp_preshaping/src/config.rs:71`  
  Rationale: This is likely the single biggest hidden tax. With `true`, every pipeline invocation clones all particles across all iterations and writes a debug npz. With `false`, the compiler eliminates the entire code path. No feature loss — debug is re-enabled for development by flipping the constant.

- [ ] **1.2. Replace `select_elite_indices` full sort with `select_nth_unstable_by` in `grasp_preshaping/src/predictor.rs:427-439`.**  
  File: `grasp_preshaping/src/predictor.rs:431-438`  
  Rationale: We only need the top `elite_ratio` fraction, not a full ordering. `select_nth_unstable_by` is O(N) average vs O(N log N) for full sort. Same results, different algorithm.

- [ ] **1.3. Replace `select_best_grasps` full sort with a single-pass top-2 scan in `grasp_preshaping/src/c_api.rs:187-195`.**  
  File: `grasp_preshaping/src/c_api.rs:187-195`  
  Rationale: Finding the best and second-best is O(N) with a single pass. No need to sort the entire array.

- [ ] **1.4. Replace `HashSet` in finger diversity check with a `u8` bitfield in `grasp_preshaping/src/planner.rs:362-366`.**  
  File: `grasp_preshaping/src/planner.rs:362-366`  
  Rationale: There are only 3-4 finger groups. A `u8` bitfield avoids heap allocation and is faster. Same logic, different data structure.

### Phase 2: Camera Sign Correction Fix (CRITICAL)

- [ ] **2.1. Restore `nearest`-based O(1) camera sign lookup in `grasp_preshaping/src/pointcloud_helper.rs:417-544`.**  
  File: `grasp_preshaping/src/pointcloud_helper.rs:459-526`  
  Rationale: Replace the inner `for &sp in &surface_points` loop (lines 476-507) with a single lookup using `nearest[flat_idx]`, similar to the old code at `grasp_preshaping_old/src/pointcloud_helper.rs:429-460`. Use the nearest surface point for the primary camera distance comparison. Keep the new per-camera voting logic (`inside_votes` / `outside_votes`) but base it on the `nearest` point.

- [ ] **2.2. Keep the SQ tiebreaker for opposing-camera disagreements (lines 535-543).**  
  Rationale: When cameras disagree (`inside_votes > 0 && outside_votes > 0`), the existing `sq.is_inside()` tiebreaker handles the case correctly. This is the reason the full surface scan was introduced, and the SQ tiebreaker solves it more efficiently.

- [ ] **2.3. Remove the `surface_points` collection (lines 431-438).**  
  File: `grasp_preshaping/src/pointcloud_helper.rs:431-438`  
  Rationale: Eliminates the allocation and the O(P) scan entirely. No longer needed with `nearest`-based lookup.

### Phase 3: Sparse Superquadric Evaluation

- [ ] **3.1. Build a negative-voxel mask after the camera sign pass, inflate by truncation band.**  
  File: `grasp_preshaping/src/pointcloud_helper.rs` (between the camera sign pass at ~line 544 and the SQ blend pass at ~line 560)  
  Rationale: After the brushfire + camera sign pass, every voxel has a signed distance. Build a boolean mask identifying voxels where:
  - `distance[flat_idx] < 0.0` (inside the object — these are the backside voxels that need SQ correction)
  - `distance[flat_idx] == f32::MAX` (unobserved — potentially SQ territory)
  Inflate this mask by `truncation_cells` in all directions to include the transition zone where blending occurs.

- [ ] **3.2. Modify the SQ blend pass to skip voxels outside the mask.**  
  File: `grasp_preshaping/src/pointcloud_helper.rs:566-643`  
  Rationale: Add an early `if !mask[flat_idx] { return; }` at the start of the `par_iter_mut` closure. This skips the expensive `taubin_distance` + `evaluate` calls for voxels that don't need SQ correction.

- [ ] **3.3. Consider using `evaluate` (cheap) as a gate before `taubin_distance` (expensive) for mask-boundary voxels.**  
  Rationale: `evaluate` is 1 SQ evaluation. `taubin_distance` is 4-5 SQ evaluations. If `evaluate` shows the voxel is far from the SQ surface (|F| >> 0), skip the expensive Taubin refinement and use the simpler F/||grad_F|| approximation.

- [ ] **3.4. Verify blending correctness at mask boundaries.**  
  Rationale: The smoothstep blending logic (lines 612-642) transitions between camera authority and SQ authority. Voxels just inside the mask boundary should get proper blending. The truncation-band inflation should ensure this, but it needs to be verified with the existing SQ tests.

### Phase 4: Tier 4 Proximity Score Simplification

- [ ] **4.1. Replace `min_tsdf_distance` full sweep with a sparse key-point check in `grasp_preshaping/src/planner.rs:301-314`.**  
  File: `grasp_preshaping/src/planner.rs:527-546` (the `min_tsdf_distance` function)  
  Rationale: Instead of evaluating all 25 sweep points, evaluate only 2-3 key points (e.g., palm center using `PalmDistRadi` or `PalmDistUlna` at locked sample 0, and `IndexTip` at mid-closure). This reduces TSDF lookups from 25 to 2-3 per Tier 4 particle.

- [ ] **4.2. Consider using point-cloud centroid distance as a fallback for out-of-bounds Tier 4 particles.**  
  Rationale: The point cloud centroid is already available from the Morton encoding step. A simple Euclidean distance from the hand position to the centroid gives a rough proximity signal without any TSDF lookups. This addresses the out-of-bounds limitation (Root Cause 8) for Tier 4 particles.

### Phase 5: Out-of-Bounds TSDF Semantics

- [ ] **5.1. Keep the current `f32::MAX` return for out-of-bounds queries.**  
  File: `grasp_preshaping/src/pointcloud_helper.rs:132-137`  
  Rationale: The current behavior is more correct than the old clamping approach. Out-of-bounds points should not produce interpolated values from boundary voxels.

- [ ] **5.2. Ensure the sweep and scoring logic handles `f32::MAX` correctly everywhere.**  
  Rationale: Verify that all callers of `get_distance` treat `f32::MAX` as "no data" rather than "very far from surface". The current collision detection (`< collision_tol`) correctly treats `f32::MAX` as no collision. The `find_active_contacts` threshold check (`dist >= threshold`) correctly skips `f32::MAX`. The `min_tsdf_distance` function returns `f32::MAX` when all points are out-of-bounds, which gives proximity_score = 0.0 — this is correct but provides no gradient (addressed in Phase 4).

### Phase 6: Config and Grid Sizing

- [ ] **6.1. Evaluate reducing `MAX_TSDF_DIM_M` from 0.5 to a smaller value (e.g., 0.35-0.4) in `grasp_preshaping/src/config.rs:19`.**  
  File: `grasp_preshaping/src/config.rs:19`  
  Rationale: The 0.5 value creates a ~4.6× larger grid volume than 0.3. The SMC search space may need the larger range, but the TSDF grid doesn't need to cover the entire search space — only the region around the object. Consider whether the ROI/AABB clipping already handles this adequately, or if a separate "TSDF max dim" vs "sampling max dim" is needed.

- [ ] **6.2. Investigate whether the SQ bounding box should influence the TSDF grid size.**  
  File: `grasp_preshaping/src/pointcloud_helper.rs:342-356`  
  Rationale: Currently the grid is sized to the point cloud Morton extents + truncation padding. When SQ backside estimation is enabled, the SQ may predict shape beyond the observed points. If the grid doesn't cover the SQ's extent, backside voxels are cut off. Consider expanding the grid to `max(point_cloud_extents, sq_bounding_box) + truncation_padding` when SQ is enabled. This should be a minor expansion since the SQ is fitted to the same point cloud.

### Phase 7: Additional Optimizations (Lower Priority)

- [ ] **7.1. Fuse `compute_grasp_type_weights` into the scoring loop.**  
  File: `grasp_preshaping/src/predictor.rs:444-493`, `grasp_preshaping/src/c_api.rs:473`  
  Rationale: Instead of a separate pass over all particles, accumulate per-type score sums during `score_all_particles`. Saves one full iteration over the particle array.

- [ ] **7.2. Reuse the particle `Vec` across SMC iterations.**  
  File: `grasp_preshaping/src/predictor.rs:318`, `grasp_preshaping/src/c_api.rs:480-488`  
  Rationale: `resample_around_elites` returns a new `Vec<SmcParticle>` every iteration. At 1M particles, this is an 80+ MB allocation per iteration. Pass a mutable buffer instead.

- [ ] **7.3. Consider adaptive sample counts per SMC iteration.**  
  File: `grasp_preshaping/src/c_api.rs:407`  
  Rationale: Start with fewer samples (e.g., 2K-5K) in early iterations when the proposal distribution is broad, then increase in later iterations when the distribution is narrower and more samples are needed for precise refinement.

---

## Part 4: Verification Criteria

After implementing the fixes, verify:

- [ ] `cargo test` passes in `grasp_preshaping/` (all unit tests, especially SQ backside tests)
- [ ] `full_pipeline_predict_to_score` (10k samples, no cameras, no SQ) stays at ~126ms or improves
- [ ] `smc_pipeline` (10k samples, no cameras, no SQ, 5 iter) drops from ~689ms to under 400ms
- [ ] `tsdf_construction_with_superquadric` (5k pts, 1 camera, SQ on) stays at ~8ms or improves
- [ ] The production path (`grasp_preshaping_compute` with cameras + SQ + SMC) is measurably faster
- [ ] SMC convergence rate is not degraded (valid grasps still found within 5 iterations)
- [ ] SQ backside completion still works correctly (existing tests + visual verification with debug export)

---

## Part 5: Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Restoring `nearest`-based sign loses opposing-camera accuracy | Low | Medium | SQ tiebreaker already handles this case at lines 535-543 |
| Sparse SQ mask misses backside voxels that need correction | Medium | Medium | Inflate mask by full truncation band; verify with existing SQ tests |
| Simplified Tier 4 scoring reduces SMC gradient quality | Low | Low | Tier 4 scores are 0.0-0.05; the real signal is in Tier 1 (0.8-1.0). Monitor convergence. |
| `DEBUG_VISUALIZATION = false` removes diagnostic capability | None | None | Compile-time flag; re-enable for development |
| Grid size reduction clips valid grasps at boundary | Low | Medium | The ROI/AABB clipping already limits the grid to the relevant region |
| SQ bounding box expansion increases grid size | Low | Low | The SQ is fitted to the same point cloud, so expansion should be minimal |

---

## Part 6: Expected Outcome

If all phases are implemented:

1. **Camera sign pass**: O(V × C) instead of O(V × C × P) — estimated 10-50× speedup for this pass when cameras are enabled
2. **SQ blend pass**: 70-90% fewer voxels evaluated — estimated 3-10× speedup for this pass
3. **Debug export**: eliminated entirely in production — saves 50K+ clone operations per invocation
4. **Sorting**: O(N) instead of O(N log N) — saves time at high sample counts
5. **Tier 4 scoring**: 8-12× fewer TSDF lookups — saves ~20% of scoring time
6. **Out-of-bounds**: correct semantics retained, Tier 4 fallback addresses gradient gap

Combined, these should bring the production path time down significantly while retaining all new features (SMC, SQ backside, dense scoring).
