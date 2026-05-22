# SQ Distance: f32::MAX Beyond Truncation + Early-Out Optimization

## Objective

1. Assign `f32::MAX` to voxels whose SQ distance exceeds the truncation threshold (they are beyond meaningful data and should not participate in Marching Cubes).
2. Add an early-out check using the cheap `evaluate()` call before computing the expensive `taubin_distance()` to skip voxels clearly far from the SQ surface.

## Root Cause Analysis

### Bug: No f32::MAX beyond truncation

In `pointcloud_helper.rs:560-643`, the SQ sign-correction pass processes every voxel in the grid:

1. **`f32::MAX` voxels** (line 574-589): These get SQ distance clamped to `[-trunc, trunc]`. They should instead get `f32::MAX` if the SQ says they're far outside the object.

2. **Regular voxels** (line 601-642): These get blended SQ distances. If the final blended distance exceeds the truncation threshold, it should be `f32::MAX`.

Without `f32::MAX` assignment, Marching Cubes may find zero-crossings at the grid boundary, creating phantom surfaces. Also, the visualization was showing a "full cube" because all voxels had valid distances.

### Performance: Taubin distance is expensive

`taubin_distance()` calls `evaluate()` + `gradient()` + another `evaluate()` + `gradient()` (Newton refinement) = 4 transcendental function evaluations per voxel. For voxels clearly far from the SQ surface (e.g., `F(x,y,z) > 5.0`), the distance is guaranteed to exceed the truncation threshold, so the full Taubin computation is wasted.

A cheap pre-check: call `evaluate()` first (1 call). If `|F|` is very large, the distance will exceed truncation — assign `f32::MAX` or the clamped value directly without computing the gradient.

## Implementation Plan

### Part 1: Add f32::MAX assignment beyond truncation

- [ ] Task 1.1. For `f32::MAX` voxels (line 574-589): if the SQ distance in cell units exceeds `truncation_cells`, assign `f32::MAX` instead of the clamped distance
  - Rationale: These voxels are beyond meaningful data. Marching Cubes should ignore them.

- [ ] Task 1.2. For disagree-path voxels (line 612-632): after computing the final blended distance, if `abs(blended) > truncation_cells`, assign `f32::MAX` with the appropriate sign
  - Actually: assign `f32::MAX` (unsigned, since it's beyond the band). Marching Cubes only cares about zero-crossings, so voxels beyond the truncation band are irrelevant.
  - Rationale: Prevents phantom surfaces at the grid edge.

- [ ] Task 1.3. For agree-path voxels (line 633-641): same check after blending
  - Rationale: Consistency.

### Part 2: Early-out optimization

- [ ] Task 2.1. Add a helper method `taubin_distance_or_max` to `SuperquadricParams` that:
  1. Calls `evaluate(world_pt)` first (cheap)
  2. If `|F| > threshold` (where threshold is derived from truncation distance), returns `f32::MAX` immediately
  3. Otherwise calls the full `taubin_distance()` 
  - The threshold: if `|F| / grad_norm_typical > truncation_cells * resolution`, then distance exceeds truncation. Since `grad_norm` varies, use a conservative estimate: `|F| > truncation_cells * resolution * min_expected_grad_norm`. For simplicity, use `|F| > 10.0` as a generous cutoff (the gradient norm is typically 1-10 for these shapes, so `|F| > 10` means distance > 1-10 cells, well beyond the truncation band for most cases).
  - Actually, a better approach: pass the `max_distance_m` (truncation_cells * resolution) to the function. If `F > 0` (outside) and `F > max_distance_m * estimated_grad_norm`, skip. But we don't know `estimated_grad_norm` without computing the gradient.
  - Simplest correct approach: compute `evaluate()`. If `|F| > some_large_value`, return `f32::MAX`. Otherwise compute full Taubin. The `some_large_value` can be calibrated: for a unit sphere at distance `d` from surface, `F = (1+d)^2 - 1 = 2d + d^2`. At `d = 0.04m` (truncation distance), `F ≈ 0.08`. For a box with `e=0.1`, the gradient is much steeper, so `F` grows faster. A conservative cutoff of `|F| > 5.0` should work for all templates.
  - Rationale: Avoids 3 out of 4 transcendental evaluations for far voxels.

- [ ] Task 2.2. Use `taubin_distance_or_max` in the SQ sign-correction pass instead of `taubin_distance`
  - Pass `truncation_cells * resolution_m` as the max distance
  - Rationale: The hot path now skips expensive computation for ~30-50% of voxels (those at the grid edge).

### Part 3: Update the f32::MAX handling

- [ ] Task 3.1. In the SQ correction pass, when `taubin_distance_or_max` returns `f32::MAX`, assign `f32::MAX` to the voxel
  - This means the voxel is "unvisited" from Marching Cubes' perspective
  - Rationale: Consistent with the BFS convention.

## Verification Criteria

- Voxels beyond the truncation band from the SQ surface get `f32::MAX` (not a clamped distance)
- Marching Cubes does not produce phantom surfaces at the grid edge
- The early-out optimization reduces SQ evaluation time by skipping far voxels
- All 51 existing tests pass
- New test: verify that a voxel far outside the SQ gets `f32::MAX`

## Potential Risks and Mitigations

1. **Early-out threshold too aggressive**
   - If `|F| > 5.0` is too low, some voxels near the SQ surface but with large F values (e.g., box corners) get incorrectly skipped.
   - Mitigation: Use a generous threshold. For the box template with e=0.1, the gradient norm near the surface is ~1-10, so `|F| = 5` corresponds to distance 0.5-5 cells. With truncation at 8 cells, a threshold of `|F| > 20.0` is safer.

2. **f32::MAX breaks the sign convention**
   - The TSDF uses signed distances: negative = inside, positive = outside, f32::MAX = unvisited.
   - Mitigation: Only assign `f32::MAX` to voxels that are clearly outside the truncation band (distance > truncation_cells in either direction). Inside voxels at the grid edge should keep their negative distance.

3. **Performance gain may be modest**
   - If most voxels are near the SQ surface, the early-out rarely fires.
   - Mitigation: The early-out is essentially free (one `evaluate()` call vs four). Even a 10% skip rate is worthwhile.

## Alternative Approaches

1. **Bounding box pre-filter**: Before evaluating the SQ, check if the voxel is within `max(a,b,c) + truncation_distance` from the SQ center. This is an axis-aligned check that's much cheaper than `evaluate()`.
   - Trade-off: Requires transforming the voxel to the SQ's local frame first (one matrix multiply). Still cheaper than `evaluate()` but adds complexity.
   - Decision: Can be added later as a further optimization on top of the `evaluate()` pre-check.

2. **Use `evaluate()` sign only for far voxels**: Instead of computing Taubin distance at all, use just the `evaluate()` sign for voxels beyond a certain distance. The Taubin accuracy doesn't matter for far voxels since Marching Cubes ignores them anyway.
   - Trade-off: Simpler but less accurate for the blending zone.
   - Decision: The `taubin_distance_or_max` approach is better — it uses the sign from `evaluate()` and skips the distance computation for far voxels.
