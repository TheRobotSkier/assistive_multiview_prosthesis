# SQ Distance: f32::MAX Beyond Truncation + Bounding Box Early-Out

## Objective

1. Assign `f32::MAX` to voxels whose distance exceeds the truncation threshold, aligning with Domain A convention.
2. Add a cheap bounding-box early-out to skip SQ evaluation for voxels clearly too far from the SQ.

## Domain A Convention (Reference)

From `pointcloud_helper.rs:361-544`:
- All voxels start as `f32::MAX` (unvisited)
- BFS fills `[0, truncation_cells]` unsigned distances from surface
- Sign logic flips to negative for "inside" → range is `[-truncation_cells, +truncation_cells]`
- `f32::MAX` = unvisited/beyond band (no `-f32::MAX` exists)
- Surface voxels are `0.0`

**Domain B/C must follow this convention**: voxels beyond the truncation band get `f32::MAX`, regardless of inside/outside. Signed distances stay in `[-truncation_cells, +truncation_cells]`.

## Implementation Plan

### Part 1: Add f32::MAX beyond truncation

- [ ] Task 1.1. In the `f32::MAX` fallback path (lines 574-589): compute SQ distance. If `abs(sq_tdf) > truncation_cells`, assign `f32::MAX` instead of the clamped distance. Otherwise assign the clamped distance with correct sign.
  - Rationale: Aligns with Domain A. Voxels far from the SQ are "unvisited".

- [ ] Task 1.2. In the disagree-path (lines 612-632): after computing the final blended distance, if `abs(blended) >= truncation_cells`, assign `f32::MAX`.
  - Rationale: Same convention. Blended voxels at the band edge are unvisited.

- [ ] Task 1.3. In the agree-path (lines 633-641): same check after blending.
  - Rationale: Consistency.

### Part 2: Bounding box early-out (replaces |F| approach)

- [ ] Task 2.1. Add a method `is_within_band(&self, world_pt, band_m) -> bool` to `SuperquadricParams` that:
  1. Transforms `world_pt` to local frame (one matrix multiply)
  2. Checks if `abs(lx) < a + band_m && abs(ly) < b + band_m && abs(lz) < c + band_m`
  3. Returns `false` if the point is clearly outside the SQ bounding box + band
  - Cost: ~10 multiplications + 3 comparisons (negligible)
  - Rationale: This is a geometric check that's correct for all templates and doesn't depend on `F` values. Voxels outside the bounding box + band are guaranteed to have distance > truncation.

- [ ] Task 2.2. In the SQ sign-correction pass, call `is_within_band` before `taubin_distance`. If it returns `false`, assign `f32::MAX` and skip the expensive evaluation.
  - Rationale: For a typical scene, ~30-50% of grid voxels are outside the SQ bounding box + band. This saves 4 transcendental evaluations per skipped voxel.

## Verification Criteria

- Voxels beyond the truncation band get `f32::MAX` (not clamped distances)
- No `-f32::MAX` values in the TSDF (inside voxels use negative distances)
- The bounding box early-out correctly identifies far voxels
- All 51 existing tests pass
- New test: voxel far outside SQ bounding box gets `f32::MAX`

## Potential Risks and Mitigations

1. **Bounding box is too conservative for elongated objects**
   - If `a >> b` (e.g., a flat ellipsoid), the bounding box check is loose along the `a` axis but tight along `b` and `c`. Voxels just outside the `b`/`c` extent are correctly filtered.
   - Mitigation: The bounding box is an axis-aligned check in local frame. It's always correct (never falsely skips a voxel that needs evaluation) because the SQ is always contained within its bounding box.

2. **Performance gain depends on object-to-grid ratio**
   - If the SQ fills most of the grid, few voxels are skipped.
   - Mitigation: The check is essentially free (~10 ops vs ~100 ops for Taubin). Even skipping 10% of voxels is worthwhile.

3. **Sign convention edge case at exactly truncation_cells distance**
   - A voxel at exactly `truncation_cells` distance: should it be `f32::MAX` or the clamped distance?
   - Mitigation: Use `>= truncation_cells` for the `f32::MAX` assignment. This matches the BFS behavior (BFS stops expanding at `cdist >= truncation_cells`).
