# Fix TSDF Domain Logic for Backside Estimation

## Objective

The backside fill is broken because Domain B never triggers, Domain C is too narrow, and the blend logic doesn't properly handle wrong camera signs on the backside. This causes negative signs outside the object, floating distance markers, and sign reversals at edges.

## Root Cause Analysis

### Bug 1 (CRITICAL): Domain B never triggers — all voxels are within BFS reach

**Location**: `src/pointcloud_helper.rs:588`

The grid is allocated as `width = max_gx + 1 + 2 * trunc` (line 353), extending exactly `trunc` cells beyond the point cloud in each direction. The BFS fills all voxels up to `truncation_cells` distance. Since the grid extends exactly `trunc` cells from the surface, **every voxel in the grid has a BFS distance ≤ `truncation_cells`**.

Therefore `cam_tdf == f32::MAX` is **never true** for any voxel. Domain B (pure SQ fill) never executes. The SQ data only enters through Domain C (the 2-cell blend band at cells 6-8 with `TRUNCATION_CELLS=8, SQ_BLEND_DELTA_CELLS=2`).

**Impact**: The entire backside of the object has only camera-authority data (Domain A), with wrong signs from the ray-based logic. The SQ fill does almost nothing.

### Bug 2 (SIGNIFICANT): Domain C blend uses wrong-sign camera data

**Location**: `src/pointcloud_helper.rs:593-604`

The blend zone (cells 6-8) mixes camera TDF with SQ TDF. But on the backside, the camera TDF has the **wrong sign** — the ray-based logic marks backside voxels as "outside" (positive) because the camera ray doesn't hit any surface before reaching them. Blending `+6.5` (wrong camera) with `-3.0` (correct SQ) gives an intermediate value that may still be positive, creating the "band of wrong signs" the user sees.

### Bug 3 (MODERATE): Blend zone is too narrow (only 2 cells)

**Location**: `src/config.rs:62`

With `SQ_BLEND_DELTA_CELLS = 2` and `TRUNCATION_CELLS = 8`, the blend zone is only cells 6-8. This is too narrow to smoothly transition from camera to SQ authority, especially given that camera signs are wrong on the backside.

### Bug 4 (MODERATE): SQ sign override should apply more broadly

**Location**: `src/pointcloud_helper.rs:535-543`

The SQ tiebreaker only applies when cameras **disagree** (both inside and outside votes). But on the backside with a single camera, there's no disagreement — the single camera votes "outside" (no occlusion), and the SQ says "inside". The tiebreaker never fires because `outside_votes > 0 && inside_votes == 0`.

## Implementation Plan

- [ ] **Task 1.** Redesign the domain logic in `get_tsdf()` at `src/pointcloud_helper.rs:547-608`

  Replace the current Domain A/B/C logic with a unified approach:

  **New approach: SQ sign override + distance blend for ALL non-surface voxels**

  Instead of three separate domains, use a single pass that:
  1. For every voxel where `cam_tdf != f32::MAX` (i.e., all voxels in the grid):
     - Compute the SQ Taubin distance (in cell units)
     - Determine the SQ sign (inside/outside)
  2. If the camera sign disagrees with the SQ sign AND the voxel is far from the visible surface:
     - Use the SQ distance and sign entirely (backside authority)
  3. If the voxel is near the visible surface (camera has good data):
     - Keep the camera distance and sign (front-side authority)
  4. In the transition zone:
     - Blend camera and SQ distances, but use the SQ sign as the ground truth

  The key insight: **the SQ sign should be authoritative for all voxels where the camera data is unreliable** (backside, deep interior). The camera distance is authoritative near the visible surface.

  Concrete logic:
  ```rust
  if let Some(sq) = sq_params {
      distance.par_iter_mut().enumerate().for_each(|(flat_idx, dist)| {
          let cam_tdf = *dist;
          if cam_tdf == 0.0 { return; } // Surface voxel, don't touch
          
          // Compute world position
          let (gx, gy, gz) = decode_flat(flat_idx, stride_y, stride_z);
          let vw = origin + Vector3::new(gx, gy, gz) * resolution_m;
          
          // SQ distance and sign
          let sq_dist_m = sq.taubin_distance(vw);
          let sq_tdf = sq_dist_m / resolution_m;
          let sq_tdf_clamped = sq_tdf.clamp(-(truncation_cells as f32), truncation_cells as f32);
          let sq_sign = sq.evaluate(vw) < 0.0; // true = inside
          
          let abs_cam = cam_tdf.abs();
          let cam_is_inside = cam_tdf < 0.0;
          
          if cam_tdf == f32::MAX {
              // Shouldn't happen given grid sizing, but handle it
              *dist = sq_tdf_clamped;
              return;
          }
          
          // Determine if camera data is reliable at this voxel.
          // Camera data is unreliable when:
          // 1. The voxel is far from the visible surface (large abs_cam)
          // 2. The camera sign disagrees with the SQ sign
          let signs_agree = cam_is_inside == sq_sign;
          
          if !signs_agree {
              // Camera and SQ disagree on sign.
              // Trust SQ for sign, blend distances based on proximity to surface.
              // Near the visible surface (small abs_cam): lean toward camera distance
              // Far from surface (large abs_cam): lean toward SQ distance
              let blend_start = 1.0; // Start transitioning at 1 cell from surface
              let blend_end = (truncation_cells - 1) as f32; // Full SQ at trunc-1 cells
              
              if abs_cam <= blend_start {
                  // Very close to visible surface — camera distance is good,
                  // but flip the sign to match SQ
                  *dist = if sq_sign { -abs_cam } else { abs_cam };
              } else if abs_cam >= blend_end {
                  // Far from surface — SQ is authoritative
                  *dist = sq_tdf_clamped;
              } else {
                  // Transition zone — blend distances, SQ sign
                  let t = (abs_cam - blend_start) / (blend_end - blend_start);
                  let w = 1.0 - t * t * (3.0 - 2.0 * t); // smoothstep: 1→0
                  let blended_abs = w * abs_cam + (1.0 - w) * sq_tdf_clamped.abs();
                  *dist = if sq_sign { -blended_abs } else { blended_abs };
              }
          } else {
              // Signs agree — blend distances in the outer band only
              let blend_start = (truncation_cells - config::SQ_BLEND_DELTA_CELLS) as f32;
              let blend_end = truncation_cells as f32;
              
              if abs_cam >= blend_start && abs_cam < blend_end {
                  let t = (abs_cam - blend_start) / (blend_end - blend_start);
                  let w = 1.0 - t * t * (3.0 - 2.0 * t);
                  *dist = w * cam_tdf + (1.0 - w) * sq_tdf_clamped;
              }
          }
      });
  }
  ```

- [ ] **Task 2.** Remove the old SQ tiebreaker from the ray-based sign section at `src/pointcloud_helper.rs:535-543`

  The old tiebreaker (`if inside_votes > 0 && outside_votes > 0`) is now redundant because the new unified domain logic handles sign disagreements. Remove the `if let Some(sq) = sq_params { ... }` block inside the ray-based sign loop to avoid double-correction.

  Actually, keep the tiebreaker for the case where cameras disagree — it's still useful for voxels near the surface where the camera data is ambiguous. But make sure it doesn't conflict with the new domain logic.

  **Better approach**: Keep the tiebreaker as-is. The new domain logic runs AFTER the sign step, so it can override the tiebreaker's result if needed. The tiebreaker provides a first-pass correction, and the domain logic provides the final correction.

- [ ] **Task 3.** Increase `SQ_BLEND_DELTA_CELLS` from 2 to 3 in `src/config.rs:62`

  Wider blend zone for smoother transitions. This is now less critical with the new logic but still helps.

- [ ] **Task 4.** Add integration test: backside voxel has correct positive sign

  Add a test that verifies a point just outside the backside of a hemisphere has a **positive** TSDF distance (outside the object). This would have failed before the fix.

- [ ] **Task 5.** Add integration test: no floating distance markers

  Add a test that verifies TSDF distances are monotonically increasing away from the surface on the backside (no isolated clumps of wrong-sign voxels).

- [ ] **Task 6.** Run `cargo test` to verify all tests pass

- [ ] **Task 7.** Run `cargo build` to verify clean compilation

## Verification Criteria

- [VC-1] Backside voxels outside the fitted superquadric have positive TSDF distances
- [VC-2] Backside voxels inside the fitted superquadric have negative TSDF distances
- [VC-3] Front-side voxels retain camera-authority distances (unchanged from before)
- [VC-4] No "band of wrong signs" at the blend boundary
- [VC-5] All existing tests pass

## Potential Risks and Mitigations

1. **SQ fitting is inaccurate — wrong signs propagate**
   Mitigation: The SQ sign is only used when it disagrees with the camera. If the SQ is a good fit (low fit_error), the signs will be correct. The `SQ_FIT_ERROR_THRESHOLD` already gates SQ usage.

2. **Performance: computing SQ distance for every voxel**
   Mitigation: The old code already computed SQ distance for every voxel in the parallel pass. The new logic doesn't add extra computations, just changes how the results are used.

3. **Blend zone changes may affect front-side quality**
   Mitigation: The new logic only changes behavior when signs disagree. Front-side voxels where camera and SQ agree are untouched.
