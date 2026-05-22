# Fix TSDF Sign Correction: Holes, Floating Markers, and Boundary Artifacts

## Objective

Fix three visual artifacts in the TSDF after SQ sign correction:
1. Holes near the zero-crossing (SQ overrides correct camera "inside" signs near the visible surface)
2. Negative signs slightly outside the object (SQ surface slightly larger than camera surface)
3. Floating distance markers and far-backside discontinuities (agree/disagree blend boundary mismatch)

## Implementation Plan

- [ ] Task 1. Add `SQ_MIN_SIGN_OVERRIDE_CELLS = 2` constant to `src/config.rs`
  - Rationale: Voxels within 2 cells of the visible surface keep their camera sign. The camera data is reliable here — the ray-based sign logic correctly identifies inside/outside for voxels near the surface on both front and back sides. Only voxels further away (where camera data degrades) should use the SQ sign.

- [ ] Task 2. Refactor the SQ sign-correction pass in `src/pointcloud_helper.rs:560-644` with three changes:
  
  **Change A: One-directional sign override (positive→negative only)**
  - Only override the camera sign when `sq_inside == true && cam_inside == false` (camera says outside, SQ says inside — the backside fill case)
  - When `sq_inside == false && cam_inside == true` (camera says inside, SQ says outside), keep the camera sign. This preserves the interior near the front surface and prevents holes.
  - Rationale: The SQ's purpose is to fill in the unobserved backside interior. It should never remove interior that the camera confidently sees.
  
  **Change B: Minimum distance threshold**
  - Skip sign override entirely when `abs_cam <= SQ_MIN_SIGN_OVERRIDE_CELLS`
  - For these voxels, the camera sign and distance are authoritative
  - Rationale: Even when the SQ says "inside" and the camera says "outside", if the voxel is only 1 cell from the visible surface, the camera data is more reliable than the SQ. The SQ surface is a coarse approximation that may be slightly misaligned.
  
  **Change C: Fix blend boundary for agree path**
  - Change `abs_cam < blend_end_agree` to `abs_cam <= blend_end_agree` so the last cell (distance = truncation_cells) is also blended
  - Rationale: Currently, voxels at exactly `truncation_cells` distance in the agree path keep raw camera data while their disagree-path neighbors get SQ data, creating a discontinuity at the grid edge.

- [ ] Task 3. Update the disagree blend zone boundaries
  - Set `blend_start_disagree = SQ_MIN_SIGN_OVERRIDE_CELLS as f32` (e.g., 2.0) instead of 1.0
  - Set `blend_end_disagree = (truncation_cells - 1) as f32` (unchanged at 7.0)
  - Rationale: The blend starts at the minimum override threshold, not at cell 1. This ensures a smooth transition from camera authority (cells 0-2) through the blend zone (cells 2-7) to full SQ authority (cells 7+).

- [ ] Task 4. Update existing integration tests for the new behavior
  - `tsdf_backside_outside_positive`: Verify it still passes (backside voxels outside the SQ should be positive)
  - `tsdf_backside_monotonic`: Verify it still passes (no sign flips on backside)
  - Add new test `tsdf_no_sign_flip_near_surface`: Create a scenario where the SQ is slightly smaller than the point cloud, verify that voxels at distance 1-2 inside the camera surface keep their negative sign (no holes)
  - Add new test `tsdf_no_negative_outside_near_surface`: Create a scenario where the SQ is slightly larger, verify that voxels at distance 1-2 outside the camera surface keep their positive sign

- [ ] Task 5. Build and run all tests to verify

## Verification Criteria

- All existing tests pass (currently 51)
- New tests for near-surface sign preservation pass
- Visual inspection: no holes near the zero-crossing when viewing the TSDF mesh
- Visual inspection: no negative signs slightly outside the object
- Visual inspection: no floating distance markers at the blend boundary

## Potential Risks and Mitigations

1. **Backside voxels at distance 1-2 may have wrong camera signs**
   - The ray-based sign logic should correctly identify these as "inside" (the front surface occludes the camera's view). If this assumption is wrong for some camera configurations, we may need to lower the threshold to 1.
   - Mitigation: The threshold is a config constant, easy to tune.

2. **SQ fit quality affects the blend zone**
   - If the SQ fit is poor (high MSE), the SQ surface may be far from the camera surface, and the 2-cell threshold may not be enough to prevent artifacts.
   - Mitigation: The existing `SQ_FIT_ERROR_THRESHOLD = 0.15` already rejects poor fits. If the fit passes this threshold, the SQ surface should be within 1-2 cells of the camera surface.

3. **The one-directional override may miss cases where the SQ correctly identifies exterior**
   - If the camera incorrectly marks a voxel as "inside" (e.g., due to noise), the SQ can't correct it to "outside" with the one-directional override.
   - Mitigation: This is a rare case, and the camera data near the surface is generally reliable. The cost of false "inside" (slightly thicker object) is less than the cost of false "outside" (holes).

## Alternative Approaches

1. **Use camera vote counts for confidence**: Store `inside_votes` and `outside_votes` alongside the distance, and only override when the camera confidence is low (e.g., `inside_votes == 0`). This is more precise but requires changing the data flow between the sign logic and the SQ correction.
   
2. **SQ confidence based on |F(x,y,z)|**: Only override when `|F(x,y,z)| > threshold` (the SQ is confident about inside/outside). Near the SQ surface where F ≈ 0, the SQ sign is uncertain. This adds another threshold but provides sub-cell precision.
