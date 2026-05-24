# Fix TSDF Visualization and Sign Artifacts

## Objective

Fix three issues:
1. **Missing cells near the zero-crossing** — holes in the TSDF surface visualization
2. **Negative signs on the far backside** — SQ overriding camera sign where it shouldn't
3. **PyVista `pickpoint` warning** — harmless but annoying

## Root Cause Analysis

### Bug 1 (CRITICAL): `TRUNCATION_CELLS` mismatch between Rust and Python

**Rust** (`src/config.rs:7`): `TRUNCATION_CELLS = 8`
**Python** (`scripts/visualize_grasp_debug.py:54`): `TRUNCATION_CELLS = 4`

This mismatch causes:
- **Surface mode**: `clim=[-4, 4]` clips the color range at ±4, but data goes to ±8. Voxels at distances 5-8 are shown at max saturation, making it impossible to distinguish near-surface from far.
- **Points mode**: `value=[-4.5, 4.5]` filters OUT voxels with distance > 4.5. This is the direct cause of the "holes" — voxels on the backside with distance 5-8 are simply not rendered.
- **Color mapping**: Distorted — a voxel at distance 2 appears at 50% saturation instead of 25%.

**Fix**: Change `TRUNCATION_CELLS = 4` to `TRUNCATION_CELLS = 8` in the Python visualizer.

### Bug 2 (SIGNIFICANT): Surface mode threshold too narrow

The surface mode uses `value=[-1.5, 1.5]` to filter voxels near the zero-crossing. With the SQ sign corrections, some zero-crossing voxels may have distances slightly beyond ±1.5 (e.g., the SQ distance at the surface might be ±2 due to the Taubin approximation). These get filtered out, creating holes.

**Fix**: Widen the surface threshold to `[-2.5, 2.5]` to accommodate the SQ distance uncertainty.

### Bug 3 (MODERATE): Agree-path boundary excludes last cell

At `pointcloud_helper.rs:635`, the agree-path blend uses `abs_cam < blend_end_agree` (strict less-than). Voxels at exactly `truncation_cells` distance (the grid boundary) in the agree path keep their raw camera data. But their disagree-path neighbors at the same distance get full SQ data. This creates a discontinuity at the grid edge — neighboring voxels can have completely different distances, producing "floating markers".

**Fix**: Change `<` to `<=` so the last cell is also blended.

### Bug 4 (MODERATE): No minimum distance guard for SQ sign override

The disagree-path at `pointcloud_helper.rs:616` starts at `blend_start_disagree = 1.0` cells. This means a voxel at distance 1 from the visible surface can have its sign flipped. If the SQ surface is slightly misaligned with the camera surface (which is expected — the SQ is a coarse approximation), this creates:
- Voxels at distance 1 inside the camera surface but outside the SQ → sign flipped to positive → hole
- Voxels at distance 1 outside the camera surface but inside the SQ → sign flipped to negative → negative poking out

The camera sign is reliable within 2-3 cells of the visible surface. The SQ sign should only override at greater distances.

**Fix**: Add a `SQ_MIN_SIGN_OVERRIDE_CELLS = 2` constant. Skip SQ sign override for voxels within this distance of the surface. Only override sign (positive→negative) when `abs_cam > SQ_MIN_SIGN_OVERRIDE_CELLS`.

### Issue 5 (LOW): `pickpoint` PyVista warning

`pyvista.core.errors.PyVistaAttributeError: Attribute 'pickpoint' does not exist` — this is a known PyVista/VTK version incompatibility. The visualizer doesn't use pickpoint; it's triggered internally by VTK's interactor. It's harmless.

**Fix**: Suppress the warning at startup with a `warnings.filterwarnings` call.

## Implementation Plan

- [ ] Task 1. Fix `TRUNCATION_CELLS` in `scripts/visualize_grasp_debug.py:54`
  - Change `TRUNCATION_CELLS = 4` to `TRUNCATION_CELLS = 8`
  - Rationale: Must match `src/config.rs:7`. This is the primary cause of missing cells.

- [ ] Task 2. Widen surface mode threshold in `scripts/visualize_grasp_debug.py:705`
  - Change `value=[-1.5, 1.5]` to `value=[-2.5, 2.5]`
  - Rationale: The SQ Taubin distance approximation has ~5-15% error, which can push zero-crossing voxels beyond ±1.5 cells. A wider band captures them.

- [ ] Task 3. Fix agree-path boundary in `src/pointcloud_helper.rs:635`
  - Change `abs_cam < blend_end_agree` to `abs_cam <= blend_end_agree`
  - Rationale: Include the last cell in the blend to avoid discontinuity at the grid edge.

- [ ] Task 4. Add `SQ_MIN_SIGN_OVERRIDE_CELLS = 2` to `src/config.rs`
  - New constant after the existing SQ constants
  - Rationale: Voxels within 2 cells of the visible surface have reliable camera signs. The SQ should not override them.

- [ ] Task 5. Update disagree-path logic in `src/pointcloud_helper.rs:563-632`
  - Set `blend_start_disagree = SQ_MIN_SIGN_OVERRIDE_CELLS as f32` (2.0) instead of 1.0
  - For voxels with `abs_cam <= SQ_MIN_SIGN_OVERRIDE_CELLS`, keep camera sign unchanged (don't enter disagree path)
  - Rationale: Prevents SQ from flipping signs near the visible surface where camera data is authoritative.

- [ ] Task 6. Suppress `pickpoint` warning in `scripts/visualize_grasp_debug.py`
  - Add `import warnings` and `warnings.filterwarnings("ignore", message=".*pickpoint.*")` near the top
  - Rationale: Harmless VTK/PyVista version incompatibility; suppress to avoid confusion.

- [ ] Task 7. Build and run all tests to verify
  - Ensure all 51 existing tests still pass
  - Verify the visualizer loads without the pickpoint error

## Verification Criteria

- `TRUNCATION_CELLS` matches between Rust (8) and Python (8)
- Surface mode shows a continuous zero-crossing band without holes
- Points mode shows all voxels up to distance ±8
- No floating distance markers at the grid boundary
- No negative signs poking out within 2 cells of the visible surface
- `pickpoint` warning is suppressed
- All 51 tests pass

## Potential Risks and Mitigations

1. **Wider surface threshold may show too many voxels**
   - Going from ±1.5 to ±2.5 adds ~67% more voxels to the surface view. This may make the surface appear thicker.
   - Mitigation: The surface is rendered with opacity 0.7, so extra voxels just add context. If too thick, can adjust downward.

2. **SQ_MIN_SIGN_OVERRIDE_CELLS = 2 may be too conservative**
   - If the camera sign is wrong at distance 2 (e.g., noisy data), the SQ can't correct it.
   - Mitigation: The constant is configurable. Can be lowered to 1 if needed.

3. **The agree-path fix may change distances at the grid edge**
   - Voxels at exactly `truncation_cells` distance will now be blended with SQ data instead of keeping raw camera data.
   - Mitigation: These voxels are at the very edge of the grid and are far from the surface. The blend produces smoother, more consistent values.
