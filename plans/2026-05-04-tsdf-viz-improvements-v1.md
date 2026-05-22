# Fix TSDF Points Mode and Add Sign Visualization

## Objective

Fix the "full cube" problem in points mode and add better visualization tools for understanding the TSDF structure, particularly the SQ backside fill.

## Root Cause

The points mode threshold `[-TRUNCATION_CELLS - 0.5, TRUNCATION_CELLS + 0.5]` with `TRUNCATION_CELLS = 8` includes ALL voxels in the grid (since BFS + SQ fills everything to ±8). This renders the entire grid as a solid cube of points.

Before the SQ backside fill was added, voxels beyond the truncation band had `f32::MAX` and were filtered out. Now the SQ fills those voxels with valid distances, so nothing is filtered.

## Implementation Plan

- [ ] Task 1. Add a `TSDF_SURFACE_BAND` constant separate from `TRUNCATION_CELLS`
  - `TSDF_SURFACE_BAND = 2.5` — the width of the zero-crossing band for surface mode
  - Rationale: The surface band should be independent of the truncation distance. It controls how thick the rendered surface shell is.

- [ ] Task 2. Fix points mode to show the actual truncation band, not the full grid
  - Change the points threshold to `[-TRUNCATION_CELLS, TRUNCATION_CELLS]` (without the ±0.5 fudge)
  - Add NaN replacement for voxels with `abs(distance) >= TRUNCATION_CELLS` BEFORE the threshold, so voxels at the grid edge (distance = truncation_cells exactly) that were filled by the SQ but are at the boundary of meaningful data are excluded
  - Rationale: Voxels at distance = truncation_cells are at the edge of the BFS reach. Their distance values are the least accurate. The SQ fill beyond this is for the Marching Cubes algorithm, not for visualization.

- [ ] Task 3. Add a fourth TSDF mode: "signs" — inside/outside visualization
  - Cycle order: surface → points → signs → off
  - `TSDF_MODES = ["surface", "points", "signs", "off"]`
  - Signs mode shows:
    - Negative (inside) voxels as red points with opacity 0.3
    - Positive (outside) voxels as blue points with opacity 0.3
    - Near-zero (surface) voxels as white/green points with opacity 0.8
  - This makes it easy to see where the SQ is filling the backside and whether signs are correct
  - Rationale: The most informative view for debugging the SQ sign corrections is seeing the inside/outside classification directly.

- [ ] Task 4. Improve points mode readability
  - Reduce default point size from 6 to 4
  - Reduce opacity from 0.8 to 0.5
  - Rationale: With all voxels visible, smaller semi-transparent points let you see through the outer shell to the interior structure.

- [ ] Task 5. Update key bindings and help text
  - Update docstring, help text, info text, argparse epilog to include "signs" mode
  - Rationale: Users need to know about the new mode.

- [ ] Task 6. Revert surface threshold to a constant
  - Use `TSDF_SURFACE_BAND` instead of the hardcoded 2.5
  - Rationale: One source of truth for the surface band width.

## Verification Criteria

- Points mode shows the truncation band around the object (not a full cube)
- Signs mode clearly shows inside (red) vs outside (blue) classification
- Surface mode is unchanged (thin band near zero-crossing)
- All modes are documented in help text
- TSDF modes cycle: surface → points → signs → off

## Potential Risks and Mitigations

1. **Points mode may still show too many voxels**
   - Even with the truncation band filter, the band is 16 cells wide (±8), which is a lot of points.
   - Mitigation: The reduced opacity and point size help. The signs mode provides a clearer alternative.

2. **Signs mode performance**
   - Rendering all observed voxels as colored points could be slow for large grids.
   - Mitigation: Use the same threshold as points mode (truncation band). Only render voxels within the band.

3. **The TSDF_SURFACE_BAND constant adds complexity**
   - Another constant to keep in sync.
   - Mitigation: It's only used in the Python visualizer, not in the Rust pipeline. It's purely a visualization parameter.
