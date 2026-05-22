# Fix TSDF Visualization: Points Mode, Signs Mode, and Readability

## Objective

Fix the "full cube" problem in points mode, add a "signs" mode for inside/outside debugging, and improve overall TSDF visualization readability. This is a **Python-only** change to `scripts/visualize_grasp_debug.py` — no Rust changes needed.

## Root Cause Analysis

1. **Points mode shows a full cube** because the SQ backside fill now assigns valid distances (±1 to ±8) to ALL voxels in the grid. The threshold `[-8.5, 8.5]` passes everything. Before the SQ fill, boundary voxels had `f32::MAX` and were filtered out.

2. **No way to see sign correctness** — the user needs to see which voxels are inside vs outside to debug the SQ sign corrections. The current coolwarm colormap doesn't make this distinction obvious.

## Implementation Plan

### Part 1: Constants and Modes

- [ ] Task 1.1. Add `TSDF_SURFACE_BAND = 2.5` constant after `TRUNCATION_CELLS`
  - Location: after line 58 (`TRUNCATION_CELLS = 8`)
  - This separates the surface display width from the truncation distance
  - Rationale: The surface band is a visualization parameter, not a pipeline parameter

- [ ] Task 1.2. Update `TSDF_MODES` to include "signs"
  - Change line 523: `TSDF_MODES = ["surface", "points", "off"]` → `TSDF_MODES = ["surface", "points", "signs", "off"]`
  - Rationale: New mode for inside/outside visualization

### Part 2: Fix Surface Mode

- [ ] Task 2.1. Use `TSDF_SURFACE_BAND` constant in surface mode threshold
  - Location: line 709, change `value=[-2.5, 2.5]` → `value=[-TSDF_SURFACE_BAND, TSDF_SURFACE_BAND]`
  - Rationale: Single source of truth for the surface band width

### Part 3: Fix Points Mode

- [ ] Task 3.1. Clamp SQ-filled boundary voxels to NaN before threshold
  - After line 676 (`tsdf_vis[tsdf_vis > 1e10] = np.nan`), add: mask voxels with `abs(distance) >= TRUNCATION_CELLS` to NaN **only in points mode**
  - This filters out the SQ-filled boundary voxels that are at the grid edge, restoring the original truncation band appearance
  - Rationale: The SQ fills voxels up to the grid edge. These edge voxels are the least accurate. Showing only the BFS-filled band (±8 cells from surface) gives a clearer picture.

- [ ] Task 3.2. Improve points mode readability
  - Change line 755: `point_size=6` → `point_size=4`
  - Change line 757: `opacity=0.8` → `opacity=0.5`
  - Rationale: Smaller, semi-transparent points let you see through the outer shell to the interior structure

### Part 4: Add Signs Mode

- [ ] Task 4.1. Add signs mode rendering in `add_tsdf_actors()`
  - Location: after the `elif mode == "points":` block (after line 770)
  - Signs mode shows:
    - Negative (inside) voxels as red points
    - Positive (outside) voxels as blue points
    - Near-zero (|d| < 0.5, surface) voxels as green points with higher opacity
  - Filter to truncation band only (same as points mode)
  - Use `cell_centers()` for point rendering, same as points mode
  - Create a custom "signed" array: -1 for inside, +1 for outside, 0 for surface
  - Use a custom colormap: red → green → blue
  - Rationale: The most informative view for debugging SQ sign corrections is seeing the inside/outside classification directly

### Part 5: Update Documentation and Help

- [ ] Task 5.1. Update docstring at line 14
  - Change: `t  - cycle TSDF mode: surface / points / off` → `t  - cycle TSDF mode: surface / points / signs / off`

- [ ] Task 5.2. Update help text in `on_key_help()` at line 1664
  - Change: `print("  t  - cycle TSDF mode: surface / points / off")` → include "signs"

- [ ] Task 5.3. Update info text key hints at line 1451
  - Already says `t=TSDF` which is fine, but the mode label will show "signs" naturally via `tsdf_str`

- [ ] Task 5.4. Update argparse epilog at line 1792
  - Change: `t  cycle TSDF: surface/points/off` → `t  cycle TSDF: surface/points/signs/off`

- [ ] Task 5.5. Update legend to include signs mode
  - Add ("Inside (signs)", "red"), ("Outside (signs)", "dodgerblue"), ("Surface (signs)", "lime") to legend
  - Or simplify: just add ("SQ signs: in/out", "red/blue") as a single entry

## Verification Criteria

- [ ] Points mode shows a truncation band around the object (NOT a full cube)
- [ ] Signs mode clearly shows inside (red) vs outside (blue) vs surface (green) classification
- [ ] Surface mode is unchanged (thin band near zero-crossing)
- [ ] TSDF modes cycle: surface → points → signs → off
- [ ] All modes documented in help text, docstring, and argparse epilog
- [ ] Python syntax is valid (verified with `ast.parse`)
- [ ] Backward compatible with NPZ dumps that lack SQ data

## Potential Risks and Mitigations

1. **Points mode NaN masking may hide useful SQ data**
   - The mask at `abs(distance) >= TRUNCATION_CELLS` hides all SQ-filled boundary voxels.
   - Mitigation: The signs mode provides an alternative view that shows ALL voxels with their sign classification, including the SQ-filled ones.

2. **Signs mode custom colormap complexity**
   - Creating a custom 3-color colormap for sign classification adds code.
   - Mitigation: Use a simple approach — create a scalar array with values -1/0/+1 and use the built-in `coolwarm` or a simple RGB mapping.

3. **Signs mode performance with many voxels**
   - Rendering all observed voxels as colored points could be slow for large grids.
   - Mitigation: Apply the same truncation band filter as points mode. Only render voxels within the band.

4. **TSDF_SURFACE_BAND constant adds maintenance**
   - Another constant to keep in sync.
   - Mitigation: It's only used in the Python visualizer, not in the Rust pipeline. It's purely a visualization parameter with a clear comment.

## Alternative Approaches

1. **Cross-section / slice mode instead of signs mode**: Show a 2D plane through the TSDF. More complex to implement (needs a slider for slice position) but very informative. Trade-off: more code, more interactive controls, but clearer spatial understanding.
   - Decision: Defer to future work. Signs mode is simpler and addresses the immediate need.

2. **Opacity-based sign visualization**: Instead of separate colors, use opacity (opaque = inside, transparent = outside). Simpler but less visually distinct.
   - Decision: Color-based is clearer for debugging.
