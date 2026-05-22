# Fix Hand Skeleton Import + TSDF Occlusion

## Objective

Fix two issues:
1. Hand skeleton import failure — pinocchio/model import path is wrong
2. TSDF only shows outer shell — opaque voxels occlude everything inside

## Implementation Plan

### Phase 1: Fix hand skeleton import path

- [ ] **Task 1.1**: Change the import block (lines 82-100) to use a direct sibling import instead of the `mia_hand_ros2_pkgs` package path. Add `_script_dir` to `sys.path` and import `from model import ...` directly, since `model.py` is in the same directory as `visualize_grasp_debug.py`.

### Phase 2: Fix TSDF occlusion — show interior structure

- [ ] **Task 2.1**: Change TSDF rendering from solid voxels to a multi-mode approach. Add a cycling TSDF display mode with 3 options:
  - **"surface"**: Show only voxels near the zero-crossing (|distance| < 1.0 cell) — this is the actual surface shell, rendered with higher opacity
  - **"shell"**: Show all observed voxels but with distance-based opacity — voxels far from surface are more transparent, near-surface voxels are more opaque. This lets you see the full truncation band structure without the outer shell hiding everything
  - **"off"**: No TSDF display

  This gives the user control to see both the raw data (shell mode) and the meaningful surface (surface mode).

- [ ] **Task 2.2**: Update the `on_key_t()` handler to cycle through the 3 TSDF modes instead of a simple toggle.

- [ ] **Task 2.3**: Update the info text and legend to show the current TSDF mode.

## Verification Criteria

- [ ] `python visualize_grasp_debug.py <dump.npz>` starts without the "Hand skeleton: unavailable" message — it should print "Hand skeleton: pinocchio + URDF loaded"
- [ ] Pressing `h` key shows a hand skeleton with colored joints and links
- [ ] Pressing `t` key cycles through TSDF modes: surface → shell → off → surface
- [ ] In "surface" mode, only near-zero voxels are shown — no occlusion issue
- [ ] In "shell" mode, the full truncation band is visible with distance-based transparency

## Potential Risks and Mitigations

1. **Risk: model.py import fails due to pinocchio not being installed**
   Mitigation: Keep the try/except with graceful fallback — if pinocchio isn't available, hand skeleton stays disabled but everything else works

2. **Risk: Distance-based opacity is too slow for large grids**
   Mitigation: Use `grid.threshold()` first to reduce voxel count, then apply opacity mapping. For grids up to ~250K voxels this should be fine

3. **Risk: Surface band too thin or empty for some dumps**
   Mitigation: Use |distance| < 1.5 cells for surface mode to ensure enough voxels are shown

## Alternative Approaches

1. **Clipping plane**: Could add a PyVista clipping plane widget, but this requires manual interaction and doesn't solve the fundamental occlusion issue
2. **Point cloud rendering**: Could render TSDF as points instead of voxels, but this loses the spatial structure that's useful for debugging
3. **Isosurface (original approach)**: Could go back to isosurface, but this hides the raw data and silently fails when data is wrong — the voxel approach is better for debugging
