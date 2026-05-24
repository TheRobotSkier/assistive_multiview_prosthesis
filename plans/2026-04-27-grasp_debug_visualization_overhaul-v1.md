# Grasp Debug Visualization Overhaul Plan

## Objective

Fix three critical visualization issues in `visualize_grasp_debug.py` and simplify the grasp mode cycling behavior:
1. TSDF isosurface not rendering (highest priority)
2. Hand skeleton/spline visualization is wrong — replace with URDF-based joint+link rendering
3. Grasp markers redesigned as camera-like dots with axis triangles
4. Simplify grasp modes from 4-cycle to 2-state toggle (best/all)

## Key Findings

### Issue 1: TSDF Not Showing — Data Layout Mismatch (CONFIRMED)

**Root cause**: The TSDF data is stored in Rust with **X-fastest** memory layout (`index = x + y*width + z*width*height`). When loaded in Python, `tsdf_flat.reshape(shape)` with `shape = (W, H, D)` correctly produces `tsdf[w, h, d]` in C-order. However, PyVista's `ImageData` (VTK's `vtkImageData`) expects cell data in **Fortran order** — Z-fastest, then Y, then X. The current code at `visualize_grasp_debug.py:387` uses `tsdf_vis.ravel(order="C")`, which produces X-fastest order. This mismatch causes the contour algorithm to see garbled spatial data, producing no valid isosurface.

**Evidence**: 
- Rust layout: `pointcloud_helper.rs:146-156` — stride is `stride_y = width`, `stride_z = width * height`, indexing is `data[x + y*stride_y + z*stride_z]` (X-fastest)
- Python reshape: `visualize_grasp_debug.py:102` — `tsdf_flat.reshape(shape)` where `shape = [W, H, D]`
- PyVista assignment: `visualize_grasp_debug.py:387` — `grid.cell_data["distance"] = tsdf_vis.ravel(order="C")` (X-fastest)
- PyVista expects: Fortran (Z-fastest) ordering for structured grid cell data

**Secondary issue — "not equal length on each side"**: The ROI `enforce_min_dims` at `pointcloud_helper.rs:49-61` only *increases* dimensions to the minimum. If the point cloud is asymmetric (e.g., elongated in one axis), the ROI won't be cubic. The config at `config.rs:18-19` sets `MIN_TSDF_DIM_M = 0.1` and `MAX_TSDF_DIM_M = 0.3`, which are the same on all axes, but `enforce_min_dims` doesn't force equality — it only enforces minimums per-axis. The `clip_max_dims` can also clip asymmetrically based on anchor position. This is a Rust-side concern but doesn't affect the visualization bug.

### Issue 2: Hand Skeleton Uses Spline Through Contact Points (CONFIRMED)

**Root cause**: The current `add_hand_skeleton()` at `visualize_grasp_debug.py:731-795` draws splines through LUT contact points. This is fundamentally wrong because:
- Contact points are surface positions on finger pads, not joint centers
- Splines through arbitrary contact indices produce meaningless curves
- The finger chain definitions in `FINGER_CHAINS` at lines 60-85 mix indices from different LUT tables in ways that don't correspond to kinematic chains

**Better approach**: The `hand_tip_visualizer.py` and `model.py` already demonstrate the correct method — use Pinocchio FK to compute joint positions from the URDF (`mia_hand_flat.urdf`), then draw lines between parent-child joint pairs. The `COLLISION_GEOMETRIES` dict in `model.py` provides geometry info, and `pin.forwardKinematics()` gives all joint transforms. This gives actual joint positions and link geometries.

### Issue 3: Grasp Markers Use Arrow+Axis (CONFIRMED)

**Current rendering** at `visualize_grasp_debug.py:620-715`: Each grasp is rendered as an approach direction arrow + sphere + (for best) a small XYZ coordinate frame. This is confusing because the arrows don't clearly convey orientation.

**Desired rendering**: Like the camera markers (sphere + direction cone at `visualize_grasp_debug.py:509-533`), but with 3 small triangles (one per axis) emanating from a dot. This gives an intuitive orientation indicator similar to camera frustum visualizations.

### Issue 4: Grasp Mode Cycling (CONFIRMED)

**Current behavior** at `visualize_grasp_debug.py:895-901`: The `g` key cycles through `["best", "top10", "all", "none"]` — 4 modes. The `top10` mode is awkward and the cycling is unintuitive.

**Desired behavior**: Two independent toggle states — `g` toggles between `best` and `all`, with no cycling. Remove `top10` and `none` modes.

---

## Implementation Plan

### Phase 1: Fix TSDF Rendering (Highest Priority)

- [ ] **1.1** Fix the data layout mismatch in `add_tsdf_actors()` at `visualize_grasp_debug.py:387`. Change `tsdf_vis.ravel(order="C")` to `tsdf_vis.ravel(order="F")` (Fortran order) so the flat array matches PyVista/VTK's expected Z-fastest cell data layout. This is the single-line fix that should make the isosurface appear.
- [ ] **1.2** Apply the same Fortran-order fix to the TSDF slice fallback at `visualize_grasp_debug.py:440`: change `slice_data.ravel(order="C")` to `slice_data.ravel(order="F")`.
- [ ] **1.3** Add diagnostic logging to `add_tsdf_actors()` that prints the number of non-NaN voxels, the value range of observed voxels, and whether the contour produced any points. This helps verify the fix works and aids future debugging. Place after line 381 (after NaN replacement).
- [ ] **1.4** Verify the isosurface contour level is appropriate. The TSDF stores distances in *cells* (integer steps from BFS at `pointcloud_helper.rs:370`), not meters. The zero-level contour should work for the actual surface, but the fallback levels (0.5, 1.0, etc.) are also in cells. This is correct as-is, but add a comment clarifying the unit.

### Phase 2: Redesign Grasp Markers as Camera-Like Dots with Axis Triangles

- [ ] **2.1** Create a helper function `make_grasp_marker(plotter, position, rotation_matrix, size, color, opacity)` that renders a grasp as:
  - A small sphere (dot) at the grasp position
  - Three flat triangles (cones or custom meshes), one per axis, colored red/green/blue, emanating from the dot. Each triangle lies in the plane perpendicular to its axis and points outward along that axis. Use `pv.Cone()` with small height and flat tip radius, oriented along each column of the rotation matrix.
- [ ] **2.2** Refactor `add_grasp_actors()` at `visualize_grasp_debug.py:620-715` to use the new `make_grasp_marker()` instead of the current arrow+sphere+axis approach. Remove the approach direction arrow logic entirely (lines 644-674). Keep the score label logic (lines 701-715).
- [ ] **2.3** For the best grasp, make the marker slightly larger and use gold color. For other grasps, use grasp-type color and scale by score. Maintain the existing score-based opacity scaling.
- [ ] **2.4** Remove the old coordinate frame arrows for the best grasp (lines 685-699) since the axis triangles now convey orientation.

### Phase 3: Replace Hand Spline with URDF-Based Joint/Link Skeleton

- [ ] **3.1** Add Pinocchio and model.py imports to `visualize_grasp_debug.py`. Import `model`, `data`, `get_q_full`, `pin` from the model module (following the same import pattern as `hand_tip_visualizer.py:7-36` with the try/except fallback).
- [ ] **3.2** Define a joint connectivity map for the MIA hand based on the URDF at `mia_hand_flat.urdf`. The kinematic tree is:
  - `base_link` → `mia_palm` (fixed)
  - `mia_palm` → `mia_thumb_opp` (revolute) → `mia_thumb_sensor` (fixed) → `mia_thumb_fle` (revolute)
  - `mia_palm` → `mia_index_fle` (revolute) → `mia_index_sensor` (fixed)
  - `mia_palm` → `mia_middle_fle` (revolute) → `mia_middle_sensor` (fixed)
  - `mia_palm` → `mia_ring_fle` (revolute)
  - `mia_palm` → `mia_little_fle` (revolute)
  Define this as a list of `(parent_joint_name, child_joint_name)` pairs for drawing lines.
- [ ] **3.3** Rewrite `add_hand_skeleton()` at `visualize_grasp_debug.py:731-795`. Instead of using LUT contact points:
  1. Compute `q_full` from the best grasp's closure amount and grasp type using the same mapping as `model.py:get_q_full()` (or `_q_full_with_thumb_mode()` for lateral vs cylindrical/pinch thumb modes).
  2. Run `pin.forwardKinematics(model, data, q_full)` to get all joint transforms via `data.oMi`.
  3. For each joint, get its world position from `data.oMi[joint_id].translation`.
  4. Draw lines (using `pv.Line()` or `pv.Spline()`) between parent-child joint pairs from the connectivity map.
  5. Draw small spheres at each joint position.
  6. Transform all positions by the grasp pose `T` (the 4x4 matrix from the best grasp).
- [ ] **3.4** Optionally draw collision geometry wireframes (cylinders, spheres, boxes) at their FK-computed world positions, similar to `hand_tip_visualizer.py:243-277`. This would use `COLLISION_GEOMETRIES` from model.py. This is a nice-to-have — the joint lines+spheres are the minimum viable improvement.
- [ ] **3.5** Remove the old `FINGER_CHAINS` constant (lines 60-85) and the old LUT-based skeleton logic since it's replaced by FK-based rendering. Also remove the `lut_data` loading logic (lines 722-729) if no longer needed (but keep the `--lut-path` arg for backward compat, just unused).
- [ ] **3.6** Remove the `load_finger_lut()` function (lines 137-167) and `_dq_to_se3()` (lines 170-203) and `_quat_to_matrix()` (lines 206-218) since they were only used for the old hand skeleton. Keep `_quat_rotate()` as it's used for the input hand pose arrows.

### Phase 4: Simplify Grasp Mode Toggle

- [ ] **4.1** Change `GRASP_MODES` at line 313 from `["best", "top10", "all", "none"]` to just `["best", "all"]`.
- [ ] **4.2** Update `on_key_g()` at lines 895-901 to toggle between "best" and "all" instead of cycling through 4 modes. Simple boolean flip: if current is "best", switch to "all"; if "all", switch to "best".
- [ ] **4.3** Remove the `get_grasp_indices()` logic for "top10" mode (lines 611-614) and "none" mode (line 589). Keep "best" and "all" branches.
- [ ] **4.4** Update the docstring keyboard shortcuts (line 15-16) and help text (lines 944-956) to reflect the new toggle behavior: `g - toggle grasp display: best / all`.
- [ ] **4.5** Update the `--show-all-grasps` CLI flag (line 1077-1079) behavior: when set, start in "all" mode; otherwise start in "best" mode (this already works with the current code at line 345).

## Verification Criteria

- [ ] **TSDF renders**: Running the viewer on a debug dump shows a cyan isosurface mesh in the 3D viewport. The terminal diagnostic output confirms non-zero contour points.
- [ ] **TSDF shape matches ROI**: The isosurface visually fits within or near the orange ROI bounding box and corresponds to where the point cloud points are.
- [ ] **Hand skeleton shows joints+links**: Pressing `h` displays a recognizable hand skeleton with lines between joints and spheres at joint positions, positioned at the best grasp location. The thumb, index, middle, ring, and little fingers are visually distinct chains.
- [ ] **Grasp markers show dot+triangles**: Each grasp is rendered as a small dot with 3 colored triangles (RGB for XYZ axes). No approach arrows or coordinate frame arrows remain.
- [ ] **Grasp mode toggles cleanly**: Pressing `g` switches between showing only the best grasp and showing all grasps. No "top10" or "none" modes exist. The info text updates correctly.
- [ ] **No regressions**: Point cloud, ROI box, camera markers, input pose arrows, and info text all still work correctly.

## Potential Risks and Mitigations

1. **Fortran-order fix may not be sufficient if PyVista version handles cell data differently**
   - Mitigation: Add diagnostic print statements showing the grid dimensions, data range, and contour result. If `order="F"` doesn't work, try transposing the array before ravel: `tsdf_vis.transpose(2, 1, 0).ravel(order="C")` which is equivalent. Also test with `pv.ImageData`'s `point_data` instead of `cell_data` as an alternative approach.

2. **Pinocchio may not be installed in the user's environment**
   - Mitigation: Wrap the Pinocchio import in a try/except. If unavailable, fall back to the old LUT-based skeleton (or skip hand skeleton entirely with a warning). The `hand_tip_visualizer.py` already demonstrates this pattern with its import fallback.

3. **URDF file may not be found at runtime**
   - Mitigation: The `model.py` already handles this with `URDF_PATH` computed relative to the script directory. Use the same path resolution. If the URDF is missing, fall back gracefully.

4. **Grasp closure → q_full mapping may not exactly match Rust's scoring logic**
   - Mitigation: The closure amount in the dump comes from the Rust scorer. The mapping from closure to joint angles should use the same formula. Verify by comparing the visualized hand position against the contact points in the TSDF. The `model.py:get_q_full()` function should match since the LUT was generated from it.

5. **Performance of FK computation for hand skeleton**
   - Mitigation: Pinocchio FK for a 7-DOF hand is extremely fast (<1ms). No performance concern.

## Alternative Approaches

1. **TSDF: Use `pv.ImageData` with point_data instead of cell_data**
   - Assign values to grid points (nodes) rather than cells. This avoids the cell-data ordering issue entirely but requires dimensions to match the data shape exactly (not shape+1). Simpler but slightly different contour behavior.

2. **TSDF: Use marching cubes directly via `skimage`**
   - Use `skimage.measure.marching_cubes()` on the raw numpy array, then create a `pv.PolyData` mesh from the vertices/faces. This bypasses PyVista's `ImageData` entirely and gives full control over the isosurface extraction. More dependencies but more reliable.

3. **Hand: Load and render the actual STL meshes from the URDF**
   - Instead of wireframe skeleton, load the STL mesh files referenced in the URDF and render them at FK-computed positions. This would look much more realistic but requires the mesh files to be available and is significantly more complex. Not recommended for a debug viewer.

4. **Grasp markers: Use PyVista's built-in axes widget**
   - Use `pv.Axes()` or `plotter.add_axes()` at each grasp location. Simpler but less customizable and may not scale well with many grasps.
