# Grasp Debug Visualization Overhaul Plan

## Objective

Fix three critical visualization issues in `visualize_grasp_debug.py` and simplify the grasp mode cycling behavior:
1. TSDF not rendering — replace broken isosurface with transparent voxel grid (better for debugging)
2. Hand skeleton/spline visualization is wrong — replace with URDF-based joint+link rendering
3. Grasp markers redesigned as camera-like dots with axis triangles
4. Simplify grasp modes from 4-cycle to 2-state toggle (best/all)

## Key Findings

### Issue 1: TSDF Not Showing — Data Layout Mismatch + Wrong Visualization Choice

**Root cause (layout bug)**: The TSDF data is stored in Rust with **X-fastest** memory layout (`index = x + y*width + z*width*height` per `pointcloud_helper.rs:146-156`). When loaded in Python, `tsdf_flat.reshape(shape)` with `shape = (W, H, D)` correctly produces `tsdf[w, h, d]` in C-order. However, PyVista's `ImageData` (VTK's `vtkImageData`) expects cell data in **Fortran order** — Z-fastest, then Y, then X. The current code at `visualize_grasp_debug.py:387` uses `tsdf_vis.ravel(order="C")`, which produces X-fastest order. This mismatch garbles the spatial data, producing no valid isosurface.

**Why voxel grid is better than isosurface for debugging**: 
- The typical grid is small: with 5mm resolution and 0.1–0.3m ROI, we get ~8K–250K total voxels, but only the observed shell (within truncation distance) has data — typically 5K–40K voxels after filtering `f32::MAX`. Very manageable for PyVista.
- Individual voxels are color-coded by signed distance (red=inside, blue=outside, white=surface), making the truncation band directly visible.
- You can visually verify sign flipping, truncation extent, and surface thickness — critical for debugging the TSDF construction pipeline.
- No ambiguity from isosurface extraction — you see the raw data.

**Evidence**: 
- Rust layout: `pointcloud_helper.rs:146-156` — `data[x + y*stride_y + z*stride_z]` (X-fastest)
- Python: `visualize_grasp_debug.py:102` — `tsdf_flat.reshape(shape)` where `shape = [W, H, D]`
- PyVista: `visualize_grasp_debug.py:387` — `grid.cell_data["distance"] = tsdf_vis.ravel(order="C")` (wrong order)
- Config: `config.rs:6-7` — resolution=5mm, truncation=4 cells → truncation band = 20mm thick

### Issue 2: Hand Skeleton Uses Spline Through Contact Points (CONFIRMED)

**Root cause**: `add_hand_skeleton()` at `visualize_grasp_debug.py:731-795` draws splines through LUT contact points, which are surface contact positions, not joint centers. The `FINGER_CHAINS` at lines 60-85 mix indices from different LUT tables in ways that don't represent kinematic chains.

**Better approach**: Use Pinocchio FK (same as `hand_tip_visualizer.py` + `model.py`) to compute joint positions from the URDF (`mia_hand_flat.urdf`), then draw lines between parent-child joints. The kinematic tree from the URDF is:
- `base_link` → `mia_palm` → `mia_thumb_opp` → `mia_thumb_sensor` → `mia_thumb_fle`
- `mia_palm` → `mia_index_fle` → `mia_index_sensor`
- `mia_palm` → `mia_middle_fle` → `mia_middle_sensor`
- `mia_palm` → `mia_ring_fle`
- `mia_palm` → `mia_little_fle`

### Issue 3: Grasp Markers Use Arrow+Axis (CONFIRMED)

**Current**: Approach direction arrow + sphere + coordinate frame arrows at `visualize_grasp_debug.py:644-699`. Confusing.

**Desired**: Like camera markers (sphere + direction cone at lines 509-533), but with 3 small triangles per axis for orientation.

### Issue 4: Grasp Mode Cycling (CONFIRMED)

**Current**: 4-mode cycle `["best", "top10", "all", "none"]` at `visualize_grasp_debug.py:895-901`.

**Desired**: Simple toggle between "best" and "all".

---

## Implementation Plan

### Phase 1: Replace TSDF Isosurface with Transparent Voxel Grid

- [ ] **1.1** Rewrite `add_tsdf_actors()` at `visualize_grasp_debug.py:375-464`. Replace the entire isosurface + slice logic with a voxel grid approach:
  1. Create the `pv.ImageData` with correct dimensions and spacing (keep `grid.dimensions = np.array(tsdf_vis.shape) + 1`, `grid.origin = origin`, `grid.spacing = [res, res, res]`).
  2. Fix the data layout: use `tsdf_vis.ravel(order="F")` (Fortran order) when assigning to `grid.cell_data["distance"]`. This is the same fix the isosurface needed — VTK expects Z-fastest cell data.
  3. Use `grid.threshold(value=[-TRUNCATION_CELLS, TRUNCATION_CELLS], scalars="distance")` to extract only observed voxels (filtering out `f32::MAX` / NaN values). The truncation constant is 4 cells from `config.rs:7`.
  4. Render the thresholded voxels with `style="wireframe"` or low opacity `style="surface"` using a diverging colormap (`coolwarm` or `RdBu`) centered at 0. This shows inside (red) vs outside (blue) vs surface (white).
  5. Add a scalar bar labeled "TSDF distance (cells)".

- [ ] **1.2** Add diagnostic print to `add_tsdf_actors()` showing: grid dimensions, number of observed voxels (non-f32-MAX), value range of observed voxels, and how many are negative/positive/near-zero. Place after the NaN replacement (line 381).

- [ ] **1.3** Update the `on_key_t()` toggle at `visualize_grasp_debug.py:886-893` to work with the new voxel grid actors (same toggle logic, just operates on the new actor group).

- [ ] **1.4** Update the legend entry at `visualize_grasp_debug.py:839` from `("TSDF surface", "cyan")` to `("TSDF voxels", "coolwarm")` or similar.

- [ ] **1.5** Remove the now-dead `tsdf_slice_mode` state variable (line 352) and the slice-specific rendering path, since the voxel grid replaces both isosurface and slice views.

### Phase 2: Redesign Grasp Markers as Camera-Like Dots with Axis Triangles

- [ ] **2.1** Create a helper function `make_grasp_marker(plotter, position, rotation_matrix, size, color, opacity)` that renders a grasp as:
  - A small sphere (dot) at the grasp position
  - Three flat triangles (using `pv.Cone()` with small height), one per axis, colored red/green/blue, emanating from the dot. Each triangle is oriented along its respective column of the rotation matrix.
- [ ] **2.2** Refactor `add_grasp_actors()` at `visualize_grasp_debug.py:620-715` to use `make_grasp_marker()` instead of the current arrow+sphere+axis approach. Remove the approach direction arrow logic (lines 644-674). Keep the score label logic (lines 701-715).
- [ ] **2.3** For the best grasp, make the marker slightly larger and use gold color. For other grasps, use grasp-type color and scale by score. Maintain existing score-based opacity scaling.
- [ ] **2.4** Remove the old coordinate frame arrows for the best grasp (lines 685-699) since the axis triangles now convey orientation.

### Phase 3: Replace Hand Spline with URDF-Based Joint/Link Skeleton

- [ ] **3.1** Add Pinocchio and model.py imports to `visualize_grasp_debug.py`. Follow the same try/except import pattern as `hand_tip_visualizer.py:7-36` with fallback for direct execution. Import `model`, `data`, `get_q_full`, `_q_full_with_thumb_mode`, `pin`, `COLLISION_GEOMETRIES`, `JOINT_ORDER` from the model module.
- [ ] **3.2** Define a joint connectivity list for the MIA hand based on the URDF kinematic tree. Use `(parent_joint_id, child_joint_id)` pairs:
  - `mia_palm` → `mia_thumb_opp` → `mia_thumb_sensor` → `mia_thumb_fle`
  - `mia_palm` → `mia_index_fle` → `mia_index_sensor`
  - `mia_palm` → `mia_middle_fle` → `mia_middle_sensor`
  - `mia_palm` → `mia_ring_fle`
  - `mia_palm` → `mia_little_fle`
  Use `model.getJointId()` to resolve names to IDs at module load time.
- [ ] **3.3** Rewrite `add_hand_skeleton()` at `visualize_grasp_debug.py:731-795`:
  1. Compute `q_full` from the best grasp's closure amount and grasp type using `get_q_full()` or `_q_full_with_thumb_mode()` (cylindrical/pinch use thumb opposition mode 1, lateral uses mode 0).
  2. Run `pin.forwardKinematics(model, data, q_full)`.
  3. Get world positions from `data.oMi[joint_id].translation` for each joint.
  4. Transform all joint positions by the grasp pose `T` (4x4 matrix from best grasp).
  5. Draw lines between parent-child joint pairs using `pv.Line()`.
  6. Draw small spheres at each joint position.
- [ ] **3.4** (Optional) Draw collision geometry wireframes (cylinders, spheres, boxes) at FK-computed world positions, reusing the wireframe drawing functions from `hand_tip_visualizer.py:39-122`. This gives a fuller hand shape but is not essential — joint lines+spheres are the minimum viable improvement.
- [ ] **3.5** Remove old LUT-based skeleton code: `FINGER_CHAINS` constant (lines 60-85), `TABLE_CONTACTS_PER_SAMPLE` (lines 49-55), `load_finger_lut()` (lines 137-167), `_dq_to_se3()` (lines 170-203), `_quat_to_matrix()` (lines 206-218). Keep `_quat_rotate()` (used by input hand pose arrows). Remove the `--lut-path` CLI argument and auto-detection logic (lines 1089-1104) since the URDF approach doesn't need it.
- [ ] **3.6** Add a graceful fallback: if Pinocchio or the URDF is not available, print a warning and skip hand skeleton rendering (the `h` key toggle will simply do nothing).

### Phase 4: Simplify Grasp Mode Toggle

- [ ] **4.1** Change `GRASP_MODES` at line 313 from `["best", "top10", "all", "none"]` to `["best", "all"]`.
- [ ] **4.2** Update `on_key_g()` at lines 895-901 to toggle between "best" and "all": simple boolean flip — if current is "best", switch to "all"; if "all", switch to "best".
- [ ] **4.3** Remove `get_grasp_indices()` logic for "top10" mode (lines 611-614) and "none" mode (line 589). Keep "best" and "all" branches.
- [ ] **4.4** Update docstring keyboard shortcuts (line 15-16) and help text (lines 944-956): `g - toggle grasp display: best / all`.
- [ ] **4.5** Verify `--show-all-grasps` CLI flag (line 1077-1079) still works: when set, start in "all" mode; otherwise "best" (line 345 already handles this).

## Verification Criteria

- [ ] **TSDF voxels render**: Running the viewer shows a cloud of small colored voxels in the viewport, colored red (inside) / white (surface) / blue (outside), fitting within the ROI bounding box. Terminal shows diagnostic info about observed voxel counts.
- [ ] **TSDF data layout correct**: Voxels appear at the correct spatial positions (coincident with point cloud points, inside the ROI box). If layout is still wrong, voxels would appear scattered or in the wrong location.
- [ ] **Hand skeleton shows joints+links**: Pressing `h` displays a recognizable hand skeleton with lines between joints and spheres at joint positions, positioned at the best grasp location. Thumb, index, middle, ring, little fingers are visually distinct chains.
- [ ] **Grasp markers show dot+triangles**: Each grasp is a small dot with 3 colored triangles (RGB for XYZ). No approach arrows or coordinate frame arrows remain.
- [ ] **Grasp mode toggles cleanly**: Pressing `g` switches between best-only and all grasps. No "top10" or "none" modes. Info text updates correctly.
- [ ] **No regressions**: Point cloud, ROI box, camera markers, input pose arrows, and info text all still work.

## Potential Risks and Mitigations

1. **Fortran-order fix may not resolve layout if PyVista version differs**
   - Mitigation: The diagnostic print will show whether data is being read correctly. If `order="F"` doesn't work, try `tsdf_vis.transpose(2, 1, 0).ravel(order="C")` which is equivalent. The voxel grid approach makes layout errors immediately visible (voxels in wrong places) vs the isosurface which silently fails.

2. **Voxel count may be high for large grids (0.3m ROI → ~250K voxels)**
   - Mitigation: The `threshold()` filter removes all `f32::MAX` (unobserved) voxels, typically leaving only the truncation shell (5K–40K). If still too many, add a secondary threshold to show only voxels within a narrower band (e.g., `|d| < 2` cells). Also, rendering wireframe voxels is much lighter than surface voxels.

3. **Pinocchio may not be installed in the user's environment**
   - Mitigation: Wrap the import in try/except. If unavailable, print a warning and skip hand skeleton rendering. The `hand_tip_visualizer.py` already demonstrates this fallback pattern.

4. **URDF file may not be found at runtime**
   - Mitigation: `model.py` computes `URDF_PATH` relative to its own script directory. Use the same path resolution. If missing, fall back gracefully.

5. **Grasp closure → q_full mapping may not exactly match Rust's scoring logic**
   - Mitigation: The LUT was generated from `model.py`, so the FK should be consistent. Verify visually that the hand skeleton contacts align with the TSDF surface voxels.

## Alternative Approaches

1. **TSDF: Use `pv.ImageData` with point_data instead of cell_data**
   - Assign values to grid nodes rather than cells. Avoids the cell-data ordering issue but requires dimensions to match data shape exactly (not shape+1). Slightly different visual result.

2. **TSDF: Hybrid — voxel grid + optional isosurface toggle**
   - Show voxel grid by default, but add a key (e.g., `s`) to toggle an isosurface overlay for when a cleaner surface view is desired. More complex but gives both views.

3. **Hand: Load and render actual STL meshes from URDF**
   - Use the mesh files referenced in the URDF for photorealistic rendering. Much more complex and requires mesh files to be available. Overkill for a debug viewer.

4. **Hand: Keep LUT-based approach but fix the chain definitions**
   - Fix `FINGER_CHAINS` to use correct contact indices and draw straight lines instead of splines. Simpler change but still fundamentally limited — contact points aren't joint centers.
