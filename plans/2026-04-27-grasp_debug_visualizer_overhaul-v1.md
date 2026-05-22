# Grasp Debug Visualizer Overhaul

## Objective

Transform the `visualize_grasp_debug.py` script from a barely-legible viewer into an intuitive, interactive 3D visualization where the user can clearly see and understand:
1. The point cloud (currently the only visible element)
2. The TSDF isosurface (currently invisible due to scale/opacity issues)
3. Candidate grasp poses (currently too small and indistinguishable)
4. The spatial relationship between all elements

---

## Root Cause Analysis

After thorough investigation of the visualization script (`scripts/visualize_grasp_debug.py:1-427`), the Rust data export (`src/debug_export.rs:1-303`), the pipeline logic (`src/c_api.rs:1-520`, `src/planner.rs:1-480`, `src/pointcloud_helper.rs:1-694`), and the config (`src/config.rs:1-32`), the following issues have been identified:

### Issue 1: TSDF Isosurface Likely Invisible or Broken (HIGH)
**Source**: `scripts/visualize_grasp_debug.py:136-159`

The TSDF data uses integer cell-count distances (not meters). The distance field from `get_tsdf()` stores cell counts (0, 1, 2, 3, 4 for `TRUNCATION_CELLS=4`) multiplied by `resolution_m=0.005` — actually looking at the code in `pointcloud_helper.rs:384-410`, the BFS stores integer distances (`cdist + 1.0`), so values are 0.0, 1.0, 2.0, 3.0, 4.0 or `f32::MAX`. These are NOT in meters. The contour at `d=0.0` should still work for extracting the surface, BUT the values >1e10 filter at line 139 correctly catches `f32::MAX` unobserved voxels.

However, the real problem is that the TSDF sign-flipping logic (`pointcloud_helper.rs:412-461`) uses camera positions to determine inside/outside. If the camera data is sparse or the geometry is small, the sign may not flip correctly, resulting in no zero-crossing at all — meaning the contour returns an empty mesh. Additionally, the isosurface is rendered at `opacity=0.6` with `color="lightblue"` on a white/lightgray background, making it very hard to see.

**Impact**: The TSDF surface either doesn't render at all or is nearly invisible.

### Issue 2: Grasp Candidate Arrows Are Microscopic (HIGH)
**Source**: `scripts/visualize_grasp_debug.py:248-273`

The scene operates at centimeter scale (hand radius ~5cm, ROI ~10-30cm, objects ~2-4cm). The grasp axes arrows are drawn with:
- Best grasp: `axes_len = 0.012` (12mm), `shaft_radius = 0.0012` (1.2mm), `tip_radius = 0.003` (3mm)
- Non-best grasps: `axes_len = 0.004` (4mm), `shaft_radius = 0.00025` (0.25mm), `tip_radius = 0.0007` (0.7mm)

While the absolute sizes are reasonable for the scene scale, the non-best grasps at `opacity=0.18` are essentially invisible. Even the best grasp's coordinate frame (3 small arrows) is hard to interpret — there's no hand shape, no approach direction indicator, and no clear grasp type visualization.

**Impact**: Grasp candidates are nearly impossible to see and interpret.

### Issue 3: Only Best Grasp Shown by Default (HIGH)
**Source**: `scripts/visualize_grasp_debug.py:231-241`

By default (`--show-all-grasps` not set), only the single best grasp is rendered. With `--show-all-grasps`, all grasps above threshold are shown but at 18% opacity with 4mm arrows — essentially invisible. There's no middle ground and no way to toggle individual grasps interactively.

**Impact**: User cannot compare or understand the distribution of grasp candidates.

### Issue 4: No Hand/Finger Geometry Visualization (MEDIUM-HIGH)
**Source**: `scripts/visualize_grasp_debug.py:191-204`

The input hand pose is shown as three tiny arrows (RGB axes) with `hand_axes_len = 0.015` (15mm). The grasp candidates show only a 3-axis coordinate frame. There is no visualization of:
- The hand shape/outline
- Finger positions at the closure amount
- The approach direction
- The contact points on the object

The LUT data (`lut_helper.rs`) contains full finger contact geometry that could be used to draw actual hand configurations.

**Impact**: Impossible to understand what the hand would look like at each grasp pose.

### Issue 5: Camera Positions Too Small (MEDIUM)
**Source**: `scripts/visualize_grasp_debug.py:186-188`

Camera positions are rendered as spheres with `radius=0.005` (5mm). Given the scene scale, these are barely visible dots.

### Issue 6: ROI Box May Be Confusing (LOW)
**Source**: `scripts/visualize_grasp_debug.py:176-183`

The ROI AABB is drawn as a wireframe box in orange. This is functional but doesn't indicate the relationship between the ROI and the predicted hand trajectory.

### Issue 7: Point Cloud Too Transparent (LOW)
**Source**: `scripts/visualize_grasp_debug.py:163-173`

The point cloud is rendered at `opacity=0.35` with `color="gray"` and `point_size=6`. On a white background, this is quite washed out.

### Issue 8: No Interactive Controls or Information Panel (MEDIUM)
**Source**: `scripts/visualize_grasp_debug.py:275-297`

The viewer shows a static text overlay with basic info. There are no:
- Keyboard shortcuts to toggle element visibility
- Click-to-select grasps with score details
- Ability to cycle through grasps
- Color coding by score quality

---

## Implementation Plan

### Phase 1: Fix Critical Visibility Issues

- [ ] **Task 1.1**: Fix TSDF isosurface rendering. In `visualize_pyvista()` (line 136-159), after computing the contour, add a fallback: if the zero-crossing contour is empty, try contouring at small positive values (e.g., `[0.5, 1.0]`) to at least show the truncation band. Also add a TSDF voxel slice visualization as an alternative when isosurface fails. Increase TSDF surface opacity to 0.8 and use a more visible color (e.g., `"deepskyblue"` with edge coloring).

- [ ] **Task 1.2**: Add TSDF debug slice rendering. Add a new `--tsdf-slice` argument (default: `"z"`) and `--tsdf-slice-index` argument to render a 2D heatmap slice of the TSDF values as a colored plane in the 3D view. This makes the TSDF data visible even when isosurface extraction fails. Create a `pv.ImageData` for the selected slice and use `plotter.add_mesh(slice, scalars="distance", cmap="coolwarm", clim=[-4, 4])`.

- [ ] **Task 1.3**: Increase point cloud visibility. Change point cloud opacity from `0.35` to `0.7`, point size from `6` to `8`, and color from `"gray"` to `"dimgray"` or `"slategray"`. This ensures the point cloud remains the primary visual anchor.

- [ ] **Task 1.4**: Fix camera position markers. Increase sphere radius from `0.005` to `0.01` and add a small pyramid/cone pointing toward the ROI center to indicate viewing direction.

### Phase 2: Grasp Visualization Overhaul

- [ ] **Task 2.1**: Replace arrow-based grasp visualization with hand-shaped glyphs. For each grasp candidate, render a simplified hand outline using the finger contact positions from the LUT. Create a new function `draw_hand_glyph(plotter, T, grasp_type, closure, lut_data, color, opacity)` that:
  - Draws a small palm rectangle at the grasp pose origin
  - Draws finger lines from palm to fingertip positions at the given closure amount
  - Uses different finger configurations for cylindrical (all fingers curved), pinch (thumb+index opposed), and lateral (thumb side-pressed) grasp types
  - This requires loading the LUT data in the Python visualizer — add a `--lut-path` argument and a `load_lut_contacts()` function that reads the finger_contact_lut.npz

- [ ] **Task 2.2**: Add grasp approach direction arrows. For each displayed grasp, draw a thick arrow (shaft_radius=0.002, tip_radius=0.004, length=0.03) along the grasp approach direction (typically the -Z axis of the grasp pose). Color by grasp type. This makes each grasp immediately visible and indicates the approach vector.

- [ ] **Task 2.3**: Implement score-based coloring and sizing. Instead of binary best/non-best, use a continuous color map based on `combined_score`. Map the score range to a colormap (e.g., `plasma` or `viridis`). Scale the hand glyph size and approach arrow length proportionally to the score. This allows the user to visually identify high-scoring regions.

- [ ] **Task 2.4**: Add grasp type labels. Use `plotter.add_point_labels()` to place text labels at each grasp position showing the grasp type name and combined score (e.g., "cyl 0.73"). Only show labels for grasps above a configurable threshold to avoid clutter.

### Phase 3: Interactive Controls

- [ ] **Task 3.1**: Add PyVista key-event handlers for toggling visibility. Implement keyboard shortcuts:
  - `t` — toggle TSDF surface on/off
  - `g` — toggle grasp candidates on/off
  - `p` — toggle point cloud on/off
  - `r` — toggle ROI box on/off
  - `c` — toggle camera positions on/off
  - `a` — cycle through showing: best only → all above threshold → all grasps → back to best
  - `1/2/3` — filter by grasp type (cylindrical/pinch/lateral)
  - `h` — print help text to console

  This requires storing actor references and using `plotter.add_key_event()`.

- [ ] **Task 3.2**: Add point-picking for grasp inspection. Enable `plotter.enable_point_picking()` so the user can click on a grasp to see its full score breakdown printed to the console. Use `plotter.enable_cell_picking()` or custom callback with `picked_callback`.

- [ ] **Task 3.3**: Add an info panel. Replace the static `add_text()` call with a dynamic info box that updates when the user toggles elements. Show: number of grasps displayed, current filter mode, threshold, best grasp details.

### Phase 4: Hand Geometry Visualization

- [ ] **Task 4.1**: Create a Python-side LUT loader. Add a `load_finger_lut(path)` function that reads `finger_contact_lut.npz` and returns a dict mapping `(contact_name, sample_index) -> (position, transform_4x4)`. This mirrors the Rust `FingerLUT` but in pure numpy.

- [ ] **Task 4.2**: Implement hand skeleton drawing. Using the LUT data and a grasp's 4x4 pose + closure amount, compute all finger contact positions in world space. Draw lines (using `pv.Line()` or `pv.Spline()`) connecting palm → MCP → PIP → DIP → tip for each finger. Draw small spheres at contact points. This creates a stick-figure hand that clearly shows the grasp configuration.

- [ ] **Task 4.3**: Draw the best grasp's hand skeleton with collision contact points highlighted. For the best grasp, use a distinct color (gold) and larger spheres at contact points that are within `collision_tol` of the TSDF surface. This shows exactly where the hand would touch the object.

### Phase 5: Polish and Robustness

- [ ] **Task 5.1**: Set an appropriate initial camera position. Compute the centroid of the point cloud and set the camera to look at it from a reasonable distance. Use `plotter.camera_position` to set an isometric-like view that shows the full scene.

- [ ] **Task 5.2**: Improve the legend. Replace the simple legend with a more informative one that includes all element types, grasp type color coding, and score range.

- [ ] **Task 5.3**: Add a `--summary-only` mode that prints detailed statistics without opening the 3D viewer. Include: score distribution histogram (text-based), grasp type breakdown, collision rate, etc.

- [ ] **Task 5.4**: Handle edge cases gracefully. Add checks for: empty TSDF, no grasps with collision, degenerate ROI, and missing LUT file. Print informative messages instead of crashing or showing an empty scene.

- [ ] **Task 5.5**: Add `--output-screenshot` argument to save a PNG of the view for documentation/sharing.

---

## Files to Modify

### Primary file (all changes):
- `docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py`

### No changes needed to:
- `src/debug_export.rs` — data format is well-designed and sufficient
- `src/c_api.rs` — pipeline logic is correct
- `src/pointcloud_helper.rs` — TSDF construction is correct
- `src/planner.rs` — scoring logic is correct
- `src/config.rs` — constants are appropriate

The visualization issues are entirely in the Python rendering layer, not in the data generation pipeline.

---

## Verification Criteria

1. **TSDF isosurface is clearly visible**: Running the viewer with a debug dump shows a clearly visible blue surface mesh representing the object, or a colored slice plane if isosurface extraction fails
2. **Grasp candidates are distinguishable**: Each grasp candidate shows as a visible hand-shaped glyph or approach arrow, colored by grasp type, with labels showing scores
3. **Best grasp is immediately identifiable**: The best grasp is visually prominent (larger, gold-colored, with finger skeleton)
4. **Interactive toggling works**: Pressing `t`, `g`, `p`, `r` toggles respective elements on/off
5. **Score distribution is visible**: Multiple grasps shown with color gradient indicating quality
6. **Point cloud remains clearly visible**: Point cloud is the primary visual anchor at reasonable opacity
7. **No crashes on edge cases**: Empty TSDF, no grasps, or missing LUT handled gracefully

---

## Potential Risks and Mitigations

1. **LUT file may not be available at visualization time**
   Mitigation: Make hand skeleton drawing optional (`--lut-path` argument). Fall back to approach-arrow visualization if LUT is not provided. The LUT is at `docker_ws/dev/grasp_preshaping/data/finger_contact_lut.npz`.

2. **PyVista version compatibility**
   Mitigation: Test with both PyVista >=0.38 (current stable) and provide matplotlib fallback that includes the improved grasp visualization (colored dots + approach lines).

3. **Performance with many grasp candidates (1000 samples × 3 types = 3000 grasps)**
   Mitigation: By default only render grasps above threshold. For `--show-all-grasps`, use point glyphs instead of full hand skeletons. Add `--max-grasps` argument to limit rendering.

4. **TSDF contour may genuinely be empty (no zero-crossing)**
   Mitigation: Always provide the TSDF slice as a fallback visualization. Print a warning when isosurface is empty explaining why.

5. **Coordinate frame conventions may differ**
   Mitigation: The data uses a consistent convention throughout the Rust pipeline. The Python visualizer already uses the same 4x4 matrices. Verify by checking that the input pose axes align with the point cloud centroid.

---

## Alternative Approaches

1. **Use Open3D instead of PyVista**: Open3D has better point cloud rendering and supports custom geometries. Trade-off: less interactive (no key events), but better performance for large point clouds. Not recommended since PyVista is already a dependency.

2. **Use a web-based viewer (e.g., Polyscope or Trimesh)**: Would allow sharing visualizations. Trade-off: more dependencies, less mature 3D interaction. Not recommended for a debug tool.

3. **Export to MeshLab/Blender format**: Instead of an interactive viewer, export all elements as a PLY/OBJ scene file. Trade-off: requires external tool, but gives maximum rendering quality. Could be added as a `--export` option alongside the interactive viewer.

4. **Minimal fix approach**: Only fix the scale/opacity issues without adding interactivity. This would be Tasks 1.1-1.4 and 2.2-2.3 only. Trade-off: faster to implement but less useful for ongoing debugging.
