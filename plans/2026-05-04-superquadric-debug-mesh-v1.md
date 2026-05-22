# Superquadric Primitive Mesh in Debug Visualizer

## Objective

Add a toggleable superquadric mesh overlay to the PyVista debug visualizer. Rather than evaluating the exact superquadric implicit function to extract a mesh (expensive and complex), we approximate it with the **geometric primitive** that best matches the winning template (sphere, box, or cylinder). This gives an intuitive visual representation of what the backside estimator "thinks" the object looks like, togglable with a single keypress.

## Current State

### What's already in place
- **Rust side** (`src/debug_export.rs:247-285`): The NPZ already exports `sq_params` (14 floats: e1, e2, a, b, c, tx, ty, tz, r00..r22) and `sq_meta` (fit_error, template_index) when superquadric fitting succeeds.
- **Python side** (`scripts/visualize_grasp_debug.py`): The `load_dump()` function does **not** yet read `sq_params`/`sq_meta`. The visualizer has no superquadric display.

### NPZ data format (from `src/debug_export.rs:247-285`)
```
sq_params: [e1, e2, a, b, c, tx, ty, tz, r00, r01, r02, r10, r11, r12]  (f32, 14)
sq_meta:   [fit_error, template_index]  (f32, 2)
           template_index: 0=sphere, 1=box, 2=cylinder
```

Note: The rotation matrix is stored as a 3x2 matrix (first two rows only: r00,r01,r02,r10,r11,r12). The third row can be recovered as the cross product of the first two rows.

## Implementation Plan

### Part 1: Python — Load superquadric data from NPZ

- [ ] **1.1** In `load_dump()` (`scripts/visualize_grasp_debug.py:171-325`), add loading of `sq_params` and `sq_meta` arrays from the NPZ file. These arrays are optional (only present when SQ fitting succeeded), so use a backward-compatible `try/except` or `in data.files` check. Parse into a dict:
  ```python
  sq = None
  if "sq_params" in data.files:
      raw = data["sq_params"]
      meta = data["sq_meta"]
      sq = {
          "e1": float(raw[0]),
          "e2": float(raw[1]),
          "a": float(raw[2]), "b": float(raw[3]), "c": float(raw[4]),
          "translation": raw[5:8].astype(np.float64),
          "rotation": np.array([
              [raw[8], raw[9], raw[10]],
              [raw[11], raw[12], raw[13]],
          ], dtype=np.float64),
          "fit_error": float(meta[0]),
          "template_index": int(meta[1]),
      }
  ```
  Add `"sq": sq` to the returned dict.

- [ ] **1.2** In `print_summary()` (`scripts/visualize_grasp_debug.py:387-462`), add a line printing the superquadric info when present: template name, fit error, and scale parameters.

### Part 2: Python — Build primitive mesh from SQ params

- [ ] **2.1** Create a helper function `_build_sq_primitive_mesh(sq)` that takes the parsed SQ dict and returns a `pv.PolyData` mesh. The function reads `template_index` and constructs the corresponding PyVista primitive, then transforms it:

  **Template 0 (sphere):** Use `pv.Sphere(radius=a)` centered at origin, then scale Y by `b/a` and Z by `c/a` to make an ellipsoid matching the three axis scales.

  **Template 1 (box):** Use `pv.Box(bounds=[-a, a, -b, b, -c, c])` centered at origin.

  **Template 2 (cylinder):** Use `pv.Cylinder(radius=a, height=2*c, direction=[0,0,1], center=[0,0,0])`, then scale Y by `b/a` to make an elliptical cross-section if `b != a`.

  After creating the local-frame primitive, apply the rotation matrix and translation:
  - Compute the full 3x3 rotation: first two rows from NPZ, third row = cross product of row 0 and row 1.
  - Apply the transformation using `pv.PolyData.transform()` with a 4x4 homogeneous matrix `[R | t; 0 0 0 1]`.

  This avoids evaluating the superquadric implicit function entirely — we just use PyVista's built-in primitives as approximations.

- [ ] **2.2** Add a `_SQ_TEMPLATE_NAMES` constant: `{0: "sphere", 1: "box", 2: "cylinder"}` for display purposes.

### Part 3: Python — Integrate into PyVista visualizer

- [ ] **3.1** Add `"show_sq": False` to the `state` dict (`scripts/visualize_grasp_debug.py:510-521`) and add `"sq_mesh"` to `actor_groups` (`scripts/visualize_grasp_debug.py:529-540`).

- [ ] **3.2** Create an `add_sq_actors()` function (following the same pattern as `add_camera_actors`, `add_roi_actors`, etc.) that:
  1. Checks `dump["sq"]` is not None
  2. Calls `_build_sq_primitive_mesh(dump["sq"])`
  3. Adds it as a semi-transparent wireframe mesh (e.g., `opacity=0.3, style="wireframe", color="cyan", line_width=2`) so it doesn't obscure the point cloud or TSDF
  4. Stores the actor in `actor_groups["sq_mesh"]`

- [ ] **3.3** Bind the `s` key to a toggle function `on_key_s()` that:
  1. Flips `state["show_sq"]`
  2. If turning on and no actors exist, calls `add_sq_actors()`
  3. If turning off, hides existing actors via `SetVisibility(False)` (or removes and rebuilds on next toggle)
  4. Updates info text and re-renders

- [ ] **3.4** Register the key binding: `plotter.add_key_event("s", on_key_s)` alongside the existing key bindings (`scripts/visualize_grasp_debug.py:1504-1519`).

### Part 4: Python — Update info text and help

- [ ] **4.1** Update `update_info_text()` (`scripts/visualize_grasp_debug.py:1259-1302`) to include the SQ state in the status line (e.g., `SQ: sphere (err=0.012)` or `SQ: off`).

- [ ] **4.2** Update the help text in `on_key_help()` (`scripts/visualize_grasp_debug.py:1485-1502`) and the docstring header (`scripts/visualize_grasp_debug.py:1-30`) to document the `s` key.

- [ ] **4.3** Add "SQ mesh" to the legend entries (`scripts/visualize_grasp_debug.py:1307-1318`) with color "cyan".

## Verification Criteria

- [ ] Loading an NPZ **without** `sq_params` (old dumps) still works without errors
- [ ] Loading an NPZ **with** `sq_params` shows the SQ info in the console summary
- [ ] Pressing `s` toggles the primitive mesh on/off
- [ ] The mesh is positioned and oriented correctly (matches the point cloud surface)
- [ ] The mesh shape matches the winning template (sphere ≈ round, box ≈ rectangular, cylinder ≈ elongated)
- [ ] The mesh does not obscure other visual elements (semi-transparent wireframe)
- [ ] The info text updates to show SQ state

## Potential Risks and Mitigations

1. **Rotation matrix reconstruction**: Only 6 of 9 rotation entries are stored (first two rows). The third row must be computed as `row0 × row1`. If the stored rows aren't orthonormal (due to float precision), the reconstruction may have slight distortion.
   - Mitigation: Re-orthogonalize using SVD or just accept the minor visual artifact for a debug tool.

2. **Primitive approximation fidelity**: A superquadric with ε₁=0.5 doesn't look like any of the three primitives. The visual will be "close but not exact."
   - Mitigation: This is a debug visualization, not a production mesh. The approximation is intentional for performance and simplicity. The wireframe style makes it clear it's an approximation.

3. **Scale mismatch**: The Gauss-Newton solver may push the translation into the unobserved region, making the mesh appear offset from the visible point cloud.
   - Mitigation: This is actually the desired behavior — the mesh should show where the solver thinks the backside is. The offset is informative for debugging.

4. **Backward compatibility**: Old NPZ dumps won't have `sq_params`/`sq_meta`.
   - Mitigation: Use `"sq_params" in data.files` check in `load_dump()`. Default `sq` to `None`.

## Alternative Approaches

1. **Exact superquadric mesh via marching cubes**: Could evaluate F(x,y,z) on a grid and extract the zero-crossing. This gives a precise superquadric surface but adds complexity (marching cubes in Python, grid sampling) and is slower.
   - Trade-off: More accurate but overkill for a debug overlay. The primitive approximation is sufficient for visual verification.

2. **Parametric superquadric surface**: Sample the parametric form (θ, φ) → (x, y, z) and build a mesh from the grid. More accurate than primitives for intermediate ε values.
   - Trade-off: Better visual fidelity for non-standard ε values, but adds ~30 lines of math. Could be added later if the primitive approach feels insufficient.

3. **Point cloud sampling of the superquadric surface**: Scatter points on the SQ surface and display as a point cloud.
   - Trade-off: Fast to implement but visually noisy and harder to see the shape. Wireframe mesh is clearer.
