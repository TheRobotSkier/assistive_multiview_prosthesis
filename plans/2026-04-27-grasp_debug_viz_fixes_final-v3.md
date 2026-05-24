# Grasp Debug Visualization Fixes — Final Plan

## Objective

Fix 4 issues in `visualize_grasp_debug.py`:
1. Hand skeleton import fails — wrong `sys.path` calculation
2. TSDF occlusion — solid voxels hide interior structure
3. Grasp markers — simplify to points with Z-axis arrows, no labels
4. Camera markers — too small, scale up 2-3x

## File to Modify

`docker_ws/dev/grasp_preshaping/scripts/visualize_grasp_debug.py`

---

## Phase 1: Fix hand skeleton import (lines 82-100)

### Root Cause

Lines 85-88 compute `_repo_root` as 3 levels up from the script → `docker_ws/`. Then tries `from mia_hand_ros2_pkgs.dev.grasp_preshaping.scripts.model import ...`. But `model.py` is a sibling file in the same directory — no package path needed.

### Tasks

- [ ] **Task 1.1**: Replace the import block (lines 84-100). Add `_script_dir` to `sys.path` and do `from model import model as pin_model, data as pin_data, get_q_full, _q_full_with_thumb_mode, COLLISION_GEOMETRIES` plus `import pinocchio as pin`. Keep try/except for graceful fallback.

---

## Phase 2: TSDF multi-mode display (lines 355-397)

### Root Cause

All observed voxels rendered as solid at `opacity=0.5`. The TSDF shell is thick (up to 4 cells per side), so outer voxels completely occlude interior structure.

### Tasks

- [ ] **Task 2.1**: Add `TSDF_MODES = ["surface", "points", "off"]` constant. Add `"tsdf_mode"` to state dict, default `"surface"`.

- [ ] **Task 2.2**: Rewrite `add_tsdf_actors()` with two rendering paths:
  - **surface mode**: Threshold to `|distance| < 1.5` cells, render as solid voxels at `opacity=0.7` with `coolwarm` colormap. Only the thin zero-crossing band is shown — no occlusion.
  - **points mode**: Show all observed voxels as points (`style="points"`, `render_points_as_spheres=True`), colored by `coolwarm` distance, `point_size=6`. Full structure visible from any angle.

- [ ] **Task 2.3**: Update `on_key_t()` to cycle through `TSDF_MODES`, removing and re-adding TSDF actors each time.

- [ ] **Task 2.4**: Update info text to show current TSDF mode name.

---

## Phase 3: Simplify grasp markers (lines 247-294, 530-600)

### Root Cause

Current grasp markers use `_make_grasp_marker()` with sphere + 3 axis cones + score labels. This is visually cluttered, slow to render with many grasps, and the labels add noise.

### Tasks

- [ ] **Task 3.1**: Remove `_make_grasp_marker()` function entirely (lines 247-294).

- [ ] **Task 3.2**: Rewrite `add_grasp_actors()` to render grasps as:
  - A **point** at the grasp position, colored by grasp type (`GRASP_TYPE_COLORS`), sized by score (larger = better). Use `pv.PolyData` with all grasp positions, `add_mesh(style="points", render_points_as_spheres=True)`.
  - A single **Z-axis arrow** per grasp (short line or thin arrow along `-T[:3, 2]`, the approach direction), same color as the point. For "best" mode, make the arrow slightly longer/thicker.
  - **No text labels** — remove all `add_point_labels()` calls for grasps.

- [ ] **Task 3.3**: For "best" mode, still render the best grasp slightly larger and in gold, but still as a point + Z-arrow (no extra geometry).

- [ ] **Task 3.4**: Remove `actor_groups["grasp_labels"]` and all label-related code from `rebuild_grasps()`.

---

## Phase 4: Enlarge camera markers (lines 418-445)

### Root Cause

Camera sphere radius is `scene_extent * 0.02` — too small relative to the scene.

### Tasks

- [ ] **Task 4.1**: Change camera sphere radius from `scene_extent * 0.02` to `scene_extent * 0.05` (2.5x larger). Scale the direction cone proportionally.

---

## Verification Criteria

- [ ] Script starts without "Hand skeleton: unavailable" — prints "pinocchio + URDF loaded"
- [ ] `h` key renders hand skeleton with colored joints and links
- [ ] `t` key cycles: surface → points → off → surface
- [ ] Surface mode shows thin zero-crossing band only, no occlusion
- [ ] Points mode shows all observed voxels, visible from any angle
- [ ] Grasps render as colored points (size = score) with Z-axis arrows, no labels
- [ ] Cameras are visibly larger (2.5x previous size)

## Potential Risks and Mitigations

1. **Risk: `model.py` import fails if pinocchio not installed**
   Mitigation: try/except keeps graceful fallback — hand skeleton disabled but everything else works

2. **Risk: Points mode slow with very large grids (>200K observed voxels)**
   Mitigation: PyVista point rendering is efficient; if needed, subsample to ~50K points

3. **Risk: Grasp points too small to see**
   Mitigation: Use `point_size` scaled to scene extent (min 8px), and `render_points_as_spheres=True`
