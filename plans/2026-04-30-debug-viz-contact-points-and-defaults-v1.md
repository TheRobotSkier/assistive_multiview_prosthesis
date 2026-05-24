# Debug Visualizer Enhancements: LUT Contact Points, Default State, and TSDF Off

## Objective

Add three enhancements to the PyVista debug visualizer (`scripts/visualize_grasp_debug.py`):

1. **LUT Contact Points Visualization** — Render the LUT's contact point positions for the best (and top-N) grasps so the user can see where each finger's contact points land relative to the object surface.
2. **Default Startup State** — Change the default initial state so the viewer starts with: grasp mode = "all", iteration filter = last iteration, TSDF = off.
3. **TSDF Legend Color** — The TSDF scalar bar text is already white (fixed in the current code at `scripts/visualize_grasp_debug.py:411` with `pv.global_theme.font.color = 'white'`), but this should be verified.

## Analysis Summary

### Current Architecture

The debug pipeline works as follows:

1. **Rust side** (`src/c_api.rs:482-543`): The SMC loop collects all scored grasps across all iterations. Each grasp stores: sample_index, grasp_type, closure, alignment, force_closure, contact_count_score, contact_score, active_contact_count, found_collision, combined_score, probability, wrist_rotation, smc_iteration, and a 4x4 pose matrix. This is exported to `.npz` via `src/debug_export.rs`.

2. **Python side** (`scripts/visualize_grasp_debug.py`): Loads the dump, renders TSDF, point cloud, ROI, cameras, grasp markers, and optionally hand skeletons (using Pinocchio FK). The hand skeleton is rendered by running FK through `scripts/model.py` which uses Pinocchio + the URDF.

3. **LUT contact points** (`src/lut_helper.rs`): The `FingerLUT` stores dual-quaternion transforms for 25 contact points at multiple closure samples. The `Contact` enum defines all contact names (IndexMcp, IndexTip, ThumbAbdTip, etc.). In `src/planner.rs`, the scorer uses `pos_at_control()` / `pos_at_sample()` to transform these contact points via the base_transform to get world-space positions.

### Key Insight: Contact Points in the Viewer

The contact points from the LUT are **not currently exported** in the debug dump. However, the viewer already has Pinocchio-based FK that computes contact positions via `_compute_hand_positions()` (line 885-908). This function uses `CONTACT_DEFINITIONS` from `model.py` which defines the same 25 contact points as the LUT.

**The critical difference**: The Pinocchio FK path uses full joint-space FK with `q_active = [closure, closure, closure]`, while the LUT uses pre-computed dual-quaternion interpolation. For visualization purposes, the Pinocchio FK path is sufficient and already available — it produces the same contact positions that the LUT would produce (they were generated from the same model).

**However**, the current hand skeleton (`_HAND_SKELETON_SPECS` at line 53-59) only renders 5 contact points (one tip per finger: ThumbAddTip, IndexTip, MiddleTip, RingTip, LittleTip). The LUT has 25 contact points covering MCP, PIP, DIP, Tip, and Side contacts for each finger, plus 4 palm contacts.

### Plan for Contact Points

There are two viable approaches:

- **Approach A (Recommended)**: Extend the existing Pinocchio FK path to render ALL 25 contact points from `CONTACT_DEFINITIONS` instead of just the 5 tips. This reuses the existing infrastructure and gives a comprehensive view of where the LUT's contacts land.

- **Approach B**: Load the actual `.npz` LUT file in the viewer and compute contact positions using the LUT's dual quaternions directly. This would be more faithful to the actual scoring path but requires loading the LUT file and reimplementing the DQ interpolation in Python.

Approach A is recommended because: (a) the Pinocchio FK already works and produces identical positions, (b) it requires no new file loading, (c) `get_sampled_contact_transforms()` in `model.py` already computes all 25 contact transforms.

## Implementation Plan

### Phase 1: Default Startup State Changes

- [x] **Task 1.1**: Modify the `state` dictionary initialization in `visualize_pyvista()` (line 433-443) to change defaults:
  - Set `"grasp_mode": "all"` (currently `"best"` when `--show-all-grasps` is not passed)
  - Set `"tsdf_mode": "off"` (currently `"surface"` when `--no-tsdf` is not passed)
  - Set `"iteration_filter"` to the last SMC iteration (`max_iteration`) instead of `None`
  
  Rationale: The user wants to immediately see the final optimization results without TSDF clutter.

- [x] **Task 1.2**: Move the `max_iteration` computation (currently at line 692-695) **before** the `state` dictionary initialization so it can be used as the default iteration filter. Currently `max_iteration` is computed after `state`, so it needs to be reordered.

- [x] **Task 1.3**: Ensure the initial TSDF actors are not added when default is "off". The guard at line 567 (`if state["tsdf_mode"] != "off": add_tsdf_actors()`) already handles this correctly, so no change needed — just verify.

- [x] **Task 1.4**: Update the `--no-tsdf` and `--show-all-grasps` CLI flags to be the default behavior (or remove them / make them no-ops since they match the new defaults). The `args` object is only used at line 434-435 for initial state, so changing the defaults in `state` supersedes these flags. Keep the flags for backward compatibility but note they are now the default.

### Phase 2: LUT Contact Points Visualization

- [x] **Task 2.1**: Add a new toggle state key `"show_contacts"` to the `state` dict (default `True` since it's the new feature and will help debugging). Add a corresponding `actor_groups` key `"contact_points"` for managing the actors.

- [x] **Task 2.2**: Create a new function `add_contact_points()` that renders all 25 LUT contact points for the top-N grasps. This function should:
  - Use `get_sampled_contact_transforms()` from `model.py` (already imported at line 83-90) to compute all 25 contact positions for a given grasp's closure amount and grasp type
  - For each top grasp (from `get_grasp_indices()` → `top_indices`):
    1. Extract the grasp's closure amount, grasp type, and pose
    2. Build `q_active` from the closure amount (handle grasp-type-specific thumb mode: 0.0 for lateral, 1.0 for cylindrical/pinch)
    3. Call `get_sampled_contact_transforms(q_active, thumb_opp_mode=thumb_mode)` to get all 25 contact transforms in hand-local frame
    4. Transform each contact position to world frame using the grasp's 4x4 pose matrix
    5. Render as colored points, using `FINGER_COLORS` for finger-group coloring (same as hand skeleton)
  - Render all contacts as a single batched point cloud for efficiency (one `add_mesh` call per grasp or one for all)
  
  Rationale: Using `get_sampled_contact_transforms()` gives us all 25 contact positions that match the LUT exactly (since the LUT was generated from the same model). This is much more informative than the current 5-point hand skeleton.

- [x] **Task 2.3**: Handle the grasp-type-specific mapping for `q_active` and thumb mode:
  - Cylindrical (type 1): `q_active = [closure, closure, closure]`, `thumb_opp_mode = 1.0`
  - Pinch (type 2): `q_active = [closure, closure, 0.0]`, `thumb_opp_mode = 1.0`
  - Lateral (type 3): `q_active = [closure, closure, 0.0]`, `thumb_opp_mode = 0.0`
  
  This mirrors the grasp specs in `src/planner.rs` (cylindrical_spec, pinch_spec, lateral_spec). For pinch and lateral, MRL fingers are locked open (0.0). The thumb opposition mode determines whether ThumbAdd or ThumbAbd contacts are used.

- [x] **Task 2.4**: Differentiate active vs inactive contacts visually:
  - Contacts from `score_contacts` in the grasp spec (the ones used for scoring) should be rendered larger/brighter
  - Sweep-only contacts (used for collision detection but not scoring) should be rendered smaller/dimmer
  - Palm contacts are always present (locked at sample 0) and should be a distinct style
  
  Map the Rust grasp spec `score_contacts` to contact names:
  - Cylindrical score contacts: ThumbAbdTip, IndexMcp/Dip/Pip/Tip, MiddleMcp/Pip/Dip/Tip, RingDip/Pip/Tip, LittleDip/Pip/Tip, PalmProxUlna/Radi/DistUlna/DistRadi (21 contacts)
  - Pinch score contacts: ThumbAbdTip, IndexTip (2 contacts)
  - Lateral score contacts: ThumbAddTip, IndexMcpSide/DipSide/PipSide/TipSide (5 contacts)

- [x] **Task 2.5**: Add the contact points rendering to the `rebuild_grasps()` function (line 1134-1147) so they refresh when the user changes grasp mode, iteration filter, or grasp type filter. Also add clearing logic for `actor_groups["contact_points"]`.

- [x] **Task 2.6**: Add a keyboard shortcut (suggest `k` for "kontacts") to toggle contact point visibility. Register with `plotter.add_key_event("k", on_key_k)` and implement `on_key_k()` following the same pattern as `on_key_h()`.

- [x] **Task 2.7**: Update the info text (`update_info_text()`) to show the contact points state.

- [x] **Task 2.8**: Update the legend to include a "Contact points" entry.

- [x] **Task 2.9**: Update the docstring keyboard shortcuts section (lines 13-28) to document the new `k` key.

### Phase 3: Verify TSDF Legend Text Color

- [x] **Task 3.1**: Verify that the TSDF scalar bar text is white. The current code sets `pv.global_theme.font.color = 'white'` at line 411, and the scalar bar args at lines 519-527 and 555-563 do not override the text color. Confirm this is working correctly — if the text is still black, explicitly set `"color": "white"` in the `scalar_bar_args` dictionaries.

## Verification Criteria

- [x] Viewer starts with TSDF off (no TSDF voxels rendered on launch)
- [x] Viewer starts with grasp mode "all" (all qualifying grasps shown, not just best)
- [x] Viewer starts with iteration filter set to the last SMC iteration
- [x] Pressing `k` toggles 25 contact points per top grasp
- [x] Contact points are correctly positioned in world frame for each grasp
- [x] Contact points are colored by finger group (matching hand skeleton colors)
- [x] Score contacts are visually distinct from sweep-only contacts
- [x] TSDF legend text is white and legible on the dark background
- [x] Existing keyboard shortcuts (t, g, p, r, c, h, i, +/-, *, 0-3, ?) still work
- [x] Contact points update correctly when switching grasp mode, iteration filter, or type filter

## Potential Risks and Mitigations

1. **Performance with many contact points**
   Risk: Rendering 25 contacts × 10 top grasps = 250 additional point actors could slow the viewer.
   Mitigation: Batch all contact points into a single `PolyData` with per-point colors, using a single `add_mesh` call. This is the same pattern used for grasp points (line 789-800).

2. **Pinocchio FK mismatch with LUT positions**
   Risk: The Pinocchio FK might produce slightly different positions than the LUT's dual-quaternion interpolation at intermediate closure values.
   Mitigation: The LUT was generated from the same Pinocchio model (`generate_contact_lut` in `model.py`), so positions should match exactly at sample points and very closely at interpolated values. Any discrepancy would be sub-millimeter and irrelevant for visualization.

3. **Closure amount mapping for grasp types**
   Risk: The closure amount stored in the dump is the raw value from the scorer, which may differ from what the Pinocchio FK expects as `q_active`.
   Mitigation: The closure amount comes directly from `score_grasp()` → `refine_binary()` → `lo` control value, which is in [0, 1] normalized space. This maps directly to `q_active` as `[closure, closure, closure]` for cylindrical. For pinch/lateral, the MRL component is 0.0 (locked open), matching the grasp spec.

4. **Reordering max_iteration computation**
   Risk: Moving the `max_iteration` computation earlier could break if it depends on variables not yet initialized.
   Mitigation: `max_iteration` only depends on `grasps["smc_iteration"]` which is loaded at the top of `visualize_pyvista()`. It's safe to move before `state`.

5. **Default iteration filter with no grasps**
   Risk: If the dump has 0 grasps, `max_iteration` would be 0, and filtering to iteration 0 would show nothing.
   Mitigation: Add a guard: if `n_grasps == 0`, keep `iteration_filter = None`.

## Alternative Approaches

1. **Load LUT file directly in viewer**: Instead of using Pinocchio FK, load `data/finger_contact_lut.npz` and implement DQ-to-position conversion in Python. This would be more faithful to the actual scoring pipeline but adds complexity (DQ math in Python, file loading, resolution handling). The Pinocchio FK approach is simpler and produces identical results.

2. **Export contact positions from Rust**: Modify `debug_export.rs` to include per-grasp contact point positions in the dump. This would avoid needing Pinocchio in the viewer entirely but would significantly increase dump file size (25 contacts × 3 coords × N grasps) and require changes to both Rust and Python sides. Not recommended for a visualization-only feature.

3. **Contact points only for best grasp**: Instead of rendering contacts for all top-10 grasps, only show them for the single best grasp. Simpler but less useful for comparing grasps. The batched rendering approach makes the full version performant enough.
