# Hand Model Simplification + Expanded Score Range

## Objective

Replace the single hand skeleton model with two levels of detail, and expand the grasp score point size range for clearer visual discrimination.

---

## Part A — Hand Skeleton Models

### Available URDF Frames (verified)

From Pinocchio's frame table:

| Frame | Name | Type |
|-------|------|------|
| 3 | `mia_palm` | BODY |
| 5 | `mia_index_fle` | BODY |
| 7 | `mia_index_sensor` | BODY |
| 11 | `mia_middle_fle` | BODY |
| 13 | `mia_middle_sensor` | BODY |
| 15 | `mia_ring_fle` | BODY |
| 17 | `mia_thumb_opp` | BODY |
| 19 | `mia_thumb_sensor` | BODY |
| 21 | `mia_thumb_fle` | BODY |

**Simple model** (3 fingers: thumb, index, middle — 1 joint + tip each, no ring/little):
```
palm → thumb_opp → thumb_fle → thumb_sensor (tip)
palm → index_fle → index_sensor (tip)
palm → middle_fle → middle_sensor (tip)
```
Total: 7 edges, 8 frame positions. Compact, fast to render.

**Full model** (all 5 fingers — 2 joints + tip per finger):
```
palm → thumb_opp → thumb_fle → thumb_sensor
palm → index_fle → index_sensor
palm → middle_fle → middle_sensor
palm → ring_fle
palm → little_fle
```
Total: 8 edges, 9 frame positions. More complete but busier.

### Behaviour

- When `state["grasp_mode"] == "best"`: render **full** model for the single best grasp
- When `state["grasp_mode"] == "all"`: render **simple** models for all visible grasps
- Pressing `h` toggles visibility of all hand skeletons (same as before)
- If `best_idx < 0` or no collision: no hands rendered

### Implementation details

- Define `_FRAME_EDGES_SIMPLE` and `_FRAME_EDGES_FULL` constants with `(parent, child, finger)` tuples
- Add `FINGER_COLORS_SIMPLE` dict mapping the 3 simple fingers to colors
- The frame ID pre-resolution is done once at startup using the **full** edge list (all frames needed by both models are covered)
- `_JOINT_TO_FINGER` renamed to `_FRAME_TO_FINGER` with entries for all 9 frame names

### New helper: `_compute_hand_positions(grasp_idx, edges)`

Extracts the common computation from `add_hand_skeleton()` into a reusable function:
```python
def _compute_hand_positions(grasp_idx, edges):
    """Run FK and return dict[fid] -> world-space np.array of frame positions."""
    gt = grasps["grasp_type"][grasp_idx]
    T = grasps["pose_4x4"][grasp_idx]
    closure = grasps["closure"][grasp_idx]
    thumb_mode = 0.0 if gt == 3 else 1.0
    q_active = np.array([closure, closure, closure], dtype=float)
    q_full = _q_full_with_thumb_mode(q_active, thumb_mode)
    pin.forwardKinematics(pin_model, pin_data, q_full)
    pin.updateFramePlacements(pin_model, pin_data)

    positions = {}
    for pid, cid, _ in edges:
        if pid not in positions:
            positions[pid] = pin_data.oMf[pid].translation.copy().astype(np.float64)
        if cid not in positions:
            positions[cid] = pin_data.oMf[cid].translation.copy().astype(np.float64)

    # Transform to world frame via grasp pose
    R, t = T[:3, :3], T[:3, 3]
    for fid in positions:
        positions[fid] = R @ positions[fid] + t

    return positions
```

### New: `add_hand_skeletons()`

Replaces the old `add_hand_skeleton()`:
- If `grasp_mode == "best"`: call `_add_single_hand(best_idx, _FRAME_EDGES_FULL, opacity=0.9, line_width=4, sphere_radius=scene_extent*0.015)`
- If `grasp_mode == "all"`: loop over all visible grasps, call `_add_single_hand(i, _FRAME_EDGES_SIMPLE, opacity=0.6, line_width=2, sphere_radius=scene_extent*0.010)` per grasp

Where `_add_single_hand()` renders one hand skeleton given precomputed frame positions and style overrides.

---

## Part B — Grasp Score Point Sizes

### Current formula (line 637)
```python
point_sizes.append(6 + 10 * score_norm)   # range: 6–16
```
Best grasp always 20 (gold). Range is too compressed to easily distinguish scores.

### New formula
```python
# Non-best grasps: expand range 4–18
point_sizes.append(4 + 14 * score_norm)
```

### Z-axis indicator length (line 625)

**Current:**
```python
z_lens.append(marker_size * (1.2 if is_best else 0.4 + 0.6 * score_norm))
# Best: 1.2*marker_size, others: 0.4–1.0*marker_size
```

**New:**
```python
z_lens.append(marker_size * (1.2 if is_best else 0.2 + 0.4 * score_norm))
# Best: 1.2*marker_size, others: 0.2–0.6*marker_size (smaller for low-score)
```

The Z-arrow for low-score grasps becomes shorter (0.2× vs current 0.4×), making it less prominent for weaker grasps. High-score ones still get meaningful arrows (0.6×).

---

## Implementation Plan

### Task 1: Define hand model edge constants (lines 48–80)

- [ ] **Task 1.1**: Replace `HAND_JOINT_EDGES` with two constants:
  ```python
  # Simple hand model: 3 fingers (thumb, index, middle), 1 joint + tip each
  _FRAME_EDGES_SIMPLE = [
      ("mia_palm", "mia_thumb_opp", "thumb"),
      ("mia_thumb_opp", "mia_thumb_fle", "thumb"),
      ("mia_thumb_fle", "mia_thumb_sensor", "thumb"),
      ("mia_palm", "mia_index_fle", "index"),
      ("mia_index_fle", "mia_index_sensor", "index"),
      ("mia_palm", "mia_middle_fle", "middle"),
      ("mia_middle_fle", "mia_middle_sensor", "middle"),
  ]
  
  # Full hand model: all 5 fingers, 2 joints + tip where available
  _FRAME_EDGES_FULL = [
      ("mia_palm", "mia_thumb_opp", "thumb"),
      ("mia_thumb_opp", "mia_thumb_fle", "thumb"),
      ("mia_thumb_fle", "mia_thumb_sensor", "thumb"),
      ("mia_palm", "mia_index_fle", "index"),
      ("mia_index_fle", "mia_index_sensor", "index"),
      ("mia_palm", "mia_middle_fle", "middle"),
      ("mia_middle_fle", "mia_middle_sensor", "middle"),
      ("mia_palm", "mia_ring_fle", "ring"),
      ("mia_palm", "mia_little_fle", "little"),
  ]
  ```

- [ ] **Task 1.2**: Rename `_JOINT_TO_FINGER` → `_FRAME_TO_FINGER` and update it with all 9 frame names. Remove ring/little from `_FRAME_TO_FINGER` entries (they exist in full model but have no child entries).

- [ ] **Task 1.3**: Keep `FINGER_COLORS` as-is (thumb, index, middle, ring, little — all still used by full model). Create `FINGER_COLORS_SIMPLE` for the 3 fingers in simple model.

### Task 2: Refactor hand skeleton rendering

- [ ] **Task 2.1**: Update the pre-resolve block at line 677 to use `_FRAME_EDGES_FULL` (since it's the superset of all frames needed). Store `_frame_edges_resolved` from the full list.

- [ ] **Task 2.2**: Add `_compute_hand_positions(grasp_idx, edges)` helper function. It runs FK + updateFramePlacements and returns a dict mapping frame IDs to world-space positions. Placed right before the `add_hand_skeletons()` function definition.

- [ ] **Task 2.3**: Add `_add_single_hand(frame_positions, edges, opacity, line_width, sphere_radius)` helper. Takes precomputed positions and renders lines + spheres using the specified style parameters.

- [ ] **Task 2.4**: Replace `add_hand_skeleton()` with `add_hand_skeletons()`. The function reads `state["grasp_mode"]`:
  - **"best"**: iterate over all grasps, find the single best (`i == best_idx`), render full model with `opacity=0.9, line_width=4, sphere_radius=scene_extent*0.015`
  - **"all"**: loop over all `get_grasp_indices()`, render simple model with `opacity=0.6, line_width=2, sphere_radius=scene_extent*0.010`
  - If no grasps or `_pin_hand_available=False`, do nothing and return
  - Also skip any grasp where `grasps["found_collision"][i]` is False

- [ ] **Task 2.5**: Update all call sites of `add_hand_skeleton()`:
  - Line 673: replace `add_grasp_actors()` → `add_grasp_actors()` (no change needed)
  - Line 891 (`on_key_h`): replace `add_hand_skeleton()` → `add_hand_skeletons()`
  - Remove the old `add_hand_skeleton()` function entirely

- [ ] **Task 2.6**: Update info text to reflect which model is active: `"Hand: full (best)"` or `"Hand: simple (N)"` where N is the count of hand skeletons rendered.

### Task 3: Expand grasp score point size range (lines 625–637)

- [ ] **Task 3.1**: Change point size formula from `6 + 10 * score_norm` to `4 + 14 * score_norm`
- [ ] **Task 3.2**: Change Z-axis arrow length from `0.4 + 0.6 * score_norm` to `0.2 + 0.4 * score_norm` for non-best grasps

### Task 4: Update legend and help text

- [ ] **Task 4.1**: Update legend entry from `"Hand pose"` to `"Hand skeleton"` or split into two entries
- [ ] **Task 4.2**: Update `on_key_help()` to reflect that `h` now shows hand skeletons for all visible grasps

---

## Verification Criteria

- [ ] Pressing `g` to switch from "best" → "all" causes hand skeletons to switch from full to simple model
- [ ] In "all" mode, 3-finger simple hands are rendered for each grasp — visually distinct from the full model
- [ ] In "best" mode, 5-finger full hands are rendered for the best grasp
- [ ] Low-score grasp points are noticeably smaller than high-score ones (range 4–18 vs previous 6–16)
- [ ] Low-score Z-axis arrows are shorter than before (max 0.6× marker_size vs previous 1.0×)
- [ ] No duplicate hand skeletons accumulate when toggling `h` multiple times
- [ ] Script runs without errors or warnings

## Potential Risks

1. **Performance with many grasps in "all" mode**: Each hand skeleton creates actors. With 100 grasps in simple mode: 100×7 lines + 100×8 spheres = ~1500 actors. Manageable but monitor.
   **Mitigation**: Simple model intentionally minimizes geometry. "all" mode uses simple model only.

2. **Frame ID pre-resolution assumes full model includes all frames**: The pre-resolve uses `_FRAME_EDGES_FULL` which is a superset of `_FRAME_EDGES_SIMPLE`, so all frame IDs needed by both models are resolved once.
