# Fix Hand Skeleton Rendering

## Objective

The hand skeleton rendering silently fails because `HAND_JOINT_EDGES` uses **link names** (e.g., `mia_palm`) but the code calls `pin_model.getJointId()` which expects **joint names** (e.g., `mia_j_palm`). Furthermore, Pinocchio absorbs fixed joints, so even with correct joint names, fixed joints like `mia_j_palm` won't have entries in `data.oMi`. The fix requires switching from joint-based to frame-based position lookups.

## Root Cause Analysis

### Bug: Wrong name type + wrong lookup API

1. **`HAND_JOINT_EDGES`** at `visualize_grasp_debug.py:50-59` uses link names: `"mia_palm"`, `"mia_thumb_opp"`, etc.
2. **`getJointId()`** at `visualize_grasp_debug.py:682-683` expects joint names: `"mia_j_palm"`, `"mia_j_thumb_opp"`, etc.
3. **Pinocchio absorbs fixed joints** — `mia_j_palm` (fixed), `mia_j_index_sensor` (fixed), `mia_j_middle_sensor` (fixed) are merged into parents and don't appear in `model.joints` or `data.oMi`.
4. **But frames are preserved** — Pinocchio creates frames for all links (including those connected via fixed joints), accessible via `model.getFrameId()` and `data.oMf[]`.

### Correct approach: Use frames, not joints

- `pin_model.getFrameId("mia_palm")` → valid frame ID
- `pin.updateFramePlacements(pin_model, pin_data)` → populates `pin_data.oMf`
- `pin_data.oMf[frame_id].translation` → world-space position of that link frame
- This works for ALL links, including those connected via fixed joints

### Additional issue: Sphere size too small

- Current: `scene_extent * 0.008` — for a 0.3m scene, that's 0.0024m radius
- The hand is only ~0.1m across, so joints 0.05m apart get spheres barely visible
- Fix: Use `scene_extent * 0.015` (roughly 2x larger)

## Implementation Plan

### Phase 1: Switch from joint-based to frame-based lookups

- [ ] **Task 1.1**: In the pre-resolve block (`visualize_grasp_debug.py:678-686`), replace `getJointId()` with `getFrameId()`. Store resolved frame IDs instead of joint IDs. Validate that each frame ID < `pin_model.nframes`.

- [ ] **Task 1.2**: In `add_hand_skeleton()` (`visualize_grasp_debug.py:692-755`):
  - After `pin.forwardKinematics()`, call `pin.updateFramePlacements(pin_model, pin_data)` to populate frame transforms.
  - Replace `pin_data.oMi[jid].translation` with `pin_data.oMf[fid].translation` using the resolved frame IDs.
  - Remove the loop over `range(pin_model.njoints)` — instead, collect positions only for the frames in `_frame_edges_resolved`.

- [ ] **Task 1.3**: Increase sphere radius from `scene_extent * 0.008` to `scene_extent * 0.015` for both parent and child joint spheres.

- [ ] **Task 1.4**: Rename `_joint_edges_resolved` to `_frame_edges_resolved` for clarity. Update all references.

- [ ] **Task 1.5**: Add a diagnostic print showing how many frame edges were resolved, e.g., `"  Hand skeleton: pinocchio + URDF loaded (9 frame edges)"` to make future debugging easier.

## Verification Criteria

- [ ] Script prints `"Hand skeleton: pinocchio + URDF loaded (N frame edges)"` with N >= 8
- [ ] Pressing `h` key shows colored lines and spheres forming a recognizable hand shape at the best grasp position
- [ ] Joint spheres are clearly visible (not just dots)
- [ ] No exceptions or silent failures in the hand skeleton code path

## Potential Risks and Mitigations

1. **Frame names might differ from link names in URDF**
   Mitigation: Pinocchio creates frames with the link name by default. Verified by checking `model.py:177` which uses `geom_obj.parentJoint` (joint ID) successfully — the frame names should match link names.

2. **`updateFramePlacements` not called after `forwardKinematics`**
   Mitigation: This is explicitly part of the fix — add the call right after `forwardKinematics`.

3. **Some frame names in `HAND_JOINT_EDGES` might not exist**
   Mitigation: The `getFrameId()` call returns `model.nframes` for non-existent names. Check `fid < model.nframes` and skip invalid frames with a warning print.
