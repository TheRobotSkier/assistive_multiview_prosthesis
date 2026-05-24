# Improve Hand Contact Visualization

## Objective

Refactor the contact-point visualization in `visualize_grasp_debug.py` with three changes:
1. Contacts start **off** by default (not active on launch)
2. Draw **lines between contacts** within each finger group to show kinematic chain relationships
3. Contacts are **only shown when the hand is visible** (`h` pressed first); pressing `k` replaces the simple skeleton with the detailed contact model

## Files to Modify

- `scripts/visualize_grasp_debug.py` (all changes in this file)

## Implementation Plan

### Task 1. Change default state: contacts start off

**File:** `scripts/visualize_grasp_debug.py:479`

Change `"show_contacts": True` to `"show_contacts": False` in the `state` dict.

```python
# Line 479: change from
"show_contacts": True,
# to
"show_contacts": False,
```

### Task 2. Add per-group contact line topology constant

**File:** `scripts/visualize_grasp_debug.py` — insert after the `_CONTACT_FINGER_COLORS` dict (after line 105), before the pinocchio import block.

Add a new constant `_CONTACT_LINES` that defines which contacts should be connected by lines within each finger group. The ordering follows the kinematic chain (proximal → distal):

```python
# Per-group contact line topology.  Each list defines the chain(s) of
# contact names that should be connected by lines within a finger group.
# The ordering follows the kinematic chain (proximal -> distal).
_CONTACT_LINES = {
    "index": [
        ["IndexMcp", "IndexDip", "IndexPip", "IndexTip"],       # palmar rail
        ["IndexMcpSide", "IndexDipSide", "IndexPipSide", "IndexTipSide"],  # lateral rail
        ["IndexMcp", "IndexMcpSide"],      # cross-links at each joint
        ["IndexDip", "IndexDipSide"],
        ["IndexPip", "IndexPipSide"],
        ["IndexTip", "IndexTipSide"],
    ],
    "middle": [
        ["MiddleMcp", "MiddleDip", "MiddlePip", "MiddleTip"],
    ],
    "ring": [
        ["RingDip", "RingPip", "RingTip"],
    ],
    "little": [
        ["LittleDip", "LittlePip", "LittleTip"],
    ],
    "thumb": [
        ["ThumbAddDip", "ThumbAddPip", "ThumbAddTip"],
    ],
    "palm": [
        ["PalmProxUlna", "PalmDistUlna"],     # ulnar rail
        ["PalmProxRadi", "PalmDistRadi"],     # radial rail
        ["PalmProxUlna", "PalmProxRadi"],     # cross-links
        ["PalmDistUlna", "PalmDistRadi"],
    ],
}
```

**Rationale:** Index finger has 8 contacts (4 palmar + 4 lateral), so it gets two rails plus cross-links. Other fingers have single chains. Palm gets paired ulnar/radial rails plus cross-links.

### Task 3. Rewrite `add_contact_points()` to draw lines and require hand visibility

**File:** `scripts/visualize_grasp_debug.py` — replace the entire `add_contact_points()` function (lines 1107–1202) and its initial call (lines 1204–1205).

The new function must:
- Return early if `state["show_hand"]` is False (contacts only render when hand is visible)
- Collect per-contact world positions into a dict (contact_name → world xyz)
- Render contact points as before (colored by group, score contacts brighter)
- After rendering points, iterate `_CONTACT_LINES` to draw lines connecting contacts within each group
- Use per-grasp rendering (one point cloud + one line set per grasp) so each grasp gets its own actors

Replace lines 1107–1205 with:

```python
    def add_contact_points():
        """Render all 25 LUT contact points for the top grasps.

        Uses Pinocchio FK via get_sampled_contact_transforms() to compute
        contact positions in hand-local frame, then transforms to world frame
        using the grasp's 4x4 pose matrix. Score contacts are rendered larger
        than sweep-only contacts.  Lines connect contacts within each finger
        group to visualise the kinematic chain.

        Only rendered when the hand is visible (show_hand).  When active, this
        replaces the simple hand skeleton with the detailed contact model.
        """
        actor_groups["contact_points"].clear()
        if not _pin_hand_available:
            return
        if not state["show_hand"]:
            return

        all_indices, top_indices = get_grasp_indices()
        if not top_indices:
            return

        # Build a contact-name -> group lookup from model.py CONTACT_DEFINITIONS.
        contact_group_map = {c["name"]: c["group"] for c in CONTACT_DEFINITIONS}

        # Use iteration-filtered best when filter is active.
        display_best = best_idx if state["iteration_filter"] is None else _filtered_best()[0]

        for idx in top_indices:
            if grasps["contact_score"][idx] <= 0.0:
                continue

            gt = grasps["grasp_type"][idx]
            T = grasps["pose_4x4"][idx]
            closure = grasps["closure"][idx]
            R, t = T[:3, :3], T[:3, 3]
            is_best = (idx == display_best)

            q_active, thumb_mode = _grasp_to_fk_params(gt, closure)
            try:
                transforms = get_sampled_contact_transforms(q_active, thumb_opp_mode=thumb_mode)
            except Exception:
                continue

            score_contacts = _GRASP_SCORE_CONTACTS.get(gt, set())

            # ---- Collect per-contact world positions ----
            world_positions = {}  # contact_name -> world xyz
            for contact_name, contact_T in transforms.items():
                local_pos = contact_T[:3, 3]
                world_positions[contact_name] = R @ local_pos + t

            # ---- Render contact points ----
            points = []
            point_colors = []
            for contact_name in world_positions:
                points.append(world_positions[contact_name])
                group = contact_group_map.get(contact_name, "index")
                hex_color = _CONTACT_FINGER_COLORS.get(group, "#ffffff")
                r_c = int(hex_color[1:3], 16) / 255.0
                g_c = int(hex_color[3:5], 16) / 255.0
                b_c = int(hex_color[5:7], 16) / 255.0
                is_score = contact_name in score_contacts
                if is_score:
                    point_colors.append([r_c, g_c, b_c])
                else:
                    point_colors.append([r_c * 0.5, g_c * 0.5, b_c * 0.5])

            if not points:
                continue

            pt_size = 10 if is_best else 6
            pts = pv.PolyData(np.asarray(points, dtype=np.float64))
            pts["colors"] = np.asarray(point_colors, dtype=np.float32)
            actor = plotter.add_mesh(
                pts,
                scalars="colors",
                rgb=True,
                style="points",
                point_size=pt_size,
                render_points_as_spheres=True,
                opacity=0.85 if is_best else 0.55,
                label=f"Contacts ({len(points)} pts)" if is_best else None,
            )
            actor_groups["contact_points"].append(actor)

            # ---- Render per-group lines ----
            line_points = []
            line_cells = []
            for group, chains in _CONTACT_LINES.items():
                for chain in chains:
                    chain_positions = []
                    for cname in chain:
                        if cname in world_positions:
                            chain_positions.append(world_positions[cname])
                    if len(chain_positions) < 2:
                        continue
                    start_idx = len(line_points)
                    line_points.extend(chain_positions)
                    for j in range(len(chain_positions) - 1):
                        line_cells.append([2, start_idx + j, start_idx + j + 1])

            if line_points:
                lp = pv.PolyData(np.asarray(line_points, dtype=np.float64))
                lp.lines = np.hstack(line_cells)
                actor = plotter.add_mesh(
                    lp,
                    color="#c9d1d9",
                    line_width=3 if is_best else 1,
                    opacity=0.6 if is_best else 0.3,
                )
                actor_groups["contact_points"].append(actor)

    if state["show_contacts"] and state["show_hand"]:
        add_contact_points()
```

### Task 4. Update `add_hand_skeletons()` to suppress when contacts are active

**File:** `scripts/visualize_grasp_debug.py` — modify `add_hand_skeletons()` (lines 1045–1083).

Add an early return when `state["show_contacts"]` is True, so the simple skeleton is suppressed in favor of the detailed contact model:

```python
    def add_hand_skeletons():
        """Render hand skeletons based on current grasp mode.

        When contact mode (k) is active, the simple skeleton is suppressed and
        the detailed contact-point model is rendered instead by add_contact_points().
        """
        actor_groups["hand_skeleton"].clear()
        if not _pin_hand_available:
            return

        # If contact detail mode is active, skip the simple skeleton;
        # add_contact_points() will render the detailed model instead.
        if state["show_contacts"]:
            return

        # ... rest of the existing function body unchanged ...
```

### Task 5. Update `on_key_h()` to manage contact visibility

**File:** `scripts/visualize_grasp_debug.py` — replace the `on_key_h()` function (lines 1350–1363).

When turning the hand off, also hide contacts. When turning the hand on with contacts active, ensure contacts are visible:

```python
    def on_key_h():
        """Toggle hand skeleton(s)."""
        state["show_hand"] = not state["show_hand"]
        if state["show_hand"]:
            if not actor_groups["hand_skeleton"] and not state["show_contacts"]:
                add_hand_skeletons()
            elif not actor_groups["hand_skeleton"]:
                # Contact mode active — skeleton suppressed, contacts will render.
                pass
            else:
                for actor in actor_groups["hand_skeleton"]:
                    actor.SetVisibility(True)
            # Also ensure contact points are shown if k is active.
            if state["show_contacts"]:
                if not actor_groups["contact_points"]:
                    add_contact_points()
                else:
                    for actor in actor_groups["contact_points"]:
                        actor.SetVisibility(True)
        else:
            for actor in actor_groups["hand_skeleton"]:
                actor.SetVisibility(False)
            # Hide contacts too — they depend on the hand being visible.
            for actor in actor_groups["contact_points"]:
                actor.SetVisibility(False)
        update_info_text()
        plotter.render()
```

### Task 6. Update `on_key_k()` to require hand visibility and swap models

**File:** `scripts/visualize_grasp_debug.py` — replace the `on_key_k()` function (lines 1365–1378).

When pressing `k` to enable contacts: require hand to be visible, hide the simple skeleton, show contact model. When pressing `k` to disable contacts: restore the simple skeleton if hand is visible.

```python
    def on_key_k():
        """Toggle LUT contact points (requires hand to be visible)."""
        state["show_contacts"] = not state["show_contacts"]
        if state["show_contacts"] and state["show_hand"]:
            # Replace simple skeleton with detailed contact model.
            for actor in actor_groups["hand_skeleton"]:
                actor.SetVisibility(False)
            if not actor_groups["contact_points"]:
                add_contact_points()
            else:
                for actor in actor_groups["contact_points"]:
                    actor.SetVisibility(True)
        elif state["show_contacts"] and not state["show_hand"]:
            print("  Contacts require hand visible — press h first")
            state["show_contacts"] = False
        else:
            # Turning contacts off — restore simple skeleton if hand is visible.
            for actor in actor_groups["contact_points"]:
                actor.SetVisibility(False)
            if state["show_hand"]:
                if not actor_groups["hand_skeleton"]:
                    add_hand_skeletons()
                else:
                    for actor in actor_groups["hand_skeleton"]:
                        actor.SetVisibility(True)
        update_info_text()
        plotter.render()
```

### Task 7. Update `rebuild_grasps()` contact condition

**File:** `scripts/visualize_grasp_debug.py` — modify the contact rebuild condition in `rebuild_grasps()` (line 1311).

Change the condition to also check `state["show_hand"]`:

```python
        # Line ~1311: change from
        if state["show_contacts"]:
            add_contact_points()
        # to
        if state["show_contacts"] and state["show_hand"]:
            add_contact_points()
```

## Verification Criteria

- [ ] Launching the viewer shows contacts OFF by default (no contact points visible)
- [ ] Pressing `h` shows the simple hand skeleton (base + tip per finger)
- [ ] Pressing `k` without `h` first prints a warning and does nothing
- [ ] Pressing `h` then `k` hides the simple skeleton and shows detailed contact points with per-finger-group lines
- [ ] Pressing `k` again (with hand still visible) restores the simple skeleton and hides contacts
- [ ] Pressing `h` to hide the hand also hides any visible contacts
- [ ] Lines connect contacts within each finger: index has palmar + lateral rails with cross-links, other fingers have single chains, palm has ulnar/radial rails
- [ ] Switching grasp mode (`g`), iteration filter (`+`/`-`/`*`), or grasp type filter (`0-3`) correctly rebuilds both hand skeleton and contact models

## Potential Risks and Mitigations

1. **Performance with many grasps in "all" mode**
   Mitigation: Contact rendering is per-grasp in the top_indices list (max 10), same as the existing hand skeleton. Lines add minimal overhead since they use a single PolyData with multiple line cells.

2. **Stale actor references after rebuild**
   Mitigation: All toggle handlers check for empty actor_groups before calling SetVisibility, and rebuild_grasps properly clears and re-creates actors.

3. **State inconsistency between show_hand and show_contacts**
   Mitigation: The on_key_k handler explicitly checks both states and provides user feedback when contacts require the hand. The on_key_h handler manages contact visibility as a dependent.
