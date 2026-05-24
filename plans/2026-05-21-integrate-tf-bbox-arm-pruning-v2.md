# Integrate TF-Based Bounding Boxes for Arm Pruning in Pointcloud Fusion

## Objective

Use `camera_mounts.yaml` as the single source of truth for bounding box coordinates. The file currently has one workspace bounding box; we need to add an arm pruning box alongside it. The pointcloud fusion node must be modified to load both boxes from this YAML file and prune points inside **either** box (two-box pruning). The coordinates come from the `work/cleaned_full_test_20260515` branch where the existing `bounding_box` was defined.

## Key Finding: The Branch Has ONE Box, We Need TWO

The `camera_mounts.yaml` on `work/cleaned_full_test_20260515` (commit `8cd787b`) has a single `bounding_box` section:

```yaml
bounding_box:                           # Workspace box (where objects are)
  palm_to_corner:     { x: 0.11, y: 0.205, z: 0.11 }
  corner_to_opposite: { x: -0.22, y: -0.34, z: -0.32 }
```

This is already present on the current branch (identical file). For arm pruning we need a **second** box — the arm/hand removal box. The plan adds this to the same YAML file so all geometry lives in one place.

## Files Involved

| File | Role |
|------|------|
| `src/sensor_fusion_bringup/config/camera_mounts.yaml` | Source of truth for all bounding box geometry |
| `src/sensor_fusion_bringup/scripts/publish_camera_mounts.py` | Publishes TFs + markers for visualization |
| `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` | Consumer — loads boxes, prunes points |
| `config/prosthesis_config.yaml` | Fallback params (lines 132-145) |
| `src/prosthesis_launch/launch/pipeline.launch.py` | Launch wiring |

## Implementation Plan

### Phase 1: Add arm pruning box to `camera_mounts.yaml`

- [ ] **1.1** Add an `arm_pruning_box` section to `src/sensor_fusion_bringup/config/camera_mounts.yaml` after the existing `bounding_box` section (after line 59). Use the same `palm_to_corner` / `corner_to_opposite` schema. Initial values derived from the current hardcoded box in `prosthesis_config.yaml:140-141` (`bbox_min: [-0.30, -0.10, -0.10]`, `bbox_max: [0.22, 0.10, 0.12]`), but expressed in `palm_frame` coordinates. The section should look like:

```yaml
arm_pruning_box:
  description: "Arm/hand removal box — points inside this AABB in palm_frame are pruned from the fused cloud"
  palm_to_corner:
    translation:
      x: ...
      y: ...
      z: ...
    rotation: identity
    quaternion: [0.0, 0.0, 0.0, 1.0]
  corner_to_opposite:
    translation:
      x: ...
      y: ...
      z: ...
    rotation: identity
    quaternion: [0.0, 0.0, 0.0, 1.0]
```

The exact values need to be determined by measuring the arm/hand geometry in `palm_frame`, or by transforming the current `arm_frame` box into `palm_frame` using the known mount transform.

- [ ] **1.2** Update the header comment in `camera_mounts.yaml` (lines 1-23) to document both boxes: `bounding_box` = workspace (objects of interest), `arm_pruning_box` = arm/hand removal (points to discard).

### Phase 2: Update `publish_camera_mounts.py` to publish both boxes

- [ ] **2.1** In `_build_bounding_box_tfs()` (line 201), also publish TF frames for the arm pruning box: `palm_frame -> bb_pruning_corner -> bb_pruning_opposite`. Read from `data["arm_pruning_box"]` using the same schema as the workspace box.

- [ ] **2.2** In `__init__` (around line 165), build a second CUBE marker for the arm pruning box using `_make_bbox_marker()` but reading from `data["arm_pruning_box"]`. Use a different color (semi-transparent red, `r=1.0, g=0.3, b=0.3, a=0.25`) and namespace `"arm_pruning"` to distinguish it from the green workspace box.

- [ ] **2.3** Update `_republish_marker()` to republish both markers (workspace + arm pruning) every second.

- [ ] **2.4** Update the docstring TF tree diagram (lines 7-11) to show both boxes.

### Phase 3: Modify `pointcloud_fusion_node.py` to load and prune two boxes

- [ ] **3.1** Add a new parameter `mounts_config_path` (default: `""`). When non-empty, load `camera_mounts.yaml` at startup.

- [ ] **3.2** Add a new parameter `pruning_frame` (default: `"palm_frame"`). This is the frame in which the pruning boxes are defined. Separated from `arm_frame` which is used for the distance filter only.

- [ ] **3.3** Add a helper method `_load_pruning_boxes_from_yaml(config_path)` that:
  - Reads `camera_mounts.yaml`
  - Extracts both `bounding_box` and `arm_pruning_box` sections
  - For each box, computes `bbox_min` and `bbox_max` from `palm_to_corner + corner_to_opposite` (element-wise min/max of corner and corner+offset)
  - Returns a list of `(bbox_min, bbox_max)` numpy array pairs
  - Falls back to the ROS param `bbox_min`/`bbox_max` if the YAML is unavailable (backward compatibility)

- [ ] **3.4** In `__init__`, after reading parameters, call `_load_pruning_boxes_from_yaml()` to populate `self._pruning_boxes` (a list of `(min, max)` tuples). Store `self._pruning_frame`. Log the loaded box dimensions.

- [ ] **3.5** Update step 4 (hand/arm bbox removal, lines 441-458) to:
  - Transform points to `self._pruning_frame` (instead of `self._arm_frame`)
  - Loop over all boxes in `self._pruning_boxes`
  - For each box, compute the "keep" mask (points outside the box)
  - Intersect all keep masks (a point is kept only if it's outside ALL pruning boxes)
  - Apply the final combined mask

- [ ] **3.6** Update `_publish_bbox_marker()` (lines 504-536) to publish a marker for **each** pruning box in `self._pruning_boxes`, using `self._pruning_frame` as the header frame. Use distinct marker IDs (0, 1, ...) so both are visible.

- [ ] **3.7** Update the startup log message (lines 300-303) to report the number of pruning boxes loaded, their dimensions, and the pruning frame.

### Phase 4: Update configuration and launch

- [ ] **4.1** In `config/prosthesis_config.yaml`, add `pruning_frame: "palm_frame"` and `mounts_config_path: ""` to the `pointcloud_fusion` section (lines 132-145). Keep `bbox_min`/`bbox_max` as fallback defaults with a comment noting they are overridden when `mounts_config_path` is set.

- [ ] **4.2** In `src/prosthesis_launch/launch/pipeline.launch.py`, pass `mounts_config_path` to the fusion node. Add a `DeclareLaunchArgument` for it with a default pointing to the installed `camera_mounts.yaml` path (or empty string for fallback).

### Phase 5: Transfer from branch / worktree setup

Since the `camera_mounts.yaml` on `work/cleaned_full_test_20260515` is identical to the current branch, no file transfer is needed. The existing `bounding_box` values are already correct. The new `arm_pruning_box` needs to be **added** with appropriate values.

- [ ] **5.1** Create a git worktree for `work/cleaned_full_test_20260515` to double-check there are no uncommitted or stashed changes to `camera_mounts.yaml` that contain a second box. Command: `git worktree add /tmp/cleaned_full_test_worktree work/cleaned_full_test_20260515`. Compare the file: `diff src/sensor_fusion_bringup/config/camera_mounts.yaml /tmp/cleaned_full_test_worktree/src/sensor_fusion_bringup/config/camera_mounts.yaml`. If identical (expected), clean up: `git worktree remove /tmp/cleaned_full_test_worktree`.

- [ ] **5.2** Also check the `remotes/origin/full_test_implementation` branch for any variant of `camera_mounts.yaml` that might have two boxes. Command from worktree: `git show remotes/origin/full_test_implementation:src/sensor_fusion_bringup/config/camera_mounts.yaml`. If a second box exists there, extract those coordinates.

- [ ] **5.3** If no second box is found on any branch, determine the arm pruning box values by:
  - Using the current hardcoded values from `prosthesis_config.yaml:140-141` as a starting point
  - Transforming them from `arm_d435i_arm_depth_frame` to `palm_frame` using the mount transform from `camera_mounts.yaml` (the `screw_to_palm` of the active mount, e.g., `10_cm_cam_mount`)
  - Or measuring directly from the CAD model / physical setup

### Phase 6: Verify

- [ ] **6.1** Run `make -f Makefile.rviz mounts-viz` and verify both boxes appear in RViz: green (workspace) and red (arm pruning), correctly positioned relative to `palm_frame`.

- [ ] **6.2** Launch the full pipeline and verify the fusion node logs: number of pruning boxes loaded, dimensions, pruning frame. Confirm the red markers align with the physical arm/hand position.

- [ ] **6.3** Verify two-box pruning works: points inside either box are removed from the fused cloud. Test with `enable_hand_removal: true`.

- [ ] **6.4** Verify backward compatibility: with `mounts_config_path: ""`, the node falls back to the single hardcoded `bbox_min`/`bbox_max` from ROS params.

- [ ] **6.5** Verify the distance filter (step 3) still works correctly using `arm_frame` — it should be unaffected by the pruning frame change.

## Verification Criteria

- `camera_mounts.yaml` contains both `bounding_box` and `arm_pruning_box` sections
- `publish_camera_mounts.py` publishes TF frames and markers for both boxes
- The fusion node loads both boxes from the YAML and prunes points inside either one
- Both boxes are visible in RViz with distinct colors (green workspace, red arm pruning)
- Backward compatibility is preserved (empty `mounts_config_path` falls back to ROS params)
- The distance filter continues to work independently in `arm_frame`

## Potential Risks and Mitigations

1. **TF chain `marker_map -> palm_frame` may not exist at runtime**
   - `publish_camera_mounts.py` publishes `world -> palm_frame`, but the fusion node uses `marker_map`. If these are different frames, the TF lookup will fail.
   - Mitigation: The node already handles TF absence gracefully (warns and skips). Ensure the launch file or `publish_camera_mounts.py` connects `palm_frame` under the correct root frame. Consider changing `publish_camera_mounts.py` to accept a configurable parent frame.

2. **Arm pruning box values in `palm_frame` are unknown**
   - The current hardcoded values are in `arm_d435i_arm_depth_frame`, not `palm_frame`. Converting requires knowing the full transform chain between these frames.
   - Mitigation: Start with approximate values. The RViz visualization makes iterative tuning straightforward. The box can also be defined directly in `arm_frame` by setting `pruning_frame: arm_d435i_arm_depth_frame` and expressing the box corners in that frame.

3. **Two-box pruning adds per-cycle overhead**
   - Each box requires a point-to-frame transform + AABB check. With two boxes this doubles the bbox step cost.
   - Mitigation: The AABB check is vectorized numpy (`np.all`), so the overhead is negligible compared to the TF-transformed cloud concatenation. If needed, both boxes can be checked in a single pass by computing a union-of-inside mask.

## Alternative Approaches

1. **Define both boxes in `arm_frame` instead of `palm_frame`**: Avoids the frame conversion issue but means the boxes move with the camera, not the hand. Less intuitive for tuning.

2. **Use TF frame lookups at runtime instead of loading YAML**: Look up `bb_corner`/`bb_opposite` and `bb_pruning_corner`/`bb_pruning_opposite` frame positions each cycle. Most "TF-native" but adds per-cycle overhead for static data.

3. **Single combined box (union of both)**: Merge both boxes into one larger AABB. Simpler but less precise — would prune too much or too little depending on geometry.
