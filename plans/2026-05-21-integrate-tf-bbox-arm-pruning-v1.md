# Integrate TF-Based Bounding Box for Arm Pruning in Pointcloud Fusion

## Objective

Replace the hardcoded `bbox_min`/`bbox_max` arm pruning box in `pointcloud_fusion_node.py` with a TF-based approach that reads the bounding box geometry from `camera_mounts.yaml` at runtime. The pruning box will be expressed in `palm_frame` and looked up via TF2, making it configurable from a single source of truth and visually verifiable in RViz via the existing `publish_camera_mounts.py` markers.

## Current State

- **Arm pruning** in `pointcloud_fusion_node.py:211-212,441-458` uses hardcoded `bbox_min: [-0.30, -0.10, -0.10]`, `bbox_max: [0.22, 0.10, 0.12]` in `arm_frame` (default: `arm_d435i_arm_depth_frame`).
- **Bounding box definition** in `camera_mounts.yaml:37-59` defines a box in `palm_frame` via two corner transforms: `palm_to_corner` and `corner_to_opposite`.
- **TF publisher** `publish_camera_mounts.py` already publishes `palm_frame -> bb_corner -> bb_opposite` static TFs and a green CUBE marker.
- The config values are also duplicated in `prosthesis_config.yaml:140-141`.

## Design Decisions

1. **Pruning frame changes from `arm_d435i_arm_depth_frame` to `palm_frame`**. The bounding box in `camera_mounts.yaml` is already defined in `palm_frame`. This keeps all geometry in one coordinate system and lets `publish_camera_mounts.py` visualize the exact box used for pruning.
2. **The node loads the box from `camera_mounts.yaml` at startup** (not via ROS params for the box values). A new parameter `mounts_config_path` points to the YAML file. The `bbox_min`/`bbox_max` ROS params become a fallback if the YAML is not found.
3. **A new `arm_pruning_box` section is added to `camera_mounts.yaml`** alongside the existing `bounding_box`. The existing `bounding_box` is the workspace (where objects of interest are); the arm pruning box is the volume to remove (the arm/hand). These serve opposite purposes and need separate definitions.

## Implementation Plan

### Phase 1: Add arm pruning box to `camera_mounts.yaml`

- [ ] **1.1** Add an `arm_pruning_box` section to `src/sensor_fusion_bringup/config/camera_mounts.yaml` after the existing `bounding_box` section (after line 59). Use the same `palm_to_corner` / `corner_to_opposite` schema. Initial values should match the current hardcoded box but expressed in `palm_frame` coordinates. This requires computing the equivalent box in `palm_frame` from the current `arm_frame` values, or defining new values based on the physical arm/hand geometry. A reasonable starting point: the arm extends roughly along the -Y axis of `palm_frame` (toward the wrist), so the pruning box should cover that region.

### Phase 2: Modify `pointcloud_fusion_node.py` to load and use TF-based bbox

- [ ] **2.1** Add a new parameter `mounts_config_path` (default: empty string). When non-empty, load `camera_mounts.yaml` at startup and extract `arm_pruning_box` to compute `bbox_min`/`bbox_max` in `palm_frame`. Add a helper `_load_pruning_box_from_yaml()` method that reads the YAML, computes the min/max corners from `palm_to_corner + corner_to_opposite`, and returns `(bbox_min, bbox_max)` as numpy arrays. If the YAML load fails, fall back to the `bbox_min`/`bbox_max` ROS params (preserving backward compatibility).

- [ ] **2.2** Add a new parameter `pruning_frame` (default: `"palm_frame"`). This replaces the use of `arm_frame` for the bbox removal step. The `arm_frame` parameter continues to be used for the distance filter (step 3) only.

- [ ] **2.3** Update the `__init__` parameter declarations: add `mounts_config_path` and `pruning_frame`. After reading params, call `_load_pruning_box_from_yaml()` to populate `self._bbox_min` and `self._bbox_max`. Store `self._pruning_frame` from the new param.

- [ ] **2.4** Update step 4 (hand/arm bbox removal, lines 441-458) to use `self._pruning_frame` instead of `self._arm_frame` in the `_transform_points_to_frame()` call. The rest of the logic (`_bbox_filter`) remains unchanged since it is frame-agnostic.

- [ ] **2.5** Update `_publish_bbox_marker()` (lines 504-536) to use `self._pruning_frame` as the marker's `header.frame_id` instead of `self._arm_frame`. This ensures the visualization matches the actual pruning frame.

- [ ] **2.6** Update the startup log message (lines 300-303) to include `pruning_frame` and the loaded box dimensions.

### Phase 3: Update `publish_camera_mounts.py` to also publish the arm pruning box

- [ ] **3.1** Add a `_make_pruning_bbox_marker()` function similar to `_make_bbox_marker()` but reading from the `arm_pruning_box` section. Use a different color (semi-transparent red, matching the fusion node's existing marker color) and a different namespace (`"arm_pruning"` vs `"workspace"`).

- [ ] **3.2** In `CameraMountTFPublisher.__init__`, after publishing the workspace bbox marker, also build and publish the arm pruning bbox marker. Add TF frames `palm_frame -> bb_pruning_corner -> bb_pruning_opposite` so the pruning box is visible in the TF tree.

- [ ] **3.3** Republish the pruning marker in `_republish_marker()` alongside the workspace marker.

### Phase 4: Update configuration files

- [ ] **4.1** In `config/prosthesis_config.yaml`, update the `pointcloud_fusion` section: add `pruning_frame: "palm_frame"` and `mounts_config_path: ""` (empty = use ROS params as fallback). Keep `bbox_min`/`bbox_max` as fallback defaults but add a comment noting they are overridden when `mounts_config_path` is set.

- [ ] **4.2** In `src/prosthesis_launch/launch/pipeline.launch.py`, pass the `mounts_config_path` parameter to the fusion node. Consider adding a launch argument for it with a sensible default (e.g., the installed path to `camera_mounts.yaml`).

### Phase 5: Verify and test

- [ ] **5.1** Run `make mounts-viz` to verify both the workspace (green) and arm pruning (red) bounding boxes appear correctly in RViz, and that the TF frames `bb_pruning_corner`/`bb_pruning_opposite` are published.

- [ ] **5.2** Launch the full pipeline and verify that the fusion node logs the loaded pruning box dimensions and frame, and that the red bbox marker in RViz aligns with the actual arm/hand position.

- [ ] **5.3** Verify that when `mounts_config_path` is empty/unset, the node falls back to the hardcoded `bbox_min`/`bbox_max` params without errors (backward compatibility).

- [ ] **5.4** Check that the distance filter (step 3) still works correctly — it should continue using `arm_frame`, not `pruning_frame`.

## Verification Criteria

- The arm pruning box is loaded from `camera_mounts.yaml` at node startup
- Both workspace and pruning bounding boxes are visible in RViz with distinct colors
- The fusion node logs the pruning box dimensions and frame on startup
- Points inside the pruning box are correctly removed from the fused cloud
- Backward compatibility: setting `mounts_config_path: ""` falls back to ROS param values
- The distance filter continues to work independently in `arm_frame`

## Potential Risks and Mitigations

1. **TF chain `marker_map -> palm_frame` may not exist at runtime**
   - The `publish_camera_mounts.py` publishes `world -> palm_frame`, but the fusion node operates in `marker_map`. If `world != marker_map`, the TF lookup for `palm_frame` will fail.
   - Mitigation: The pruning frame lookup uses `rclpy.time.Time()` (latest available). If the TF chain is incomplete, the node already has a fallback path (logs a warning and skips the filter). Additionally, ensure `publish_camera_mounts.py` or the pipeline publishes the `palm_frame` under `marker_map` when running the real system.

2. **The initial `arm_pruning_box` values in `camera_mounts.yaml` may not match the physical arm geometry**
   - The current hardcoded values were tuned for `arm_d435i_arm_depth_frame`. Expressing an equivalent box in `palm_frame` requires knowing the transform between these frames, which may not be identity.
   - Mitigation: Start with the existing box values as a first approximation. The visual markers in RViz make it easy to tune. Add a comment in the YAML noting the values may need tuning.

3. **Race condition: fusion node starts before `publish_camera_mounts.py` publishes TFs**
   - The pruning TF lookup happens per-cloud-cycle, not just at startup. Once the TFs appear, pruning activates automatically. The node already handles TF absence gracefully with a warning.

## Alternative Approaches

1. **Keep pruning in `arm_frame`, transform box corners at startup**: Load the box from YAML in `palm_frame`, then do a one-time TF lookup to transform the corners into `arm_frame`. This avoids changing the pruning frame but requires the TF chain to be available at node startup. More fragile.

2. **Use TF frames directly (lookup bb_corner and bb_opposite at runtime)**: Instead of loading the YAML, look up the positions of `bb_corner` and `bb_opposite` frames in `palm_frame` via TF2, and compute the AABB from those. This is the most "TF-native" approach but adds per-cycle TF lookups for something that is static. Over-engineering for a static box.

3. **Keep everything as-is, just add the YAML loading**: Don't change the pruning frame. Just load the values from YAML and convert to `arm_frame` coordinates. Simplest change but doesn't give the visual feedback benefit since the markers are in `palm_frame`.
