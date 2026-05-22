# Plan: Integrate TF-Based Bounding Boxes for Arm Pruning in Pointcloud Fusion

**Date:** 2026-05-21
**Source branch:** `work/cleaned_full_test_20260515` (commit `4913a30`)
**Target branch:** current HEAD

---

## 1. Verified Findings

### 1.1 Branch Relationship

The branch `work/cleaned_full_test_20260515` has **2 new commits** that are NOT on the current branch:

| Commit | Description |
|--------|-------------|
| `7492b33` | Add grasp contact transformation and update documentation |
| `4913a30` | Add camera bounding box configuration and visualization support |

These were added on top of commit `8cd787b` (which IS on the current branch). The current branch has since diverged with additional work (twist propagation, pipeline, etc.).

### 1.2 What the branch has that the current branch does NOT

**3 new YAML sections** in `src/sensor_fusion_bringup/config/camera_mounts.yaml`:

1. **`grasp_contact`** — A point in `palm_frame` for the grasp contact location:
   ```yaml
   grasp_contact:
     palm_to_grasp_contact:
       translation: { x: 0.0, y: 0.13, z: 0.03 }
   ```
   TF: `palm_frame -> grasp_contact_frame`

2. **`cam_bounding_box_8cm`** — The **camera arm pruning box**, defined in the screw frame of the 8cm mount:
   ```yaml
   cam_bounding_box_8cm:
     screw_to_bbcam1:
       translation: { x: 0.02, y: 0.05, z: 0.14 }
     screw_to_bbcam2:
       translation: { x: -0.035, y: -0.10, z: -0.01 }
   ```
   TFs: `d435i_arm_bottom_screw_frame_8_cm_cam_mount -> bbcam1_frame` and `-> bbcam2_frame`

3. **`bounding_box`** (workspace) — Already on current branch, unchanged.

**Updated `publish_camera_mounts.py`** with:
- New publisher `/camera_mounts/cam_bounding_box` (orange CUBE marker)
- New method `_build_cam_bbox_tfs()` — publishes `bbcam1_frame` and `bbcam2_frame`
- New method `_make_cam_bbox_marker()` — orange CUBE marker in screw frame
- New method `_build_grasp_contact_tf()` — publishes `grasp_contact_frame`
- **Fixed `world -> palm_frame` rotation**: changed from `(0, 1, 0, 0)` (roty 180) to `(1, 0, 0, 0)` (identity)

**Updated `camera_mounts.rviz`** with:
- New Marker display for `/camera_mounts/cam_bounding_box` (CameraBoundingBox)

**Updated `docs/camera_mounts.md`** with:
- `world -> palm_frame` changed from "roty 180" to "identity"

### 1.3 Coordinate System Analysis

**Box 1: `bounding_box` (workspace) — in `palm_frame`**
- Corner 1: `(0.11, 0.205, 0.11)`
- Corner 2: `(0.11 + (-0.22), 0.205 + (-0.34), 0.11 + (-0.32))` = `(-0.11, -0.135, -0.21)`
- AABB min: `(-0.11, -0.135, -0.21)`, max: `(0.11, 0.205, 0.11)`
- Dimensions: 0.22 x 0.34 x 0.32 m
- Purpose: workspace volume where objects of interest are expected

**Box 2: `cam_bounding_box_8cm` (camera arm pruning) — in `d435i_arm_bottom_screw_frame_8_cm_cam_mount`**
- Corner 1 (bbcam1): `(0.02, 0.05, 0.14)`
- Corner 2 (bbcam2): `(-0.035, -0.10, -0.01)`
- AABB min: `(-0.035, -0.10, -0.01)`, max: `(0.02, 0.05, 0.14)`
- Dimensions: 0.055 x 0.15 x 0.15 m
- Purpose: small box around the camera/hand assembly to prune the arm from the pointcloud

**Current fusion node pruning** — in `arm_d435i_arm_depth_frame`:
- AABB min: `(-0.30, -0.10, -0.10)`, max: `(0.22, 0.10, 0.12)`
- Dimensions: 0.52 x 0.20 x 0.22 m
- This is a MUCH larger box — it's the existing hardcoded pruning volume

### 1.4 Key Design Decision

The two boxes from the branch serve different purposes:
- **`bounding_box`** = workspace (keep points inside) — NOT for pruning
- **`cam_bounding_box_8cm`** = camera/hand assembly pruning (remove points inside)

For the fusion node's arm pruning, we want to use **`cam_bounding_box_8cm`** as the pruning box instead of (or in addition to) the current hardcoded box. The box is already defined in the screw frame of the 8cm mount, which connects to the TF tree via `palm_frame -> d435i_arm_bottom_screw_frame_8_cm_cam_mount`.

---

## 2. Implementation Plan

### Phase 1: Port YAML config from branch

**File:** `src/sensor_fusion_bringup/config/camera_mounts.yaml`

- [ ] **1.1** Add `grasp_contact` section after `bounding_box` (lines 60+), before `mounts`
- [ ] **1.2** Add `cam_bounding_box_8cm` section after `grasp_contact`, before `mounts`
- [ ] **1.3** Update header comment to document new transform chains:
  ```
  # palm_frame -> grasp_contact        (grasp contact point, below)
  # screw_frame -> bbcam1/bbcam2       (cam arm pruning box, below)
  ```

**Exact YAML to add** (from branch, verified byte-for-byte):
```yaml
grasp_contact:
  palm_to_grasp_contact:
    translation:
      x: 0.0
      y: 0.13
      z: 0.03
    rotation: identity
    quaternion:
    - 0.0
    - 0.0
    - 0.0
    - 1.0

cam_bounding_box_8cm:
  screw_to_bbcam1:
    translation:
      x: 0.02
      y: 0.05
      z: 0.14
    rotation: identity
    quaternion:
    - 0.0
    - 0.0
    - 0.0
    - 1.0
  screw_to_bbcam2:
    translation:
      x: -0.035
      y: -0.10
      z: -0.01
    rotation: identity
    quaternion:
    - 0.0
    - 0.0
    - 0.0
    - 1.0
```

### Phase 2: Port `publish_camera_mounts.py` from branch

**File:** `src/sensor_fusion_bringup/scripts/publish_camera_mounts.py`

- [ ] **2.1** Add new publisher `/camera_mounts/cam_bounding_box` in `__init__` (after line 154)
- [ ] **2.2** Add conditional cam bbox marker creation in `__init__` (after line 167):
  ```python
  if "cam_bounding_box_8cm" in data:
      self._cam_bbox_marker = self._make_cam_bbox_marker(data["cam_bounding_box_8cm"])
      self._cam_bbox_marker.header.stamp = self.get_clock().now().to_msg()
      self._cam_marker_pub.publish(self._cam_bbox_marker)
  else:
      self._cam_bbox_marker = None
  ```
- [ ] **2.3** Update `_republish_marker()` to also republish cam bbox marker
- [ ] **2.4** Update `_publish_single()` to include grasp_contact + cam_bbox TFs
- [ ] **2.5** Update `_publish_all()` to include grasp_contact + cam_bbox TFs
- [ ] **2.6** Fix `world -> palm_frame` quaternion in `_build_shared_tfs()`:
  - Change from `(0.0, 1.0, 0.0, 0.0)` to `(1.0, 0.0, 0.0, 0.0)` (identity)
- [ ] **2.7** Add `_build_grasp_contact_tf()` method
- [ ] **2.8** Add `_build_cam_bbox_tfs()` method
- [ ] **2.9** Add `_make_cam_bbox_marker()` method (orange CUBE, `r=1.0, g=0.6, b=0.0, a=0.2`)

### Phase 3: Port `camera_mounts.rviz` from branch

**File:** `rviz/camera_mounts.rviz`

- [ ] **3.1** Add new Marker display entry for `/camera_mounts/cam_bounding_box` (CameraBoundingBox)
  - Insert after the existing BoundingBox display (after line 43)

### Phase 4: Port `docs/camera_mounts.md` from branch

**File:** `docs/camera_mounts.md`

- [ ] **4.1** Change `world -> palm_frame` description from "(roty 180)" to "(identity)" (line 23)
- [ ] **4.2** Change `world -> palm_frame` description from "roty 180" to "identity" (line 130)

### Phase 5: Modify `pointcloud_fusion_node.py` for TF-based pruning

**File:** `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py`

- [ ] **5.1** Add new parameters in `__init__` (after line 220):
  ```python
  self.declare_parameter("mounts_config_path", "")
  self.declare_parameter("active_mount", "8_cm_cam_mount")
  ```

- [ ] **5.2** Add helper method `_load_pruning_boxes(config_path, mount_name)`:
  - Reads `camera_mounts.yaml`
  - Extracts `cam_bounding_box_8cm` section
  - Computes `bbox_min`/`bbox_max` from `screw_to_bbcam1` and `screw_to_bbcam2`:
    ```python
    c1 = [0.02, 0.05, 0.14]
    c2 = [-0.035, -0.10, -0.01]
    bbox_min = elementwise_min(c1, c2)  # [-0.035, -0.10, -0.01]
    bbox_max = elementwise_max(c1, c2)  # [0.02, 0.05, 0.14]
    ```
  - Returns `(frame_id, bbox_min, bbox_max)` where frame_id is `d435i_arm_bottom_screw_frame_{mount_name}`
  - Falls back to the existing `bbox_min`/`bbox_max` ROS params if YAML is unavailable

- [ ] **5.3** In `__init__`, after reading params (after line 238), call `_load_pruning_boxes()`:
  ```python
  mounts_config = self.get_parameter("mounts_config_path").value
  if mounts_config:
      self._pruning_frame, self._bbox_min, self._bbox_max = \
          self._load_pruning_boxes(mounts_config, self.get_parameter("active_mount").value)
      self.get_logger().info(
          f"Loaded pruning box from {mounts_config}: "
          f"frame={self._pruning_frame}, min={self._bbox_min}, max={self._bbox_max}")
  else:
      self._pruning_frame = self._arm_frame  # fallback to existing behavior
  ```

- [ ] **5.4** Update step 4 (hand/arm bbox removal, lines 441-458) to use `self._pruning_frame` instead of `self._arm_frame`:
  ```python
  # Change line 444 from:
  xyz_arm = _transform_points_to_frame(
      xyz_all, self._tf_buffer, self._arm_frame,
      self._target_frame, rclpy.time.Time())
  # To:
  xyz_arm = _transform_points_to_frame(
      xyz_all, self._tf_buffer, self._pruning_frame,
      self._target_frame, rclpy.time.Time())
  ```

- [ ] **5.5** Update `_publish_bbox_marker()` (lines 504-536) to use `self._pruning_frame`:
  ```python
  # Change line 510 from:
  marker.header.frame_id = self._arm_frame
  # To:
  marker.header.frame_id = self._pruning_frame
  ```

- [ ] **5.6** Update startup log (lines 300-304) to include pruning frame info.

### Phase 6: Update `prosthesis_config.yaml`

**File:** `config/prosthesis_config.yaml`

- [ ] **6.1** Add new parameters to `pointcloud_fusion` section (after line 144):
  ```yaml
  mounts_config_path: ""
  active_mount: "8_cm_cam_mount"
  ```
  When `mounts_config_path` is empty (default), the node falls back to the existing `bbox_min`/`bbox_max` params — full backward compatibility.

- [ ] **6.2** For deployment, set `mounts_config_path` to the installed path:
  ```yaml
  mounts_config_path: "/prosthesis_ws/install/sensor_fusion_bringup/share/sensor_fusion_bringup/config/camera_mounts.yaml"
  ```

### Phase 7: Verify

- [ ] **7.1** Run `make -f Makefile.rviz mounts-viz MOUNT=8_cm_cam_mount` and verify:
  - Green workspace bbox visible in `palm_frame`
  - Orange cam bbox visible in `d435i_arm_bottom_screw_frame_8_cm_cam_mount`
  - `grasp_contact_frame` TF published
  - `bbcam1_frame` and `bbcam2_frame` TFs published
  - `world -> palm_frame` has identity rotation (not roty 180)

- [ ] **7.2** Launch the full pipeline with `mounts_config_path` set and verify:
  - Fusion node logs: "Loaded pruning box from ... frame=d435i_arm_bottom_screw_frame_8_cm_cam_mount"
  - Red bbox marker appears at the correct position in RViz
  - Points inside the cam bbox are removed from the fused cloud

- [ ] **7.3** Verify backward compatibility: with `mounts_config_path: ""`, the node uses the existing hardcoded `bbox_min`/`bbox_max` in `arm_d435i_arm_depth_frame`.

---

## 3. Files Changed Summary

| File | Action | Lines Changed |
|------|--------|---------------|
| `src/sensor_fusion_bringup/config/camera_mounts.yaml` | Add 2 sections | +38 lines |
| `src/sensor_fusion_bringup/scripts/publish_camera_mounts.py` | Port from branch | +67 lines, ~3 modified |
| `rviz/camera_mounts.rviz` | Add marker display | +9 lines |
| `docs/camera_mounts.md` | Fix rotation description | 2 lines changed |
| `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` | Add TF-based pruning | ~40 lines added/modified |
| `config/prosthesis_config.yaml` | Add 2 params | +2 lines |

---

## 4. Risks and Mitigations

1. **TF chain for cam bbox pruning**: The pruning box is in `d435i_arm_bottom_screw_frame_8_cm_cam_mount`. In the live pipeline, this frame is published by `publish_camera_mounts.py`. If that node is not running, the fusion node won't be able to transform points to that frame. **Mitigation**: The fusion node already handles TF lookup failures gracefully (warns and skips). The launch file should include `publish_camera_mounts.py` when cameras are enabled.

2. **`world -> palm_frame` rotation change**: The branch changes this from roty-180 to identity. In the standalone visualization (`mounts-viz`), this affects how `palm_frame` is oriented relative to `world`. In the live pipeline, `palm_frame` is published by a different source (OpenVINS/hand tracking), so this change only affects offline visualization. **Mitigation**: Verify the standalone viz looks correct after the change.

3. **Box size difference**: The cam bbox (0.055 x 0.15 x 0.15 m) is MUCH smaller than the current hardcoded box (0.52 x 0.20 x 0.22 m). This means less of the arm will be pruned. The cam bbox is designed to cover just the camera body, not the full arm. **Mitigation**: This is intentional — the cam bbox is for removing the camera's self-view. If full arm removal is still needed, consider using BOTH boxes (cam bbox + a separate arm bbox), or keep the existing large box as a fallback.

4. **Mount-specific naming**: `cam_bounding_box_8cm` is tied to the 8cm mount's screw frame. If the mount changes, the config section and frame name must change too. **Mitigation**: The `active_mount` parameter addresses this — the fusion node constructs the frame name dynamically.

---

## 5. Future Considerations

- **Two-box pruning**: If both the workspace box AND the cam bbox should be used for pruning (keep points inside workspace, remove points inside cam bbox), the fusion node would need to support multiple pruning boxes with different semantics (keep-inside vs. remove-inside). This is a future enhancement.
- **Dynamic mount switching**: If the mount is changed at runtime, the fusion node would need to reload the config. Currently, the box is loaded once at startup.
- **The `grasp_contact` frame**: This is not used by the fusion node but may be useful for the grasp preshaping service or proximity controller in the future.
