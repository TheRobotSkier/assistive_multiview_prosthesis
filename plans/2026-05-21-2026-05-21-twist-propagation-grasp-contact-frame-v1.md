# Plan: Change Twist Propagation Frame to `grasp_contact_frame`

**Date:** 2026-05-21
**Objective:** Move the twist propagation origin from the OpenVINS camera pose (`arm_cam0`) to the grasp contact point (`grasp_contact_frame`) so that predictions start from the fingertips rather than the camera body.

---

## 1. Verified Findings

### 1.1 Current Twist Propagation Flow

The twist propagation node receives hand pose via `/hand_pose` (a `PoseStamped`):

1. **OpenVINS** publishes `/ov_msckf_arm/odomimu` (nav_msgs/Odometry) with `header.frame_id = "marker_map"` — the pose is the **camera position** in `marker_map`.
2. **`odom_to_pose_relay.py`** (`src/camera/camera/odom_to_pose_relay.py:64-68`) copies the header verbatim:
   ```python
   ps.header = msg.header   # frame_id = "marker_map"
   ps.pose = msg.pose.pose  # camera position in marker_map
   ```
3. **`twist_propagation_node.py`** receives this on `/hand_pose` and stores the pose with its frame_id (`twist_propagation_node.py:578-588`):
   ```python
   frame_id = msg.header.frame_id  # "marker_map"
   self._pose_buf.append((stamp, px, py, pz, qx, qy, qz, qw, frame_id))
   ```
4. In `_run_idle_cycle` (line 1105), it transforms the pose position to the cloud frame:
   ```python
   transformed = self._transform_pose_to_cloud_frame(px, py, pz, pose_frame)
   ```
   Since `pose_frame == "marker_map"` and the cloud is also in `marker_map`, this is a no-op.
5. Propagation starts from `(px_cloud, py_cloud, pz_cloud)` — the **camera position** in `marker_map`.

**Problem:** The propagation origin is at the camera body, not the fingertips. The grasp contact point is approximately 13cm forward and 3cm up from the palm origin, which itself is offset from the camera. The predictions travel from the camera forward, not from where the hand will actually contact objects.

### 1.2 The `grasp_contact_frame` TF

Defined in `camera_mounts.yaml:63-74`:
```yaml
grasp_contact:
  palm_to_grasp_contact:
    translation: { x: 0.0, y: 0.13, z: 0.03 }
```

Published by `publish_camera_mounts.py:220-227` as a static TF:
```
palm_frame -> grasp_contact_frame  (translation: 0, 0.13, 0.03)
```

### 1.3 Complete TF Chain (Live Pipeline)

```
marker_map (OpenVINS root)
  └── arm_imu (OpenVINS dynamic)
        └── arm_cam0 (OpenVINS dynamic)  ← /hand_pose reports THIS position
              └── arm_d435i_arm_link (bridge: openvins_realsense_tf_bridge)
                    └── arm_d435i_arm_depth_frame
                    └── arm_d435i_arm_color_optical_frame
                    └── ... (other RealSense frames)

publish_camera_mounts.py publishes (STATIC):
  world -> palm_frame
    ├── d435i_arm_bottom_screw_frame_8_cm_cam_mount
    │     └── d435i_arm_link_8_cm_cam_mount
    ├── bb_corner -> bb_opposite
    ├── grasp_contact_frame
    └── bbcam1_frame, bbcam2_frame
```

### 1.4 Critical TF Chain Gap

**The chain `marker_map -> palm_frame -> grasp_contact_frame` is NOT resolvable in the live pipeline.**

Reason: `publish_camera_mounts.py` publishes `world -> palm_frame` (static identity), but OpenVINS operates in `marker_map`. The frames `world` and `marker_map` are different frame IDs. There is no TF edge connecting them.

Furthermore, `palm_frame` is published as a **static identity** transform (0,0,0 with identity rotation). It doesn't move with the hand. In the live system, the hand moves dynamically — tracked by OpenVINS through `arm_cam0`.

The chain from `marker_map` to `palm_frame` would need:
```
marker_map -> arm_imu -> arm_cam0 -> arm_d435i_arm_link -> d435i_arm_link_8_cm_cam_mount
  -> d435i_arm_bottom_screw_frame_8_cm_cam_mount (inverse) -> palm_frame
  -> grasp_contact_frame
```

But this chain requires:
1. `arm_d435i_arm_link -> d435i_arm_link_8_cm_cam_mount` — NOT published (the bridge publishes `arm_d435i_arm_link` as a child of `arm_cam0`, but `d435i_arm_link_8_cm_cam_mount` is a separate static frame under `palm_frame`)
2. `palm_frame` is under `world`, not under the dynamic `marker_map` tree

### 1.5 Two Approaches

Given the TF chain gap, there are two clean approaches:

**Approach A: Config-based offset (recommended, simpler)**
- Add a `propagation_origin_offset` parameter (3D vector) to the twist propagation node
- Apply this offset to the hand pose position **in the pose's own frame** before propagation
- The offset is the vector from the camera tracking origin to the grasp contact point, expressed in the camera's frame
- No TF changes needed — works immediately with the existing pipeline
- Can be computed from `camera_mounts.yaml` at startup if desired

**Approach B: Fix the TF chain and use TF lookup**
- Change `publish_camera_mounts.py` to publish `palm_frame` under `arm_d435i_arm_link` (or `arm_cam0`) instead of under `world`
- This requires computing the `arm_d435i_arm_link -> palm_frame` transform from the mount config
- Then `grasp_contact_frame` would be dynamically connected to `marker_map`
- The twist propagation node would look up `marker_map -> grasp_contact_frame` at runtime
- More complex, requires careful coordinate transform math, but gives a "pure TF" solution

### 1.6 Recommended Approach: A (Config-based offset)

The offset from the OpenVINS camera origin (`arm_cam0`) to the grasp contact point is a **fixed rigid transform** (the camera is bolted to the hand). We can compute it from the existing config:

From `camera_mounts.yaml`:
- `arm_cam0` ≈ `arm_d435i_arm_color_optical_frame` (OpenVINS uses the color camera)
- Chain: `color_optical -> link -> screw_frame -> palm_frame -> grasp_contact_frame`

The offset can be precomputed from the YAML at node startup, or hardcoded as a parameter. The twist node applies it to the pose position (and rotates it by the pose orientation) before propagation.

---

## 2. Implementation Plan

### Phase 1: Compute the camera-to-grasp-contact offset

- [ ] **1.1** Add a new parameter `propagation_origin_offset` to `twist_propagation_node.py` (default: `[0.0, 0.0, 0.0]`)
  - This is a 3D vector in the **pose frame** (i.e., `marker_map` frame when using OpenVINS)
  - When non-zero, the node adds this offset to the pose position before propagation
  - **Rationale:** Simple, no TF dependency, backward compatible

- [ ] **1.2** Add a new parameter `mounts_config_path` (default: `""`) to optionally load the offset from `camera_mounts.yaml`
  - If provided, compute the offset from the full transform chain: `arm_cam0 -> link -> screw -> palm -> grasp_contact`
  - This requires the mount quaternion and translation from the YAML
  - **Rationale:** Single source of truth for the offset, auto-computed from the same config used for pruning boxes

- [ ] **1.3** Add `_compute_grasp_contact_offset(config_path, mount_name)` static method
  - Reads `camera_mounts.yaml`
  - Builds the chain: `color_optical_frame -> link -> screw -> palm -> grasp_contact`
  - Returns a 3D translation vector in the color_optical frame
  - **Note:** This is the offset from where OpenVINS tracks (color_optical ≈ cam0) to the grasp contact point, expressed in the cam0 frame
  - Since OpenVINS reports pose in `marker_map`, and the pose orientation is `marker_map -> cam0`, we need to rotate this offset by the inverse of the pose orientation to get the offset in `marker_map` frame... OR we apply the offset in the pose's local frame before transforming to cloud frame.

### Phase 2: Modify `_run_idle_cycle` to apply the offset

- [ ] **2.1** In `_run_idle_cycle` (around line 1100-1106), after reading the latest pose:
  ```python
  # Apply grasp contact offset (in pose local frame)
  if self._grasp_contact_offset is not None:
      qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
      ox, oy, oz = self._grasp_contact_offset
      # Rotate offset by pose orientation
      r00 = 1.0 - 2.0 * (qy*qy + qz*qz)
      r01 = 2.0 * (qx*qy - qz*qw)
      r02 = 2.0 * (qx*qz + qy*qw)
      r10 = 2.0 * (qx*qy + qz*qw)
      r11 = 1.0 - 2.0 * (qx*qx + qz*qz)
      r12 = 2.0 * (qy*qz - qx*qw)
      r20 = 2.0 * (qx*qz - qy*qw)
      r21 = 2.0 * (qy*qz + qx*qw)
      r22 = 1.0 - 2.0 * (qx*qx + qy*qy)
      px += r00*ox + r01*oy + r02*oz
      py += r10*ox + r11*oy + r12*oz
      pz += r20*ox + r21*oy + r22*oz
  ```
  **Rationale:** The offset is in the camera's local frame. We rotate it by the camera's orientation (in marker_map) to get the world-frame offset, then add it to the position. This correctly places the propagation origin at the grasp contact point regardless of hand orientation.

- [ ] **2.2** Store the offset in `__init__` after parameter reading:
  ```python
  offset_param = self.get_parameter("propagation_origin_offset").value
  mounts_config = self.get_parameter("mounts_config_path").value
  if mounts_config:
      self._grasp_contact_offset = tuple(
          self._compute_grasp_contact_offset(mounts_config, 
              self.get_parameter("active_mount").value))
  elif offset_param and any(v != 0.0 for v in offset_param):
      self._grasp_contact_offset = tuple(offset_param)
  else:
      self._grasp_contact_offset = None
  ```

- [ ] **2.3** Update the startup log (line 540-546) to report the offset:
  ```python
  offset_str = (f"grasp_contact_offset={self._grasp_contact_offset}"
                if self._grasp_contact_offset else "no offset")
  self.get_logger().info(
      f"Twist propagation node started -- active={self._active}, "
      f"..., {offset_str}")
  ```

### Phase 3: Implement `_compute_grasp_contact_offset`

- [ ] **3.1** Add the static method to compute the offset from `camera_mounts.yaml`:
  - Read the YAML
  - Build the transform chain as 4x4 matrices:
    1. `T_color_optical_to_link` = inverse of `T_link_to_color_optical` (from RealSense URDF: identity rotation + (0, 0.015, 0) translation)
    2. `T_link_to_screw` = inverse of `screw_to_link` from YAML: (−0.0106, −0.0175, −0.0125)
    3. `T_screw_to_palm` = from mount config `screw_to_palm` (includes quaternion rotation!)
    4. `T_palm_to_grasp_contact` = from `grasp_contact` section: (0, 0.13, 0.03)
  - Multiply: `T = T_color_optical_to_link @ T_link_to_screw @ T_screw_to_palm @ T_palm_to_grasp_contact`
  - Extract the translation column from T — this is the offset in `color_optical_frame` (= `arm_cam0`)
  - **Rationale:** This chain gives us the rigid transform from the OpenVINS tracking origin to the grasp contact point. Since the camera is rigidly mounted on the hand, this offset is constant.

### Phase 4: Update configuration files

- [ ] **4.1** Add parameters to `prosthesis_config.yaml` twist_propagation section:
  ```yaml
  twist_propagation:
    ros__parameters:
      # ... existing params ...
      propagation_origin_offset: [0.0, 0.0, 0.0]
      mounts_config_path: ""
      active_mount: "8_cm_cam_mount"
  ```

- [ ] **4.2** Update `src/twist_propagation/config/twist_propagation.yaml` with the same new params and documentation comments.

### Phase 5: Update pipeline launch

- [ ] **5.1** In `pipeline.launch.py`, pass `mounts_config_path` and `active_mount` to the twist propagation node so it can auto-compute the offset from the same config used by the fusion node:
  ```python
  twist_params = _node_params(config, "twist_propagation")
  twist_params.update({
      "mounts_config_path": mounts_config,
      "active_mount": camera_mount,
  })
  ```
  This reuses the same `mounts_config` and `camera_mount` launch arguments already used for the fusion node.

### Phase 6: Verification

- [ ] **6.1** Standalone test: Set `propagation_origin_offset: [0.0, 0.13, 0.0]` (rough approximation) and verify that the predicted path starts ~13cm forward from the camera position.

- [ ] **6.2** Config-based test: Set `mounts_config_path` to the camera mounts YAML and verify the auto-computed offset matches the expected value. Log the computed offset at startup.

- [ ] **6.3** Visual verification in RViz: The `/twist_propagation/current_pose` should appear at the grasp contact point (near fingertips) rather than at the camera body.

- [ ] **6.4** Backward compatibility: With default params (`propagation_origin_offset: [0,0,0]`, `mounts_config_path: ""`), behavior is unchanged — propagation starts from the camera position.

---

## 3. Files Changed Summary

| File | Action | Scope |
|------|--------|-------|
| `src/twist_propagation/twist_propagation/twist_propagation_node.py` | Add offset params, apply offset in cycle, add compute method | ~50 lines added |
| `config/prosthesis_config.yaml` | Add 3 params to twist_propagation section | +3 lines |
| `src/twist_propagation/config/twist_propagation.yaml` | Add 3 params with documentation | +6 lines |
| `src/prosthesis_launch/launch/pipeline.launch.py` | Pass mounts_config + active_mount to twist node | ~5 lines |

---

## 4. Risks and Mitigations

1. **Offset computed in wrong frame**: The offset must be in the `arm_cam0` (color_optical) frame, not `marker_map`. If the rotation is wrong, the offset will point in the wrong direction.
   **Mitigation:** The `_compute_grasp_contact_offset` method builds the full chain from `color_optical` through all intermediate frames to `grasp_contact_frame`. The math is explicit and testable. Add a unit test that verifies the chain against known values.

2. **Orientation mismatch**: OpenVINS reports the camera orientation in `marker_map`. The offset must be rotated by this orientation before being added to the position. If the quaternion convention is wrong, the offset direction will be incorrect.
   **Mitigation:** Use the same rotation matrix code already in `_transform_pose_to_cloud_frame` (line 811-819). This is proven to work correctly.

3. **Mount quaternion complexity**: The `screw_to_palm` quaternion `(-0.5, -0.5, 0.5, -0.5)` represents a 120° rotation. Getting the chain direction wrong (palm->screw vs screw->palm) would give an incorrect offset.
   **Mitigation:** The `_invert_transform` function in `publish_camera_mounts.py` already handles this correctly. Reuse the same logic or import it.

4. **The `world` vs `marker_map` frame gap**: This plan deliberately avoids the TF chain approach (Approach B) because of this gap. The config-based offset is independent of the TF tree.
   **Mitigation:** N/A — the offset approach doesn't depend on TF.

---

## 5. Open Questions

1. **Should we also fix the TF chain for Approach B?** This would make `grasp_contact_frame` dynamically available in the TF tree, which could be useful for other nodes (e.g., grasp preshaping, proximity controller). But it's a separate task with its own complexity. Recommend doing Approach A first, then Approach B as a follow-up if needed.

2. **Should the twist node also publish the offset-corrected pose?** Currently `/twist_propagation/current_pose` shows the camera position. After the change, it would show the grasp contact position. This is likely desirable for visualization, but downstream consumers should be aware.

3. **Exact offset value verification**: The computed offset depends on the mount quaternion `(-0.5, -0.5, 0.5, -0.5)` and the screw-to-palm translation. We should verify the computed value against a physical measurement or RViz visualization before relying on it.
