# Plan: Connect Camera Mounts TF Tree to Live Pipeline + Twist Propagation Grasp Contact

**Date:** 2026-05-21
**Objective:** Fix the disconnected TF tree so that `palm_frame`, `grasp_contact_frame`, and pruning box frames are accessible from `marker_map` in the live pipeline. Then use `grasp_contact_frame` as the twist propagation origin.

---

## 1. Verified Findings

### 1.1 The Problem: Disconnected TF Trees

The live pipeline has **two disconnected TF trees**:

**Tree A (dynamic, from OpenVINS + bridge):**
```
marker_map -> arm_imu -> arm_cam0 -> arm_d435i_arm_link -> arm_d435i_arm_depth_frame -> ...
```

**Tree B (static, from publish_camera_mounts.py):**
```
world -> palm_frame -> d435i_arm_bottom_screw_frame_8_cm_cam_mount -> d435i_arm_link_8_cm_cam_mount
                    -> grasp_contact_frame
                    -> bb_corner -> bb_opposite
                    -> (screw_frame -> bbcam1_frame, bbcam2_frame)
```

`world` and `marker_map` are **different frame IDs**. There is no TF edge connecting them.

### 1.2 Consequence: Pruning Boxes Don't Work

The fusion node (`pointcloud_fusion_node.py:464-468`) calls `_transform_points_to_frame(xyz_all, tf_buffer, "palm_frame", "marker_map", ...)` to transform points from `marker_map` to `palm_frame` for bbox pruning. This TF lookup **fails silently** (returns None, logs a warning) because `palm_frame` is under `world`, not `marker_map`.

**The same applies to the screw frame** for the cam bounding box.

This means the two-box TF-based pruning we just implemented is **not actually pruning anything** in the live pipeline. It falls through with a warning every cycle.

### 1.3 The Connection Point

`arm_d435i_arm_link` (from the bridge tree) and `d435i_arm_link_8_cm_cam_mount` (from the mounts tree) are the **same physical point** — the D435i camera body origin.

The bridge publishes: `arm_cam0 -> arm_d435i_arm_link`
The mounts config has: `screw_frame -> link_frame` where `link_frame = d435i_arm_link_8_cm_cam_mount`

Since `arm_d435i_arm_link ≈ d435i_arm_link_8_cm_cam_mount` (same physical D435i body), and `screw_frame` is offset by `(0.0106, 0.0175, 0.0125)` from `link_frame`, we can connect the trees by publishing an identity transform: `arm_d435i_arm_link -> screw_frame`.

Wait — not identity. The `screw_frame` is the tripod mount hole, and `link_frame` is the camera body origin. They differ by `screw_to_link = (0.0106, 0.0175, 0.0125)`. So:

```
arm_d435i_arm_link -> d435i_arm_bottom_screw_frame_8_cm_cam_mount
```
needs to be the inverse of `screw_to_link`, i.e., `(-0.0106, -0.0175, -0.0125)` with identity rotation.

### 1.4 Current Tree Structure (from publish_camera_mounts.py)

```
_build_shared_tfs:    world -> palm_frame (identity)
_build_mount_tfs:     palm_frame -> screw_frame (INVERTED screw_to_palm)
                      screw_frame -> link_frame (screw_to_link)
_build_grasp_contact: palm_frame -> grasp_contact_frame
_build_bounding_box:  palm_frame -> bb_corner -> bb_opposite
_build_cam_bbox:      screw_frame -> bbcam1_frame, bbcam2_frame
```

### 1.5 Proposed Tree Structure

Replace `world -> palm_frame` root with `arm_d435i_arm_link -> screw_frame`, and restructure so `palm_frame` is a child of `screw_frame`:

```
arm_d435i_arm_link -> d435i_arm_bottom_screw_frame_8_cm_cam_mount  (inverse of screw_to_link)
                      screw_frame -> d435i_arm_link_8_cm_cam_mount (screw_to_link)
                      screw_frame -> palm_frame (screw_to_palm, NOT inverted)
                      screw_frame -> bbcam1_frame, bbcam2_frame
                      palm_frame -> grasp_contact_frame
                      palm_frame -> bb_corner -> bb_opposite
```

This connects to the live pipeline because `arm_d435i_arm_link` is already in the `marker_map` tree via the bridge.

### 1.6 The Full Resolved TF Chain (after fix)

```
marker_map -> arm_imu -> arm_cam0 -> arm_d435i_arm_link
                                        ↓ (inverse of screw_to_link)
                                  d435i_arm_bottom_screw_frame_8_cm_cam_mount
                                        ↓ (screw_to_link)
                                  d435i_arm_link_8_cm_cam_mount
                                        ↓ (screw_to_palm, direct)
                                  palm_frame
                                        ↓ (palm_to_grasp_contact)
                                  grasp_contact_frame
                                        ↓ (palm_to_corner)
                                  bb_corner -> bb_opposite
                                  d435i_arm_bottom_screw_frame_8_cm_cam_mount
                                        ↓ (screw_to_bbcam1/2)
                                  bbcam1_frame, bbcam2_frame
```

All frames are now reachable from `marker_map`.

---

## 2. Implementation Plan

### Phase 1: Restructure `publish_camera_mounts.py` TF tree

**File:** `src/sensor_fusion_bringup/scripts/publish_camera_mounts.py`

- [ ] **1.1** Add a `--world-frame` CLI argument (default: `""`). When empty, the node uses the arm link frame as root instead of `world`. When set to `"world"`, it publishes the old `world -> palm_frame` tree for standalone RViz visualization.

- [ ] **1.2** Add a `--link-frame` CLI argument (default: `"arm_d435i_arm_link"`). This is the bridge's link frame that the mounts tree connects to.

- [ ] **1.3** Rewrite `_build_shared_tfs()` to publish the connection edge:
  ```python
  def _build_shared_tfs(self, data: dict, link_frame: str, mount_name: str) -> list[TransformStamped]:
      s = self._stamp
      screw_frame = f"d435i_arm_bottom_screw_frame_{mount_name}"
      # arm_d435i_arm_link -> screw_frame (inverse of screw_to_link)
      sl = data["screw_to_link"]
      tfs = [_make_tf(s, link_frame, screw_frame,
                      -_t(sl, "x"), -_t(sl, "y"), -_t(sl, "z"),
                      0.0, 0.0, 0.0, 1.0)]  # identity rotation (inverse of identity)
      return tfs
  ```
  **Rationale:** `screw_to_link` is `(0.0106, 0.0175, 0.0125)` with identity rotation. The inverse is `(-0.0106, -0.0175, -0.0125)` with identity rotation. This connects `arm_d435i_arm_link` to `screw_frame`.

- [ ] **1.4** Rewrite `_build_mount_tfs()` to publish `screw_frame -> palm_frame` (direct, NOT inverted):
  ```python
  def _build_mount_tfs(self, data, mount_name, suffix):
      s = self._stamp
      mount = data["mounts"][mount_name]
      sp = mount["screw_to_palm"]
      screw_frame = f"d435i_arm_bottom_screw_frame{suffix}"
      link_frame = f"d435i_arm_link{suffix}"
      
      tfs = []
      # screw_frame -> palm_frame (direct screw_to_palm, NOT inverted)
      tfs.append(_make_tf(s, screw_frame, "palm_frame",
                          _t(sp, "x"), _t(sp, "y"), _t(sp, "z"),
                          *_q(sp)))
      # screw_frame -> link_frame (screw_to_link)
      sl = data["screw_to_link"]
      tfs.append(_make_tf(s, screw_frame, link_frame,
                          _t(sl, "x"), _t(sl, "y"), _t(sl, "z"),
                          *_q(sl)))
      return tfs
  ```
  **Rationale:** Previously, `screw_to_palm` was inverted to get `palm -> screw`. Now we keep it as `screw -> palm` because `screw_frame` is the root (connected to `arm_d435i_arm_link`).

- [ ] **1.5** Update `_publish_single()` and `_publish_all()` to pass the new args:
  ```python
  def _publish_single(self, data, mount_name, link_frame):
      tfs = self._build_shared_tfs(data, link_frame, mount_name)
      tfs += self._build_mount_tfs(data, mount_name, f"_{mount_name}")
      tfs += self._build_grasp_contact_tf(data)
      tfs += self._build_bounding_box_tfs(data)
      if "cam_bounding_box_8cm" in data:
          tfs += self._build_cam_bbox_tfs(data)
      self._broadcaster.sendTransform(tfs)
  ```

- [ ] **1.6** Update `__init__` to accept and store `link_frame` and `world_frame` args, and use them to decide the tree structure.

- [ ] **1.7** Keep the `world -> palm_frame` mode for standalone visualization: when `--world-frame world` is passed, publish the old tree structure (useful for `make -f Makefile.rviz mounts-viz`).

### Phase 2: Update `pipeline.launch.py` to pass new args

**File:** `src/prosthesis_launch/launch/pipeline.launch.py`

- [ ] **2.1** Add `--link-frame arm_d435i_arm_link` to the `publish_camera_mounts.py` ExecuteProcess command (line 211-213):
  ```python
  camera_nodes.append(
      ExecuteProcess(
          cmd=["python3", mounts_script,
               "--mount", camera_mount,
               "--config", mounts_config,
               "--link-frame", "arm_d435i_arm_link"],
          name="camera_mount_tf_publisher",
          output="screen",
      )
  )
  ```

- [ ] **2.2** Verify this doesn't break the standalone viz target in `Makefile.rviz` (which uses `--mount` without `--link-frame`, so it should default to the old `world` root behavior... actually, we need to decide the default. If the default is `arm_d435i_arm_link`, standalone viz breaks. If the default is `world`, the pipeline needs to explicitly pass `--link-frame`.)

  **Recommendation:** Default `--link-frame` to `""` (empty). When empty, use `world -> palm_frame` root (backward compatible for standalone viz). When set, use the link frame as root (for live pipeline).

### Phase 3: Verify pruning boxes work

- [ ] **3.1** After the TF tree is connected, verify that the fusion node can successfully look up `marker_map -> palm_frame` and `marker_map -> d435i_arm_bottom_screw_frame_8_cm_cam_mount`.

- [ ] **3.2** The warning `"Cannot transform to palm_frame for bbox removal — skipping"` should stop appearing in the fusion node logs.

- [ ] **3.3** The `bbox_removed` stat should increase when the hand is in view.

### Phase 4: Use `grasp_contact_frame` for twist propagation (now accessible via TF)

Now that `grasp_contact_frame` is in the `marker_map` tree, the twist propagation node can look it up via TF.

**File:** `src/twist_propagation/twist_propagation/twist_propagation_node.py`

- [ ] **4.1** Add parameter `propagation_frame` (default: `""` — use the pose frame as-is). When set to a frame ID (e.g., `"grasp_contact_frame"`), the node looks up the transform from the pose frame to this frame and applies it to the pose position.

- [ ] **4.2** In `_run_idle_cycle` (around line 1100-1106), after getting the latest pose:
  ```python
  if self._propagation_frame:
      # Look up T(pose_frame -> propagation_frame)
      try:
          t = self._tf_buffer.lookup_transform(
              self._propagation_frame, pose_frame, Time())
          # The offset from pose origin to grasp contact in marker_map
          ox = t.transform.translation.x
          oy = t.transform.translation.y
          oz = t.transform.translation.z
          # Rotate offset by pose orientation and add to position
          qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
          # ... rotation matrix application ...
          px += r00*ox + r01*oy + r02*oz
          py += r10*ox + r11*oy + r12*oz
          pz += r20*ox + r21*oy + r22*oz
      except Exception:
          self.get_logger().warn(
              f"Cannot look up {pose_frame} -> {self._propagation_frame}, "
              f"using raw pose", throttle_duration_sec=5.0)
  ```
  
  Wait — this isn't quite right. The TF `pose_frame -> propagation_frame` gives us the transform from `marker_map` to `grasp_contact_frame`, which is the **position of grasp_contact_frame origin expressed in marker_map**. But what we want is the **offset from the camera position to the grasp contact point, in marker_map**.
  
  Actually, since the pose IS the camera position in marker_map, and `grasp_contact_frame` is a child of the camera tree, the transform `marker_map -> grasp_contact_frame` gives us the grasp contact position in marker_map. We should use THIS as the propagation origin, not the camera position.
  
  So the simpler approach: **look up `marker_map -> grasp_contact_frame` and use that position directly** instead of the camera position.

- [ ] **4.3** Revised approach for Phase 4: In `_run_idle_cycle`, if `propagation_frame` is set:
  ```python
  if self._propagation_frame:
      try:
          t = self._tf_buffer.lookup_transform(
              pose_frame, self._propagation_frame, Time())
          # Override the pose position with the grasp contact position
          px = t.transform.translation.x
          py = t.transform.translation.y
          pz = t.transform.translation.z
          # Keep the orientation from the original pose (camera orientation)
      except Exception:
          self.get_logger().warn(...)
  ```
  
  **Wait — this is wrong too.** The TF lookup `marker_map -> grasp_contact_frame` gives us the position of the grasp contact frame origin in marker_map. But this position is computed from the **static** TF chain, not from the dynamic hand pose. The dynamic hand pose comes from OpenVINS (`/hand_pose`), which tracks the camera. The `grasp_contact_frame` position in marker_map would only be correct if `palm_frame` is dynamically updated from OpenVINS.
  
  **But it IS dynamically updated!** After Phase 1, the chain is:
  ```
  marker_map -> arm_imu -> arm_cam0 -> arm_d435i_arm_link -> screw_frame -> palm_frame -> grasp_contact_frame
  ```
  
  The `arm_cam0 -> arm_d435i_arm_link` edge is dynamic (from the bridge, which re-publishes at 0.2 Hz). So `grasp_contact_frame` in `marker_map` IS dynamically updated as the hand moves.
  
  However, the twist propagation node receives the hand pose from `/hand_pose` (OpenVINS odometry), which has its own timestamp and might be slightly different from the TF timestamp. This could cause jitter.
  
  **Better approach:** Keep using `/hand_pose` for the pose, but add the offset. The offset from camera to grasp contact is a **constant** rigid transform (the camera is bolted to the hand). We can compute it once at startup from the YAML config (as in the previous plan's Approach A), or look it up from TF once at startup.
  
  **Simplest correct approach:** Look up `arm_cam0 -> grasp_contact_frame` once at startup (it's a static transform chain). Store the translation offset. Apply it to the pose position (rotated by pose orientation) each cycle.

- [ ] **4.4** Final approach: Add `grasp_contact_offset` parameter (default: `""`). If set to a frame ID (e.g., `"grasp_contact_frame"`), look up `arm_cam0 -> grasp_contact_frame` once at startup, store the translation as the offset, and apply it in each cycle.

### Phase 5: Update config and launch

- [ ] **5.1** Add to `prosthesis_config.yaml` twist_propagation section:
  ```yaml
  grasp_contact_offset: "grasp_contact_frame"  # frame to look up offset from
  ```

- [ ] **5.2** Update `pipeline.launch.py` to pass `grasp_contact_offset` to the twist propagation node.

- [ ] **5.3** Update `twist_propagation.yaml` default config with documentation.

### Phase 6: Verify

- [ ] **6.1** `ros2 run tf2_ros tf2_echo marker_map grasp_contact_frame` should return a valid transform (not an error).
- [ ] **6.2** `ros2 run tf2_ros tf2_echo marker_map palm_frame` should return a valid transform.
- [ ] **6.3** Fusion node bbox pruning should work (no more "Cannot transform" warnings).
- [ ] **6.4** Twist propagation `/twist_propagation/current_pose` should appear at the grasp contact point.
- [ ] **6.5** Standalone viz (`make -f Makefile.rviz mounts-viz MOUNT=8_cm_cam_mount`) should still work (uses `world` root).

---

## 3. Files Changed Summary

| File | Action | Scope |
|------|--------|-------|
| `src/sensor_fusion_bringup/scripts/publish_camera_mounts.py` | Restructure TF tree, add `--link-frame` arg | ~30 lines modified |
| `src/prosthesis_launch/launch/pipeline.launch.py` | Pass `--link-frame` to mounts publisher | ~2 lines |
| `src/twist_propagation/twist_propagation/twist_propagation_node.py` | Add grasp contact offset via TF | ~40 lines |
| `config/prosthesis_config.yaml` | Add `grasp_contact_offset` param | +1 line |
| `src/twist_propagation/config/twist_propagation.yaml` | Add param with docs | +3 lines |
| `Makefile.rviz` | No change needed (default `--link-frame ""` uses `world` root) | 0 |

---

## 4. Risks and Mitigations

1. **TF tree cycle**: If both `arm_d435i_arm_link -> screw_frame` AND `screw_frame -> link_frame` are published, and `link_frame = d435i_arm_link_8_cm_cam_mount` is similar to `arm_d435i_arm_link`, TF2 might complain about duplicate paths.
   **Mitigation:** The frame IDs are different (`arm_d435i_arm_link` vs `d435i_arm_link_8_cm_cam_mount`), so there's no cycle. TF2 handles diamond shapes fine.

2. **Bridge timing**: The bridge re-publishes `arm_cam0 -> arm_d435i_arm_link` at 0.2 Hz for liveness. The `publish_camera_mounts.py` publishes static TFs. The static TFs under `arm_d435i_arm_link` will only be resolvable when the bridge is running.
   **Mitigation:** This is already the case for the RealSense frames. The mounts tree just adds more children to the same link frame.

3. **Standalone viz regression**: If the default `--link-frame` is empty (uses `world` root), standalone viz works as before. The pipeline explicitly passes `--link-frame arm_d435i_arm_link`.
   **Mitigation:** Default is safe. Pipeline is explicit.

4. **`screw_to_link` inversion**: The transform `arm_d435i_arm_link -> screw_frame` is the inverse of `screw_to_link = (0.0106, 0.0175, 0.0125)` with identity rotation. The inverse is `(-0.0106, -0.0175, -0.0125)` with identity rotation. This is a pure translation.
   **Mitigation:** Simple math, easy to verify in RViz.

5. **`screw_to_palm` direction change**: Currently published as `palm -> screw` (inverted). Need to change to `screw -> palm` (direct). The quaternion and translation values come directly from the YAML `screw_to_palm` entry.
   **Mitigation:** Use the values directly from `data["mounts"][mount_name]["screw_to_palm"]` without calling `_invert_transform`.

---

## 5. Key Insight

The **most important fix** is Phase 1 (restructuring the TF tree). This alone fixes:
- Pruning boxes in the fusion node (they start working)
- Makes `grasp_contact_frame` accessible from `marker_map`
- Makes `palm_frame` accessible from `marker_map`

Phase 4 (twist propagation) is a separate improvement that becomes possible **because** Phase 1 fixed the TF tree.
