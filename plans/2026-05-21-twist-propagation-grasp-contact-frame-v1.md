# Change Twist Propagation Frame to grasp_contact_frame

## Objective

Move the twist propagation origin from the OpenVINS odometry frame (where `/hand_pose` arrives) to `grasp_contact_frame`, which is closer to the fingertips of the Mia hand. This means the propagation trajectory starts at the grasp contact point rather than the camera/IMU origin, giving a more accurate prediction of where the hand will contact objects.

## Current Architecture

### How `/hand_pose` flows into twist propagation

1. **`odom_to_pose_relay.py`** (`src/camera/camera/odom_to_pose_relay.py:64-68`) subscribes to OpenVINS odometry (`/ov_msckf_arm/odomimu`) and republishes the pose as `/hand_pose` **without changing the frame**. The frame comes from the OpenVINS odom `header.frame_id`, which is `marker_map`.

2. **`twist_propagation_node.py`** subscribes to `/hand_pose` at line 477. In `_on_hand_pose` (line 578-588), it stores the pose position, orientation, and `frame_id` into the pose buffer.

3. In `_run_idle_cycle` (line 1067), it reads the latest pose from the buffer (line 1101-1102), then transforms the position to the cloud frame via `_transform_pose_to_cloud_frame` (line 1105). The cloud frame comes from the `/fused_pointcloud` header, which is `marker_map`.

4. The twist is published in `pose_frame` (the original frame from `/hand_pose`, i.e. `marker_map`) — see line 1112.

### TF tree (live pipeline)

```
marker_map (OpenVINS world frame)
  ├── head_imu -> head_cam0 -> head_d435i_head_link -> ... (via bridge)
  └── arm_imu -> arm_cam0
        └── arm_d435i_arm_link (via bridge)
              └── arm_d435i_arm_depth_frame -> ... (RealSense static chain)

publish_camera_mounts.py adds:
  world -> palm_frame -> grasp_contact_frame
  world -> palm_frame -> d435i_arm_bottom_screw_frame_8_cm_cam_mount -> ...
```

### Key problem: `marker_map` vs `world`

`publish_camera_mounts.py` publishes `world -> palm_frame -> grasp_contact_frame` as a static TF. But the live pipeline uses `marker_map` as the world frame (from OpenVINS). The TF chain `marker_map -> palm_frame` does NOT exist unless `world == marker_map`.

In the live system, OpenVINS dynamically publishes the camera pose in `marker_map`. The bridge node connects `arm_cam0 -> arm_d435i_arm_link`. So the full chain is:

```
marker_map -> arm_imu -> arm_cam0 -> arm_d435i_arm_link -> arm_d435i_arm_depth_frame
```

But `palm_frame` is published as a child of `world` (not `marker_map`). Unless OpenVINS or another node also publishes `marker_map -> palm_frame`, the `grasp_contact_frame` is disconnected from the `marker_map` tree.

## Findings

### 1. The `grasp_contact_frame` TF is published by `publish_camera_mounts.py`

Verified at `src/sensor_fusion_bringup/scripts/publish_camera_mounts.py:220-227`:
```python
def _build_grasp_contact_tf(self, data: dict) -> list[TransformStamped]:
    s = self._stamp
    gc = data["grasp_contact"]["palm_to_grasp_contact"]
    return [
        _make_tf(s, "palm_frame", "grasp_contact_frame",
                 _t(gc, "x"), _t(gc, "y"), _t(gc, "z"),
                 *_q(gc)),
    ]
```

The transform is `palm_frame -> grasp_contact_frame` at translation `(0.0, 0.13, 0.03)` — 13cm forward and 3cm up from the palm origin, roughly at the fingertips.

### 2. TF chain gap: `marker_map -> palm_frame` is missing

The `publish_camera_mounts.py` publishes `world -> palm_frame` (line 215-217). In the live pipeline, `marker_map` is the root frame, not `world`. These are the same physical frame (both are the SLAM world origin), but they have different frame IDs.

**Solution options:**
- **Option A**: Change `publish_camera_mounts.py` to publish `marker_map -> palm_frame` instead of `world -> palm_frame`. This would break standalone visualization (`make -f Makefile.rviz mounts-viz` uses `world` as the fixed frame).
- **Option B**: Add a `--world-frame` argument to `publish_camera_mounts.py` that defaults to `world` but can be overridden to `marker_map` in the pipeline launch.
- **Option C**: Publish `marker_map -> palm_frame` as an additional static TF in the pipeline (e.g., identity transform). This creates a frame alias.
- **Option D**: In the twist propagation node, instead of using `grasp_contact_frame` directly, compute the `grasp_contact_frame` offset at startup from the YAML config and apply it as a local transform after looking up `marker_map -> arm_d435i_arm_link`.

### 3. How the twist propagation uses the pose frame

The node uses the pose frame in two ways:
- **`_transform_pose_to_cloud_frame`** (line 782-830): Transforms the hand position from `pose_frame` to `cloud_frame` (`marker_map`) via TF2. If `pose_frame == cloud_frame`, no transform is needed.
- **`_publish_twist`** (line 687-694): Publishes the twist in `pose_frame`.

The propagation itself (line 1117-1120) operates in `cloud_frame` coordinates. So changing the "origin" of propagation means changing where the initial pose position comes from — we need to transform the hand pose to `grasp_contact_frame` first, then to `cloud_frame`.

### 4. The odom_to_pose_relay doesn't transform frames

At `odom_to_pose_relay.py:66-68`:
```python
ps = PoseStamped()
ps.header = msg.header  # frame_id comes from OpenVINS (marker_map)
ps.pose = msg.pose.pose
```

The pose arrives in `marker_map` frame. To move the origin to `grasp_contact_frame`, we need an additional TF step.

## Recommended Approach

**Option D is the cleanest**: The twist propagation node already has TF2 support. We add a new parameter `propagation_origin_frame` (default: `""`, meaning use the pose frame directly). When set to `grasp_contact_frame`, the node:

1. Receives `/hand_pose` in `marker_map` frame (as before)
2. Looks up `marker_map -> grasp_contact_frame` to get the grasp contact position
3. Uses that as the propagation origin instead of the raw hand pose position
4. The twist estimation still uses the raw poses (velocity is frame-independent for translation)
5. Propagation starts from the grasp contact point, not the camera

This requires the TF chain `marker_map -> palm_frame -> grasp_contact_frame` to be resolvable. We need to fix the `world -> palm_frame` gap.

**For the TF gap**, **Option B** is best: add `--world-frame` to `publish_camera_mounts.py`. In the pipeline launch, pass `--world-frame marker_map`. In standalone viz, use the default `world`.

## Implementation Plan

### Phase 1: Fix TF chain — add `--world-frame` to `publish_camera_mounts.py`

- [ ] **1.1** Add `--world-frame` argument to the argparse in `main()` (default: `"world"`)
- [ ] **1.2** Pass `world_frame` to `CameraMountTFPublisher.__init__` and store as `self._world_frame`
- [ ] **1.3** In `_build_shared_tfs`, use `self._world_frame` instead of hardcoded `"world"` as the parent frame for `palm_frame`
- [ ] **1.4** In `_make_bbox_marker` and `_make_cam_bbox_marker`, keep `"palm_frame"` as the marker frame (unchanged — markers are in palm_frame, not world)
- [ ] **1.5** Update `pipeline.launch.py` to pass `--world-frame marker_map` when launching `publish_camera_mounts.py`

### Phase 2: Add `propagation_origin_frame` parameter to twist propagation

- [ ] **2.1** In `twist_propagation_node.py`, add parameter `propagation_origin_frame` (default: `""`)
- [ ] **2.2** In `_run_idle_cycle`, after getting the latest pose and transforming it to cloud frame:
  - If `propagation_origin_frame` is set, look up the TF from `cloud_frame` to `propagation_origin_frame`
  - Use the `propagation_origin_frame` origin position as the propagation start point
  - Keep the orientation from the original pose (the grasp contact frame has identity rotation relative to palm, so orientation is still useful)
- [ ] **2.3** Add a helper `_get_frame_origin_in_cloud_frame(frame)` that looks up the origin of a frame in the cloud frame via TF2
- [ ] **2.4** Update startup log to show the propagation origin frame

### Phase 3: Update config and launch

- [ ] **3.1** Add `propagation_origin_frame: "grasp_contact_frame"` to `prosthesis_config.yaml` under `twist_propagation`
- [ ] **3.2** Add `propagation_origin_frame` to `twist_propagation/config/twist_propagation.yaml` with default `""`
- [ ] **3.3** No changes needed to `pipeline.launch.py` for the twist propagation node — it already passes params from config

### Phase 4: Verify TF chain

- [ ] **4.1** With `publish_camera_mounts.py --mount 8_cm_cam_mount --world-frame marker_map` running, verify `marker_map -> palm_frame -> grasp_contact_frame` is resolvable
- [ ] **4.2** Verify the twist propagation node can look up `marker_map -> grasp_contact_frame` at runtime
- [ ] **4.3** Verify the propagation trajectory now starts at the grasp contact point (13cm forward, 3cm up from palm)

## Verification Criteria

- `propagation_origin_frame: "grasp_contact_frame"` causes the twist propagation to start from the fingertip region instead of the camera/IMU
- `propagation_origin_frame: ""` (default) preserves existing behavior exactly
- `make -f Makefile.rviz mounts-viz MOUNT=8_cm_cam_mount` still works (uses `world` frame)
- The TF chain `marker_map -> palm_frame -> grasp_contact_frame` is resolvable when the pipeline is running
- The hit point and trajectory visualization are correct in RViz

## Potential Risks and Mitigations

1. **`marker_map` and `world` are different frames in some configurations**
   Mitigation: The `--world-frame` argument makes this explicit. If someone runs the pipeline without `publish_camera_mounts.py`, the TF chain won't exist and the node falls back gracefully.

2. **Static `palm_frame` vs dynamic `palm_frame`**
   `publish_camera_mounts.py` publishes `marker_map -> palm_frame` as a static identity. But in the real system, the hand moves. The twist propagation needs the DYNAMIC position of `grasp_contact_frame`, not the static one.
   **This is a critical issue.** The `grasp_contact_frame` from `publish_camera_mounts.py` is static — it doesn't move with the hand. OpenVINS publishes the camera pose dynamically in `marker_map`, but `palm_frame` is a fixed offset from `world`.
   
   **Fix**: Instead of using `grasp_contact_frame` from TF, we should compute the grasp contact offset in the **camera frame** and apply it to the dynamic pose. The camera-to-palm transform is known from `camera_mounts.yaml`, so we can compute `palm_frame -> grasp_contact_frame` offset and apply it in the camera's local frame.

   Alternative: Use the dynamic `arm_cam0 -> arm_d435i_arm_link` chain (from OpenVINS + bridge) to get the camera's live position, then apply the screw-to-palm + palm-to-grasp-contact offsets locally.

3. **Orientation at grasp contact point**
   The grasp contact frame has identity rotation relative to palm_frame. The propagation uses orientation for angular velocity propagation. This should be fine — the orientation comes from the hand pose, and we only shift the position origin.

## Alternative Approaches

1. **Transform in odom_to_pose_relay**: Instead of changing twist propagation, transform the pose in the relay node from `marker_map` to `grasp_contact_frame` before publishing. This would change the frame of `/hand_pose`, which might affect other subscribers.

2. **Add a new relay node**: Create a `grasp_contact_pose_relay` that subscribes to `/hand_pose`, applies the grasp contact offset, and publishes `/hand_pose_at_grasp`. The twist propagation subscribes to this instead. Clean separation but adds another node.

3. **Pure config approach**: Don't use TF at all for the grasp contact offset. Add a parameter `propagation_origin_offset: [0.0, 0.13, 0.03]` to the twist propagation node and apply it in the camera frame. Simpler but less general.
