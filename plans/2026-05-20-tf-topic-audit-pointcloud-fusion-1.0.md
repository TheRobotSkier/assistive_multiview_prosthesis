# TF Topic Audit for Point Cloud Fusion Node

## Objective

Verify that the `pointcloud_fusion_node` uses the correct TF frames for transforming dual-camera point clouds into a common world frame, and identify any mismatches between the frames the RealSense cameras actually publish vs. what the fusion node expects.

## Findings

### 1. The Fusion Node's TF Strategy

The fusion node (`src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py:345-375`) works as follows:

- It receives PointCloud2 messages from both cameras.
- For each cloud, it reads `cloud.header.frame_id` (set by the RealSense driver).
- It calls `tf_buffer.lookup_transform(target_frame, cloud.header.frame_id, ...)` to transform each cloud into the target frame.
- **target_frame** = `"marker_map"` (from config).
- **arm_frame** = `"arm_d435i_arm_depth_frame"` (from config, used for distance filtering and bbox removal).

### 2. What frame_id do the RealSense PointCloud2 messages carry?

The RealSense `realsense2_camera_node` publishes `/depth/color/points` with `header.frame_id` set to the **depth optical frame**. For the Jetson camera stack with namespace/prefix conventions:

| Camera | Topic | PointCloud2 header.frame_id |
|--------|-------|----------------------------|
| Head | `/head/d435i_head/depth/color/points` | `head_d435i_head_depth_optical_frame` |
| Arm | `/arm/d435i_arm/depth/color/points` | `arm_d435i_arm_depth_optical_frame` |

### 3. The Full TF Chain Required

For the fusion node to transform both clouds to `marker_map`, these chains must exist:

```
marker_map -> head_imu -> head_cam0 -> head_d435i_head_link -> head_d435i_head_depth_frame -> head_d435i_head_depth_optical_frame
marker_map -> arm_imu  -> arm_cam0  -> arm_d435i_arm_link   -> arm_d435i_arm_depth_frame   -> arm_d435i_arm_depth_optical_frame
```

### 4. How the TF Chain is Connected

The `openvins_realsense_tf_bridge_node` (`src/camera/camera/openvins_realsense_tf_bridge_node.py`) publishes the critical missing links:

- `head_cam0 -> head_d435i_head_link` (dynamic, 15 Hz)
- `arm_cam0 -> arm_d435i_arm_link` (dynamic, 15 Hz)

Plus the nominal static fan-out below `*_link` (depth_frame, color_frame, optical frames, etc.).

### 5. Assessment: Are the frames correct?

**YES — the frames are correct for the production pipeline (`pipeline.launch.py`).** Here's the verification:

- **target_frame = `"marker_map"`**: Correct. This is the root of the OpenVINS SLAM tree, and the common reference frame for all downstream consumers (segmentation, twist propagation, grasp preshaping).

- **cam1_topic = `"/head/d435i_head/depth/color/points"`**: Correct. Matches the Jetson RealSense topic naming.

- **cam2_topic = `"/arm/d435i_arm/depth/color/points"`**: Correct. Same pattern for the arm camera.

- **arm_frame = `"arm_d435i_arm_depth_frame"`**: Correct for distance filtering and bbox removal. This frame is in the RealSense TF subtree, connected to `marker_map` via the bridge. The fusion node looks up `marker_map -> arm_d435i_arm_depth_frame` for:
  - Distance filter: gets the arm camera's origin in `marker_map` to cull far-away points (`pointcloud_fusion_node.py:424`).
  - Bbox removal: transforms all fused points from `marker_map` into `arm_d435i_arm_depth_frame` coordinates to apply the AABB crop (`pointcloud_fusion_node.py:443-446`).

- The PointCloud2 messages arrive with `frame_id = *_depth_optical_frame`, and the TF chain from `marker_map` to `*_depth_optical_frame` is fully connected via the bridge.

### 6. Digital Twin Launch Uses Different Frames (Correctly)

The `digital_twin.launch.py` overrides the fusion parameters:

```python
# digital_twin.launch.py:163
'target_frame': 'world',
'arm_frame': 'wrist_link',
```

This is correct for the digital twin scenario which uses ChArUco tracking instead of OpenVINS, and has a different TF tree rooted at `world` with `wrist_link` as the arm reference.

### 7. One Minor Observation: arm_frame depth_frame vs depth_optical_frame

The `arm_frame` parameter is set to `arm_d435i_arm_depth_frame` (not `_optical`). This is intentional and correct:

- The bbox coordinates (`bbox_min`, `bbox_max`) are specified in the `depth_frame` coordinate system (which has the same orientation as the camera link, not the optical frame's rotated convention).
- Using the optical frame would require the bbox to be specified in the rotated optical coordinate system (Z-forward, X-right, Y-down), which would be less intuitive for specifying hand/arm bounding boxes.

## Verification Criteria

- [x] `target_frame` matches the OpenVINS root frame (`marker_map`)
- [x] Camera topics match the Jetson RealSense published topics
- [x] `arm_frame` is reachable from `marker_map` via the TF bridge
- [x] PointCloud2 `header.frame_id` values (`*_depth_optical_frame`) are reachable from `marker_map`
- [x] Digital twin launch correctly overrides to its own frame tree
- [x] The TF bridge publishes both the dynamic `cam0 -> link` edge AND the static fan-out

## Potential Risks and Mitigations

1. **Jetson `/tf_static` not crossing DDS reliably**
   Mitigation: The bridge node re-publishes the nominal static chain both via `StaticTransformBroadcaster` and periodically on the dynamic `/tf` topic (`openvins_realsense_tf_bridge_node.py:343-346`). This is already handled.

2. **Bridge not running when fusion node starts**
   Mitigation: The fusion node uses `can_transform` with a 50ms timeout and logs warnings on TF failures. It gracefully skips clouds it can't transform. Stats are logged every 10 seconds.

3. **Frame name changes if Jetson camera config changes**
   Mitigation: All frame names are parameterized via `prosthesis_config.yaml` and launch arguments, not hardcoded.

## Summary

**The TF topics and frames used by the point cloud fusion node are correct.** The node properly uses:
- `marker_map` as the target/world frame (matching the OpenVINS SLAM root)
- The RealSense depth optical frames (which arrive in PointCloud2 headers) as source frames
- `arm_d435i_arm_depth_frame` as the arm reference frame for distance filtering and bbox removal
- The `openvins_realsense_tf_bridge_node` correctly bridges the OpenVINS camera frames to the RealSense TF trees

No changes are needed.
