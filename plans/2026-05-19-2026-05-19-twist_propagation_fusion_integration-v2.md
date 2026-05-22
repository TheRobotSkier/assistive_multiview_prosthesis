# Integrate Pointcloud Fusion into twist_propagation_test.launch.py

## Objective

Add the `pointcloud_fusion_node` to `twist_propagation_test.launch.py` so that:
- Both Jetson camera clouds (head + arm) are fused into a single `marker_map`-frame cloud
- The fused cloud is filtered (distance, hand removal, downsampled)
- Segmentation bridge and twist propagation both consume `/fused_pointcloud`

## Findings

### Topics (from Jetson, confirmed by user)

| Camera | Topic | Frame |
|---|---|---|
| Head | `/head/d435i/head/depth/color/points` | `head_d435i_head_depth_frame` |
| Arm | `/arm/d435i/arm/depth/color/points` | `arm_d435i_arm_depth_frame` |

**Note**: User specified singular "point" but all existing codebase references use plural "points". Using plural as per existing convention.

### Target frame: `marker_map`
- Confirmed by `rviz/twist_propagation.rviz:199` (Fixed Frame: `marker_map`)
- The OpenVINS TF tree has `marker_map` as root (confirmed by `rviz/phase2_dual_openvins_head_preview.rviz:94`)
- The Jetson publishes TFs: `marker_map → arm_d435i_arm_*` frames

### Arm frame for collision prediction: `arm_d435i_arm_depth_frame`
- User-specified, will change later
- This is the frame used for distance filtering and bbox removal

### Current topic wiring (BEFORE)

```
Jetson → /arm/d435i_arm/points_marker_map ──→ segmentation_bridge (remap)
                                            ──→ twist_propagation (input_cloud_topic)
```

Both segmentation and twist propagation subscribe to the SAME raw single-camera topic.

### Target topic wiring (AFTER)

```
Jetson → /head/d435i/head/depth/color/points ──┐
Jetson → /arm/d435i/arm/depth/color/points ────┤
                                                ↓
                                      pointcloud_fusion_node
                                                ↓
                                      /fused_pointcloud (marker_map frame)
                                                ↓
                                      pointcloud_relay_node
                                                ↓
                                      /segmentation/input_cloud ──→ segmentation_bridge

                                      /fused_pointcloud ──→ twist_propagation
```

### Key observations

1. **The old default topic `/arm/d435i_arm/points_marker_map` is abandoned** — the fusion node subscribes to the raw camera topics (`/head/d435i/head/depth/color/points` and `/arm/d435i/arm/depth/color/points`) and does its own TF transform to `marker_map`.

2. **The fusion node already handles this** — it subscribes to `cam1_topic` + `cam2_topic`, transforms both to `target_frame` via TF2. All parameters are configurable.

3. **No changes needed to the fusion node code** — it's fully parameterized. Only the launch file needs updating.

4. **The `pointcloud_relay_node`** (`src/camera/camera/pointcloud_relay_node.py:1-37`) bridges `/fused_pointcloud` → `/segmentation/input_cloud` so the segmentation bridge doesn't need remapping.

5. **Segmentation bridge currently uses remapping** to override `/segmentation/input_cloud`. With the relay node, we can remove the remapping and let it use the default topic.

6. **The `input_cloud_topic` launch argument** in the current launch file is used by both segmentation (remap) and twist propagation (param). After fusion, twist propagation subscribes to `/fused_pointcloud` directly, and segmentation gets `/segmentation/input_cloud` from the relay. The `input_cloud_topic` argument becomes unnecessary for the fusion case.

## Implementation Plan

- [ ] 1. **Update docstring** — Add pointcloud fusion node and relay to the node list, update the description to mention dual-camera fusion in `marker_map` frame
- [ ] 2. **Add pointcloud fusion node** — Insert `pointcloud_fusion_node` from `pointcloud_fusion` package with parameters:
  - `target_frame`: `marker_map`
  - `cam1_topic`: `/head/d435i/head/depth/color/points`
  - `cam2_topic`: `/arm/d435i/arm/depth/color/points`
  - `arm_frame`: `arm_d435i_arm_depth_frame`
  - `max_distance`: `2.0`
  - `voxel_size`: `0.005`
  - `bbox_min`: `[-0.30, -0.10, -0.10]`
  - `bbox_max`: `[0.22, 0.10, 0.12]`
- [ ] 3. **Add pointcloud relay node** — `pointcloud_relay_node` from `camera` package to bridge `/fused_pointcloud` → `/segmentation/input_cloud`
- [ ] 4. **Update segmentation bridge** — Remove the `remappings` for `/segmentation/input_cloud` since the relay node now provides it on the default topic
- [ ] 5. **Update twist propagation** — Change `input_cloud_topic` parameter from the launch argument to `/fused_pointcloud`
- [ ] 6. **Update `input_cloud_topic` default** — Change default from `/arm/d435i_arm/points_marker_map` to `/fused_pointcloud` (for backward compat if fusion is disabled, but this launch always uses fusion now)
- [ ] 7. **Update RViz config** — Change `rviz/twist_propagation.rviz:93` Scene PointCloud topic from `/arm/d435i_arm/points_marker_map` to `/fused_pointcloud`
- [ ] 8. **Verify syntax** — Parse the launch file and RViz config for errors

## Verification Criteria

- [ ] Launch file parses without syntax errors
- [ ] Fusion node subscribes to both `/head/d435i/head/depth/color/points` and `/arm/d435i/arm/depth/color/points`
- [ ] Fusion node publishes on `/fused_pointcloud` with `frame_id=marker_map`
- [ ] Relay node bridges `/fused_pointcloud` → `/segmentation/input_cloud`
- [ ] Segmentation bridge subscribes to `/segmentation/input_cloud` (default, no remap)
- [ ] Twist propagation subscribes to `/fused_pointcloud`
- [ ] RViz Scene PointCloud displays `/fused_pointcloud`

## Potential Risks and Mitigations

1. **TF not available from Jetson for head camera** — If the Jetson only publishes TF for `arm_d435i_arm_*` frames and not `head_d435i_head_*`, the head cloud can't be transformed. The fusion node handles this gracefully (warns, skips that cloud).
   Mitigation: Verify TF tree on Jetson includes `marker_map → head_d435i_head_depth_frame`. If not, add a static TF or configure OpenVINS to publish it.

2. **Topic name mismatch** — User wrote singular "point" but all codebase uses plural "points". 
   Mitigation: Using plural "points" as per established convention. User can override via launch arg if needed.

3. **QoS mismatch** — Jetson may publish with RELIABLE QoS while the fusion node subscribes BEST_EFFORT.
   Mitigation: The fusion node uses BEST_EFFORT which is compatible with both RELIABLE and BEST_EFFORT publishers.

## Files to Modify

| File | Change |
|---|---|
| `src/prosthesis_launch/launch/twist_propagation_test.launch.py` | Add fusion node, relay node, update wiring |
| `rviz/twist_propagation.rviz` | Update Scene PointCloud topic to `/fused_pointcloud` |

## Files NOT Modified

| File | Reason |
|---|---|
| `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` | Already fully parameterized, no changes needed |
| `src/camera/camera/pointcloud_relay_node.py` | Already hardcoded `/fused_pointcloud` → `/segmentation/input_cloud`, no changes needed |
| `src/segmentation/segmentation_bridge/segmentation_ros2_node.py` | Already subscribes to `/segmentation/input_cloud` by default |
| `src/twist_propagation/twist_propagation/twist_propagation_node.py` | Topic is set via parameter from launch file |
