# Integrate Pointcloud Fusion into twist_propagation_test.launch.py

## Objective

Add the `pointcloud_fusion_node` to `twist_propagation_test.launch.py` so that the twist propagation test pipeline uses the fused, filtered point cloud instead of a raw single-camera cloud. This makes the test representative of the full digital twin pipeline.

## Context

The twist propagation test launch (`src/prosthesis_launch/launch/twist_propagation_test.launch.py`) is designed to run on the host machine, connecting to a Jetson that provides camera + OpenVINS data. Currently:

- It subscribes to a single cloud topic (`/arm/d435i_arm/points_marker_map` by default — a Jetson-side OpenVINS-aligned cloud)
- Segmentation bridge remaps to that same topic
- Twist propagation subscribes to that same topic
- There is **no** pointcloud fusion, no distance filtering, no hand removal

The digital twin launch (`digital_twin.launch.py`) already has the fusion node integrated. We need a similar integration here, adapted for the Jetson context.

## Key Differences from digital_twin.launch.py

| Aspect | digital_twin.launch.py | twist_propagation_test.launch.py |
|---|---|---|
| **Camera source** | Local RealSense D435i cameras | Jetson streaming over network |
| **Cloud topics** | `/head/d435i_head/depth/color/points` + `/arm/d435i_arm/depth/color/points` | `/arm/d435i_arm/points_marker_map` (head topic TBD) |
| **TF source** | Local ChArUco + static TFs | Jetson OpenVINS (publishes `marker_map` frame TFs over network) |
| **Target frame** | `world` (static TF from cam1) | `marker_map` (from OpenVINS on Jetson) |
| **Arm frame** | `wrist_link` (ChArUco tracked) | `wrist_link` (from OpenVINS on Jetson) |

## Critical Questions for Investigation

Before implementation, Forge needs to determine:

1. **What topics does the Jetson stream?** — The current default is `/arm/d435i_arm/points_marker_map`. Does the Jetson also stream a head camera cloud? If so, what's the topic name?
2. **What TF frames are available over the network?** — The fusion node needs TF from cloud frame → target frame. With OpenVINS, the target frame is likely `marker_map`. Does the Jetson publish `marker_map→d435i_arm_depth_optical_frame` and `marker_map→d435i_head_depth_optical_frame`?
3. **Is there a head camera on the Jetson?** — If only the arm camera streams from the Jetson, the fusion node would only have one cloud and fusion is pointless. In that case, we might still want the filtering (distance, bbox, downsampling) but not the merge step.
4. **What is `points_marker_map`?** — This appears to be a Jetson-side node that transforms the arm camera cloud into the `marker_map` frame. If the Jetson already does this, our fusion node's TF transform step would be redundant for the arm cloud.

## Implementation Plan

### Phase 1: Investigation (Forge)

- [ ] 1. Check what topics the Jetson streams by examining Jetson launch files and OpenVINS config
- [ ] 2. Check what TF frames the Jetson publishes over DDS (examine OpenVINS launch/config)
- [ ] 3. Determine if a head camera cloud is available from the Jetson
- [ ] 4. Understand the `points_marker_map` topic — is it already TF-transformed?
- [ ] 5. Check if `marker_map` frame is the correct target frame for the fusion node

### Phase 2: Implementation (Forge)

Based on findings, the integration will likely follow one of these patterns:

**Scenario A: Both cameras stream from Jetson**
- [ ] 6. Add `pointcloud_fusion_node` to the launch with Jetson topic names
- [ ] 7. Set `target_frame` to `marker_map` (or whatever OpenVINS provides)
- [ ] 8. Set `arm_frame` to `wrist_link` (provided by OpenVINS on Jetson)
- [ ] 9. Update segmentation bridge remapping to use `/fused_pointcloud`
- [ ] 10. Update twist propagation `input_cloud_topic` to `/fused_pointcloud`
- [ ] 11. Add pointcloud relay node (`/fused_pointcloud` → `/segmentation/input_cloud`)
- [ ] 12. Add new launch arguments for cam1/cam2 topics and target frame

**Scenario B: Only arm camera from Jetson (single cloud)**
- [ ] 6. Still add `pointcloud_fusion_node` but with only cam2 topic (or a dummy cam1)
- [ ] 7. The node already handles single-cloud gracefully (cam1_only path)
- [ ] 8. Filtering (distance, bbox, downsampling) still applies
- [ ] 9. Update segmentation + twist propagation to use `/fused_pointcloud`
- [ ] 10. Add pointcloud relay node

**Scenario C: Fusion is not useful for Jetson-only setup**
- [ ] 6. Skip fusion node entirely
- [ ] 7. Add a simpler filtering node (or reuse twist propagation's built-in voxel downsample)
- [ ] 8. Document that fusion is only for the digital twin (dual local cameras)

### Phase 3: Launch file modifications

- [ ] 13. Update docstring to document the fusion integration
- [ ] 14. Add new `DeclareLaunchArgument` entries for fusion parameters
- [ ] 15. Add the `pointcloud_fusion` Node to `_launch_setup()`
- [ ] 16. Add `pointcloud_relay_node` to bridge fused → segmentation
- [ ] 17. Update segmentation bridge remapping
- [ ] 18. Update twist propagation `input_cloud_topic` default to `/fused_pointcloud`
- [ ] 19. Verify the node ordering (fusion must start before segmentation/twist)

## Verification Criteria

- [ ] Launch file parses without syntax errors
- [ ] `ros2 launch prosthesis_launch twist_propagation_test.launch.py` starts all nodes including fusion
- [ ] Fusion node subscribes to correct Jetson topics
- [ ] `/fused_pointcloud` is published with correct frame_id
- [ ] Segmentation bridge receives the fused cloud
- [ ] Twist propagation receives the fused cloud
- [ ] Distance filtering and hand removal work correctly with Jetson TF frames

## Potential Risks and Mitigations

1. **Jetson doesn't stream head camera cloud**
   Mitigation: Fusion node handles single-cloud gracefully; filtering still applies

2. **TF frames differ between Jetson and local setup**
   Mitigation: Make target_frame configurable via launch argument; default to `marker_map`

3. **Network latency causes sync issues**
   Mitigation: The fusion node already has a timer-based fallback for when message_filters can't sync

4. **`points_marker_map` is already in `marker_map` frame**
   Mitigation: The fusion node checks if cloud frame == target_frame and skips transform if so

## Files to Examine

- `src/prosthesis_launch/launch/twist_propagation_test.launch.py` — the file to modify
- `src/prosthesis_launch/launch/digital_twin.launch.py:152-177` — reference for fusion node integration
- `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` — the fusion node itself
- Jetson launch files (in `jetson/` directory or on the Jetson itself) — to determine available topics/frames
- `src/camera/camera/pointcloud_relay_node.py` — relay node to add
