# X86 Segmentation Pointcloud Plan

## Context

The Jetson should prioritize RealSense RGB/IMU, OpenVINS, marker updates, and
the `marker_map` TF tree. Pointcloud publication can be enabled for grasping,
but expensive per-point processing should move to the Ubuntu x86 PC whenever
possible.

The segmentation code on the older integration branch consumes:

```text
/segmentation/input_cloud
/segmentation/click_positive
/segmentation/click_negative
/segmentation/reset
```

It expects `sensor_msgs/msg/PointCloud2` with `x`, `y`, `z`, and preferably
packed PCL-style `rgb` fields. Unordered `height=1` clouds are acceptable.
Use PointCloud2 as the segmentation contract; voxel filtering/downsampling
should be an optional preprocessing step, not the primary message interface.

As of 2026-05-14, Jetson live validation confirmed color pointclouds are visible
in RViz2 while OpenVINS remains stable. The status topic showed
`accepted:true`/`reason:"published"` interleaved with expected
`reason:"rate_limited"` messages.

## Recommended X86 Responsibilities

- First integration path: subscribe to the already validated Jetson marker-map
  pointclouds:
  - `/head/d435i_head/points_marker_map`
  - `/arm/d435i_arm/points_marker_map`
- Optionally crop, range-filter, voxel/downsample, and merge the two clouds.
- Apply a prosthesis self-filter before collision checking or segmentation when
  user-measured prosthesis bounding boxes are available. Planning prompt:
  `.agents/prosthesis_self_filter_pointcloud_prompt_2026-05-15.md`.
- Publish the segmentation-ready cloud on:
  - `/segmentation/input_cloud`
- Keep the output cloud header frame as:
  - `marker_map`
- Preserve `x`, `y`, `z`, and `rgb` fields so the segmentation node can consume
  the cloud without format rewrites.

If Jetson load becomes too high, move more work to x86:

- Subscribe to raw Jetson RealSense color pointclouds:
  - `/head/d435i_head/depth/color/points`
  - `/arm/d435i_arm/depth/color/points`
- Subscribe to `/tf` and `/tf_static` from the Jetson.
- Transform each raw cloud into `marker_map` on the x86 PC.
- Merge the transformed clouds on the x86 PC and range-filter distant points,
  initially beyond `2.0 m`, without voxel downsampling or point subsampling.
- Planning prompt for this offload path:
  `.agents/x86_raw_pointcloud_marker_map_offload_prompt_2026-05-19.md`.

## Jetson Responsibilities

- Continue publishing OpenVINS and marker-map TF on the Jetson.
- Keep fixed ID0 and dynamic ID2 marker update behavior unchanged.
- Keep `marker_fixed_ids` ID0-only; do not add ID2.
- Keep pointclouds opt-in with `enable_pointclouds:=true`.
- Keep the current Jetson-side `points_marker_map` republishers for near-term
  RViz and segmentation integration because they are now validated.
- Prefer raw pointcloud transport to x86 later if Jetson CPU/EMC is high.

## QoS Recommendation

Use sensor-data/best-effort QoS for high-bandwidth pointcloud streams where the
subscriber supports it. This lets DDS drop old pointcloud samples instead of
building a reliable backlog that can increase Jetson memory bandwidth pressure.
Reliable QoS is still reasonable for low-rate status/control topics.

As of 2026-05-14, the Jetson marker-map pointcloud republishers use
sensor-data/best-effort QoS for their raw RealSense input subscriptions, then
publish `/head/d435i_head/points_marker_map` and
`/arm/d435i_arm/points_marker_map` with reliable shallow-queue QoS for RViz and
plain ROS 2 CLI compatibility. The x86 segmentation subscriber can use default
reliable QoS for those transformed topics. If it subscribes directly to raw
RealSense pointcloud topics, it should match best-effort QoS.

## Initial X86 Validation

1. Confirm the x86 PC sees the transformed pointcloud topics:
   - `/head/d435i_head/points_marker_map`
   - `/arm/d435i_arm/points_marker_map`
   - `/tf`
   - `/tf_static`
2. Confirm both pointcloud headers are already `marker_map`.
3. Publish `/segmentation/input_cloud` at a limited rate, initially 3-6 Hz.
4. Confirm:
   - `ros2 topic echo --once /segmentation/input_cloud --field header.frame_id`
     prints `marker_map`.
   - `ros2 topic echo --once /segmentation/input_cloud --field fields`
     includes `x`, `y`, `z`, and `rgb`.
5. Run the segmentation node and verify click-driven segmentation still accepts
   `/segmentation/click_positive`.

## Open Questions For Implementation

- Whether to feed segmentation one merged cloud or one selected camera cloud.
- Whether crop/downsample should happen before or after merging.
- Whether prosthesis self-filtering should happen before or after merging. The
  likely first version is after transform/merge in `marker_map`, using boxes
  defined relative to the arm D435i camera and transformed through TF.
- Whether the segmentation node should expose a configurable input QoS profile.
- Whether the click-producing future-pose collision node should publish one
  click per predicted collision or rate-limit/cluster clicks.
