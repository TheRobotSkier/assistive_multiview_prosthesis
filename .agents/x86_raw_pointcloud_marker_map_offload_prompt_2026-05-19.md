# X86 Raw Pointcloud Marker-Map Offload Planning Prompt

Use this prompt in a fresh Codex/Copilot/Coding Agent chat when ready to plan
moving D435i color pointcloud transform/merge work from the Jetson to the x86
Ubuntu PC.

```text
Plan mode only. Do not change code.

I need to investigate and plan how to move the D435i color pointcloud
combination pipeline into the shared `marker_map` frame from the Jetson to the
x86 Ubuntu PC.

High-level goal:
- The Jetson should run only the time-critical live stack:
  - RealSense RGB/IMU streams
  - the modified OpenVINS nodes
  - fixed marker ID0 updates/reanchor
  - dynamic marker ID2 arm updates/reanchor
  - `marker_map` TF publication
  - raw D435i color pointcloud publication
- The x86 PC should subscribe to raw pointclouds and TF from the Jetson, then:
  - transform the head raw color pointcloud into `marker_map`
  - transform the arm raw color pointcloud into `marker_map`
  - combine/merge the transformed clouds into one common cloud
  - remove points farther away than a configurable range, initially `2.0 m`
  - preserve full point density/resolution on the x86 side
  - publish the combined cloud for segmentation/collision work

Important non-goals and constraints:
- Do not lower pointcloud resolution on the x86 PC.
- Do not voxel downsample, decimate, subsample, randomly sample, or otherwise
  reduce point density on the x86 PC in the first version.
- Range filtering is allowed and desired; removing points beyond `2.0 m` is not
  considered downsampling.
- If bandwidth or CPU is too high, plan separate rate limiting or transport
  options, but do not silently reduce point density.
- Preserve `x`, `y`, `z`, and packed `rgb` fields.
- Preserve OpenVINS RGB input at `640x480x30`.
- Preserve fixed ID0 and dynamic ID2 behavior.
- Do not add ID2 to `marker_fixed_ids`.
- Do not re-enable RealSense infra streams.
- Do not break the currently working Jetson marker-map pointcloud path until the
  x86 offload path is validated and can be selected by launch arguments.

Current validated context:
- OpenVINS dual D435i live path works well on the Jetson.
- Color pointclouds are opt-in and currently work in RViz2.
- Current validated transformed topics on the Jetson are:
  - `/head/d435i_head/points_marker_map`
  - `/arm/d435i_arm/points_marker_map`
- Raw RealSense pointcloud topics are expected to be:
  - `/head/d435i_head/depth/color/points`
  - `/arm/d435i_arm/depth/color/points`
- Common output frame must be `marker_map`.
- 2026-05-19 follow-up: generic raw cloud consumers should not assume direct TF
  from `marker_map` to `*_d435i_*_depth_optical_frame`. Use the composed
  transform path through `head_cam0`/`arm_cam0`, as in
  `x86_raw_pointcloud_marker_map.launch.py` and `pointcloud_to_frame_node`.
- 2026-05-19 follow-up: the canonical raw topic names use underscores:
  `/head/d435i_head/depth/color/points` and
  `/arm/d435i_arm/depth/color/points`. Slash-style names such as
  `/head/d435i/head/depth/color/points` are wrong unless a separate relay
  explicitly publishes them.
- Current transformed pointcloud status topics use `published` and
  `rate_limited`; rate limiting is normal.
- The current pointcloud transform node is:
  `docker_ws/multi_cam_localization/sensor_fusion_bringup/src/pointcloud_to_frame_node.cpp`
- Existing x86 notes:
  `.agents/x86_segmentation_pointcloud_plan.md`
- Existing prosthesis self-filter planning prompt:
  `.agents/prosthesis_self_filter_pointcloud_prompt_2026-05-15.md`

Read first:
- `.agents/AGENTS.md`
- `.agents/future_tasks.md`
- `.agents/x86_segmentation_pointcloud_plan.md`
- `.agents/prosthesis_self_filter_pointcloud_prompt_2026-05-15.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_openvins_pointcloud_implementation_report_2026-05-14.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_color_pointcloud_marker_map_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/dynamic_id2_arm_update_live_validation_commands.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dynamic_id2_arm_update_live.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dual_d435i.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/src/pointcloud_to_frame_node.cpp`
- `docker_ws/docker-deployment/docker-compose.yml`
- `docker_ws/docker-deployment/cyclonedds_peer.xml`

Investigation questions:

1. Current Jetson pointcloud pipeline
- Which launch arguments currently enable raw RealSense pointcloud publication?
- Does `enable_pointclouds:=true` always also start Jetson-side
  `pointcloud_to_frame_node` instances?
- What launch changes would let the Jetson publish raw pointclouds without
  also transforming them into `marker_map`?
- Should the first implementation add a launch mode such as:
  - `enable_pointclouds:=true`
  - `start_pointcloud_transformers:=false`
  - `publish_raw_pointclouds:=true`
  or a clearer equivalent?
- What existing defaults must remain unchanged for OpenVINS-only validation?

2. Network and DDS transport
- Can the x86 PC see Jetson topics over CycloneDDS using the current
  `cyclonedds_peer.xml` setup?
- Which topics must cross the network:
  - `/head/d435i_head/depth/color/points`
  - `/arm/d435i_arm/depth/color/points`
  - `/tf`
  - `/tf_static`
  - optional status topics
- Should raw high-bandwidth pointcloud subscriptions use sensor-data/best-effort
  QoS?
- Should the combined x86 output use reliable shallow-queue QoS for RViz and
  segmentation compatibility?
- Estimate bandwidth for two full-density color pointcloud streams at the
  configured depth FPS. Identify whether rate limiting raw publication, not
  point subsampling, may be needed.

3. X86 transform and merge architecture
- Should the x86 reuse/extend the existing C++ `pointcloud_to_frame_node`, or
  should it get a new dedicated node such as `dual_pointcloud_marker_map_merge`?
- Should the x86 node subscribe to both raw clouds and transform each one using
  `/tf` into `marker_map`, then concatenate the transformed clouds?
- How should timestamps be handled?
  - transform each cloud at its own stamp
  - approximate-sync head and arm clouds before merging
  - publish latest-pair merge with max allowed time difference
- What should happen when one cloud is stale or TF is unavailable?
- Should output include one combined cloud plus optional per-camera transformed
  debug topics?

4. Range filtering without downsampling
- Define a configurable max range, default `2.0 m`.
- Decide which frame/distance metric to use:
  - radial distance from each source camera before transform,
  - radial distance from the origin in `marker_map`,
  - z-depth in each source camera,
  - or configurable mode.
- Prefer camera-frame range filtering before transform if it removes physically
  distant points from each D435i view while preserving all near-field points.
- Preserve all remaining points exactly; do not voxelize or decimate.
- Keep RGB fields and PointCloud2 layout valid.

5. Output topics and contracts
- Propose exact output topics. Suggested first version:
  - `/segmentation/input_cloud_raw` or `/d435i/points_marker_map_combined`
    for the merged full-density cloud
  - `/segmentation/input_cloud` only after optional prosthesis self-filtering
    is inserted
- Keep output header frame as `marker_map`.
- Include a status topic with:
  - input counts per camera
  - output count
  - dropped beyond range count
  - stale cloud/TF reasons
  - processing time
  - publish rate
  - max range
  - whether full-density mode is active

6. Jetson/x86 launch separation
- Plan how to launch the Jetson in "raw-cloud offload" mode.
- Plan how to launch the x86 merger node.
- Decide whether x86 runs:
  - native ROS 2 Jazzy,
  - its own Docker/Jazzy container,
  - or the existing project Docker image adapted for x86.
- Include environment and DDS setup checks for cross-machine ROS discovery.
- Ensure the old all-on-Jetson marker-map pointcloud path remains available as
  a rollback mode.

7. Validation and acceptance
- Include commands to verify the x86 can see raw topics and TF.
- Include commands to verify raw pointcloud rates and message sizes.
- Include TF checks from raw cloud frames to `marker_map`.
- Include RViz checks on x86 for:
  - raw head cloud
  - raw arm cloud
  - transformed/combined cloud in `marker_map`
  - range-filtered result
- Include checks that output has `x`, `y`, `z`, and `rgb` fields.
- Include checks that point density is not reduced except by range filtering:
  - output count should be close to input count minus points beyond range
  - no voxel leaf size or decimation parameter should be active
- Include Jetson load comparison before/after offload.
- Include network load observations.

Acceptance criteria:
- Jetson OpenVINS with fixed ID0 and dynamic ID2 still behaves as before.
- Jetson can publish raw head and arm color pointclouds without running the
  Jetson marker-map pointcloud transform nodes.
- X86 receives both raw pointclouds and the required TF.
- X86 publishes a combined `marker_map` PointCloud2 with preserved RGB.
- Points beyond the configured range, initially `2.0 m`, are removed.
- No x86 downsampling/subsampling/voxel filtering occurs in the first version.
- Output is usable by the segmentation/future collision pipeline.
- If TF/clouds are stale, the node reports status and does not publish
  misleading data.
- Rollback to the current Jetson-side `/head/d435i_head/points_marker_map` and
  `/arm/d435i_arm/points_marker_map` path remains one launch option away.

Deliverable for this planning turn:
- First provide a concise investigation plan.
- Then read the relevant files and current launch structure.
- Then provide a decision-complete implementation plan.
- Do not implement until the user approves.
- The implementation plan must include:
  - file-by-file launch/code/config/doc changes
  - x86 and Jetson commands
  - topic names and QoS
  - range filter semantics
  - no-downsampling guarantee
  - validation commands
  - acceptance criteria
  - rollback flags
  - suggested commit checkpoints/messages
```
