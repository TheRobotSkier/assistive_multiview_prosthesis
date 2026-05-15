# Prosthesis Self-Filter Pointcloud Planning Prompt

Use this prompt in a fresh Codex/Copilot/Coding Agent chat when ready to plan a
pointcloud self-filter that removes points belonging to the prosthesis/arm D435i
mount before collision checking.

```text
Plan mode only. Do not change code.

I need to investigate and plan a pointcloud self-filter for the D435i
marker-map/fused pointcloud pipeline.

Goal:
- Remove points from a combined or segmentation-ready pointcloud that belong to
  the prosthesis itself.
- The user will measure and provide configurable bounding boxes around parts of
  the prosthesis.
- The bounding boxes will be defined relative to the arm D435i camera frame
  unless investigation finds a better frame.
- These points must be removed before future-pose collision/proximity checking
  so the system does not detect collisions with the prosthesis itself.

Current validated context:
- OpenVINS and RViz2 color pointclouds now work.
- Stable Jetson marker-map pointcloud topics:
  - `/head/d435i_head/points_marker_map`
  - `/arm/d435i_arm/points_marker_map`
- Common frame is `marker_map`.
- The x86 PC is the preferred place for heavier pointcloud processing:
  fusion/merge, crop, self-filter, KD-tree/proximity checking, and segmentation
  input preparation.
- Jetson should keep doing RealSense RGB/IMU, OpenVINS, marker updates, TF, and
  the validated opt-in marker-map pointcloud republishing.
- Do not re-enable RealSense infra streams.
- Do not add marker ID2 to `marker_fixed_ids`.

Read first:
- `.agents/AGENTS.md`
- `.agents/future_tasks.md`
- `.agents/x86_segmentation_pointcloud_plan.md`
- `.agents/future_pose_prediction_collision_prompt.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_openvins_pointcloud_implementation_report_2026-05-14.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_color_pointcloud_marker_map_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/src/pointcloud_to_frame_node.cpp`

Investigation questions:

1. Input/output contract
- Should the self-filter consume:
  - `/arm/d435i_arm/points_marker_map`,
  - a merged head+arm cloud,
  - `/segmentation/input_cloud`, or
  - a new pre-segmentation topic?
- What should the filtered output topic be?
  Suggested first version:
  - input: `/segmentation/input_cloud_raw`
  - output: `/segmentation/input_cloud`
- Should the filter preserve `x`, `y`, `z`, and `rgb` fields exactly?
- Should the output frame remain `marker_map`?

2. Bounding-box frame semantics
- The user plans to provide boxes relative to the arm D435i camera. Identify the
  exact frame to use, likely `arm_d435i_arm_color_optical_frame` or `arm_cam0`.
- Determine whether the boxes should be axis-aligned in the camera frame or
  full oriented boxes with translation + quaternion.
- Determine how to transform either:
  - each point from cloud frame `marker_map` into the box frame, or
  - each box from camera frame into `marker_map`.
- Decide how to handle missing/stale TF.

3. Geometry config
- Propose a YAML config format for multiple prosthesis boxes:
  - name
  - frame_id
  - center_xyz_m
  - size_xyz_m
  - optional rotation_rpy_deg or quaternion_xyzw
  - inflation_margin_m
  - enabled
- Include separate global margins for safety and tuning.
- Support future shapes if needed, but start with boxes only unless a sphere or
  capsule is clearly simpler.

4. Filtering algorithm
- Prefer a simple deterministic first implementation:
  - transform points into each box frame
  - remove points whose absolute coordinates are within half-size plus margin
  - publish filtered PointCloud2
- Consider whether numpy, PCL, or pure C++/Python is best in the current
  Docker/Jazzy environment.
- Keep it fast enough for 3-8 Hz segmentation/collision processing.
- Publish status with input point count, removed point count, output point
  count, active boxes, stale TF, and processing time.

5. Placement
- Prefer running the self-filter on the x86 PC as part of the segmentation input
  preparation pipeline.
- Keep Jetson-side pointcloud republishers unchanged unless measurements show
  that moving transform/fusion to x86 is necessary.

6. Validation
- Include a synthetic test with a small PointCloud2 and a known box.
- Include live checks:
  - point counts before/after filtering
  - RViz display of raw and filtered clouds
  - status topic showing removed points
  - no removal when boxes are disabled
  - no output or explicit stale status when TF is unavailable
- Include bag/replay validation commands through Docker/Jazzy.

7. Integration with future pose collision
- The future collision checker should consume the self-filtered pointcloud, not
  the raw combined cloud.
- The checker should treat stale self-filtered cloud data as no-check/no-click.
- The self-filter should not publish `/segmentation/click_positive`; it only
  prepares pointcloud data.

Deliverable for this planning turn:
- First provide a concise investigation plan.
- Then read the relevant files and identify existing pointcloud/segmentation
  contracts.
- Then provide a file-by-file implementation plan.
- Do not implement until the user approves.
- Include config format, launch arguments, topics, status messages, validation
  commands, acceptance criteria, rollback flags, and suggested commit message.
```
