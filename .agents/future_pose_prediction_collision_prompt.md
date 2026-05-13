# Future Pose Prediction And Pointcloud Collision Planning Prompt

Use this prompt in a fresh Codex chat when ready to investigate and plan the
future pose prediction + pointcloud collision/click workflow.

Start in plan mode. Do not change code until the investigation is complete and
the implementation plan has been approved.

```text
Plan mode only. Do not change code.

I need to investigate and plan a future pose prediction node for the prosthesis
trajectory, plus a pointcloud collision/proximity checker that publishes a
positive click/hit command on `/segmentation/click_positive`.

High-level goal:
- For a configurable horizon `y` seconds into the future, produce `x`
  predicted poses along the trajectory.
- Each predicted pose must include estimated uncertainty/covariance.
- Then this node or a follow-on node must check whether any predicted future
  pose collides with, or gets close enough to, points in the D435i color
  pointclouds transformed into a shared frame.
- If a predicted pose intersects or is close enough to the pointcloud, publish
  a hit/click command on `/segmentation/click_positive`.

Important current context:
- Current branch is the OpenVINS + marker update + D435i pointcloud branch.
- OpenVINS RGB input should remain at 30 Hz.
- D435i pointclouds are opt-in and default off.
- Stable transformed pointcloud topics are:
  - `/head/d435i_head/points_marker_map`
  - `/arm/d435i_arm/points_marker_map`
- Common frame is `marker_map`.
- Transformed pointclouds publish only after marker-map lock.
- Preserve existing OpenVINS + fixed ID0 + dynamic ID2 behavior.
- Do not add ID2 to `marker_fixed_ids`.
- Avoid TF frame collisions.

Read first:
- `.agents/AGENTS.md`
- `.agents/future_tasks.md`
- `.agents/phase2_openvins_handoff_status.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_color_pointcloud_marker_map_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dynamic_id2_arm_update_live.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/src/pointcloud_to_frame_node.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h`
- OpenVINS propagation/state files under `docker_ws/src/open_vins`, especially
  files related to IMU propagation, state covariance, odometry/pose publishing,
  and state cloning.

Older branch / older implementation to compare:
- The user said an earlier attempt exists on branch `test1-daniel`.
- First check whether the branch is locally available:
  `git branch --all --list '*test1*' '*daniel*'`
- If it is not local, check whether it is available remotely:
  `git ls-remote --heads origin | rg 'test1|daniel'`
- If available, fetch it safely without switching the current branch unless
  needed:
  `git fetch origin test1-daniel:test1-daniel`
- Then inspect, or find equivalent paths:
  - `assistive_multiview_prosthesis/src/twist_propagation/twist_propagation/twist_propagation_node.py`
  - `assistive_multiview_prosthesis/config/prosthesis_config.yaml`
- If those exact paths differ, find equivalents with:
  `git ls-tree -r --name-only test1-daniel | rg 'twist_propagation|prosthesis_config|prediction|trajectory|click_positive|segmentation|pointcloud|points'`

Investigation questions to answer before planning implementation:

1. OpenVINS prediction model suitability
- Does OpenVINS expose enough state and covariance externally to support future
  pose prediction without modifying OpenVINS internals?
- Does `/ov_msckf/odomimu` or any related topic include usable pose/twist
  covariance?
- Does OpenVINS publish velocity and angular velocity in a frame useful for
  short-horizon prediction?
- Is the internal OpenVINS propagation model reusable cleanly from a separate
  ROS node, or would that require invasive coupling to `ov_msckf` internals?
- Would it be safer to add a small OpenVINS-side publisher for predicted future
  poses/covariances, or keep prediction in `sensor_fusion_bringup` using
  published odometry/twist and covariance?
- What is the expected covariance quality if using OpenVINS state covariance
  vs. a simpler external twist propagation model?

2. Older `twist_propagation` implementation
- What input topics, output topics, frames, and message types did it use?
- What motion model did it implement?
- How did it estimate uncertainty, if at all?
- Did it already publish to `/segmentation/click_positive`, or did another node
  do that?
- What config values from `prosthesis_config.yaml` are still useful?
- Which parts are safe to reuse, and which parts are obsolete because OpenVINS
  marker-map locking and pointcloud topics now exist?

3. Frame and target semantics
- Which frame should be predicted: `arm_imu`, `arm_cam0`, an end-effector/tool
  frame, or another prosthesis frame?
- Which frame should the prediction output use? Prefer `marker_map` if the
  pointclouds and collision checks are in `marker_map`.
- What TF chain exists from OpenVINS to the physical collision geometry?
- Is there a known prosthesis collision geometry, radius, capsule, sphere, or
  bounding shape in config?
- If the tool/prosthesis geometry is missing, plan a simple configurable model
  first, for example one or more spheres/capsules attached to a target frame.

4. Pointcloud proximity/collision checker
- Should collision checking consume:
  - `/head/d435i_head/points_marker_map`,
  - `/arm/d435i_arm/points_marker_map`,
  - both, or
  - a fused/downsampled cloud?
- How should stale pointclouds be handled?
- How should stale predictions be handled?
- How should pointcloud density/noise be handled? Consider voxel downsampling,
  radius outlier filtering, minimum nearby point count, ROI limits, and max
  point age.
- Should the checker use a KD-tree, PCL, numpy/scipy, or another dependency
  already available in the Docker/Jazzy environment?
- What threshold means "hit" or "close enough"? Make it configurable.

5. `/segmentation/click_positive` contract
- Find the consumer of `/segmentation/click_positive`.
- Determine the expected message type, coordinate frame, timestamp semantics,
  and whether the click is a 2D image click, a 3D point, or another custom
  command.
- Do not assume the message type. If no consumer exists on the current branch,
  identify the intended type from `test1-daniel`; if still unknown, propose a
  conservative explicit interface.

6. Architecture choices
- Decide whether to implement:
  - one combined prediction + collision node,
  - separate `future_pose_predictor` and `future_pose_pointcloud_checker`
    nodes, or
  - an OpenVINS-side predictor plus a separate checker.
- Favor separation if prediction should be reusable without pointclouds.
- Keep pointcloud/collision work opt-in so OpenVINS-only validation is not
  slowed down.

7. ROS interfaces
- Specify exact input topics.
- Specify exact output topics for predicted trajectory and status.
- Specify exact output topic/message for `/segmentation/click_positive`.
- Decide whether to create a new message type for predicted poses with
  covariance, or use existing messages such as:
  - `nav_msgs/Path` plus separate covariance array/status,
  - `geometry_msgs/PoseArray` plus custom covariance topic,
  - `PoseWithCovarianceStamped` sequence,
  - a new `sensor_fusion_msgs/FuturePoseTrajectory`.
- Include status topics with reasons for no prediction/no click.

8. Timing and uncertainty
- Define horizon `y`, sample count `x`, update rate, and max input age.
- Define how covariance grows along the horizon.
- Define how IMU/OpenVINS covariance, twist covariance, process noise, and
  model uncertainty are combined.
- Define fallback behavior if covariance is unavailable.
- Define gating based on marker-map lock and OpenVINS initialization.

9. Validation and acceptance
- Include build commands through Docker/Jazzy.
- Include launch smoke commands.
- Include topic/type checks.
- Include TF checks.
- Include RViz checks for predicted trajectory, covariance visualization, and
  pointcloud proximity.
- Include bag recording/replay commands.
- Acceptance criteria must include:
  - predicted poses publish in `marker_map`;
  - uncertainty is finite and increases sensibly over the horizon;
  - checker publishes no clicks when pointcloud/prediction/TF are stale;
  - checker publishes a click when a known test trajectory intersects a known
    pointcloud region;
  - OpenVINS fixed ID0 and dynamic ID2 behavior is unchanged;
  - CPU/memory load remains acceptable on the Jetson.

10. Risks and rollback
- Identify risks from using OpenVINS internals directly.
- Identify risks from external twist prediction drifting from OpenVINS state.
- Identify risks from false positive clicks due to noisy pointclouds.
- Identify rollback flags and launch arguments to disable prediction and
  collision checking independently.

Deliverable for this planning turn:
- First provide a concise investigation plan.
- Then perform the investigation by reading the files/branches above.
- Then provide a decision-complete implementation plan based on the findings.
- Do not implement until the user approves.
- The implementation plan must be file-by-file, include launch/config changes,
  docs updates, validation commands, acceptance criteria, rollback, and suggested
  commit checkpoints/messages.
```
