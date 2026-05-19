# Agent Notes

Repo working directory for ROS work:
`~/Documents/assistive_multiview_prosthesis/docker_ws`

Docker compose directory:
`~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment`

ROS 2 Jazzy is inside the `realsense_camera` Docker container, not on the host.
The host may have ROS 2 Humble, so do not use host ROS commands for Jazzy
validation. Validate builds, launches, and ROS package discovery through
Docker/Jazzy.

Active source package:
`multi_cam_localization/sensor_fusion_bringup`

Generated overlay artifacts must not be committed:
`build_overlay/`, `install_overlay/`, `log_overlay/`

Jetson build safety:
- OpenVINS / `ov_msckf` builds can crash the Jetson if run with other heavy
  work. Build sequentially only, and do not run camera streams, RViz, file
  transfers, or other memory-heavy tasks at the same time.
- Use:
  `MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon --executor sequential --parallel-workers 1 ...`

If Docker creates root-owned overlay files, fix ownership only for generated
overlay directories from `docker_ws`:

```bash
sudo chown -R "$(id -u):$(id -g)" build_overlay install_overlay log_overlay
```

When a task requires the user to run many validation commands, create or update
a command recipe markdown file. Use user-facing docs for repeatable project
procedures, for example
`multi_cam_localization/sensor_fusion_bringup/docs/head_openvins_marker_bag_replay_validation.md`.
Use `.agents/` notes for agent-only handoff context.

Head D435i:
- Serial: `336222071386`
- OpenVINS config: `multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/`

Arm D435i:
- Serial: `310622071850`
- Calibration output: `calibration/arm/d435i_310622071850/`

Dynamic ID2 arm workflow:
- User-facing config:
  `multi_cam_localization/sensor_fusion_bringup/config/dynamic_id2_arm_update.yaml`
- One-command launch:
  `multi_cam_localization/sensor_fusion_bringup/launch/dynamic_id2_arm_update_live.launch.py`
- Focused ID2 arm-camera extrinsic calibration commands:
  `multi_cam_localization/sensor_fusion_bringup/docs/arm_marker_id2_extrinsic_calibration_commands.md`
- Current default is `mode: active`, which enables fixed ID0 updates/reanchor
  plus dynamic ID2 normal updates and guarded dynamic ID2 initial-lock/reanchor.
- Roll back with `mode:=observe`, `mode:=update`, `mode:=would_reanchor`, or
  full dynamic disable via `use_dynamic_arm_pose_updates:=false`.
- Keep ID2 separate from fixed marker maps. Do not add ID2 to `marker_fixed_ids`.
- Before treating live dynamic ID2 reanchor as validated, require a bag where
  `/ov_msckf_arm/odomimu`, `/arm/marker_pose/dynamic_arm_pose_observation`, and
  `/ov_msckf_arm/dynamic_arm_update/status` overlap.
- 2026-05-19: the redesigned arm-mounted ID2 fixture was calibrated from
  `dynamic_id2_arm_update_live_20260519_113936` and installed in
  `config/markers/arm_marker_extrinsics.yaml`. Result: `166` inliers, p95
  translation residual `0.0104 m`, p95 rotation residual `2.349 deg`. CAD
  old-to-new mount delta was consistent with the calibrated `T_armcam_marker`
  delta; do not redo solely on geometry unless live validation shows trouble.

D435i color pointcloud support:
- Pointclouds are opt-in for the dynamic ID2 live workflow:
  `enable_pointclouds:=true`. Leave them off for OpenVINS-only timing tests.
- OpenVINS RGB streams should remain `640x480x30`; pointcloud depth is
  `640x480x15` and transformed clouds are rate-limited to 10 Hz by default.
- 2026-05-14 live validation: RViz2 color pointclouds now work while OpenVINS
  remains stable. Status messages alternating `published` and `rate_limited`
  are expected.
- The live OpenVINS D435i launches default `hold_back_imu_for_frames:=true`.
  This is a RealSense live-publication ordering setting, not a rosbag playback
  rewrite switch.
- The D435i launch files pass RealSense parameters directly to
  `realsense2_camera_node` instead of depending on the upstream `rs_launch.py`
  argument list.
- Keep `enable_infra`, `enable_infra1`, and `enable_infra2` false in custom
  D435i launches. Enabling the extra infra streams caused RealSense depth
  timeouts and blank pointclouds.
- `enable_pointcloud_neon_fix` is a legacy fallback and now defaults false.
  Startup `pointcloud__neon_` parameters are sufficient on the validated Jetson.
- Stable grasping topics are `/head/d435i_head/points_marker_map` and
  `/arm/d435i_arm/points_marker_map`; both are in `marker_map` once OpenVINS
  has initialized marker-map camera TF.
- Validation commands:
  `multi_cam_localization/sensor_fusion_bringup/docs/d435i_color_pointcloud_marker_map_validation.md`
- Planned x86 segmentation-side pointcloud work:
  `.agents/x86_segmentation_pointcloud_plan.md`
- Future pose prediction/collision planning should use:
  `.agents/future_pose_prediction_collision_prompt.md`
  Key rule: publish/check future predictions only on accepted pointcloud updates
  so prediction cadence follows pointcloud cadence.
- Prosthesis self-filter planning should use:
  `.agents/prosthesis_self_filter_pointcloud_prompt_2026-05-15.md`
  Goal: remove points inside user-measured prosthesis boxes before collision
  checking so the system does not collide with itself.

Phase 1 marker correction is external only. Do not modify the OpenVINS internal
EKF until the external marker-corrected odom path is validated and committed.

Marker pose frame conventions:
- `marker_map` should be z-up/navigation-compatible for RViz and later
  point-cloud fusion.
- `camera_pose_raw` is the raw ROS/OpenCV optical frame:
  `+X` right, `+Y` down, `+Z` forward through the lens.
- `camera_body_pose` is visualization-only:
  `+X` forward, `+Y` left, `+Z` up.
- `imu_pose` is the true calibrated IMU pose and should not be hacked for
  display.


When you think performance would improve by starting a fresh Codex chat/agent, tell me explicitly and provide a ready-to-copy handoff prompt for the next agent.

Tell the user when it is a good point to commit and push to GitHub. Include: which files should be committed, which files should not be committed, cleanup commands for generated artifacts, and a suggested commit message. Never stage, commit, push, stash, or clean broad file sets without user approval.
