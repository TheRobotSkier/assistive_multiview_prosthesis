# Dynamic ID2 Arm Update Live Validation Commands

The primary workflow is now one launch file plus one user-editable YAML config.
Dynamic ID2 is not part of `marker_fixed_ids`; fixed ID0 remains the marker-map
lock/reanchor path.

Run commands from:

```bash
cd /home/robotlab/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment
```

If RViz cannot open from Docker, run once on the host:

```bash
xhost +si:localuser:root
```

## Main Live Command

Default live workflow. This starts fixed ID0 updates/reanchor and dynamic ID2
normal updates plus guarded dynamic ID2 initial-lock/reanchor:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py start_rviz:=true'
```

Conservative diagnostic start:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py mode:=observe start_rviz:=true'
```

Modes:

```text
observe         publish dynamic measurements; arm OpenVINS logs measurement-only status
update          enable bounded normal dynamic EKF updates; dynamic reanchor disabled
would_reanchor  log dynamic initial-lock/reanchor decisions without mutating state
active          enable normal updates plus guarded dynamic initial-lock/reanchor
```

Recommended live validation order for a new marker mount, calibration, or
network/display setup:

```text
observe -> update -> would_reanchor -> active
```

The repository default is now `active`, so pass `mode:=observe` or
`mode:=would_reanchor` explicitly when you want diagnostic behavior before
trusting the default active workflow.

## User Config

The editable config is:

```text
docker_ws/multi_cam_localization/sensor_fusion_bringup/config/dynamic_id2_arm_update.yaml
```

Parameter descriptions are in
`docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/dynamic_id2_arm_update_parameters.md`.

Common overrides can be passed on the launch command line:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py mode:=would_reanchor start_rviz:=true record_bag:=true'
```

The measurement node should report:

```text
head_pose_source_topic: /ov_msckf/odomimu
head_pose_source_type: odometry
```

It should not queue stale ID2 observations. If no already-buffered head odometry
sample matches within `max_head_pose_dt_s`, status reports `no_head_pose_match`.

## Status Checks

Measurement producer status:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo /arm/marker_pose/dynamic_arm_measurement/status'
```

Arm OpenVINS dynamic update status:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo /ov_msckf_arm/dynamic_arm_update/status'
```

Topic rates:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /head/marker_pose/dynamic_observation'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /arm/marker_pose/dynamic_arm_pose_observation'
```

TF sanity:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 run tf2_ros tf2_echo marker_map arm_imu'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 run tf2_ros tf2_echo marker_map head_imu'
```

## What To Look For

Minimum checks before trusting the default active workflow:

- `/ov_msckf_arm/odomimu` keeps publishing through the full motion.
- `/ov_msckf_arm/dynamic_arm_update/status` publishes after
  `/arm/marker_pose/dynamic_arm_pose_observation` starts.
- If `/ov_msckf_arm/odomimu` goes stale, stop and debug arm OpenVINS before
  judging dynamic ID2 update or reanchor behavior.

Good observe or would-reanchor behavior:

- `/head/marker_pose/observation` and `/arm/marker_pose/observation` are ID0-only.
- `/head/marker_pose/dynamic_observation` is ID2-only.
- `/arm/marker_pose/dynamic_arm_measurement/status` uses `/ov_msckf/odomimu`.
- `head_pose_time_offset_s` is small and explainable.
- Measurement covariance standard deviations are finite and not tiny.
- Rejections have clear reasons such as `no_head_pose_match`,
  `dynamic_marker_not_stable`, or `head_pose_wrong_child_frame`.
- `would_dynamic_initial_lock` or `would_dynamic_reanchor` appears only with
  enough consistent samples.
- RViz keeps separate `marker_map -> head_imu -> head_cam0` and
  `marker_map -> arm_imu -> arm_cam0` chains.

Good update or active behavior:

- Accepted normal dynamic updates are below `0.35 m` and `15 deg`.
- Large post-gap innovations are rejected or routed to reanchor evaluation.
- Recent fixed ID0 updates suppress dynamic updates/reanchor for the configured
  skip windows.
- No sudden normal EKF snaps, no non-finite warnings, no TF frame collisions,
  and no repeated reset loop.

Do not judge success by perfect visual overlap between the preview and arm
OpenVINS. The head-derived ID2 pose is a covariance-bearing measurement, not
ground truth.

## Replay Commands

Build/test sequentially on the Jetson, with other heavy processes closed:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon --log-base log_overlay build --symlink-install --build-base build_overlay --install-base install_overlay --executor sequential --parallel-workers 1 --packages-select sensor_fusion_msgs sensor_fusion_bringup ov_msckf'
```

Replay observe mode:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py use_sim_time:=true start_cameras:=false start_rviz:=false mode:=observe'
```

In another terminal:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 bag play bags/openvins_tests/phase2_live/dynamic_id2_arm_update_live_20260512_124920 --clock'
```

Repeat the launch/play pair for:

```text
dynamic_id2_arm_update_live_20260512_124920
dynamic_id2_arm_update_live_20260512_132629
```

and for modes:

```text
observe
update
would_reanchor
active
```

Only run `active` after tests, observe/update, and would-reanchor pass.

## Record A Live Validation Bag

The one-command launch can record the standard validation topics:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py start_rviz:=true record_bag:=true'
```

Diagnostic recording can still override the mode:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py mode:=would_reanchor start_rviz:=false record_bag:=true'
```

The generated bag includes raw streams, fixed and dynamic marker observations,
dynamic arm pose observations, both status topics, OpenVINS outputs, preview
topics, `/tf`, `/tf_static`, and `/rosout`.

## Fallback Multi-Terminal Debug

Use this only when debugging one node at a time. The single launch above should
be preferred for normal live use.

Head OpenVINS:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py start_camera:=true use_sim_time:=false verbosity:=INFO'
```

Arm OpenVINS in measurement-only mode:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py start_camera:=true use_sim_time:=false verbosity:=INFO use_dynamic_arm_pose_updates:=true dynamic_arm_measurement_only:=true dynamic_arm_allow_initial_lock:=false dynamic_arm_allow_reanchor:=false'
```

Head marker node:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py use_sim_time:=false'
```

Arm marker node:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup arm_marker_pose_phase2.launch.py use_sim_time:=false'
```

Dynamic measurement producer:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_arm_pose_measurement.launch.py use_sim_time:=false publish_dynamic_arm_pose_observation:=true head_pose_topic:=/ov_msckf/odomimu head_pose_message_type:=odometry'
```

Optional debug preview:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup head_derived_arm_pose_preview.launch.py use_sim_time:=false require_stable_dynamic_marker:=true publish_tf:=true'
```

RViz:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && rviz2 -d install_overlay/sensor_fusion_bringup/share/sensor_fusion_bringup/config/rviz/phase2_dual_openvins_head_preview.rviz'
```

## Rollback

Safe rollback for live use:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py mode:=observe'
```

Disable dynamic reanchor but keep normal dynamic EKF updates:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py mode:=update'
```

Full dynamic-disable fallback:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py use_dynamic_arm_pose_updates:=false dynamic_arm_measurement_only:=true dynamic_arm_allow_initial_lock:=false dynamic_arm_allow_reanchor:=false'
```

Stop validation containers with `Ctrl-C`, then check:

```bash
docker ps
```

Stop any leftover validation container by id:

```bash
docker stop <container_id>
```
