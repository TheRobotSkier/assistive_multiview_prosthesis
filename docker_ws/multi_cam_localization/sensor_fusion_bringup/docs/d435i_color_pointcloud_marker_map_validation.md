# D435i Color Pointcloud Marker-Map Validation

This workflow keeps OpenVINS RGB input at 30 Hz and makes color pointclouds
opt-in for grasping runs.

Run commands from:

```bash
cd /home/robotlab/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment
```

## Build

Run OpenVINS builds only when the Jetson is otherwise idle. Do not run camera
streams, RViz, large file transfers, or other memory-heavy work during this
build.

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon --log-base log_overlay build --symlink-install --build-base build_overlay --install-base install_overlay --executor sequential --parallel-workers 1 --packages-select sensor_fusion_bringup ov_msckf'
```

Smoke-check the launch arguments and executable:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 pkg executables sensor_fusion_bringup | grep pointcloud_to_frame_node && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py --show-args'
```

Confirm the marker-enabled OpenVINS binary was installed:

```bash
docker compose run --rm realsense_camera 'cd /miahand_ws/src && test -x install_overlay/ov_msckf/lib/ov_msckf/run_subscribe_msckf_marker && ls -1 install_overlay/ov_msckf/lib/ov_msckf/run_subscribe_msckf_marker'
```

The marker executable should also appear in the ROS executable index:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 pkg executables ov_msckf | grep run_subscribe_msckf_marker'
```

If a Jetson build is interrupted and launch reports
`executable 'run_subscribe_msckf_marker' not found`, check whether the build
target is a zero-byte or non-executable file:

```bash
docker compose run --rm realsense_camera 'cd /miahand_ws/src && ls -l build_overlay/ov_msckf/run_subscribe_msckf_marker install_overlay/ov_msckf/lib/ov_msckf/run_subscribe_msckf_marker'
```

Repair only that target with a sequential relink:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 cmake --build build_overlay/ov_msckf --target run_subscribe_msckf_marker --clean-first -- -j1'
```

Confirm the D435i launch exposes the IMU hold-back argument:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py --show-args | grep hold_back_imu_for_frames'
```

## OpenVINS Timing Stabilization

The marker-enabled OpenVINS ROS2 path now prefers dropping stale images over
building a backlog:

- Mono image subscriptions keep the upstream reliable queue-depth path for now;
  a sensor-data QoS experiment caused an immediate `double free or corruption`
  startup failure on the Jetson and was rolled back.
- OpenVINS parameter access now returns a const reference instead of repeatedly
  deep-copying the full options object.
- The internal camera queue is capped; if OpenVINS falls behind, expect logs
  like `[QUEUE]: dropping stale camera frame`.
- Incomplete IMU coverage no longer aborts the process at the old
  `Propagator.cpp` assertion. Expect `[PROP]: unable to propagate ...` logs if
  an image cannot be matched to a complete IMU interval.
- The head and arm OpenVINS configs use a Jetson-safe dual-camera profile:
  `track_frequency: 21.0`, `num_pts: 200`, `fast_threshold: 25`,
  `min_px_dist: 15`, `max_clones: 8`, `max_slam: 25`,
  `max_msckf_in_update: 25`, and `num_opencv_threads: 2`.

These logs mean the estimator is protecting itself from stale or incomplete
updates. A few `[QUEUE]` messages under load are acceptable. Repeated `[PROP]`
messages with the same old start timestamp mean one OpenVINS instance is still
stuck behind and should be treated as a failed baseline.

The RealSense RGB streams remain `640x480x30`; no camera recalibration is needed
for this profile because OpenVINS is only processing a subset of the incoming
images internally.

## Launch Modes

The live OpenVINS launches now pass `hold_back_imu_for_frames:=true` to both
RealSense drivers by default. This keeps IMU/image publication order
chronological when image processing lags under Jetson load. Use this as the
baseline before changing OpenVINS image rate.

OpenVINS-only default, with pointcloud processing off:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=false start_preview:=false start_rviz:=false'
```

Grasping run, with 15 Hz depth/color pointclouds and marker-map republishers:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true'
```

To compare the RealSense publication-order behavior, override the default
explicitly:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=false start_preview:=false start_rviz:=false hold_back_imu_for_frames:=false'
```

Do not treat `hold_back_imu_for_frames` as a rosbag playback switch. It changes
how the live RealSense driver publishes messages. A bag recorded with the flag
true or false is useful evidence, but replaying an already recorded bag with a
different value does not rewrite the bag's recorded message timing.

The default dynamic-ID2 config uses conservative Jetson pointcloud settings:

```text
pointcloud_max_rate_hz:=10.0
pointcloud_voxel_leaf_m:=0.02
pointcloud_max_range_m:=2.0
pointcloud_decimation_magnitude:=3
enable_pointcloud_neon_fix:=true
```

These keep OpenVINS RGB at `640x480x30`, keep RealSense depth at
`640x480x15`, clip depth/pointcloud data beyond 2 m, and publish transformed
clouds at up to 10 Hz. For a denser test, override them explicitly:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true pointcloud_max_rate_hz:=15.0 pointcloud_voxel_leaf_m:=0.01 pointcloud_decimation_magnitude:=2'
```

If OpenVINS timing becomes unstable, try the lighter command first:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true pointcloud_max_rate_hz:=8.0 pointcloud_voxel_leaf_m:=0.03 pointcloud_max_range_m:=1.5 pointcloud_decimation_magnitude:=4'
```

On the Jetson RealSense build, the pointcloud filter is declared as
`pointcloud__neon_` instead of plain `pointcloud`. The launch sets both the
plain and `pointcloud__neon_` startup parameters directly so the filter and RGB
texture stream are active when pointclouds are enabled. The top-level live
launch also re-applies `pointcloud__neon_.enable:=true` after camera startup by
default for pointcloud-enabled runs.

By default, transformed pointclouds publish whenever TF to `marker_map` is
available. To require an explicit OpenVINS marker-map lock before transformed
clouds publish, add:

```bash
pointcloud_require_marker_map_locked:=true
```

Record pointcloud topics only when needed:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true record_bag:=true record_pointclouds:=true start_rviz:=false'
```

## Topic Checks

OpenVINS image streams should stay near 30 Hz:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /head/d435i_head/color/image_raw'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /arm/d435i_arm/color/image_raw'
```

OpenVINS IMU streams should remain healthy while RGB runs near 30 Hz:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /head/d435i_head/imu'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /arm/d435i_arm/imu'
```

Confirm the RealSense nodes received the publication-order setting:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 param get /head/d435i_head hold_back_imu_for_frames && ros2 param get /arm/d435i_arm hold_back_imu_for_frames'
```

Raw RealSense color pointcloud topics should be near 15 Hz or below. These raw
topics are in RealSense optical frames, so RViz cannot display them with fixed
frame `marker_map` unless a full TF chain exists outside OpenVINS:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /head/d435i_head/depth/color/points'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /arm/d435i_arm/depth/color/points'
```

If RGB images publish but these raw pointcloud topics have no rate, inspect:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic info --verbose /head/d435i_head/depth/color/points'
```

`Publisher count: 0` means the RealSense pointcloud filter is not active. Make
sure the launch was rebuilt and restarted after the startup parameter changes.

On the Jetson build, the active RealSense pointcloud parameters should be under
`pointcloud__neon_`:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 param get /head/d435i_head pointcloud__neon_.enable && ros2 param get /head/d435i_head pointcloud__neon_.stream_filter && ros2 param get /head/d435i_head pointcloud__neon_.stream_index_filter'
```

Expected values are `true`, `2`, and `0`.

The RealSense depth clipping parameter should match the requested pointcloud max
range when pointclouds are enabled:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 param get /head/d435i_head clip_distance && ros2 param get /arm/d435i_arm clip_distance'
```

Transformed grasping topics should publish whenever TF to `marker_map` is
available. If `pointcloud_require_marker_map_locked:=true`, they publish only
after marker-map lock:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /head/d435i_head/points_marker_map'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /arm/d435i_arm/points_marker_map'
```

The raw RealSense pointcloud stream and the transform node input use
sensor-data/best-effort QoS to avoid DDS backpressure. The transformed
`points_marker_map` outputs are republished as reliable, shallow-queue topics so
default RViz PointCloud2 displays and plain `ros2 topic hz` checks can subscribe.

The transformed pointcloud headers must use `marker_map`:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo --once /head/d435i_head/points_marker_map --field header.frame_id'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo --once /arm/d435i_arm/points_marker_map --field header.frame_id'
```

## TF And Lock Checks

Marker-map lock topics:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo /ov_msckf/marker_map_locked'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo /ov_msckf_arm/marker_map_locked'
```

Common-frame TF checks:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 run tf2_ros tf2_echo marker_map head_cam0'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 run tf2_ros tf2_echo marker_map arm_cam0'
```

RealSense optical-frame TF checks:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 run tf2_ros tf2_echo head_d435i_head_color_optical_frame head_d435i_head_depth_optical_frame'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 run tf2_ros tf2_echo arm_d435i_arm_color_optical_frame arm_d435i_arm_depth_optical_frame'
```

Status topics explain published clouds, dropped clouds before lock, stale TF, or
rate limiting:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo /head/d435i_head/points_marker_map/status'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo /arm/d435i_arm/points_marker_map/status'
```

With the default `pointcloud_require_marker_map_locked:=false`, valid TF before
explicit lock gives status reason `published_unlocked`. After lock, expect
`published`. If strict locking is enabled, expect `marker_map_not_locked` before
lock.

The status JSON also includes `input_points`, `transform_input_points`, and
`output_points`. If range clipping and decimation are helping,
`transform_input_points` should be much lower than the raw 640x480 point count,
and `output_points` should drop further after voxel filtering.

If the status reason is `tf_unavailable:"marker_map" passed to lookupTransform
argument target_frame does not exist`, the pointcloud path is running but
OpenVINS has not created the `marker_map` TF in that launch yet. Confirm with:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic info --verbose /ov_msckf/marker_map_locked'
```

`Publisher count: 0` on the lock topic, repeated OpenVINS logs like `failed
static init: no accel jerk detected`, and no `/ov_msckf/odomimu` messages all
mean the VIO has not initialized. Move/tilt the D435i enough for OpenVINS
initialization and keep the marker board visible, then re-check
`tf2_echo marker_map head_cam0` and the transformed pointcloud topics.

## RViz Checks

Use `marker_map` as the fixed frame. Add these `PointCloud2` displays for the
common-frame grasping clouds:

```text
/head/d435i_head/points_marker_map
/arm/d435i_arm/points_marker_map
```

The raw clouds are useful for RealSense debugging, but they are normally viewed
with fixed frame set to the cloud's optical frame, for example
`arm_d435i_arm_depth_optical_frame`. The grasping algorithm should consume the
transformed `points_marker_map` topics.

For color in RViz, use the `RGB8` color transformer if available. If RViz does
not offer `RGB8`, remove and re-add the `PointCloud2` display after messages are
flowing, or re-select the topic. Echo one message and confirm that the fields
include `rgb`:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo --once /arm/d435i_arm/points_marker_map --field fields'
```

For raw RealSense color clouds, this check can be run before `marker_map` TF is
available:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo --once /arm/d435i_arm/depth/color/points --field fields'
```

Expected fields are `x`, `y`, `z`, and `rgb`.

## Troubleshooting The Blank RViz Case

- If RViz says it cannot transform from
  `arm_d435i_arm_depth_optical_frame` to `marker_map`, the selected topic is the
  raw RealSense cloud. Use `/arm/d435i_arm/points_marker_map` with fixed frame
  `marker_map`, or view the raw cloud with fixed frame
  `arm_d435i_arm_depth_optical_frame`.
- If `/arm/d435i_arm/points_marker_map` is selected but no points appear, echo
  `/arm/d435i_arm/points_marker_map/status`. `tf_unavailable` means OpenVINS has
  not initialized `marker_map` yet.
- CycloneDDS `Failed to parse type hash` warnings are discovery noise. Treat
  them as non-fatal if the topic has publishers and messages are flowing.

## Acceptance

- `/head/d435i_head/color/image_raw` and `/arm/d435i_arm/color/image_raw` stay
  near 30 Hz.
- `/head/d435i_head/imu` and `/arm/d435i_arm/imu` keep publishing and OpenVINS
  does not hit the `Propagator.cpp` timing assert with
  `enable_pointclouds:=false`.
- Raw pointcloud topics publish near 15 Hz or below when
  `enable_pointclouds:=true`.
- `/head/d435i_head/points_marker_map` and
  `/arm/d435i_arm/points_marker_map` publish when TF to `marker_map` is
  available. The conservative Jetson default is up to 10 Hz; override
  `pointcloud_max_rate_hz:=15.0` only if OpenVINS timing remains stable.
- Transformed pointcloud headers are `frame_id: marker_map`.
- With `enable_pointclouds:=false`, no pointcloud transform nodes are launched.
- Fixed ID0 and dynamic ID2 behavior is unchanged; ID2 is not added to
  `marker_fixed_ids`.
- CPU and memory load remain acceptable on the Jetson during grasping runs.

Rollback for pointcloud-specific OpenVINS testing is:

```bash
enable_pointclouds:=false
```

Rollback for the RealSense IMU/image publication-order change is:

```bash
hold_back_imu_for_frames:=false
```
