# D435i OpenVINS And Color Pointcloud Implementation Report

Date: 2026-05-14

## Summary

This implementation adds opt-in color pointcloud support for both D435i cameras
while preserving the existing OpenVINS plus fixed-ID0 and dynamic-ID2 marker
update workflow.

The largest practical result from today's testing is that the dual-camera
OpenVINS live baseline is now much more stable on the Jetson after applying a
Jetson-safe OpenVINS profile. RealSense RGB still publishes at `640x480x30`, so
the camera calibration target rate is preserved. OpenVINS now processes a
lighter subset of that stream internally.

Latest pointcloud visibility update: the transform node now subscribes to raw
RealSense clouds with sensor-data/best-effort QoS, then republishes transformed
`points_marker_map` topics with reliable shallow-queue QoS. This keeps the raw
high-bandwidth path drop-friendly while making the transformed topics visible to
default RViz PointCloud2 displays and plain `ros2 topic` CLI commands.

Final live validation update: after disabling the unintended RealSense infrared
streams, color pointclouds are visible in RViz2 while OpenVINS remains healthy.
The status topic reports alternating `published` and `rate_limited`, which is
the expected behavior when raw pointclouds arrive faster than
`pointcloud_max_rate_hz`.

## Implemented Changes

### Opt-In D435i Color Pointclouds

Pointclouds are disabled by default for OpenVINS-only runs and enabled only when
requested:

```bash
enable_pointclouds:=true
```

The live workflow defaults remain safe for OpenVINS testing:

```bash
enable_pointclouds:=false
```

RealSense color image input remains:

```text
enable_color: true
rgb_camera.color_profile: 640x480x30
```

Pointcloud/depth settings when enabled:

```text
enable_depth: true
enable_infra: false
enable_infra1: false
enable_infra2: false
depth_module.depth_profile: 640x480x15
pointcloud.enable: true
pointcloud__neon_.enable: true
enable_pointcloud_neon_fix: true
pointcloud.stream_filter: 2
pointcloud.stream_index_filter: 0
pointcloud.ordered_pc: false
pointcloud.allow_no_texture_points: false
align_depth.enable: false
decimation_filter.enable: true
decimation_filter.filter_magnitude: 3
clip_distance: 2.0
enable_pointcloud_neon_fix: false
```

The `pointcloud__neon_` parameters are included because the Jetson RealSense
build exposes the pointcloud filter under that parameter prefix. The delayed
`enable_pointcloud_neon_fix` setter is now a legacy fallback only; startup
parameters enabled raw pointclouds reliably during validation, while the delayed
setter could print distracting `Node not found` errors when DDS discovery lagged.

The explicit infrared disables were added after a raw camera test showed the
custom launch opening `Infra(1)` and `Infra(2)` profiles in addition to depth and
color, followed by RealSense `Frames didn't arrived within 5 seconds` depth
timeouts. The official `rs_launch.py` defaults keep infrared streams disabled and
allowed depth frames to publish, so the custom D435i launches now match that
behavior.

### Stable Pointcloud Topics

Raw RealSense color pointclouds:

```text
/head/d435i_head/depth/color/points
/arm/d435i_arm/depth/color/points
```

Transformed grasping topics:

```text
/head/d435i_head/points_marker_map
/arm/d435i_arm/points_marker_map
```

The transformed topics publish in:

```text
marker_map
```

By default, transformed pointclouds are gated by TF availability rather than by
the explicit marker-map lock topic:

```text
pointcloud.require_marker_map_locked: false
```

This avoids dropping clouds when `marker_map` TF is already available but the
lock status topic is not latched/visible to a late subscriber.

### Pointcloud Transform Node

Added/updated:

```text
docker_ws/multi_cam_localization/sensor_fusion_bringup/src/pointcloud_to_frame_node.cpp
```

The node:

- subscribes to each raw RealSense pointcloud
- looks up TF into `marker_map`
- republishes a stable common-frame pointcloud topic
- uses sensor-data/best-effort QoS for raw pointcloud input to match RealSense
  high-bandwidth sensor streams and avoid DDS backpressure
- republishes transformed `points_marker_map` topics with reliable shallow-queue
  QoS so default RViz PointCloud2 displays and plain `ros2 topic` checks work
- rate-limits output
- optionally applies source-frame range filtering
- optionally applies voxel downsampling
- publishes JSON status messages on `<output_topic>/status`

Default pointcloud performance controls:

```text
pointcloud.max_rate_hz: 10.0
pointcloud.voxel_leaf_m: 0.02
pointcloud.max_range_m: 2.0
pointcloud.decimation_magnitude: 3
pointcloud.require_marker_map_locked: false
```

These defaults are intentionally conservative for the Jetson. Denser pointclouds
can be tested only after the OpenVINS-only baseline is stable.

### Direct RealSense Launch Parameters

The head, arm, single, and dual D435i launch files now pass the important
RealSense parameters directly to `realsense2_camera_node` instead of depending on
the upstream `rs_launch.py` argument list. This avoids failures when the local
RealSense launch file does not expose every needed argument.

Updated launch/config files include:

```text
docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py
docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py
docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dynamic_id2_arm_update_live.launch.py
docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dual_d435i.launch.py
docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/single_d435i.launch.py
docker_ws/multi_cam_localization/sensor_fusion_bringup/config/d435i_cameras.yaml
docker_ws/multi_cam_localization/sensor_fusion_bringup/config/dynamic_id2_arm_update.yaml
```

### RealSense IMU Publication Ordering

Both D435i launches now default to:

```text
hold_back_imu_for_frames: true
```

This is a live RealSense driver setting. It is intended to keep image and IMU
publication order more chronological under load. It does not rewrite timing when
replaying an already-recorded rosbag.

### OpenVINS Crash And Backlog Guards

OpenVINS changes:

```text
docker_ws/src/open_vins/ov_msckf/src/core/VioManager.h
docker_ws/src/open_vins/ov_msckf/src/ros/ROS1Visualizer.cpp
docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp
docker_ws/src/open_vins/ov_msckf/src/state/Propagator.cpp
```

Implemented:

- `VioManager::get_params()` now returns a `const` reference instead of
  deep-copying the full options object repeatedly.
- The ROS2 OpenVINS update-thread gate uses atomic compare-and-set.
- The detached ROS2 update thread captures only the IMU timestamp by value,
  avoiding unsafe references to callback-local data.
- The internal OpenVINS camera queue is capped at `3` frames and drops stale
  frames if processing falls behind.
- The old fatal `Propagator.cpp` assertion is now a non-fatal `[PROP]`
  diagnostic.

Important caveat: repeated `[PROP]` messages from the same old start timestamp
still indicate a failed OpenVINS baseline. The change prevents process death so
the failure can be diagnosed instead of crashing immediately.

An attempted OpenVINS image subscriber sensor-data QoS change was rolled back
because it caused immediate `double free or corruption` startup failures on the
Jetson.

### Jetson-Safe OpenVINS Profile

Both head and arm D435i OpenVINS configs were changed to a lighter dual-camera
profile:

```text
track_frequency: 21.0
num_pts: 200
fast_threshold: 25
min_px_dist: 15
max_clones: 8
max_slam: 25
max_msckf_in_update: 25
num_opencv_threads: 2
```

Previous heavy live profile:

```text
track_frequency: 31.0
num_pts: 300
fast_threshold: 15
min_px_dist: 10
max_clones: 11
max_slam: 50
max_msckf_in_update: 40
num_opencv_threads: 4
```

Why this was needed:

- Two OpenVINS instances were competing for CPU and memory bandwidth.
- The Jetson showed high CPU and EMC load.
- Even with pointclouds disabled, the heavy profile could fall behind and hit
  the propagation timing failure path.
- The lighter profile keeps RealSense at `30 Hz` but reduces the estimator work
  enough for the live baseline to run much better.

Validated user result after this change:

- dual-camera OpenVINS-only run was tested for about 10 minutes
- pointclouds, preview, and RViz were off
- user reported it was the best dual-camera run so far
- only occasional `[QUEUE]` near the end
- drift/reanchor behavior still needs separate investigation later

## Build And Repair Notes

OpenVINS builds on the Jetson must be sequential and should be done only when
the Jetson is otherwise idle:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon --log-base log_overlay build --symlink-install --build-base build_overlay --install-base install_overlay --executor sequential --parallel-workers 1 --packages-select sensor_fusion_bringup ov_msckf'
```

An interrupted build left `run_subscribe_msckf_marker` as a zero-byte file. The
repair command was:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 cmake --build build_overlay/ov_msckf --target run_subscribe_msckf_marker --clean-first -- -j1'
```

After repair:

```text
ros2 pkg executables ov_msckf
```

reported:

```text
ov_msckf run_subscribe_msckf_marker
```

## Validation Status

Validated:

- raw RealSense color image topics remained close to 30 Hz in earlier tests
- raw RealSense depth topics published around 15 Hz in single and dual-camera
  tests after disabling infrared streams
- raw RealSense pointcloud topics published around 15 Hz in dual-camera raw
  tests and around 10-11 Hz in the full OpenVINS launch under load
- transformed color pointcloud topics published when test TFs supplied
  `marker_map -> head_cam0` and `marker_map -> arm_cam0`, proving the transform
  node and RViz-facing topics work once OpenVINS camera TF exists
- live RViz2 validation confirmed color pointclouds are visible in `marker_map`
  while OpenVINS remains stable
- `/head/d435i_head/points_marker_map/status` reported `accepted:true` with
  `reason:"published"` interleaved with expected `reason:"rate_limited"` drops
- OpenVINS-only dual-camera baseline now runs much better with the Jetson-safe
  profile
- no ID2 was added to `marker_fixed_ids`
- fixed ID0 and dynamic ID2 topic separation was preserved

Remaining validation before deeper follow-up work:

- optionally repeat the arm status/rate check for
  `/arm/d435i_arm/points_marker_map/status` and
  `/arm/d435i_arm/points_marker_map`
- record one short pointcloud-enabled bag if later regression testing needs a
  known-good reference
- investigate the remaining long-run drift/reanchor behavior in a fresh focused
  task after committing this implementation baseline

## Recommended Next Tests

First rerun OpenVINS-only baseline if needed:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=false start_preview:=false start_rviz:=false'
```

Then run the conservative pointcloud test:

```bash
docker compose run --rm --name openvins_pc realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true start_preview:=false start_rviz:=false pointcloud_require_marker_map_locked:=false pointcloud_max_rate_hz:=8.0 pointcloud_voxel_leaf_m:=0.02 pointcloud_max_range_m:=2.0 pointcloud_decimation_magnitude:=3 enable_pointcloud_neon_fix:=false'
```

In another terminal, first confirm raw pointclouds are flowing:

```bash
docker exec openvins_pc bash -lc 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && timeout 8 ros2 topic hz /head/d435i_head/depth/color/points'
```

```bash
docker exec openvins_pc bash -lc 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && timeout 8 ros2 topic hz /arm/d435i_arm/depth/color/points'
```

Then check transformed pointcloud status. If this says `tf_unavailable` for
`head_cam0` or `arm_cam0`, OpenVINS has not initialized the camera TF yet; move
or tilt the cameras enough for initialization while keeping the marker visible.

```bash
docker exec openvins_pc bash -lc 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && timeout 8 ros2 topic echo /head/d435i_head/points_marker_map/status'
```

```bash
docker exec openvins_pc bash -lc 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && timeout 8 ros2 topic echo /arm/d435i_arm/points_marker_map/status'
```

Once status reports published clouds, check rates and frame IDs:

```bash
docker exec openvins_pc bash -lc 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && timeout 8 ros2 topic hz /head/d435i_head/points_marker_map'
```

```bash
docker exec openvins_pc bash -lc 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && timeout 8 ros2 topic hz /arm/d435i_arm/points_marker_map'
```

```bash
docker exec openvins_pc bash -lc 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo --once /head/d435i_head/points_marker_map --field header.frame_id'
```

```bash
docker exec openvins_pc bash -lc 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo --once /arm/d435i_arm/points_marker_map --field header.frame_id'
```

Expected frame:

```text
marker_map
```

## X86 Segmentation Follow-Up

The x86 segmentation-side plan is documented in:

```text
.agents/x86_segmentation_pointcloud_plan.md
```

The recommended architecture is:

- keep OpenVINS, marker-map TF, RealSense image/IMU, and the opt-in
  `points_marker_map` republishers on the Jetson for now
- move segmentation, optional cloud merge, segmentation-specific crop/downsample,
  and any heavier grasp-input preparation to the x86 Ubuntu PC
- publish the segmentation-ready pointcloud as `/segmentation/input_cloud` in
  `marker_map`
- preserve the existing `/segmentation/click_positive` target for future hit
  or click commands

For the first x86 integration, subscribe to the already validated Jetson topics:

```text
/head/d435i_head/points_marker_map
/arm/d435i_arm/points_marker_map
```

If Jetson CPU/EMC becomes too high, move the marker-map pointcloud transform
and merge/downsample step to the x86 PC later by subscribing to raw RealSense
pointclouds plus `/tf` and `/tf_static`.

## Commit Recommendation

This is a reasonable point to commit the implementation baseline if the user has
confirmed RViz2 color pointcloud visibility and OpenVINS stability.

Before committing, remove generated build/install/log overlay changes from the
git index/worktree view so only source/config/docs changes are committed.

Suggested commit message after final validation:

```text
Add Jetson-safe D435i marker-map pointcloud support
```

If splitting into multiple commits:

```text
Add D435i marker-map color pointcloud support
Stabilize dual D435i OpenVINS on Jetson
Document x86 pointcloud segmentation handoff
```
