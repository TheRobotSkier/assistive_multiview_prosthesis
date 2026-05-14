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
```

The `pointcloud__neon_` parameters are included because the Jetson RealSense
build exposes the pointcloud filter under that parameter prefix. The top-level
live launch also passes through the delayed `enable_pointcloud_neon_fix` setter
so `pointcloud__neon_.enable` is re-applied after camera startup when
pointclouds are enabled.

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
- raw RealSense pointcloud topics published around 15 Hz when enabled
- transformed color pointclouds were visible in RViz in `marker_map`
- OpenVINS-only dual-camera baseline now runs much better with the Jetson-safe
  profile
- no ID2 was added to `marker_fixed_ids`
- fixed ID0 and dynamic ID2 topic separation was preserved

Still to validate before commit:

- OpenVINS-only baseline remains stable for one more run after all current file
  edits
- pointcloud-enabled run with conservative settings does not destabilize
  OpenVINS
- transformed pointcloud headers are `marker_map`
- both `/head/d435i_head/points_marker_map` and
  `/arm/d435i_arm/points_marker_map` publish while OpenVINS remains alive

## Recommended Next Tests

First rerun OpenVINS-only baseline if needed:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=false start_preview:=false start_rviz:=false'
```

Then run the conservative pointcloud test:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true start_preview:=false start_rviz:=false pointcloud_max_rate_hz:=8.0 pointcloud_voxel_leaf_m:=0.03 pointcloud_max_range_m:=1.5 pointcloud_decimation_magnitude:=4'
```

Check transformed pointcloud rates:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /head/d435i_head/points_marker_map'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /arm/d435i_arm/points_marker_map'
```

Check pointcloud frame IDs:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo --once /head/d435i_head/points_marker_map --field header.frame_id'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo --once /arm/d435i_arm/points_marker_map --field header.frame_id'
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

- keep OpenVINS, marker-map TF, and raw camera streams on the Jetson
- move optional merge, segmentation-specific crop/downsample, and segmentation
  input preparation to the x86 Ubuntu PC
- publish the segmentation-ready pointcloud as `/segmentation/input_cloud`
  in `marker_map`
- preserve the existing `/segmentation/click_positive` target for future hit
  or click commands

## Commit Recommendation

Do not commit until the conservative pointcloud-enabled test is complete.

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
