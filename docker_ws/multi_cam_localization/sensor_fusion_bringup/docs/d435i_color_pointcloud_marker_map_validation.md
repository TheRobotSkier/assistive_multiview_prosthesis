# D435i Color Pointcloud Marker-Map Validation

This workflow keeps OpenVINS RGB input at 30 Hz and makes color pointclouds
opt-in for grasping runs.

Run commands from:

```bash
cd /home/robotlab/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment
```

## Build

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon --log-base log_overlay build --symlink-install --build-base build_overlay --install-base install_overlay --executor sequential --parallel-workers 1 --packages-select sensor_fusion_bringup ov_msckf'
```

Smoke-check the launch arguments and executable:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 pkg executables sensor_fusion_bringup | grep pointcloud_to_frame_node && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py --show-args'
```

## Launch Modes

OpenVINS-only default, with pointcloud processing off:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=false'
```

Grasping run, with 15 Hz depth/color pointclouds and marker-map republishers:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true'
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

Raw RealSense color pointcloud topics should be near 15 Hz or below:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /head/d435i_head/depth/color/points'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /arm/d435i_arm/depth/color/points'
```

Transformed grasping topics should publish only after marker-map lock:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /head/d435i_head/points_marker_map'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /arm/d435i_arm/points_marker_map'
```

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

Status topics explain dropped clouds before lock, stale TF, or rate limiting:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo /head/d435i_head/points_marker_map/status'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo /arm/d435i_arm/points_marker_map/status'
```

Before marker-map lock, expect status reason `marker_map_not_locked`. After lock
and valid TF, expect `published`.

## RViz Checks

Use `marker_map` as the fixed frame. Add these `PointCloud2` displays:

```text
/head/d435i_head/depth/color/points
/head/d435i_head/points_marker_map
/arm/d435i_arm/depth/color/points
/arm/d435i_arm/points_marker_map
```

The raw clouds are useful for RealSense debugging. The grasping algorithm should
consume the transformed `points_marker_map` topics.

## Acceptance

- `/head/d435i_head/color/image_raw` and `/arm/d435i_arm/color/image_raw` stay
  near 30 Hz.
- Raw pointcloud topics publish near 15 Hz or below when
  `enable_pointclouds:=true`.
- `/head/d435i_head/points_marker_map` and
  `/arm/d435i_arm/points_marker_map` publish after marker-map lock.
- Transformed pointcloud headers are `frame_id: marker_map`.
- With `enable_pointclouds:=false`, no pointcloud transform nodes are launched.
- Fixed ID0 and dynamic ID2 behavior is unchanged; ID2 is not added to
  `marker_fixed_ids`.
- CPU and memory load remain acceptable on the Jetson during grasping runs.

Rollback for OpenVINS testing is simply:

```bash
enable_pointclouds:=false
```
