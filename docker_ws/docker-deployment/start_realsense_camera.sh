#!/bin/bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [ -f /miahand_ws/install/setup.bash ]; then
  source /miahand_ws/install/setup.bash
fi

set -u

ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true \
  align_depth.enable:=true \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15 &
CAM_PID=$!

sleep 6
ros2 param set /camera/camera pointcloud__neon_.enable true || true

wait $CAM_PID