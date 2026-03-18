#!/bin/bash
source /opt/ros/humble/setup.bash

gnome-terminal -- bash -c "
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=cam1 \
  camera_name:=d435_1 \
  serial_no:=_829212072207 \
  enable_sync:=true \
  align_depth.enable:=true \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15; exec bash"

gnome-terminal -- bash -c "
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=cam2 \
  camera_name:=d435_2 \
  serial_no:=_827112072033 \
  enable_sync:=true \
  align_depth.enable:=true \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15; exec bash"


sleep 5

ros2 param set /cam1/d435_1 pointcloud__neon_.enable true
ros2 param set /cam2/d435_2 pointcloud__neon_.enable true

rviz2