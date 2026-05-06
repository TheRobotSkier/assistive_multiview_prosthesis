#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u

launch_args=(
  "enable_color:=${REALSENSE_ENABLE_COLOR:-true}"
  "pointcloud.enable:=${REALSENSE_ENABLE_POINTCLOUD:-true}"
  "align_depth.enable:=${REALSENSE_ALIGN_DEPTH:-true}"
  "depth_module.depth_profile:=${REALSENSE_DEPTH_PROFILE:-640x480x15}"
  "rgb_camera.color_profile:=${REALSENSE_COLOR_PROFILE:-640x480x15}"
  "camera_namespace:=${REALSENSE_CAMERA_NAMESPACE:-cam1}"
  "camera_name:=${REALSENSE_CAMERA_NAME:-d435_1}"
  "initial_reset:=${REALSENSE_INITIAL_RESET:-false}"
)

if [ -n "${REALSENSE_SERIAL_NO:-}" ]; then
  launch_args+=("serial_no:=${REALSENSE_SERIAL_NO}")
fi

echo "Launching official realsense2_camera node with arguments:"
printf '  %s\n' "${launch_args[@]}"

exec ros2 launch realsense2_camera rs_launch.py "${launch_args[@]}"
