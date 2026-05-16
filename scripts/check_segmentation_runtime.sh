#!/bin/bash
# check_segmentation_runtime.sh — runtime topic check for segmentation bridge.

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi

set -euo pipefail

ros2 topic list | grep -qx /segmentation/input_cloud
ros2 topic list | grep -qx /segmentation/click_positive
ros2 topic list | grep -qx /segmentation/object_cloud

echo "Segmentation runtime topics listed"
