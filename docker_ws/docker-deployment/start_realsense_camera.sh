#!/bin/bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [ -f /miahand_ws/install/setup.bash ]; then
  source /miahand_ws/install/setup.bash
fi

exec ros2 launch sensor_fusion_bringup dual_d435i.launch.py