#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
# Prefer workspace overlay mounted at /miahand_ws (bind-mounted by compose) if present
if [ -f /miahand_ws/install/setup.bash ]; then
  source /miahand_ws/install/setup.bash
elif [ -f /haptic_ws/install/setup.bash ]; then
  source /haptic_ws/install/setup.bash
fi
exec "$@"
