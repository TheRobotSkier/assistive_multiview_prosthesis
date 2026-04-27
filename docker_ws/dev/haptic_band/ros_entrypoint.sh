#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /haptic_ws/install/setup.bash
exec "$@"
