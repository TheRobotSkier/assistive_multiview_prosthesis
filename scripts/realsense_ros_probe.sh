#!/usr/bin/env bash
# Probe script for the RealSense camera(s) using the fixed launch files.
#
# This script uses one_d435_launch.py or two_d435_launch.py (which work around
# the YAML serialisation bug by using ExecuteProcess directly).
#
# Environment variables (all optional):
#   CAM1_SERIAL             Camera 1 serial (default: 829212072207)
#   CAM2_SERIAL             Camera 2 serial (default: empty → single-camera mode)
#   CAM2_OFFSET_X           X-offset cam1→cam2 (default: 0.15)
#   REALSENSE_ENABLE_COLOR  "true" / "false" (default: true)
#   REALSENSE_INITIAL_RESET "true" / "false" (default: false)

set -eo pipefail
source /opt/ros/humble/setup.bash
set -u

if [ -n "${CAM2_SERIAL:-}" ]; then
  echo "=== Dual-camera mode: cam1=${CAM1_SERIAL:-829212072207} cam2=${CAM2_SERIAL:-827112072033} ==="
  exec ros2 launch /ros_ws/launch/two_d435_launch.py
else
  echo "=== Single-camera mode: cam1=${CAM1_SERIAL:-829212072207} ==="
  exec ros2 launch /ros_ws/launch/one_d435_launch.py
fi
