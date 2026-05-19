#!/bin/bash
# EMG armband container entrypoint.
#
# The MindRove armband connects over WiFi — no USB serial device is needed.

set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /emg_ws/install/setup.bash ]; then
    source /emg_ws/install/setup.bash
fi

exec "$@"
