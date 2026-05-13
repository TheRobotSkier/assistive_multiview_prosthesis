#!/bin/bash
# jetson/scripts/list_cameras.sh
# List connected Intel RealSense cameras and their serial numbers.
# Runs inside the cameras_test container.

SUDO=${SUDO:-}

if ! $SUDO docker ps --format '{{.Names}}' 2>/dev/null | grep -q cameras_test; then
    echo "cameras_test container not running. Start it first with 'make cameras'."
    exit 1
fi

$SUDO docker exec cameras_test bash -c '
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    /opt/ros/jazzy/bin/rs-enumerate-devices -s 2>/dev/null
' 2>&1 | grep -v "^ " | grep -v "^$" | grep -v "^ 01/"
