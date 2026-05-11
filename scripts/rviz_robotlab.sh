#!/bin/bash
# Launch RViz on robotlab's Docker container, displaying on this host via SSH X11 forwarding
# Usage: ./scripts/rviz_robotlab.sh

set -e

# Allow local X connections
xhost +localhost >/dev/null 2>&1 || true

echo "Connecting to robotlab and launching RViz in Docker container..."

ssh -X -o ForwardX11Trusted=yes robotlab bash << 'RVEOF'
# Get display from SSH
echo "DISPLAY=$DISPLAY"

# Get running container
CID=$(echo robotlab | sudo -S docker ps -q 2>/dev/null | head -1)
if [ -z "$CID" ]; then
    echo "ERROR: No robotlab container running. Start with: make up"
    exit 1
fi

echo "Container: $CID"

# Launch RViz in container with forwarded display
echo robotlab | sudo -S docker exec -e DISPLAY="$DISPLAY" "$CID" \
    bash -c 'source /opt/ros/jazzy/setup.bash && rviz2 -d /miahand_ws/src/jetson_folder/two_d435_test.rviz 2>&1' &
RVEOF

echo "RViz launched. It should appear on your display shortly."
