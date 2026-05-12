#!/bin/bash
# Launch RViz from robotlab's Docker, displayed on this host via X11
# Usage: ./scripts/rviz_robotlab.sh

set -e

# Generate and share X authority cookie
xauth list "$DISPLAY" 2>/dev/null | head -1 > /tmp/rviz_xcookie

# Copy cookie to robotlab
ssh robotlab "cat > /tmp/rviz_xcookie" < /tmp/rviz_xcookie
ssh robotlab "xauth add \$(cat /tmp/rviz_xcookie) 2>/dev/null; echo 'XAUTH ready'"

# Get running container
CID=$(ssh robotlab 'echo robotlab | sudo -S docker ps -q 2>/dev/null | head -1')
if [ -z "$CID" ]; then
    echo "ERROR: No running container on robotlab. Run: ssh robotlab 'cd ~/jetson_ws/jetson-docker && make up'"
    exit 1
fi

echo "Container: $CID"
echo "Launching RViz via X forwarding..."

# Launch RViz in container
ssh -X -o ForwardX11Trusted=yes robotlab \
    "echo robotlab | sudo -S docker exec -e DISPLAY=\$DISPLAY $CID \
     bash -c 'source /opt/ros/jazzy/setup.bash && rviz2'"

echo "RViz closed."
