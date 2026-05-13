#!/bin/bash
# Launch RViz2 via podman with X11 forwarding to view robotlab camera data
# Robotlab Docker publishes on host network (10.42.0.2)
# This host is 10.42.0.1 on the same subnet
# ROS2 discovery works via CycloneDDS on host network

set -e

RVIZ_CONFIG="${1:-/tmp/robotlab_cameras.rviz}"

# Create RViz config if it doesn't exist
if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "RViz config not found at $RVIZ_CONFIG"
    echo "Creating default config..."
    python3 -c "
import os
config = '''Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ''
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
    - Class: rviz_default_plugins/TF
      Name: TF
    - Alpha: 1
      Class: rviz_default_plugins/PointCloud2
      Color Transformer: RGB8
      Enabled: true
      Name: Head PointCloud
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
        Value: /head/d435_head/depth/color/points
      Size (Pixels): 3
      Use Fixed Frame: true
      Use rainbow: false
    - Alpha: 1
      Class: rviz_default_plugins/PointCloud2
      Color Transformer: RGB8
      Enabled: true
      Name: Arm PointCloud
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
        Value: /arm/d435_arm/depth/color/points
      Size (Pixels): 3
      Use Fixed Frame: true
      Use rainbow: false
    - Class: rviz_default_plugins/Image
      Enabled: true
      Name: Head Color Image
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
        Value: /head/d435_head/color/image_raw
    - Class: rviz_default_plugins/Image
      Enabled: true
      Name: Arm Color Image
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
        Value: /arm/d435_arm/color/image_raw
  Enabled: true
  Global Options:
    Fixed Frame: head_d435_head_depth_optical_frame
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.0
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Name: Current View
      Pitch: 0.5
      Yaw: 0.5
'''
with open('$RVIZ_CONFIG', 'w') as f:
    f.write(config)
"
fi

echo "Launching RViz2 via podman..."
echo "Config: $RVIZ_CONFIG"
echo "Display: $DISPLAY"

podman run --rm -it \
    --network host \
    -e DISPLAY="$DISPLAY" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e ROS_DOMAIN_ID=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(dirname "$RVIZ_CONFIG"):/rviz_config:ro" \
    docker.io/osrf/ros:jazzy-desktop \
    bash -c "source /opt/ros/jazzy/setup.bash && rviz2 -d /rviz_config/$(basename "$RVIZ_CONFIG")"
