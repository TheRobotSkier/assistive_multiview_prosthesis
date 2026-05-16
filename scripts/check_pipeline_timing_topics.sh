#!/bin/bash
# check_pipeline_timing_topics.sh — runtime sanity check for stamped pipeline topics.

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi

set -euo pipefail

TIMEOUT="${TOPIC_TIMEOUT_SEC:-8}"

check_header_topic() {
    local topic="$1"
    local type="$2"
    local out="/tmp/$(echo "$topic" | tr / _)_timing.log"
    echo "Checking stamped topic ${topic}"
    timeout "$TIMEOUT" ros2 topic echo --once "$topic" "$type" > "$out"
    grep -q "stamp:" "$out"
    grep -q "frame_id:" "$out"
}

check_header_topic /fused_pointcloud sensor_msgs/msg/PointCloud2
check_header_topic /segmentation/input_cloud sensor_msgs/msg/PointCloud2
check_header_topic /hand_pose geometry_msgs/msg/PoseStamped
check_header_topic /hand_twist geometry_msgs/msg/TwistStamped

echo "Pipeline timing topics OK"
