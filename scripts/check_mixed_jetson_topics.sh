#!/bin/bash
# check_mixed_jetson_topics.sh — verify current mixed D435 + D435i Jetson topics.

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi

set -euo pipefail

TIMEOUT="${TOPIC_TIMEOUT_SEC:-8}"

check_topic_once() {
    local topic="$1"
    local type="$2"
    echo "Checking ${topic}"
    timeout "$TIMEOUT" ros2 topic echo --once "$topic" "$type" >/tmp/check_mixed_topic.log
}

check_topic_listed() {
    local topic="$1"
    ros2 topic list | grep -qx "$topic"
}

check_topic_listed /head/d435i_head/color/image_raw
check_topic_listed /head/d435i_head/depth/color/points
check_topic_listed /head/d435i_head/imu
check_topic_listed /arm/d435i_arm/color/image_raw
check_topic_listed /arm/d435i_arm/depth/color/points
check_topic_listed /arm/d435i_arm/imu

check_topic_once /head/d435i_head/imu sensor_msgs/msg/Imu
check_topic_once /arm/d435i_arm/imu sensor_msgs/msg/Imu
check_topic_once /head/d435i_head/depth/color/points sensor_msgs/msg/PointCloud2
check_topic_once /arm/d435i_arm/depth/color/points sensor_msgs/msg/PointCloud2

echo "Mixed Jetson camera/IMU topics OK"
