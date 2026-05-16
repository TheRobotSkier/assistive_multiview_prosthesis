#!/usr/bin/env bash
# Runtime smoke check for final full pipeline after make up-full-digital-twin.

set -euo pipefail

TIMEOUT="${TIMEOUT:-8}"

set +u
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi
set -u

require_topic_once() {
    local topic="$1"
    local type="$2"
    echo "Checking $topic"
    timeout "$TIMEOUT" ros2 topic echo --once "$topic" "$type" >/tmp/check_full_pipeline_topic.log
}

require_topic_listed() {
    local topic="$1"
    echo "Checking $topic exists"
    timeout "$TIMEOUT" bash -c "until ros2 topic list | grep -Fx '$topic' >/dev/null; do sleep 0.25; done"
}

require_topic_once "/head/d435i_head/depth/color/points" "sensor_msgs/msg/PointCloud2"
require_topic_once "/arm/d435i_arm/depth/color/points" "sensor_msgs/msg/PointCloud2"
require_topic_once "/ov_msckf_head/odomimu" "nav_msgs/msg/Odometry"
require_topic_once "/ov_msckf_arm/odomimu" "nav_msgs/msg/Odometry"
require_topic_once "/fused_pointcloud" "sensor_msgs/msg/PointCloud2"

require_topic_listed "/segmentation/object_cloud"
require_topic_listed "/joint_states"
require_topic_listed "/thumb_fle/commands"
require_topic_listed "/index_fle/commands"
require_topic_listed "/mrl_fle/commands"

echo "Full pipeline runtime topics OK"
