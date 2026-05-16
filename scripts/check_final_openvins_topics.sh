#!/bin/bash
# check_final_openvins_topics.sh — verify final dual-D435i OpenVINS runtime topics.

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi

set -euo pipefail

TIMEOUT="${TOPIC_TIMEOUT_SEC:-10}"

check_topic_listed() {
    local topic="$1"
    ros2 topic list | grep -qx "$topic"
}

check_topic_once() {
    local topic="$1"
    local type="$2"
    echo "Checking ${topic}"
    timeout "$TIMEOUT" ros2 topic echo --once "$topic" "$type" >/tmp/check_final_openvins_topic.log
}

check_tf_frame() {
    local parent="$1"
    local child="$2"
    echo "Checking TF ${parent} -> ${child}"
    timeout "$TIMEOUT" ros2 run tf2_ros tf2_echo "$parent" "$child" >/tmp/check_final_openvins_tf.log
}

check_topic_listed /ov_msckf_head/odomimu
check_topic_listed /ov_msckf_arm/odomimu
check_topic_listed /head/marker_pose/observation
check_topic_listed /arm/marker_pose/observation

check_topic_once /ov_msckf_head/odomimu nav_msgs/msg/Odometry
check_topic_once /ov_msckf_arm/odomimu nav_msgs/msg/Odometry
check_topic_once /head/marker_pose/observation sensor_fusion_msgs/msg/MarkerPoseObservation
check_topic_once /arm/marker_pose/observation sensor_fusion_msgs/msg/MarkerPoseObservation

check_tf_frame marker_map head_imu
check_tf_frame marker_map arm_imu

echo "Final dual-D435i OpenVINS topics/TF OK"
