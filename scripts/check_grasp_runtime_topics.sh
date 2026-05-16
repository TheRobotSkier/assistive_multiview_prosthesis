#!/bin/bash
# check_grasp_runtime_topics.sh — runtime topic/service check for grasp pipeline.

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi

set -euo pipefail

ros2 service list | grep -qx /grasp_preshaping/compute_grasp
ros2 topic list | grep -qx /grasp_preshaping/target_hand_pose
ros2 topic list | grep -qx /grasp_preshaping/target_finger_closures
ros2 topic list | grep -qx /thumb_pos_ff_controller/commands
ros2 topic list | grep -qx /index_pos_ff_controller/commands
ros2 topic list | grep -qx /mrl_pos_ff_controller/commands
ros2 topic list | grep -qx /joint_states

echo "Grasp runtime interfaces listed"
