#!/bin/bash
# test_grasp_contracts.sh — static checks for segmentation -> grasp -> hand commands.

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi

set -euo pipefail

WS_ROOT="${PROSTHESIS_WS:-/prosthesis_ws}"
if [ ! -d "$WS_ROOT/src" ]; then
    WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

python3 -m py_compile \
    "$WS_ROOT/src/pipeline_manager/pipeline_manager/digital_twin_joint_state_publisher.py" \
    "$WS_ROOT/src/grasp_preshaping/nodes/grasp_proximity_controller_node.py"

python3 - "$WS_ROOT" <<'PY'
import sys
from pathlib import Path
import yaml

root = Path(sys.argv[1])
cfg = yaml.safe_load((root / "config/prosthesis_config.yaml").read_text())
topics = cfg["topics"]
bridge = (root / "src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp").read_text()
launch = (root / "src/prosthesis_launch/launch/digital_twin.launch.py").read_text()
joint = (root / "src/pipeline_manager/pipeline_manager/digital_twin_joint_state_publisher.py").read_text()
pipeline = (root / "src/pipeline_manager/pipeline_manager/pipeline_manager_node.py").read_text()
twist = (root / "src/twist_propagation/twist_propagation/twist_propagation_node.py").read_text()

assert topics["segmented_object_cloud"] == "/segmentation/object_cloud"
assert topics["compute_grasp_service"] == "/grasp_preshaping/compute_grasp"
assert topics["preshaping_target_hand_pose"] == "/grasp_preshaping/target_hand_pose"
assert topics["preshaping_target_closures"] == "/grasp_preshaping/target_finger_closures"
assert topics["thumb_cmd"] == "/thumb_pos_ff_controller/commands"
assert topics["index_cmd"] == "/index_pos_ff_controller/commands"
assert topics["mrl_cmd"] == "/mrl_pos_ff_controller/commands"

assert 'declare_parameter<std::string>("cloud_topic", "/segmentation/object_cloud")' in bridge
assert 'declare_parameter<std::string>("compute_service", "/grasp_preshaping/compute_grasp")' in bridge
assert "create_subscription<sensor_msgs::msg::PointCloud2>" in bridge
assert "create_publisher<geometry_msgs::msg::PoseStamped>" in bridge
assert "create_publisher<std_msgs::msg::Float64MultiArray>" in bridge
assert "target_pose.header.frame_id = \"world\"" in bridge
assert "publish_joint_commands(preshape_thumb, preshape_index, preshape_mrl)" in bridge
assert "target_finger_closures_pub_->publish(closures)" in bridge

assert '"camera_frames": [' in launch
assert '"head_d435i_head_color_optical_frame"' in launch
assert '"arm_d435i_arm_color_optical_frame"' in launch
assert '"publish_initial_commands": False' in launch
assert 'executable="digital_twin_joint_state_publisher"' in launch

assert '"/thumb_pos_ff_controller/commands"' in joint
assert '"/index_pos_ff_controller/commands"' in joint
assert '"/mrl_pos_ff_controller/commands"' in joint
assert '"/joint_states"' in joint

assert 'PointCloud2, \'/segmentation/object_cloud\'' in pipeline
assert 'Trigger, \'/grasp_preshaping/compute_grasp\'' in pipeline
assert 'self._compute_client.call_async' in twist
PY

echo "Grasp contracts OK"
