#!/bin/bash
# test_digital_twin_hardware_bypass_contracts.sh — RViz hand, no physical hand.

set -euo pipefail

WS_ROOT="${PROSTHESIS_WS:-/prosthesis_ws}"
if [ ! -d "$WS_ROOT/src" ]; then
    WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

python3 - "$WS_ROOT" <<'PY'
import sys
from pathlib import Path
import yaml

root = Path(sys.argv[1])
compose = yaml.safe_load((root / "docker/docker-compose.yml").read_text())
digital = compose["services"]["digital_twin"]
rviz = compose["services"]["digital_twin_rviz"]
launch = (root / "src/prosthesis_launch/launch/digital_twin.launch.py").read_text()
joint = (root / "src/pipeline_manager/pipeline_manager/digital_twin_joint_state_publisher.py").read_text()
position = (root / "src/pipeline_manager/pipeline_manager/digital_twin_position_grasp_controller.py").read_text()
urdf = (root / "src/mia_hand_ros2_control/description/urdf/mia_hand_digital_twin.urdf.xacro").read_text()

cmd = " ".join(digital["command"])
env = set(digital["environment"])
volumes = set(digital["volumes"])
rviz_cmd = " ".join(rviz["command"])
rviz_env = set(rviz["environment"])

assert "DT_CAMERA" in cmd and "DT_MOCK_EMG" in cmd
assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in env
assert "CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml" in env
assert "ROS_DOMAIN_ID=0" in env
assert "../config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro" in volumes
assert "rviz2 -d /prosthesis_ws/rviz/digital_twin.rviz" in rviz_cmd
assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in rviz_env
assert "CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml" in rviz_env
assert "devices" not in digital
assert "MIA_SERIAL_PORT" not in str(digital)
assert "mia_hand_driver_node" not in launch
assert "force_controller_node" not in launch
assert "grasp_proximity_controller_node.py" not in launch
assert 'executable="robot_state_publisher"' in launch
assert 'executable="digital_twin_joint_state_publisher"' in launch
assert 'executable="digital_twin_position_grasp_controller"' in launch
assert 'executable="mock_emg_publisher"' in launch
assert '"/joint_states"' in joint
assert '"/grasp_preshaping/target_finger_closures"' in position
assert '"publish_frequency"' in launch
assert 'DeclareLaunchArgument(\n            "mock_emg"' in launch
assert "ros2_control" not in urdf
assert "hardware" not in urdf.lower()
PY

echo "Digital twin hardware-bypass contracts OK"
