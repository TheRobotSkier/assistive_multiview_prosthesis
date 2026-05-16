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
launch = (root / "src/prosthesis_launch/launch/digital_twin.launch.py").read_text()
joint = (root / "src/pipeline_manager/pipeline_manager/digital_twin_joint_state_publisher.py").read_text()
urdf = (root / "src/mia_hand_ros2_control/description/urdf/mia_hand_digital_twin.urdf.xacro").read_text()

cmd = " ".join(digital["command"])
env = set(digital["environment"])
volumes = set(digital["volumes"])

assert "digital_twin.launch.py camera:=true gui:=false" in cmd
assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in env
assert "CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml" in env
assert "ROS_DOMAIN_ID=0" in env
assert "../config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro" in volumes
assert "devices" not in digital
assert "MIA_SERIAL_PORT" not in str(digital)
assert "mia_hand_driver_node" not in launch
assert "force_controller_node" not in launch
assert 'executable="robot_state_publisher"' in launch
assert 'executable="digital_twin_joint_state_publisher"' in launch
assert '"/joint_states"' in joint
assert "ros2_control" not in urdf
assert "hardware" not in urdf.lower()
PY

echo "Digital twin hardware-bypass contracts OK"
