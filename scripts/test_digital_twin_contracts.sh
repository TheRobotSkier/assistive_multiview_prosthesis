#!/bin/bash
# test_digital_twin_contracts.sh — focused checks for the host-side RViz hand pipeline.

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
    "$WS_ROOT/src/prosthesis_launch/launch/digital_twin.launch.py" \
    "$WS_ROOT/src/camera/camera/pointcloud_relay_node.py"

python3 - "$WS_ROOT" <<'PY'
import sys
from pathlib import Path
import yaml

root = Path(sys.argv[1])
for rel in (
    "src/sensor_fusion_bringup/config/markers/head_aruco_map.yaml",
    "src/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml",
):
    data = yaml.safe_load((root / rel).read_text())
    markers = {int(k) for k in data["markers"].keys()}
    assert {0, 1}.issubset(markers), f"{rel} must include ArUco marker IDs 0 and 1"

bridge = (root / "src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp").read_text()
assert "create_publisher<geometry_msgs::msg::PoseStamped>" in bridge
assert "target_pose.header.frame_id = \"world\"" in bridge
PY

ros2 run pipeline_manager digital_twin_joint_state_publisher >/tmp/digital_twin_joint_state_publisher.log 2>&1 &
NODE_PID=$!
cleanup() {
    kill "$NODE_PID" 2>/dev/null || true
    wait "$NODE_PID" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 10); do
    if ros2 node list 2>/dev/null | grep -q "/digital_twin_joint_state_publisher"; then
        break
    fi
    sleep 0.5
done

ros2 node list | grep -q "/digital_twin_joint_state_publisher"

ros2 topic pub --once /thumb_pos_ff_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5]}" >/tmp/digital_twin_thumb_pub.log 2>&1
ros2 topic pub --once /index_pos_ff_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.6]}" >/tmp/digital_twin_index_pub.log 2>&1
ros2 topic pub --once /mrl_pos_ff_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.7]}" >/tmp/digital_twin_mrl_pub.log 2>&1

timeout 8 ros2 topic echo --once /joint_states >/tmp/digital_twin_joint_states.log
grep -q "j_thumb_fle" /tmp/digital_twin_joint_states.log
grep -q "j_index_fle" /tmp/digital_twin_joint_states.log
grep -q "j_mrl_fle" /tmp/digital_twin_joint_states.log
grep -q "j_thumb_opp" /tmp/digital_twin_joint_states.log

echo "Digital twin contracts OK"
