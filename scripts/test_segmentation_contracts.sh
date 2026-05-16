#!/bin/bash
# test_segmentation_contracts.sh — static checks for segmentation bridge wiring.

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
    "$WS_ROOT/src/segmentation/segmentation_bridge/segmentation_ros2_node.py" \
    "$WS_ROOT/src/segmentation/segmentation_bridge/demo_click_relay_node.py"

python3 - "$WS_ROOT" <<'PY'
import sys
from pathlib import Path

root = Path(sys.argv[1])
seg = (root / "src/segmentation/segmentation_bridge/segmentation_ros2_node.py").read_text()
launch = (root / "src/prosthesis_launch/launch/digital_twin.launch.py").read_text()
twist = (root / "src/twist_propagation/twist_propagation/twist_propagation_node.py").read_text()
click_relay = (root / "src/segmentation/segmentation_bridge/demo_click_relay_node.py").read_text()

assert 'PointCloud2, "/segmentation/input_cloud"' in seg
assert 'PointStamped, "/segmentation/click_positive"' in seg
assert 'PointStamped, "/segmentation/click_negative"' in seg
assert 'PointCloud2, "/segmentation/object_cloud"' in seg
assert 'requests.post(f"{url}/segment"' in seg
assert '"positive_clicks": pos_clicks' in seg
assert '"negative_clicks": neg_clicks' in seg
assert "copy.deepcopy(self._cloud_header)" in seg
assert "Inference mask length mismatch" in seg

assert '"output_topic": "/segmentation/input_cloud"' in launch
assert 'executable="segmentation_ros2_node"' in launch
assert 'executable="demo_click_relay_node"' in launch
assert '"/segmentation/click_positive"' in twist
assert '"/segmentation/object_cloud"' in twist
assert "self._pub.publish(msg)" in click_relay
PY

echo "Segmentation contracts OK"
