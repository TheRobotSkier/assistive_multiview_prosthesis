#!/bin/bash
# test_final_openvins_contracts.sh — static checks for final OpenVINS wiring.

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
    "$WS_ROOT/src/sensor_fusion_bringup/launch/dual_openvins_phase2.launch.py" \
    "$WS_ROOT/src/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py" \
    "$WS_ROOT/src/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py" \
    "$WS_ROOT/src/sensor_fusion_bringup/launch/head_marker_pose_phase2.launch.py" \
    "$WS_ROOT/src/sensor_fusion_bringup/launch/arm_marker_pose_phase2.launch.py" \
    "$WS_ROOT/src/sensor_fusion_bringup/scripts/marker_pose_odometry_bridge.py"

python3 - "$WS_ROOT" <<'PY'
import sys
from pathlib import Path
import yaml

root = Path(sys.argv[1])
head_map = yaml.safe_load((root / "src/sensor_fusion_bringup/config/markers/head_aruco_map.yaml").read_text())
arm_map = yaml.safe_load((root / "src/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml").read_text())

assert head_map["frames"]["map_frame"] == "marker_map"
assert head_map["frames"]["camera_frame"] == "head_d435i_head_color_optical_frame"
assert head_map["frames"]["imu_frame"] == "head_imu"
assert head_map["topics"]["image"] == "/head/d435i_head/color/image_raw"
assert head_map["topics"]["openvins_odom"] == "/ov_msckf_head/odomimu"
assert head_map["topics"]["output_prefix"] == "/head/marker_pose"
assert "head_d435i_336222071386" in head_map["calibration"]["kalibr_imucam_chain"]
assert {0, 1}.issubset({int(k) for k in head_map["markers"].keys()})

assert arm_map["frames"]["map_frame"] == "marker_map"
assert arm_map["frames"]["camera_frame"] == "arm_d435i_arm_color_optical_frame"
assert arm_map["frames"]["imu_frame"] == "arm_imu"
assert arm_map["topics"]["image"] == "/arm/d435i_arm/color/image_raw"
assert arm_map["topics"]["openvins_odom"] == "/ov_msckf_arm/odomimu"
assert arm_map["topics"]["output_prefix"] == "/arm/marker_pose"
assert "arm_d435i_310622071850" in arm_map["calibration"]["kalibr_imucam_chain"]
assert {0, 1}.issubset({int(k) for k in arm_map["markers"].keys()})

head_launch = (root / "src/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py").read_text()
arm_launch = (root / "src/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py").read_text()
dual_launch = (root / "src/sensor_fusion_bringup/launch/dual_openvins_phase2.launch.py").read_text()
make = (root / "Makefile").read_text()

for text, side, imu in ((head_launch, "head", "head_imu"), (arm_launch, "arm", "arm_imu")):
    assert '{"global_frame_id": "marker_map"}' in text
    assert f'{{"imu_frame_id": "{imu}"}}' in text
    assert '{"publish_global_to_imu_tf": True}' in text
    assert '{"use_marker_pose_updates": True}' in text
    assert f'{{"marker_pose_topic": "/{side}/marker_pose/observation"}}' in text
    assert '{"marker_fixed_ids": "0,1"}' in text

assert 'default_value="dual_d435i"' in dual_launch
assert 'default_value="true"' in dual_launch
assert "marker_pose_odometry_bridge.py" in dual_launch
assert "/ov_msckf_head/odomimu" in dual_launch
assert "/ov_msckf_arm/odomimu" in dual_launch
assert "check-final-openvins" in make
PY

echo "Final dual-D435i OpenVINS contracts OK"
