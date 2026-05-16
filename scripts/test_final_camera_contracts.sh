#!/bin/bash
# test_final_camera_contracts.sh — static checks for final dual-D435i mode.

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
    "$WS_ROOT/src/sensor_fusion_bringup/launch/dual_d435i.launch.py" \
    "$WS_ROOT/src/sensor_fusion_bringup/launch/dual_openvins_phase2.launch.py" \
    "$WS_ROOT/src/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py" \
    "$WS_ROOT/src/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py"

python3 - "$WS_ROOT" <<'PY'
import sys
from pathlib import Path
import yaml

root = Path(sys.argv[1])
cfg = yaml.safe_load((root / "src/sensor_fusion_bringup/config/d435i_cameras.yaml").read_text())
head = cfg["cameras"]["head"]
arm = cfg["cameras"]["arm"]
common = cfg["common"]

assert head["namespace"] == "head"
assert head["name"] == "d435i_head"
assert head["serial_no"].startswith("_")
assert arm["namespace"] == "arm"
assert arm["name"] == "d435i_arm"
assert arm["serial_no"].startswith("_")
assert common["enable_gyro"] is True
assert common["enable_accel"] is True
assert int(common["unite_imu_method"]) == 2

head_imu = (root / "src/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/kalibr_imu_chain.yaml").read_text()
head_cam = (root / "src/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/kalibr_imucam_chain.yaml").read_text()
arm_imu = (root / "src/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/kalibr_imu_chain.yaml").read_text()
arm_cam = (root / "src/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/kalibr_imucam_chain.yaml").read_text()
assert "rostopic: /head/d435i_head/imu" in head_imu
assert "rostopic: /head/d435i_head/color/image_raw" in head_cam
assert "rostopic: /arm/d435i_arm/imu" in arm_imu
assert "rostopic: /arm/d435i_arm/color/image_raw" in arm_cam

dual = (root / "src/sensor_fusion_bringup/launch/dual_openvins_phase2.launch.py").read_text()
assert 'default_value="dual_d435i"' in dual
assert "head_d435i_openvins_phase2.launch.py" in dual
assert "arm_d435i_openvins_phase2.launch.py" in dual
assert '"marker_fixed_ids": "0,1"' in (root / "src/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py").read_text()
assert '"marker_fixed_ids": "0,1"' in (root / "src/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py").read_text()

make = (root / "Makefile").read_text()
jetson_make = (root / "jetson/Makefile").read_text()
assert "check-final-topics" in make
assert "make cameras-final" in make
assert "make openvins-final" in make
assert "CAMERA_CONFIG ?= d435i_cameras.yaml" in jetson_make
assert "RIG_MODE      ?= dual_d435i" in jetson_make
PY

echo "Final dual-D435i camera contracts OK"
