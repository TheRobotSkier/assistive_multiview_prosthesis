#!/bin/bash
# test_mixed_camera_contracts.sh — static checks for temporary D435 + external IMU mode.

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
    "$WS_ROOT/src/sensor_fusion_bringup/scripts/i2c_mpu9250_imu_node.py" \
    "$WS_ROOT/src/sensor_fusion_bringup/launch/dual_d435i.launch.py" \
    "$WS_ROOT/src/sensor_fusion_bringup/launch/dual_openvins_phase2.launch.py" \
    "$WS_ROOT/src/sensor_fusion_bringup/launch/head_d435_openvins_phase2.launch.py" \
    "$WS_ROOT/src/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py"

python3 - "$WS_ROOT" <<'PY'
import sys
from pathlib import Path
import yaml

root = Path(sys.argv[1])
mixed = yaml.safe_load((root / "src/sensor_fusion_bringup/config/mixed_d435_d435i_cameras.yaml").read_text())
head = mixed["cameras"]["head"]
arm = mixed["cameras"]["arm"]
assert head["has_builtin_imu"] is False
assert head["external_imu"]["enabled"] is True
assert head["external_imu"]["topic"] == "/head/d435i_head/imu"
assert int(head["external_imu"]["i2c_bus"]) == 7
assert arm["has_builtin_imu"] is True

imu_chain = (root / "src/sensor_fusion_bringup/config/openvins/head_d435_829212072207/kalibr_imu_chain.yaml").read_text()
assert "rostopic: /head/d435i_head/imu" in imu_chain
assert "accelerometer_noise_density: 0.08" in imu_chain
assert "gyroscope_noise_density: 0.01" in imu_chain

jetson_make_path = root / "jetson/Makefile"
if jetson_make_path.exists():
    jetson_make = jetson_make_path.read_text()
    assert "CAMERA_CONFIG ?= d435i_cameras.yaml" in jetson_make
    assert "RIG_MODE      ?= dual_d435i" in jetson_make
    assert "$(MAKE) cameras CAMERA_CONFIG=mixed_d435_d435i_cameras.yaml" in jetson_make
    assert "$(MAKE) openvins RIG_MODE=mixed_d435_d435i" in jetson_make
    assert "cameras-final" in jetson_make
    assert "openvins-final" in jetson_make
PY

echo "Mixed camera contracts OK"
