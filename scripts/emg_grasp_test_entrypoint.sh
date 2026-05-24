#!/usr/bin/env bash

set -euo pipefail

WORKSPACE=/prosthesis_ws

echo ""
echo "[grasp-test] preparing workspace..."

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$WORKSPACE"

if [ ! -f install/setup.bash ]; then
    echo "[grasp-test] first run: full workspace build"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    echo "[grasp-test] build: emg_bridge (symlink)"
    colcon build --packages-select emg_bridge prosthesis_launch --symlink-install
    echo "[grasp-test] build: mia_hand_ros2_control dependencies (copy)"
    colcon build --packages-up-to mia_hand_ros2_control wrist_driver \
        --packages-skip emg_bridge prosthesis_launch \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

set +u
source install/setup.bash
set -u

python3 /prosthesis_ws/scripts/emg_grasp_test_runtime.py \
    --config-path /prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml \
    --model-dir /prosthesis_ws/models \
    --serial-port /dev/ttyUSB0 \
    --wrist-serial-port /dev/ttyUSB1 \
    --cyclonedds-uri /tmp/cyclonedds_peer.xml
