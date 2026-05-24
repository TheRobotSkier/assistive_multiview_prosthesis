#!/usr/bin/env bash
# Entrypoint for EMG-driven force-aware grasp test inside the prosthesis container.
#
# Builds the workspace (if needed) and launches the force-aware EMG grasp pipeline.

set -euo pipefail

WORKSPACE=/prosthesis_ws
MODEL_DIR="${MODEL_DIR:-/prosthesis_ws/models}"
CONFIG_PATH="${CONFIG_PATH:-/prosthesis_ws/tests/emg_grasp/emg_grasp_test_force.yaml}"
EMG_CONFIG="${EMG_CONFIG:-}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
WRIST_SERIAL_PORT="${WRIST_SERIAL_PORT:-/dev/ttyUSB1}"

echo ""
echo "[force-grasp] preparing workspace..."

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$WORKSPACE"

if [ ! -f install/setup.bash ]; then
    echo "[force-grasp] first run: full workspace build"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    echo "[force-grasp] incremental build: force_controller + prosthesis_launch + emg_bridge"
    colcon build --packages-select force_controller prosthesis_launch emg_bridge --symlink-install
    echo "[force-grasp] incremental build: mia_hand_ros2_control + wrist_driver (copy)"
    colcon build --packages-up-to mia_hand_ros2_control wrist_driver \
        --packages-skip force_controller prosthesis_launch emg_bridge \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

set +u
source install/setup.bash
set -u

EMG_FLAG="emg:=true"
if [ ! -e "$SERIAL_PORT" ]; then
    echo "[force-grasp] WARNING: $SERIAL_PORT not found — starting with mock EMG"
    EMG_FLAG="emg:=false mock_emg:=true"
fi

EMG_CONFIG_FLAG=""
if [ -n "$EMG_CONFIG" ]; then
    EMG_CONFIG_FLAG="emg_config:=$EMG_CONFIG"
fi

echo "[force-grasp] launching emg_force_grasp..."
ros2 launch prosthesis_launch emg_force_grasp.launch.py \
    $EMG_FLAG \
    $EMG_CONFIG_FLAG \
    "serial_port:=$SERIAL_PORT" \
    "wrist_serial_port:=$WRIST_SERIAL_PORT" \
    "config_path:=$CONFIG_PATH" \
    "model_dir:=$MODEL_DIR"
