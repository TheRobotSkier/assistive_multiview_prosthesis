#!/bin/bash
# EMG Mia Hand haptic force test wrapper.
#
# Runs inside the dedicated test container. It builds the ROS packages needed
# for the no-camera test graph, prepares or reuses an EMG classifier, then
# launches the staged haptic force test.

set -euo pipefail

_bold="\033[1m"
_green="\033[92m"
_yellow="\033[93m"
_cyan="\033[96m"
_red="\033[91m"
_reset="\033[0m"

banner() { echo -e "${_bold}$*${_reset}"; }
info() { echo -e "${_cyan}$*${_reset}"; }
ok() { echo -e "${_green}$*${_reset}"; }
warn() { echo -e "${_yellow}$*${_reset}"; }
err() { echo -e "${_red}$*${_reset}"; }

EMG_DATA_DIR="${EMG_DATA_DIR:-/app/data}"
EMG_MODEL_DIR="${EMG_MODEL_DIR:-/app/models}"
MIA_PORT="${MIA_PORT:-${MIA_SERIAL_PORT:-/dev/ttyUSB0}}"
WRIST_PORT="${WRIST_PORT:-${WRIST_SERIAL_PORT:-/dev/ttyUSB1}}"
CONFIG_PATH="${CONFIG_PATH:-/prosthesis_ws/config/mia_haptic_force_test.yaml}"
WRIST_ENABLE="${WRIST_ENABLE:-true}"
HAPTIC_ENABLE="${HAPTIC_ENABLE:-true}"
EMG_ENABLE="${EMG_ENABLE:-true}"
FORCE_RETRAIN="${FORCE_RETRAIN:-false}"
MOCK_HARDWARE="${MOCK_HARDWARE:-false}"
LOG_LEVEL="${LOG_LEVEL:-info}"
AUTO_KILL_S="${AUTO_KILL_S:-0}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

banner "=========================================================="
banner "   Mia Hand EMG Haptic Force Test"
banner "=========================================================="
info "Configuration:"
echo "  EMG_DATA_DIR   = $EMG_DATA_DIR"
echo "  EMG_MODEL_DIR  = $EMG_MODEL_DIR"
echo "  MIA_PORT       = $MIA_PORT"
echo "  WRIST_PORT     = $WRIST_PORT"
echo "  WRIST_ENABLE   = $WRIST_ENABLE"
echo "  HAPTIC_ENABLE  = $HAPTIC_ENABLE"
echo "  EMG_ENABLE     = $EMG_ENABLE"
echo "  CONFIG_PATH    = $CONFIG_PATH"
echo "  FORCE_RETRAIN  = $FORCE_RETRAIN"
echo "  MOCK_HARDWARE  = $MOCK_HARDWARE"
echo "  LOG_LEVEL      = $LOG_LEVEL"
echo ""

if [ ! -f "$CONFIG_PATH" ]; then
    err "ERROR: Config file not found: $CONFIG_PATH"
    exit 1
fi

info "Building required ROS packages..."
cd /prosthesis_ws
colcon build --packages-up-to \
    mia_hand_ros2_control \
    emg_bridge \
    wrist_driver \
    haptic_bridge \
    prosthesis_launch \
    force_controller \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

set +u
source /prosthesis_ws/install/setup.bash
set -u

mkdir -p "$EMG_DATA_DIR" "$EMG_MODEL_DIR"

if [ "$EMG_ENABLE" = "true" ]; then
    MODEL_FILE="$EMG_MODEL_DIR/classifier.pkl"
    NPZ_COUNT=$(ls -1 "$EMG_DATA_DIR"/*.npz 2>/dev/null | wc -l || true)

    if [ "$NPZ_COUNT" -eq 0 ]; then
        banner "Step 1/3: EMG data collection"
        warn "No .npz training files found in $EMG_DATA_DIR"
        warn "Interactive recording will start now."
        ros2 run emg_bridge collect_data --output-dir "$EMG_DATA_DIR"
        NPZ_COUNT=$(ls -1 "$EMG_DATA_DIR"/*.npz 2>/dev/null | wc -l || true)
        if [ "$NPZ_COUNT" -eq 0 ]; then
            err "ERROR: No EMG recordings were produced."
            exit 1
        fi
    else
        ok "Found $NPZ_COUNT EMG recording file(s)."
    fi

    if [ "$FORCE_RETRAIN" = "true" ] || [ ! -f "$MODEL_FILE" ]; then
        banner "Step 2/3: Training EMG classifier"
        ros2 run emg_bridge train --data-dir "$EMG_DATA_DIR" --model-dir "$EMG_MODEL_DIR"
        if [ ! -f "$MODEL_FILE" ]; then
            err "ERROR: Training completed but $MODEL_FILE was not created."
            exit 1
        fi
    else
        ok "Using cached EMG model at $MODEL_FILE"
    fi
else
    warn "Skipping EMG data/model preparation because EMG_ENABLE=$EMG_ENABLE."
fi

banner "Step 3/3: Launching test graph"
warn "Hold OPEN for the configured duration to stop the test and return horizontal."
if [ "$AUTO_KILL_S" = "0" ]; then
    warn "No auto-stop timeout is active. Press Ctrl-C to abort."
else
    warn "Auto-stop timeout: ${AUTO_KILL_S}s"
fi

cleanup() {
    echo ""
    warn "Shutting down Mia haptic force test..."
    pkill -P $$ 2>/dev/null || true
}
trap cleanup INT TERM EXIT

LAUNCH_CMD=(
    ros2 launch prosthesis_launch mia_haptic_force_test.launch.py
    config_path:="$CONFIG_PATH"
    emg_model_dir:="$EMG_MODEL_DIR"
    mia_port:="$MIA_PORT"
    wrist_port:="$WRIST_PORT"
    wrist_enable:="$WRIST_ENABLE"
    haptic_enable:="$HAPTIC_ENABLE"
    emg_enable:="$EMG_ENABLE"
    mock_hardware:="$MOCK_HARDWARE"
    log_level:="$LOG_LEVEL"
)

if [ "$AUTO_KILL_S" = "0" ]; then
    "${LAUNCH_CMD[@]}"
else
    timeout --foreground --kill-after=5s "${AUTO_KILL_S}s" "${LAUNCH_CMD[@]}"
fi
