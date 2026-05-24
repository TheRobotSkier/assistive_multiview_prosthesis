#!/bin/bash
# EMG-Driven Grasp Test Orchestration
# ----------------------------------------
# Runs inside the prosthesis container.
# Flow: collect EMG data → train classifier → launch grasp system.
#
# Env overrides (set before `make emg-grasp-test`):
#   EMG_DEVICE        MindRove board IP/host (default: auto-discover)
#   EMG_DATA_DIR      Directory for recorded .npz files  (default: /app/data)
#   EMG_MODEL_DIR     Directory for trained models       (default: /app/models)
#   MIA_PORT          Mia hand serial port               (default: /dev/ttyUSB0)
#   CONFIG_PATH       EMG grasp test YAML config         (default: …/emg_grasp_test.yaml)
#   WRIST_ENABLE      Enable wrist Dynamixel             (default: false)
#   MOCK_HARDWARE     Skip collect/train, use mock       (default: false)

set -euo pipefail

# ── ANSI helpers ──────────────────────────────────────────────────────────────
_bold="\033[1m"
_green="\033[92m"
_yellow="\033[93m"
_cyan="\033[96m"
_red="\033[91m"
_reset="\033[0m"

banner()  { echo -e "${_bold}$*${_reset}"; }
info()    { echo -e "${_cyan}$*${_reset}"; }
ok()      { echo -e "${_green}$*${_reset}"; }
warn()    { echo -e "${_yellow}$*${_reset}"; }
err()     { echo -e "${_red}$*${_reset}"; }

# ── Env vars with defaults ───────────────────────────────────────────────────
EMG_DEVICE="${EMG_DEVICE:-}"
EMG_DATA_DIR="${EMG_DATA_DIR:-/app/data}"
EMG_MODEL_DIR="${EMG_MODEL_DIR:-/app/models}"
MIA_PORT="${MIA_PORT:-/dev/ttyUSB0}"
CONFIG_PATH="${CONFIG_PATH:-/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml}"
WRIST_ENABLE="${WRIST_ENABLE:-false}"
MOCK_HARDWARE="${MOCK_HARDWARE:-false}"

# ── Source ROS 2 workspace ───────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
else
    err "ERROR: Workspace not built. Run 'make build' inside the container first."
    exit 1
fi

# ── Banner ───────────────────────────────────────────────────────────────────
echo ""
banner "=========================================================="
banner "    EMG-Driven Velocity-Based Force Grasp Test"
banner "=========================================================="
echo ""
info "Configuration:"
echo "  EMG_DEVICE     = ${EMG_DEVICE:-<auto-discover>}"
echo "  EMG_DATA_DIR   = $EMG_DATA_DIR"
echo "  EMG_MODEL_DIR  = $EMG_MODEL_DIR"
echo "  MIA_PORT       = $MIA_PORT"
echo "  CONFIG_PATH    = $CONFIG_PATH"
echo "  WRIST_ENABLE   = $WRIST_ENABLE"
echo "  MOCK_HARDWARE  = $MOCK_HARDWARE"
echo ""

# ── Config file check ────────────────────────────────────────────────────────
if [ ! -f "$CONFIG_PATH" ]; then
    err "ERROR: Config file not found: $CONFIG_PATH"
    exit 1
fi

# ══════════════════════════════════════════════════════════════════════════════
# MOCK MODE — skip collect/train, launch with mock hardware
# ══════════════════════════════════════════════════════════════════════════════
if [ "$MOCK_HARDWARE" = "true" ]; then
    banner "── MOCK MODE ───────────────────────────────────────────────"
    warn "Skipping EMG data collection and training."
    warn "Using mock EMG publisher and mock Mia hardware."
    echo ""

    if [ "$MIA_PORT" != "/dev/ttyUSB0" ] && [ "$MIA_PORT" != "mock" ]; then
        warn "MIA_PORT is set but MOCK_HARDWARE=true — device ignored."
    fi

    info "Launching EMG grasp test with mock hardware …"
    echo ""
    ros2 launch prosthesis_launch emg_grasp_test.launch.py \
        mock_emg:=true \
        use_mock_hardware:=true \
        serial_port:="mock" \
        config_path:="$CONFIG_PATH"
    exit $?
fi

# ══════════════════════════════════════════════════════════════════════════════
# HARDWARE MODE — full collect → train → launch pipeline
# ══════════════════════════════════════════════════════════════════════════════

# ── Check Mia device ─────────────────────────────────────────────────────────
if [ ! -e "$MIA_PORT" ]; then
    err "ERROR: Mia hand device not found at $MIA_PORT"
    err "  - Is the USB cable connected?"
    err "  - Try: MIA_PORT=/dev/ttyUSB1 make emg-grasp-test"
    err "  - Or: MOCK_HARDWARE=true make emg-grasp-test"
    exit 1
fi

# ── Step 1: EMG Data Collection ──────────────────────────────────────────────
banner "── Step 1/3: EMG Data Collection ───────────────────────────"
warn "INTERACTIVE — follow the prompts below."
warn "You will be asked to perform each gesture for recording."
echo ""
info "Gestures: REST (relaxed), POWER (grip), PINCH, OPEN (extend), POINT"
info "Each gesture: hold steady when prompted, relax between reps."
echo ""

mkdir -p "$EMG_DATA_DIR"

if [ -n "${EMG_DEVICE:-}" ]; then
    info "Connecting to MindRove board at $EMG_DEVICE …"
    export MINDROVE_IP="$EMG_DEVICE"
fi

# Run collect_data — this is interactive
set +e
ros2 run emg_bridge collect_data --output-dir "$EMG_DATA_DIR"
COLLECT_EXIT=$?
set -e

if [ $COLLECT_EXIT -ne 0 ]; then
    err "ERROR: Data collection exited with code $COLLECT_EXIT"
    exit 1
fi

# Verify data was collected
NPZ_COUNT=$(ls -1 "$EMG_DATA_DIR"/*.npz 2>/dev/null | wc -l)
if [ "$NPZ_COUNT" -eq 0 ]; then
    err "ERROR: No .npz files found in $EMG_DATA_DIR"
    err "  Data collection produced no output. Aborting."
    exit 1
fi
ok "✓ Collected $NPZ_COUNT session file(s) in $EMG_DATA_DIR"
echo ""

# ── Step 2: Train EMG Classifier ─────────────────────────────────────────────
banner "── Step 2/3: Training EMG Classifier ────────────────────────"
echo ""

mkdir -p "$EMG_MODEL_DIR"

set +e
ros2 run emg_bridge train --data-dir "$EMG_DATA_DIR" --model-dir "$EMG_MODEL_DIR"
TRAIN_EXIT=$?
set -e

if [ $TRAIN_EXIT -ne 0 ]; then
    err "ERROR: Training exited with code $TRAIN_EXIT"
    exit 1
fi

# Verify model was produced
if [ ! -f "$EMG_MODEL_DIR/classifier.pkl" ]; then
    err "ERROR: Training completed but no model file found at $EMG_MODEL_DIR/classifier.pkl"
    err "  Check training output above for details."
    exit 1
fi
ok "✓ Model saved to $EMG_MODEL_DIR/classifier.pkl"
echo ""

# ── Step 3: Launch Grasp System ──────────────────────────────────────────────
banner "── Step 3/3: Launching EMG Grasp System ─────────────────────"
echo ""
info "Starting:"
info "  1. EMG classifier bridge (run_classifier)"
info "  2. Mia Hand ros2_control"
info "  3. EMG grasp test node"
echo ""
warn "Press Ctrl-C to stop and return hand to safe position."

# Start the EMG classifier in the background
ros2 run emg_bridge run_classifier \
    --model-dir "$EMG_MODEL_DIR" &
CLASSIFIER_PID=$!

# Cleanup handler
cleanup() {
    echo ""
    warn "Shutting down …"
    kill "$CLASSIFIER_PID" 2>/dev/null || true
    wait "$CLASSIFIER_PID" 2>/dev/null || true
    echo "EMG classifier stopped."
    exit 0
}
trap cleanup INT TERM

# Brief pause to let the classifier connect to MindRove board
sleep 2

# Launch the grasp test (foreground)
ros2 launch prosthesis_launch emg_grasp_test.launch.py \
    emg:=false \
    mock_emg:=false \
    use_mock_hardware:=false \
    serial_port:="$MIA_PORT" \
    config_path:="$CONFIG_PATH"

# If launch exits naturally, clean up
cleanup
