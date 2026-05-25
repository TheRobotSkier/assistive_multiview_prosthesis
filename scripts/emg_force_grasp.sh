#!/bin/bash
# EMG Force Grasp + Wrist Controller
# -----------------------------------
# Runs inside the prosthesis container.
# Flow: collect EMG data (if needed) → train classifier → launch force-hold + wrist controllers.
#
# Env overrides (set before `make emg-force-grasp`):
#   EMG_DATA_DIR      Directory with .npz training files   (default: /prosthesis_ws/data)
#   EMG_MODEL_DIR     Directory for trained models         (default: /prosthesis_ws/models)
#   MIA_PORT          Mia hand serial port                 (default: /dev/ttyUSB0)
#   WRIST_PORT        Wrist Dynamixel serial port          (default: /dev/ttyUSB1)
#   CONFIG_PATH       EMG grasp test YAML config           (default: …/emg_grasp_test.yaml)
#   WRIST_ENABLE      Enable wrist Dynamixel               (default: true)
#   FORCE_RETRAIN     Re-train even if model exists        (default: false)
#   LOG_LEVEL         ROS 2 log level                      (default: info)

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
EMG_DATA_DIR="${EMG_DATA_DIR:-/prosthesis_ws/data}"
EMG_MODEL_DIR="${EMG_MODEL_DIR:-/prosthesis_ws/models}"
MIA_PORT="${MIA_PORT:-/dev/ttyUSB0}"
WRIST_PORT="${WRIST_PORT:-/dev/ttyUSB1}"
CONFIG_PATH="${CONFIG_PATH:-/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml}"
WRIST_ENABLE="${WRIST_ENABLE:-true}"
FORCE_RETRAIN="${FORCE_RETRAIN:-false}"
MOCK_HARDWARE="${MOCK_HARDWARE:-false}"
LOG_LEVEL="${LOG_LEVEL:-info}"

# ── Source ROS 2 workspace ───────────────────────────────────────────────────
# ROS setup scripts may reference unbound vars — relax -u temporarily
set +u
source /opt/ros/jazzy/setup.bash
if [ -f /prosthesis_ws/install/setup.bash ]; then
    info "Rebuilding EMG force-grasp packages with latest sources …"
    colcon build --packages-up-to mia_hand_ros2_control emg_bridge wrist_driver prosthesis_launch \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    source /prosthesis_ws/install/setup.bash
else
    info "Workspace not built. Building required packages …"
    colcon build --packages-up-to mia_hand_ros2_control emg_bridge wrist_driver prosthesis_launch \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    source /prosthesis_ws/install/setup.bash
fi
set -u

# ── Banner ───────────────────────────────────────────────────────────────────
echo ""
banner "=========================================================="
banner "   EMG Force Grasp + Wrist Controller"
banner "=========================================================="
echo ""
info "Configuration:"
echo "  EMG_DATA_DIR   = $EMG_DATA_DIR"
echo "  EMG_MODEL_DIR  = $EMG_MODEL_DIR"
echo "  MIA_PORT       = $MIA_PORT"
echo "  WRIST_PORT     = $WRIST_PORT"
echo "  WRIST_ENABLE   = $WRIST_ENABLE"
echo "  CONFIG_PATH    = $CONFIG_PATH"
echo "  FORCE_RETRAIN  = $FORCE_RETRAIN"
echo "  LOG_LEVEL      = $LOG_LEVEL"
echo ""

# ── Config file check ────────────────────────────────────────────────────────
if [ ! -f "$CONFIG_PATH" ]; then
    err "ERROR: Config file not found: $CONFIG_PATH"
    exit 1
fi

# ══════════════════════════════════════════════════════════════════════════════
# MOCK MODE
# ══════════════════════════════════════════════════════════════════════════════
if [ "$MOCK_HARDWARE" = "true" ]; then
    banner "── MOCK MODE ───────────────────────────────────────────────"
    warn "Skipping training. Using mock EMG and mock Mia hand."
    echo ""
    ros2 launch prosthesis_launch emg_grasp_test.launch.py \
        emg_model_dir:="$EMG_MODEL_DIR" \
        mock_hardware:=true \
        wrist_enable:="$WRIST_ENABLE" \
        config_path:="$CONFIG_PATH" \
        log_level:="$LOG_LEVEL"
    exit $?
fi

# ══════════════════════════════════════════════════════════════════════════════
# STEP 0: Collect EMG Data (if needed)
# ══════════════════════════════════════════════════════════════════════════════
mkdir -p "$EMG_DATA_DIR" "$EMG_MODEL_DIR"

MODEL_FILE="$EMG_MODEL_DIR/classifier.pkl"
NPZ_COUNT=$(ls -1 "$EMG_DATA_DIR"/*.npz 2>/dev/null | wc -l || true)

if [ "$NPZ_COUNT" -eq 0 ]; then
    banner "── Step 0/3: EMG Data Collection ────────────────────────────"
    warn "No .npz training files found in $EMG_DATA_DIR"
    warn "INTERACTIVE — follow the prompts below."
    warn "You will be asked to perform each gesture for recording."
    echo ""
    info "Gestures: REST (relaxed), POWER (grip), PINCH, OPEN (extend), POINT"
    info "Each gesture: hold steady when prompted, relax between reps."
    echo ""

    if [ -n "${EMG_DEVICE:-}" ]; then
        info "Connecting to MindRove board at $EMG_DEVICE …"
        export MINDROVE_IP="$EMG_DEVICE"
    fi

    set +e
    ros2 run emg_bridge collect_data --output-dir "$EMG_DATA_DIR"
    COLLECT_EXIT=$?
    set -e

    if [ $COLLECT_EXIT -ne 0 ]; then
        err "ERROR: Data collection exited with code $COLLECT_EXIT"
        exit 1
    fi

    NPZ_COUNT=$(ls -1 "$EMG_DATA_DIR"/*.npz 2>/dev/null | wc -l)
    if [ "$NPZ_COUNT" -eq 0 ]; then
        err "ERROR: No .npz files found in $EMG_DATA_DIR after collection"
        exit 1
    fi
    ok "✓ Collected $NPZ_COUNT session file(s) in $EMG_DATA_DIR"
else
    ok "✓ Found $NPZ_COUNT .npz session file(s) in $EMG_DATA_DIR"
fi
echo ""

# ══════════════════════════════════════════════════════════════════════════════
# STEP 1: Train EMG Classifier
# ══════════════════════════════════════════════════════════════════════════════
NEED_TRAIN=false

if [ "$FORCE_RETRAIN" = "true" ]; then
    NEED_TRAIN=true
    banner "── Step 1/3: Training EMG Classifier (forced) ───────────────"
elif [ -f "$MODEL_FILE" ]; then
    ok "✓ Model found at $MODEL_FILE — skipping training."
    info "  Set FORCE_RETRAIN=true to retrain."
else
    NEED_TRAIN=true
    banner "── Step 1/3: Training EMG Classifier ────────────────────────"
    warn "No model found at $MODEL_FILE"
fi

if [ "$NEED_TRAIN" = "true" ]; then
    info "Training on $NPZ_COUNT session file(s) …"
    echo ""

    set +e
    ros2 run emg_bridge train --data-dir "$EMG_DATA_DIR" --model-dir "$EMG_MODEL_DIR"
    TRAIN_EXIT=$?
    set -e

    if [ $TRAIN_EXIT -ne 0 ]; then
        err "ERROR: Training exited with code $TRAIN_EXIT"
        exit 1
    fi

    if [ ! -f "$MODEL_FILE" ]; then
        err "ERROR: Training completed but no model file found at $MODEL_FILE"
        exit 1
    fi
    ok "✓ Model saved to $MODEL_FILE"
else
    banner "── Step 1/3: EMG Classifier (cached) ───────────────────────"
fi
echo ""

# ══════════════════════════════════════════════════════════════════════════════
# STEP 2: Launch EMG classifier + Mia hand + force grasp + wrist controllers
# ══════════════════════════════════════════════════════════════════════════════
banner "── Step 2/3: Launching Controllers ──────────────────────────"
echo ""
info "Starting:"
info "  1. Mia Hand ros2_control  (position + velocity controllers)"
info "  2. EMG classifier bridge  (run_classifier --model-dir $EMG_MODEL_DIR)"
info "  3. EMG force-grasp bridge (hand force hold + wrist control)"
if [ "$WRIST_ENABLE" = "true" ]; then
    info "  4. Wrist Dynamixel driver ($WRIST_PORT)"
fi
info "  5. Safety preflight + status publisher"
echo ""
AUTO_KILL_S="${AUTO_KILL_S:-60}"
warn "Auto-stop in ${AUTO_KILL_S}s — press ENTER to stop earlier."

# ENTER-to-stop listener
STOP_SIGNAL=0
{ read -r _; STOP_SIGNAL=1; kill "$$" 2>/dev/null; } &
READER_PID=$!

cleanup() {
    echo ""
    warn "Shutting down …"
    kill "$READER_PID" 2>/dev/null || true
    # kill any remaining ros2/controller_manager processes
    pkill -P $$ 2>/dev/null || true
    echo "EMG force-grasp launch stopped."
    exit 0
}
trap cleanup INT TERM

timeout --foreground --kill-after=3s "${AUTO_KILL_S}s" ros2 launch prosthesis_launch emg_grasp_test.launch.py \
    emg_model_dir:="$EMG_MODEL_DIR" \
    mia_port:="$MIA_PORT" \
    wrist_port:="$WRIST_PORT" \
    wrist_enable:="$WRIST_ENABLE" \
    mock_hardware:=false \
    config_path:="$CONFIG_PATH" \
    log_level:="$LOG_LEVEL" || true

cleanup
