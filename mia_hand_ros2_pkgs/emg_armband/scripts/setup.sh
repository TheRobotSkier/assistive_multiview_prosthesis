#!/usr/bin/env bash
# EMG Guided Setup
#
# Walks the user through the full pipeline in a single terminal session:
#   1. Data collection  – record labelled EMG gestures with the MindRove armband
#   2. Training         – fit the LDA/SVM classifier on the collected data
#   3. Launch ROS node  – start emg_node + emg_parser_node in the background
#   4. Live monitor     – display all configured ROS topics in the terminal
#
# Run this script via:
#   docker compose run --rm emg_setup
# or directly inside any emg_armband container:
#   bash /emg_ws/install/emg_armband/lib/emg_armband/scripts/setup.sh

INSTALL_SCRIPTS="/emg_ws/install/emg_armband/lib/emg_armband/scripts"
DATA_DIR="/emg_ws/src/emg_armband/data"
MODELS_DIR="/emg_ws/src/emg_armband/models"
CONFIG_FILE="/emg_ws/src/emg_armband/config/emg.yaml"

ROS_LAUNCH_PID=""

# ── cleanup trap ──────────────────────────────────────────────────────────────
_cleanup() {
    echo ""
    if [ -n "$ROS_LAUNCH_PID" ]; then
        echo "  Stopping ROS node (pid $ROS_LAUNCH_PID) …"
        kill "$ROS_LAUNCH_PID" 2>/dev/null || true
        wait "$ROS_LAUNCH_PID" 2>/dev/null || true
    fi
    echo "  Bye."
}
trap _cleanup EXIT INT TERM

# ── source workspace ──────────────────────────────────────────────────────────
# shellcheck disable=SC1091
source /emg_ws/install/setup.bash

# ── header ────────────────────────────────────────────────────────────────────
clear
echo ""
echo "  ╔══════════════════════════════════════════════════════╗"
echo "  ║         MindRove EMG — Guided Setup                  ║"
echo "  ╚══════════════════════════════════════════════════════╝"
echo ""
echo "  This session will guide you through four steps:"
echo ""
echo "    1/4  Data collection  — hold each gesture while recording"
echo "    2/4  Training         — fit classifier on your recordings"
echo "    3/4  Launch ROS node  — connect the armband to the ROS network"
echo "    4/4  Live monitor     — watch gesture + topic values in real time"
echo ""
echo "  Make sure the MindRove armband is powered on and within WiFi range."
echo ""
read -rp "  Press ENTER to begin, or Ctrl-C to abort … "

# ─────────────────────────────────────────────────────────────────────────────
# STEP 1 — Data collection
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "  ══════════════════════════════════════════════════════"
echo "  Step 1/4 — Data Collection"
echo "  ══════════════════════════════════════════════════════"
echo ""
echo "  You will be guided through recording each gesture."
echo "  Default: 3 repetitions × 5 seconds each."
echo "  Press ENTER before each gesture when you are ready."
echo ""

python3 "$INSTALL_SCRIPTS/collect_data.py" --output-dir "$DATA_DIR"

# ─────────────────────────────────────────────────────────────────────────────
# STEP 2 — Training
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "  ══════════════════════════════════════════════════════"
echo "  Step 2/4 — Training"
echo "  ══════════════════════════════════════════════════════"
echo ""

python3 "$INSTALL_SCRIPTS/train.py" \
    --data-dir  "$DATA_DIR" \
    --model-dir "$MODELS_DIR"

# ─────────────────────────────────────────────────────────────────────────────
# STEP 3 — Launch ROS2 node
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "  ══════════════════════════════════════════════════════"
echo "  Step 3/4 — Launching ROS2 EMG Node"
echo "  ══════════════════════════════════════════════════════"
echo ""

ros2 launch emg_armband emg_launch.py &
ROS_LAUNCH_PID=$!

# Wait until /emg/data topic appears (board connection confirmed) or time out.
echo "  Waiting for /emg/data to become available …"
TIMEOUT=60
ELAPSED=0
while ! ros2 topic list 2>/dev/null | grep -qF "/emg/data"; do
    sleep 1
    ELAPSED=$((ELAPSED + 1))
    printf "\r  %ds / %ds" "$ELAPSED" "$TIMEOUT"
    if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
        echo ""
        echo "  Warning: /emg/data not visible after ${TIMEOUT}s."
        echo "  The board may still be connecting. Starting monitor anyway."
        break
    fi
done
echo ""
echo "  ROS node is up."

# ─────────────────────────────────────────────────────────────────────────────
# STEP 4 — Live monitor
# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "  ══════════════════════════════════════════════════════"
echo "  Step 4/4 — Live EMG Monitor  (Ctrl-C to stop)"
echo "  ══════════════════════════════════════════════════════"
echo ""

python3 "$INSTALL_SCRIPTS/emg_monitor.py" --config "$CONFIG_FILE"
