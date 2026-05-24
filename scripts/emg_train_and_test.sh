#!/usr/bin/env bash
# emg_train_and_test.sh
#
# In-container entrypoint for `make up-grasp-test-train`.
# Supports the three-phase `make up-grasp-test-train` flow:
#
#   Phase 1 — collect_data  (interactive — guides you through each gesture)
#   Phase 2 — train         (automated  — feature extraction + LDA + calibration)
#   Phase 3 — grasp test    (host launches after training confirmation)
#
# Required hardware:
#   Phase 1  MindRove WiFi armband — must be on the same network as the host
#   Phase 3  Mia Hand on /dev/ttyUSB0
#
# Tuning knobs (set as env vars or via make variables):
#   EMG_REPS      Number of recording repetitions per gesture  (default: 3)
#   EMG_DURATION  Recording duration per rep in seconds        (default: 5)
#
# Example (more reps, longer recordings):
#   make up-grasp-test-train EMG_REPS=5 EMG_DURATION=7

set -euo pipefail

WORKSPACE=/prosthesis_ws
DATA_DIR=/prosthesis_ws/data
MODEL_DIR=/prosthesis_ws/models

# Tunable defaults — overridden by env vars injected from the Makefile
EMG_REPS="${EMG_REPS:-3}"
EMG_DURATION="${EMG_DURATION:-5}"

# ── ANSI helpers ──────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

banner() {
    local title="$1"
    local subtitle="${2:-}"
    local width=54
    echo ""
    printf "${BOLD}╔%s╗${NC}\n" "$(printf '%0.s═' $(seq 1 $width))"
    printf "${BOLD}║  %-${width}s║${NC}\n" "$title"
    [ -n "$subtitle" ] && printf "${BOLD}║  %-${width}s║${NC}\n" "$subtitle"
    printf "${BOLD}╚%s╝${NC}\n" "$(printf '%0.s═' $(seq 1 $width))"
    echo ""
}

ok()   { echo -e "${GREEN}[✓]${NC} $*"; }
info() { echo -e "${CYAN}[→]${NC} $*"; }
warn() { echo -e "${YELLOW}[!]${NC} $*"; }

# ── 0. Build changed Python packages ─────────────────────────────────────────
echo ""
info "Sourcing ROS 2 and building workspace packages..."
# ROS 2 setup files reference variables like AMENT_TRACE_SETUP_FILES without
# defaults, which trips set -u.  Suspend nounset around every source call.
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$WORKSPACE"

if [ ! -f install/setup.bash ]; then
    warn "No prior build found — performing full workspace build (this may take a few minutes)..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -6
else
    # emg_bridge needs --symlink-install so that __file__ resolves to the source
    # tree and run_classifier_entry.py can find the scripts/ directory.
    # prosthesis_launch has packages=[] (data files only) so plain copy-install
    # is more reliable and avoids ament_python symlink conflicts on re-runs.
    info "Incremental build: emg_bridge (symlink-install)"
    colcon build --packages-select emg_bridge --symlink-install 2>&1 | tail -4
    info "Incremental build: prosthesis_launch mia_hand_ros2_control dependencies (copy-install)"
    colcon build --packages-up-to prosthesis_launch mia_hand_ros2_control \
        --packages-skip emg_bridge \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
set +u
source install/setup.bash
set -u
ok "Workspace ready."

mkdir -p "$DATA_DIR" "$MODEL_DIR"

# ── Phase 1: interactive data collection ─────────────────────────────────────
banner \
    "Phase 1 — EMG Data Collection" \
    "Connect the MindRove WiFi armband first."

echo -e "  Reps per gesture : ${BOLD}${EMG_REPS}${NC}"
echo -e "  Duration per rep : ${BOLD}${EMG_DURATION} s${NC}"
echo -e "  Output directory : ${BOLD}${DATA_DIR}${NC}"
echo ""

ros2 run emg_bridge collect_data \
    --output-dir "$DATA_DIR" \
    --reps      "$EMG_REPS" \
    --duration  "$EMG_DURATION"

ok "Data collection complete."

# ── Phase 2: automated training ───────────────────────────────────────────────
banner \
    "Phase 2 — Training Classifier" \
    "Automated — no action needed."

ros2 run emg_bridge train \
    --data-dir  "$DATA_DIR" \
    --model-dir "$MODEL_DIR"

ok "Classifier and proportional calibration saved to ${MODEL_DIR}."

# ── Phase 3: live EMG grasp test ──────────────────────────────────────────────
banner \
    "Phase 3 — Live EMG Grasp Test" \
    "Waiting for ENTER before host launches it."

# The train script (phases 1 & 2) and the live grasp test (phase 3) run in
# different container lifecycles.  We exit here so the Makefile can hand off
# to `make up-grasp-test`, which uses the exact same compose service that is
# known to work.
ok "Training complete. Models saved to ${MODEL_DIR}"
info "Return to the host terminal, review the prompt, and press ENTER to launch the live grasp test."
