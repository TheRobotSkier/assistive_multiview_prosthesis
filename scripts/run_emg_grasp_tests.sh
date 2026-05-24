#!/bin/bash
# run_emg_grasp_tests.sh — EMG grasp test orchestrator
#
# Runs mock integration tests (no hardware required) and, if env flags are set,
# the hardware validation checklist.
#
# Usage:
#   ./scripts/run_emg_grasp_tests.sh              # mock tests only
#   EMG_GRASP_HW_TEST=1 MIA_PORT=/dev/ttyUSB0 \
#     ./scripts/run_emg_grasp_tests.sh             # mock + hardware tests
#
# Exit code is non-zero if any test fails.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONTAINER_WS="/prosthesis_ws"

# ── Colors ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
DIM='\033[2m'
RESET='\033[0m'

PASS=0
FAIL=0
SKIP=0
RESULTS=()

# ── Helpers ───────────────────────────────────────────────────────────────────

run_test() {
    local name="$1"
    local cmd="$2"
    local logfile="/tmp/test_emg_grasp_${name}.log"
    local start elapsed

    start=$(date +%s%N)
    if eval "$cmd" > "$logfile" 2>&1; then
        elapsed=$(( ($(date +%s%N) - start) / 1000000 ))
        RESULTS+=("PASS|${name}|${elapsed}ms")
        PASS=$((PASS + 1))
        echo -e "  ${GREEN}PASS${RESET} ${name} (${elapsed}ms)"
    else
        elapsed=$(( ($(date +%s%N) - start) / 1000000 ))
        RESULTS+=("FAIL|${name}|${elapsed}ms")
        FAIL=$((FAIL + 1))
        echo -e "  ${RED}FAIL${RESET} ${name} (${elapsed}ms)"
        echo -e "  ${DIM}--- ${name} LOG ---${RESET}"
        cat "$logfile" | head -60
        echo -e "  ${DIM}--- END ---${RESET}"
    fi
}

run_test_skip_on_flag() {
    local name="$1"
    local cmd="$2"
    local env_flag="$3"

    if [[ "${!env_flag:-}" == "1" || "${!env_flag:-}" == "true" ]]; then
        run_test "$name" "$cmd"
    else
        SKIP=$((SKIP + 1))
        RESULTS+=("SKIP|${name}|0ms")
        echo -e "  ${YELLOW}SKIP${RESET} ${name} (${env_flag}=0)"
    fi
}

# ── Check test tools ──────────────────────────────────────────────────────────
echo "=========================================="
echo "  EMG Grasp Test Suite"
echo "=========================================="
echo ""

# Detect Python 3 and pytest
PYTHON3="${PYTHON3:-python3}"
if ! command -v "$PYTHON3" &>/dev/null; then
    echo -e "${RED}FATAL: $PYTHON3 not found${RESET}"
    exit 1
fi

# Check if pytest is available
if $PYTHON3 -m pytest --version &>/dev/null; then
    PYTEST="$PYTHON3 -m pytest"
elif $PYTHON3 -c "import pytest" &>/dev/null; then
    PYTEST="$PYTHON3 -m pytest"
elif command -v pytest &>/dev/null; then
    PYTEST="pytest"
else
    echo -e "${YELLOW}WARN: pytest not found — attempting simple import test${RESET}"
    PYTEST=""
fi

# Check for ROS2 (optional, only needed for hardware tests)
HAS_ROS=false
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    if command -v ros2 &>/dev/null; then
        HAS_ROS=true
    fi
fi

TEST_DIR="$WS_DIR/tests/emg_grasp"

echo -e "  Python:   $PYTHON3"
echo -e "  pytest:   ${PYTEST:-not found (import test only)}"
echo -e "  ROS2:     $HAS_ROS"
echo -e "  Test dir: $TEST_DIR"
echo ""

# ── Mock Integration Tests (no hardware, no ROS required) ─────────────────────
echo "--- Mock Integration Tests ---"
echo ""

MOCK_TEST_FILE="$TEST_DIR/test_mock_integration.py"
CONTRACT_TEST_FILE="$TEST_DIR/test_emg_grasp_contract.py"

if [ -f "$MOCK_TEST_FILE" ]; then
    if [ -n "$PYTEST" ]; then
        run_test "mock_integration" "$PYTEST -xvs $MOCK_TEST_FILE --tb=short"
    else
        run_test "mock_integration" "$PYTHON3 $MOCK_TEST_FILE"
    fi
else
    echo -e "  ${RED}FAIL${RESET} Mock test file not found: $MOCK_TEST_FILE"
    FAIL=$((FAIL + 1))
fi

if [ -f "$CONTRACT_TEST_FILE" ]; then
    if [ -n "$PYTEST" ]; then
        run_test "contract" "$PYTEST -xvs $CONTRACT_TEST_FILE --tb=short"
    else
        run_test "contract" "$PYTHON3 -c \"import sys; sys.path.insert(0, '$TEST_DIR'); from test_emg_grasp_contract import *; test_velocity_ramp_linear(); test_velocity_ramp_single_step(); test_check_stop_conditions_force(); test_check_stop_conditions_position(); test_check_stop_conditions_none(); print('All contract tests passed')\""
    fi
else
    echo -e "  ${YELLOW}SKIP${RESET} Contract tests not found"
    SKIP=$((SKIP + 1))
fi

echo ""

# ── Hardware Validation Tests (require env flags) ─────────────────────────────
echo "--- Hardware Validation Tests ---"
echo ""

if [ -f "$TEST_DIR/test_hardware_checklist.py" ]; then
    if $HAS_ROS; then
        if [ "${EMG_GRASP_HW_TEST:-}" = "1" ] || [ "${EMG_GRASP_HW_TEST:-}" = "true" ]; then
            echo -e "  ${CYAN}Hardware tests enabled ($EMG_GRASP_HW_TEST)${RESET}"
            run_test "hardware_checklist" "$PYTHON3 $TEST_DIR/test_hardware_checklist.py"
        else
            echo -e "  ${YELLOW}SKIP${RESET} Hardware tests not enabled."
            echo -e "  ${DIM}Set EMG_GRASP_HW_TEST=1 MIA_PORT=/dev/ttyUSB0 to enable.${RESET}"
            SKIP=$((SKIP + 1))
        fi
    else
        echo -e "  ${YELLOW}SKIP${RESET} ROS2 not available for hardware tests"
        SKIP=$((SKIP + 1))
    fi
else
    echo -e "  ${YELLOW}SKIP${RESET} Hardware test script not found"
    SKIP=$((SKIP + 1))
fi

echo ""

# ── EMG Bridge Unit Tests (if available) ──────────────────────────────────────
BRIDGE_TEST_PATH="$WS_DIR/src/emg_bridge"
if [ -d "$BRIDGE_TEST_PATH" ]; then
    echo "--- EMG Bridge Tests ---"
    echo ""
    # Run any existing pytest files in emg_bridge
    BRIDGE_TESTS=$(find "$BRIDGE_TEST_PATH" -name "test_*.py" 2>/dev/null || true)
    if [ -n "$BRIDGE_TESTS" ] && [ -n "$PYTEST" ]; then
        # Quick import check for core modules (no hardware needed)
        run_test "emg_bridge_imports" "$PYTHON3 -c \"
import sys; sys.path.insert(0, '$WS_DIR/src/emg_bridge')
from emg_bridge.config import GESTURE_NAMES, N_CHANNELS, SAMPLING_RATE
from emg_bridge.features import compute_features, N_FEATURES_TOTAL
import numpy as np
window = np.random.randn(8, 100).astype(np.float64)
feats = compute_features(window)
assert len(feats) == N_FEATURES_TOTAL, f'Expected {N_FEATURES_TOTAL}, got {len(feats)}'
print(f'EMG bridge: {N_CHANNELS} channels, {N_FEATURES_TOTAL} features, {len(GESTURE_NAMES)} gestures')
\""
    else
        echo -e "  ${YELLOW}SKIP${RESET} No emg_bridge unit tests found"
        SKIP=$((SKIP + 1))
    fi
    echo ""
fi

# ── Summary ───────────────────────────────────────────────────────────────────
echo "=========================================="
echo "  Results"
echo "=========================================="
echo ""
if [ ${#RESULTS[@]} -gt 0 ]; then
    printf "  %-8s %-35s %s\n" "STATUS" "TEST" "TIME"
    for result in "${RESULTS[@]}"; do
        IFS='|' read -r status name time <<< "$result"
        case "$status" in
            PASS) printf "  ${GREEN}%-8s${RESET} %-35s %s\n" "$status" "$name" "$time" ;;
            FAIL) printf "  ${RED}%-8s${RESET} %-35s %s\n" "$status" "$name" "$time" ;;
            SKIP) printf "  ${YELLOW}%-8s${RESET} %-35s %s\n" "$status" "$name" "$time" ;;
        esac
    done
fi

echo ""
echo "  Passed:  ${PASS}"
echo "  Failed:  ${FAIL}"
echo "  Skipped: ${SKIP}"
echo "=========================================="

if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
exit 0
