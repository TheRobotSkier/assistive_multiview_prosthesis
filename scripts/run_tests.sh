#!/bin/bash
# run_tests.sh — orchestrator for all smoke tests
# Runs each test script and prints a summary table.
# Exit code is non-zero if any test fails.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS workspace BEFORE set -euo pipefail — ROS setup scripts
# reference unset variables (e.g. AMENT_TRACE_SETUP_FILES) that would
# trigger the -u (nounset) guard.
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi

set -euo pipefail

PASS=0
FAIL=0
RESULTS=()

run_test() {
    local name="$1"
    local script="$2"
    local start
    start=$(date +%s%N)
    if bash "$script" > /tmp/test_"${name}".log 2>&1; then
        local elapsed=$(( ($(date +%s%N) - start) / 1000000 ))
        RESULTS+=("PASS|${name}|${elapsed}ms")
        PASS=$((PASS + 1))
    else
        local elapsed=$(( ($(date +%s%N) - start) / 1000000 ))
        RESULTS+=("FAIL|${name}|${elapsed}ms")
        FAIL=$((FAIL + 1))
        echo "--- ${name} LOG ---"
        cat /tmp/test_"${name}".log
        echo "--- END ---"
    fi
}

echo "=========================================="
echo "  Prosthesis Smoke Tests"
echo "=========================================="
echo ""

run_test "build"       "$SCRIPT_DIR/test_build.sh"
run_test "launch"      "$SCRIPT_DIR/test_launch_syntax.sh"
run_test "preshaping"  "$SCRIPT_DIR/test_preshaping_so.sh"
run_test "nodes_start" "$SCRIPT_DIR/test_nodes_start.sh"
run_test "twist_prop"  "$SCRIPT_DIR/test_twist_propagation.sh"

# Run the full pytest unit test suite (247 tests across 8 packages).
# This is the highest-value hardware-free test layer.
run_test "unit"        "$SCRIPT_DIR/test_unit.sh"

echo ""
echo "=========================================="
echo "  Results"
echo "=========================================="
printf "  %-6s %-25s %s\n" "STATUS" "TEST" "TIME"
for result in "${RESULTS[@]}"; do
    IFS='|' read -r status name time <<< "$result"
    if [ "$status" = "PASS" ]; then
        printf "  \033[32m%-6s\033[0m %-25s %s\n" "$status" "$name" "$time"
    else
        printf "  \033[31m%-6s\033[0m %-25s %s\n" "$status" "$name" "$time"
    fi
done
echo ""
echo "  Passed: ${PASS}  Failed: ${FAIL}"
echo "=========================================="

if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
exit 0
