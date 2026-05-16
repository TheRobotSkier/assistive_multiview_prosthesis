#!/bin/bash
# test_twist_propagation.sh — integration test for the twist propagation node
#
# Runs a Python test script that:
#   1. Starts the twist_propagation node
#   2. Publishes mock hand poses and point clouds
#   3. Verifies twist estimation, hit detection, and state machine transitions
#
# Must run inside the prosthesis Docker container where ROS 2 and the workspace
# are available.

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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_SCRIPT="${SCRIPT_DIR}/test_twist_propagation_integration.py"

if [ ! -f "$TEST_SCRIPT" ]; then
    echo "FAIL: test script not found at $TEST_SCRIPT"
    exit 1
fi

ros2 run twist_propagation twist_propagation_node >/tmp/twist_propagation_node.log 2>&1 &
NODE_PID=$!
cleanup() {
    kill "$NODE_PID" 2>/dev/null || true
    wait "$NODE_PID" 2>/dev/null || true
}
trap cleanup EXIT

python3 "$TEST_SCRIPT"
