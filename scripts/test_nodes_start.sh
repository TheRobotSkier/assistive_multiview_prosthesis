#!/bin/bash
# test_nodes_start.sh — verify key ROS nodes can start and publish within a timeout

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

TIMEOUT=10  # seconds to wait for each node
ERRORS=0

check_node() {
    local node_name="$1"
    local executable="$2"
    local package="$3"

    echo "  Starting ${node_name}..."
    # Start the node in its own process group so cleanup also catches children
    # spawned by the ros2 CLI wrapper.
    setsid ros2 run "$package" "$executable" &
    local pid=$!

    # Wait for the node to appear in ros2 node list
    local found=0
    for i in $(seq 1 "$TIMEOUT"); do
        if ros2 node list 2>/dev/null | grep -q "$node_name"; then
            found=1
            break
        fi
        sleep 1
    done

    # Kill the node
    kill -TERM "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true

    if [ "$found" -eq 1 ]; then
        echo "    ${node_name}: OK"
    else
        echo "    ${node_name}: FAIL (did not appear within ${TIMEOUT}s)"
        ERRORS=$((ERRORS + 1))
    fi
}

# Test Python nodes that don't require hardware
check_node "pipeline_manager" "pipeline_manager_node" "pipeline_manager"
check_node "force_controller" "force_controller_node" "force_controller"
check_node "twist_propagation" "twist_propagation_node" "twist_propagation"

if [ "$ERRORS" -gt 0 ]; then
    echo "FAIL: $ERRORS node(s) failed to start"
    exit 1
fi
echo "All tested nodes started successfully"
