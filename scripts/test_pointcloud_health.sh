#!/bin/bash
# test_pointcloud_health.sh — verify pointcloud data is flowing from cameras.
# Can be run inside the container or via: docker exec grasp_test bash scripts/test_pointcloud_health.sh

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi

set -euo pipefail

TOPICS=(
    "/cam1/d435_1/depth/color/points"
    "/cam2/d435_2/depth/color/points"
    "/fused_pointcloud"
)
TIMEOUT=15
FAIL=0

echo "=== Pointcloud Health Test ==="

# 1. Check that required topics exist
echo "--- Checking topic existence ---"
for topic in "${TOPICS[@]}"; do
    if ros2 topic list 2>/dev/null | grep -qF "$topic"; then
        echo "  OK: $topic exists"
    else
        echo "  FAIL: $topic not found"
        FAIL=$((FAIL + 1))
    fi
done

# 2. Check that nodes are alive
echo "--- Checking node existence ---"
NODES=(
    "pointcloud_fuser"
    "pointcloud_relay"
    "segmentation_bridge"
    "cloud_snapshot_node"
    "preshaping_service"
    "proximity_controller"
    "pipeline_manager"
    "twist_propagation"
    "robot_state_publisher"
    "hand_pose_publisher"
)
for node in "${NODES[@]}"; do
    if ros2 node list 2>/dev/null | grep -qF "$node"; then
        echo "  OK: $node is alive"
    else
        echo "  WARN: $node not found (may start later)"
    fi
done

# 3. Subscribe to each pointcloud topic and check for non-empty data
echo "--- Checking pointcloud data ---"
for topic in "${TOPICS[@]}"; do
    if ! ros2 topic list 2>/dev/null | grep -qF "$topic"; then
        echo "  SKIP: $topic (not available)"
        continue
    fi
    echo "  Waiting for data on $topic (timeout: ${TIMEOUT}s)..."
    OUTPUT=$(timeout "$TIMEOUT" ros2 topic echo "$topic" --once --field width 2>/dev/null || echo "TIMEOUT")
    if [ "$OUTPUT" = "TIMEOUT" ] || [ -z "$OUTPUT" ]; then
        echo "  FAIL: $topic — no data received within ${TIMEOUT}s"
        FAIL=$((FAIL + 1))
    else
        echo "  OK: $topic — width=$OUTPUT (non-empty)"
    fi
done

# 4. Check robot_description is available
echo "--- Checking robot_description ---"
if ros2 param get /robot_state_publisher robot_description &>/dev/null; then
    echo "  OK: robot_description param available"
else
    echo "  WARN: robot_description not available"
fi

echo ""
if [ "$FAIL" -gt 0 ]; then
    echo "FAIL: $FAIL check(s) failed"
    exit 1
else
    echo "PASS: all pointcloud health checks passed"
fi
