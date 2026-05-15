#!/bin/bash
# print_force.sh — launch MIA ros2_control, reset to relaxed, continuously print force values

# Source ROS workspace BEFORE set -euo pipefail — ROS setup scripts
# reference unset variables (e.g. AMENT_TRACE_SETUP_FILES) that would
# trigger the -u (nounset) guard.
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi

set -eo pipefail

HAND_PORT="${MIA_PORT:-/dev/ttyUSB0}"

echo "=== MIA Hand Force Monitor ==="
echo "Hand port: $HAND_PORT"
echo ""

# ---- rebuild with emergency_stop_on_ fix ----
echo "[1/5] Rebuilding MIA driver with safety fixes..."
cd /prosthesis_ws
colcon build --packages-select mia_hand_driver mia_hand_ros2_control --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
source /prosthesis_ws/install/setup.bash 2>/dev/null || true

# ---- fix controller config ----
echo "[2/5] Patching controller config..."
sed -i "/j_thumb_opp/d" /prosthesis_ws/install/mia_hand_ros2_control/share/mia_hand_ros2_control/config/mia_hand_controllers.yaml 2>/dev/null || true

# ---- launch ros2_control ----
echo "[3/5] Launching ros2_control..."
ros2 launch mia_hand_ros2_control mia_hand_system_interface_launch.py \
  serial_port:="$HAND_PORT" \
  rviz2_gui:=false \
  use_mock_hardware:=false \
  controller:=group_pos_ff_controller &
LAUNCH_PID=$!
trap "kill $LAUNCH_PID 2>/dev/null; echo ''; echo 'Force monitor stopped.'" EXIT INT TERM

# ---- wait for controller_manager ----
echo "[4/5] Waiting for controller_manager..."
for i in $(seq 1 30); do
  if ros2 service list 2>/dev/null | grep -q "/controller_manager/list_controllers"; then
    echo "       Controller manager ready."
    break
  fi
  sleep 1
done
sleep 2

# ---- reset to relaxed ----
echo "[5/5] Resetting hand to relaxed position (0.0 rad)..."
ros2 topic pub --once /group_pos_ff_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0]}" 2>/dev/null
sleep 2

echo ""
echo "=========================================="
echo "  Force Monitor Active"
echo "  Format: [thumb, index, mrl] raw ADC"
echo "  Press Ctrl+C to stop"
echo "=========================================="
echo ""

ros2 topic echo /joint_states --field effort
