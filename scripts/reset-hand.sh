#!/bin/bash
# reset-hand.sh — Open hand, zero haptics, stop wrist/hand.
#
# Usage (inside container, after source install/setup.bash):
#   bash /prosthesis_ws/src/reset-hand.sh
#
# Requires ros2_control + controller_manager to be running for hand reset.
# Haptics and wrist are best-effort — they succeed only if the respective
# nodes are running.
set -e

if [ -f /opt/ros/jazzy/setup.bash ]; then
    . /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    . /prosthesis_ws/install/setup.bash
fi
HAPTIC_TOPIC="/haptic_band/motors"
WRIST_TOPIC="/wrist/set_position"
HAND_POS_TOPIC="/group_pos_ff_controller/commands"
HAND_VEL_TOPIC="/group_vel_ff_controller/commands"
CMGR="/controller_manager"

topic_exists()  { ros2 topic list 2>/dev/null | grep -qF "$1" && return 0 || return 1; }
svc_exists()    { ros2 service list 2>/dev/null | grep -qF "$1" && return 0 || return 1; }

# ── Haptics ──────────────────────────────────────────────────────────
echo "[reset] Stopping haptics..."
if topic_exists "$HAPTIC_TOPIC"; then
    ros2 topic pub -1 "$HAPTIC_TOPIC" std_msgs/msg/Float32MultiArray \
        "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" 2>/dev/null || true
    echo "[reset]   Haptics zeroed"
else
    echo "[reset]   (haptic bridge not running)"
fi

# ── Wrist ────────────────────────────────────────────────────────────
echo "[reset] Stopping wrist..."
if topic_exists "$WRIST_TOPIC"; then
    ros2 topic pub -1 "$WRIST_TOPIC" std_msgs/msg/Float64MultiArray \
        "{data: [0.0, 0.0]}" 2>/dev/null || true
    echo "[reset]   Wrist stop sent"
else
    echo "[reset]   (wrist driver not running)"
fi

# ── Hand ─────────────────────────────────────────────────────────────
echo "[reset] Opening hand..."
if svc_exists "${CMGR}/switch_controller"; then
    ros2 service call "${CMGR}/configure_controller" \
        controller_manager_msgs/srv/ConfigureController \
        "{name: group_pos_ff_controller}" 2>/dev/null || true

    ros2 service call "${CMGR}/switch_controller" \
        controller_manager_msgs/srv/SwitchController \
        "{activate_controllers: [group_pos_ff_controller], deactivate_controllers: [group_vel_ff_controller, group_pos_vel_controller], strictness: 1, activate_asap: true}" 2>/dev/null || true

    sleep 0.5

    ros2 topic pub -1 "$HAND_POS_TOPIC" std_msgs/msg/Float64MultiArray \
        "{data: [0.0, 0.0, 0.0]}" 2>/dev/null || true
    echo "[reset]   Hand opened"

    if topic_exists "$HAND_VEL_TOPIC"; then
        ros2 topic pub -1 "$HAND_VEL_TOPIC" std_msgs/msg/Float64MultiArray \
            "{data: [0.0, 0.0, 0.0]}" 2>/dev/null || true
        echo "[reset]   Velocity zeroed"
    fi
else
    echo "[reset]   (controller_manager not running)"
fi

echo "[reset] Done."
