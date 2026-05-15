#!/bin/bash
# test_mia_hardware_smoke.sh — verify MIA serial connection, ros2_control
# controller graph, and /joint_states.effort.  Run on the Jetson or
# any host connected to the prosthesis hardware.

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi

set -euo pipefail

FAIL=0
SERVICE_WAIT=30

echo "=== MIA Hardware Smoke Test ==="
echo ""

# ------------------------------------------------------------------
# 1. Serial device check
# ------------------------------------------------------------------
echo "--- Checking serial device ---"
if [ -e /dev/ttyUSB0 ]; then
    echo "  PASS: /dev/ttyUSB0 present"
elif [ -e /dev/mia_hand ]; then
    echo "  PASS: /dev/mia_hand present"
else
    echo "  FAIL: neither /dev/ttyUSB0 nor /dev/mia_hand found"
    FAIL=$((FAIL + 1))
fi

# ------------------------------------------------------------------
# 2. Wait for controller_manager service
# ------------------------------------------------------------------
echo ""
echo "--- Waiting for /controller_manager/list_controllers ---"
SERVICE_READY=0
for i in $(seq 1 "$SERVICE_WAIT"); do
    if ros2 service list 2>/dev/null | grep -qF "/controller_manager/list_controllers"; then
        SERVICE_READY=1
        break
    fi
    sleep 1
done
if [ "$SERVICE_READY" -eq 1 ]; then
    echo "  PASS: /controller_manager/list_controllers available"
else
    echo "  FAIL: /controller_manager/list_controllers not available after ${SERVICE_WAIT}s"
    FAIL=$((FAIL + 1))
fi

# ------------------------------------------------------------------
# 3. Controller checks
# ------------------------------------------------------------------
echo ""
echo "--- Checking controllers ---"

CTRL_OUTPUT=$(ros2 control list_controllers 2>/dev/null || echo "")

if echo "$CTRL_OUTPUT" | grep -q "joint_state_broadcaster.*active"; then
    echo "  PASS: joint_state_broadcaster is active"
else
    echo "  FAIL: joint_state_broadcaster not found or not active"
    FAIL=$((FAIL + 1))
fi

FINGER_POS_CTRLS=(
    "thumb_pos_ff_controller"
    "index_pos_ff_controller"
    "mrl_pos_ff_controller"
    "group_pos_ff_controller"
)
FOUND_POS_CTRL=0
for ctrl in "${FINGER_POS_CTRLS[@]}"; do
    if echo "$CTRL_OUTPUT" | grep -qF "$ctrl"; then
        echo "  PASS: finger position controller $ctrl exists"
        FOUND_POS_CTRL=1
        break
    fi
done
if [ "$FOUND_POS_CTRL" -eq 0 ]; then
    echo "  FAIL: no finger position controller found"
    FAIL=$((FAIL + 1))
fi

# ------------------------------------------------------------------
# 4. Joint states check
# ------------------------------------------------------------------
echo ""
echo "--- Checking /joint_states ---"

JS_RAW=$(timeout 5 ros2 topic echo /joint_states --once 2>/dev/null || echo "TIMEOUT")

if [ "$JS_RAW" = "TIMEOUT" ] || [ -z "$JS_RAW" ]; then
    echo "  FAIL: /joint_states — no data received"
    FAIL=$((FAIL + 1))
else
    for joint in "j_thumb_fle" "j_index_fle" "j_mrl_fle"; do
        if echo "$JS_RAW" | grep -qF "$joint"; then
            echo "  PASS: /joint_states contains $joint"
        else
            echo "  FAIL: /joint_states missing $joint"
            FAIL=$((FAIL + 1))
        fi
    done

    POS_COUNT=$(echo "$JS_RAW" \
        | sed -n '/^position:/,/^velocity:/p' \
        | grep -c '^  -' || echo "0")
    POS_COUNT=$(echo "$POS_COUNT" | tr -d ' ')
    if [ "$POS_COUNT" -ge 3 ]; then
        echo "  PASS: /joint_states position has $POS_COUNT entries (>= 3)"
    else
        echo "  FAIL: /joint_states position has $POS_COUNT entries (< 3)"
        FAIL=$((FAIL + 1))
    fi

    EFF_COUNT=$(echo "$JS_RAW" \
        | sed -n '/^effort:/,$p' \
        | grep -c '^  -' || echo "0")
    EFF_COUNT=$(echo "$EFF_COUNT" | tr -d ' ')
    if [ "$EFF_COUNT" -ge 3 ]; then
        echo "  PASS: /joint_states effort has $EFF_COUNT entries (>= 3)"
    else
        echo "  FAIL: /joint_states effort has $EFF_COUNT entries (< 3)"
        FAIL=$((FAIL + 1))
    fi
fi

# ------------------------------------------------------------------
# 5. Motion test — guarded by ALLOW_MIA_MOTION env var
# ------------------------------------------------------------------
echo ""
echo "--- Motion test ---"

ALLOW_MOTION="${ALLOW_MIA_MOTION:-0}"

if [ "$ALLOW_MOTION" != "1" ]; then
    echo "  SKIP: ALLOW_MIA_MOTION is not 1 (value='${ALLOW_MIA_MOTION:-unset}')"
else
    CMD_TOPIC="/group_pos_ff_controller/commands"
    MOTION_OK=0

    if ! ros2 topic list 2>/dev/null | grep -qF "$CMD_TOPIC"; then
        echo "  FAIL: command topic $CMD_TOPIC not found"
        FAIL=$((FAIL + 1))
    else
        echo "  Reading initial positions from /joint_states..."
        INIT_POS=$(timeout 5 ros2 topic echo /joint_states --once --field position 2>/dev/null \
            | grep '^  -' | sed 's/^  - //' | tr '\n' ',' | sed 's/,$//' || echo "")

        if [ -z "$INIT_POS" ] || [ "$(echo "$INIT_POS" | tr ',' '\n' | wc -l)" -lt 3 ]; then
            echo "  FAIL: could not read 3 initial positions"
            FAIL=$((FAIL + 1))
        else
            echo "  Initial positions: [$INIT_POS]"

            P1=$(echo "$INIT_POS" | cut -d',' -f1)
            P2=$(echo "$INIT_POS" | cut -d',' -f2)
            P3=$(echo "$INIT_POS" | cut -d',' -f3)

            TARGET1=$(python3 -c "print($P1 + 0.01)")
            TARGET2=$(python3 -c "print($P2 + 0.01)")
            TARGET3=$(python3 -c "print($P3 + 0.01)")

            echo "  Sending position command: [$TARGET1, $TARGET2, $TARGET3] ..."
            ros2 topic pub --once "$CMD_TOPIC" std_msgs/msg/Float64MultiArray \
                "{data: [$TARGET1, $TARGET2, $TARGET3]}" 2>/dev/null || true

            echo "  Waiting 1s for motion..."
            sleep 1

            NEW_POS=$(timeout 5 ros2 topic echo /joint_states --once --field position 2>/dev/null \
                | grep '^  -' | sed 's/^  - //' | tr '\n' ',' | sed 's/,$//' || echo "")

            echo "  New positions: [$NEW_POS]"

            N1=$(echo "$NEW_POS" | cut -d',' -f1)
            N2=$(echo "$NEW_POS" | cut -d',' -f2)
            N3=$(echo "$NEW_POS" | cut -d',' -f3)

            CHANGED=$(python3 -c "
p1,p2,p3 = $P1,$P2,$P3
n1,n2,n3 = $N1,$N2,$N3
changed = abs(n1-p1) > 0.001 or abs(n2-p2) > 0.001 or abs(n3-p3) > 0.001
print(1 if changed else 0)
")
            if [ "$CHANGED" -eq 1 ]; then
                echo "  PASS: positions changed after command"
                MOTION_OK=1
            else
                echo "  FAIL: positions did not change (controller may not be active)"
                FAIL=$((FAIL + 1))
            fi

            echo "  Returning to 0.0 rad..."
            ros2 topic pub --once "$CMD_TOPIC" std_msgs/msg/Float64MultiArray \
                "{data: [0.0, 0.0, 0.0]}" 2>/dev/null || true
            echo "  Sent return-to-zero command"
        fi
    fi
fi

# ------------------------------------------------------------------
# Summary
# ------------------------------------------------------------------
echo ""
if [ "$FAIL" -gt 0 ]; then
    echo "FAIL: $FAIL check(s) failed"
    exit 1
else
    echo "PASS: all MIA hardware smoke checks passed"
fi
