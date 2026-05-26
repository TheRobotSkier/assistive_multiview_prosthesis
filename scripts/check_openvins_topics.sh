#!/usr/bin/env bash
#
# check_openvins_topics.sh
# Preflight: verify that both OpenVINS odometry topics are publishing.
# Exits 0 if both are present, 1 if one or both are missing.
#
set -euo pipefail

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f /prosthesis_ws/install/setup.bash ]; then
    source /prosthesis_ws/install/setup.bash
fi

HEAD_ODOM="/ov_msckf/odomimu"
ARM_ODOM="/ov_msckf_arm/odomimu"
TIMEOUT_S="${1:-10}"

echo "[preflight-openvins] Checking OpenVINS odometry topics (timeout=${TIMEOUT_S}s)..."

head_ok=false
arm_ok=false

echo "[preflight-openvins] Waiting for head odom topic: ${HEAD_ODOM}..."
if ros2 topic list 2>/dev/null | grep -qF "${HEAD_ODOM}"; then
    head_ok=true
    echo "[preflight-openvins]   Head odom topic FOUND."
else
    echo "[preflight-openvins]   Head odom topic MISSING (${HEAD_ODOM})"
fi

echo "[preflight-openvins] Waiting for arm odom topic: ${ARM_ODOM}..."
if ros2 topic list 2>/dev/null | grep -qF "${ARM_ODOM}"; then
    arm_ok=true
    echo "[preflight-openvins]   Arm odom topic FOUND."
else
    echo "[preflight-openvins]   Arm odom topic MISSING (${ARM_ODOM})"
fi

if $head_ok && $arm_ok; then
    echo "[preflight-openvins] PASS: Both OpenVINS odometry topics present."
    exit 0
else
    echo "[preflight-openvins] FAIL: One or both OpenVINS odometry topics missing."
    exit 1
fi
