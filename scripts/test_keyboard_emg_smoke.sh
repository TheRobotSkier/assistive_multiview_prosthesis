#!/usr/bin/env bash
# Smoke test for the keyboard EMG emulation launch path.
#
# Boots the mia-haptic-force-test container and runs the keyboard-EMG
# test inside it.  The in-container monitor (test_keyboard_emg_smoke.py)
# starts the launch, watches the supervisor's /test/stage topic and quits
# cleanly as soon as the test reaches ``waiting_for_activation`` (the point
# where it would normally wait for a sustained POWER gesture).
#
# This is a graph-health smoke test, not a full grasp run: the keyboard
# emulator stays idle so no gesture ever fires, but every node, every
# topic, and the controller_manager must come up.
#
# Usage:
#   scripts/test_keyboard_emg_smoke.sh            # default 20s timeout
#   TIMEOUT=30 scripts/test_keyboard_emg_smoke.sh # custom timeout
#
# Exit codes:
#   0  launch graph reached waiting_for_activation within timeout
#   1  timeout, or launch exited non-zero
#   2  container could not be started
#   3  missing prerequisites

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TIMEOUT="${TIMEOUT:-20}"

# ── Pre-flight ────────────────────────────────────────────────────────
[ -f "${REPO_ROOT}/scripts/menu_test.sh" ] || { echo "Missing menu_test.sh"; exit 3; }
[ -f "${REPO_ROOT}/scripts/test_keyboard_emg_smoke.py" ] || { echo "Missing test_keyboard_emg_smoke.py"; exit 3; }
[ -f "${REPO_ROOT}/config/mia_haptic_force_test.yaml" ] || { echo "Missing config YAML"; exit 3; }

# Use the detected MIA / wrist ports from the host so the hardware
# interface opens the real device instead of falling back to defaults.
DETECTED="$(bash "${REPO_ROOT}/scripts/detect_usb_host.sh" 2>/dev/null || true)"
eval "${DETECTED:-}"
MIA_PORT="${MIA_PORT:-${DETECTED_MIA_PORT:-}}"
WRIST_PORT="${WRIST_PORT:-${DETECTED_WRIST_PORT:-}}"

# ── Ensure container is up ───────────────────────────────────────────
COMPOSE_DIR="${REPO_ROOT}/docker"
cd "${COMPOSE_DIR}"
DOCKER_CMD="${DOCKER_CMD:-$(command -v podman || command -v docker)}"
if [ "${DOCKER_CMD}" = "podman" ] && command -v podman-compose >/dev/null 2>&1; then
    COMPOSE="podman-compose"
elif [ "${DOCKER_CMD}" = "docker" ] || command -v docker >/dev/null 2>&1; then
    COMPOSE="docker compose"
else
    COMPOSE="${DOCKER_CMD} compose"
fi

echo "[smoke] ensuring mia-haptic-force-test container is running…"
COMPOSE_PROFILES=mia-haptic-force-test \
    MIA_SERIAL_PORT="${MIA_PORT:-/dev/ttyMiaHand}" \
    WRIST_SERIAL_PORT="${WRIST_PORT:-/dev/ttyDynamixel}" \
    ${COMPOSE} up -d mia-haptic-force-test >/dev/null 2>&1 || {
    echo "[smoke] failed to start container via compose"
    exit 2
}

# ── Build env-arg string, guarding empty port values ─────────────────
ENV_ARGS=""
[ -n "${MIA_PORT}" ]   && ENV_ARGS="${ENV_ARGS} -e MIA_SERIAL_PORT=${MIA_PORT}"
[ -n "${WRIST_PORT}" ] && ENV_ARGS="${ENV_ARGS} -e WRIST_SERIAL_PORT=${WRIST_PORT}"
ENV_ARGS="${ENV_ARGS} \
    -e EMG_DATA_DIR=/app/data \
    -e EMG_MODEL_DIR=/app/models \
    -e CONFIG_PATH=/prosthesis_ws/config/mia_haptic_force_test.yaml \
    -e WRIST_ENABLE=true \
    -e HAPTIC_ENABLE=true \
    -e FORCE_RETRAIN=false \
    -e MOCK_HARDWARE=true \
    -e USE_MULTI_NODE=true \
    -e KEYBOARD_EMG=true \
    -e AUTO_KILL_S=0 \
    -e HAPTIC_BT_ADDR1=${HAPTIC_BT_ADDR1:-842E1409E14E} \
    -e EMG_BOARD_IP=${EMG_BOARD_IP:-10.27.30.3}"

# ── Build launch-arg string for the in-container monitor ─────────────
# wrist_enable:=false → no wrist driver crash on missing port
# haptic_enable:=false → no Bluetooth bridge / device-busy noise
LAUNCH_ARGS="config_path:=/prosthesis_ws/config/mia_haptic_force_test.yaml \
    emg_enable:=false \
    keyboard_emg:=true \
    use_multi_node:=true \
    wrist_enable:=false \
    mock_hardware:=true \
    haptic_enable:=false"

# ── Run the in-container monitor with a wall-clock timeout ───────────
echo "[smoke] timeout: ${TIMEOUT}s"
echo "[smoke] exec: ${DOCKER_CMD} exec -it mia-haptic-force-test \\"
echo "          python3 /prosthesis_ws/scripts/test_keyboard_emg_smoke.py \\"
echo "          --timeout-s ${TIMEOUT}"

# Copy the smoke test into the container (it's bind-mounted from scripts/
# but the monitor reads it from /prosthesis_ws/scripts/ inside).
SCRIPT_IN_CONTAINER="/prosthesis_ws/scripts/test_keyboard_emg_smoke.py"

# Use a foreground subshell so we can apply the timeout with `timeout`.
set +e
TEST_GRASP_LAUNCH_ARGS="${LAUNCH_ARGS}" \
    timeout --foreground --kill-after=5s "${TIMEOUT}s" \
    ${DOCKER_CMD} exec ${ENV_ARGS} \
    mia-haptic-force-test \
    /bin/bash -lc "cd /prosthesis_ws && source /opt/ros/jazzy/setup.bash && source /prosthesis_ws/install/setup.bash 2>/dev/null; export PYTHONPATH=/prosthesis_ws/scripts:\"\$PYTHONPATH\"; exec python3 ${SCRIPT_IN_CONTAINER} --timeout-s ${TIMEOUT} --launch-args '${LAUNCH_ARGS}'"
rc=$?

# timeout exit code is 124 on timeout
if [ "${rc}" -eq 124 ]; then
    echo "[smoke] ✗ test exceeded ${TIMEOUT}s wall-clock timeout"
    exit 1
fi

if [ "${rc}" -eq 0 ]; then
    echo "[smoke] ✓ test passed"
else
    echo "[smoke] ✗ test failed (exit code ${rc})"
fi
exit "${rc}"
