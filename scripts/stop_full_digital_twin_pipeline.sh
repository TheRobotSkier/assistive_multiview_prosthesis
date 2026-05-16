#!/usr/bin/env bash
# Stop host digital twin and, when reachable, final Jetson camera/OpenVINS stack.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_DIR="$ROOT_DIR/docker"
JETSON_HOST="${JETSON_HOST:-robotlab}"
JETSON_DEPLOY_DIR="${JETSON_DEPLOY_DIR:-/home/robotlab/multiview_prosthesis}"
STOP_JETSON="${STOP_JETSON:-1}"

if command -v podman-compose >/dev/null 2>&1; then
    COMPOSE=(podman-compose)
elif command -v docker >/dev/null 2>&1; then
    COMPOSE=(docker compose)
else
    echo "Need podman-compose or docker compose" >&2
    exit 1
fi

cd "$COMPOSE_DIR"
"${COMPOSE[@]}" --profile digital_twin down

if [ "$STOP_JETSON" = "1" ]; then
    cd "$ROOT_DIR"
    if scripts/robotlab_connect.sh >/tmp/robotlab_stop_connect.log 2>&1; then
        ssh "$JETSON_HOST" "cd '$JETSON_DEPLOY_DIR/jetson' && make openvins-stop && make cameras-stop"
    else
        echo "Jetson not reachable; host stack stopped, Jetson stop skipped." >&2
        cat /tmp/robotlab_stop_connect.log >&2
    fi
fi

echo "Full digital-twin pipeline stopped."
