#!/usr/bin/env bash
# Start final dual-D435i Jetson stack plus host digital-twin grasp pipeline.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_DIR="$ROOT_DIR/docker"
JETSON_HOST="${JETSON_HOST:-robotlab}"
JETSON_DEPLOY_DIR="${JETSON_DEPLOY_DIR:-/home/robotlab/multiview_prosthesis}"
SKIP_JETSON="${SKIP_JETSON:-0}"

if command -v podman-compose >/dev/null 2>&1; then
    COMPOSE=(podman-compose)
elif command -v docker >/dev/null 2>&1; then
    COMPOSE=(docker compose)
else
    echo "Need podman-compose or docker compose" >&2
    exit 1
fi

cd "$ROOT_DIR"

if [ "$SKIP_JETSON" != "1" ]; then
    make jetson-sync
    ssh "$JETSON_HOST" "cd '$JETSON_DEPLOY_DIR/jetson' && make cameras-final && make openvins-final"
fi

cd "$COMPOSE_DIR"
"${COMPOSE[@]}" --profile digital_twin up -d segmentation digital_twin

echo "Full digital-twin pipeline started."
echo "Runtime checks:"
echo "  make check-full-pipeline"
echo "  make logs-digital-twin"
