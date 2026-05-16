#!/usr/bin/env bash
# Start final dual-D435i Jetson stack plus host digital-twin grasp pipeline.
# Auto-detects Jetson reachability and falls back to mock/host-only mode.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_DIR="$ROOT_DIR/docker"
JETSON_HOST="${JETSON_HOST:-robotlab}"
JETSON_DEPLOY_DIR="${JETSON_DEPLOY_DIR:-/home/robotlab/multiview_prosthesis}"
SKIP_JETSON="${SKIP_JETSON:-auto}"

if command -v podman-compose >/dev/null 2>&1; then
    COMPOSE=(podman-compose)
elif command -v docker >/dev/null 2>&1; then
    COMPOSE=(docker compose)
else
    echo "Need podman-compose or docker compose" >&2
    exit 1
fi

cd "$ROOT_DIR"

# ── Auto-detect Jetson reachability ──────────────────────────────────────
jetson_reachable=false
if [ "$SKIP_JETSON" = "auto" ] || [ "$SKIP_JETSON" = "1" ]; then
    if [ "$SKIP_JETSON" = "1" ]; then
        echo "SKIP_JETSON=1: skipping Jetson, host-only digital twin mode"
    else
        # Try to reach Jetson via SSH
        if bash scripts/robotlab_connect.sh >/tmp/robotlab_startup_connect.log 2>&1; then
            if ssh -o ConnectTimeout=5 -o BatchMode=yes "$JETSON_HOST" "echo ok" >/dev/null 2>&1; then
                jetson_reachable=true
                echo "Jetson reachable at $JETSON_HOST"
            else
                echo "Jetson SSH failed — falling back to host-only digital twin mode"
            fi
        else
            echo "Jetson not reachable — falling back to host-only digital twin mode"
        fi
    fi
else
    jetson_reachable=true
fi

# ── Jetson side ──────────────────────────────────────────────────────────
if [ "$jetson_reachable" = true ]; then
    make jetson-sync
    ssh "$JETSON_HOST" "cd '$JETSON_DEPLOY_DIR/jetson' && make cameras-final && make openvins-final"
    export DT_CAMERA=true
    export DT_MOCK_EMG=false
else
    # Host-only mode: mock cloud + mock EMG so pipeline can run without Jetson
    export DT_CAMERA=false
    export DT_MOCK_EMG=true
    echo "Host-only mode: using mock cloud publisher and mock EMG trigger"
fi

export DT_GUI=false

# ── X11 access ───────────────────────────────────────────────────────────
if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
    xhost +local: >/dev/null 2>&1 || true
fi

# ── Start host containers ────────────────────────────────────────────────
cd "$COMPOSE_DIR"
"${COMPOSE[@]}" --profile digital_twin up -d segmentation digital_twin digital_twin_rviz

echo ""
echo "Full digital-twin pipeline started."
if [ "$jetson_reachable" = true ]; then
    echo "Mode: Jetson cameras + host digital twin"
else
    echo "Mode: Host-only (mock cloud + mock EMG)"
fi
echo "Host RViz container: digital_twin_rviz"
echo ""
echo "Runtime checks:"
echo "  make check-full-pipeline"
echo "  make logs-digital-twin"
echo ""
echo "To stop:"
echo "  make stop-full-digital-twin"
