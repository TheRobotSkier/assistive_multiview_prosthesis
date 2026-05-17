#!/usr/bin/env bash
# Start the final dual-D435i perception stack plus digital-twin grasp pipeline.
# Default backend is x86: cameras, marker/OpenVINS-compatible odometry, digital
# twin, segmentation, and RViz all run on this machine.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_DIR="$ROOT_DIR/docker"
JETSON_HOST="${JETSON_HOST:-robotlab}"
JETSON_DEPLOY_DIR="${JETSON_DEPLOY_DIR:-/home/robotlab/multiview_prosthesis}"
DT_PERCEPTION_BACKEND="${DT_PERCEPTION_BACKEND:-x86}"

if command -v podman-compose >/dev/null 2>&1; then
    COMPOSE=(podman-compose)
elif command -v docker >/dev/null 2>&1; then
    COMPOSE=(docker compose)
else
    echo "Need podman-compose or docker compose" >&2
    exit 1
fi

cd "$ROOT_DIR"

# ── Backend selection ────────────────────────────────────────────────────
services=(segmentation digital_twin digital_twin_rviz)
export DT_GUI=false
export DT_CAMERA=true

case "$DT_PERCEPTION_BACKEND" in
    x86)
        echo "Starting x86 perception backend: local RealSense cameras + local OpenVINS-compatible odometry"
        export DT_MOCK_EMG="${DT_MOCK_EMG:-false}"
        services=(segmentation x86_cameras x86_openvins digital_twin digital_twin_rviz)
        ;;
    jetson)
        echo "Starting legacy Jetson perception backend"
        export DT_MOCK_EMG="${DT_MOCK_EMG:-false}"
        bash scripts/robotlab_connect.sh
        make jetson-sync
        ssh "$JETSON_HOST" "cd '$JETSON_DEPLOY_DIR/jetson' && make cameras-final && make openvins-final"
        ;;
    mock)
        echo "Starting mock backend: no physical cameras"
        export DT_CAMERA=false
        export DT_MOCK_EMG="${DT_MOCK_EMG:-true}"
        ;;
    *)
        echo "Unknown DT_PERCEPTION_BACKEND='$DT_PERCEPTION_BACKEND' (expected x86, jetson, or mock)" >&2
        exit 2
        ;;
esac

# ── X11 access ───────────────────────────────────────────────────────────
if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
    xhost +local: >/dev/null 2>&1 || true
fi

# ── Start host containers ────────────────────────────────────────────────
cd "$COMPOSE_DIR"
"${COMPOSE[@]}" --profile digital_twin up -d "${services[@]}"

echo ""
echo "Full digital-twin pipeline started."
echo "Mode: $DT_PERCEPTION_BACKEND"
echo "Host RViz container: digital_twin_rviz"
echo ""
echo "Runtime checks:"
echo "  make check-full-pipeline"
echo "  make logs-digital-twin"
echo ""
echo "To stop:"
echo "  make stop-full-digital-twin"
