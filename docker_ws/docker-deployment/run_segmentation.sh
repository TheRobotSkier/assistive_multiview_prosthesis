#!/usr/bin/env bash
# Starts both segmentation containers (inference server + ROS2 node) and
# removes them automatically on exit. Press Ctrl+C to stop both.
set -e
cd "$(dirname "$0")"

INFERENCE_SERVICE="segmentation_inference"
INFERENCE_CONTAINER="segmentation_inference"
ROS2_SERVICE="segmentation_ros2"
INFERENCE_URL="http://127.0.0.1:5678/health"
HEALTH_TIMEOUT=120

cleanup() {
    echo ""
    echo "[run_segmentation] Stopping containers..."
    docker stop "$INFERENCE_CONTAINER" 2>/dev/null || true
    docker rm -f "$INFERENCE_CONTAINER" 2>/dev/null || true
    docker compose stop "$ROS2_SERVICE" 2>/dev/null || true
    docker compose rm -f "$ROS2_SERVICE" 2>/dev/null || true
    echo "[run_segmentation] Done."
}
trap cleanup EXIT INT TERM

# Remove any stale containers from a previous interrupted run
echo "[run_segmentation] Removing any stale containers..."
docker rm -f "$INFERENCE_CONTAINER" 2>/dev/null || true

echo "[run_segmentation] Starting inference server (detached)..."
docker compose run --rm -d "$INFERENCE_SERVICE"

echo "[run_segmentation] Waiting for inference server at $INFERENCE_URL (up to ${HEALTH_TIMEOUT}s)..."
for i in $(seq 1 "$HEALTH_TIMEOUT"); do
    if curl -sf "$INFERENCE_URL" >/dev/null 2>&1 || wget -q -O- "$INFERENCE_URL" >/dev/null 2>&1; then
        echo "[run_segmentation] Inference server ready after ${i}s."
        break
    fi
    if [ "$i" -eq "$HEALTH_TIMEOUT" ]; then
        echo "[run_segmentation] ERROR: Inference server did not become ready within ${HEALTH_TIMEOUT}s. Aborting."
        exit 1
    fi
    sleep 1
done

echo "[run_segmentation] Starting ROS2 segmentation node (foreground — Ctrl+C to stop)..."
docker compose run --rm "$ROS2_SERVICE"
