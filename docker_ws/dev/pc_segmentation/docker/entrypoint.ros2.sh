#!/usr/bin/env bash
# entrypoint for the segmentation_ros2 service.
#
# 1. Download weights if not present.
# 2. Start the Python 3.8 inference server in the background.
# 3. Poll /health until the model is loaded (model load takes ~10–30 s on CPU).
# 4. Source ROS2 and exec the ROS2 segmentation node.

set -e

WEIGHTS_URL="https://omnomnom.vision.rwth-aachen.de/data/3d_inter_obj_seg/scannet_official/weights/weights_exp14_14.pth"
WEIGHTS_FILE="/weights/weights_exp14_14.pth"
INFERENCE_PORT="${INFERENCE_SERVER_PORT:-5678}"

# ── Download weights ─────────────────────────────────────────────────────────
if [ ! -f "$WEIGHTS_FILE" ]; then
    echo "[entrypoint] Downloading pretrained weights (~145 MB)..."
    mkdir -p /weights
    wget -q --show-progress "$WEIGHTS_URL" -O "$WEIGHTS_FILE"
fi

# ── Start inference server ────────────────────────────────────────────────────
echo "[entrypoint] Starting inference server (Python 3.8, port $INFERENCE_PORT)..."
python3.8 /nodes/inference_server.py &
INFERENCE_PID=$!

# ── Wait until model is loaded ────────────────────────────────────────────────
echo "[entrypoint] Waiting for inference server to load model..."
for i in $(seq 1 120); do
    if wget -q -O /dev/null "http://127.0.0.1:${INFERENCE_PORT}/health" 2>/dev/null; then
        echo "[entrypoint] Inference server ready (${i}s)."
        break
    fi
    if ! kill -0 "$INFERENCE_PID" 2>/dev/null; then
        echo "[entrypoint] ERROR: Inference server process exited unexpectedly."
        exit 1
    fi
    sleep 1
done

# Final check
if ! wget -q -O /dev/null "http://127.0.0.1:${INFERENCE_PORT}/health" 2>/dev/null; then
    echo "[entrypoint] ERROR: Inference server did not respond after 120 s."
    exit 1
fi

# ── Launch ROS2 node ──────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
exec python3 /nodes/segmentation_ros2_node.py "$@"
