#!/usr/bin/env bash
# Download weights on first run, then start the Flask inference server.
set -e

WEIGHTS_PATH="${WEIGHTS_PATH:-/weights/weights_exp14_14.pth}"
WEIGHTS_URL="https://omnomnom.vision.rwth-aachen.de/data/3d_inter_obj_seg/scannet_official/weights/weights_exp14_14.pth"

if [ ! -f "$WEIGHTS_PATH" ]; then
    echo "[inference-entrypoint] Weights not found at $WEIGHTS_PATH — downloading..."
    mkdir -p "$(dirname "$WEIGHTS_PATH")"
    wget -q --show-progress "$WEIGHTS_URL" -O "$WEIGHTS_PATH"
    if [ ! -s "$WEIGHTS_PATH" ]; then
        echo "[inference-entrypoint] ERROR: Weights download failed (file is empty or missing)" >&2
        exit 1
    fi
    echo "[inference-entrypoint] Weights downloaded ($(du -h "$WEIGHTS_PATH" | cut -f1))."
else
    echo "[inference-entrypoint] Weights found at $WEIGHTS_PATH ($(du -h "$WEIGHTS_PATH" | cut -f1))."
fi

echo "[inference-entrypoint] Starting inference server..."
exec "$@"
