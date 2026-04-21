#!/usr/bin/env bash
# run_inference.sh - Build the CPU image and run InterObject3D inference
#
# Usage:
#   ./docker/run_inference.sh                          # run all 5 toy instances
#   ./docker/run_inference.sh --number_of_instances=1  # run a single instance
#
# Run from the repo root directory.
# Pretrained weights are downloaded once to ./weights/ and reused on subsequent runs.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE_NAME="interobject3d-cpu"
WEIGHTS_DIR="$REPO_ROOT/weights"
WEIGHTS_FILE="$WEIGHTS_DIR/weights_exp14_14.pth"
WEIGHTS_URL="https://omnomnom.vision.rwth-aachen.de/data/3d_inter_obj_seg/scannet_official/weights/weights_exp14_14.pth"

# --- Build ---
echo "==> Building Docker image: $IMAGE_NAME"
docker build \
    -f "$SCRIPT_DIR/Dockerfile.cpu" \
    -t "$IMAGE_NAME" \
    "$REPO_ROOT"

# --- Weights ---
if [ ! -f "$WEIGHTS_FILE" ]; then
    echo "==> Downloading pretrained weights (~145 MB) to $WEIGHTS_DIR"
    mkdir -p "$WEIGHTS_DIR"
    wget -q --show-progress "$WEIGHTS_URL" -O "$WEIGHTS_FILE"
else
    echo "==> Weights already present at $WEIGHTS_FILE"
fi

# --- Run inference ---
echo "==> Running inference (CPU)"
docker run --rm \
    -v "$WEIGHTS_DIR:/weights:ro" \
    "$IMAGE_NAME" \
    python run_inter3d.py \
        --verbal=True \
        --instance_counter_id=0 \
        --number_of_instances=5 \
        --cubeedge=0.05 \
        --pretraining_weights=/weights/weights_exp14_14.pth \
        --dataset=scannet \
        --save_results_file=True \
        --results_file_name=results_scannet_mini \
        "$@"
