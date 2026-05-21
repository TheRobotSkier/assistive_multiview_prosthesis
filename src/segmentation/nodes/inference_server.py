#!/usr/bin/env python3
"""
Flask inference server for InterObject3D segmentation.

Runs under Python 3.8 (required by MinkowskiEngine + PyTorch 1.12).
Exposes two endpoints:
  GET  /health   → {"status": "ok"}
  POST /segment  → accepts base64-encoded xyz/rgb arrays + click lists,
                    returns binary per-point mask {"mask": [0, 1, ...]}

The model is loaded once at startup; subsequent requests share the loaded weights.
All requests are processed single-threaded to avoid torch/ME concurrency issues.

GPU support: CUDA is used automatically if available at runtime, otherwise
falls back to CPU transparently.
"""

import sys
import os

# Resolve InterObject3D training dir before any local imports
_APP_DIR = os.environ.get("INTEROBJECT3D_TRAINING_DIR",
                          "/app/InterObject3D/Minkowski/training")
sys.path.insert(0, _APP_DIR)

import base64
import json

import numpy as np
import torch
from flask import Flask, request, jsonify

from interactive_adaptation.interactive_adaptation import InteractiveSegmentationModel

WEIGHTS_PATH = os.environ.get("WEIGHTS_PATH", "/weights/weights_exp14_14.pth")
PORT = int(os.environ.get("INFERENCE_SERVER_PORT", "5678"))

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"[inference_server] Detected device: {device}", flush=True)

print(f"[inference_server] Loading model from {WEIGHTS_PATH} ...", flush=True)
_inseg = InteractiveSegmentationModel(pretraining_weights=WEIGHTS_PATH)
_model = _inseg.create_model(device, _inseg.pretraining_weights_file)
_model.eval()
print("[inference_server] Model ready.", flush=True)

app = Flask(__name__)


def _decode(b64: str, dtype) -> np.ndarray:
    return np.frombuffer(base64.b64decode(b64), dtype=dtype).copy()


def _build_click_mask(xyz: np.ndarray, clicks: list, cubeedge: float) -> np.ndarray:
    """Return (N,) float32 mask: 1 if point falls inside any click cube."""
    mask = np.zeros(len(xyz), dtype=np.float32)
    for click in clicks:
        click_arr = np.array(click, dtype=np.float32)
        in_cube = np.all(np.abs(xyz - click_arr) < cubeedge, axis=1)
        mask[in_cube] = 1.0
    return mask


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok"})


@app.route("/segment", methods=["POST"])
def segment():
    data = request.get_json(force=True)

    xyz = _decode(data["xyz"], np.float32).reshape(-1, 3)   # (N, 3)
    n = xyz.shape[0]

    if data.get("rgb"):
        rgb = _decode(data["rgb"], np.float32).reshape(-1, 3)
    else:
        rgb = np.zeros((n, 3), dtype=np.float32)

    pos_clicks = data.get("positive_clicks", [])
    neg_clicks = data.get("negative_clicks", [])
    cubeedge = float(data.get("cubeedge", 0.05))

    pos_mask = _build_click_mask(xyz, pos_clicks, cubeedge)
    neg_mask = _build_click_mask(xyz, neg_clicks, cubeedge)

    # feats: (N, 5) = [R, G, B, pos_click, neg_click]
    feats = np.column_stack([rgb, pos_mask, neg_mask]).astype(np.float32)
    feats_tensor = torch.from_numpy(feats).float().to(device)

    with torch.no_grad():
        pred, _ = _inseg.prediction(feats_tensor, xyz, _model, device)

    # Move to CPU for post-processing (click masks live on CPU as numpy arrays)
    pred = pred.cpu()

    # Enforce click constraints: clicked points are definitively fg/bg
    pred[pos_mask > 0.5] = 1
    pred[neg_mask > 0.5] = 0

    mask = pred.numpy().astype(np.int32).tolist()
    return jsonify({"mask": mask})


if __name__ == "__main__":
    # Single-threaded to avoid torch/ME concurrency issues
    app.run(host="127.0.0.1", port=PORT, threaded=False)
