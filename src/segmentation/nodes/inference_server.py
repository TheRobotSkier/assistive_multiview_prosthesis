#!/usr/bin/env python3
"""
Flask inference server for InterObject3D segmentation.

Runs under Python 3.8 (required by MinkowskiEngine + PyTorch 1.12).
Exposes two endpoints:
  GET  /health   → runtime diagnostics (device, CUDA availability, ME mode)
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

# Safety limit: reject clouds larger than this to prevent CUDA OOM on small GPUs.
# RTX 3050 (4 GB) struggles above ~50k points with MinkowskiEngine.
MAX_POINTS = int(os.environ.get("INFERENCE_MAX_POINTS", "50000"))

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"[inference_server] Detected device: {device}", flush=True)

# Collect diagnostics for health endpoint and startup logging
try:
    import MinkowskiEngine as ME
    me_version = ME.__version__
    # MinkowskiEngine does not expose a direct CUDA flag, but we can infer
    # from whether torch.cuda is available and ME compiled successfully.
    # If the import works on a non-CUDA build, it's CPU-only.
    me_cuda_available = torch.cuda.is_available()
except Exception as e:
    me_version = f"import_error: {e}"
    me_cuda_available = False

_cuda_version = torch.version.cuda if torch.version.cuda else None

def _gpu_memory_info() -> dict:
    """Return GPU memory stats if CUDA is available, else empty dict."""
    if not torch.cuda.is_available():
        return {}
    return {
        "gpu_memory_allocated_mb": round(torch.cuda.memory_allocated() / 1e6, 1),
        "gpu_memory_reserved_mb": round(torch.cuda.memory_reserved() / 1e6, 1),
        "gpu_memory_total_mb": round(torch.cuda.get_device_properties(0).total_memory / 1e6, 1),
    }


diagnostics = {
    "device": str(device),
    "torch_cuda_available": torch.cuda.is_available(),
    "torch_cuda_version": _cuda_version,
    "minkowski_engine_version": me_version,
    "minkowski_engine_cuda": me_cuda_available,
    **_gpu_memory_info(),
}

print(f"[inference_server] Diagnostics: {json.dumps(diagnostics)}", flush=True)

print(f"[inference_server] Loading model from {WEIGHTS_PATH} ...", flush=True)
_model_ready = False
try:
    _inseg = InteractiveSegmentationModel(pretraining_weights=WEIGHTS_PATH)
    _model = _inseg.create_model(device, _inseg.pretraining_weights_file)
    _model.eval()
    _model_ready = True
    print("[inference_server] Model ready.", flush=True)
except Exception as e:
    print(f"[inference_server] FATAL: Model loading failed: {e}", flush=True)
    import traceback
    traceback.print_exc()
    # Keep _model_ready = False so /health reports the failure
    # and /segment returns a clear error instead of crashing.

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
    """Return runtime diagnostics suitable for smoke-test assertions."""
    return jsonify({
        "status": "ok" if _model_ready else "model_not_loaded",
        "model_ready": _model_ready,
        "max_points": MAX_POINTS,
        **diagnostics,
        **_gpu_memory_info(),
    })


@app.route("/segment", methods=["POST"])
def segment():
    data = request.get_json(force=True)

    xyz = _decode(data["xyz"], np.float32).reshape(-1, 3)   # (N, 3)
    n = xyz.shape[0]

    if n > MAX_POINTS:
        print(f"[inference_server] Cloud too large ({n} points > {MAX_POINTS}) "
              f"— truncating to {MAX_POINTS}", flush=True)
        indices = np.random.choice(n, MAX_POINTS, replace=False)
        indices.sort()  # preserve spatial locality
        xyz = xyz[indices]
        n = MAX_POINTS
        truncated = True
    else:
        indices = None
        truncated = False

    if data.get("rgb"):
        rgb_full = _decode(data["rgb"], np.float32).reshape(-1, 3)
        rgb = rgb_full[indices] if truncated else rgb_full
    else:
        rgb = np.zeros((n, 3), dtype=np.float32)

    pos_clicks = data.get("positive_clicks", [])
    neg_clicks = data.get("negative_clicks", [])
    if not _model_ready:
        return jsonify({"error": "Model not loaded — check container logs for loading errors"}), 503

    cubeedge = float(data.get("cubeedge", 0.05))

    pos_mask = _build_click_mask(xyz, pos_clicks, cubeedge)
    neg_mask = _build_click_mask(xyz, neg_clicks, cubeedge)

    # feats: (N, 5) = [R, G, B, pos_click, neg_click]
    feats = np.column_stack([rgb, pos_mask, neg_mask]).astype(np.float32)
    feats_tensor = torch.from_numpy(feats).float().to(device)

    try:
        with torch.no_grad():
            pred, _ = _inseg.prediction(feats_tensor, xyz, _model, device)
    except RuntimeError as exc:
        # CUDA OOM or other GPU errors — clear cache and report
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        print(f"[inference_server] Inference error: {exc}", flush=True)
        return jsonify({"error": f"Inference failed: {exc}"}), 500
    except Exception as exc:
        print(f"[inference_server] Unexpected inference error: {exc}", flush=True)
        return jsonify({"error": f"Inference failed: {exc}"}), 500

    # Move to CPU for post-processing (click masks live on CPU as numpy arrays)
    pred = pred.cpu()

    # Release GPU memory eagerly
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    # Enforce click constraints: clicked points are definitively fg/bg
    pred[pos_mask > 0.5] = 1
    pred[neg_mask > 0.5] = 0

    mask = pred.numpy().astype(np.int32).tolist()

    # If cloud was truncated, expand mask back to original size
    # (non-sampled points default to background=0)
    if truncated and indices is not None:
        full_mask = [0] * len(data["xyz"])  # original size
        for i, idx in enumerate(indices):
            full_mask[idx] = mask[i]
        mask = full_mask

    return jsonify({"mask": mask, "truncated": truncated})


if __name__ == "__main__":
    # Single-threaded to avoid torch/ME concurrency issues
    print(f"[inference_server] Starting server on http://127.0.0.1:{PORT}", flush=True)
    app.run(host="127.0.0.1", port=PORT, threaded=False)
