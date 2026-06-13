#!/usr/bin/env python3
"""
MobileSAM inference server (V6 plan §6.4).

Standalone Flask service — zero ROS dependencies.  Takes an RGB image plus a
2D click and returns a dilated binary mask.  Replaces InterObject3D for the
2D segmentation stage of the TSDF-fusion pipeline.

Endpoints:
  GET  /health       → {"status": "ready"|"loading"|"error", ...}
  POST /segment_2d   → accepts base64 RGB image + click, returns mask PNG

Request JSON (/segment_2d):
  {
    "image_b64": "<base64-encoded PNG/JPG bytes>",
    "click_x":   int,   # pixel column of the positive click
    "click_y":   int,   # pixel row of the positive click
    "dilation_px": int  # optional, default 15
  }

Response JSON:
  {
    "mask_b64": "<base64-encoded PNG of the binary mask>",
    "height":   int,
    "width":    int,
    "inference_ms": float
  }

Validation gates (V6 plan):
  - Cold-start to /health=ready: < 5 s
  - Single inference:            < 300 ms (burst of 20 keyframes)
"""

import base64
import io
import os
import time

import numpy as np
from flask import Flask, request, jsonify

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PORT = int(os.environ.get("MOBILE_SAM_PORT", "5679"))
SAM_WEIGHTS = os.environ.get("SAM_WEIGHTS", "/weights/mobile_sam.pt")
DEFAULT_DILATION_PX = int(os.environ.get("MOBILE_SAM_DILATION_PX", "15"))

# ---------------------------------------------------------------------------
# Model loading (done once at startup)
# ---------------------------------------------------------------------------
_model_ready = False
_sam = None
_device = "cpu"
_load_error = None

print(f"[mobile_sam_server] SAM_WEIGHTS={SAM_WEIGHTS}", flush=True)


def _load_model():
    """Load MobileSAM.  Imported lazily so the module can be imported on the
    host (for unit tests) without torch/mobile_sam installed."""
    global _model_ready, _sam, _device, _load_error

    try:
        import torch
        _device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"[mobile_sam_server] Detected device: {_device}", flush=True)

        from mobile_sam import sam_model_registry, SamPredictor

        # The mobile_sam package uses "vit_t" for the TinyViT backbone.
        model_type = "vit_t"

        if not os.path.exists(SAM_WEIGHTS):
            raise FileNotFoundError(
                f"Weights not found at {SAM_WEIGHTS}. "
                f"Download mobile_sam.pt to models/ or mount at /weights/.")

        _sam = sam_model_registry[model_type](checkpoint=SAM_WEIGHTS)
        _sam.to(device=_device)
        _sam.eval()

        # Warm up the predictor with a dummy image to trigger any lazy CUDA
        # initialisation before the first real request.
        _predictor = SamPredictor(_sam)
        dummy = np.zeros((64, 64, 3), dtype=np.uint8)
        _predictor.set_image(dummy)

        _model_ready = True
        print("[mobile_sam_server] Model ready.", flush=True)
    except Exception as exc:  # noqa: BLE001
        _load_error = str(exc)
        _model_ready = False
        print(f"[mobile_sam_server] FATAL: Model loading failed: {exc}",
              flush=True)
        import traceback
        traceback.print_exc()


# Load immediately at import time (module-level) so the server is ready when
# Flask starts.  This measures cold-start time.
_cold_start_t0 = time.monotonic()
_load_model()
_cold_start_ms = (time.monotonic() - _cold_start_t0) * 1000.0
print(f"[mobile_sam_server] Cold-start: {_cold_start_ms:.0f} ms", flush=True)


# ---------------------------------------------------------------------------
# Flask app
# ---------------------------------------------------------------------------
app = Flask(__name__)


def _decode_image(image_b64: str) -> np.ndarray:
    """Decode a base64-encoded image into an (H, W, 3) uint8 RGB array."""
    import cv2  # lazy import — heavy dependency
    raw = base64.b64decode(image_b64)
    buf = np.frombuffer(raw, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)  # BGR
    if img is None:
        raise ValueError("Could not decode image from base64 payload")
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def segment_2d(
    image: np.ndarray,
    click_x: int,
    click_y: int,
    dilation_px: int = DEFAULT_DILATION_PX,
) -> np.ndarray:
    """Run MobileSAM on *image* with a single positive click.

    Returns a (H, W) uint8 mask (0/255) dilated by *dilation_px*.

    This is a pure function (no HTTP) so it can be unit-tested directly on the
    host without Flask.
    """
    import cv2
    from mobile_sam import SamPredictor

    if _sam is None:
        raise RuntimeError("MobileSAM model is not loaded")

    predictor = SamPredictor(_sam)
    predictor.set_image(image)

    masks, scores, _ = predictor.predict(
        point_coords=np.array([[click_x, click_y]]),
        point_labels=np.array([1]),  # 1 = foreground click
        multimask_output=False,
    )
    # masks: (1, H, W) bool
    mask = masks[0].astype(np.uint8) * 255

    # Dilate to expand the mask slightly — improves TSDF coverage.
    if dilation_px > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (dilation_px, dilation_px))
        mask = cv2.dilate(mask, kernel, iterations=1)

    return mask


@app.route("/health", methods=["GET"])
def health():
    """Return readiness status.  Used by TSDF fusion to check availability."""
    status = "ready" if _model_ready else ("error" if _load_error else "loading")
    return jsonify({
        "status": status,
        "model_ready": _model_ready,
        "device": _device,
        "weights": SAM_WEIGHTS,
        "cold_start_ms": round(_cold_start_ms, 1),
        "error": _load_error,
    })


@app.route("/segment_2d", methods=["POST"])
def segment_2d_endpoint():
    data = request.get_json(force=True)

    if not _model_ready:
        return jsonify({
            "error": "Model not loaded — check /health and container logs",
        }), 503

    try:
        image = _decode_image(data["image_b64"])
    except Exception as exc:  # noqa: BLE001
        return jsonify({"error": f"Image decode failed: {exc}"}), 400

    click_x = int(data["click_x"])
    click_y = int(data["click_y"])
    dilation_px = int(data.get("dilation_px", DEFAULT_DILATION_PX))

    h, w = image.shape[:2]
    if not (0 <= click_x < w and 0 <= click_y < h):
        return jsonify({
            "error": f"Click ({click_x},{click_y}) out of image bounds "
                     f"(W={w}, H={h})",
        }), 400

    import cv2

    t0 = time.monotonic()
    try:
        mask = segment_2d(image, click_x, click_y, dilation_px)
    except Exception as exc:  # noqa: BLE001
        print(f"[mobile_sam_server] Inference error: {exc}", flush=True)
        return jsonify({"error": f"Inference failed: {exc}"}), 500
    inference_ms = (time.monotonic() - t0) * 1000.0

    print(f"[mobile_sam_server] /segment_2d: {inference_ms:.1f} ms "
          f"({w}x{h}, dilation={dilation_px})", flush=True)

    # Encode mask as PNG → base64 for compact transport.
    mask_png = cv2.imencode(".png", mask)[1].tobytes()
    mask_b64 = base64.b64encode(mask_png).decode("ascii")

    return jsonify({
        "mask_b64": mask_b64,
        "height": h,
        "width": w,
        "inference_ms": round(inference_ms, 1),
    })


if __name__ == "__main__":
    print(f"[mobile_sam_server] Starting on http://0.0.0.0:{PORT}", flush=True)
    # threaded=False keeps torch inference single-threaded (same pattern as
    # the InterObject3D server).
    app.run(host="0.0.0.0", port=PORT, threaded=False)
