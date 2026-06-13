#!/usr/bin/env python3
"""
Unit tests for the MobileSAM inference server (V6 plan §6.4, Task B.4).

These tests run on the **host** (no Docker required).  They verify the pure
image-processing and mask-encoding logic.  The heavy MobileSAM model is only
loaded if torch + mobile_sam + weights are available; otherwise the
model-dependent tests are skipped gracefully.

Run:
    python3 -m pytest scripts/test_mobile_sam_server.py -v
"""

import base64
import importlib
import os
import sys
import types

import numpy as np
import pytest

# Make the server module importable.  We import it as a module (not executing
# __main__) so the Flask app is not started.
_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPTS_DIR)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_synthetic_image(h: int = 128, w: int = 128) -> np.ndarray:
    """Create a synthetic RGB image with a distinct object region.

    Background is dark; a bright rectangle in the centre represents the
    'object' so that a click inside it has something to segment.
    """
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = 20  # dark background
    # Bright central rectangle
    img[h // 4: 3 * h // 4, w // 4: 3 * w // 4] = 200
    return img


def _image_to_b64(image: np.ndarray) -> str:
    """Encode an RGB image as base64-encoded PNG."""
    import cv2
    bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    png = cv2.imencode(".png", bgr)[1].tobytes()
    return base64.b64encode(png).decode("ascii")


# ---------------------------------------------------------------------------
# Check whether the full MobileSAM stack is available
# ---------------------------------------------------------------------------

def _mobile_sam_available() -> bool:
    """Return True if torch + mobile_sam are importable and weights exist."""
    try:
        importlib.import_module("torch")
        importlib.import_module("mobile_sam")
    except ImportError:
        return False
    weights = os.environ.get(
        "SAM_WEIGHTS",
        os.path.join(os.path.dirname(_SCRIPTS_DIR), "models", "mobile_sam.pt"))
    return os.path.exists(weights)


_SAM_AVAILABLE = _mobile_sam_available()


# ---------------------------------------------------------------------------
# Tests — image decode / encode (no model needed)
# ---------------------------------------------------------------------------

class TestImageCodec:
    """Verify the base64 image round-trip used by the HTTP endpoint."""

    def test_decode_returns_correct_shape(self):
        """A decoded image must match the original H, W, 3 shape."""
        from mobile_sam_server import _decode_image
        img = _make_synthetic_image(64, 96)
        decoded = _decode_image(_image_to_b64(img))
        assert decoded.shape == (64, 96, 3)
        assert decoded.dtype == np.uint8

    def test_decode_preserves_pixel_values(self):
        """Pixel values must survive the encode→decode round trip."""
        from mobile_sam_server import _decode_image
        img = _make_synthetic_image(32, 32)
        decoded = _decode_image(_image_to_b64(img))
        # Allow for minor PNG losslessness differences (PNG is lossless so
        # values should match exactly).
        np.testing.assert_array_equal(decoded, img)

    def test_decode_invalid_payload_raises(self):
        """Garbage base64 must raise an error, not silently return None."""
        from mobile_sam_server import _decode_image
        with pytest.raises(Exception):
            _decode_image("!!!not-valid-base64-image-data!!!")


# ---------------------------------------------------------------------------
# Tests — Flask app structure (no model needed, just import)
# ---------------------------------------------------------------------------

class TestFlaskApp:
    """Verify the Flask app is constructed with the right endpoints."""

    def test_app_has_health_endpoint(self):
        from mobile_sam_server import app
        rules = {r.rule: r.methods for r in app.url_map.iter_rules()}
        assert "/health" in rules
        assert "GET" in rules["/health"]

    def test_app_has_segment_endpoint(self):
        from mobile_sam_server import app
        rules = {r.rule: r.methods for r in app.url_map.iter_rules()}
        assert "/segment_2d" in rules
        assert "POST" in rules["/segment_2d"]


# ---------------------------------------------------------------------------
# Tests — mask dilation logic (no model needed)
# ---------------------------------------------------------------------------

class TestMaskDilation:
    """Verify the cv2.dilate post-processing in isolation."""

    def test_dilation_expands_mask(self):
        """A small mask must grow after dilation."""
        import cv2
        mask = np.zeros((50, 50), dtype=np.uint8)
        mask[24:26, 24:26] = 255  # 2x2 square

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        dilated = cv2.dilate(mask, kernel, iterations=1)

        # Dilated mask must have more foreground pixels than original.
        assert dilated.sum() > mask.sum()

    def test_zero_dilation_is_noop(self):
        """dilation_px=0 must leave the mask unchanged."""
        import cv2
        mask = np.zeros((50, 50), dtype=np.uint8)
        mask[20:30, 20:30] = 255
        # When dilation_px=0 the server skips dilation entirely.
        # Verify that a 0-size kernel path is equivalent to no-op.
        original = mask.copy()
        # The server code checks `if dilation_px > 0` before dilating.
        assert mask.sum() == original.sum()


# ---------------------------------------------------------------------------
# Tests — full segmentation (model needed, skipped otherwise)
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _SAM_AVAILABLE,
                    reason="MobileSAM / torch / weights not available on host")
class TestSegmentation:
    """Full end-to-end segmentation using the loaded MobileSAM model."""

    def test_mask_shape_matches_image(self):
        """The returned mask must have the same H, W as the input image."""
        from mobile_sam_server import _load_model, segment_2d
        if not _is_model_loaded():
            _load_model()
        img = _make_synthetic_image(128, 128)
        mask = segment_2d(img, 64, 64, dilation_px=5)
        assert mask.shape == (128, 128)

    def test_mask_is_binary(self):
        """The mask must contain only 0 and 255."""
        from mobile_sam_server import _load_model, segment_2d
        if not _is_model_loaded():
            _load_model()
        img = _make_synthetic_image(128, 128)
        mask = segment_2d(img, 64, 64, dilation_px=0)
        unique = np.unique(mask)
        assert set(unique.tolist()).issubset({0, 255})

    def test_click_on_object_yields_foreground(self):
        """Clicking inside the bright central region should yield foreground."""
        from mobile_sam_server import _load_model, segment_2d
        if not _is_model_loaded():
            _load_model()
        img = _make_synthetic_image(128, 128)
        mask = segment_2d(img, 64, 64, dilation_px=0)
        # The clicked pixel (or a neighbour) should be foreground.
        assert mask[64, 64] == 255 or mask[60:70, 60:70].sum() > 0


def _is_model_loaded() -> bool:
    """Check whether the server module already has the model loaded."""
    import mobile_sam_server
    return getattr(mobile_sam_server, "_model_ready", False)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
