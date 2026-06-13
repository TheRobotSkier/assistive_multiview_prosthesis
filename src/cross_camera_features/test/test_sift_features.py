#!/usr/bin/env python3
"""Unit tests for cross_camera_features — SIFT matching + Umeyama alignment.

Pure numpy + OpenCV, runs on the host (no ROS needed).

Usage:
    python3 -m pytest src/cross_camera_features/test/test_sift_features.py -v
"""

from __future__ import annotations

import math

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from cross_camera_features.sift_feature_node import (  # noqa: E402
    match_and_align,
    match_descriptors,
    extract_sift,
    AlignResult,
)
from gtsam_tracker.umeyama import umeyama  # noqa: E402


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def sift():
    return cv2.SIFT_create()


@pytest.fixture
def matcher():
    return cv2.BFMatcher(cv2.NORM_L2)


@pytest.fixture
def intrinsics():
    fx = fy = 500.0
    cx, cy = 160.0, 120.0
    return np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ])


@pytest.fixture
def identity_pose():
    return np.eye(4)


def make_textured_image(width=320, height=240, seed=0):
    """Generate a synthetic textured image with rich SIFT features."""
    rng = np.random.default_rng(seed)
    img = np.zeros((height, width, 3), dtype=np.uint8)
    # Random rectangles + circles for texture.
    for _ in range(40):
        x0 = rng.integers(0, width - 30)
        y0 = rng.integers(0, height - 30)
        w = rng.integers(10, 40)
        h = rng.integers(10, 40)
        color = tuple(int(c) for c in rng.integers(0, 256, 3))
        cv2.rectangle(img, (int(x0), int(y0)),
                      (int(x0 + w), int(y0 + h)), color, -1)
    for _ in range(20):
        cx = rng.integers(0, width)
        cy = rng.integers(0, height)
        r = rng.integers(5, 20)
        color = tuple(int(c) for c in rng.integers(0, 256, 3))
        cv2.circle(img, (int(cx), int(cy)), int(r), color, -1)
    # Add noise.
    noise = rng.integers(-20, 20, img.shape, dtype=np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    return img


def apply_homography(img, H):
    """Warp an image with a 3×3 homography."""
    h, w = img.shape[:2]
    return cv2.warpPerspective(img, H, (w, h))


# ---------------------------------------------------------------------------
# SIFT extraction
# ---------------------------------------------------------------------------

class TestSiftExtraction:
    def test_extract_returns_keypoints(self, sift):
        img = make_textured_image()
        kp, des = extract_sift(sift, img)
        assert len(kp) > 0
        assert des is not None
        assert des.shape[0] == len(kp)
        assert des.shape[1] == 128

    def test_greyscale_input(self, sift):
        """extract_sift should accept a greyscale image too."""
        img = make_textured_image()
        grey = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        kp, des = extract_sift(sift, grey)
        assert len(kp) > 0


# ---------------------------------------------------------------------------
# Descriptor matching
# ---------------------------------------------------------------------------

class TestDescriptorMatching:
    def test_match_count(self, sift, matcher):
        """Two views of the same textured image should produce matches."""
        img = make_textured_image(seed=1)
        # Small rotation as a second viewpoint.
        M = cv2.getRotationMatrix2D((160, 120), 5, 1.0)
        img2 = cv2.warpAffine(img, M, (320, 240))

        _, des1 = extract_sift(sift, img)
        _, des2 = extract_sift(sift, img2)

        matches = match_descriptors(matcher, des1, des2, top_k=100)
        assert len(matches) > 0
        assert len(matches) <= 100

    def test_empty_descriptors(self, matcher):
        """Empty descriptors → no matches."""
        matches = match_descriptors(
            matcher, np.zeros((0, 128), dtype=np.float32),
            np.zeros((10, 128), dtype=np.float32))
        assert len(matches) == 0

    def test_none_descriptors(self, matcher):
        matches = match_descriptors(matcher, None, None)
        assert len(matches) == 0


# ---------------------------------------------------------------------------
# Synthetic image-pair alignment test
# ---------------------------------------------------------------------------

class TestSyntheticImagePair:
    def test_match_and_align_organized(self, sift, matcher, intrinsics,
                                       identity_pose):
        """Two organised clouds + matching images → Umeyama recovers ~identity.

        We build two organised clouds that are identical (same world points)
        and two images that are the same texture, so the alignment should be
        near-identity.
        """
        H, W = 240, 320
        img = make_textured_image(seed=2)

        # Organised cloud: every pixel has a 3-D point at z = 1.0.
        fx, fy = intrinsics[0, 0], intrinsics[1, 1]
        cx, cy = intrinsics[0, 2], intrinsics[1, 2]
        uu, vv = np.meshgrid(np.arange(W), np.arange(H))
        z = np.full((H, W), 1.0, dtype=np.float32)
        x = ((uu - cx) * z / fx).astype(np.float32)
        y = ((vv - cy) * z / fy).astype(np.float32)
        cloud = np.stack([x, y, z], axis=-1)  # (H, W, 3)

        result = match_and_align(
            sift, matcher,
            img, img.copy(),
            cloud, cloud.copy(),
            head_organized=True, arm_organized=True,
            head_K=intrinsics, arm_K=intrinsics,
            head_pose=identity_pose, arm_pose=identity_pose,
            min_matches=5,
        )

        # Should find matches and align.
        assert result.num_matches > 0
        assert result.T is not None
        # Same cloud → near-identity transform.
        np.testing.assert_allclose(result.T[:3, :3], np.eye(3), atol=0.1)
        np.testing.assert_allclose(result.T[:3, 3], [0, 0, 0], atol=0.1)

    def test_match_and_align_unorganized(self, sift, matcher, intrinsics):
        """Unorganised clouds: depth lookup via ray-projection should work."""
        H, W = 240, 320
        img = make_textured_image(seed=3)

        # Build an unorganised cloud: points at z=1.0 across the image.
        fx, fy = intrinsics[0, 0], intrinsics[1, 1]
        cx, cy = intrinsics[0, 2], intrinsics[1, 2]
        uu, vv = np.meshgrid(np.arange(W), np.arange(H))
        z = np.full((H * W,), 1.0, dtype=np.float32)
        x = ((uu.flatten() - cx) * z / fx).astype(np.float32)
        y = ((vv.flatten() - cy) * z / fy).astype(np.float32)
        cloud = np.column_stack([x, y, z])  # (N, 3)

        pose = np.eye(4)

        result = match_and_align(
            sift, matcher,
            img, img.copy(),
            cloud, cloud.copy(),
            head_organized=False, arm_organized=False,
            head_K=intrinsics, arm_K=intrinsics,
            head_pose=pose, arm_pose=pose,
            min_matches=5,
            depth_search_radius_m=0.05,
        )

        # Unorganised depth lookup is approximate but should still find some.
        assert result.num_matches > 0
        assert result.T is not None


# ---------------------------------------------------------------------------
# Umeyama integration test
# ---------------------------------------------------------------------------

class TestUmeyamaIntegration:
    def test_known_transform_recovery(self):
        """Known 3-D point pairs with a known rigid transform + noise →
        umeyama recovers it within tolerance."""
        rng = np.random.default_rng(42)

        angle = math.radians(20)
        R_true = np.array([
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle), 0],
            [0, 0, 1],
        ])
        t_true = np.array([0.1, -0.05, 0.03])

        src = rng.uniform(-0.1, 0.1, (30, 3))
        dst = (R_true @ src.T).T + t_true
        dst += rng.normal(0, 0.002, dst.shape)  # 2mm noise

        T, cov = umeyama(src, dst)

        np.testing.assert_allclose(T[:3, :3], R_true, atol=1e-2)
        np.testing.assert_allclose(T[:3, 3], t_true, atol=5e-3)
        assert cov.shape == (6, 6)


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------

class TestEdgeCases:
    def test_no_matches_no_publish(self, sift, matcher, intrinsics,
                                   identity_pose):
        """Two blank images (no features) → no matches, T is None."""
        H, W = 240, 320
        blank = np.zeros((H, W, 3), dtype=np.uint8)
        cloud = np.zeros((0, 3), dtype=np.float32)

        result = match_and_align(
            sift, matcher,
            blank, blank.copy(),
            cloud, cloud.copy(),
            head_organized=False, arm_organized=False,
            head_K=intrinsics, arm_K=intrinsics,
            head_pose=identity_pose, arm_pose=identity_pose,
            min_matches=5,
        )
        assert result.T is None
        assert result.num_matches == 0

    def test_all_depth_lookups_fail(self, sift, matcher, intrinsics):
        """All depth lookups fail (points behind camera) → no publish."""
        H, W = 240, 320
        img = make_textured_image(seed=5)
        # Cloud with points far off-axis → ray lookup fails.
        cloud = np.array([
            [100.0, 100.0, 1.0],
            [-100.0, -100.0, 1.0],
        ])
        pose = np.eye(4)

        result = match_and_align(
            sift, matcher,
            img, img.copy(),
            cloud, cloud.copy(),
            head_organized=False, arm_organized=False,
            head_K=intrinsics, arm_K=intrinsics,
            head_pose=pose, arm_pose=pose,
            min_matches=5,
            depth_search_radius_m=0.001,  # tiny radius → all fail
        )
        # Either no matches found, or T is None.
        assert result.T is None

    def test_below_min_matches(self, sift, matcher, intrinsics,
                               identity_pose):
        """Fewer valid matches than min_matches → no publish."""
        H, W = 240, 320
        img = make_textured_image(seed=6)
        # Cloud with only 2 valid points.
        cloud = np.array([
            [0.0, 0.0, 1.0],
            [0.1, 0.1, 1.0],
        ])

        result = match_and_align(
            sift, matcher,
            img, img.copy(),
            cloud, cloud.copy(),
            head_organized=False, arm_organized=False,
            head_K=intrinsics, arm_K=intrinsics,
            head_pose=identity_pose, arm_pose=identity_pose,
            min_matches=50,  # impossibly high
            depth_search_radius_m=0.05,
        )
        assert result.T is None
        assert result.num_matches < 50

    def test_align_result_repr(self):
        r = AlignResult(None, None, 3, 5, "test message")
        assert "matches=3" in repr(r)
        assert r.num_matches == 3
        assert r.num_raw_matches == 5
        assert r.message == "test message"
