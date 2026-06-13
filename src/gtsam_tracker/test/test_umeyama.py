#!/usr/bin/env python3
"""Unit tests for umeyama — SVD-based 3-D to 3-D alignment.

Pure numpy, runs on host (no GTSAM needed).

Usage:
    python3 -m pytest src/gtsam_tracker/test/test_umeyama.py -v
"""

import math

import numpy as np
import pytest

from gtsam_tracker.umeyama import umeyama


def _random_rotation(rng):
    """Generate a random rotation matrix via QR decomposition."""
    A = rng.standard_normal((3, 3))
    Q, R = np.linalg.qr(A)
    Q = Q @ np.diag(np.sign(np.diag(R)))
    if np.linalg.det(Q) < 0:
        Q[:, 0] *= -1
    return Q


class TestUmeyama:
    def test_identity(self):
        """Same source and destination → identity transform."""
        rng = np.random.default_rng(42)
        pts = rng.uniform(-1, 1, (20, 3))
        T, cov = umeyama(pts, pts.copy())
        np.testing.assert_allclose(T, np.eye(4), atol=1e-9)

    def test_known_transform_recovery(self):
        """Generate known R, t, apply to random points + small noise, verify
        recovery within tolerance."""
        rng = np.random.default_rng(123)

        # Known transform
        angle = math.radians(35)
        R_true = np.array([
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle), 0],
            [0, 0, 1],
        ])
        t_true = np.array([0.5, -0.3, 0.2])

        # Source points
        N = 50
        src = rng.uniform(-1, 1, (N, 3))

        # Apply transform
        dst = (R_true @ src.T).T + t_true

        # Add small noise
        noise = rng.normal(0, 0.001, dst.shape)
        dst_noisy = dst + noise

        T_est, cov = umeyama(src, dst_noisy)

        # Verify rotation recovery
        R_est = T_est[:3, :3]
        np.testing.assert_allclose(R_est, R_true, atol=1e-2)

        # Verify translation recovery
        t_est = T_est[:3, 3]
        np.testing.assert_allclose(t_est, t_true, atol=5e-3)

    def test_translation_only(self):
        """Pure translation, no rotation."""
        rng = np.random.default_rng(7)
        src = rng.uniform(-2, 2, (30, 3))
        t_true = np.array([1.0, 2.0, 3.0])
        dst = src + t_true

        T, _ = umeyama(src, dst)
        np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-9)
        np.testing.assert_allclose(T[:3, 3], t_true, atol=1e-9)

    def test_rotation_only(self):
        """Pure rotation about origin, no translation."""
        rng = np.random.default_rng(99)
        angle = math.radians(60)
        R_true = np.array([
            [1, 0, 0],
            [0, math.cos(angle), -math.sin(angle)],
            [0, math.sin(angle), math.cos(angle)],
        ])

        src = rng.uniform(-1, 1, (40, 3))
        dst = (R_true @ src.T).T

        T, _ = umeyama(src, dst)
        np.testing.assert_allclose(T[:3, :3], R_true, atol=1e-9)
        np.testing.assert_allclose(T[:3, 3], [0, 0, 0], atol=1e-9)

    def test_reflection_produces_proper_rotation(self):
        """When the optimal alignment would be a reflection (det < 0), the
        algorithm should still return a *proper* rotation (det = +1) that is
        the closest SO(3) approximation."""
        rng = np.random.default_rng(55)
        src = rng.uniform(-1, 1, (20, 3))
        # Create a reflection (improper rotation, det = -1)
        R_reflect = np.diag([1.0, 1.0, -1.0])
        dst = (R_reflect @ src.T).T

        T, _ = umeyama(src, dst)
        R_est = T[:3, :3]

        # The result must be a proper rotation (det ≈ +1), not a reflection.
        assert np.linalg.det(R_est) > 0.99, (
            f"Expected proper rotation (det≈1), got det={np.linalg.det(R_est):.4f}"
        )

    def test_covariance_shape(self):
        """The returned covariance should be (6, 6)."""
        rng = np.random.default_rng(33)
        src = rng.uniform(-1, 1, (10, 3))
        dst = src + np.array([0.1, 0.2, 0.3])
        T, cov = umeyama(src, dst)
        assert cov.shape == (6, 6)

    def test_too_few_points(self):
        """Fewer than 3 points should raise ValueError."""
        src = np.array([[0.0, 0.0, 0.0]])
        dst = np.array([[1.0, 0.0, 0.0]])
        with pytest.raises(ValueError):
            umeyama(src, dst)

    def test_shape_mismatch(self):
        """Mismatched shapes should raise ValueError."""
        src = np.zeros((5, 3))
        dst = np.zeros((4, 3))
        with pytest.raises(ValueError):
            umeyama(src, dst)

    def test_perfect_alignment(self):
        """Perfect alignment (no noise) should give near-zero covariance."""
        rng = np.random.default_rng(77)
        angle = math.radians(45)
        R = np.array([
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle), 0],
            [0, 0, 1],
        ])
        t = np.array([0.3, 0.4, 0.5])
        src = rng.uniform(-1, 1, (30, 3))
        dst = (R @ src.T).T + t

        T, cov = umeyama(src, dst)
        # Covariance should be near-zero for perfect alignment
        assert np.max(np.abs(cov)) < 1e-12
