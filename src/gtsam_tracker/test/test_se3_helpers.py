#!/usr/bin/env python3
"""Unit tests for se3_helpers — SE(3) math utilities.

The core math tests (quaternion, inverse, compose) run on the host with just
numpy.  The GTSAM round-trip tests are marked ``@pytest.mark.gtsam`` and will
be skipped if ``gtsam`` is not importable (run them in the container).

Usage:
    python3 -m pytest src/gtsam_tracker/test/test_se3_helpers.py -v
"""

import math

import numpy as np
import pytest

from gtsam_tracker.se3_helpers import (
    pose_to_matrix,
    matrix_to_pose,
    pose3_to_matrix,
    matrix_to_pose3,
    inverse_se3,
    compose_se3,
    relative_transform,
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    angle_between_quaternions,
)
from gtsam_tracker.se3_helpers import _SimplePose


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_pose(tx, ty, tz, qx, qy, qz, qw):
    """Create a _SimplePose (duck-typed geometry_msgs/Pose)."""
    p = _SimplePose()
    p.position = _SimplePose._Point(tx, ty, tz)
    p.orientation = _SimplePose._Quaternion(qx, qy, qz, qw)
    return p


def _random_rotation(rng):
    """Generate a random rotation matrix via QR decomposition of a random matrix."""
    A = rng.standard_normal((3, 3))
    Q, R = np.linalg.qr(A)
    # Ensure proper rotation (det = +1)
    Q = Q @ np.diag(np.sign(np.diag(R)))
    if np.linalg.det(Q) < 0:
        Q[:, 0] *= -1
    return Q


# ---------------------------------------------------------------------------
# Quaternion ↔ rotation matrix round-trip
# ---------------------------------------------------------------------------

class TestQuaternionRotation:
    def test_identity_quaternion(self):
        q = np.array([0.0, 0.0, 0.0, 1.0])
        R = quaternion_to_rotation_matrix(q)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_roundtrip(self):
        rng = np.random.default_rng(123)
        for _ in range(50):
            R = _random_rotation(rng)
            q = rotation_matrix_to_quaternion(R)
            R2 = quaternion_to_rotation_matrix(q)
            np.testing.assert_allclose(R2, R, atol=1e-9)

    def test_90deg_z_rotation(self):
        """Rotation by 90° about z: q = (0, 0, sin45, cos45)."""
        angle = math.pi / 2
        q = np.array([0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])
        R = quaternion_to_rotation_matrix(q)
        expected = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        np.testing.assert_allclose(R, expected, atol=1e-12)


# ---------------------------------------------------------------------------
# Pose message ↔ matrix round-trip
# ---------------------------------------------------------------------------

class TestPoseMatrixRoundTrip:
    def test_identity(self):
        pose = _make_pose(0, 0, 0, 0, 0, 0, 1)
        T = pose_to_matrix(pose)
        np.testing.assert_allclose(T, np.eye(4), atol=1e-12)

    def test_roundtrip(self):
        """matrix → pose → matrix, verify identity within 1e-9."""
        rng = np.random.default_rng(42)
        for _ in range(20):
            R = _random_rotation(rng)
            t = rng.uniform(-5, 5, 3)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t

            trans, quat = matrix_to_pose(T)
            pose = _make_pose(*trans, *quat)
            T2 = pose_to_matrix(pose)

            np.testing.assert_allclose(T2, T, atol=1e-9)

    def test_translation(self):
        pose = _make_pose(1.0, 2.0, 3.0, 0, 0, 0, 1)
        T = pose_to_matrix(pose)
        np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, 3.0], atol=1e-12)


# ---------------------------------------------------------------------------
# SE(3) inverse
# ---------------------------------------------------------------------------

class TestInverseSE3:
    def test_identity(self):
        T = np.eye(4)
        np.testing.assert_allclose(inverse_se3(T), np.eye(4), atol=1e-12)

    def test_inverse_times_original_is_identity(self):
        """T @ inv(T) ≈ I."""
        rng = np.random.default_rng(7)
        for _ in range(20):
            R = _random_rotation(rng)
            t = rng.uniform(-3, 3, 3)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t

            T_inv = inverse_se3(T)
            product = T @ T_inv
            np.testing.assert_allclose(product, np.eye(4), atol=1e-10)

    def test_compare_with_np_inv(self):
        """Our efficient inverse should match np.linalg.inv."""
        rng = np.random.default_rng(99)
        for _ in range(10):
            R = _random_rotation(rng)
            t = rng.uniform(-2, 2, 3)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t
            np.testing.assert_allclose(inverse_se3(T), np.linalg.inv(T), atol=1e-10)


# ---------------------------------------------------------------------------
# SE(3) compose
# ---------------------------------------------------------------------------

class TestComposeSE3:
    def test_compose_equals_matmul(self):
        """verify compose(A, B) == A @ B."""
        rng = np.random.default_rng(55)
        for _ in range(20):
            R1 = _random_rotation(rng)
            t1 = rng.uniform(-2, 2, 3)
            T1 = np.eye(4)
            T1[:3, :3] = R1
            T1[:3, 3] = t1

            R2 = _random_rotation(rng)
            t2 = rng.uniform(-2, 2, 3)
            T2 = np.eye(4)
            T2[:3, :3] = R2
            T2[:3, 3] = t2

            result = compose_se3(T1, T2)
            np.testing.assert_allclose(result, T1 @ T2, atol=1e-12)

    def test_compose_with_identity(self):
        rng = np.random.default_rng(33)
        R = _random_rotation(rng)
        t = rng.uniform(-3, 3, 3)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t

        np.testing.assert_allclose(compose_se3(T, np.eye(4)), T, atol=1e-12)
        np.testing.assert_allclose(compose_se3(np.eye(4), T), T, atol=1e-12)


# ---------------------------------------------------------------------------
# Relative transform
# ---------------------------------------------------------------------------

class TestRelativeTransform:
    def test_identity_relative(self):
        rng = np.random.default_rng(11)
        R = _random_rotation(rng)
        t = rng.uniform(-2, 2, 3)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t

        rel = relative_transform(T, T)
        np.testing.assert_allclose(rel, np.eye(4), atol=1e-10)

    def test_relative_from_origin(self):
        """relative_transform(I, T) should equal T."""
        rng = np.random.default_rng(22)
        R = _random_rotation(rng)
        t = rng.uniform(-2, 2, 3)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t

        rel = relative_transform(np.eye(4), T)
        np.testing.assert_allclose(rel, T, atol=1e-10)


# ---------------------------------------------------------------------------
# Angle between quaternions
# ---------------------------------------------------------------------------

class TestAngleBetweenQuaternions:
    def test_identical_quaternions(self):
        q = np.array([0.0, 0.0, 0.0, 1.0])
        assert angle_between_quaternions(q, q) < 1e-10

    def test_known_30deg(self):
        """Known 30° rotation → angle_between returns ~0.524 rad."""
        angle = math.radians(30)
        q1 = np.array([0.0, 0.0, 0.0, 1.0])  # identity
        q2 = np.array([0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])
        result = angle_between_quaternions(q1, q2)
        expected = math.radians(30)
        assert abs(result - expected) < 1e-6, f"Expected {expected}, got {result}"

    def test_90deg(self):
        angle = math.radians(90)
        q1 = np.array([0.0, 0.0, 0.0, 1.0])
        q2 = np.array([0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])
        result = angle_between_quaternions(q1, q2)
        expected = math.radians(90)
        assert abs(result - expected) < 1e-6

    def test_double_cover(self):
        """q and -q represent the same rotation → angle should be 0."""
        q = np.array([0.1, 0.2, 0.3, math.sqrt(1 - 0.01 - 0.04 - 0.09)])
        result = angle_between_quaternions(q, -q)
        assert result < 1e-10

    def test_180deg(self):
        angle = math.radians(180)
        q1 = np.array([0.0, 0.0, 0.0, 1.0])
        q2 = np.array([0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])
        result = angle_between_quaternions(q1, q2)
        assert abs(result - math.pi) < 1e-6


# ---------------------------------------------------------------------------
# GTSAM round-trip (requires gtsam — run in container)
# ---------------------------------------------------------------------------

try:
    import gtsam
    HAS_GTSAM = True
except ImportError:
    HAS_GTSAM = False


@pytest.mark.skipif(not HAS_GTSAM, reason="gtsam not available")
class TestGTSAMRoundTrip:
    def test_matrix_to_pose3_to_matrix(self):
        """numpy → Pose3 → numpy, verify identity."""
        rng = np.random.default_rng(77)
        for _ in range(20):
            R = _random_rotation(rng)
            t = rng.uniform(-3, 3, 3)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t

            pose3 = matrix_to_pose3(T)
            T2 = pose3_to_matrix(pose3)
            np.testing.assert_allclose(T2, T, atol=1e-9)

    def test_pose3_identity(self):
        pose3 = matrix_to_pose3(np.eye(4))
        np.testing.assert_allclose(pose3_to_matrix(pose3), np.eye(4), atol=1e-12)
