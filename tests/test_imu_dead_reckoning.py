"""Unit tests for pure-numpy IMU math helpers.

Run with: python -m pytest tests/test_imu_dead_reckoning.py -v
No ROS2 installation required.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

# Import helpers without triggering ROS2 imports by loading only the
# pure-numpy section (the file has no module-level rclpy import yet).
sys.path.insert(0, str(Path(__file__).parent.parent /
                       "src/sensor_fusion_bringup/scripts"))
from imu_dead_reckoning_node import (
    integrate_gyro,
    quat_mult,
    quat_normalize,
    rotate_vec,
)


def _identity():
    return np.array([0.0, 0.0, 0.0, 1.0])


def test_quat_normalize_unit():
    q = np.array([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(quat_normalize(q), q)


def test_quat_normalize_scales():
    q = np.array([2.0, 0.0, 0.0, 0.0])
    assert np.allclose(quat_normalize(q), [1.0, 0.0, 0.0, 0.0])


def test_quat_normalize_zero_returns_identity():
    q = np.array([0.0, 0.0, 0.0, 0.0])
    assert np.allclose(quat_normalize(q), [0.0, 0.0, 0.0, 1.0])


def test_quat_mult_identity():
    q = np.array([0.1, 0.2, 0.3, math.sqrt(1 - 0.01 - 0.04 - 0.09)])
    q = quat_normalize(q)
    result = quat_mult(q, _identity())
    assert np.allclose(result, q, atol=1e-9)


def test_quat_mult_180_z_twice_is_identity():
    # 180° around Z: [0, 0, 1, 0]
    q180z = np.array([0.0, 0.0, 1.0, 0.0])
    result = quat_mult(q180z, q180z)
    assert np.allclose(np.abs(result), [0.0, 0.0, 0.0, 1.0], atol=1e-9)


def test_rotate_vec_identity_unchanged():
    v = np.array([1.0, 2.0, 3.0])
    assert np.allclose(rotate_vec(_identity(), v), v)


def test_rotate_vec_90_around_z():
    # 90° around Z rotates [1,0,0] → [0,1,0]
    q = np.array([0.0, 0.0, math.sin(math.pi/4), math.cos(math.pi/4)])
    result = rotate_vec(q, np.array([1.0, 0.0, 0.0]))
    assert np.allclose(result, [0.0, 1.0, 0.0], atol=1e-9)


def test_rotate_vec_90_around_x():
    # 90° around X rotates [0,1,0] → [0,0,1]
    q = np.array([math.sin(math.pi/4), 0.0, 0.0, math.cos(math.pi/4)])
    result = rotate_vec(q, np.array([0.0, 1.0, 0.0]))
    assert np.allclose(result, [0.0, 0.0, 1.0], atol=1e-9)


def test_integrate_gyro_identity_zero_rate():
    q = _identity()
    result = integrate_gyro(q, np.zeros(3), 0.01)
    assert np.allclose(result, q)


def test_integrate_gyro_90_around_z_in_four_steps():
    q = _identity()
    omega = np.array([0.0, 0.0, math.pi / 2.0])  # 90°/s around Z
    for _ in range(4):
        q = integrate_gyro(q, omega, 0.25)  # 4 × 0.25 s = 1 s → 90°
    # After 90° around Z, [1,0,0] should map to [0,1,0]
    v = rotate_vec(q, np.array([1.0, 0.0, 0.0]))
    assert np.allclose(v, [0.0, 1.0, 0.0], atol=1e-6)


def test_integrate_gyro_output_is_unit():
    q = _identity()
    omega = np.array([1.0, 2.0, 3.0])
    result = integrate_gyro(q, omega, 0.01)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-9
