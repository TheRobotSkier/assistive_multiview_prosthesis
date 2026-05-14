#!/usr/bin/env python3
"""IMU-only dead reckoning node for RealSense cameras.

Subscribes to IMU topics, runs a per-camera CALIBRATING→TRACKING state
machine, integrates gyro+accel, and publishes TF imu_test_world→camN_imu.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


# ── Pure numpy quaternion helpers ──────────────────────────────────────────

def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of two quaternions in [x, y, z, w] form."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ], dtype=float)


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion; return identity if near-zero norm."""
    n = np.linalg.norm(q)
    if n < 1e-10:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def rotate_vec(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate 3-vector v by unit quaternion q (active rotation, [x,y,z,w])."""
    x, y, z, w = q
    R = np.array([
        [1.0 - 2*(y*y + z*z),  2*(x*y - w*z),       2*(x*z + w*y)],
        [2*(x*y + w*z),        1.0 - 2*(x*x + z*z),  2*(y*z - w*x)],
        [2*(x*z - w*y),        2*(y*z + w*x),         1.0 - 2*(x*x + y*y)],
    ], dtype=float)
    return R @ np.asarray(v, dtype=float)


def integrate_gyro(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
    """First-order body-frame gyro integration.

    Rotates q by the angular displacement omega*dt expressed in body frame.
    Returns normalized result.
    """
    angle = np.linalg.norm(omega) * dt
    if angle < 1e-10:
        return q
    axis = omega / np.linalg.norm(omega)
    s = math.sin(angle / 2.0)
    dq = np.array([axis[0]*s, axis[1]*s, axis[2]*s, math.cos(angle / 2.0)], dtype=float)
    return quat_normalize(quat_mult(q, dq))
