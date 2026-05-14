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


# ── Per-camera state ────────────────────────────────────────────────────────

class CalibState(Enum):
    CALIBRATING = "CALIBRATING"
    TRACKING = "TRACKING"


@dataclass
class CameraState:
    name: str
    imu_topic: str
    calib_duration: float

    state: CalibState = CalibState.CALIBRATING
    accel_samples: list = field(default_factory=list)
    gyro_samples: list = field(default_factory=list)
    calib_start_sec: float = -1.0

    # Calibration results
    g_world: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))

    # Integration state
    q: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    v: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    p: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    last_stamp_sec: float = -1.0


def finish_calibration(state: CameraState, logger) -> None:
    """Compute g_world and gyro_bias from collected samples; switch to TRACKING."""
    if not state.accel_samples or not state.gyro_samples:
        logger.warning(f"[{state.name}] Calibration ended with no samples — staying in CALIBRATING")
        return
    state.g_world = np.mean(state.accel_samples, axis=0)
    state.gyro_bias = np.mean(state.gyro_samples, axis=0)
    state.state = CalibState.TRACKING
    g_mag = float(np.linalg.norm(state.g_world))
    logger.info(
        f"[{state.name}] Calibration done: "
        f"g_world={state.g_world.tolist()}, "
        f"gyro_bias={state.gyro_bias.tolist()}, "
        f"|g|={g_mag:.3f} m/s² (expected ~9.81)"
    )


def integration_step(state: CameraState, accel: np.ndarray, gyro: np.ndarray,
                     stamp_sec: float) -> None:
    """One IMU integration step. Mutates state in-place."""
    if state.last_stamp_sec < 0:
        state.last_stamp_sec = stamp_sec
        return
    dt = stamp_sec - state.last_stamp_sec
    state.last_stamp_sec = stamp_sec
    if dt <= 0.0 or dt > 0.5:
        return

    omega = gyro - state.gyro_bias
    state.q = integrate_gyro(state.q, omega, dt)

    a_world = rotate_vec(state.q, accel)
    a_lin = a_world - state.g_world

    state.v += a_lin * dt
    state.p += state.v * dt
