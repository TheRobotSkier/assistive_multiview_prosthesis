"""se3_helpers — SE(3) math utilities (pure numpy + optional gtsam).

ZERO ROS imports.  Only ``numpy``, ``scipy`` (for quaternion utilities) and
``gtsam`` (for Pose3 conversion) are used.

Conventions
-----------
- Homogeneous matrices ``T`` are ``(4, 4)`` with the block form::

      T = [[R, t],
           [0, 1]]

  where ``R`` is ``(3, 3)`` rotation and ``t`` is ``(3,)`` translation.

- Quaternions are ``(x, y, z, w)`` — the ROS / Hamilton convention.

- ``pose`` in ``pose_to_matrix`` / ``matrix_to_pose`` accepts/returns a duck-typed
  object with ``.position`` (``.x, .y, .z``) and ``.orientation``
  (``.x, .y, .z, .w``).  This matches ``geometry_msgs/Pose`` but does not
  require the ROS import.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

__all__ = [
    "pose_to_matrix",
    "matrix_to_pose",
    "pose3_to_matrix",
    "matrix_to_pose3",
    "inverse_se3",
    "compose_se3",
    "relative_transform",
    "quaternion_to_rotation_matrix",
    "rotation_matrix_to_quaternion",
    "angle_between_quaternions",
]


# ---------------------------------------------------------------------------
# Quaternion ↔ rotation matrix
# ---------------------------------------------------------------------------

def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert a quaternion ``(x, y, z, w)`` to a ``(3, 3)`` rotation matrix.

    Uses the standard Hamilton-product-derived formula.
    """
    q = np.asarray(q, dtype=np.float64).reshape(4)
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-15:
        return np.eye(3)

    s = 2.0 / n
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs

    R = np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])
    return R


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert a ``(3, 3)`` rotation matrix to a quaternion ``(x, y, z, w)``.

    Uses Shepperd's method — numerically stable for all rotation angles.
    """
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    m11, m12, m13 = R[0, 0], R[0, 1], R[0, 2]
    m21, m22, m23 = R[1, 0], R[1, 1], R[1, 2]
    m31, m32, m33 = R[2, 0], R[2, 1], R[2, 2]

    trace = m11 + m22 + m33

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m32 - m23) * s
        y = (m13 - m31) * s
        z = (m21 - m12) * s
    elif m11 > m22 and m11 > m33:
        s = 2.0 * np.sqrt(1.0 + m11 - m22 - m33)
        w = (m32 - m23) / s
        x = 0.25 * s
        y = (m12 + m21) / s
        z = (m13 + m31) / s
    elif m22 > m33:
        s = 2.0 * np.sqrt(1.0 + m22 - m11 - m33)
        w = (m13 - m31) / s
        x = (m12 + m21) / s
        y = 0.25 * s
        z = (m23 + m32) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m33 - m11 - m22)
        w = (m21 - m12) / s
        x = (m13 + m31) / s
        y = (m23 + m32) / s
        z = 0.25 * s

    q = np.array([x, y, z, w])
    # Normalise to unit length
    q /= np.linalg.norm(q)
    return q


# ---------------------------------------------------------------------------
# Pose message ↔ matrix (duck-typed, no ROS import)
# ---------------------------------------------------------------------------

class _SimplePose:
    """Lightweight stand-in for ``geometry_msgs.msg.Pose``.

    Used by :func:`matrix_to_pose` so that tests can run without ROS.
    The returned object has the same attribute structure as ``Pose``.
    """

    class _Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)

    class _Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)
            self.w = float(w)

    def __init__(self):
        self.position = self._Point()
        self.orientation = self._Quaternion()


def pose_to_matrix(pose_msg) -> np.ndarray:
    """Convert a ``geometry_msgs/Pose``-like object to a ``(4, 4)`` matrix.

    Parameters
    ----------
    pose_msg :
        Object with ``.position`` (``.x, .y, .z``) and ``.orientation``
        (``.x, .y, .z, .w``).
    """
    p = pose_msg.position
    q = pose_msg.orientation
    R = quaternion_to_rotation_matrix(np.array([q.x, q.y, q.z, q.w]))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def matrix_to_pose(T: np.ndarray) -> Tuple[tuple, tuple]:
    """Convert a ``(4, 4)`` matrix to ``(translation_tuple, quaternion_tuple)``.

    Returns
    -------
    (translation, quaternion) : tuple of tuples
        ``translation`` is ``(x, y, z)``, ``quaternion`` is ``(x, y, z, w)``.
    """
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    t = (float(T[0, 3]), float(T[1, 3]), float(T[2, 3]))
    q = rotation_matrix_to_quaternion(T[:3, :3])
    return t, (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


# ---------------------------------------------------------------------------
# GTSAM Pose3 ↔ numpy matrix
# ---------------------------------------------------------------------------

def pose3_to_matrix(pose3) -> np.ndarray:
    """Convert a ``gtsam.Pose3`` to a ``(4, 4)`` numpy matrix."""
    T = np.eye(4)
    T[:3, :3] = pose3.rotation().matrix()
    T[:3, 3] = pose3.translation()
    return T


def matrix_to_pose3(T: np.ndarray):
    """Convert a ``(4, 4)`` numpy matrix to a ``gtsam.Pose3``."""
    import gtsam

    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    R = gtsam.Rot3(T[:3, :3])
    t = gtsam.Point3(T[:3, 3])
    return gtsam.Pose3(R, t)


# ---------------------------------------------------------------------------
# SE(3) algebra
# ---------------------------------------------------------------------------

def inverse_se3(T: np.ndarray) -> np.ndarray:
    """Efficient SE(3) inverse.

    For ``T = [[R, t], [0, 1]]`` the inverse is ``[[R^T, -R^T t], [0, 1]]``.
    """
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    R = T[:3, :3]
    t = T[:3, 3]
    inv = np.eye(4)
    inv[:3, :3] = R.T
    inv[:3, 3] = -R.T @ t
    return inv


def compose_se3(T1: np.ndarray, T2: np.ndarray) -> np.ndarray:
    """Compose two SE(3) transforms: ``T1 @ T2``.

    This is simply matrix multiplication, but documented here to make the
    convention explicit: the result maps a point ``p`` as
    ``result @ p = T1 @ (T2 @ p)``.
    """
    T1 = np.asarray(T1, dtype=np.float64).reshape(4, 4)
    T2 = np.asarray(T2, dtype=np.float64).reshape(4, 4)
    return T1 @ T2


def relative_transform(T_from: np.ndarray, T_to: np.ndarray) -> np.ndarray:
    """Return the relative transform ``inv(T_from) @ T_to``.

    This gives the transform *from* the ``T_from`` frame *to* the ``T_to``
    frame, expressed in the ``T_from`` frame's coordinates.
    """
    return compose_se3(inverse_se3(T_from), T_to)


# ---------------------------------------------------------------------------
# Quaternion angular distance
# ---------------------------------------------------------------------------

def angle_between_quaternions(q1: np.ndarray, q2: np.ndarray) -> float:
    """Return the angular distance between two quaternions in radians.

    Handles the double-cover: ``q`` and ``-q`` represent the same rotation.

    Parameters
    ----------
    q1, q2 : ndarray (4,)
        Quaternions in ``(x, y, z, w)`` order.
    """
    q1 = np.asarray(q1, dtype=np.float64).reshape(4)
    q2 = np.asarray(q2, dtype=np.float64).reshape(4)
    q1 /= np.linalg.norm(q1)
    q2 /= np.linalg.norm(q2)

    dot = abs(np.dot(q1, q2))
    dot = min(dot, 1.0)  # clamp for numerical safety
    return 2.0 * np.arccos(dot)
