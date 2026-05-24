"""Hand approach poses and twists for each test object.

For each object, defines a canonical hand approach:
  - pose: hand position ~15cm away from the object (grasp-planning distance),
    oriented toward it. All poses are rotated 45° around the Z-axis so that
    the hand approaches from the side, giving the wrist camera a complementary
    viewing angle to the head camera (top-down).
  - twist: small forward linear velocity, near-zero angular velocity

The SMC sampler's ROI prediction must cover the object for the planner
to work. These poses may need iterative tuning — run with debug_visualization
enabled and verify the ROI covers the object.
"""

import numpy as np

# 45° Z-rotation constants
_COS45 = 0.7071067811865476
_SIN45 = 0.7071067811865475
_QW45 = 0.9238795325112867   # cos(22.5°)
_QZ45 = 0.3826834323650898   # sin(22.5°)


def _rotated_pose(px, py, pz, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
    """Rotate a pose 45° around the Z-axis.
    
    Multiplies the existing orientation quaternion by a 45° Z-rotation
    and rotates the position vector by 45° around Z.
    """
    # Rotate position
    px_new = px * _COS45 - py * _SIN45
    py_new = px * _SIN45 + py * _COS45
    # Multiply quaternions: q_rot(45° Z) * q_current
    qw_new = _QW45 * qw - _QZ45 * qz
    qx_new = _QW45 * qx + _QZ45 * qy
    qy_new = _QW45 * qy - _QZ45 * qx
    qz_new = _QW45 * qz + _QZ45 * qw
    n = np.sqrt(qw_new**2 + qx_new**2 + qy_new**2 + qz_new**2)
    return {
        "px": px_new, "py": py_new, "pz": pz,
        "qx": qx_new / n, "qy": qy_new / n, "qz": qz_new / n, "qw": qw_new / n,
    }


def _rotated_twist(lx, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
    """Rotate a twist vector 45° around the Z-axis."""
    return {
        "lx": lx * _COS45 - ly * _SIN45,
        "ly": lx * _SIN45 + ly * _COS45,
        "lz": lz,
        "ax": ax, "ay": ay, "az": az,
    }


# Default approach rotated 45° around Z
_DEFAULT_POSE_15CM = _rotated_pose(-0.15, 0.0, 0.0)
_DEFAULT_POSE_12CM = _rotated_pose(-0.12, 0.0, 0.0)
_DEFAULT_TWIST_10 = _rotated_twist(0.10)
_DEFAULT_TWIST_08 = _rotated_twist(0.08)

APPROACHES = {
    # ---- Parametric objects ----
    "cylinder_upright": {
        "pose": _rotated_pose(-0.15, 0.0, 0.0),
        "twist": _DEFAULT_TWIST_10,
    },
    "cylinder_tilted": {
        "pose": _rotated_pose(-0.15, 0.0, 0.0),
        "twist": _DEFAULT_TWIST_10,
    },
    "ellipsoid": {
        "pose": _rotated_pose(-0.15, 0.0, 0.0),
        "twist": _DEFAULT_TWIST_10,
    },
    "tapered_bottle": {
        "pose": _rotated_pose(-0.15, 0.0, -0.02),
        "twist": _DEFAULT_TWIST_10,
    },
    "l_block": {
        "pose": _rotated_pose(-0.15, 0.0, 0.01),
        "twist": _DEFAULT_TWIST_10,
    },
    "small_cube": {
        "pose": _rotated_pose(-0.12, 0.0, 0.0),
        "twist": _DEFAULT_TWIST_08,
    },
    "thin_plate": {
        "pose": _rotated_pose(-0.15, 0.0, 0.005),
        "twist": _DEFAULT_TWIST_10,
    },

    # ---- Non-convex objects ----
    "notched_box": {
        "pose": _rotated_pose(-0.15, 0.0, 0.0),
        "twist": _DEFAULT_TWIST_10,
    },
    "cross_shape": {
        "pose": _rotated_pose(-0.15, 0.0, 0.0),
        "twist": _DEFAULT_TWIST_10,
    },

    # ---- YCB objects ----
    "banana": {
        "pose": _rotated_pose(-0.15, 0.0, 0.0),
        "twist": _DEFAULT_TWIST_10,
    },
    "mug": {
        "pose": _rotated_pose(-0.15, 0.0, 0.0),
        "twist": _DEFAULT_TWIST_10,
    },
    "power_drill": {
        "pose": _rotated_pose(-0.15, 0.0, -0.03),
        "twist": _DEFAULT_TWIST_10,
    },
}


def get_approach(object_name: str) -> dict:
    """Get the hand approach for an object.

    Returns dict with 'pose' and 'twist' sub-dicts.
    Raises KeyError if object has no defined approach.
    """
    if object_name not in APPROACHES:
        raise KeyError(
            f"No approach defined for '{object_name}'. "
            f"Available: {sorted(APPROACHES.keys())}"
        )
    return APPROACHES[object_name]