"""Hand approach poses and twists for each test object.

For each object, defines a canonical hand approach:
  - pose: hand position ~50cm away from the object (grasp-planning distance),
    oriented straight toward it (no rotation — the hand approaches along -X
    in the object frame). The cameras are in their natural orientation:
    head camera offset behind/above, wrist camera from its mount position.
  - twist: small forward linear velocity, near-zero angular velocity

The SMC sampler's ROI prediction must cover the object for the planner
to work. These poses may need iterative tuning — run with debug_visualization
enabled and verify the ROI covers the object.
"""


def _pose(px, py, pz, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
    """Build a pose dict (no rotation applied — hand approaches straight ahead)."""
    return {
        "px": px, "py": py, "pz": pz,
        "qx": qx, "qy": qy, "qz": qz, "qw": qw,
    }


def _twist(lx, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
    """Build a twist dict (no rotation applied)."""
    return {
        "lx": lx, "ly": ly, "lz": lz,
        "ax": ax, "ay": ay, "az": az,
    }


# Default approach: hand 50 cm in front of object, no rotation
_DEFAULT_POSE = _pose(-0.50, 0.0, 0.0)
_DEFAULT_TWIST = _twist(0.10)

APPROACHES = {
    # ---- Parametric objects ----
    "cylinder_upright": {
        "pose": _pose(-0.50, 0.0, 0.0),
        "twist": _DEFAULT_TWIST,
    },
    "cylinder_tilted": {
        "pose": _pose(-0.50, 0.0, 0.0),
        "twist": _DEFAULT_TWIST,
    },
    "ellipsoid": {
        "pose": _pose(-0.50, 0.0, 0.0),
        "twist": _DEFAULT_TWIST,
    },
    "tapered_bottle": {
        "pose": _pose(-0.50, 0.0, -0.02),
        "twist": _DEFAULT_TWIST,
    },
    "l_block": {
        "pose": _pose(-0.50, 0.0, 0.01),
        "twist": _DEFAULT_TWIST,
    },
    "small_cube": {
        "pose": _pose(-0.50, 0.0, 0.0),
        "twist": _DEFAULT_TWIST,
    },
    "thin_plate": {
        "pose": _pose(-0.50, 0.0, 0.005),
        "twist": _DEFAULT_TWIST,
    },

    # ---- Non-convex objects ----
    "notched_box": {
        "pose": _pose(-0.50, 0.0, 0.0),
        "twist": _DEFAULT_TWIST,
    },
    "cross_shape": {
        "pose": _pose(-0.50, 0.0, 0.0),
        "twist": _DEFAULT_TWIST,
    },

    # ---- YCB objects ----
    "banana": {
        "pose": _pose(-0.50, 0.0, 0.0),
        "twist": _DEFAULT_TWIST,
    },
    "mug": {
        "pose": _pose(-0.50, 0.0, 0.0),
        "twist": _DEFAULT_TWIST,
    },
    "power_drill": {
        "pose": _pose(-0.50, 0.0, -0.03),
        "twist": _DEFAULT_TWIST,
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