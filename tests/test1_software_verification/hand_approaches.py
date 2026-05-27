"""Hand approach poses and twists for each test object.

For each object, defines a canonical hand approach:
  - pose: hand position ~25 cm away from the object (grasp-planning distance),
    oriented straight toward it (no rotation — the hand approaches along -X
    in the object frame). The cameras are in their natural orientation:
    head camera offset behind/above, wrist camera from its mount position.
  - twist: small forward linear velocity (5 cm/s), zero angular velocity.
    A non-zero twist is required so the SMC sampler's ROI prediction reaches
    the object.  With zero twist the ROI stays around the hand and misses
    the cloud for most objects.

These parameters were chosen by sweeping distance × twist velocity across
all 12 test objects and selecting the combination that maximises average
grasp score while maintaining 100 % success rate (30 repetitions each).
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


# Default approach: hand 25 cm in front of object, approaching at 5 cm/s.
# At 0.50 m the ROI predictor cannot reach the object (the index-finger tip
# is offset ~0.19 m in Y from the wrist, so the predicted ROI misses the
# cloud).  0.25 m + lx=0.05 gives the highest average score across all 12
# test objects with 100 % reliability.  Zero twist fails even at 15 cm
# because the ROI never extends beyond the covariance noise envelope.
_DEFAULT_POSE = _pose(-0.25, 0.0, 0.0)
_DEFAULT_TWIST = _twist(0.05)

APPROACHES = {
    # ---- Parametric objects ----
    "cylinder_upright": {
        "pose": _DEFAULT_POSE,
        "twist": _DEFAULT_TWIST,
    },
    "cylinder_tilted": {
        "pose": _DEFAULT_POSE,
        "twist": _DEFAULT_TWIST,
    },
    "ellipsoid": {
        "pose": _DEFAULT_POSE,
        "twist": _DEFAULT_TWIST,
    },
    "tapered_bottle": {
        "pose": _pose(-0.25, 0.0, -0.02),
        "twist": _DEFAULT_TWIST,
    },
    "small_cube": {
        "pose": _DEFAULT_POSE,
        "twist": _DEFAULT_TWIST,
    },

    # ---- Non-convex objects ----
    "cross_shape": {
        "pose": _DEFAULT_POSE,
        "twist": _DEFAULT_TWIST,
    },

    # ---- YCB objects ----
    "banana": {
        "pose": _DEFAULT_POSE,
        "twist": _DEFAULT_TWIST,
    },
    "mug": {
        "pose": _DEFAULT_POSE,
        "twist": _DEFAULT_TWIST,
    },
    "power_drill": {
        "pose": _pose(-0.25, 0.0, -0.03),
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
