"""Camera frustum definitions for the multi-view sensing system.

Defines the two camera viewpoints used in the real system:
  - Head-mounted camera (on glasses, ~40cm behind and ~45cm above hand)
  - Wrist-mounted camera (on prosthesis, 8cm mount from camera_mounts.yaml)

Camera geometry is derived from:
  - src/sensor_fusion_bringup/config/camera_mounts.yaml (8_cm_cam_mount)
  - src/sensor_fusion_bringup/config/d435i_cameras.yaml (D435 intrinsics)

The wrist camera transform chain:
  palm_frame -> screw_frame (screw_to_palm quaternion [-0.5, -0.5, 0.5, -0.5])
  screw_frame -> camera_link (24mm along screw Z, identity rotation)
  camera_link -> depth_optical (D435 internal, lens looks along camera_link -Z)

Result: the wrist camera is at (0.016, -0.092, 0.161) in palm frame,
looking FORWARD (+X palm) from an elevated position 16cm above the palm.

A 15-degree downward pitch is applied to the forward vector to model the
natural hand tilt during grasp approach. This gives the wrist camera visibility
of the upper portion of objects at typical grasp-planning distances (~25cm).
"""

import numpy as np

# ---------------------------------------------------------------------------
# Camera definitions (relative to hand/palm frame)
# ---------------------------------------------------------------------------

# Head-mounted camera: on the user's glasses/forehead
# Position relative to hand: ~40cm behind (in -X), ~45cm above
# Looking forward and downward at ~55 deg from horizontal
# This simulates a person looking down at their hand during a reach.
#
# At 25cm approach distance, the object center is at:
#   horizontal offset: 0.40 + 0.25 = 0.65m from head
#   vertical offset: 0.45m above object
#   angle from forward: atan2(0.45, 0.65) ≈ 35 deg (within 22.5 deg half-FOV)
#   distance: sqrt(0.65² + 0.45²) ≈ 0.79m
#
# The head camera sees the TOP of objects but NOT the front face
# (the face facing the approaching hand). This creates meaningful
# occlusion that the wrist camera's forward-looking view can resolve.
HEAD_CAMERA_LOCAL = {
    "position": np.array([-0.30, 0.20, 0.45], dtype=np.float32),
    # Forward + downward: the user looks at their hand which is in front and below
    "forward": np.array([0.80, 0.0, -0.76], dtype=np.float32),
    "up": np.array([0.0, 0.0, 1.0], dtype=np.float32),
    "fov_h_deg": 58.0,  # D435 horizontal FOV
    "fov_v_deg": 45.0,  # D435 vertical FOV
    "near_m": 0.20,
    "far_m": 3.00,
}

# Wrist-mounted camera: derived from camera_mounts.yaml 8_cm_cam_mount
#
# screw_to_palm transform:
#   translation: (0.01585, -0.091762, 0.160955)
#   quaternion [x,y,z,w]: (-0.5, -0.5, 0.5, -0.5)
#
# The quaternion maps palm->screw (parent->child in ROS TF convention).
# Its transpose maps screw->palm.
#
# Camera optical axis (depth_optical +Z) = camera_link -Z = screw -Z.
# In palm frame: R_screw_to_palm @ [0,0,-1] = (+1, 0, 0) = palm +X (forward).
#
# So the camera looks FORWARD from an elevated position 16cm above the palm,
# offset to the right side (-Y) of the hand.
#
# A 15-degree downward pitch is applied to the forward vector to model the
# natural hand tilt during grasp approach. This gives the wrist camera
# visibility of the upper portion of objects at grasp-planning distances.
# The up vector is correspondingly rotated to maintain orthogonality.
#
# Without this pitch, the camera at 16cm above the palm looking purely forward
# cannot see objects at table height within the 45-degree vertical FOV at
# approach distances < 46cm (geometric constraint).
#
# The 15-degree pitch is conservative: during a real power-grasp approach,
# the hand typically tilts 20-30 degrees, but we use a smaller value to
# avoid over-estimating the multi-view advantage.
_WRIST_PITCH_DEG = 15.0
_pitch = np.radians(_WRIST_PITCH_DEG)
_WRIST_FWD = np.array([np.cos(_pitch), 0.0, -np.sin(_pitch)], dtype=np.float32)
_WRIST_UP = np.array([np.sin(_pitch), 0.0, np.cos(_pitch)], dtype=np.float32)

WRIST_CAMERA_LOCAL = {
    "position": np.array([0.01585, -0.091762, 0.160955], dtype=np.float32),
    "forward": _WRIST_FWD,
    "up": _WRIST_UP,
    "fov_h_deg": 58.0,
    "fov_v_deg": 45.0,
    "near_m": 0.20,
    "far_m": 3.00,
}


# ---------------------------------------------------------------------------
# World-frame camera computation
# ---------------------------------------------------------------------------


def _quat_to_rotation_matrix(qx, qy, qz, qw):
    """Convert quaternion to 3x3 rotation matrix."""
    # Normalize
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    return np.array(
        [
            [
                1 - 2 * (qy * qy + qz * qz),
                2 * (qx * qy - qw * qz),
                2 * (qx * qz + qw * qy),
            ],
            [
                2 * (qx * qy + qw * qz),
                1 - 2 * (qx * qx + qz * qz),
                2 * (qy * qz - qw * qx),
            ],
            [
                2 * (qx * qz - qw * qy),
                2 * (qy * qz + qw * qx),
                1 - 2 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float32,
    )


def get_camera_world_positions(hand_pose: dict) -> list[tuple[float, float, float]]:
    """Get camera world-frame positions given a hand pose.

    Args:
        hand_pose: dict with keys px, py, pz, qx, qy, qz, qw

    Returns:
        List of (x, y, z) camera positions in world frame.
    """
    pos = np.array(
        [hand_pose["px"], hand_pose["py"], hand_pose["pz"]], dtype=np.float32
    )
    R = _quat_to_rotation_matrix(
        hand_pose["qx"], hand_pose["qy"], hand_pose["qz"], hand_pose["qw"]
    )

    cameras = []
    for cam_def in [HEAD_CAMERA_LOCAL, WRIST_CAMERA_LOCAL]:
        local_pos = cam_def["position"]
        world_pos = R @ local_pos + pos
        cameras.append(tuple(world_pos.tolist()))

    return cameras


def get_camera_world_frames(hand_pose: dict) -> list[dict]:
    """Get full camera frame info (position + orientation) in world frame.

    Returns:
        List of dicts with 'position', 'forward', 'up', 'fov_h_deg', etc.
    """
    pos = np.array(
        [hand_pose["px"], hand_pose["py"], hand_pose["pz"]], dtype=np.float32
    )
    R = _quat_to_rotation_matrix(
        hand_pose["qx"], hand_pose["qy"], hand_pose["qz"], hand_pose["qw"]
    )

    frames = []
    for cam_def in [HEAD_CAMERA_LOCAL, WRIST_CAMERA_LOCAL]:
        world_pos = R @ cam_def["position"] + pos
        world_fwd = R @ cam_def["forward"]
        world_fwd /= np.linalg.norm(world_fwd)
        world_up = R @ cam_def["up"]

        frames.append(
            {
                "position": world_pos,
                "forward": world_fwd,
                "up": world_up,
                "fov_h_deg": cam_def["fov_h_deg"],
                "fov_v_deg": cam_def["fov_v_deg"],
                "near_m": cam_def["near_m"],
                "far_m": cam_def["far_m"],
            }
        )

    return frames


def validate_camera_frustum(
    camera_def: dict, test_point: tuple = (0, 0, 0), label: str = "camera"
) -> dict:
    """Check if a test point falls within a camera frustum.

    Args:
        camera_def: dict with position, forward, up, fov_h_deg, fov_v_deg, near_m, far_m
        test_point: (x, y, z) point to test (in the same frame as camera_def)
        label: name for diagnostic output

    Returns:
        dict with 'in_frustum', 'distance', 'angle_h', 'angle_v' diagnostics
    """
    pos = np.array(camera_def["position"], dtype=np.float64)
    fwd = np.array(camera_def["forward"], dtype=np.float64)
    fwd /= np.linalg.norm(fwd)
    up = np.array(camera_def["up"], dtype=np.float64)
    right = np.cross(fwd, up)
    right_norm = np.linalg.norm(right)
    if right_norm > 1e-6:
        right /= right_norm
    up = np.cross(right, fwd)

    target = np.array(test_point, dtype=np.float64)
    rel = target - pos
    dist = np.linalg.norm(rel)

    if dist < 1e-6:
        return {
            "in_frustum": False,
            "distance": 0,
            "angle_h": 0,
            "angle_v": 0,
            "label": label,
        }

    z = np.dot(rel, fwd)
    x = np.dot(rel, right)
    y = np.dot(rel, up)

    angle_h = np.degrees(np.arctan2(x, z))
    angle_v = np.degrees(np.arctan2(y, z))

    half_fov_h = camera_def["fov_h_deg"] / 2
    half_fov_v = camera_def["fov_v_deg"] / 2

    in_frustum = (
        abs(angle_h) < half_fov_h
        and abs(angle_v) < half_fov_v
        and camera_def["near_m"] < dist < camera_def["far_m"]
    )

    return {
        "label": label,
        "in_frustum": in_frustum,
        "distance": dist,
        "distance_cm": dist * 100,
        "angle_h": angle_h,
        "angle_v": angle_v,
        "half_fov_h": half_fov_h,
        "half_fov_v": half_fov_v,
    }
