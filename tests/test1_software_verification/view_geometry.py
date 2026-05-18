"""Camera frustum definitions for the multi-view sensing system.

Defines the two camera viewpoints used in the real system:
  - Head-mounted camera (on glasses, ~40cm above and behind hand)
  - Wrist-mounted camera (on prosthesis, ~8cm from hand center)

Also provides utilities for transforming camera positions into world frame
given a hand pose.
"""

import numpy as np


# ---------------------------------------------------------------------------
# Camera definitions (relative to hand frame)
# ---------------------------------------------------------------------------

# Head-mounted camera: on glasses, above and behind the hand
# Position relative to hand: ~15cm behind (in -X), ~30cm above
# Looking forward and downward at ~40 deg from horizontal
# This simulates a person looking at their hand during a reach
HEAD_CAMERA_LOCAL = {
    "position": np.array([-0.15, 0.0, 0.30], dtype=np.float32),
    # Forward + downward: the user looks at their hand which is in front and below
    "forward": np.array([0.75, 0.0, -0.65], dtype=np.float32),
    "up": np.array([0.0, 0.0, 1.0], dtype=np.float32),
    "fov_h_deg": 58.0,   # D435 horizontal FOV
    "fov_v_deg": 45.0,   # D435 vertical FOV
    "near_m": 0.10,
    "far_m": 1.50,
}

# Wrist-mounted camera: on the prosthesis wrist, close to the hand
# Position relative to hand: ~8cm forward, ~2cm below
WRIST_CAMERA_LOCAL = {
    "position": np.array([0.08, -0.02, 0.0], dtype=np.float32),
    "forward": np.array([1.0, 0.0, 0.0], dtype=np.float32),
    "up": np.array([0.0, 0.0, 1.0], dtype=np.float32),
    "fov_h_deg": 58.0,
    "fov_v_deg": 45.0,
    "near_m": 0.05,
    "far_m": 1.00,
}


# ---------------------------------------------------------------------------
# World-frame camera computation
# ---------------------------------------------------------------------------

def _quat_to_rotation_matrix(qx, qy, qz, qw):
    """Convert quaternion to 3x3 rotation matrix."""
    # Normalize
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float32)


def get_camera_world_positions(hand_pose: dict) -> list[tuple[float, float, float]]:
    """Get camera world-frame positions given a hand pose.

    Args:
        hand_pose: dict with keys px, py, pz, qx, qy, qz, qw

    Returns:
        List of (x, y, z) camera positions in world frame.
    """
    pos = np.array([hand_pose["px"], hand_pose["py"], hand_pose["pz"]], dtype=np.float32)
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
    pos = np.array([hand_pose["px"], hand_pose["py"], hand_pose["pz"]], dtype=np.float32)
    R = _quat_to_rotation_matrix(
        hand_pose["qx"], hand_pose["qy"], hand_pose["qz"], hand_pose["qw"]
    )

    frames = []
    for cam_def in [HEAD_CAMERA_LOCAL, WRIST_CAMERA_LOCAL]:
        world_pos = R @ cam_def["position"] + pos
        world_fwd = R @ cam_def["forward"]
        world_fwd /= np.linalg.norm(world_fwd)
        world_up = R @ cam_def["up"]

        frames.append({
            "position": world_pos,
            "forward": world_fwd,
            "up": world_up,
            "fov_h_deg": cam_def["fov_h_deg"],
            "fov_v_deg": cam_def["fov_v_deg"],
            "near_m": cam_def["near_m"],
            "far_m": cam_def["far_m"],
        })

    return frames
