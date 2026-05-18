"""Frustum culling and depth-buffer occlusion simulation.

Provides two levels of occlusion:
  1. frustum_cull() — keep only points inside the camera frustum
  2. depth_buffer_occlude() — additionally remove self-occluded points via z-buffer

The depth-buffer approach rasterizes points into a 2D grid and keeps only
the nearest point per pixel, simulating what a real depth camera would see.
"""

import numpy as np


def _build_frustum_planes(position, forward, up, fov_h_deg, fov_v_deg, near, far):
    """Build 6 frustum planes (normal, distance) for culling.

    Returns list of (normal_vector, d_value) where a point p is inside
    if normal @ p + d >= 0 for all planes.
    """
    forward = forward / np.linalg.norm(forward)
    right = np.cross(forward, up)
    right_norm = np.linalg.norm(right)
    if right_norm < 1e-6:
        # forward and up are parallel — pick arbitrary right
        right = np.array([1, 0, 0], dtype=np.float32)
    else:
        right = right / right_norm
    up = np.cross(right, forward)
    up = up / np.linalg.norm(up)

    fov_h = np.radians(fov_h_deg) / 2
    fov_v = np.radians(fov_v_deg) / 2

    planes = []

    # Near plane
    planes.append((forward, -(np.dot(forward, position) + near)))
    # Far plane: outward normal is -forward, passes through position + far*forward
    planes.append((-forward, np.dot(forward, position) + far))

    # Left / Right planes
    # Left plane (sign=+1): clips points too far to the right
    # Right plane (sign=-1): clips points too far to the left
    tan_h = np.tan(fov_h)
    for sign in [1, -1]:
        normal = forward + sign * tan_h * right
        normal = normal / np.linalg.norm(normal)
        d = -np.dot(normal, position)
        planes.append((normal, d))

    # Top / Bottom planes
    # Top plane (sign=+1): clips points too far below
    # Bottom plane (sign=-1): clips points too far above
    tan_v = np.tan(fov_v)
    for sign in [1, -1]:
        normal = forward + sign * tan_v * up
        normal = normal / np.linalg.norm(normal)
        d = -np.dot(normal, position)
        planes.append((normal, d))

    return planes


def frustum_cull(points, position, forward, up, fov_h_deg, fov_v_deg, near, far):
    """Keep only points inside the camera viewing frustum.

    Args:
        points: (N, 3) float32 array
        position: (3,) camera position
        forward: (3,) camera forward direction
        up: (3,) camera up direction
        fov_h_deg: horizontal FOV in degrees
        fov_v_deg: vertical FOV in degrees
        near, far: near/far clip distances

    Returns:
        (N', 3) float32 array of visible points
        (N,) bool mask
    """
    points = np.asarray(points, dtype=np.float32)
    position = np.asarray(position, dtype=np.float32)
    forward = np.asarray(forward, dtype=np.float32)
    up = np.asarray(up, dtype=np.float32)

    planes = _build_frustum_planes(position, forward, up, fov_h_deg, fov_v_deg, near, far)

    mask = np.ones(len(points), dtype=bool)
    for normal, d in planes:
        normal = np.asarray(normal, dtype=np.float32)
        # Inside if normal @ p + d >= 0
        dots = points @ normal + d
        mask &= (dots >= 0)

    return points[mask], mask


def depth_buffer_occlude(points, position, forward, up,
                         fov_h_deg, fov_v_deg, near, far,
                         resolution=(640, 480)):
    """Remove self-occluded points using a z-buffer.

    First applies frustum culling, then rasterizes surviving points into
    a 2D grid and keeps only the nearest point per pixel (with a small
    depth tolerance to preserve points at similar depths).

    Args:
        points: (N, 3) float32 array
        position, forward, up: camera frame
        fov_h_deg, fov_v_deg: field of view
        near, far: clip distances
        resolution: (width, height) of the virtual depth buffer

    Returns:
        (N', 3) float32 array of visible points
        (N,) bool mask (True = visible)
    """
    points = np.asarray(points, dtype=np.float32)
    position = np.asarray(position, dtype=np.float32)
    forward = np.asarray(forward, dtype=np.float32)
    up = np.asarray(up, dtype=np.float32)

    # Step 1: Frustum cull
    frustum_mask = np.ones(len(points), dtype=bool)
    planes = _build_frustum_planes(position, forward, up, fov_h_deg, fov_v_deg, near, far)
    for normal, d in planes:
        normal = np.asarray(normal, dtype=np.float32)
        dots = points @ normal + d
        frustum_mask &= (dots >= 0)

    if not np.any(frustum_mask):
        return np.zeros((0, 3), dtype=np.float32), frustum_mask

    visible_pts = points[frustum_mask]

    # Step 2: Build camera coordinate system
    fwd = forward / np.linalg.norm(forward)
    right = np.cross(fwd, up)
    r_norm = np.linalg.norm(right)
    if r_norm < 1e-6:
        right = np.array([1, 0, 0], dtype=np.float32)
    else:
        right = right / r_norm
    cam_up = np.cross(right, fwd)
    cam_up = cam_up / np.linalg.norm(cam_up)

    # Step 3: Project points into camera space
    rel = visible_pts - position
    z = rel @ fwd  # depth along forward axis
    x = rel @ right
    y = rel @ cam_up

    # Normalize to pixel coordinates
    fov_h = np.radians(fov_h_deg)
    fov_v = np.radians(fov_v_deg)
    px = x / (z * np.tan(fov_h / 2) + 1e-8)  # [-1, 1]
    py = y / (z * np.tan(fov_v / 2) + 1e-8)  # [-1, 1]

    w, h = resolution
    col = ((px + 1) * 0.5 * (w - 1)).astype(np.int32)
    row = ((1 - py) * 0.5 * (h - 1)).astype(np.int32)  # flip y

    # Clip to image bounds
    valid = (col >= 0) & (col < w) & (row >= 0) & (row < h) & (z > near) & (z < far)

    # Step 4: Z-buffer — keep nearest point per pixel
    # Use a depth tolerance to keep points at similar depths (surface thickness)
    depth_tol = 0.005  # 5mm tolerance

    depth_buffer = np.full((h, w), np.inf, dtype=np.float32)
    pixel_mask = np.zeros(len(visible_pts), dtype=bool)

    # Sort by depth (far to near) so nearest points overwrite
    order = np.argsort(-z[valid])
    valid_indices = np.where(valid)[0][order]

    for idx in valid_indices:
        r, c = row[idx], col[idx]
        if z[idx] < depth_buffer[r, c] + depth_tol:
            pixel_mask[idx] = True
            if z[idx] < depth_buffer[r, c]:
                depth_buffer[r, c] = z[idx]

    # Map back to original point indices
    result_mask = frustum_mask.copy()
    visible_indices = np.where(frustum_mask)[0]
    result_mask[visible_indices[~pixel_mask]] = False

    return points[result_mask], result_mask


def generate_view_cloud(points, camera_frame, use_depth_buffer=True):
    """Generate a single-view point cloud from a camera frame.

    Convenience function that applies frustum culling and optionally
    depth-buffer occlusion.

    Args:
        points: (N, 3) complete point cloud
        camera_frame: dict from view_geometry.get_camera_world_frames()
        use_depth_buffer: if True, apply depth-buffer occlusion

    Returns:
        (N', 3) visible point cloud
    """
    kwargs = dict(
        position=camera_frame["position"],
        forward=camera_frame["forward"],
        up=camera_frame["up"],
        fov_h_deg=camera_frame["fov_h_deg"],
        fov_v_deg=camera_frame["fov_v_deg"],
        near=camera_frame["near_m"],
        far=camera_frame["far_m"],
    )
    if use_depth_buffer:
        visible, _ = depth_buffer_occlude(points, **kwargs)
    else:
        visible, _ = frustum_cull(points, **kwargs)
    return visible
