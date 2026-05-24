"""Frustum culling, back-face culling, and depth-buffer occlusion simulation.

Provides three levels of occlusion:
  1. frustum_cull() — keep only points inside the camera viewing frustum
  2. backface_cull() — remove points whose surface normal faces away from camera
  3. depth_buffer_occlude() — additionally remove self-occluded points via z-buffer

The depth-buffer approach rasterizes points into a 2D grid and keeps only
the nearest point per pixel. Back-face culling estimates surface normals
via local PCA and removes points facing away from the camera, which is
essential for thin structures where the depth buffer alone may not
correctly exclude back surfaces visible at grazing angles.
"""

import numpy as np


def _estimate_normals(points: np.ndarray,
                      k: int = 12) -> np.ndarray:
    """Estimate surface normals via local plane fitting (PCA on kNN).

    Args:
        points: (N, 3) float32 point cloud
        k: number of nearest neighbours for local neighbourhood

    Returns:
        (N, 3) float32 normals oriented outward from the centre of mass
    """
    from scipy.spatial import KDTree

    n = len(points)
    if n < k:
        return np.zeros((n, 3), dtype=np.float32)

    tree = KDTree(points)
    _, indices = tree.query(points, k=k)

    neighbours = points[indices]                      # (N, k, 3)
    centroids = neighbours.mean(axis=1, keepdims=True)  # (N, 1, 3)
    centred = neighbours - centroids                   # (N, k, 3)

    # Covariance matrices  (N, 3, 3)
    cov = np.einsum('nki,nkj->nij', centred, centred) / (k - 1)

    # Eigen-decomposition — smallest eigenvector = surface normal
    eigvals, eigvecs = np.linalg.eigh(cov)  # (N, 3), (N, 3, 3)
    normals = eigvecs[:, :, 0].copy()       # (N, 3)  smallest eigenvalue

    # Orient outward from the centre of mass
    centre = points.mean(axis=0)
    outward = points - centre
    flip = np.einsum('ni,ni->n', normals, outward) < 0
    normals[flip] *= -1

    return normals.astype(np.float32)


def backface_cull(points: np.ndarray,
                  camera_position: np.ndarray,
                  normals: np.ndarray | None = None) -> np.ndarray:
    """Return mask of points whose surface normal faces toward the camera.

    A point is *front-facing* (visible) when its outward surface normal
    has a positive dot-product with the vector from the point to the
    camera.

    Args:
        points: (N, 3) point cloud
        camera_position: (3,) camera origin in world frame
        normals: optional pre-computed (N, 3) normals; computed on the fly
                 if not supplied.

    Returns:
        (N,) bool mask  (True = front-facing = keep)
    """
    if normals is None:
        normals = _estimate_normals(points)

    view_dir = camera_position - points                # (N, 3)
    vn = np.linalg.norm(view_dir, axis=1, keepdims=True)
    view_dir = view_dir / (vn + 1e-10)

    dot = np.einsum('ni,ni->n', normals, view_dir)    # (N,)
    return dot > 0


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
                         resolution=(640, 480),
                         use_backface_cull=True):
    """Remove self-occluded and back-facing points.

    Three-stage pipeline:
      1. Frustum cull — discard points outside the viewing frustum.
      2. Back-face cull — discard points whose surface normal faces away.
      3. Z-buffer — keep only the nearest point per pixel (with small
         depth tolerance to preserve points at similar depths).

    Args:
        points: (N, 3) float32 array
        position, forward, up: camera frame
        fov_h_deg, fov_v_deg: field of view
        near, far: clip distances
        resolution: (width, height) of the virtual depth buffer
        use_backface_cull: if True, apply surface-normal back-face culling

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

    # Step 2: Back-face cull — remove points facing away from camera
    if use_backface_cull and len(visible_pts) >= 12:
        bf_mask = backface_cull(visible_pts, position)
        frustum_mask[frustum_mask] = bf_mask  # update combined mask
        visible_pts = visible_pts[bf_mask]

    if not np.any(visible_pts):
        return np.zeros((0, 3), dtype=np.float32), frustum_mask

    # Step 3: Build camera coordinate system
    fwd = forward / np.linalg.norm(forward)
    right = np.cross(fwd, up)
    r_norm = np.linalg.norm(right)
    if r_norm < 1e-6:
        right = np.array([1, 0, 0], dtype=np.float32)
    else:
        right = right / r_norm
    cam_up = np.cross(right, fwd)
    cam_up = cam_up / np.linalg.norm(cam_up)

    # Step 4: Project points into camera space
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

    # Step 5: Z-buffer — keep nearest point per pixel
    # Depth tolerance scales with distance: at 1m, tolerance is 2mm;
    # at 0.2m, tolerance is 0.4mm. This prevents over-aggressive culling
    # of nearby surfaces while correctly occluding distant ones.
    median_depth = float(np.median(z[valid])) if np.any(valid) else 1.0
    depth_tol = 0.002 * median_depth  # 0.2% of distance

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


def voxel_fuse_multi_view(points_list, voxel_size=0.002):
    """Fuse multiple view point clouds using voxel-grid averaging.

    This simulates the TSDF fusion that occurs in the real multi-camera
    system. Points from different views are merged by voxelising the
    combined cloud and averaging points within each voxel. This produces
    a cleaner, more uniform point cloud than simple concatenation.

    Args:
        points_list: list of (N_i, 3) float32 arrays from different views
        voxel_size: voxel grid resolution in metres (default: 2mm)

    Returns:
        (M, 3) float32 array of fused, deduplicated points
    """
    if not points_list:
        return np.zeros((0, 3), dtype=np.float32)

    # Concatenate all views
    combined = np.vstack(points_list)

    if len(combined) == 0:
        return combined

    # Voxelise: compute voxel key for each point
    voxel_keys = np.floor(combined / voxel_size).astype(np.int64)

    # Group by voxel and average
    unique_keys, inverse = np.unique(voxel_keys, axis=0, return_inverse=True)

    # Compute mean position per voxel
    n_voxels = len(unique_keys)
    sums = np.zeros((n_voxels, 3), dtype=np.float64)
    counts = np.zeros(n_voxels, dtype=np.int32)

    np.add.at(sums, inverse, combined)
    np.add.at(counts, inverse, 1)

    # Average positions (this is the TSDF-like fusion step)
    fused = (sums / counts[:, np.newaxis]).astype(np.float32)

    return fused
