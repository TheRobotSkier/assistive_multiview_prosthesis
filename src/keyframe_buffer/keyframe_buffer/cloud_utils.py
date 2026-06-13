"""cloud_utils — unorganized/organized point cloud handling utilities.

Pure numpy implementation with ZERO ROS imports. All functions are vectorised
(no Python loops over individual points).

The V6 design treats unorganized clouds (height==1) as the *default* path.
Organized clouds (height>1) are an optimisation handled by a separate fast-path.

Conventions
-----------
- ``cloud_xyz`` for unorganized clouds: ``(N, 3)`` float array in the *world* frame.
- ``cloud_xyz`` for organized clouds: ``(H, W, 3)`` float array.
- ``mask``: ``(H, W)`` binary (0/1 or False/True) array.
- ``K``: ``(3, 3)`` camera intrinsics matrix::

      K = [[fx,  0, cx],
           [ 0, fy, cy],
           [ 0,  0,  1]]

- ``pose``: ``(4, 4)`` homogeneous matrix ``T_world_camera`` — maps a point in
  *camera* frame to *world* frame (``p_world = pose @ p_camera``).

Reference: V6 plan §5.5.
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "detect_organization",
    "mask_unorganized_cloud",
    "mask_organized_cloud",
    "build_depth_image",
    "project_3d_to_2d",
    "lookup_depth_3d",
    "project_points_to_pixels",
]


# ---------------------------------------------------------------------------
# Organisation detection
# ---------------------------------------------------------------------------

def detect_organization(height: int) -> bool:
    """Return ``True`` if the cloud is organised (``height > 1``).

    In ROS ``sensor_msgs/PointCloud2`` an *organised* cloud has ``height > 1``
    (the image rows) and ``width`` equal to the image columns.  An
    *unorganised* cloud has ``height == 1`` and ``width == N`` (the total point
    count).

    Parameters
    ----------
    height:
        The ``height`` field of the incoming ``PointCloud2`` message.
    """
    return height > 1


# ---------------------------------------------------------------------------
# Internal helper: batch-project world points to camera-frame and pixel coords
# ---------------------------------------------------------------------------

def project_points_to_pixels(
    cloud_xyz: np.ndarray,
    K: np.ndarray,
    pose: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project ``(N, 3)`` world-frame points to camera-frame and pixel coords.

    Returns
    -------
    p_cam : ndarray (N, 3)
        Points in the camera optical frame (z points forward).
    uv : ndarray (N, 2)
        Floating-point pixel coordinates ``(u, v)``.
    in_front : ndarray (N,) bool
        ``True`` where the point is in front of the camera (``z > 0``).
    """
    pts = np.asarray(cloud_xyz, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"cloud_xyz must be (N, 3), got {pts.shape}")

    N = pts.shape[0]
    homogeneous = np.empty((N, 4), dtype=np.float64)
    homogeneous[:, :3] = pts
    homogeneous[:, 3] = 1.0

    pose_inv = np.linalg.inv(pose)
    p_cam = (pose_inv @ homogeneous.T).T[:, :3]

    in_front = p_cam[:, 2] > 0

    # Project only the in-front points to avoid division by zero / negatives.
    uv = np.zeros((N, 2), dtype=np.float64)
    if np.any(in_front):
        proj = (K @ p_cam[in_front].T).T  # (M, 3)
        uv[in_front] = proj[:, :2] / proj[:, 2:3]

    return p_cam, uv, in_front


# ---------------------------------------------------------------------------
# Unorganised-cloud masking (the V6 default path)
# ---------------------------------------------------------------------------

def mask_unorganized_cloud(
    cloud_xyz: np.ndarray,
    mask: np.ndarray,
    K: np.ndarray,
    pose: np.ndarray,
) -> np.ndarray:
    """Apply a 2-D mask to an *unorganised* cloud via projection.

    Parameters
    ----------
    cloud_xyz : ndarray (N, 3)
        Points in the world frame.
    mask : ndarray (H, W)
        Binary mask (e.g. from SAM).
    K : ndarray (3, 3)
        Camera intrinsics.
    pose : ndarray (4, 4)
        ``T_world_camera``.

    Returns
    -------
    keep : ndarray (N,) bool
        ``True`` for points that project inside the image **and** whose
        projected pixel is set in *mask*.
    """
    H, W = mask.shape
    N = cloud_xyz.shape[0]
    keep = np.zeros(N, dtype=bool)
    if N == 0:
        return keep

    _, uv, in_front = project_points_to_pixels(cloud_xyz, K, pose)

    idx_front = np.where(in_front)[0]
    if len(idx_front) == 0:
        return keep

    u = uv[in_front, 0].astype(np.int32)
    v = uv[in_front, 1].astype(np.int32)

    valid = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    idx_valid = idx_front[valid]

    if len(idx_valid) == 0:
        return keep

    keep[idx_valid] = mask[v[valid], u[valid]].astype(bool)
    return keep


# ---------------------------------------------------------------------------
# Organised-cloud masking (the fast-path optimisation)
# ---------------------------------------------------------------------------

def mask_organized_cloud(
    cloud_xyz: np.ndarray,
    mask: np.ndarray,
) -> np.ndarray:
    """Return the points of an *organised* cloud selected by *mask*.

    Parameters
    ----------
    cloud_xyz : ndarray (H, W, 3)
        Organised point cloud.
    mask : ndarray (H, W)
        Binary mask.

    Returns
    -------
    masked_points : ndarray (K, 3)
        The ``(x, y, z)`` rows whose pixel is ``True`` in *mask*.
    """
    return cloud_xyz[mask.astype(bool)]


# ---------------------------------------------------------------------------
# Depth-image rasterisation (for TSDF integration of unorganised clouds)
# ---------------------------------------------------------------------------

def build_depth_image(
    cloud_xyz: np.ndarray,
    K: np.ndarray,
    pose: np.ndarray,
    H: int,
    W: int,
    mask: np.ndarray | None = None,
) -> np.ndarray:
    """Rasterise an unorganised cloud into an ``(H, W)`` float32 depth image.

    A z-buffer keeps the **nearest** point per pixel (sort far-to-near so that
    near points overwrite far ones).

    Parameters
    ----------
    cloud_xyz : ndarray (N, 3)
        Points in the world frame.
    K : ndarray (3, 3)
        Camera intrinsics.
    pose : ndarray (4, 4)
        ``T_world_camera``.
    H, W : int
        Output depth-image dimensions.
    mask : ndarray (H, W) or None
        Optional pre-mask to reduce work.  When provided, points whose
        projected pixel is outside the mask are discarded *before* rasterising.

    Returns
    -------
    depth : ndarray (H, W) float32
        Depth values in metres (0 where no point falls).
    """
    depth = np.zeros((H, W), dtype=np.float32)
    N = cloud_xyz.shape[0]
    if N == 0:
        return depth

    p_cam, uv, in_front = project_points_to_pixels(cloud_xyz, K, pose)
    if not np.any(in_front):
        return depth

    u = np.clip(uv[in_front, 0].astype(np.int32), 0, W - 1)
    v = np.clip(uv[in_front, 1].astype(np.int32), 0, H - 1)
    z = p_cam[in_front, 2]

    if mask is not None:
        keep_px = mask[v, u].astype(bool)
        u, v, z = u[keep_px], v[keep_px], z[keep_px]
        if len(z) == 0:
            return depth

    # Z-buffer: sort far-to-near so near overwrites far.
    order = np.argsort(-z)
    depth[v[order], u[order]] = z[order].astype(np.float32)
    return depth


# ---------------------------------------------------------------------------
# Single-point projection (used by TSDF fusion to find the SAM click point)
# ---------------------------------------------------------------------------

def project_3d_to_2d(
    point_3d: np.ndarray,
    K: np.ndarray,
    pose: np.ndarray,
) -> tuple[int, int]:
    """Project a single 3-D world point to pixel ``(u, v)``.

    Parameters
    ----------
    point_3d : ndarray (3,)
        Point in the world frame.
    K : ndarray (3, 3)
        Camera intrinsics.
    pose : ndarray (4, 4)
        ``T_world_camera``.

    Returns
    -------
    (u, v) : tuple of int
        Pixel coordinates.  Returns ``(-1, -1)`` if the point is behind the
        camera.
    """
    p = np.asarray(point_3d, dtype=np.float64).reshape(3)
    homogeneous = np.append(p, 1.0)

    pose_inv = np.linalg.inv(pose)
    p_cam = (pose_inv @ homogeneous)[:3]

    if p_cam[2] <= 0:
        return (-1, -1)

    proj = K @ p_cam
    u = int(round(proj[0] / proj[2]))
    v = int(round(proj[1] / proj[2]))
    return (u, v)


# ---------------------------------------------------------------------------
# Depth lookup at a pixel (used by SIFT node for keypoint depth)
# ---------------------------------------------------------------------------

def lookup_depth_3d(
    cloud: np.ndarray,
    organized: bool,
    u: int,
    v: int,
    K: np.ndarray | None = None,
    pose: np.ndarray | None = None,
    search_radius_m: float = 0.02,
) -> np.ndarray | None:
    """Look up the 3-D point at pixel ``(u, v)``.

    For **organised** clouds this is a direct index: ``cloud[v, u]``.

    For **unorganised** clouds the nearest 3-D point to the ray through pixel
    ``(u, v)`` is found.  ``K`` and ``pose`` are required in that case.

    Parameters
    ----------
    cloud : ndarray
        ``(H, W, 3)`` if *organised*, else ``(N, 3)``.
    organized : bool
        Whether *cloud* is organised.
    u, v : int
        Pixel coordinates.
    K : ndarray (3, 3), optional
        Required for unorganised lookup.
    pose : ndarray (4, 4), optional
        Required for unorganised lookup.
    search_radius_m : float
        Maximum perpendicular distance from the ray for a point to be
        considered (default 2 cm).

    Returns
    -------
    point : ndarray (3,) or None
        The 3-D point, or ``None`` if no point is within range.
    """
    if organized:
        H, W = cloud.shape[:2]
        if 0 <= v < H and 0 <= u < W:
            pt = cloud[v, u]
            if np.all(np.isfinite(pt)) and not np.allclose(pt, 0.0):
                return pt
            return None
        return None

    # --- Unorganised path ---
    if K is None or pose is None:
        raise ValueError("K and pose are required for unorganised depth lookup")

    pts = np.asarray(cloud, dtype=np.float64)
    if pts.shape[0] == 0:
        return None

    # Build the camera-frame ray for pixel (u, v).
    ray_cam = np.array([
        (u - K[0, 2]) / K[0, 0],
        (v - K[1, 2]) / K[1, 1],
        1.0,
    ])
    ray_cam /= np.linalg.norm(ray_cam)

    # Transform ray to world frame using the rotation part of *pose*.
    R = pose[:3, :3]
    ray_world = R @ ray_cam
    cam_origin = pose[:3, 3]

    # Perpendicular distance from each point to the ray.
    rel = pts - cam_origin  # (N, 3)
    dot = rel @ ray_world   # (N,) projection along ray
    # Only consider points in front of the camera.
    front = dot > 0
    if not np.any(front):
        return None
    proj = dot[front][:, None] * ray_world[None, :]
    perp = np.linalg.norm(rel[front] - proj, axis=1)

    if np.min(perp) > search_radius_m:
        return None

    best = np.argmin(perp)
    idx_front = np.where(front)[0]
    return pts[idx_front[best]]
