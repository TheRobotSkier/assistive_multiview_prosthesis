"""umeyama — SVD-based 3-D to 3-D point cloud alignment (pure numpy).

Given two sets of corresponding 3-D points, computes the rigid transform
(rotation + translation) that best aligns the source to the destination.

The algorithm:
1. Compute centroids of both point sets.
2. Compute the cross-covariance matrix ``H``.
3. SVD of ``H`` to find the optimal rotation.
4. Translation from the rotation and centroid difference.

Also returns an approximate 6×6 covariance for the estimated transform,
derived from the RMS residual.

Reference: V6 plan §3.1.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

__all__ = ["umeyama"]


def umeyama(
    src_points: np.ndarray,
    dst_points: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute the rigid transform aligning *src_points* to *dst_points*.

    Parameters
    ----------
    src_points : ndarray (N, 3)
        Source 3-D points.
    dst_points : ndarray (N, 3)
        Destination 3-D points (same length as *src_points*).

    Returns
    -------
    T : ndarray (4, 4)
        Homogeneous transform such that ``dst ≈ (T @ [src; 1]^T)^T``.
    covariance : ndarray (6, 6)
        Approximate diagonal covariance ``[sx2, sy2, sz2, sr2, sp2, sy2]``
        estimated from the RMS residual.  Translation variances are in m²,
        rotation variances are in rad².
    """
    src = np.asarray(src_points, dtype=np.float64)
    dst = np.asarray(dst_points, dtype=np.float64)

    if src.shape != dst.shape:
        raise ValueError(
            f"src and dst must have the same shape, got {src.shape} vs {dst.shape}"
        )
    if src.ndim != 2 or src.shape[1] != 3:
        raise ValueError(f"points must be (N, 3), got {src.shape}")

    N = src.shape[0]
    if N < 3:
        raise ValueError(f"Need at least 3 correspondences, got {N}")

    # Step 1: centroids
    src_centroid = np.mean(src, axis=0)
    dst_centroid = np.mean(dst, axis=0)

    src_centered = src - src_centroid
    dst_centered = dst - dst_centroid

    # Step 2: cross-covariance matrix
    H = src_centered.T @ dst_centered  # (3, 3)

    # Step 3: SVD
    U, S, Vt = np.linalg.svd(H)

    # Handle reflection: if det(V @ U^T) < 0, flip the last column of V.
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])

    R = Vt.T @ D @ U.T

    # Step 4: translation
    t = dst_centroid - R @ src_centroid

    # Build 4×4 transform
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    # Step 5: estimate covariance from residuals
    aligned = (R @ src.T).T + t  # (N, 3)
    residuals = dst - aligned     # (N, 3)
    rms = np.sqrt(np.mean(np.sum(residuals ** 2, axis=1)))

    # Approximate covariance: translation sigma ~ rms, rotation sigma ~ rms / typical_distance
    # Use the mean distance of dst points from centroid as the scale.
    distances = np.linalg.norm(dst_centered, axis=1)
    mean_dist = np.mean(distances) if np.mean(distances) > 1e-6 else 1.0

    sigma_t = rms
    sigma_r = rms / max(mean_dist, 1e-6)

    cov = np.diag([sigma_t ** 2] * 3 + [sigma_r ** 2] * 3)

    return T, cov
