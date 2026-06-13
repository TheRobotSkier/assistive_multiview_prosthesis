"""keyframe — the Keyframe dataclass stored by the keyframe buffer.

ZERO ROS imports.  Pure numpy + ``dataclasses``.

A ``Keyframe`` bundles everything TSDF fusion and cross-camera features need to
reconstruct an object at grasp time:

- the point cloud (xyz + rgb) in the *world* (``marker_map``) frame,
- the RGB image used for SAM segmentation,
- the camera intrinsics ``K``,
- the GTSAM-optimised camera pose ``T_marker_map_camera``.

Conventions
-----------
- ``cloud_xyz`` / ``cloud_rgb`` are ``(N, 3)`` for *unorganised* clouds (the V6
  default) or ``(H, W, 3)`` for *organised* clouds (an optimisation).
- ``image`` is ``(H, W, 3)`` ``uint8`` RGB.
- ``K`` is ``(3, 3)``.
- ``pose`` is ``(4, 4)`` homogeneous ``T_world_camera``.

Reference: V6 plan §6.2.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

__all__ = ["Keyframe"]


@dataclass(slots=True)
class Keyframe:
    """A single spatially-gated keyframe.

    Parameters
    ----------
    timestamp : float
        Message timestamp in seconds (epoch or ROS time).
    camera_id : str
        ``"head"`` or ``"arm"``.
    cloud_xyz : np.ndarray
        ``(N, 3)`` unorganised or ``(H, W, 3)`` organised, world frame.
    cloud_rgb : np.ndarray
        Matching RGB array ``(N, 3)`` or ``(H, W, 3)`` ``uint8``.
    image : np.ndarray
        ``(H, W, 3)`` ``uint8`` RGB image for SAM.
    K : np.ndarray
        ``(3, 3)`` camera intrinsics.
    pose : np.ndarray
        ``(4, 4)`` ``T_world_camera`` from GTSAM.
    organized : bool
        ``True`` if the cloud is ``(H, W, 3)``.
    """

    timestamp: float
    camera_id: str
    cloud_xyz: np.ndarray
    cloud_rgb: np.ndarray
    image: np.ndarray
    K: np.ndarray
    pose: np.ndarray
    organized: bool = False

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------
    @property
    def memory_mb(self) -> float:
        """Approximate memory footprint of the stored arrays in megabytes.

        Sums ``nbytes`` of ``cloud_xyz``, ``cloud_rgb``, ``image``, ``K`` and
        ``pose``.  Scalar / non-ndarray fields are ignored.
        """
        total = 0
        for arr in (self.cloud_xyz, self.cloud_rgb, self.image, self.K, self.pose):
            if isinstance(arr, np.ndarray):
                total += arr.nbytes
        return total / (1024.0 * 1024.0)

    @property
    def translation(self) -> np.ndarray:
        """Camera translation ``(3,)`` extracted from :attr:`pose`."""
        return np.asarray(self.pose, dtype=np.float64)[:3, 3]

    @property
    def num_points(self) -> int:
        """Number of points in the cloud (``N`` for unorganised, ``H*W`` otherwise)."""
        if self.organized:
            return int(self.cloud_xyz.shape[0]) * int(self.cloud_xyz.shape[1])
        return int(self.cloud_xyz.shape[0])
