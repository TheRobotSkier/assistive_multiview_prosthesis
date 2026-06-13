"""tsdf_fusion_core — pure-logic TSDF fusion pipeline (NO ROS imports).

This is the heart of the V6 plan (§5.6, §6.5).  It is extracted as a pure-Python
function so it can be tested without ROS, using a synthetic scene and a mock SAM
segmentation callable.

The only heavy dependency is **Open3D** (for the ``ScalableTSDFVolume``), which is
imported lazily inside :func:`fuse_object_cloud` so that the pure-numpy helpers
(:func:`shift_inward`) remain importable / testable on a host without Open3D.

Conventions (identical to :mod:`keyframe_buffer.cloud_utils`):

- ``cloud_xyz`` for unorganised clouds: ``(N, 3)`` in the *world* frame.
- ``cloud_xyz`` for organised clouds: ``(H, W, 3)``.
- ``pose``: ``(4, 4)`` homogeneous ``T_world_camera`` (``p_world = pose @ p_camera``).
- ``K``: ``(3, 3)`` intrinsics.

Reference: V6 plan §4.4, §6.5.
"""

from __future__ import annotations

from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

# Pure-numpy cloud helpers from the keyframe_buffer package (Phase 1 Task B).
# These modules have ZERO ROS imports.
from keyframe_buffer.cloud_utils import (
    build_depth_image,
    mask_organized_cloud,
    mask_unorganized_cloud,
    project_3d_to_2d,
)

__all__ = [
    "shift_inward",
    "fuse_object_cloud",
    "FuseResult",
]

# Type alias for the SAM segmentation callable.
#   sam_segment_fn(image: ndarray (H,W,3) uint8, uv: tuple(int,int),
#                  dilation_px: int) -> ndarray (H,W) bool
SamSegmentFn = Callable[[np.ndarray, Tuple[int, int], int], np.ndarray]


# ---------------------------------------------------------------------------
# Step 1 — shift the hit point inward along the camera viewing ray
# ---------------------------------------------------------------------------

def shift_inward(
    hit_point_3d: np.ndarray,
    pose: np.ndarray,
    offset: float = 0.015,
) -> np.ndarray:
    """Shift *hit_point_3d* toward the camera origin by *offset* metres.

    The viewing ray is ``(camera_origin - hit_point)`` normalised.  Moving the
    hit point along this ray biases the 2-D SAM prompt toward the object
    interior (V6 §4.4).

    Parameters
    ----------
    hit_point_3d : ndarray (3,)
        Hit point in the world frame.
    pose : ndarray (4, 4)
        ``T_world_camera`` of the camera that observed the click.
    offset : float
        Shift distance in metres (default 1.5 cm).

    Returns
    -------
    hit_internal : ndarray (3,)
        The shifted hit point.
    """
    p = np.asarray(hit_point_3d, dtype=np.float64).reshape(3)
    cam_origin = np.asarray(pose, dtype=np.float64)[:3, 3]
    ray = cam_origin - p
    norm = float(np.linalg.norm(ray))
    if norm < 1e-9:
        return p.copy()
    ray = ray / norm
    return p + ray * float(offset)


# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------

class FuseResult(Tuple):
    """Named-tuple-like result returned by :func:`fuse_object_cloud`.

    Fields
    ------
    xyz : ndarray (M, 3) float32
        Cleaned object-cloud points in the world frame (empty if fusion failed).
    rgb : ndarray (M, 3) uint8
        Matching per-point colours.
    success : bool
        ``True`` when a non-empty cloud was produced.
    message : str
        Human-readable status / error message.
    """

    __slots__ = ()

    def __new__(cls, xyz: np.ndarray, rgb: np.ndarray,
                success: bool, message: str):
        return tuple.__new__(cls, (xyz, rgb, success, message))

    @property
    def xyz(self) -> np.ndarray:
        return self[0]

    @property
    def rgb(self) -> np.ndarray:
        return self[1]

    @property
    def success(self) -> bool:
        return self[2]

    @property
    def message(self) -> str:
        return self[3]

    def __repr__(self) -> str:
        n = 0 if self.xyz is None else int(np.asarray(self.xyz).shape[0])
        return (f"FuseResult(success={self.success}, num_points={n}, "
                f"message={self.message!r})")


# ---------------------------------------------------------------------------
# The main fusion pipeline
# ---------------------------------------------------------------------------

def fuse_object_cloud(
    keyframes: Sequence,
    hit_point_3d: np.ndarray,
    K_click: Optional[np.ndarray],
    pose_click: Optional[np.ndarray],
    sam_segment_fn: SamSegmentFn,
    voxel_size: float = 0.005,
    sdf_trunc: float = 0.02,
    dbscan_eps: float = 0.02,
    dbscan_min_points: int = 10,
    hit_point_shift: float = 0.015,
    mask_dilation: int = 15,
) -> FuseResult:
    """Pure-logic TSDF fusion.  No ROS imports.

    Parameters
    ----------
    keyframes : list of Keyframe
        Keyframes (from the keyframe buffer) to fuse.  Each must expose
        ``.cloud_xyz``, ``.cloud_rgb``, ``.image``, ``.K``, ``.pose``,
        ``.organized`` (matching :class:`keyframe_buffer.keyframe.Keyframe`).
    hit_point_3d : ndarray (3,)
        Hit point in the world frame.
    K_click, pose_click : ndarray or None
        Intrinsics ``(3, 3)`` and pose ``(4, 4)`` of the camera that observed
        the click.  Used to compute the inward shift.  If ``None`` the shift is
        skipped (the unshifted hit point is used for SAM prompts).
    sam_segment_fn : callable
        ``sam_segment_fn(image, uv, dilation_px) -> (H, W) bool mask``.
    voxel_size : float
        TSDF voxel length in metres.
    sdf_trunc : float
        TSDF truncation distance in metres.
    dbscan_eps : float
        DBSCAN neighbourhood radius (metres) for cleanup.
    dbscan_min_points : int
        DBSCAN minimum cluster size.
    hit_point_shift : float
        Inward shift distance (metres) for the SAM prompt.
    mask_dilation : int
        Dilation (pixels) passed to ``sam_segment_fn``.

    Returns
    -------
    FuseResult
        ``(xyz (M,3) float32, rgb (M,3) uint8, success, message)``.
    """
    # ── Open3D is required for the TSDF volume — import lazily ──────────
    import open3d as o3d  # noqa: WPS433 — lazy import on purpose

    empty = FuseResult(
        np.zeros((0, 3), dtype=np.float32),
        np.zeros((0, 3), dtype=np.uint8),
        False, "no keyframes",
    )

    # ── Step 0 — guard: empty keyframe list ─────────────────────────────
    if keyframes is None or len(keyframes) == 0:
        return FuseResult(
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 3), dtype=np.uint8),
            False, "empty keyframe list",
        )

    # ── Step 1 — shift hit point inward along the click camera's ray ────
    hit = np.asarray(hit_point_3d, dtype=np.float64).reshape(3)
    if pose_click is not None and hit_point_shift > 0.0:
        hit_internal = shift_inward(hit, pose_click, hit_point_shift)
    else:
        hit_internal = hit.copy()

    # ── Step 3 — initialise the TSDF volume ─────────────────────────────
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=float(voxel_size),
        sdf_trunc=float(sdf_trunc),
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    integrated_count = 0

    # ── Step 4 — per-keyframe loop (V6 §6.5, unorganised-safe) ──────────
    for idx, kf in enumerate(keyframes):
        K = np.asarray(kf.K, dtype=np.float64).reshape(3, 3)
        pose = np.asarray(kf.pose, dtype=np.float64).reshape(4, 4)
        image = np.asarray(kf.image)
        H, W = int(image.shape[0]), int(image.shape[1])

        # 4a. Project the shifted hit point to 2-D for the SAM prompt.
        uv = project_3d_to_2d(hit_internal, K, pose)
        if uv == (-1, -1):
            # Hit point behind this camera — skip it.
            continue

        # 4b. Run SAM to get the 2-D object mask.
        try:
            mask = sam_segment_fn(image, uv, int(mask_dilation))
        except Exception as exc:  # pragma: no cover — SAM failures are logged
            # A failing SAM call must not abort the whole fusion.
            mask = np.zeros((H, W), dtype=bool)
            _ = exc  # swallowed; the node logs per-keyframe errors

        if mask is None or not np.any(mask):
            continue

        # 4c. Mask the cloud — organisation-aware.
        if kf.organized:
            cloud_xyz = np.asarray(kf.cloud_xyz)
            cloud_rgb = np.asarray(kf.cloud_rgb)
            masked_xyz = cloud_xyz[mask.astype(bool)]
            masked_rgb = cloud_rgb[mask.astype(bool)].reshape(-1, 3)
        else:
            cloud_xyz = np.asarray(kf.cloud_xyz).reshape(-1, 3)
            cloud_rgb = np.asarray(kf.cloud_rgb).reshape(-1, 3)
            keep = mask_unorganized_cloud(cloud_xyz, mask, K, pose)
            masked_xyz = cloud_xyz[keep]
            masked_rgb = cloud_rgb[keep]

        if masked_xyz.shape[0] == 0:
            continue

        # 4d. Build a depth image by rasterising the masked points.
        depth = build_depth_image(masked_xyz, K, pose, H, W)
        if not np.any(depth > 0):
            continue

        # 4e. Create the RGBD image (RGB from keyframe, depth from cloud).
        rgb_image = np.ascontiguousarray(image.astype(np.uint8))
        # Ensure depth is float32 for Open3D.
        depth_f32 = np.ascontiguousarray(depth.astype(np.float32))
        o3d_rgb = o3d.geometry.Image(rgb_image)
        o3d_depth = o3d.geometry.Image(depth_f32)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_rgb, o3d_depth,
            depth_scale=1.0,
            depth_trunc=float(max(sdf_trunc * 10.0, 0.5)),
            convert_rgb_to_intensity=False,
        )

        # 4f. Integrate into the TSDF volume.
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            W, H,
            float(K[0, 0]), float(K[1, 1]),
            float(K[0, 2]), float(K[1, 2]),
        )
        # Open3D expects the *extrinsic* as T_camera_world = inv(T_world_camera).
        extrinsic = np.linalg.inv(pose)
        volume.integrate(rgbd, intrinsic, extrinsic)
        integrated_count += 1

    # ── No keyframe integrated anything → empty result ──────────────────
    if integrated_count == 0:
        return FuseResult(
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 3), dtype=np.uint8),
            False, "no keyframe produced visible geometry",
        )

    # ── Step 5 — extract mesh → point cloud ─────────────────────────────
    mesh = volume.extract_triangle_mesh()
    pts = np.asarray(mesh.vertices, dtype=np.float32)
    cols = np.asarray(mesh.vertex_colors, dtype=np.float32)

    if pts.shape[0] == 0:
        return FuseResult(
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0, 3), dtype=np.uint8),
            False, "TSDF extraction yielded no vertices",
        )

    # ── Step 6 — DBSCAN cleanup (V6 §4.4) ───────────────────────────────
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(pts)
    o3d_cloud.colors = o3d.utility.Vector3dVector(cols)

    labels = np.array(
        o3d_cloud.cluster_dbscan(
            eps=float(dbscan_eps),
            min_points=int(dbscan_min_points),
            print_progress=False,
        ),
        dtype=np.int64,
    )

    if labels.size == 0:
        # No clustering result — return the raw extracted cloud.
        rgb_out = np.clip(cols * 255.0, 0, 255).astype(np.uint8)
        return FuseResult(
            np.ascontiguousarray(pts),
            rgb_out,
            True,
            f"fused {pts.shape[0]} points from {integrated_count} keyframes "
            f"(no DBSCAN clusters)",
        )

    # Remove noise (label == -1) and keep the largest cluster near the hit.
    valid = labels >= 0
    if not np.any(valid):
        # Everything classified as noise — fall back to the raw cloud.
        rgb_out = np.clip(cols * 255.0, 0, 255).astype(np.uint8)
        return FuseResult(
            np.ascontiguousarray(pts),
            rgb_out,
            True,
            f"fused {pts.shape[0]} points from {integrated_count} keyframes "
            f"(all DBSCAN noise, kept raw)",
        )

    unique_labels, counts = np.unique(labels[valid], return_counts=True)

    # Prefer the cluster whose centroid is closest to the (shifted) hit point.
    best_label = None
    best_dist = np.inf
    for lbl in unique_labels:
        members = pts[labels == lbl]
        centroid = members.mean(axis=0)
        d = float(np.linalg.norm(centroid - hit_internal))
        if d < best_dist:
            best_dist = d
            best_label = lbl

    if best_label is None:
        # Fallback: largest cluster.
        best_label = int(unique_labels[np.argmax(counts)])

    keep_mask = labels == best_label
    clean_xyz = np.ascontiguousarray(pts[keep_mask])
    clean_cols = cols[keep_mask]
    clean_rgb = np.clip(clean_cols * 255.0, 0, 255).astype(np.uint8)

    # ── Step 7 — return the cleaned cloud ───────────────────────────────
    return FuseResult(
        clean_xyz,
        clean_rgb,
        True,
        f"fused {clean_xyz.shape[0]} points from {integrated_count} keyframes",
    )
