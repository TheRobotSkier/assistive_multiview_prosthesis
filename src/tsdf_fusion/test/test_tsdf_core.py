#!/usr/bin/env python3
"""Unit tests for tsdf_fusion_core — pure-logic TSDF fusion pipeline.

The synthetic-scene test (the key de-risking artifact) builds a known 3-D
object, renders it into several synthetic keyframes with known poses, and
verifies the fused cloud recovers the object geometry within 2 cm RMS.

Requires Open3D (run in the container where it is installed).

Usage:
    python3 -m pytest src/tsdf_fusion/test/test_tsdf_core.py -v
"""

from __future__ import annotations

import math

import numpy as np
import pytest

# Skip the entire module if Open3D is not available — the fusion core needs it.
o3d = pytest.importorskip("open3d")

from keyframe_buffer.keyframe import Keyframe  # noqa: E402
from tsdf_fusion.tsdf_fusion_core import (  # noqa: E402
    fuse_object_cloud,
    shift_inward,
)


# ---------------------------------------------------------------------------
# Synthetic scene helpers
# ---------------------------------------------------------------------------

def make_box_surface(center, size, n_per_face=400, rng=None):
    """Sample points on the surface of an axis-aligned box.

    Returns ``(N, 3)`` points in the world frame.
    """
    if rng is None:
        rng = np.random.default_rng(0)
    center = np.asarray(center, dtype=np.float64)
    hx, hy, hz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
    pts = []
    faces = [
        # (axis, sign, u_range, v_range, h_u, h_v)
        (0, +1, (-hy, hy), (-hz, hz), hy, hz),  # +x face
        (0, -1, (-hy, hy), (-hz, hz), hy, hz),  # -x face
        (1, +1, (-hx, hx), (-hz, hz), hx, hz),  # +y face
        (1, -1, (-hx, hx), (-hz, hz), hx, hz),  # -y face
        (2, +1, (-hx, hx), (-hy, hy), hx, hy),  # +z face
        (2, -1, (-hx, hx), (-hy, hy), hx, hy),  # -z face
    ]
    for axis, sign, u_rng, v_rng, hu, hv in faces:
        u = rng.uniform(u_rng[0], u_rng[1], n_per_face)
        v = rng.uniform(v_rng[0], v_rng[1], n_per_face)
        face_pts = np.zeros((n_per_face, 3))
        for i in range(3):
            if i == axis:
                face_pts[:, i] = sign * (size[i] / 2.0)
            elif face_pts[:, i].sum() == 0 and (i not in (axis,)):
                pass
        # Assign the two free axes.
        free = [j for j in range(3) if j != axis]
        face_pts[:, free[0]] = u
        face_pts[:, free[1]] = v
        face_pts[:, axis] = sign * (size[axis] / 2.0)
        pts.append(face_pts)
    return center + np.vstack(pts)


def make_cylinder_surface(center, radius, height, n=2000, rng=None):
    """Sample points on the surface of a cylinder (axis = z).

    Returns ``(N, 3)`` points in the world frame.
    """
    if rng is None:
        rng = np.random.default_rng(0)
    center = np.asarray(center, dtype=np.float64)
    # Side surface
    theta = rng.uniform(0, 2 * math.pi, n)
    z = rng.uniform(-height / 2, height / 2, n)
    side = np.column_stack([
        radius * np.cos(theta),
        radius * np.sin(theta),
        z,
    ])
    # Caps
    n_cap = n // 3
    r_top = np.sqrt(rng.uniform(0, radius ** 2, n_cap))
    th_top = rng.uniform(0, 2 * math.pi, n_cap)
    top = np.column_stack([
        r_top * np.cos(th_top),
        r_top * np.sin(th_top),
        np.full(n_cap, height / 2),
    ])
    bot = top.copy()
    bot[:, 2] = -height / 2
    return center + np.vstack([side, top, bot])


def look_at_pose(cam_pos, target, up=(0, 0, 1)):
    """Build a ``T_world_camera`` pose where the camera at *cam_pos* looks at
    *target*.

    Camera optical convention: +z forward, +x right, +y down.
    """
    cam_pos = np.asarray(cam_pos, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    forward = target - cam_pos
    forward /= np.linalg.norm(forward)

    up = np.asarray(up, dtype=np.float64)
    right = np.cross(forward, up)
    if np.linalg.norm(right) < 1e-6:
        up = np.array([0.0, 1.0, 0.0])
        right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)

    R = np.column_stack([right, down, forward])  # camera-to-world rotation
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = cam_pos
    return T


def render_surface_to_cloud(surface_pts, K, pose, H, W, organized=False):
    """Project *surface_pts* into a camera to form a (synthetic) cloud.

    Only points in front of the camera and inside the image are kept.

    Returns ``(cloud_xyz, cloud_rgb, image, mask)`` where:
    - ``cloud_xyz``: ``(N, 3)`` world-frame points (unorganized) or
      ``(H, W, 3)`` (organized).
    - ``cloud_rgb``: matching colours (constant grey).
    - ``image``: ``(H, W, 3)`` uint8 RGB (black background).
    - ``mask``: ``(H, W)`` bool — the projected object silhouette (ground
      truth for the mock SAM).
    """
    from keyframe_buffer.cloud_utils import project_points_to_pixels

    pts = np.asarray(surface_pts, dtype=np.float64)
    p_cam, uv, in_front = project_points_to_pixels(pts, K, pose)

    u = np.round(uv[in_front, 0]).astype(int)
    v = np.round(uv[in_front, 1]).astype(int)
    valid = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    u, v = u[valid], v[valid]
    idx = np.where(in_front)[0][valid]
    z = p_cam[idx, 2]

    # Z-buffer per pixel (keep nearest).
    mask = np.zeros((H, W), dtype=bool)
    image = np.zeros((H, W, 3), dtype=np.uint8)
    if organized:
        cloud_xyz = np.zeros((H, W, 3), dtype=np.float32)
    else:
        cloud_xyz = np.zeros((0, 3), dtype=np.float32)

    # Build a depth map first.
    depth = np.zeros((H, W), dtype=np.float32)
    order = np.argsort(-z)  # far-to-near
    for o in order:
        depth[v[o], u[o]] = z[o]
        mask[v[o], u[o]] = True
        image[v[o], u[o]] = (180, 180, 180)  # grey object

    if organized:
        # Back-project each filled pixel to world frame.
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        for vv in range(H):
            for uu in range(W):
                d = depth[vv, uu]
                if d > 0:
                    p_c = np.array([(uu - cx) * d / fx,
                                    (vv - cy) * d / fy, d])
                    cloud_xyz[vv, uu] = (pose @ np.append(p_c, 1.0))[:3]
        cloud_rgb = np.tile(image.reshape(H, W, 3), (1, 1, 1)).astype(np.uint8)
    else:
        # Unorganised: collect the z-buffered points.
        filled = np.where(depth > 0)
        vv, uu = filled
        dd = depth[filled]
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        p_cam_arr = np.column_stack([
            (uu - cx) * dd / fx,
            (vv - cy) * dd / fy,
            dd,
        ])
        homog = np.column_stack([p_cam_arr, np.ones(len(dd))])
        cloud_xyz = (pose @ homog.T).T[:, :3].astype(np.float32)
        cloud_rgb = np.tile(np.array([180, 180, 180], dtype=np.uint8),
                            (len(dd), 1))

    return cloud_xyz, cloud_rgb, image, mask


def make_mock_sam(mask_gt):
    """Return a ``sam_segment_fn`` that returns the ground-truth mask.

    The mock ignores the click pixel and dilation — it always returns the
    precomputed object silhouette for the corresponding keyframe.
    """
    # We need per-keyframe masks, so store a list and pop by index.
    masks = list(mask_gt)
    counter = {"i": 0}

    def _sam(image, uv, dilation_px=15):
        i = counter["i"] % len(masks)
        counter["i"] += 1
        m = masks[i].copy()
        # Simulate dilation by expanding the mask slightly.
        if dilation_px > 0:
            try:
                import cv2
                kernel = np.ones(
                    (dilation_px, dilation_px), dtype=np.uint8)
                m = cv2.dilate(m.astype(np.uint8), kernel).astype(bool)
            except Exception:
                pass
        return m

    return _sam


def make_keyframes_from_surface(surface_pts, K, cam_positions, target, H, W,
                                organized=False, rng=None):
    """Render *surface_pts* from several camera positions into Keyframes."""
    if rng is None:
        rng = np.random.default_rng(0)
    keyframes = []
    masks = []
    for i, cp in enumerate(cam_positions):
        pose = look_at_pose(cp, target)
        xyz, rgb, image, mask = render_surface_to_cloud(
            surface_pts, K, pose, H, W, organized=organized)
        kf = Keyframe(
            timestamp=float(i),
            camera_id="head",
            cloud_xyz=xyz,
            cloud_rgb=rgb,
            image=image,
            K=K,
            pose=pose,
            organized=organized,
        )
        keyframes.append(kf)
        masks.append(mask)
    return keyframes, masks


def rms_to_surface(cloud_xyz, surface_pts):
    """Compute the RMS distance from each cloud point to the nearest surface
    point (one-way nearest-neighbour)."""
    from scipy.spatial import cKDTree

    if cloud_xyz.shape[0] == 0:
        return float("inf")
    tree = cKDTree(surface_pts)
    dists, _ = tree.query(cloud_xyz, k=1)
    return float(np.sqrt(np.mean(dists ** 2)))


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def intrinsics():
    fx = fy = 400.0
    cx, cy = 160.0, 120.0
    return np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ])


@pytest.fixture
def img_dims():
    return 240, 320  # H, W


# ---------------------------------------------------------------------------
# shift_inward
# ---------------------------------------------------------------------------

class TestShiftInward:
    def test_basic_shift(self):
        pose = np.eye(4)
        pose[:3, 3] = [0.0, 0.0, 0.5]  # camera at z=0.5
        hit = np.array([0.0, 0.0, 0.4])
        shifted = shift_inward(hit, pose, 0.015)
        # Should move toward +z (toward camera at 0.5).
        assert shifted[2] > hit[2]
        assert abs(np.linalg.norm(shifted - hit) - 0.015) < 1e-9

    def test_zero_offset(self):
        pose = np.eye(4)
        pose[:3, 3] = [1.0, 0.0, 0.0]
        hit = np.array([0.5, 0.0, 0.0])
        shifted = shift_inward(hit, pose, 0.0)
        np.testing.assert_allclose(shifted, hit)

    def test_coincident(self):
        """Hit point == camera origin → no shift (avoid div by zero)."""
        pose = np.eye(4)
        pose[:3, 3] = [0.0, 0.0, 0.0]
        hit = np.array([0.0, 0.0, 0.0])
        shifted = shift_inward(hit, pose, 0.015)
        np.testing.assert_allclose(shifted, hit)


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------

class TestEdgeCases:
    def test_empty_keyframe_list(self, intrinsics):
        """Empty keyframe list → empty cloud, success=False."""
        result = fuse_object_cloud(
            keyframes=[],
            hit_point_3d=np.array([0.0, 0.0, 0.0]),
            K_click=intrinsics,
            pose_click=np.eye(4),
            sam_segment_fn=lambda img, uv, d: np.zeros(
                (10, 10), dtype=bool),
        )
        assert result.success is False
        assert result.xyz.shape[0] == 0
        assert "empty" in result.message.lower()

    def test_all_behind_camera(self, intrinsics, img_dims):
        """All keyframes have no points in front of the camera → empty result."""
        H, W = img_dims
        # A keyframe whose cloud is entirely behind it.
        pose = np.eye(4)
        pose[:3, 3] = [0.0, 0.0, 1.0]  # camera at z=1
        # Cloud points at z=-1 (behind).
        cloud = np.array([[0.0, 0.0, -1.0], [0.1, 0.1, -1.0]])
        kf = Keyframe(
            timestamp=0.0, camera_id="head",
            cloud_xyz=cloud,
            cloud_rgb=np.zeros((2, 3), dtype=np.uint8),
            image=np.zeros((H, W, 3), dtype=np.uint8),
            K=intrinsics, pose=pose, organized=False)
        result = fuse_object_cloud(
            keyframes=[kf],
            hit_point_3d=np.array([0.0, 0.0, 0.5]),
            K_click=intrinsics,
            pose_click=pose,
            sam_segment_fn=lambda img, uv, d: np.ones((H, W), dtype=bool),
        )
        assert result.success is False
        assert result.xyz.shape[0] == 0

    def test_sam_returns_empty_mask(self, intrinsics, img_dims):
        """SAM returns an all-False mask → no integration, empty result."""
        H, W = img_dims
        pose = np.eye(4)
        pose[:3, 3] = [0.0, 0.0, 0.5]
        cloud = np.array([[0.0, 0.0, 0.4]])
        kf = Keyframe(
            timestamp=0.0, camera_id="head",
            cloud_xyz=cloud,
            cloud_rgb=np.zeros((1, 3), dtype=np.uint8),
            image=np.zeros((H, W, 3), dtype=np.uint8),
            K=intrinsics, pose=pose, organized=False)
        result = fuse_object_cloud(
            keyframes=[kf],
            hit_point_3d=np.array([0.0, 0.0, 0.4]),
            K_click=intrinsics,
            pose_click=pose,
            sam_segment_fn=lambda img, uv, d: np.zeros((H, W), dtype=bool),
        )
        assert result.success is False

    def test_hit_outside_fov(self, intrinsics, img_dims):
        """Hit point outside all keyframe FOVs → graceful empty handling."""
        H, W = img_dims
        pose = np.eye(4)
        pose[:3, 3] = [0.0, 0.0, 0.5]
        cloud = np.array([[0.0, 0.0, 0.4]])
        kf = Keyframe(
            timestamp=0.0, camera_id="head",
            cloud_xyz=cloud,
            cloud_rgb=np.zeros((1, 3), dtype=np.uint8),
            image=np.zeros((H, W, 3), dtype=np.uint8),
            K=intrinsics, pose=pose, organized=False)
        # Hit point far off to the side (projects outside the image).
        result = fuse_object_cloud(
            keyframes=[kf],
            hit_point_3d=np.array([100.0, 100.0, 0.4]),
            K_click=intrinsics,
            pose_click=pose,
            sam_segment_fn=lambda img, uv, d: np.ones((H, W), dtype=bool),
        )
        # Should not crash; either empty or a small cloud.
        assert isinstance(result.success, bool)


# ---------------------------------------------------------------------------
# THE synthetic scene test — the key de-risking artifact
# ---------------------------------------------------------------------------

class TestSyntheticScene:
    def test_box_recovery_unorganized(self, intrinsics, img_dims):
        """Fuse a synthetic box from multiple viewpoints and verify the fused
        cloud recovers the surface within 2 cm RMS."""
        H, W = img_dims
        rng = np.random.default_rng(42)

        # Ground-truth object: a 6cm box at the origin.
        box_center = np.array([0.0, 0.0, 0.0])
        box_size = np.array([0.06, 0.06, 0.06])
        surface = make_box_surface(box_center, box_size, n_per_face=600,
                                   rng=rng)

        # Camera positions around the box (radius ~30 cm).
        target = box_center
        cam_positions = [
            [0.30, 0.0, 0.20],
            [-0.30, 0.0, 0.20],
            [0.0, 0.30, 0.20],
            [0.0, -0.30, 0.20],
            [0.20, 0.20, 0.30],
            [-0.20, -0.20, 0.30],
        ]

        keyframes, masks = make_keyframes_from_surface(
            surface, intrinsics, cam_positions, target, H, W,
            organized=False, rng=rng)

        assert len(keyframes) == 6
        # Sanity: at least some keyframes have points.
        total_pts = sum(kf.cloud_xyz.shape[0] for kf in keyframes)
        assert total_pts > 0, "Synthetic clouds should contain points"

        mock_sam = make_mock_sam(masks)

        result = fuse_object_cloud(
            keyframes=keyframes,
            hit_point_3d=box_center,
            K_click=intrinsics,
            pose_click=keyframes[0].pose,
            sam_segment_fn=mock_sam,
            voxel_size=0.005,
            sdf_trunc=0.02,
            dbscan_eps=0.02,
            dbscan_min_points=10,
            hit_point_shift=0.015,
            mask_dilation=3,
        )

        assert result.success, f"Fusion failed: {result.message}"
        assert result.xyz.shape[0] > 0, "Fused cloud should be non-empty"

        rms = rms_to_surface(result.xyz, surface)
        # The verification gate requires < 2 cm RMS.
        assert rms < 0.02, (
            f"Fused cloud RMS {rms * 100:.2f} cm exceeds 2 cm tolerance. "
            f"Points: {result.xyz.shape[0]}")

    def test_cylinder_recovery_unorganized(self, intrinsics, img_dims):
        """Fuse a synthetic cylinder and verify recovery within 2 cm RMS."""
        H, W = img_dims
        rng = np.random.default_rng(7)

        cyl_center = np.array([0.0, 0.0, 0.0])
        surface = make_cylinder_surface(
            cyl_center, radius=0.03, height=0.08, n=3000, rng=rng)

        target = cyl_center
        cam_positions = [
            [0.25, 0.0, 0.15],
            [-0.25, 0.0, 0.15],
            [0.0, 0.25, 0.15],
            [0.0, -0.25, 0.15],
            [0.18, 0.18, 0.25],
        ]

        keyframes, masks = make_keyframes_from_surface(
            surface, intrinsics, cam_positions, target, H, W,
            organized=False, rng=rng)

        mock_sam = make_mock_sam(masks)

        result = fuse_object_cloud(
            keyframes=keyframes,
            hit_point_3d=cyl_center,
            K_click=intrinsics,
            pose_click=keyframes[0].pose,
            sam_segment_fn=mock_sam,
            voxel_size=0.005,
            sdf_trunc=0.02,
            dbscan_eps=0.02,
            dbscan_min_points=10,
            hit_point_shift=0.015,
            mask_dilation=3,
        )

        assert result.success, f"Fusion failed: {result.message}"
        assert result.xyz.shape[0] > 0

        rms = rms_to_surface(result.xyz, surface)
        assert rms < 0.02, (
            f"Fused cloud RMS {rms * 100:.2f} cm exceeds 2 cm tolerance.")

    def test_organized_fast_path(self, intrinsics, img_dims):
        """The organised fast-path should also recover the object."""
        H, W = img_dims
        rng = np.random.default_rng(11)

        box_center = np.array([0.0, 0.0, 0.0])
        box_size = np.array([0.06, 0.06, 0.06])
        surface = make_box_surface(box_center, box_size, n_per_face=600,
                                   rng=rng)

        target = box_center
        cam_positions = [
            [0.30, 0.0, 0.20],
            [0.0, 0.30, 0.20],
            [0.20, 0.20, 0.30],
        ]

        keyframes, masks = make_keyframes_from_surface(
            surface, intrinsics, cam_positions, target, H, W,
            organized=True, rng=rng)

        # Organised clouds are (H, W, 3).
        assert keyframes[0].cloud_xyz.shape == (H, W, 3)
        assert keyframes[0].organized is True

        mock_sam = make_mock_sam(masks)

        result = fuse_object_cloud(
            keyframes=keyframes,
            hit_point_3d=box_center,
            K_click=intrinsics,
            pose_click=keyframes[0].pose,
            sam_segment_fn=mock_sam,
            voxel_size=0.005,
            sdf_trunc=0.02,
            dbscan_eps=0.02,
            dbscan_min_points=10,
            hit_point_shift=0.015,
            mask_dilation=3,
        )

        assert result.success, f"Fusion failed: {result.message}"
        assert result.xyz.shape[0] > 0

        rms = rms_to_surface(result.xyz, surface)
        assert rms < 0.02, (
            f"Fused cloud RMS {rms * 100:.2f} cm exceeds 2 cm tolerance.")
