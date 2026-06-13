#!/usr/bin/env python3
"""Unit tests for cloud_utils — pure numpy point cloud handling.

These tests verify the unorganised-safe cloud utilities with synthetic data.
No ROS dependencies are required.

Usage:
    python3 -m pytest src/keyframe_buffer/test/test_cloud_utils.py -v
"""

import numpy as np
import pytest

from keyframe_buffer.cloud_utils import (
    detect_organization,
    mask_unorganized_cloud,
    mask_organized_cloud,
    build_depth_image,
    project_3d_to_2d,
    lookup_depth_3d,
    project_points_to_pixels,
)


# ---------------------------------------------------------------------------
# Fixtures / helpers
# ---------------------------------------------------------------------------

@pytest.fixture
def intrinsics():
    """A simple pinhole camera at 640x480."""
    fx = fy = 500.0
    cx, cy = 320.0, 240.0
    return np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ])


@pytest.fixture
def identity_pose():
    """Camera at world origin looking down +z."""
    return np.eye(4)


@pytest.fixture
def img_dims():
    return 480, 640  # H, W


# ---------------------------------------------------------------------------
# detect_organization
# ---------------------------------------------------------------------------

class TestDetectOrganization:
    def test_unorganized(self):
        assert detect_organization(1) is False

    def test_organized(self):
        assert detect_organization(480) is True

    def test_boundary(self):
        # height == 1 is unorganised; height == 2 is organised
        assert detect_organization(1) is False
        assert detect_organization(2) is True


# ---------------------------------------------------------------------------
# Projection round-trip
# ---------------------------------------------------------------------------

class TestProjectionRoundTrip:
    def test_projection_round_trip(self, intrinsics, identity_pose):
        """Generate random 3-D points in front of the camera, project to 2-D,
        and verify the re-projection error is < 0.5 px."""
        rng = np.random.default_rng(42)
        N = 500
        # Random points in front of camera: z in [0.5, 5], x,y in [-2, 2]
        pts_cam = np.column_stack([
            rng.uniform(-2, 2, N),
            rng.uniform(-2, 2, N),
            rng.uniform(0.5, 5.0, N),
        ])

        # Since pose is identity, world == camera frame.
        _, uv, in_front = project_points_to_pixels(pts_cam, intrinsics, identity_pose)

        assert np.all(in_front), "All points should be in front of camera"

        # Re-project: from pixel + depth back to 3-D
        fx, fy = intrinsics[0, 0], intrinsics[1, 1]
        cx, cy = intrinsics[0, 2], intrinsics[1, 2]
        z = pts_cam[:, 2]
        x_reproj = (uv[:, 0] - cx) * z / fx
        y_reproj = (uv[:, 1] - cy) * z / fy

        err = np.sqrt((x_reproj - pts_cam[:, 0]) ** 2 + (y_reproj - pts_cam[:, 1]) ** 2)
        assert np.max(err) < 0.5, f"Max reprojection error {np.max(err):.4f} m exceeds 0.5"

    def test_single_point_projection(self, intrinsics, identity_pose):
        """A point directly in front of the camera at the optical centre should
        project to the principal point (cx, cy)."""
        pt = np.array([[0.0, 0.0, 2.0]])
        _, uv, in_front = project_points_to_pixels(pt, intrinsics, identity_pose)
        assert in_front[0]
        assert abs(uv[0, 0] - 320.0) < 0.5
        assert abs(uv[0, 1] - 240.0) < 0.5

    def test_project_3d_to_2d_behind_camera(self, intrinsics, identity_pose):
        """A point behind the camera should return (-1, -1)."""
        pt = np.array([0.0, 0.0, -1.0])
        u, v = project_3d_to_2d(pt, intrinsics, identity_pose)
        assert (u, v) == (-1, -1)


# ---------------------------------------------------------------------------
# Unorganised masking
# ---------------------------------------------------------------------------

class TestUnorganizedMasking:
    def test_half_image_mask(self, intrinsics, identity_pose, img_dims):
        """Create points in front of the camera; mask covers the top half of
        the image.  Verify the keep array matches expected geometry."""
        H, W = img_dims
        rng = np.random.default_rng(99)
        N = 2000
        # Points in camera frame spanning the FOV
        pts_cam = np.column_stack([
            rng.uniform(-2, 2, N),
            rng.uniform(-2, 2, N),
            rng.uniform(0.5, 5.0, N),
        ])

        mask = np.zeros((H, W), dtype=bool)
        mask[:H // 2, :] = True  # top half

        keep = mask_unorganized_cloud(pts_cam, mask, intrinsics, identity_pose)

        # Verify: kept points must project to top half (v < H//2)
        _, uv, in_front = project_points_to_pixels(pts_cam, intrinsics, identity_pose)
        v_px = uv[:, 1].astype(int)
        v_px = np.clip(v_px, 0, H - 1)

        # Every kept point should have v < H//2
        kept_v = v_px[keep]
        assert np.all(kept_v < H // 2), "Some kept points project to bottom half"

        # Every not-kept point that projects inside the image should have v >= H//2
        inside = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
        not_kept_inside = ~keep & inside
        if np.any(not_kept_inside):
            not_kept_v = v_px[not_kept_inside]
            assert np.all(not_kept_v >= H // 2), "Some rejected points project to top half"

    def test_empty_cloud(self, intrinsics, identity_pose, img_dims):
        H, W = img_dims
        mask = np.ones((H, W), dtype=bool)
        cloud = np.zeros((0, 3), dtype=np.float32)
        keep = mask_unorganized_cloud(cloud, mask, intrinsics, identity_pose)
        assert keep.shape == (0,)
        assert keep.dtype == bool

    def test_all_behind_camera(self, intrinsics, identity_pose, img_dims):
        """All points behind the camera → empty keep array."""
        H, W = img_dims
        mask = np.ones((H, W), dtype=bool)
        cloud = np.array([
            [1.0, 1.0, -2.0],
            [-1.0, 0.5, -0.1],
        ])
        keep = mask_unorganized_cloud(cloud, mask, intrinsics, identity_pose)
        assert not np.any(keep)


# ---------------------------------------------------------------------------
# Depth rasterisation
# ---------------------------------------------------------------------------

class TestDepthRasterization:
    def test_known_depths(self, intrinsics, identity_pose, img_dims):
        """Generate points at known depths, rasterise, verify depth values."""
        H, W = img_dims
        # A single point at (0, 0, 3) projects to centre pixel
        cloud = np.array([[0.0, 0.0, 3.0]])
        depth = build_depth_image(cloud, intrinsics, identity_pose, H, W)
        u_c, v_c = 320, 240
        assert abs(depth[v_c, u_c] - 3.0) < 0.001  # within 1 mm

    def test_z_buffer_nearest_wins(self, intrinsics, identity_pose, img_dims):
        """Two points project to the same pixel at different depths — the
        nearest one must be kept."""
        H, W = img_dims
        # Two points along the optical axis: near (z=1) and far (z=5)
        cloud = np.array([
            [0.0, 0.0, 5.0],
            [0.0, 0.0, 1.0],
        ])
        depth = build_depth_image(cloud, intrinsics, identity_pose, H, W)
        u_c, v_c = 320, 240
        assert abs(depth[v_c, u_c] - 1.0) < 0.001, "Nearest point should win z-buffer"

    def test_empty_cloud(self, intrinsics, identity_pose, img_dims):
        H, W = img_dims
        cloud = np.zeros((0, 3), dtype=np.float32)
        depth = build_depth_image(cloud, intrinsics, identity_pose, H, W)
        assert depth.shape == (H, W)
        assert np.all(depth == 0.0)

    def test_all_behind_camera(self, intrinsics, identity_pose, img_dims):
        H, W = img_dims
        cloud = np.array([[1.0, 1.0, -2.0]])
        depth = build_depth_image(cloud, intrinsics, identity_pose, H, W)
        assert np.all(depth == 0.0)

    def test_depth_image_shape(self, intrinsics, identity_pose, img_dims):
        H, W = img_dims
        rng = np.random.default_rng(7)
        N = 500
        pts = np.column_stack([
            rng.uniform(-1, 1, N),
            rng.uniform(-1, 1, N),
            rng.uniform(0.5, 3.0, N),
        ])
        depth = build_depth_image(pts, intrinsics, identity_pose, H, W)
        assert depth.shape == (H, W)
        assert depth.dtype == np.float32


# ---------------------------------------------------------------------------
# Organised fast-path
# ---------------------------------------------------------------------------

class TestOrganizedFastPath:
    def test_mask_shape_and_values(self, img_dims):
        H, W = img_dims
        cloud = np.zeros((H, W, 3), dtype=np.float32)
        # Fill with identifiable values: x = row, y = col, z = row+col
        for r in range(H):
            for c in range(W):
                cloud[r, c] = [r, c, r + c]

        mask = np.zeros((H, W), dtype=bool)
        mask[10:20, 30:40] = True

        result = mask_organized_cloud(cloud, mask)
        assert result.shape[0] == 100  # 10x10 region
        # Verify the values are correct
        expected = cloud[10:20, 30:40, :].reshape(-1, 3)
        np.testing.assert_array_equal(result, expected)


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------

class TestEdgeCases:
    def test_image_boundary_points(self, intrinsics, identity_pose, img_dims):
        """Points that project exactly to the image boundary should be kept."""
        H, W = img_dims
        fx, fy = intrinsics[0, 0], intrinsics[1, 1]
        cx, cy = intrinsics[0, 2], intrinsics[1, 2]
        z = 1.0
        # Point that projects to (0, 0)
        x = (0 - cx) * z / fx
        y = (0 - cy) * z / fy
        cloud = np.array([[x, y, z]])
        mask = np.ones((H, W), dtype=bool)
        keep = mask_unorganized_cloud(cloud, mask, intrinsics, identity_pose)
        assert keep[0], "Point at image corner should be kept"

    def test_point_outside_image(self, intrinsics, identity_pose, img_dims):
        """A point projecting outside the image should not be kept."""
        H, W = img_dims
        z = 1.0
        # Point far off to the side
        cloud = np.array([[100.0, 0.0, z]])
        mask = np.ones((H, W), dtype=bool)
        keep = mask_unorganized_cloud(cloud, mask, intrinsics, identity_pose)
        assert not keep[0]


# ---------------------------------------------------------------------------
# lookup_depth_3d
# ---------------------------------------------------------------------------

class TestLookupDepth3D:
    def test_organized_lookup(self, img_dims):
        """Organised cloud: direct index returns the correct point."""
        H, W = img_dims
        cloud = np.zeros((H, W, 3), dtype=np.float32)
        cloud[100, 200] = [1.0, 2.0, 3.0]
        result = lookup_depth_3d(cloud, organized=True, u=200, v=100)
        np.testing.assert_array_equal(result, [1.0, 2.0, 3.0])

    def test_organized_lookup_zero_point(self, img_dims):
        """Organised cloud: a zero point (no depth) returns None."""
        H, W = img_dims
        cloud = np.zeros((H, W, 3), dtype=np.float32)
        result = lookup_depth_3d(cloud, organized=True, u=100, v=100)
        assert result is None

    def test_unorganized_lookup(self, intrinsics, identity_pose):
        """Unorganised cloud: find the nearest point to the ray through a pixel."""
        # A point at (0, 0, 2) in front of the camera.
        cloud = np.array([
            [0.0, 0.0, 2.0],
            [5.0, 5.0, 2.0],  # far off-axis
        ])
        # Pixel (320, 240) = principal point → ray through (0, 0, 1)
        result = lookup_depth_3d(cloud, organized=False, u=320, v=240,
                                 K=intrinsics, pose=identity_pose)
        assert result is not None
        np.testing.assert_allclose(result, [0.0, 0.0, 2.0], atol=1e-6)

    def test_unorganized_lookup_no_match(self, intrinsics, identity_pose):
        """All points far from the ray → None."""
        cloud = np.array([
            [10.0, 10.0, 2.0],
            [-10.0, -10.0, 2.0],
        ])
        result = lookup_depth_3d(cloud, organized=False, u=320, v=240,
                                 K=intrinsics, pose=identity_pose,
                                 search_radius_m=0.01)
        assert result is None

    def test_unorganized_empty_cloud(self, intrinsics, identity_pose):
        cloud = np.zeros((0, 3), dtype=np.float32)
        result = lookup_depth_3d(cloud, organized=False, u=320, v=240,
                                 K=intrinsics, pose=identity_pose)
        assert result is None
