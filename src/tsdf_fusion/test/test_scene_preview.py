#!/usr/bin/env python3
"""Unit tests for fuse_scene_preview — mask-free scene-level TSDF fusion.

These tests verify the scene-preview variant that integrates the *full* cloud
from each keyframe without a hit point or SAM segmentation.  They reuse the
synthetic-scene helpers from ``test_tsdf_core.py``.

Requires Open3D (run in the container where it is installed).

Usage:
    python3 -m pytest src/tsdf_fusion/test/test_scene_preview.py -v
"""

from __future__ import annotations

import numpy as np
import pytest

# Skip the entire module if Open3D is not available — the fusion core needs it.
o3d = pytest.importorskip("open3d")

from keyframe_buffer.keyframe import Keyframe  # noqa: E402
from tsdf_fusion.tsdf_fusion_core import fuse_scene_preview  # noqa: E402

# Reuse the synthetic-scene helpers from the object-fusion test suite.
from test_tsdf_core import (  # noqa: E402
    make_box_surface,
    make_cylinder_surface,
    make_keyframes_from_surface,
    rms_to_surface,
)


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
# Edge cases
# ---------------------------------------------------------------------------

class TestEdgeCases:
    def test_empty_keyframe_list(self):
        """Empty keyframe list → empty cloud, success=False."""
        result = fuse_scene_preview(keyframes=[])
        assert result.success is False
        assert result.xyz.shape[0] == 0
        assert "empty" in result.message.lower()

    def test_none_keyframe_list(self):
        """None keyframe list → empty cloud, success=False."""
        result = fuse_scene_preview(keyframes=None)
        assert result.success is False
        assert result.xyz.shape[0] == 0

    def test_all_behind_camera(self, intrinsics, img_dims):
        """All keyframes have no points in front of the camera → empty result."""
        H, W = img_dims
        pose = np.eye(4)
        pose[:3, 3] = [0.0, 0.0, 1.0]  # camera at z=1
        # Cloud points at z=-1 (behind the camera).
        cloud = np.array([[0.0, 0.0, -1.0], [0.1, 0.1, -1.0]])
        kf = Keyframe(
            timestamp=0.0, camera_id="head",
            cloud_xyz=cloud,
            cloud_rgb=np.zeros((2, 3), dtype=np.uint8),
            image=np.zeros((H, W, 3), dtype=np.uint8),
            K=intrinsics, pose=pose, organized=False)
        result = fuse_scene_preview(keyframes=[kf])
        assert result.success is False
        assert result.xyz.shape[0] == 0


# ---------------------------------------------------------------------------
# Synthetic scene recovery (no hit point, no SAM)
# ---------------------------------------------------------------------------

class TestSyntheticScene:
    def test_box_recovery_unorganized(self, intrinsics, img_dims):
        """Fuse a synthetic box from multiple viewpoints and verify the fused
        cloud recovers the surface within 2 cm RMS — without a hit point or
        SAM mask."""
        H, W = img_dims
        rng = np.random.default_rng(42)

        box_center = np.array([0.0, 0.0, 0.0])
        box_size = np.array([0.06, 0.06, 0.06])
        surface = make_box_surface(box_center, box_size, n_per_face=600,
                                   rng=rng)

        target = box_center
        cam_positions = [
            [0.30, 0.0, 0.20],
            [-0.30, 0.0, 0.20],
            [0.0, 0.30, 0.20],
            [0.0, -0.30, 0.20],
            [0.20, 0.20, 0.30],
            [-0.20, -0.20, 0.30],
        ]

        keyframes, _masks = make_keyframes_from_surface(
            surface, intrinsics, cam_positions, target, H, W,
            organized=False, rng=rng)

        assert len(keyframes) == 6
        total_pts = sum(kf.cloud_xyz.shape[0] for kf in keyframes)
        assert total_pts > 0, "Synthetic clouds should contain points"

        result = fuse_scene_preview(
            keyframes=keyframes,
            voxel_size=0.005,
            sdf_trunc=0.02,
            dbscan_eps=0.02,
            dbscan_min_points=10,
        )

        assert result.success, f"Fusion failed: {result.message}"
        assert result.xyz.shape[0] > 0, "Fused cloud should be non-empty"

        rms = rms_to_surface(result.xyz, surface)
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

        keyframes, _masks = make_keyframes_from_surface(
            surface, intrinsics, cam_positions, target, H, W,
            organized=False, rng=rng)

        result = fuse_scene_preview(
            keyframes=keyframes,
            voxel_size=0.005,
            sdf_trunc=0.02,
            dbscan_eps=0.02,
            dbscan_min_points=10,
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

        keyframes, _masks = make_keyframes_from_surface(
            surface, intrinsics, cam_positions, target, H, W,
            organized=True, rng=rng)

        assert keyframes[0].cloud_xyz.shape == (H, W, 3)
        assert keyframes[0].organized is True

        result = fuse_scene_preview(
            keyframes=keyframes,
            voxel_size=0.005,
            sdf_trunc=0.02,
            dbscan_eps=0.02,
            dbscan_min_points=10,
        )

        assert result.success, f"Fusion failed: {result.message}"
        assert result.xyz.shape[0] > 0

        rms = rms_to_surface(result.xyz, surface)
        assert rms < 0.02, (
            f"Fused cloud RMS {rms * 100:.2f} cm exceeds 2 cm tolerance.")


# ---------------------------------------------------------------------------
# Workspace bbox crop
# ---------------------------------------------------------------------------

class TestWorkspaceBboxCrop:
    def test_bbox_excludes_outside_points(self, intrinsics, img_dims):
        """A tight workspace bbox should exclude points outside it."""
        H, W = img_dims
        rng = np.random.default_rng(3)

        # Two boxes: one inside the bbox, one far outside.
        surface_in = make_box_surface(
            np.array([0.0, 0.0, 0.0]),
            np.array([0.05, 0.05, 0.05]),
            n_per_face=400, rng=rng)
        surface_out = make_box_surface(
            np.array([2.0, 2.0, 2.0]),  # far outside any reasonable bbox
            np.array([0.05, 0.05, 0.05]),
            n_per_face=400, rng=rng)
        surface = np.vstack([surface_in, surface_out])

        target = np.array([0.0, 0.0, 0.0])
        cam_positions = [
            [0.25, 0.0, 0.20],
            [0.0, 0.25, 0.20],
            [0.20, 0.20, 0.25],
        ]

        keyframes, _masks = make_keyframes_from_surface(
            surface, intrinsics, cam_positions, target, H, W,
            organized=False, rng=rng)

        # Tight bbox around the origin only.
        result = fuse_scene_preview(
            keyframes=keyframes,
            voxel_size=0.005,
            sdf_trunc=0.02,
            workspace_bbox_min=np.array([-0.1, -0.1, -0.1]),
            workspace_bbox_max=np.array([0.1, 0.1, 0.1]),
            dbscan_eps=0.02,
            dbscan_min_points=10,
        )

        assert result.success, f"Fusion failed: {result.message}"
        assert result.xyz.shape[0] > 0

        # No fused point should be near the far-away box.
        far_dist = np.linalg.norm(
            result.xyz - np.array([2.0, 2.0, 2.0]), axis=1)
        assert np.min(far_dist) > 0.5, (
            "Fused cloud contains points near the far-away box that the bbox "
            "should have excluded.")

        # The fused cloud should recover the near box surface.
        rms = rms_to_surface(result.xyz, surface_in)
        assert rms < 0.02, (
            f"Fused cloud RMS to near surface {rms * 100:.2f} cm exceeds "
            f"2 cm tolerance.")

    def test_no_bbox_integrates_everything(self, intrinsics, img_dims):
        """Without a bbox, all points (including far ones) are integrated."""
        H, W = img_dims
        rng = np.random.default_rng(5)

        surface = make_box_surface(
            np.array([0.0, 0.0, 0.0]),
            np.array([0.05, 0.05, 0.05]),
            n_per_face=400, rng=rng)

        target = np.array([0.0, 0.0, 0.0])
        cam_positions = [
            [0.25, 0.0, 0.20],
            [0.0, 0.25, 0.20],
        ]

        keyframes, _masks = make_keyframes_from_surface(
            surface, intrinsics, cam_positions, target, H, W,
            organized=False, rng=rng)

        # No bbox — should still succeed.
        result = fuse_scene_preview(
            keyframes=keyframes,
            voxel_size=0.005,
            sdf_trunc=0.02,
            workspace_bbox_min=None,
            workspace_bbox_max=None,
        )

        assert result.success, f"Fusion failed: {result.message}"
        assert result.xyz.shape[0] > 0


# ---------------------------------------------------------------------------
# DBSCAN cleanup toggle
# ---------------------------------------------------------------------------

class TestDbsanToggle:
    def test_dbscan_disabled_returns_raw(self, intrinsics, img_dims):
        """When DBSCAN cleanup is disabled, the raw extracted cloud is returned."""
        H, W = img_dims
        rng = np.random.default_rng(9)

        surface = make_box_surface(
            np.array([0.0, 0.0, 0.0]),
            np.array([0.06, 0.06, 0.06]),
            n_per_face=600, rng=rng)

        target = np.array([0.0, 0.0, 0.0])
        cam_positions = [
            [0.30, 0.0, 0.20],
            [0.0, 0.30, 0.20],
            [0.20, 0.20, 0.30],
        ]

        keyframes, _masks = make_keyframes_from_surface(
            surface, intrinsics, cam_positions, target, H, W,
            organized=False, rng=rng)

        result = fuse_scene_preview(
            keyframes=keyframes,
            voxel_size=0.005,
            sdf_trunc=0.02,
            enable_dbscan_cleanup=False,
        )

        assert result.success, f"Fusion failed: {result.message}"
        assert result.xyz.shape[0] > 0
        assert "DBSCAN disabled" in result.message
