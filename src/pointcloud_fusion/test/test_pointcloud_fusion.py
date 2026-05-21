#!/usr/bin/env python3
"""Unit tests for pointcloud_fusion_node _voxel_downsample RGB correctness.

These tests verify that per-channel RGB averaging in _voxel_downsample
produces correct colours with no cross-channel carry propagation when
averaging packed 0x00RRGGBB values.

Because the full module requires rclpy (unavailable outside a ROS 2
environment), the pure helper function is re-defined here for isolated
testing.  Keep this copy in sync with the canonical implementation in
pointcloud_fusion_node.py::_voxel_downsample.

Usage:
    python3 -m pytest src/pointcloud_fusion/test/test_pointcloud_fusion.py -v
"""

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Copy of _voxel_downsample from pointcloud_fusion_node.py (pure numpy, no
# ROS dependencies).  Update this if the canonical implementation changes.
# ---------------------------------------------------------------------------

def _voxel_downsample(xyz: np.ndarray, rgb_packed: np.ndarray, voxel_size: float):
    """Voxel grid downsampling. Returns (xyz_down, rgb_down) with one centroid per voxel.

    xyz:         (N, 3) float32
    rgb_packed:  (N,) uint32  — 0x00RRGGBB encoding
    voxel_size:  side length of each voxel cube in metres
    """
    if voxel_size <= 0.0 or len(xyz) == 0:
        return xyz, rgb_packed

    inv = 1.0 / voxel_size
    voxel_idx = np.floor(xyz * inv).astype(np.int64)
    _, unique_idx, inverse = np.unique(
        voxel_idx, axis=0, return_index=True, return_inverse=True,
    )

    n_voxels = len(unique_idx)
    summed_xyz = np.zeros((n_voxels, 3), dtype=np.float64)
    counts = np.zeros(n_voxels, dtype=np.int32)

    np.add.at(summed_xyz, inverse, xyz.astype(np.float64))
    np.add.at(counts, inverse, 1)

    xyz_out = (summed_xyz / counts[:, None]).astype(np.float32)

    # Per-channel RGB averaging to avoid carry propagation between channels
    # when averaging packed 0x00RRGGBB integers.
    r_ch = ((rgb_packed >> 16) & 0xFF).astype(np.uint64)
    g_ch = ((rgb_packed >> 8) & 0xFF).astype(np.uint64)
    b_ch = (rgb_packed & 0xFF).astype(np.uint64)

    summed_r = np.zeros(n_voxels, dtype=np.uint64)
    summed_g = np.zeros(n_voxels, dtype=np.uint64)
    summed_b = np.zeros(n_voxels, dtype=np.uint64)

    np.add.at(summed_r, inverse, r_ch)
    np.add.at(summed_g, inverse, g_ch)
    np.add.at(summed_b, inverse, b_ch)

    counts_u64 = counts.astype(np.uint64)
    avg_r = (summed_r / counts_u64).astype(np.uint32)
    avg_g = (summed_g / counts_u64).astype(np.uint32)
    avg_b = (summed_b / counts_u64).astype(np.uint32)

    rgb_out = (avg_r << 16) | (avg_g << 8) | avg_b
    return xyz_out, rgb_out


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pack_rgb(r: int, g: int, b: int) -> int:
    """Pack R, G, B uint8 channels into a 0x00RRGGBB uint32."""
    return (r << 16) | (g << 8) | b


def _unpack_rgb(packed: int):
    """Unpack a 0x00RRGGBB uint32 into (R, G, B) uint8 tuple."""
    return ((packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF)


# ---------------------------------------------------------------------------
# Voxel downsample — RGB correctness tests
# ---------------------------------------------------------------------------

class TestVoxelDownsampleRGB:
    """Verify per-channel colour averaging in _voxel_downsample."""

    def test_single_point_preserves_colour(self):
        """A voxel with a single point should preserve its exact colour."""
        xyz = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        rgb = np.array([_pack_rgb(128, 64, 200)], dtype=np.uint32)

        xyz_out, rgb_out = _voxel_downsample(xyz, rgb, voxel_size=0.01)

        assert len(xyz_out) == 1
        assert _unpack_rgb(int(rgb_out[0])) == (128, 64, 200)

    def test_red_blue_average_no_carry(self):
        """Average of pure red and pure blue must not carry between channels.

        Pure red  = 0x00FF0000  (R=255, G=0, B=0)
        Pure blue = 0x000000FF  (R=0,   G=0, B=255)
        Expected  = 0x007F007F  (R=127, G=0, B=127) after integer truncation

        The old (broken) packed-integer averaging would produce
        (0x00FF0000 + 0x000000FF) / 2 = 0x007F8080  — note the
        spurious G=128 from the carry.
        """
        xyz = np.array([
            [0.001, 0.001, 0.001],
            [0.002, 0.002, 0.002],
        ], dtype=np.float32)
        rgb = np.array([
            _pack_rgb(255, 0, 0),
            _pack_rgb(0, 0, 255),
        ], dtype=np.uint32)

        xyz_out, rgb_out = _voxel_downsample(xyz, rgb, voxel_size=0.01)

        assert len(xyz_out) == 1
        r, g, b = _unpack_rgb(int(rgb_out[0]))
        assert r == 127
        assert g == 0, f"Green channel should be 0 but got {g} — carry from blue!"
        assert b == 127

    def test_high_blue_no_green_carry(self):
        """Two points with B=200 each: average B=200, must not add carry to G.

        With the old code, summing two B=200 values gives 400 = 0x190.
        The carry (0x100) would add +1 to the green channel.
        """
        xyz = np.array([
            [0.0, 0.0, 0.0],
            [0.001, 0.001, 0.001],
        ], dtype=np.float32)
        rgb = np.array([
            _pack_rgb(100, 50, 200),
            _pack_rgb(100, 50, 200),
        ], dtype=np.uint32)

        xyz_out, rgb_out = _voxel_downsample(xyz, rgb, voxel_size=0.01)

        assert len(xyz_out) == 1
        r, g, b = _unpack_rgb(int(rgb_out[0]))
        assert r == 100
        assert g == 50, f"Green channel should be 50 but got {g} — carry from blue!"
        assert b == 200

    def test_high_green_no_red_carry(self):
        """Two points with G=200 each: average G=200, must not carry into R."""
        xyz = np.array([
            [0.0, 0.0, 0.0],
            [0.001, 0.001, 0.001],
        ], dtype=np.float32)
        rgb = np.array([
            _pack_rgb(50, 200, 100),
            _pack_rgb(50, 200, 100),
        ], dtype=np.uint32)

        xyz_out, rgb_out = _voxel_downsample(xyz, rgb, voxel_size=0.01)

        assert len(xyz_out) == 1
        r, g, b = _unpack_rgb(int(rgb_out[0]))
        assert r == 50, f"Red channel should be 50 but got {r} — carry from green!"
        assert g == 200
        assert b == 100

    def test_three_way_average(self):
        """Three points in one voxel: per-channel integer average must be correct."""
        xyz = np.array([
            [0.0, 0.0, 0.0],
            [0.001, 0.001, 0.001],
            [0.002, 0.002, 0.002],
        ], dtype=np.float32)
        rgb = np.array([
            _pack_rgb(255, 0, 0),
            _pack_rgb(0, 255, 0),
            _pack_rgb(0, 0, 255),
        ], dtype=np.uint32)

        xyz_out, rgb_out = _voxel_downsample(xyz, rgb, voxel_size=0.01)

        assert len(xyz_out) == 1
        r, g, b = _unpack_rgb(int(rgb_out[0]))
        assert r == 85   # 255 // 3
        assert g == 85   # 255 // 3
        assert b == 85   # 255 // 3

    def test_separate_voxels_preserve_colour(self):
        """Points in different voxels should keep their own colours."""
        xyz = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
        ], dtype=np.float32)
        rgb = np.array([
            _pack_rgb(255, 0, 0),
            _pack_rgb(0, 255, 0),
        ], dtype=np.uint32)

        xyz_out, rgb_out = _voxel_downsample(xyz, rgb, voxel_size=0.01)

        assert len(xyz_out) == 2
        colours = sorted([_unpack_rgb(int(c)) for c in rgb_out])
        assert (0, 255, 0) in colours
        assert (255, 0, 0) in colours

    def test_empty_input_passthrough(self):
        """Empty input should return empty arrays."""
        xyz = np.zeros((0, 3), dtype=np.float32)
        rgb = np.zeros(0, dtype=np.uint32)

        xyz_out, rgb_out = _voxel_downsample(xyz, rgb, voxel_size=0.01)

        assert len(xyz_out) == 0
        assert len(rgb_out) == 0

    def test_zero_voxel_size_passthrough(self):
        """Voxel size of 0 should return input unchanged."""
        xyz = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        rgb = np.array([_pack_rgb(10, 20, 30)], dtype=np.uint32)

        xyz_out, rgb_out = _voxel_downsample(xyz, rgb, voxel_size=0.0)

        assert len(xyz_out) == 1
        assert _unpack_rgb(int(rgb_out[0])) == (10, 20, 30)
