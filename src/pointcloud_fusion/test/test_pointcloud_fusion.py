#!/usr/bin/env python3
"""Unit tests for pointcloud_fusion_node pure functions.

Tests cover the per-channel RGB averaging logic in _voxel_downsample
to ensure no cross-channel carry propagation when averaging packed
0x00RRGGBB colour values.

Usage:
    python3 -m pytest src/pointcloud_fusion/test/test_pointcloud_fusion.py -v
"""

import sys
import os

import numpy as np
import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'pointcloud_fusion'))

from pointcloud_fusion_node import (  # noqa: E402
    _voxel_downsample,
    _build_cloud,
    _parse_cloud,
)


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
        Expected  = 0x007F0080  (R=127, G=0, B=128) after integer division

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
        assert b == 128

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


# ---------------------------------------------------------------------------
# Round-trip test: build → parse → downsample → build → parse
# ---------------------------------------------------------------------------

class TestCloudRoundTrip:
    """Verify _build_cloud and _parse_cloud round-trip correctly."""

    def test_build_parse_roundtrip(self):
        """Build a cloud, parse it, and verify colours survive."""
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import Header

        xyz = np.array([[0.1, 0.2, 0.3]], dtype=np.float32)
        rgb = np.array([_pack_rgb(42, 84, 168)], dtype=np.uint32)

        header = Header()
        header.frame_id = "world"
        msg = _build_cloud(xyz, rgb, header)

        xyz2, rgb2 = _parse_cloud(msg)
        assert np.allclose(xyz2, xyz)
        assert _unpack_rgb(int(rgb2[0])) == (42, 84, 168)
