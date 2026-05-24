#!/usr/bin/env python3
"""Unit tests for segmentation_ros2_node click coalescing behavior.

These tests exercise the debounce/batch logic without requiring ROS 2.
They mock the Node base class enough to instantiate SegmentationNode.

Usage:
    python3 -m pytest src/segmentation/segmentation_bridge/test/test_segmentation_coalescing.py -v
"""

import sys
import os

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'segmentation_bridge'))

import numpy as np

# Mock rclpy before importing the node module
class MockClock:
    def now(self):
        class _T:
            def to_msg(self):
                from builtin_interfaces.msg import Time
                return Time(sec=0, nanosec=0)
        return _T()

class MockNode:
    def __init__(self, name):
        self._name = name
        self._params = {}
        self._subs = []
        self._pubs = []
        self._timers = []

    def declare_parameter(self, name, value):
        self._params[name] = value

    def get_parameter(self, name):
        class _P:
            def __init__(self, v):
                self._v = v
            @property
            def value(self):
                return self._v
        return _P(self._params.get(name))

    def create_subscription(self, *args):
        self._subs.append(args)

    def create_publisher(self, *args):
        self._pubs.append(args)
        return MockPublisher()

    def create_timer(self, period_s, callback):
        t = MockTimer(period_s, callback)
        self._timers.append(t)
        return t

    def get_logger(self):
        class _L:
            def info(self, msg): pass
            def warn(self, msg): pass
            def error(self, msg): pass
            def debug(self, msg): pass
        return _L()

    def get_clock(self):
        return MockClock()

class MockPublisher:
    def publish(self, msg): pass

class MockTimer:
    def __init__(self, period_s, callback):
        self._period_s = period_s
        self._callback = callback
        self._cancelled = False

    def cancel(self):
        self._cancelled = True

    def fire(self):
        if not self._cancelled:
            self._callback()

# Monkey-patch rclpy before importing the module under test
import rclpy
rclpy.node.Node = MockNode

from segmentation_ros2_node import SegmentationNode


class TestClickCoalescing:

    def test_single_click_schedules_one_inference(self):
        node = SegmentationNode()
        assert node._debounce_timer is None

        # Simulate a positive click
        node._pos_clicks.append([1.0, 2.0, 3.0])
        node._schedule_inference()

        assert node._debounce_timer is not None
        assert len(node._pos_clicks) == 1

    def test_multiple_clicks_reuse_same_timer(self):
        node = SegmentationNode()
        node._pos_clicks.append([1.0, 2.0, 3.0])
        node._schedule_inference()
        first_timer = node._debounce_timer

        node._pos_clicks.append([1.1, 2.1, 3.1])
        node._schedule_inference()
        second_timer = node._debounce_timer

        assert first_timer is not second_timer
        assert first_timer._cancelled is True
        assert len(node._pos_clicks) == 2

    def test_reset_cancels_timer_and_clears_clicks(self):
        node = SegmentationNode()
        node._pos_clicks.append([1.0, 2.0, 3.0])
        node._schedule_inference()

        node._reset_cb(None)
        assert node._debounce_timer is None
        assert node._pos_clicks == []

    def test_debounce_timer_fires_inference(self):
        node = SegmentationNode()
        node._pos_clicks.append([1.0, 2.0, 3.0])
        node._neg_clicks.append([0.0, 0.0, 0.0])
        node._cloud_xyz = np.zeros((10, 3), dtype=np.float32)
        node._cloud_rgb = np.zeros((10, 3), dtype=np.float32)
        node._cloud_header = type('H', (), {'frame_id': 'cloud', 'stamp': type('T', (), {'sec': 0, 'nanosec': 0})()})()
        node._debounce_s = 0.01  # short for test

        # Override _run_inference to capture calls
        calls = []
        original_run = node._run_inference
        def mock_run():
            calls.append((list(node._pos_clicks), list(node._neg_clicks)))
        node._run_inference = mock_run

        node._schedule_inference()
        assert node._debounce_timer is not None
        # Fire the timer immediately
        node._debounce_timer.fire()
        assert len(calls) == 1
        assert len(calls[0][0]) == 1  # 1 positive click
        assert len(calls[0][1]) == 1  # 1 negative click

    def test_negative_click_also_schedules(self):
        node = SegmentationNode()
        node._neg_clicks.append([0.0, 0.0, 0.0])
        node._schedule_inference()
        assert node._debounce_timer is not None


class TestROICrop:
    """Unit tests for SegmentationNode._crop_to_roi static method."""

    def _make_cloud(self, n: int = 100, seed: int = 42) -> tuple:
        """Create a synthetic cloud for testing."""
        rng = np.random.default_rng(seed)
        xyz = rng.uniform(-1.0, 1.0, (n, 3)).astype(np.float32)
        rgb = rng.uniform(0, 1, (n, 3)).astype(np.float32)
        return xyz, rgb

    def test_keeps_points_within_radius(self):
        """Points within radius of click centroid are retained."""
        xyz, rgb = self._make_cloud(50, seed=1)
        pos_clicks = [[0.0, 0.0, 0.0]]
        cropped_xyz, cropped_rgb, indices = SegmentationNode._crop_to_roi(
            xyz, rgb, pos_clicks, [], radius=0.5, cubeedge=0.05)
        # Points at distance >0.5 from origin should be removed
        dists = np.linalg.norm(cropped_xyz, axis=1)
        assert np.all(dists <= 0.5 + 1e-6)
        assert len(cropped_xyz) <= len(xyz)
        # Indices should be valid and monotonic
        assert np.all(np.diff(indices) >= 0)

    def test_removes_points_outside_radius(self):
        """Points far from click centroid are excluded."""
        # Create points clustered in two groups
        xyz = np.array([
            [0.0, 0.0, 0.0],   # near click
            [0.01, 0.0, 0.0],  # near click
            [0.0, 0.01, 0.0],  # near click
            [2.0, 0.0, 0.0],   # far
            [0.0, 2.0, 0.0],   # far
            [-2.0, 0.0, 0.0],  # far
        ], dtype=np.float32)
        rgb = np.zeros((6, 3), dtype=np.float32)
        pos_clicks = [[0.0, 0.0, 0.0]]
        cropped_xyz, cropped_rgb, indices = SegmentationNode._crop_to_roi(
            xyz, rgb, pos_clicks, [], radius=0.1, cubeedge=0.05)
        assert len(cropped_xyz) == 3  # only the 3 near points
        assert list(indices) == [0, 1, 2]  # first 3 original indices

    def test_negative_clicks_preserved(self):
        """Points near negative clicks are retained even if outside positive ROI."""
        xyz = np.array([
            [0.0, 0.0, 0.0],   # near positive click
            [2.0, 0.0, 0.0],   # far from positive click
            [2.01, 0.0, 0.0],  # within cubeedge of negative click (2,0,0)
            [3.0, 0.0, 0.0],   # far from both
        ], dtype=np.float32)
        rgb = np.zeros((4, 3), dtype=np.float32)
        pos_clicks = [[0.0, 0.0, 0.0]]
        neg_clicks = [[2.0, 0.0, 0.0]]
        cropped_xyz, cropped_rgb, indices = SegmentationNode._crop_to_roi(
            xyz, rgb, pos_clicks, neg_clicks, radius=0.5, cubeedge=0.05)
        # Points 0 (near pos), 1 (neg itself), 2 (within cubeedge of neg) should be kept
        # Point 3 (far from both) should be dropped
        assert len(cropped_xyz) == 3
        assert 0 in indices
        assert 1 in indices
        assert 2 in indices
        assert 3 not in indices

    def test_empty_pos_clicks_returns_full_cloud(self):
        """No positive clicks → crop is skipped, full cloud returned."""
        xyz, rgb = self._make_cloud(50, seed=2)
        cropped_xyz, cropped_rgb, indices = SegmentationNode._crop_to_roi(
            xyz, rgb, [], [], radius=0.3, cubeedge=0.05)
        assert len(cropped_xyz) == len(xyz)
        assert np.array_equal(indices, np.arange(len(xyz)))

    def test_radius_zero_returns_full_cloud(self):
        """Zero radius → crop is disabled, full cloud returned."""
        xyz, rgb = self._make_cloud(50, seed=3)
        pos_clicks = [[0.0, 0.0, 0.0]]
        cropped_xyz, cropped_rgb, indices = SegmentationNode._crop_to_roi(
            xyz, rgb, pos_clicks, [], radius=0.0, cubeedge=0.05)
        assert len(cropped_xyz) == len(xyz)
        assert np.array_equal(indices, np.arange(len(xyz)))

    def test_mask_remapping(self):
        """Verify cropped mask can be remapped to full cloud via indices."""
        # 10 points: first 5 near click, last 5 far
        xyz = np.zeros((10, 3), dtype=np.float32)
        xyz[:5] = [0.01, 0.0, 0.0]  # near click
        xyz[5:] = [2.0, 0.0, 0.0]   # far
        rgb = np.zeros((10, 3), dtype=np.float32)
        pos_clicks = [[0.0, 0.0, 0.0]]

        cropped_xyz, cropped_rgb, indices = SegmentationNode._crop_to_roi(
            xyz, rgb, pos_clicks, [], radius=0.1, cubeedge=0.05)

        assert len(cropped_xyz) == 5  # only near points
        assert list(indices) == [0, 1, 2, 3, 4]

        # Simulate inference returning a mask for the cropped cloud
        cropped_mask = np.array([True, False, True, False, True], dtype=bool)

        # Remap to full cloud
        full_mask = np.zeros(10, dtype=bool)
        full_mask[indices] = cropped_mask

        # Verify only positions 0,2,4 are True in the full mask
        expected = np.zeros(10, dtype=bool)
        expected[[0, 2, 4]] = True
        assert np.array_equal(full_mask, expected)

    def test_cropped_rgb_matches_xyz(self):
        """RGB arrays are correctly sliced alongside XYZ."""
        xyz = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
        ], dtype=np.float32)
        rgb = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ], dtype=np.float32)
        pos_clicks = [[0.0, 0.0, 0.0]]
        cropped_xyz, cropped_rgb, indices = SegmentationNode._crop_to_roi(
            xyz, rgb, pos_clicks, [], radius=0.1, cubeedge=0.05)
        assert len(cropped_xyz) == 1
        assert np.allclose(cropped_rgb[0], [1.0, 0.0, 0.0])
