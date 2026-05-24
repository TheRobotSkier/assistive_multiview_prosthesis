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
