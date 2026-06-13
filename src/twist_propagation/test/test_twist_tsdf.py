#!/usr/bin/env python3
"""Integration tests for the V6 TSDF fusion path in twist_propagation_node.

These tests verify:
  - Default construction (use_tsdf_fusion=False) — no regression.
  - TSDF-enabled construction (use_tsdf_fusion=True) — service client created.
  - Kinematic range check fail-open behaviour (no TF → passes).
  - _trigger_tsdf_fusion does not crash when client is unavailable.
  - _publish_click_cluster still works (the legacy click path).

Requires ROS 2 (run in the container after ``make build``).

Usage:
    python3 -m pytest src/twist_propagation/test/test_twist_tsdf.py -v
"""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

# Skip if ROS 2 not available.
rclpy = pytest.importorskip("rclpy")


@pytest.fixture
def rclpy_init():
    """Initialise and shutdown rclpy for each test (fresh context)."""
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node(overrides: dict | None = None):
    """Construct a TwistPropagationNode with optional parameter overrides.

    Uses rclpy node options to inject parameters before the node reads them.
    """
    from rclpy.parameter import Parameter
    from twist_propagation_node import TwistPropagationNode

    if overrides:
        # Re-init with command-line args for parameter overrides.
        rclpy.shutdown()
        args = ["--ros-args"]
        for key, val in overrides.items():
            args.extend(["-p", f"twist_propagation.{key}:={val}"])
        rclpy.init(args=args)

    node = TwistPropagationNode()
    return node


# ---------------------------------------------------------------------------
# Default construction — no regression
# ---------------------------------------------------------------------------

class TestDefaultNoRegression:
    """With use_tsdf_fusion=False (default), the legacy path is unchanged."""

    def test_default_use_tsdf_fusion_false(self, rclpy_init):
        """The node defaults to use_tsdf_fusion=False."""
        node = _make_node()
        try:
            assert node._use_tsdf_fusion is False
        finally:
            node.destroy_node()

    def test_default_no_tsdf_client(self, rclpy_init):
        """With use_tsdf_fusion=False, no TSDF service client is created."""
        node = _make_node()
        try:
            assert node._tsdf_client is None
        finally:
            node.destroy_node()

    def test_default_kinematic_constraint_disabled(self, rclpy_init):
        """The kinematic constraint is disabled by default."""
        node = _make_node()
        try:
            assert node._enforce_kinematic_constraint is False
        finally:
            node.destroy_node()

    def test_default_max_head_wrist_distance(self, rclpy_init):
        """The default max head-wrist distance is 1.0m."""
        node = _make_node()
        try:
            assert node._max_head_wrist_distance == 1.0
        finally:
            node.destroy_node()

    def test_publish_click_cluster_returns_count(self, rclpy_init):
        """_publish_click_cluster returns the correct click count.

        With click_count=0 (default), it returns 1 (just the original hit).
        """
        node = _make_node({"click_count": "0"})
        try:
            # click_count=0 means 0 synthetic + 1 original = 1 total
            count = node._publish_click_cluster(0.1, 0.2, 0.3, (0.1, 0.2, 0.3))
            assert count == 1
        finally:
            node.destroy_node()

    def test_publish_click_cluster_with_synthetic(self, rclpy_init):
        """_publish_click_cluster with click_count=4 returns 5 clicks."""
        node = _make_node({"click_count": "4"})
        try:
            count = node._publish_click_cluster(0.1, 0.2, 0.3, (0.1, 0.2, 0.3))
            assert count == 5  # 1 original + 4 synthetic
        finally:
            node.destroy_node()


# ---------------------------------------------------------------------------
# TSDF-enabled construction
# ---------------------------------------------------------------------------

class TestTsdfEnabled:
    """With use_tsdf_fusion=True, the TSDF service client is created."""

    def test_tsdf_param_read(self, rclpy_init):
        """The node reads use_tsdf_fusion=True from the parameter."""
        node = _make_node({"use_tsdf_fusion": "true"})
        try:
            assert node._use_tsdf_fusion is True
        finally:
            node.destroy_node()

    def test_tsdf_service_name(self, rclpy_init):
        """The TSDF service name is read correctly."""
        node = _make_node({
            "use_tsdf_fusion": "true",
            "tsdf_fusion_service": "/custom/tsdf_trigger",
        })
        try:
            assert node._tsdf_service == "/custom/tsdf_trigger"
        finally:
            node.destroy_node()

    def test_tsdf_client_created(self, rclpy_init):
        """When sensor_fusion_msgs is available, a service client is created."""
        node = _make_node({"use_tsdf_fusion": "true"})
        try:
            if node._use_tsdf_fusion:
                # sensor_fusion_msgs was available — client should exist
                assert node._tsdf_client is not None
            else:
                # sensor_fusion_msgs not available — graceful fallback
                assert node._tsdf_client is None
        finally:
            node.destroy_node()

    def test_kinematic_constraint_enabled(self, rclpy_init):
        """The kinematic constraint can be enabled via parameter."""
        node = _make_node({
            "use_tsdf_fusion": "true",
            "enforce_kinematic_constraint": "true",
        })
        try:
            assert node._enforce_kinematic_constraint is True
        finally:
            node.destroy_node()

    def test_max_distance_override(self, rclpy_init):
        """The max head-wrist distance can be overridden."""
        node = _make_node({
            "use_tsdf_fusion": "true",
            "max_head_wrist_distance_m": "0.5",
        })
        try:
            assert node._max_head_wrist_distance == 0.5
        finally:
            node.destroy_node()


# ---------------------------------------------------------------------------
# Kinematic range check — fail-open behaviour
# ---------------------------------------------------------------------------

class TestKinematicRangeCheck:
    """The _check_kinematic_range method fails open when TF is unavailable."""

    def test_fail_open_no_tf_buffer(self, rclpy_init):
        """Without a TF buffer, the check passes (fail-open)."""
        node = _make_node()
        try:
            # _tf_buffer may be None if tf2_ros isn't available
            result = node._check_kinematic_range()
            assert result is True
        finally:
            node.destroy_node()

    def test_fail_open_empty_pose_buf(self, rclpy_init):
        """With an empty pose buffer, the check passes."""
        node = _make_node()
        try:
            # Ensure pose buffer is empty
            node._pose_buf.clear()
            result = node._check_kinematic_range()
            assert result is True
        finally:
            node.destroy_node()


# ---------------------------------------------------------------------------
# TSDF trigger — graceful handling
# ---------------------------------------------------------------------------

class TestTsdfTrigger:
    """The _trigger_tsdf_fusion method handles missing clients gracefully."""

    def test_no_crash_when_client_none(self, rclpy_init):
        """Calling _trigger_tsdf_fusion with no client doesn't crash."""
        node = _make_node()
        try:
            # Default: client is None
            assert node._tsdf_client is None
            # Should log an error but not raise
            node._trigger_tsdf_fusion(0.1, 0.2, 0.3)
        finally:
            node.destroy_node()

    def test_tsdf_response_callback_handles_none(self, rclpy_init):
        """The response callback handles None futures gracefully."""
        node = _make_node()
        try:

            class FakeFuture:
                def result(self):
                    return None

            # Should not raise
            node._on_tsdf_fusion_response(FakeFuture())
        finally:
            node.destroy_node()

    def test_tsdf_response_callback_handles_exception(self, rclpy_init):
        """The response callback handles exceptions gracefully."""
        node = _make_node()
        try:

            class FakeFuture:
                def result(self):
                    raise RuntimeError("test error")

            # Should not raise
            node._on_tsdf_fusion_response(FakeFuture())
        finally:
            node.destroy_node()


# ---------------------------------------------------------------------------
# State machine — WAITING_FOR_SEGMENTATION still works
# ---------------------------------------------------------------------------

class TestStateMachine:
    """Verify the WAITING_FOR_SEGMENTATION state enum is unchanged."""

    def test_cycle_state_enum(self, rclpy_init):
        """The CycleState enum has the expected values."""
        from twist_propagation_node import CycleState

        assert CycleState.IDLE.value == "IDLE"
        assert (CycleState.WAITING_FOR_SEGMENTATION.value
                == "WAITING_FOR_SEGMENTATION")

    def test_node_starts_idle(self, rclpy_init):
        """The node starts in the IDLE state."""
        node = _make_node()
        try:
            from twist_propagation_node import CycleState
            assert node._cycle_state == CycleState.IDLE
        finally:
            node.destroy_node()
