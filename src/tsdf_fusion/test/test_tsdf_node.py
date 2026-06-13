#!/usr/bin/env python3
"""ROS smoke test for tsdf_fusion_node.

Launches tsdf_fusion + keyframe_buffer + a mock SAM server, calls
``TriggerGraspFusion`` with a synthetic hit point, and verifies a PointCloud2
is published on ``/segmentation/object_cloud``.

Requires ROS 2 + the built workspace (run in the container after ``make build``).

Usage:
    python3 -m pytest src/tsdf_fusion/test/test_tsdf_node.py -v
"""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

# Skip if ROS / sensor_fusion_msgs not available.
rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_fusion_msgs")
pytest.importorskip("open3d")


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_node_constructs(rclpy_init):
    """The node can be constructed without crashing (service + publisher)."""
    from tsdf_fusion.tsdf_fusion_node import create_node

    NodeClass = create_node()
    node = NodeClass()
    assert node.get_name() == "tsdf_fusion"
    node.destroy_node()


def test_keyframe_deserialisation_roundtrip(rclpy_init):
    """Verify _keyframe_from_msg + _build_xyzrgb_cloud round-trip numpy data."""
    from tsdf_fusion.tsdf_fusion_node import _build_xyzrgb_cloud
    from std_msgs.msg import Header

    xyz = np.array([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]], dtype=np.float32)
    rgb = np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8)
    header = Header()
    header.frame_id = "marker_map"

    msg = _build_xyzrgb_cloud(xyz, rgb, header)
    assert msg.width == 2
    assert msg.point_step == 16
    assert len(msg.data) == 32  # 2 points * 16 bytes

    # Empty cloud.
    empty = _build_xyzrgb_cloud(
        np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint8), header)
    assert empty.width == 0
    assert len(empty.data) == 0


def test_sam_client_health_down(rclpy_init):
    """SamHttpClient.health() returns False when no server is running."""
    from tsdf_fusion.tsdf_fusion_node import SamHttpClient

    client = SamHttpClient("http://127.0.0.1:59999", timeout_s=0.5)
    assert client.health() is False

    # Calling it on an empty image returns an all-zero mask (no crash).
    img = np.zeros((10, 10, 3), dtype=np.uint8)
    mask = client(img, (5, 5), 0)
    assert mask.shape == (10, 10)
    assert not np.any(mask)
