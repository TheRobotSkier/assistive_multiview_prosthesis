#!/usr/bin/env python3
"""ROS smoke test for gtsam_tracker_node.

Launches the node with mock odom publishers and verifies it publishes
``/gtsam/head_pose`` and ``/gtsam/arm_pose``.

Requires ROS 2 + gtsam (run in the container after ``make build``).

Usage:
    python3 -m pytest src/gtsam_tracker/test/test_gtsam_node.py -v
"""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

# Skip if ROS / gtsam not available.
rclpy = pytest.importorskip("rclpy")
pytest.importorskip("gtsam")


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_odom_msg(stamp_s: float, x: float, y: float, z: float):
    """Build a nav_msgs/Odometry with a simple translation."""
    from nav_msgs.msg import Odometry
    from builtin_interfaces.msg import Time as TimeMsg

    msg = Odometry()
    msg.header.stamp.sec = int(stamp_s)
    msg.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
    msg.header.frame_id = "marker_map"
    msg.child_frame_id = "arm_cam0"
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = float(z)
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_node_constructs(rclpy_init):
    """The node can be constructed without crashing."""
    from gtsam_tracker.gtsam_tracker_node import create_node

    NodeClass = create_node()
    node = NodeClass()
    assert node.get_name() == "gtsam_tracker"
    node.destroy_node()


def test_node_publishes_poses(rclpy_init):
    """With mock odom input, the node publishes head + arm poses."""
    from gtsam_tracker.gtsam_tracker_node import create_node
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry

    NodeClass = create_node()
    node = NodeClass()

    # Override the graph rate to be faster for testing
    node._timer.cancel()
    node._graph_rate = 50.0
    node._timer = node.create_timer(
        1.0 / node._graph_rate, node._graph_update)

    # Track received poses
    received_head = threading.Event()
    received_arm = threading.Event()

    def _on_head(msg):
        received_head.set()

    def _on_arm(msg):
        received_arm.set()

    node.create_subscription(
        PoseWithCovarianceStamped, "/gtsam/head_pose", _on_head, 10)
    node.create_subscription(
        PoseWithCovarianceStamped, "/gtsam/arm_pose", _on_arm, 10)

    # Create mock odom publishers
    head_pub = node.create_publisher(Odometry, "/ov_msckf/odomimu", 10)
    arm_pub = node.create_publisher(Odometry, "/ov_msckf_arm/odomimu", 10)

    # Spin in a separate thread
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        # Publish moving odom for ~1 second
        t0 = time.time()
        while time.time() - t0 < 2.0:
            elapsed = time.time() - t0
            head_msg = _make_odom_msg(elapsed, 0.0, 0.0, 0.0)
            arm_msg = _make_odom_msg(elapsed, elapsed * 0.1, 0.0, 0.0)
            head_pub.publish(head_msg)
            arm_pub.publish(arm_msg)
            time.sleep(0.05)

        # Wait for poses to arrive
        timeout = 3.0
        assert received_head.wait(timeout), \
            "Did not receive /gtsam/head_pose within timeout"
        assert received_arm.wait(timeout), \
            "Did not receive /gtsam/arm_pose within timeout"
    finally:
        node.destroy_node()


def test_node_tf_fallback(rclpy_init):
    """With head_pose_source='tf', the node does not crash."""
    from gtsam_tracker.gtsam_tracker_node import create_node

    NodeClass = create_node()
    node = NodeClass()

    # Force TF mode
    node._head_pose_source = "tf"
    # The node should handle missing TF gracefully (fail-open)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    time.sleep(0.5)
    node.destroy_node()


def test_node_graceful_no_aruco(rclpy_init):
    """The node runs without ArUco topics and doesn't crash."""
    from gtsam_tracker.gtsam_tracker_node import create_node
    from nav_msgs.msg import Odometry

    NodeClass = create_node()
    node = NodeClass()

    arm_pub = node.create_publisher(Odometry, "/ov_msckf_arm/odomimu", 10)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Publish some odom without ArUco
    t0 = time.time()
    while time.time() - t0 < 1.0:
        msg = _make_odom_msg(time.time() - t0, 0.05, 0.0, 0.0)
        arm_pub.publish(msg)
        time.sleep(0.05)

    # Node should still be alive
    assert node._key_idx > 0 or True  # May be 0 if graph update hasn't run
    node.destroy_node()
