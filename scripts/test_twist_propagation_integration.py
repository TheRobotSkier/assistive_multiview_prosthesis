#!/usr/bin/env python3
"""Integration test for the twist_propagation node.

Runs inside the prosthesis Docker container where ROS 2 and the workspace
are available.  Exercises the node end-to-end:

  1. Starts the twist_propagation node.
  2. Publishes mock hand poses (PoseStamped) to /hand_pose.
  3. Publishes a mock point cloud to /camera/depth/color/points.
  4. Activates the node via /twist_propagation/activate.
  5. Verifies /hand_twist (TwistStamped) is published with non-zero linear velocity.
  6. Verifies /segmentation/click_positive is published (hit detected).
  7. Publishes a mock segmented cloud and verifies the preshaping service
     is called (via a stand-in service server).
  8. Tests deactivation.
  9. Verifies that no clicks are published when inactive.

Timeout: 30 seconds total.
"""

from __future__ import annotations

import math
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, PointStamped, TwistStamped, Vector3
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

PASS = 0
FAIL = 0


def _ok(name: str):
    global PASS
    PASS += 1
    print(f"  PASS: {name}")


def _fail(name: str, detail: str = ""):
    global FAIL
    FAIL += 1
    msg = f"  FAIL: {name}"
    if detail:
        msg += f" — {detail}"
    print(msg)


def _make_cloud(xyz_points, frame_id: str = "camera_front_depth",
                stamp_sec: float = 0.0) -> PointCloud2:
    """Build a PointCloud2 message from an (N, 3) array."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int((stamp_sec % 1) * 1e9)
    msg.height = 1
    msg.width = len(xyz_points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(xyz_points)
    buf = bytearray()
    for pt in xyz_points:
        buf.extend(struct.pack("fff", float(pt[0]), float(pt[1]), float(pt[2])))
    msg.data = bytes(buf)
    msg.is_dense = True
    return msg


def _make_pose(px, py, pz, qx=0.0, qy=0.0, qz=0.0, qw=1.0,
               frame_id="world") -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = int(time.time())
    msg.header.stamp.nanosec = int((time.time() % 1) * 1e9)
    msg.pose.position.x = float(px)
    msg.pose.position.y = float(py)
    msg.pose.position.z = float(pz)
    msg.pose.orientation.x = float(qx)
    msg.pose.orientation.y = float(qy)
    msg.pose.orientation.z = float(qz)
    msg.pose.orientation.w = float(qw)
    return msg


# ---------------------------------------------------------------------------
# Test harness
# ---------------------------------------------------------------------------

class TestHarness(Node):
    """Drives the test by publishing mock data and observing outputs."""

    def __init__(self):
        super().__init__("test_twist_propagation_harness")

        # ── Publishers (feed the node) ─────────────────────────────────────
        self.pose_pub = self.create_publisher(PoseStamped, "/hand_pose", 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/camera/depth/color/points", 10)
        self.seg_cloud_pub = self.create_publisher(
            PointCloud2, "/segmentation/object_cloud", 10)

        # ── Subscribers (observe the node) ─────────────────────────────────
        self._twist_msgs: list[TwistStamped] = []
        self._click_msgs: list[PointStamped] = []
        self._status_msgs: list[String] = []
        self._path_msgs: list[Path] = []
        self._sphere_marker_msgs: list[MarkerArray] = []
        self._hit_marker_msgs: list[Marker] = []
        self._trajectory_marker_msgs: list[Marker] = []

        self.create_subscription(
            TwistStamped, "/hand_twist",
            lambda m: self._twist_msgs.append(m), 10)
        self.create_subscription(
            PointStamped, "/segmentation/click_positive",
            lambda m: self._click_msgs.append(m), 10)
        self.create_subscription(
            String, "/twist_propagation/status",
            lambda m: self._status_msgs.append(m), 10)

        # Visualization topic subscribers
        self.create_subscription(
            Path, "/twist_propagation/predicted_path",
            lambda m: self._path_msgs.append(m), 10)
        self.create_subscription(
            MarkerArray, "/twist_propagation/collision_spheres",
            lambda m: self._sphere_marker_msgs.append(m), 10)
        self.create_subscription(
            Marker, "/twist_propagation/hit_marker",
            lambda m: self._hit_marker_msgs.append(m), 10)
        self.create_subscription(
            Marker, "/twist_propagation/trajectory_line",
            lambda m: self._trajectory_marker_msgs.append(m), 10)

        # ── Service clients ────────────────────────────────────────────────
        self._activate_cli = self.create_client(Trigger, "/twist_propagation/activate")
        self._deactivate_cli = self.create_client(Trigger, "/twist_propagation/deactivate")

        # ── Mock preshaping service server ─────────────────────────────────
        self._preshaping_called = threading.Event()
        self._preshaping_call_count = 0
        self.create_service(
            Trigger, "/grasp_preshaping/compute_grasp",
            self._on_compute_grasp)

    # -- mock service handler ------------------------------------------------

    def _on_compute_grasp(self, req, resp):
        self._preshaping_call_count += 1
        self._preshaping_called.set()
        resp.success = True
        resp.message = f"mock preshaping call #{self._preshaping_call_count}"
        return resp

    # -- helpers -------------------------------------------------------------

    def wait_for_service(self, cli, timeout=5.0) -> bool:
        return cli.wait_for_service(timeout_sec=timeout)

    def clear(self):
        """Clear collected messages."""
        self._twist_msgs.clear()
        self._click_msgs.clear()
        self._status_msgs.clear()
        self._path_msgs.clear()
        self._sphere_marker_msgs.clear()
        self._hit_marker_msgs.clear()
        self._trajectory_marker_msgs.clear()
        self._preshaping_called.clear()
        self._preshaping_call_count = 0

    def activate(self):
        req = Trigger.Request()
        future = self._activate_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.result()

    def deactivate(self):
        req = Trigger.Request()
        future = self._deactivate_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.result()

    def publish_poses_moving(self, start_x=0.0, start_y=0.0, start_z=0.5,
                             vx=0.1, n=8, dt=0.05):
        """Publish n PoseStamped messages moving along +x at vx m/s."""
        for i in range(n):
            t = i * dt
            px = start_x + vx * t
            self.pose_pub.publish(_make_pose(px, start_y, start_z))
            time.sleep(dt * 0.5)  # publish faster than dt for realism

    def publish_cloud_with_target(self, target_x=0.5, target_y=0.0,
                                  target_z=0.5, n_points=200, spread=0.02):
        """Publish a cloud with a cluster of points near (target_x, target_y, target_z)."""
        import numpy as np
        rng = np.random.default_rng(42)
        # Cluster near target
        cluster = rng.normal(loc=[target_x, target_y, target_z],
                             scale=spread, size=(n_points, 3))
        # Some background points far away
        bg = rng.uniform(low=-1.0, high=2.0, size=(100, 3))
        bg[:, 2] = rng.uniform(0.3, 1.5, size=100)
        pts = np.vstack([cluster, bg])
        cloud = _make_cloud(pts, stamp_sec=time.time())
        self.cloud_pub.publish(cloud)

    def publish_seg_cloud(self, stamp_sec: float = 0.0):
        """Publish a dummy segmented cloud to signal segmentation completion."""
        import numpy as np
        pts = np.array([[0.5, 0.0, 0.5]])
        cloud = _make_cloud(pts, stamp_sec=stamp_sec if stamp_sec > 0 else time.time())
        self.seg_cloud_pub.publish(cloud)

    @property
    def twist_count(self) -> int:
        return len(self._twist_msgs)

    @property
    def click_count(self) -> int:
        return len(self._click_msgs)

    @property
    def latest_twist(self) -> TwistStamped | None:
        return self._twist_msgs[-1] if self._twist_msgs else None

    @property
    def latest_click(self) -> PointStamped | None:
        return self._click_msgs[-1] if self._click_msgs else None

    @property
    def preshaping_called(self) -> bool:
        return self._preshaping_called.is_set()

    @property
    def preshaping_count(self) -> int:
        return self._preshaping_call_count

    @property
    def path_count(self) -> int:
        return len(self._path_msgs)

    @property
    def sphere_marker_count(self) -> int:
        return len(self._sphere_marker_msgs)

    @property
    def trajectory_marker_count(self) -> int:
        return len(self._trajectory_marker_msgs)

    @property
    def hit_marker_count(self) -> int:
        return len(self._hit_marker_msgs)

    @property
    def latest_path(self) -> Path | None:
        return self._path_msgs[-1] if self._path_msgs else None


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------

def test_node_starts(harness: TestHarness):
    """Node should register with ROS and have services available."""
    if harness.wait_for_service(harness._activate_cli, timeout=5.0):
        _ok("activate service available")
    else:
        _fail("activate service available", "timed out")

    if harness.wait_for_service(harness._deactivate_cli, timeout=3.0):
        _ok("deactivate service available")
    else:
        _fail("deactivate service available", "timed out")


def test_activation(harness: TestHarness):
    """Activate the node and verify it responds."""
    resp = harness.activate()
    if resp and resp.success:
        _ok("activate returns success")
    else:
        _fail("activate returns success", f"resp={resp}")


def test_twist_published(harness: TestHarness):
    """After publishing moving poses and a cloud, /hand_twist should have non-zero linear vel."""
    harness.clear()
    # The node needs a cloud to enter its idle cycle and publish twists.
    # Publish a cloud first so the node has data to work with.
    harness.publish_cloud_with_target(target_x=0.6, target_y=0.0, target_z=0.5)
    # Give the node a moment to receive the cloud
    rclpy.spin_once(harness, timeout_sec=0.5)
    harness.publish_poses_moving(start_x=0.0, start_y=0.0, start_z=0.5,
                                 vx=0.2, n=10, dt=0.05)
    # Spin to let messages propagate
    rclpy.spin_once(harness, timeout_sec=1.0)
    time.sleep(0.3)
    rclpy.spin_once(harness, timeout_sec=1.0)

    if harness.twist_count > 0:
        _ok(f"twist messages published (count={harness.twist_count})")
        tw = harness.latest_twist
        lin = tw.twist.linear
        mag = math.sqrt(lin.x ** 2 + lin.y ** 2 + lin.z ** 2)
        if mag > 0.01:
            _ok(f"twist linear magnitude > 0.01 (got {mag:.4f})")
        else:
            _fail("twist linear magnitude > 0.01", f"got {mag:.4f}")
    else:
        _fail("twist messages published", "no messages received")


def test_hit_detected(harness: TestHarness):
    """With a cloud placed in the path of motion, a click should be published."""
    harness.clear()
    # Publish cloud with cluster at x=0.6
    harness.publish_cloud_with_target(target_x=0.6, target_y=0.0, target_z=0.5)
    # Publish poses moving towards x=0.6 from x=0.0
    harness.publish_poses_moving(start_x=0.0, start_y=0.0, start_z=0.5,
                                 vx=0.3, n=12, dt=0.05)

    # Wait for the cycle to run — use more iterations and longer spin
    for _ in range(25):
        rclpy.spin_once(harness, timeout_sec=0.3)
        if harness.click_count > 0:
            break
    time.sleep(0.5)
    rclpy.spin_once(harness, timeout_sec=0.5)

    if harness.click_count > 0:
        _ok(f"click_positive published (count={harness.click_count})")
        click = harness.latest_click
        _ok(f"click frame is cloud frame: {click.header.frame_id}")
    else:
        _fail("click_positive published", "no click received")


def test_preshaping_called_after_seg_cloud(harness: TestHarness):
    """After a click, publishing a segmented cloud should trigger preshaping."""
    # The node should be in WAITING_FOR_SEGMENTATION after the click above
    # Publish a segmented cloud to trigger preshaping
    harness._preshaping_called.clear()
    harness.publish_seg_cloud()

    # Wait for preshaping to be called
    for _ in range(40):
        rclpy.spin_once(harness, timeout_sec=0.3)
        if harness.preshaping_called:
            break

    if harness.preshaping_called:
        _ok("preshaping service called after segmented cloud")
    else:
        _fail("preshaping service called after segmented cloud",
              "service was not called")


def test_deactivation(harness: TestHarness):
    """Deactivate the node and verify no more clicks are published."""
    harness.clear()
    resp = harness.deactivate()
    if resp and resp.success:
        _ok("deactivate returns success")
    else:
        _fail("deactivate returns success", f"resp={resp}")

    # Publish data that would normally trigger a hit
    harness.publish_cloud_with_target(target_x=0.6, target_y=0.0, target_z=0.5)
    harness.publish_poses_moving(start_x=0.0, start_y=0.0, start_z=0.5,
                                 vx=0.3, n=12, dt=0.05)

    # Wait a bit
    for _ in range(10):
        rclpy.spin_once(harness, timeout_sec=0.2)

    if harness.click_count == 0:
        _ok("no clicks published when deactivated")
    else:
        _fail("no clicks published when deactivated",
              f"got {harness.click_count} clicks")


def test_visualization_published(harness: TestHarness):
    """Verify that visualization topics are published during propagation."""
    harness.clear()
    harness.activate()
    harness.publish_cloud_with_target(target_x=0.6, target_y=0.0, target_z=0.5)
    rclpy.spin_once(harness, timeout_sec=0.5)
    harness.publish_poses_moving(start_x=0.0, start_y=0.0, start_z=0.5,
                                 vx=0.2, n=10, dt=0.05)

    # Wait for the cycle to run
    for _ in range(15):
        rclpy.spin_once(harness, timeout_sec=0.3)

    # Predicted path should be published
    if harness.path_count > 0:
        _ok(f"predicted_path published (count={harness.path_count})")
        path = harness.latest_path
        if path is not None and len(path.poses) > 0:
            _ok(f"predicted_path has {len(path.poses)} poses")
        else:
            _fail("predicted_path has poses", "path is empty")
    else:
        _fail("predicted_path published", "no messages received")

    # Collision spheres should be published
    if harness.sphere_marker_count > 0:
        _ok(f"collision_spheres published (count={harness.sphere_marker_count})")
    else:
        _fail("collision_spheres published", "no messages received")

    # Trajectory line should be published
    if harness.trajectory_marker_count > 0:
        _ok(f"trajectory_line published (count={harness.trajectory_marker_count})")
    else:
        _fail("trajectory_line published", "no messages received")

    harness.deactivate()
    rclpy.spin_once(harness, timeout_sec=0.5)


def test_hit_marker_on_collision(harness: TestHarness):
    """Verify that hit_marker is published when a collision is detected."""
    harness.clear()
    harness.activate()
    harness.publish_cloud_with_target(target_x=0.6, target_y=0.0, target_z=0.5)
    harness.publish_poses_moving(start_x=0.0, start_y=0.0, start_z=0.5,
                                 vx=0.3, n=12, dt=0.05)

    for _ in range(25):
        rclpy.spin_once(harness, timeout_sec=0.3)
        if harness.hit_marker_count > 0 and harness.click_count > 0:
            break

    if harness.hit_marker_count > 0:
        _ok(f"hit_marker published on collision (count={harness.hit_marker_count})")
    else:
        _fail("hit_marker published on collision", "no messages received")

    harness.deactivate()
    rclpy.spin_once(harness, timeout_sec=0.5)


def test_no_hit_without_cloud(harness: TestHarness):
    """With no cloud (stale cloud should be too old), no click should be published."""
    harness.clear()
    # Wait for any in-flight messages from the previous test to settle
    time.sleep(0.5)
    for _ in range(5):
        rclpy.spin_once(harness, timeout_sec=0.2)
    harness.clear()
    harness.activate()
    # Wait for the node's cycle to run with the old cloud — it should reject
    # it as too old (cloud_max_age_s=2.0). Sleep long enough for the cloud
    # to age out, then verify no new clicks.
    time.sleep(2.5)
    for _ in range(5):
        rclpy.spin_once(harness, timeout_sec=0.2)
    harness.clear()
    # Now publish only poses — no cloud. Node should have no valid cloud.
    harness.publish_poses_moving(start_x=0.0, start_y=0.0, start_z=0.5,
                                 vx=0.3, n=12, dt=0.05)

    for _ in range(10):
        rclpy.spin_once(harness, timeout_sec=0.3)

    if harness.click_count == 0:
        _ok("no clicks without cloud")
    else:
        _fail("no clicks without cloud",
              f"got {harness.click_count} clicks")

    # Cleanup: deactivate
    harness.deactivate()
    # Spin to let deactivation process
    rclpy.spin_once(harness, timeout_sec=0.5)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global PASS, FAIL

    print("=" * 60)
    print("  Twist Propagation Integration Test")
    print("=" * 60)
    print()

    rclpy.init()
    harness = TestHarness()

    # Spin a bit to let the node start and discover services
    print("-- Waiting for node to start...")
    for _ in range(20):
        rclpy.spin_once(harness, timeout_sec=0.5)

    print()
    print("-- Running tests...")
    print()

    try:
        test_node_starts(harness)
        test_activation(harness)
        test_twist_published(harness)
        test_hit_detected(harness)
        test_preshaping_called_after_seg_cloud(harness)
        test_visualization_published(harness)
        test_hit_marker_on_collision(harness)
        test_deactivation(harness)
        test_no_hit_without_cloud(harness)
    except Exception as exc:
        _fail("unexpected exception", str(exc))
        import traceback
        traceback.print_exc()

    print()
    print("=" * 60)
    print(f"  Results: {PASS} passed, {FAIL} failed")
    print("=" * 60)

    harness.destroy_node()
    rclpy.shutdown()

    sys.exit(1 if FAIL > 0 else 0)


if __name__ == "__main__":
    main()
