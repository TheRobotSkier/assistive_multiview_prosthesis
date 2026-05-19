#!/usr/bin/env python3
"""Unit tests for twist_propagation_node pure functions.

These tests import and exercise the pure helper functions from the node
module without requiring a running ROS 2 environment.  They cover:
  - Propagation (stationary, linear, rotational)
  - Covariance (growth, symmetry, initial build)
  - Proximity (hit/no-hit with collision radius)
  - Voxel downsampling
  - Point cloud parsing
  - Odometry initialization checks

Usage:
    python3 -m pytest src/twist_propagation/test/test_twist_propagation.py -v
"""

import math
import sys
import os

import numpy as np
from scipy.spatial import KDTree

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'twist_propagation'))

from twist_propagation_node import (  # noqa: E402
    _parse_xyz,
    _voxel_downsample,
    _quat_multiply,
    _propagate_pose,
    _propagate_covariance,
    _build_initial_covariance_from_odom,
    _build_initial_covariance_from_pose_buf,
    _is_odom_initialized,
)


# -- Propagation tests (ported from V1) --

class TestPropagation:

    def test_zero_twist_stationary(self):
        px, py, pz = 1.0, 2.0, 3.0
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        vx = vy = vz = 0.0
        wx = wy = wz = 0.0
        for _ in range(50):
            px, py, pz, qx, qy, qz, qw = _propagate_pose(
                px, py, pz, qx, qy, qz, qw,
                vx, vy, vz, wx, wy, wz, dt=0.02)
        assert px == 1.0
        assert py == 2.0
        assert pz == 3.0
        assert abs(qx) < 1e-15
        assert abs(qy) < 1e-15
        assert abs(qz) < 1e-15
        assert abs(qw - 1.0) < 1e-15

    def test_constant_velocity_linear_advance(self):
        vx, vy, vz = 1.0, 2.0, 3.0
        dt = 0.02
        px, py, pz = 0.0, 0.0, 0.0
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        for _ in range(50):
            px, py, pz, qx, qy, qz, qw = _propagate_pose(
                px, py, pz, qx, qy, qz, qw,
                vx, vy, vz, 0.0, 0.0, 0.0, dt)
        expected_x = vx * dt * 50
        expected_y = vy * dt * 50
        expected_z = vz * dt * 50
        assert abs(px - expected_x) < 1e-10
        assert abs(py - expected_y) < 1e-10
        assert abs(pz - expected_z) < 1e-10

    def test_constant_velocity_rotation(self):
        wx, wy, wz = 0.0, 0.0, 1.0
        dt = 0.02
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        for _ in range(50):
            _, _, _, qx, qy, qz, qw = _propagate_pose(
                0.0, 0.0, 0.0, qx, qy, qz, qw,
                0.0, 0.0, 0.0, wx, wy, wz, dt)
        expected_angle = wz * dt * 50 / 2.0
        assert abs(qx) < 1e-10
        assert abs(qy) < 1e-10
        assert abs(qz - math.sin(expected_angle)) < 1e-6
        assert abs(qw - math.cos(expected_angle)) < 1e-6


# -- Covariance tests (ported from V1) --

class TestCovariance:

    def test_covariance_grows_monotonically(self):
        P = np.eye(12, dtype=np.float64) * 0.01
        sigma_v_sq = 0.1
        sigma_w_sq = 0.5
        prev_trace = float(np.trace(P))
        for _ in range(50):
            P = _propagate_covariance(P, dt=0.02,
                                      sigma_v_sq=sigma_v_sq,
                                      sigma_w_sq=sigma_w_sq)
            new_trace = float(np.trace(P))
            assert new_trace > prev_trace
            prev_trace = new_trace

    def test_propagate_covariance_symmetry(self):
        P = np.random.randn(12, 12)
        P = P @ P.T
        sigma_v_sq = 0.1
        sigma_w_sq = 0.5
        for _ in range(50):
            P = _propagate_covariance(P, dt=0.02,
                                      sigma_v_sq=sigma_v_sq,
                                      sigma_w_sq=sigma_w_sq)
            assert np.allclose(P, P.T, atol=1e-12)

    def test_build_initial_covariance_twist_floor(self):
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Header
        odom = Odometry()
        odom.header = Header()
        odom.twist.covariance = [0.0] * 36
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 0.01
        odom.pose.covariance[21] = 0.001
        odom.pose.covariance[28] = 0.001
        odom.pose.covariance[35] = 0.001
        sigma_v_sq = 0.1
        sigma_w_sq = 0.5
        P = _build_initial_covariance_from_odom(odom, sigma_v_sq, sigma_w_sq)
        assert abs(P[0, 0] - 0.01) < 1e-12
        assert abs(P[1, 1] - 0.01) < 1e-12
        assert P[6, 6] >= sigma_v_sq - 1e-12
        assert P[7, 7] >= sigma_v_sq - 1e-12
        assert P[8, 8] >= sigma_v_sq - 1e-12
        assert P[9, 9] >= sigma_w_sq - 1e-12
        assert P[10, 10] >= sigma_w_sq - 1e-12
        assert P[11, 11] >= sigma_w_sq - 1e-12

    def test_build_covariance_from_pose_buf(self):
        from collections import deque
        buf = deque(maxlen=10)
        for i in range(5):
            buf.append((i * 0.1, float(i) * 0.1, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0, "world"))
        P = _build_initial_covariance_from_pose_buf(buf, 0.1, 0.5)
        assert P.shape == (12, 12)
        # Position covariance should be non-zero since poses are spread out
        assert P[0, 0] > 0
        # Velocity floor should be applied
        assert P[6, 6] >= 0.1 - 1e-12
        assert P[9, 9] >= 0.5 - 1e-12


# -- Proximity tests (with collision radius) --

class TestProximity:

    def test_hit_near_points_no_radius(self):
        """Without collision radius, must be very close to hit."""
        points = np.array([
            [0.0, 0.0, 0.0],
            [0.01, 0.01, 0.01],
            [-0.01, -0.01, -0.01],
            [0.02, 0.0, 0.0],
            [0.0, 0.02, 0.0],
            [0.0, 0.0, 0.02],
        ], dtype=np.float64)
        kdtree = KDTree(points)
        # Query point at 0.005,0.005,0.005 -- very close to cluster
        dists, _ = kdtree.query([0.005, 0.005, 0.005], k=3)
        hit_threshold = 0.05
        collision_radius = 0.0
        assert np.max(dists[:3]) <= hit_threshold + collision_radius

    def test_hit_with_collision_radius(self):
        """With collision radius, hit from further away."""
        points = np.array([
            [0.0, 0.0, 0.0],
            [0.01, 0.01, 0.01],
            [-0.01, -0.01, -0.01],
        ], dtype=np.float64)
        kdtree = KDTree(points)
        # Query point at 0.1,0,0 -- 10cm away from cluster
        dists, _ = kdtree.query([0.1, 0.0, 0.0], k=3)
        # Without radius: dists ~0.1 > 0.05 threshold -- no hit
        assert np.max(dists[:3]) > 0.05
        # With 0.10m radius: effective threshold = 0.05 + 0.10 = 0.15
        assert np.max(dists[:3]) <= 0.05 + 0.10

    def test_no_hit_far_from_points(self):
        points = np.array([
            [0.0, 0.0, 0.0],
            [0.01, 0.01, 0.01],
            [-0.01, -0.01, -0.01],
        ], dtype=np.float64)
        kdtree = KDTree(points)
        dists, _ = kdtree.query([100.0, 100.0, 100.0], k=3)
        assert np.max(dists[:3]) > 0.05 + 0.10  # even with radius, too far


# -- Odometry guard tests (ported from V1) --

class TestOdomGuard:

    def test_odom_uninitialized_rejected(self):
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Header
        odom = Odometry()
        odom.header = Header()
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance = [0.0] * 36
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0
        assert not _is_odom_initialized(odom)

    def test_odom_initialized_accepted(self):
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Header
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp.sec = 100
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 2.0
        odom.pose.pose.position.z = 3.0
        odom.pose.pose.orientation.x = 0.1
        odom.pose.pose.orientation.y = 0.2
        odom.pose.pose.orientation.z = 0.3
        odom.pose.pose.orientation.w = 0.927
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 0.01
        odom.twist.twist.linear.x = 0.5
        odom.twist.twist.linear.y = 0.1
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.1
        assert _is_odom_initialized(odom)

    def test_odom_nan_rejected(self):
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Header
        odom = Odometry()
        odom.header = Header()
        odom.pose.pose.position.x = float('nan')
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.01
        assert not _is_odom_initialized(odom)

    def test_odom_inf_rejected(self):
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Header
        odom = Odometry()
        odom.header = Header()
        odom.pose.pose.orientation.w = float('inf')
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.01
        assert not _is_odom_initialized(odom)


# -- Voxel downsampling tests (ported from V1) --

class TestVoxelDownsample:

    def test_empty_input(self):
        result = _voxel_downsample(
            np.array([], dtype=np.float64).reshape(0, 3), 0.02)
        assert len(result) == 0

    def test_single_point(self):
        pts = np.array([[1.0, 2.0, 3.0]])
        result = _voxel_downsample(pts, 0.02)
        assert len(result) == 1

    def test_merge_duplicates(self):
        pts = np.array([
            [0.001, 0.001, 0.001],
            [0.002, 0.002, 0.002],
            [1.000, 1.000, 1.000],
        ])
        result = _voxel_downsample(pts, 1.0)
        assert len(result) == 2

    def test_leaf_zero_passthrough(self):
        pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        result = _voxel_downsample(pts, 0.0)
        assert len(result) == 2

    def test_preserves_order(self):
        pts = np.array([
            [0.0, 0.0, 0.0],
            [0.001, 0.001, 0.001],  # same voxel as above
            [2.0, 2.0, 2.0],
        ])
        result = _voxel_downsample(pts, 0.02)
        assert len(result) == 2
        # First point in each voxel is kept
        assert result[0][0] == 0.0
        assert result[1][0] == 2.0


# -- Point cloud parsing tests (ported from V1) --

class TestParseXYZ:

    def test_basic_parsing(self):
        from sensor_msgs.msg import PointCloud2, PointField
        cloud = PointCloud2()
        cloud.width = 2
        cloud.height = 1
        cloud.point_step = 16
        cloud.row_step = 32
        cloud.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        data = np.zeros(2, dtype=[('x', np.float32), ('y', np.float32),
                                  ('z', np.float32), ('pad', np.float32)])
        data['x'] = [1.0, 2.0]
        data['y'] = [3.0, 4.0]
        data['z'] = [5.0, 6.0]
        cloud.data = data.tobytes()
        result = _parse_xyz(cloud)
        assert result is not None
        assert result.shape == (2, 3)
        assert np.allclose(result, [[1.0, 3.0, 5.0], [2.0, 4.0, 6.0]])

    def test_empty_cloud(self):
        from sensor_msgs.msg import PointCloud2
        cloud = PointCloud2()
        cloud.width = 0
        cloud.height = 0
        cloud.data = b''
        result = _parse_xyz(cloud)
        assert result is None

    def test_nan_filtered(self):
        from sensor_msgs.msg import PointCloud2, PointField
        cloud = PointCloud2()
        cloud.width = 2
        cloud.height = 1
        cloud.point_step = 12
        cloud.row_step = 24
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        import struct
        buf = struct.pack("fff", 1.0, 2.0, 3.0) + struct.pack("fff", float('nan'), 2.0, 3.0)
        cloud.data = buf
        result = _parse_xyz(cloud)
        assert result is not None
        assert result.shape == (1, 3)


# -- Quaternion multiply test --

class TestQuatMultiply:

    def test_identity(self):
        q = (0.0, 0.0, 0.0, 1.0)
        result = _quat_multiply(q, q)
        assert abs(result[3] - 1.0) < 1e-10

    def test_inverse(self):
        q = (0.1, 0.2, 0.3, 0.9)
        inv = (-q[0], -q[1], -q[2], q[3])
        result = _quat_multiply(q, inv)
        assert abs(result[3] - 1.0) < 1e-10
        assert abs(result[0]) < 1e-10
        assert abs(result[1]) < 1e-10
        assert abs(result[2]) < 1e-10
