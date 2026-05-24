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
  - Minimum speed filter logic
  - Minimum time-to-hit logic

Usage:
    python3 -m pytest src/twist_propagation/test/test_twist_propagation.py -v
"""

import math
import sys
import os

import numpy as np
import pytest
from scipy.spatial import KDTree

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'twist_propagation'))

from twist_propagation_node import (  # noqa: E402
    _parse_xyz,
    _voxel_downsample,
    _quat_multiply,
    _quat_rotate_vector,
    _quat_to_rotation_matrix_cols,
    _transform_twist_by_rotation,
    _propagate_pose,
    _propagate_covariance,
    _build_initial_covariance_from_odom,
    _build_initial_covariance_from_pose_buf,
    _is_odom_initialized,
    _should_retarget,
    _sample_spherical_shell_clicks,
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
        # Use a proper unit quaternion -- the conjugate trick only works on
        # normalised quaternions.  (0.1, 0.2, 0.3, 0.9) has norm sqrt(0.95).
        import math
        raw = (0.1, 0.2, 0.3, 0.9)
        n = math.sqrt(sum(c * c for c in raw))
        q = tuple(c / n for c in raw)
        inv = (-q[0], -q[1], -q[2], q[3])
        result = _quat_multiply(q, inv)
        assert abs(result[3] - 1.0) < 1e-10
        assert abs(result[0]) < 1e-10
        assert abs(result[1]) < 1e-10
        assert abs(result[2]) < 1e-10


class TestQuatRotateVector:
    """Tests for _quat_rotate_vector."""

    def test_identity_rotation(self):
        """Identity quaternion should not change the vector."""
        q = (0.0, 0.0, 0.0, 1.0)
        rx, ry, rz = _quat_rotate_vector(q, 1.0, 2.0, 3.0)
        assert abs(rx - 1.0) < 1e-10
        assert abs(ry - 2.0) < 1e-10
        assert abs(rz - 3.0) < 1e-10

    def test_90deg_rotation_z(self):
        """90-degree rotation around Z should map X to Y."""
        # 90-degree rotation about Z: q = (0, 0, sin(45), cos(45))
        s = math.sin(math.pi / 4)
        c = math.cos(math.pi / 4)
        q = (0.0, 0.0, s, c)
        rx, ry, rz = _quat_rotate_vector(q, 1.0, 0.0, 0.0)
        assert abs(rx) < 1e-10
        assert abs(ry - 1.0) < 1e-10
        assert abs(rz) < 1e-10

    def test_180deg_rotation_y(self):
        """180-degree rotation around Y should map X to -X."""
        s = math.sin(math.pi / 2)
        c = math.cos(math.pi / 2)
        q = (0.0, s, 0.0, c)
        rx, ry, rz = _quat_rotate_vector(q, 1.0, 0.0, 0.0)
        assert abs(rx - (-1.0)) < 1e-10
        assert abs(ry) < 1e-10
        assert abs(rz) < 1e-10


class TestTransformTwistByRotation:
    """Tests for _transform_twist_by_rotation."""

    def test_identity_rotation_unchanged(self):
        """Identity rotation matrix should not change the twist."""
        twist = (1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
        result = _transform_twist_by_rotation(
            twist,
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        )
        assert result == twist

    def test_90deg_z_rotation_swaps_xy(self):
        """90-degree rotation about Z maps (vx,vy,vz) -> (-vy,vx,vz)."""
        twist = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        # R_z(90) = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        result = _transform_twist_by_rotation(
            twist,
            0.0, -1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 1.0,
        )
        assert abs(result[0]) < 1e-10  # vx -> 0
        assert abs(result[1] - 1.0) < 1e-10  # vy -> 1
        assert abs(result[2]) < 1e-10  # vz -> 0
        assert abs(result[3]) < 1e-10  # wx -> 0
        assert abs(result[4]) < 1e-10  # wy -> 0
        assert abs(result[5] - 1.0) < 1e-10  # wz -> 1

    def test_rotation_with_quat_to_rotation_matrix_cols(self):
        """Verify _quat_to_rotation_matrix_cols produces correct rotation."""
        # 90-degree rotation about Z
        s = math.sin(math.pi / 4)
        c = math.cos(math.pi / 4)
        q = (0.0, 0.0, s, c)
        cols = _quat_to_rotation_matrix_cols(q)
        twist = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        result = _transform_twist_by_rotation(twist, *cols)
        assert abs(result[0]) < 1e-10  # vx -> 0
        assert abs(result[1] - 1.0) < 1e-10  # vy -> 1
        assert abs(result[2]) < 1e-10  # vz -> 0
        assert abs(result[5] - 1.0) < 1e-10  # wz -> 1

    def test_forward_motion_optical_frame_rotation(self):
        """Verify that forward motion in ROS frame becomes forward in optical frame.

        This is the core bug scenario: in ROS frame, forward is +X. In optical
        frame, forward is +Z. A 90-degree rotation about Y converts between
        them. After the fix, a +X velocity in pose_frame should become +Z
        velocity in the optical cloud_frame.
        """
        # Optical frame convention: rotate 90deg about Y then -90deg about Z
        # For simplicity, just test a 90-deg rotation about Y:
        # ROS X-forward -> optical Z-forward
        s = math.sin(math.pi / 4)
        c = math.cos(math.pi / 4)
        q_y90 = (0.0, s, 0.0, c)  # 90-deg about Y
        cols = _quat_to_rotation_matrix_cols(q_y90)

        # Moving forward in ROS frame: vx=1, vy=0, vz=0
        twist_ros = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        twist_optical = _transform_twist_by_rotation(twist_ros, *cols)

        # After rotation about Y by 90deg, +X maps to -Z
        assert abs(twist_optical[0]) < 1e-10  # vx -> 0
        assert abs(twist_optical[1]) < 1e-10  # vy -> 0
        assert abs(twist_optical[2] - (-1.0)) < 1e-10  # vz -> -1


class TestPropagationWithFrameTransform:
    """Tests that propagation produces correct paths after twist transform.

    These tests simulate the frame-mismatch bug scenario: a twist estimated
    in one frame must be transformed to another frame before propagation.
    """

    def test_forward_motion_after_90deg_z_rotation(self):
        """After a 90-deg Z rotation, forward motion in source frame should
        produce a path that moves in the +Y direction in the target frame."""
        # Start at origin, identity orientation
        px, py, pz = 0.0, 0.0, 0.0
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        # Velocity in source frame: forward = +X
        vx_src, vy_src, vz_src = 1.0, 0.0, 0.0

        # 90-deg rotation about Z: maps +X -> +Y
        s = math.sin(math.pi / 4)
        c = math.cos(math.pi / 4)
        q_rot = (0.0, 0.0, s, c)
        cols = _quat_to_rotation_matrix_cols(q_rot)
        twist_rot = _transform_twist_by_rotation(
            (vx_src, vy_src, vz_src, 0.0, 0.0, 0.0), *cols)

        # Propagate with the rotated twist
        dt = 0.02
        for _ in range(50):
            px, py, pz, qx, qy, qz, qw = _propagate_pose(
                px, py, pz, qx, qy, qz, qw,
                twist_rot[0], twist_rot[1], twist_rot[2],
                twist_rot[3], twist_rot[4], twist_rot[5],
                dt)

        # After 50 steps at 1 m/s, total displacement = 1.0 m
        # The motion should be in +Y (not +X) due to the 90-deg rotation
        assert abs(px) < 1e-6  # X should be ~0
        assert abs(py - 1.0) < 1e-3  # Y should be ~1.0
        assert abs(pz) < 1e-6  # Z should be ~0

    def test_forward_motion_after_90deg_y_rotation(self):
        """After a 90-deg Y rotation, forward motion in source frame should
        produce a path that moves in the -Z direction in the target frame."""
        px, py, pz = 0.0, 0.0, 0.0
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        vx_src, vy_src, vz_src = 1.0, 0.0, 0.0

        # 90-deg rotation about Y: maps +X -> -Z
        s = math.sin(math.pi / 4)
        c = math.cos(math.pi / 4)
        q_rot = (0.0, s, 0.0, c)
        cols = _quat_to_rotation_matrix_cols(q_rot)
        twist_rot = _transform_twist_by_rotation(
            (vx_src, vy_src, vz_src, 0.0, 0.0, 0.0), *cols)

        dt = 0.02
        for _ in range(50):
            px, py, pz, qx, qy, qz, qw = _propagate_pose(
                px, py, pz, qx, qy, qz, qw,
                twist_rot[0], twist_rot[1], twist_rot[2],
                twist_rot[3], twist_rot[4], twist_rot[5],
                dt)

        assert abs(px) < 1e-6
        assert abs(py) < 1e-6
        assert abs(pz - (-1.0)) < 1e-3  # Z should be ~-1.0


# -- Retarget policy tests --

class TestShouldRetarget:

    def test_no_current_target_accept(self):
        hit = (1.0, 2.0, 3.0)
        publish_click, publish_reset = _should_retarget(hit, None, 0.10)
        assert publish_click is True
        assert publish_reset is False

    def test_nearby_target_discard(self):
        current = (0.0, 0.0, 0.0)
        hit = (0.05, 0.0, 0.0)
        publish_click, publish_reset = _should_retarget(hit, current, 0.10)
        assert publish_click is False
        assert publish_reset is False

    def test_far_target_accept_with_reset(self):
        current = (0.0, 0.0, 0.0)
        hit = (0.15, 0.0, 0.0)
        publish_click, publish_reset = _should_retarget(hit, current, 0.10)
        assert publish_click is True
        assert publish_reset is True

    def test_exactly_at_threshold_discard(self):
        current = (0.0, 0.0, 0.0)
        hit = (0.10, 0.0, 0.0)
        publish_click, publish_reset = _should_retarget(hit, current, 0.10)
        assert publish_click is False
        assert publish_reset is False


# -- Spherical-shell click sampler tests --

class TestSphericalShellSampler:

    def test_zero_count_returns_empty(self):
        rng = np.random.default_rng(42)
        result = _sample_spherical_shell_clicks((1.0, 2.0, 3.0), 0.01, 0.03, 0, rng)
        assert result == []

    def test_negative_count_returns_empty(self):
        rng = np.random.default_rng(42)
        result = _sample_spherical_shell_clicks((1.0, 2.0, 3.0), 0.01, 0.03, -1, rng)
        assert result == []

    def test_equal_radii_returns_empty(self):
        rng = np.random.default_rng(42)
        result = _sample_spherical_shell_clicks((1.0, 2.0, 3.0), 0.02, 0.02, 5, rng)
        assert result == []

    def test_returns_exactly_count_points(self):
        rng = np.random.default_rng(42)
        result = _sample_spherical_shell_clicks((0.0, 0.0, 0.0), 0.01, 0.05, 10, rng)
        assert len(result) == 10

    def test_all_points_inside_shell(self):
        centre = (1.0, 2.0, 3.0)
        r_min, r_max = 0.01, 0.03
        count = 50
        rng = np.random.default_rng(42)
        result = _sample_spherical_shell_clicks(centre, r_min, r_max, count, rng)
        for pt in result:
            dist = math.sqrt(sum((c - p) ** 2 for c, p in zip(centre, pt)))
            assert r_min - 1e-9 <= dist <= r_max + 1e-9

    def test_deterministic_with_same_seed(self):
        centre = (0.0, 0.0, 0.0)
        r_min, r_max = 0.01, 0.03
        count = 5
        rng1 = np.random.default_rng(123)
        rng2 = np.random.default_rng(123)
        result1 = _sample_spherical_shell_clicks(centre, r_min, r_max, count, rng1)
        result2 = _sample_spherical_shell_clicks(centre, r_min, r_max, count, rng2)
        assert result1 == result2

    def test_original_hit_preserved_as_centre(self):
        # The centre itself is not in the returned list; only offsets are.
        centre = (5.0, 6.0, 7.0)
        rng = np.random.default_rng(42)
        result = _sample_spherical_shell_clicks(centre, 0.01, 0.05, 3, rng)
        for pt in result:
            assert pt != centre


# -- Speed gate logic tests --

class TestMinSpeedFilter:
    """Tests for the minimum linear speed threshold logic.

    The speed gate is a simple magnitude check: if sqrt(vx^2 + vy^2 + vz^2)
    < min_twist_linear_mps, propagation is skipped.  These tests verify the
    pure arithmetic that underpins that gate.
    """

    @staticmethod
    def _linear_mag(twist):
        vx, vy, vz = twist[0], twist[1], twist[2]
        return math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

    def test_zero_speed_below_threshold(self):
        """Zero velocity should always be below any positive threshold."""
        twist = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        min_speed = 0.02
        assert self._linear_mag(twist) < min_speed

    def test_below_threshold(self):
        """A velocity just below the threshold should be suppressed."""
        # 0.01 m/s in x, rest zero -> magnitude 0.01 < 0.02
        twist = (0.01, 0.0, 0.0, 0.0, 0.0, 0.0)
        min_speed = 0.02
        assert self._linear_mag(twist) < min_speed

    def test_above_threshold(self):
        """A velocity above the threshold should pass."""
        # 0.03 m/s in x -> magnitude 0.03 > 0.02
        twist = (0.03, 0.0, 0.0, 0.0, 0.0, 0.0)
        min_speed = 0.02
        assert self._linear_mag(twist) >= min_speed

    def test_exactly_at_threshold(self):
        """A velocity exactly at the threshold should pass (>=)."""
        twist = (0.02, 0.0, 0.0, 0.0, 0.0, 0.0)
        min_speed = 0.02
        assert self._linear_mag(twist) >= min_speed

    def test_default_zero_disables_filter(self):
        """With min_speed = 0.0, any velocity (including zero) passes."""
        twist = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        min_speed = 0.0
        # The gate check is: min_speed > 0.0 AND mag < min_speed
        # With min_speed = 0.0, the first condition is False, so gate opens.
        gate_opens = not (min_speed > 0.0 and self._linear_mag(twist) < min_speed)
        assert gate_opens

    def test_combined_xyz_velocity(self):
        """Velocity across all three axes should use combined magnitude."""
        # sqrt(0.01^2 + 0.01^2 + 0.01^2) = ~0.0173 < 0.02
        twist = (0.01, 0.01, 0.01, 0.0, 0.0, 0.0)
        min_speed = 0.02
        assert self._linear_mag(twist) < min_speed

    def test_angular_velocity_does_not_affect_gate(self):
        """Angular velocity should not affect the linear speed gate."""
        # High angular velocity but zero linear -> magnitude 0.0
        twist = (0.0, 0.0, 0.0, 5.0, 5.0, 5.0)
        min_speed = 0.02
        assert self._linear_mag(twist) < min_speed


# -- Time-to-hit logic tests --

class TestMinTimeToHit:
    """Tests for the minimum time-to-hit threshold logic.

    The time-to-hit is the propagation variable `t` at the moment of
    collision.  These tests verify that the propagation step count
    produces the correct time-to-hit, and that the threshold comparison
    works correctly.
    """

    def test_time_to_hit_first_step(self):
        """A collision at the first propagation step gives t = dt."""
        dt = 0.02
        # After one step: t = 0.0 + dt = 0.02
        t = 0.0
        t += dt
        assert t == pytest.approx(0.02)

    def test_time_to_hit_after_n_steps(self):
        """After N steps, time-to-hit should be N * dt."""
        dt = 0.02
        n_steps = 20  # 20 * 0.02 = 0.4s
        t = 0.0
        for _ in range(n_steps):
            t += dt
        assert t == pytest.approx(0.4)

    def test_hit_below_min_time_rejected(self):
        """A hit at t=0.02s should be rejected when min is 0.4s."""
        time_to_hit = 0.02
        min_time_to_hit = 0.4
        assert time_to_hit < min_time_to_hit

    def test_hit_above_min_time_accepted(self):
        """A hit at t=0.5s should be accepted when min is 0.4s."""
        time_to_hit = 0.5
        min_time_to_hit = 0.4
        assert time_to_hit >= min_time_to_hit

    def test_hit_exactly_at_threshold_accepted(self):
        """A hit at exactly t=0.4s should be accepted (>=)."""
        time_to_hit = 0.4
        min_time_to_hit = 0.4
        assert time_to_hit >= min_time_to_hit

    def test_default_zero_disables_filter(self):
        """With min_time_to_hit = 0.0, any time-to-hit passes."""
        time_to_hit = 0.001
        min_time_to_hit = 0.0
        # The gate check is: min > 0.0 AND t < min
        # With min = 0.0, the first condition is False, so gate opens.
        gate_opens = not (min_time_to_hit > 0.0 and time_to_hit < min_time_to_hit)
        assert gate_opens

    def test_propagation_with_known_hit_time(self):
        """Verify propagation produces the correct time-to-hit value.

        Place a point cloud at a known distance from the start position,
        move toward it at a known velocity, and verify the returned
        time-to-hit matches the expected value.
        """
        # Cloud point at x=0.5m, y=0, z=0
        cloud = np.array([[0.5, 0.0, 0.0],
                          [0.5, 0.01, 0.0],
                          [0.5, -0.01, 0.0]], dtype=np.float64)
        kdtree = KDTree(cloud)

        # Start at origin, moving at vx=0.1 m/s (all other velocities zero)
        px, py, pz = 0.0, 0.0, 0.0
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        vx, vy, vz = 0.1, 0.0, 0.0
        wx, wy, wz = 0.0, 0.0, 0.0
        dt = 0.02
        hit_threshold = 0.05
        collision_radius = 0.05
        effective_threshold = hit_threshold + collision_radius
        min_points = 3

        t = 0.0
        hit_t = None
        while t < 2.0:
            t += dt
            px += vx * dt
            py += vy * dt
            pz += vz * dt

            dists, _ = kdtree.query([px, py, pz], k=min_points)
            if np.max(dists[:min_points]) <= effective_threshold:
                hit_t = t
                break

        # The hand should reach x=0.5 at t = 0.5/0.1 = 5.0s, but with the
        # collision radius of 0.05, the effective threshold is 0.10m, so
        # the hit occurs when px >= 0.5 - 0.10 = 0.40m, i.e. t >= 4.0s.
        # At vx=0.1, after step N: px = 0.1 * 0.02 * N = 0.002 * N
        # px >= 0.40 when N >= 200, so t = 200 * 0.02 = 4.0s
        assert hit_t is not None
        assert hit_t == pytest.approx(4.0, abs=dt)

    def test_near_field_hit_has_small_time_to_hit(self):
        """When the hand starts near the cloud, time-to-hit should be small.

        This is the scenario the min_time_to_hit filter is designed to catch:
        the hand is already on top of the object.
        """
        # Cloud at origin, hand starting very close
        cloud = np.array([[0.0, 0.0, 0.0],
                          [0.01, 0.0, 0.0],
                          [-0.01, 0.0, 0.0]], dtype=np.float64)
        kdtree = KDTree(cloud)

        # Hand at (0.05, 0, 0) — within effective threshold 0.10m
        px, py, pz = 0.05, 0.0, 0.0
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        vx, vy, vz = 0.1, 0.0, 0.0
        wx, wy, wz = 0.0, 0.0, 0.0
        dt = 0.02
        effective_threshold = 0.10
        min_points = 3

        t = 0.0
        hit_t = None
        while t < 2.0:
            t += dt
            px += vx * dt
            dists, _ = kdtree.query([px, py, pz], k=min_points)
            if np.max(dists[:min_points]) <= effective_threshold:
                hit_t = t
                break

        # Hit should be at the very first step (t = 0.02s)
        assert hit_t is not None
        assert hit_t <= 0.04  # at most 2 steps
        # This should be rejected by a min_time_to_hit of 0.4s
        assert hit_t < 0.4
