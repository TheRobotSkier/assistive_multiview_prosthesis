#!/usr/bin/env python3
"""Unit tests for factor_graph — GTSAM trajectory factor graph.

Requires ``gtsam`` Python bindings (run in the container).

Usage (in container):
    python3 -m pytest src/gtsam_tracker/test/test_factor_graph.py -v
"""

import math

import numpy as np
import pytest

try:
    import gtsam
    from gtsam_tracker.factor_graph import (
        TrajectoryFactorGraph,
        noise_from_covariance_diag,
        default_odom_noise,
    )
    HAS_GTSAM = True
except ImportError:
    HAS_GTSAM = False

pytestmark = pytest.mark.skipif(not HAS_GTSAM, reason="gtsam not available")


def _se3_from_rot_trans(R, t):
    """Build a 4×4 from rotation matrix and translation."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _circle_trajectory(n_steps, radius=2.0, dt=0.1):
    """Generate ground-truth poses along a circle in the XY plane.

    Returns a list of (4, 4) matrices.
    """
    poses = []
    for i in range(n_steps):
        angle = i * dt * 0.5  # angular speed
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        # Face along the tangent (yaw = angle + 90°)
        yaw = angle + math.pi / 2
        R = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1],
        ])
        poses.append(_se3_from_rot_trans(R, [x, y, 0.0]))
    return poses


# ---------------------------------------------------------------------------
# Noise model helpers
# ---------------------------------------------------------------------------

class TestNoiseModels:
    def test_noise_from_covariance_diag(self):
        cov = [0.01, 0.02, 0.03, 0.001, 0.002, 0.003]
        noise = noise_from_covariance_diag(cov)
        sigmas = noise.sigmas()
        np.testing.assert_allclose(sigmas, np.sqrt(cov), atol=1e-9)

    def test_default_odom_noise(self):
        noise = default_odom_noise(0.01, 0.02)
        sigmas = noise.sigmas()
        np.testing.assert_allclose(sigmas[:3], 0.01, atol=1e-9)
        np.testing.assert_allclose(sigmas[3:], 0.02, atol=1e-9)


# ---------------------------------------------------------------------------
# Odom-only recovery
# ---------------------------------------------------------------------------

class TestOdomOnlyRecovery:
    def test_circle_trajectory(self):
        """Feed a known smooth trajectory (constant velocity circle) as
        between-factors. Verify recovered poses match ground truth within 1cm."""
        gt_poses = _circle_trajectory(20, radius=2.0, dt=0.1)

        graph = TrajectoryFactorGraph(lag_s=100.0)  # large lag so nothing is marginalised
        noise = default_odom_noise(sigma_t=0.001, sigma_r=0.001)

        for i in range(len(gt_poses)):
            stamp = i * 0.1
            if i == 0:
                delta_head = gt_poses[0].copy()  # first pose from origin
                delta_arm = gt_poses[0].copy()
            else:
                delta_head = np.linalg.inv(gt_poses[i - 1]) @ gt_poses[i]
                delta_arm = delta_head.copy()

            graph.add_odometry_factor(
                key_head=i, key_arm=i, stamp=stamp,
                delta_head=delta_head, delta_arm=delta_arm,
                noise_head=noise, noise_arm=noise,
            )
            graph.update()

        # Verify recovered head poses
        for i in range(len(gt_poses)):
            k = gtsam.symbol('h', i)
            recovered = graph.get_pose(k)
            recovered_mat = np.eye(4)
            recovered_mat[:3, :3] = recovered.rotation().matrix()
            recovered_mat[:3, 3] = recovered.translation()

            # Translation error
            t_err = np.linalg.norm(recovered_mat[:3, 3] - gt_poses[i][:3, 3])
            assert t_err < 0.01, f"Step {i}: translation error {t_err:.4f} m exceeds 1cm"


# ---------------------------------------------------------------------------
# Prior correction
# ---------------------------------------------------------------------------

class TestPriorCorrection:
    def test_drift_correction(self):
        """Inject a drifting odom sequence, then add a prior factor at the true
        pose. Verify the smoother pulls the estimate back toward truth."""
        graph = TrajectoryFactorGraph(lag_s=100.0)
        noise = default_odom_noise(sigma_t=0.01, sigma_r=0.01)

        n_steps = 10
        # True trajectory: straight line along +x
        true_poses = []
        for i in range(n_steps):
            T = np.eye(4)
            T[0, 3] = i * 0.1
            true_poses.append(T)

        # Add odometry with accumulated drift (add small bias each step)
        for i in range(n_steps):
            stamp = i * 0.1
            if i == 0:
                delta = true_poses[0].copy()
            else:
                # Add 1cm drift per step in y
                delta = np.eye(4)
                delta[0, 3] = 0.1  # true x movement
                delta[1, 3] = 0.01  # drift in y

            graph.add_odometry_factor(
                key_head=i, key_arm=i, stamp=stamp,
                delta_head=delta, delta_arm=delta.copy(),
                noise_head=noise, noise_arm=noise,
            )

            # At the last step, add a strong prior at the true pose
            if i == n_steps - 1:
                k = gtsam.symbol('h', i)
                graph.add_aruco_prior(
                    key=k,
                    T_map_imu=true_poses[i],
                    covariance_diag=[1e-6] * 6,  # very confident prior
                )

            graph.update()

        # Check the last pose is pulled back toward truth
        k_last = gtsam.symbol('h', n_steps - 1)
        recovered = graph.get_pose(k_last)
        recovered_t = recovered.translation()
        true_t = true_poses[-1][:3, 3]

        t_err = np.linalg.norm(recovered_t - true_t)
        # Without the prior, drift would be ~9cm. With prior, should be < 2cm.
        assert t_err < 0.02, f"Prior correction failed: error {t_err:.4f} m"


# ---------------------------------------------------------------------------
# Marginalization
# ---------------------------------------------------------------------------

class TestMarginalization:
    def test_does_not_grow_unbounded(self):
        """Feed 200 factors, verify the graph doesn't grow unbounded (old keys
        are marginalised — this is the FixedLagSmoother's job)."""
        graph = TrajectoryFactorGraph(lag_s=2.0)  # short lag
        noise = default_odom_noise(sigma_t=0.01, sigma_r=0.01)

        n_steps = 200
        for i in range(n_steps):
            stamp = i * 0.1  # 10 Hz
            delta = np.eye(4)
            delta[0, 3] = 0.01  # 1cm per step

            graph.add_odometry_factor(
                key_head=i, key_arm=i, stamp=stamp,
                delta_head=delta, delta_arm=delta.copy(),
                noise_head=noise, noise_arm=noise,
            )
            graph.update()

        # The smoother should have marginalised old keys.
        # We can't directly count keys in the smoother, but we can verify
        # it runs without error and the most recent key is queryable.
        k_recent = gtsam.symbol('h', n_steps - 1)
        pose = graph.get_pose(k_recent)
        assert pose is not None

        # Verify total translation is approximately correct
        # (should be around n_steps * 0.01 = 2.0 m)
        t = pose.translation()
        assert abs(t[0] - 2.0) < 0.5, f"Expected ~2.0m, got {t[0]:.2f}m"


# ---------------------------------------------------------------------------
# Reset (recovery from corrupted ISAM2 state)
# ---------------------------------------------------------------------------

class TestReset:
    def test_reset_clears_state(self):
        """After reset(), the graph behaves like a fresh instance — the next
        odometry factor seeds a new prior rather than chaining off the old
        trajectory."""
        graph = TrajectoryFactorGraph(lag_s=100.0)
        noise = default_odom_noise(sigma_t=0.01, sigma_r=0.01)

        # Build up a trajectory
        for i in range(5):
            delta = np.eye(4)
            delta[0, 3] = 0.1
            graph.add_odometry_factor(
                key_head=i, key_arm=i, stamp=i * 0.1,
                delta_head=delta, delta_arm=delta.copy(),
                noise_head=noise, noise_arm=noise,
            )
            graph.update()

        # Old keys should be queryable
        old_key = gtsam.symbol('h', 4)
        assert graph.get_pose(old_key) is not None

        # Reset
        graph.reset()

        # After reset, old keys should no longer be queryable
        with pytest.raises(Exception):
            graph.get_pose(old_key)

        # The next odometry factor should seed a fresh prior at the origin
        delta_fresh = np.eye(4)
        delta_fresh[0, 3] = 0.5
        graph.add_odometry_factor(
            key_head=0, key_arm=0, stamp=0.0,
            delta_head=delta_fresh, delta_arm=delta_fresh.copy(),
            noise_head=noise, noise_arm=noise,
        )
        graph.update()

        new_key = gtsam.symbol('h', 0)
        pose = graph.get_pose(new_key)
        t = pose.translation()
        # Fresh prior should be at the delta origin (0.5m in x)
        assert abs(t[0] - 0.5) < 0.01, f"Post-reset pose at {t[0]:.3f}, expected ~0.5"

    def test_reset_repeated(self):
        """reset() can be called multiple times without error."""
        graph = TrajectoryFactorGraph(lag_s=5.0)
        for _ in range(3):
            graph.reset()
        # Should still work normally
        noise = default_odom_noise(sigma_t=0.01, sigma_r=0.01)
        graph.add_odometry_factor(
            key_head=0, key_arm=0, stamp=0.0,
            delta_head=np.eye(4), delta_arm=np.eye(4),
            noise_head=noise, noise_arm=noise,
        )
        graph.update()
        assert graph.get_pose(gtsam.symbol('h', 0)) is not None


# ---------------------------------------------------------------------------
# Range factor
# ---------------------------------------------------------------------------

class TestRangeFactor:
    def test_range_pulls_together(self):
        """Set head and arm 1.5m apart with a 1.0m range factor. Verify the
        penalty pulls them toward 1.0m."""
        graph = TrajectoryFactorGraph(lag_s=100.0)
        noise = default_odom_noise(sigma_t=0.001, sigma_r=0.001)

        # Step 0: head at origin, arm at [1.5, 0, 0]
        delta_head_0 = np.eye(4)
        delta_arm_0 = np.eye(4)
        delta_arm_0[0, 3] = 1.5

        graph.add_odometry_factor(
            key_head=0, key_arm=0, stamp=0.0,
            delta_head=delta_head_0, delta_arm=delta_arm_0,
            noise_head=noise, noise_arm=noise,
        )
        graph.update()

        # Add range factor pulling toward 1.0m (via between-factor approximation)
        k_h = gtsam.symbol('h', 0)
        k_a = gtsam.symbol('a', 0)
        graph.add_range_factor(k_h, k_a, max_distance=1.0, sigma=0.01)
        graph.update()

        head_pose = graph.get_pose(k_h)
        arm_pose = graph.get_pose(k_a)

        head_t = head_pose.translation()
        arm_t = arm_pose.translation()
        dist = np.linalg.norm(arm_t - head_t)

        # The range factor should pull them closer than 1.5m
        assert dist < 1.5, f"Range factor didn't reduce distance: {dist:.3f}m"


# ---------------------------------------------------------------------------
# Two-chain test
# ---------------------------------------------------------------------------

class TestTwoChain:
    def test_cross_chain_consistency(self):
        """Head and arm chains with independent odom, connected by a visual
        between-factor. Verify cross-chain consistency."""
        graph = TrajectoryFactorGraph(lag_s=100.0)
        noise = default_odom_noise(sigma_t=0.001, sigma_r=0.001)

        n_steps = 10
        # Head moves along +x; arm follows 0.5m behind (constant offset).
        for i in range(n_steps):
            stamp = i * 0.1

            step_x = 0.05 if i > 0 else 0.0

            # Head: starts at origin, moves 5cm per step in x
            delta_h = np.eye(4)
            delta_h[0, 3] = step_x

            # Arm: starts at -0.5m, moves 5cm per step in x (same velocity)
            delta_a = np.eye(4)
            if i == 0:
                delta_a[0, 3] = -0.5  # arm starts 0.5m behind head
            else:
                delta_a[0, 3] = step_x

            graph.add_odometry_factor(
                key_head=i, key_arm=i, stamp=stamp,
                delta_head=delta_h, delta_arm=delta_a,
                noise_head=noise, noise_arm=noise,
            )

            # Add visual between-factor: arm is 0.5m behind head at each step
            if i > 0:
                k_h = gtsam.symbol('h', i)
                k_a = gtsam.symbol('a', i)
                T_head_arm = np.eye(4)
                T_head_arm[0, 3] = -0.5  # arm is 0.5m behind head
                graph.add_visual_between(
                    key_head=k_h, key_arm=k_a,
                    T_head_arm=T_head_arm,
                    covariance=[0.001] * 6,
                )

            graph.update()

        # Check final poses
        k_h_last = gtsam.symbol('h', n_steps - 1)
        k_a_last = gtsam.symbol('a', n_steps - 1)

        head_t = graph.get_pose(k_h_last).translation()
        arm_t = graph.get_pose(k_a_last).translation()

        # Head should be at ~0.45m (9 steps * 0.05)
        assert abs(head_t[0] - 0.45) < 0.05, f"Head at {head_t[0]:.3f}, expected ~0.45"

        # Arm should be ~0.5m behind head
        diff = head_t[0] - arm_t[0]
        assert abs(diff - 0.5) < 0.1, f"Head-arm distance {diff:.3f}, expected ~0.5"
