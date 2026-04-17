#!/usr/bin/env python3
"""
Single-IMU EKF prototype (plain Python, direct I2C read)

Purpose
-------
This script estimates the IMU pose relative to its start pose using:
- gyroscope
- accelerometer

Current stage
-------------
This version includes:
- nominal state propagation from IMU
- 15-state error-state covariance propagation
- accelerometer gravity-direction update
- startup initialization
- runtime stationary detector
- zero-velocity update (ZUPT)
- stationary gyro-bias refinement
- continuous live loop with logging

Notes
-----
- This version is a normal Python script, not a ROS 2 node.
- IMU data is read directly over I2C.
- Orientation is stored internally as a quaternion [w, x, y, z].
- Roll, pitch, yaw are computed only for debugging / printing.
- Position will drift over time because there is no external correction yet.
"""

import time
import math
import numpy as np
from smbus2 import SMBus


# ============================================================
# Quaternion and math helper functions
# ============================================================

def quat_normalize(q):
    """Normalize quaternion q = [w, x, y, z]."""
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n


def quat_mul(q1, q2):
    """Hamilton product of two quaternions q = [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=np.float64)


def quat_from_small_angle(dtheta):
    """
    Convert a small rotation vector dtheta = [dx, dy, dz]
    into a quaternion [w, x, y, z].
    """
    dtheta = np.asarray(dtheta, dtype=np.float64)
    angle = np.linalg.norm(dtheta)

    if angle < 1e-12:
        return quat_normalize(np.array([
            1.0,
            0.5 * dtheta[0],
            0.5 * dtheta[1],
            0.5 * dtheta[2]
        ], dtype=np.float64))

    axis = dtheta / angle
    half = 0.5 * angle
    s = math.sin(half)

    return np.array([
        math.cos(half),
        axis[0] * s,
        axis[1] * s,
        axis[2] * s
    ], dtype=np.float64)


def quat_to_rotmat(q):
    """
    Convert quaternion q = [w, x, y, z] into a 3x3 rotation matrix.

    Returns rotation_world_from_body.
    """
    q = quat_normalize(q)
    w, x, y, z = q

    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def quat_to_rpy(q):
    """Convert quaternion q = [w, x, y, z] to roll, pitch, yaw in radians."""
    q = quat_normalize(q)
    w, x, y, z = q

    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w*y - z*x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def rpy_to_quat(roll_rad, pitch_rad, yaw_rad):
    """Convert roll, pitch, yaw [rad] into quaternion [w, x, y, z]."""
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)

    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)

    quat = np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    ], dtype=np.float64)

    return quat_normalize(quat)


def quat_from_accel_gravity(accel_mean_mps2, yaw_rad=0.0):
    """
    Estimate initial orientation quaternion from averaged stationary accelerometer.
    Assumes world frame has +Z up and a level stationary IMU measures about [0,0,+g].
    """
    ax, ay, az = accel_mean_mps2
    roll_rad = math.atan2(ay, az)
    pitch_rad = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return rpy_to_quat(roll_rad, pitch_rad, yaw_rad)


def apply_small_angle_quat_correction(quat, dtheta):
    """Apply a small-angle correction dtheta to quaternion quat."""
    delta_quat = quat_from_small_angle(dtheta)
    return quat_normalize(quat_mul(quat, delta_quat))


def skew_symmetric(vec3):
    """Return the 3x3 skew-symmetric matrix of a 3D vector."""
    x, y, z = vec3
    return np.array([
        [0.0, -z,  y],
        [z,   0.0, -x],
        [-y,  x,   0.0]
    ], dtype=np.float64)


def compute_expected_gravity_dir_body_from_quat(quat, gravity_world):
    """Compute expected normalized gravity direction in the body frame."""
    rotation_world_from_body = quat_to_rotmat(quat)
    rotation_body_from_world = rotation_world_from_body.T

    gravity_dir_world = -gravity_world / np.linalg.norm(gravity_world)
    expected_dir_body = rotation_body_from_world @ gravity_dir_world
    expected_dir_body = expected_dir_body / np.linalg.norm(expected_dir_body)

    return expected_dir_body


def compute_accel_measurement_jacobian_numerical(quat, gravity_world, eps=1e-6):
    """
    Numerical Jacobian of expected gravity direction in body frame
    with respect to 3D small-angle attitude error dtheta.
    """
    expected_dir_body = compute_expected_gravity_dir_body_from_quat(quat, gravity_world)
    H_theta_num = np.zeros((3, 3), dtype=np.float64)

    for axis_index in range(3):
        dtheta = np.zeros(3, dtype=np.float64)
        dtheta[axis_index] = eps

        perturbed_quat = apply_small_angle_quat_correction(quat, dtheta)
        perturbed_expected_dir_body = compute_expected_gravity_dir_body_from_quat(
            perturbed_quat,
            gravity_world
        )

        H_theta_num[:, axis_index] = (
            perturbed_expected_dir_body - expected_dir_body
        ) / eps

    return H_theta_num


def compute_accel_measurement_jacobian_analytic(quat, gravity_world):
    """
    Analytic Jacobian of expected gravity direction in body frame
    with respect to 3D small-angle attitude error dtheta.

    For this implementation we use the right-multiplicative correction:
        q <- q * dq
    so the Jacobian is +skew(expected_dir_body).
    """
    expected_dir_body = compute_expected_gravity_dir_body_from_quat(quat, gravity_world)
    return skew_symmetric(expected_dir_body)


def apply_axis_remap(raw_vec, remap_matrix):
    """Convert a 3D vector from sensor frame into chosen body frame."""
    return remap_matrix @ raw_vec


# ============================================================
# EKF configuration
# ============================================================

class ImuEkfConfig:
    """Configuration values for startup behavior, updates, and thresholds."""

    def __init__(self):
        # Startup mode: "instant" or "stationary_calibration"
        self.startup_mode = "instant"

        # Optional user-defined initial pose
        self.use_user_initial_pose = False
        self.initial_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.initial_rpy_deg = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # Startup stationary calibration
        self.startup_sample_count = 100
        self.startup_sample_dt_sec = 0.01

        # Startup stillness thresholds
        self.still_gyro_max_rps = 0.08
        self.still_accel_norm_min_mps2 = 9.3
        self.still_accel_norm_max_mps2 = 10.4
        self.still_accel_std_max_mps2 = 0.25

        # Logging
        self.print_interval_sec = 0.20

        # Magnetometer placeholders for later
        self.use_magnetometer = False
        self.use_mag_calibration = False
        self.mag_bias = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.mag_scale_matrix = np.eye(3, dtype=np.float64)

        # Runtime stationary detector
        self.runtime_stationary_use = True
        self.runtime_still_gyro_max_rps = 0.06
        self.runtime_still_accel_norm_min_mps2 = 9.70
        self.runtime_still_accel_norm_max_mps2 = 10.05
        self.runtime_stationary_enter_count = 40
        self.runtime_stationary_exit_count = 12
        self.runtime_zupt_min_stationary_time_sec = 0.20
        self.runtime_gyro_bias_refine_min_stationary_time_sec = 1.0

        # Accelerometer tilt update gating
        self.accel_update_use = True
        self.accel_update_only_when_near_gravity = True
        self.accel_update_max_innovation_norm = 0.20

        # Zero-velocity update settings
        self.zupt_vel_meas_std_mps = 0.01

        # Runtime gyro-bias refinement settings
        self.runtime_gyro_bias_meas_std_rps = 0.003

        # Runtime accelerometer-bias refinement:
        # disabled in this version to keep behavior simpler and avoid
        # overfitting bias from stationary periods before visual correction
        self.enable_runtime_accel_bias_updates = False


# ============================================================
# EKF state
# ============================================================

class ImuEkfState:
    """
    Holds the nominal EKF state and its error covariance.

    Error-state ordering:
        [dpos, dvel, dtheta, dbias_gyro, dbias_acc]
    """

    def __init__(self, config):
        self.config = config

        # Nominal state
        self.pos = np.zeros(3, dtype=np.float64)
        self.vel = np.zeros(3, dtype=np.float64)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.bias_gyro = np.zeros(3, dtype=np.float64)
        self.bias_acc = np.zeros(3, dtype=np.float64)

        # Error covariance
        self.cov = np.zeros((15, 15), dtype=np.float64)
        self.cov[0:3, 0:3] = np.eye(3) * 1e-6
        self.cov[3:6, 3:6] = np.eye(3) * 1e-3
        self.cov[6:9, 6:9] = np.eye(3) * 1e-2
        self.cov[9:12, 9:12] = np.eye(3) * 1e-3
        self.cov[12:15, 12:15] = np.eye(3) * 1e-2

        # Gravity
        self.gravity_mps2 = 9.80665
        self.gravity_world = np.array([0.0, 0.0, -self.gravity_mps2], dtype=np.float64)

        # Timing / previous sample for midpoint propagation
        self.last_timestamp_sec = None
        self.prev_imu_sample = None

        # Accelerometer update settings
        self.accel_update_min_mps2 = 8.0
        self.accel_update_max_mps2 = 11.5

        # Process / measurement noise settings from your stationary analysis
        self.gyro_noise_std_rps = 0.00096
        self.accel_noise_std_mps2 = 0.0162
        self.accel_meas_std = 0.0020

        self.gyro_bias_random_walk_std = 4.07e-06
        self.accel_bias_random_walk_std = 1.05e-04

        # Runtime stationary state
        self.is_currently_stationary = False
        self.stationary_candidate_counter = 0
        self.moving_candidate_counter = 0
        self.stationary_start_time_sec = None

        # Jacobian debug
        self.debug_compare_accel_jacobian = False
        self.debug_compare_accel_jacobian_every_n_updates = 200
        self.accel_update_counter = 0

    def predict_covariance(self, gyro_unbiased_rps, accel_unbiased_mps2, dt):
        """
        Propagate the 15x15 error-state covariance through one IMU prediction step.
        """
        rotation_world_from_body = quat_to_rotmat(self.quat)

        F = np.zeros((15, 15), dtype=np.float64)

        # dpos_dot = dvel
        F[0:3, 3:6] = np.eye(3, dtype=np.float64)

        # dvel_dot = -R * skew(accel_body) * dtheta - R * dbias_acc
        F[3:6, 6:9] = -rotation_world_from_body @ skew_symmetric(accel_unbiased_mps2)
        F[3:6, 12:15] = -rotation_world_from_body

        # dtheta_dot = -skew(gyro_body) * dtheta - dbias_gyro
        F[6:9, 6:9] = -skew_symmetric(gyro_unbiased_rps)
        F[6:9, 9:12] = -np.eye(3, dtype=np.float64)

        # Noise Jacobian G
        # Noise ordering:
        #   0:3   accel white noise
        #   3:6   gyro white noise
        #   6:9   gyro bias random walk
        #   9:12  accel bias random walk
        G = np.zeros((15, 12), dtype=np.float64)
        G[3:6, 0:3] = rotation_world_from_body
        G[6:9, 3:6] = -np.eye(3, dtype=np.float64)
        G[9:12, 6:9] = np.eye(3, dtype=np.float64)
        G[12:15, 9:12] = np.eye(3, dtype=np.float64)

        accel_noise_var = self.accel_noise_std_mps2 ** 2
        gyro_noise_var = self.gyro_noise_std_rps ** 2
        gyro_bias_rw_var = self.gyro_bias_random_walk_std ** 2
        accel_bias_rw_var = self.accel_bias_random_walk_std ** 2

        Qc = np.zeros((12, 12), dtype=np.float64)
        Qc[0:3, 0:3] = np.eye(3, dtype=np.float64) * accel_noise_var
        Qc[3:6, 3:6] = np.eye(3, dtype=np.float64) * gyro_noise_var
        Qc[6:9, 6:9] = np.eye(3, dtype=np.float64) * gyro_bias_rw_var
        Qc[9:12, 9:12] = np.eye(3, dtype=np.float64) * accel_bias_rw_var

        identity = np.eye(15, dtype=np.float64)
        Phi = identity + F * dt
        Qd = G @ Qc @ G.T * dt

        self.cov = Phi @ self.cov @ Phi.T + Qd
        self.cov = 0.5 * (self.cov + self.cov.T)

    def inject_error_state(self, error_state):
        """Inject a 15D error-state correction into the nominal state."""
        dpos = error_state[0:3]
        dvel = error_state[3:6]
        dtheta = error_state[6:9]
        dbias_gyro = error_state[9:12]
        dbias_acc = error_state[12:15]

        self.pos = self.pos + dpos
        self.vel = self.vel + dvel
        self.quat = apply_small_angle_quat_correction(self.quat, dtheta)
        self.bias_gyro = self.bias_gyro + dbias_gyro
        self.bias_acc = self.bias_acc + dbias_acc

    def update_runtime_stationary_detector(self, imu_sample):
        """
        Runtime stationary detector with hysteresis.

        Uses raw measured gyro norm and accel norm.
        """
        cfg = self.config

        if not cfg.runtime_stationary_use:
            self.is_currently_stationary = False
            self.stationary_candidate_counter = 0
            self.moving_candidate_counter = 0
            self.stationary_start_time_sec = None
            return

        gyro_norm = np.linalg.norm(imu_sample["gyro_rps"])
        accel_norm = np.linalg.norm(imu_sample["accel_mps2"])

        stationary_candidate = (
            gyro_norm <= cfg.runtime_still_gyro_max_rps
            and cfg.runtime_still_accel_norm_min_mps2 <= accel_norm <= cfg.runtime_still_accel_norm_max_mps2
        )

        now_sec = imu_sample["timestamp_sec"]

        if stationary_candidate:
            self.stationary_candidate_counter += 1
            self.moving_candidate_counter = 0
        else:
            self.moving_candidate_counter += 1
            self.stationary_candidate_counter = 0

        if (not self.is_currently_stationary) and (
            self.stationary_candidate_counter >= cfg.runtime_stationary_enter_count
        ):
            self.is_currently_stationary = True
            self.stationary_start_time_sec = now_sec

        if self.is_currently_stationary and (
            self.moving_candidate_counter >= cfg.runtime_stationary_exit_count
        ):
            self.is_currently_stationary = False
            self.stationary_start_time_sec = None

    def stationary_duration_sec(self, imu_sample):
        """Return how long the filter has currently been in stationary mode."""
        if (not self.is_currently_stationary) or (self.stationary_start_time_sec is None):
            return 0.0
        return max(0.0, imu_sample["timestamp_sec"] - self.stationary_start_time_sec)

    def update_from_zupt(self):
        """
        Zero-velocity update:
            measured velocity = 0
        Applied only during confirmed stationary periods.
        """
        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 3:6] = np.eye(3, dtype=np.float64)

        z = np.zeros(3, dtype=np.float64)
        innovation = z - self.vel

        R_meas = np.eye(3, dtype=np.float64) * (self.config.zupt_vel_meas_std_mps ** 2)

        S = H @ self.cov @ H.T + R_meas
        K = self.cov @ H.T @ np.linalg.inv(S)

        error_state = K @ innovation

        # Keep ZUPT conservative in this version:
        # - allow velocity correction
        # - allow attitude / gyro-bias indirect correction through correlations
        # - do not directly adapt accelerometer bias here
        if not self.config.enable_runtime_accel_bias_updates:
            error_state[12:15] = 0.0

        identity = np.eye(15, dtype=np.float64)
        temp = identity - K @ H
        self.cov = temp @ self.cov @ temp.T + K @ R_meas @ K.T
        self.cov = 0.5 * (self.cov + self.cov.T)

        self.inject_error_state(error_state)

    def refine_gyro_bias_when_stationary(self, imu_sample):
        """
        During strong stationary periods:
            gyro_measured = bias_gyro + noise
        so this acts like a direct measurement on gyro bias.
        """
        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 9:12] = np.eye(3, dtype=np.float64)

        z = imu_sample["gyro_rps"]
        innovation = z - self.bias_gyro

        R_meas = np.eye(3, dtype=np.float64) * (
            self.config.runtime_gyro_bias_meas_std_rps ** 2
        )

        S = H @ self.cov @ H.T + R_meas
        K = self.cov @ H.T @ np.linalg.inv(S)

        error_state = K @ innovation

        # Keep this as a direct gyro-bias refinement only
        error_state[0:9] = 0.0
        error_state[12:15] = 0.0

        identity = np.eye(15, dtype=np.float64)
        temp = identity - K @ H
        self.cov = temp @ self.cov @ temp.T + K @ R_meas @ K.T
        self.cov = 0.5 * (self.cov + self.cov.T)

        self.inject_error_state(error_state)

    def update_from_accel(self, imu_sample):
        """
        Tilt correction from accelerometer gravity direction.

        Important:
        - corrects roll/pitch
        - does not directly observe yaw
        - does not directly update position/velocity
        - does not directly update accelerometer bias
        """
        accel_mps2 = imu_sample["accel_mps2"] - self.bias_acc
        accel_norm = np.linalg.norm(accel_mps2)
        if accel_norm < 1e-12:
            return

        if self.config.accel_update_only_when_near_gravity:
            if not (self.accel_update_min_mps2 <= accel_norm <= self.accel_update_max_mps2):
                return

        measured_dir_body = accel_mps2 / accel_norm
        expected_dir_body = compute_expected_gravity_dir_body_from_quat(
            self.quat,
            self.gravity_world
        )

        innovation = measured_dir_body - expected_dir_body

        if np.linalg.norm(innovation) > self.config.accel_update_max_innovation_norm:
            return

        # Optional debug: compare analytic vs numerical attitude Jacobian
        self.accel_update_counter += 1
        if (
            self.debug_compare_accel_jacobian
            and self.accel_update_counter % self.debug_compare_accel_jacobian_every_n_updates == 0
        ):
            H_theta_num = compute_accel_measurement_jacobian_numerical(
                self.quat, self.gravity_world, eps=1e-6
            )
            H_theta_analytic = compute_accel_measurement_jacobian_analytic(
                self.quat, self.gravity_world
            )
            H_diff = H_theta_num - H_theta_analytic
            print("\n[Jacobian check] accel gravity-direction Jacobian")
            print("H_theta_numerical:")
            print(H_theta_num)
            print("H_theta_analytic:")
            print(H_theta_analytic)
            print("difference (numerical - analytic):")
            print(H_diff)
            print("max abs diff:", np.max(np.abs(H_diff)))
            print("frobenius norm diff:", np.linalg.norm(H_diff))
            print("-" * 80)

        # First-order small-angle linearization of gravity-direction measurement.
        # This is the standard EKF / error-state EKF approximation.
        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 6:9] = skew_symmetric(expected_dir_body)

        R_meas = np.eye(3, dtype=np.float64) * (self.accel_meas_std ** 2)

        S = H @ self.cov @ H.T + R_meas
        K = self.cov @ H.T @ np.linalg.inv(S)

        # Observability-consistent projection:
        # remove unobservable component around gravity axis.
        gravity_axis_body = expected_dir_body / np.linalg.norm(expected_dir_body)
        tilt_projector = np.eye(3, dtype=np.float64) - np.outer(
            gravity_axis_body, gravity_axis_body
        )

        correction_projector = np.zeros((15, 15), dtype=np.float64)
        correction_projector[6:9, 6:9] = tilt_projector
        correction_projector[9:12, 9:12] = tilt_projector

        K_eff = correction_projector @ K
        error_state = K_eff @ innovation

        # Do not let gravity-direction update directly modify these states
        error_state[0:3] = 0.0
        error_state[3:6] = 0.0
        error_state[12:15] = 0.0

        identity = np.eye(15, dtype=np.float64)
        temp = identity - K_eff @ H
        self.cov = temp @ self.cov @ temp.T + K_eff @ R_meas @ K_eff.T
        self.cov = 0.5 * (self.cov + self.cov.T)

        self.inject_error_state(error_state)

    def predict_from_imu(self, imu_sample):
        """
        Midpoint IMU propagation + runtime stationary logic + measurement updates.
        """
        timestamp_sec = imu_sample["timestamp_sec"]

        if self.last_timestamp_sec is None:
            self.last_timestamp_sec = timestamp_sec
            self.prev_imu_sample = imu_sample
            self.update_runtime_stationary_detector(imu_sample)
            return

        dt = timestamp_sec - self.last_timestamp_sec
        self.last_timestamp_sec = timestamp_sec

        if dt <= 0.0 or dt > 0.1:
            self.prev_imu_sample = imu_sample
            self.update_runtime_stationary_detector(imu_sample)
            return

        # Midpoint IMU values
        if self.prev_imu_sample is None:
            gyro_mid_rps = imu_sample["gyro_rps"]
            accel_mid_mps2 = imu_sample["accel_mps2"]
        else:
            gyro_mid_rps = 0.5 * (self.prev_imu_sample["gyro_rps"] + imu_sample["gyro_rps"])
            accel_mid_mps2 = 0.5 * (self.prev_imu_sample["accel_mps2"] + imu_sample["accel_mps2"])

        # Bias-correct midpoint signals
        gyro_unbiased_rps = gyro_mid_rps - self.bias_gyro
        accel_unbiased_mps2 = accel_mid_mps2 - self.bias_acc

        # Orientation propagation
        delta_theta = gyro_unbiased_rps * dt
        delta_quat = quat_from_small_angle(delta_theta)
        self.quat = quat_normalize(quat_mul(self.quat, delta_quat))

        # Velocity / position propagation
        rotation_world_from_body = quat_to_rotmat(self.quat)
        accel_world_mps2 = rotation_world_from_body @ accel_unbiased_mps2
        linear_accel_world_mps2 = accel_world_mps2 + self.gravity_world

        old_vel = self.vel.copy()
        self.vel = self.vel + linear_accel_world_mps2 * dt
        self.pos = self.pos + 0.5 * (old_vel + self.vel) * dt

        # Covariance propagation
        self.predict_covariance(
            gyro_unbiased_rps=gyro_unbiased_rps,
            accel_unbiased_mps2=accel_unbiased_mps2,
            dt=dt
        )

        # Runtime stationary detector
        self.update_runtime_stationary_detector(imu_sample)

        # Accelerometer tilt update
        if self.config.accel_update_use:
            self.update_from_accel(imu_sample)

        # Stationary updates
        if self.is_currently_stationary:
            still_time = self.stationary_duration_sec(imu_sample)

            if still_time >= self.config.runtime_zupt_min_stationary_time_sec:
                self.update_from_zupt()

            if still_time >= self.config.runtime_gyro_bias_refine_min_stationary_time_sec:
                self.refine_gyro_bias_when_stationary(imu_sample)

        self.prev_imu_sample = imu_sample


# ============================================================
# IMU register definitions
# ============================================================

MPU9250_I2C_ADDR = 0x68

MPU_PWR_MGMT_1 = 0x6B
MPU_SMPLRT_DIV = 0x19
MPU_CONFIG = 0x1A
MPU_GYRO_CONFIG = 0x1B
MPU_ACCEL_CONFIG = 0x1C
MPU_ACCEL_CONFIG2 = 0x1D
MPU_ACCEL_XOUT_H = 0x3B


def int16_from_bytes(msb, lsb):
    """Combine two 8-bit bytes into one signed 16-bit integer."""
    value = (msb << 8) | lsb
    if value & 0x8000:
        value -= 65536
    return value


def compute_startup_imu_metrics(startup_samples):
    """Compute simple sanity metrics from startup IMU samples."""
    gyro_array = np.array([sample["gyro_rps"] for sample in startup_samples], dtype=np.float64)
    accel_array = np.array([sample["accel_mps2"] for sample in startup_samples], dtype=np.float64)

    gyro_norm_array = np.linalg.norm(gyro_array, axis=1)
    accel_norm_array = np.linalg.norm(accel_array, axis=1)

    return {
        "gyro_mean_rps": np.mean(gyro_array, axis=0),
        "accel_mean_mps2": np.mean(accel_array, axis=0),
        "gyro_norm_mean_rps": float(np.mean(gyro_norm_array)),
        "accel_norm_mean_mps2": float(np.mean(accel_norm_array)),
        "accel_norm_std_mps2": float(np.std(accel_norm_array)),
    }


def is_imu_stationary(metrics, config):
    """Decide whether startup data looks stationary enough for calibration."""
    gyro_ok = metrics["gyro_norm_mean_rps"] <= config.still_gyro_max_rps
    accel_mean_ok = (
        config.still_accel_norm_min_mps2
        <= metrics["accel_norm_mean_mps2"]
        <= config.still_accel_norm_max_mps2
    )
    accel_std_ok = metrics["accel_norm_std_mps2"] <= config.still_accel_std_max_mps2
    return gyro_ok and accel_mean_ok and accel_std_ok


def initialize_ekf_from_startup(ekf, config, startup_samples, startup_metrics):
    """Initialize EKF state from startup mode and startup IMU data."""
    init_report = {
        "mode_requested": config.startup_mode,
        "mode_used": None,
        "stationary_detected": None,
    }

    ekf.pos = np.array(config.initial_pos, dtype=np.float64)

    if config.startup_mode == "instant":
        init_report["mode_used"] = "instant"
        init_report["stationary_detected"] = None

        if config.use_user_initial_pose:
            roll_rad = math.radians(config.initial_rpy_deg[0])
            pitch_rad = math.radians(config.initial_rpy_deg[1])
            yaw_rad = math.radians(config.initial_rpy_deg[2])
            ekf.quat = rpy_to_quat(roll_rad, pitch_rad, yaw_rad)
        else:
            ekf.quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

        ekf.vel = np.zeros(3, dtype=np.float64)
        ekf.bias_gyro = np.zeros(3, dtype=np.float64)
        ekf.bias_acc = np.zeros(3, dtype=np.float64)
        return init_report

    if config.startup_mode == "stationary_calibration":
        stationary_detected = is_imu_stationary(startup_metrics, config)
        init_report["stationary_detected"] = stationary_detected

        if stationary_detected:
            init_report["mode_used"] = "stationary_calibration"

            ekf.bias_gyro = np.array(startup_metrics["gyro_mean_rps"], dtype=np.float64)

            # Update gravity magnitude from measured stationary startup norm
            ekf.gravity_mps2 = float(startup_metrics["accel_norm_mean_mps2"])
            ekf.gravity_world = np.array([0.0, 0.0, -ekf.gravity_mps2], dtype=np.float64)

            if config.use_user_initial_pose:
                yaw_rad = math.radians(config.initial_rpy_deg[2])
            else:
                yaw_rad = 0.0

            ekf.quat = quat_from_accel_gravity(
                startup_metrics["accel_mean_mps2"],
                yaw_rad=yaw_rad
            )

            ekf.vel = np.zeros(3, dtype=np.float64)
            ekf.bias_acc = np.zeros(3, dtype=np.float64)
            return init_report

        init_report["mode_used"] = "instant_fallback"

        if config.use_user_initial_pose:
            roll_rad = math.radians(config.initial_rpy_deg[0])
            pitch_rad = math.radians(config.initial_rpy_deg[1])
            yaw_rad = math.radians(config.initial_rpy_deg[2])
            ekf.quat = rpy_to_quat(roll_rad, pitch_rad, yaw_rad)
        else:
            ekf.quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

        ekf.vel = np.zeros(3, dtype=np.float64)
        ekf.bias_gyro = np.zeros(3, dtype=np.float64)
        ekf.bias_acc = np.zeros(3, dtype=np.float64)
        return init_report

    raise ValueError(f"Unknown startup_mode: {config.startup_mode}")


# ============================================================
# IMU reader
# ============================================================

class Mpu9250Reader:
    """
    Minimal MPU-6500/compatible reader for accelerometer, gyroscope, and temperature.

    Output units:
    - acceleration: m/s^2
    - angular velocity: rad/s
    - temperature: deg C
    """

    def __init__(self, i2c_bus, i2c_addr=MPU9250_I2C_ADDR):
        self.i2c_bus = i2c_bus
        self.i2c_addr = i2c_addr

        # Axis remapping from sensor frame to body frame.
        self.accel_remap_matrix = np.eye(3, dtype=np.float64)
        self.gyro_remap_matrix = np.eye(3, dtype=np.float64)

        # Accelerometer calibration from six-pose static test
        self.accel_bias_vec_mps2 = np.array(
            [0.060705097, -0.080021347, 0.136866529],
            dtype=np.float64
        )

        self.accel_scale_vec = np.array(
            [0.995836574, 0.996978119, 1.006539360],
            dtype=np.float64
        )

    def write_u8(self, register_addr, value):
        self.i2c_bus.write_byte_data(self.i2c_addr, register_addr, value)

    def read_u8(self, register_addr):
        return self.i2c_bus.read_byte_data(self.i2c_addr, register_addr)

    def check_connection(self):
        """
        Read WHO_AM_I from the IMU.

        On your board this returned 0x70, consistent with MPU-6500 / compatible.
        """
        MPU_WHO_AM_I = 0x75
        return self.read_u8(MPU_WHO_AM_I)

    def read_block(self, register_addr, length):
        return self.i2c_bus.read_i2c_block_data(self.i2c_addr, register_addr, length)

    def initialize(self):
        """
        Initialize the IMU.

        Configuration:
        - wake up sensor
        - sample rate divider = 4
        - DLPF enabled
        - gyro range = ±250 deg/s
        - accel range = ±2 g
        """
        self.write_u8(MPU_PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        self.write_u8(MPU_SMPLRT_DIV, 0x04)
        self.write_u8(MPU_CONFIG, 0x03)
        self.write_u8(MPU_GYRO_CONFIG, 0x00)
        self.write_u8(MPU_ACCEL_CONFIG, 0x00)
        self.write_u8(MPU_ACCEL_CONFIG2, 0x03)

        time.sleep(0.05)

    def read_sample(self):
        """Read one IMU sample and convert it to physical units."""
        timestamp_sec = time.time()
        data = self.read_block(MPU_ACCEL_XOUT_H, 14)

        raw_accel_x = int16_from_bytes(data[0], data[1])
        raw_accel_y = int16_from_bytes(data[2], data[3])
        raw_accel_z = int16_from_bytes(data[4], data[5])

        raw_temp = int16_from_bytes(data[6], data[7])

        raw_gyro_x = int16_from_bytes(data[8], data[9])
        raw_gyro_y = int16_from_bytes(data[10], data[11])
        raw_gyro_z = int16_from_bytes(data[12], data[13])

        accel_x_g = raw_accel_x / 16384.0
        accel_y_g = raw_accel_y / 16384.0
        accel_z_g = raw_accel_z / 16384.0

        # Standard gravity for raw unit conversion from g to m/s^2.
        g0 = 9.80665

        accel_mps2_raw = np.array([
            accel_x_g * g0,
            accel_y_g * g0,
            accel_z_g * g0
        ], dtype=np.float64)

        # Apply accelerometer bias and per-axis scale calibration
        accel_mps2 = (accel_mps2_raw - self.accel_bias_vec_mps2) / self.accel_scale_vec

        gyro_x_dps = raw_gyro_x / 131.0
        gyro_y_dps = raw_gyro_y / 131.0
        gyro_z_dps = raw_gyro_z / 131.0

        deg_to_rad = math.pi / 180.0
        gyro_rps = np.array([
            gyro_x_dps * deg_to_rad,
            gyro_y_dps * deg_to_rad,
            gyro_z_dps * deg_to_rad
        ], dtype=np.float64)

        temperature_c = (raw_temp / 333.87) + 21.0

        # Apply axis remapping to convert from sensor frame into body frame.
        accel_mps2 = apply_axis_remap(accel_mps2, self.accel_remap_matrix)
        gyro_rps = apply_axis_remap(gyro_rps, self.gyro_remap_matrix)

        return {
            "timestamp_sec": timestamp_sec,
            "accel_mps2": accel_mps2,
            "accel_mps2_raw": accel_mps2_raw,
            "gyro_rps": gyro_rps,
            "temperature_c": temperature_c,
            "raw_accel": np.array([raw_accel_x, raw_accel_y, raw_accel_z], dtype=np.int32),
            "raw_gyro": np.array([raw_gyro_x, raw_gyro_y, raw_gyro_z], dtype=np.int32),
        }


# ============================================================
# Main
# ============================================================

def main():
    print("Single-IMU EKF prototype started")

    config = ImuEkfConfig()
    config.startup_mode = "stationary_calibration"

    ekf = ImuEkfState(config)

    print("\nStartup configuration:")
    print("startup_mode:", config.startup_mode)
    print("use_user_initial_pose:", config.use_user_initial_pose)
    print("initial_pos:", config.initial_pos)
    print("initial_rpy_deg:", config.initial_rpy_deg)
    print("use_magnetometer:", config.use_magnetometer)
    print("use_mag_calibration:", config.use_mag_calibration)

    bus_num = 7

    startup_samples = []
    startup_metrics = None

    with SMBus(bus_num) as i2c_bus:
        imu_reader = Mpu9250Reader(i2c_bus)

        try:
            who_am_i = imu_reader.check_connection()
            print(f"\nMPU WHO_AM_I: 0x{who_am_i:02X}")
        except OSError:
            print("Failed to communicate with IMU before initialization.")
            print("Check power, wiring, and run: sudo i2cdetect -y -r 7")
            raise

        imu_reader.initialize()

        # Startup initialization
        if config.startup_mode == "stationary_calibration":
            print("\nCollecting startup samples for stationary calibration...")
            for _ in range(config.startup_sample_count):
                sample = imu_reader.read_sample()
                startup_samples.append(sample)
                time.sleep(config.startup_sample_dt_sec)

            startup_metrics = compute_startup_imu_metrics(startup_samples)

            print("\nStartup sanity metrics:")
            print("gyro_mean_rps:", startup_metrics["gyro_mean_rps"])
            print("accel_mean_mps2:", startup_metrics["accel_mean_mps2"])
            print("gyro_norm_mean_rps:", startup_metrics["gyro_norm_mean_rps"])
            print("accel_norm_mean_mps2:", startup_metrics["accel_norm_mean_mps2"])
            print("accel_norm_std_mps2:", startup_metrics["accel_norm_std_mps2"])
            print("stationary_detected:", is_imu_stationary(startup_metrics, config))
        else:
            startup_metrics = {
                "gyro_mean_rps": np.zeros(3, dtype=np.float64),
                "accel_mean_mps2": np.array([0.0, 0.0, ekf.gravity_mps2], dtype=np.float64),
                "gyro_norm_mean_rps": 0.0,
                "accel_norm_mean_mps2": ekf.gravity_mps2,
                "accel_norm_std_mps2": 0.0,
            }

        init_report = initialize_ekf_from_startup(
            ekf=ekf,
            config=config,
            startup_samples=startup_samples,
            startup_metrics=startup_metrics
        )

        print("\nInitialization report:")
        print("mode_requested:", init_report["mode_requested"])
        print("mode_used:", init_report["mode_used"])
        print("stationary_detected:", init_report["stationary_detected"])

        init_roll, init_pitch, init_yaw = quat_to_rpy(ekf.quat)

        print("\nInitialized EKF state:")
        print("pos:", ekf.pos)
        print("vel:", ekf.vel)
        print("quat:", ekf.quat)
        print("bias_gyro:", ekf.bias_gyro)
        print("bias_acc:", ekf.bias_acc)
        print("initial roll, pitch, yaw [rad]:", (init_roll, init_pitch, init_yaw))

        print("\nEntering live EKF loop. Press Ctrl+C to stop.\n")

        last_print_time = time.time()

        try:
            while True:
                imu_sample = imu_reader.read_sample()
                ekf.predict_from_imu(imu_sample)

                now = time.time()
                if (now - last_print_time) >= config.print_interval_sec:
                    roll_rad, pitch_rad, yaw_rad = quat_to_rpy(ekf.quat)

                    roll_deg = math.degrees(roll_rad)
                    pitch_deg = math.degrees(pitch_rad)
                    yaw_deg = math.degrees(yaw_rad)

                    accel_norm = np.linalg.norm(imu_sample["accel_mps2"])
                    gyro_norm = np.linalg.norm(imu_sample["gyro_rps"])

                    att_cov_diag = np.diag(ekf.cov)[6:9]
                    gyro_bias_cov_diag = np.diag(ekf.cov)[9:12]
                    accel_bias_cov_diag = np.diag(ekf.cov)[12:15]

                    print(
                        f"RPY deg: "
                        f"{roll_deg:+7.2f} {pitch_deg:+7.2f} {yaw_deg:+7.2f} | "
                        f"gyro_norm rps: {gyro_norm:8.4f} | "
                        f"accel_norm mps2: {accel_norm:8.4f}"
                    )
                    print(
                        f"pos cm: {ekf.pos * 100.0} | "
                        f"vel mps: {ekf.vel}"
                    )
                    print(
                        f"bias_gyro rps: {ekf.bias_gyro} | "
                        f"bias_acc mps2: {ekf.bias_acc} | "
                        f"stationary: {ekf.is_currently_stationary}"
                    )
                    print(
                        f"att_cov diag: {att_cov_diag} | "
                        f"gyro_bias_cov diag: {gyro_bias_cov_diag} | "
                        f"acc_bias_cov diag: {accel_bias_cov_diag}"
                    )
                    print("-" * 120)

                    last_print_time = now

        except KeyboardInterrupt:
            print("\nStopped by user.")

    print("\nSingle-IMU EKF prototype finished.")


if __name__ == "__main__":
    main()
