#!/usr/bin/env python3
"""
Single-IMU EKF prototype (plain Python, direct I2C read)

Purpose
-------
This script estimates the IMU pose relative to its start pose using:
- gyroscope
- accelerometer
- optional magnetometer later

Current stage
-------------
This version includes:
- nominal state propagation from IMU
- 15-state error-state covariance propagation
- accelerometer gravity-direction update
- startup initialization
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
from collections import deque


# ============================================================
# Quaternion and math helper functions
# ============================================================

def quat_normalize(q):
    """
    Normalize quaternion q = [w, x, y, z].

    Why:
    Numerical integration and floating-point operations can cause the
    quaternion to slowly stop having unit length. A valid rotation
    quaternion should always have norm 1.
    """
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n


def quat_mul(q1, q2):
    """
    Hamilton product of two quaternions.

    Both quaternions use the format:
        q = [w, x, y, z]

    This is used to update orientation:
        q_new = q_old * dq
    where dq is a small incremental rotation from gyro data.
    """
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

    Interpretation:
    - direction of dtheta = rotation axis
    - magnitude of dtheta = rotation angle in radians

    This is used for:
    - gyro integration
    - small EKF correction updates
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

    Interpretation:
    - This returns rotation_world_from_body.
    - It maps a vector from body frame into world frame.

    We use this for:
    - rotating acceleration from body frame to world frame
    - computing expected gravity direction in body frame
    """
    q = quat_normalize(q)
    w, x, y, z = q

    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def quat_to_rpy(q):
    """
    Convert quaternion q = [w, x, y, z] to roll, pitch, yaw in radians.

    This is only for debugging / printing because roll-pitch-yaw is easier
    for humans to understand than quaternions.
    """
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
    """
    Convert roll, pitch, yaw [rad] into quaternion [w, x, y, z].
    """
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
    Estimate an initial orientation quaternion from the averaged accelerometer.

    Purpose
    -------
    When the IMU is stationary, the accelerometer mostly measures gravity
    (actually specific force opposite gravity). We use that to estimate
    initial roll and pitch.

    Yaw cannot be determined from accelerometer alone, so yaw_rad is supplied
    separately and defaults to 0.

    Convention
    ----------
    This function assumes:
    - world frame has +Z up
    - a stationary level IMU typically measures approximately [0, 0, +g]
      in its body frame

    Returns
    -------
    Quaternion [w, x, y, z]
    """
    ax, ay, az = accel_mean_mps2

    roll_rad = math.atan2(ay, az)
    pitch_rad = math.atan2(-ax, math.sqrt(ay * ay + az * az))

    return rpy_to_quat(roll_rad, pitch_rad, yaw_rad)


def apply_small_angle_quat_correction(quat, dtheta):
    """
    Apply a small-angle correction dtheta to an existing quaternion.

    Inputs:
        quat   : current quaternion [w, x, y, z]
        dtheta : small rotation vector [rad]

    Returns:
        corrected normalized quaternion
    """
    delta_quat = quat_from_small_angle(dtheta)
    return quat_normalize(quat_mul(quat, delta_quat))



def skew_symmetric(vec3):
    """
    Return the 3x3 skew-symmetric matrix of a 3D vector.

    For vec3 = [x, y, z], this returns:

        [  0  -z   y ]
        [  z   0  -x ]
        [ -y   x   0 ]
    """
    x, y, z = vec3
    return np.array([
        [0.0, -z,  y],
        [z,   0.0, -x],
        [-y,  x,   0.0]
    ], dtype=np.float64)

def compute_expected_gravity_dir_body_from_quat(quat, gravity_world):
    """
    Compute expected normalized gravity direction in the body frame
    from the current quaternion and world gravity vector.
    """
    rotation_world_from_body = quat_to_rotmat(quat)
    rotation_body_from_world = rotation_world_from_body.T

    gravity_dir_world = -gravity_world / np.linalg.norm(gravity_world)
    expected_dir_body = rotation_body_from_world @ gravity_dir_world
    expected_dir_body = expected_dir_body / np.linalg.norm(expected_dir_body)

    return expected_dir_body


def compute_accel_measurement_jacobian_numerical(quat, gravity_world, eps=1e-6):
    """
    Numerical Jacobian of expected gravity direction in body frame
    with respect to the 3D small-angle attitude error dtheta.

    Returns:
        H_theta_num shape (3, 3)
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
    with respect to the 3D small-angle attitude error dtheta.

    This matches the current implementation convention:
        q <- q * dq
    """
    expected_dir_body = compute_expected_gravity_dir_body_from_quat(quat, gravity_world)
    H_theta_analytic = skew_symmetric(expected_dir_body)
    return H_theta_analytic

def apply_axis_remap(raw_vec, remap_matrix):
    """
    Convert a 3D vector from sensor frame into chosen body frame.

    remap_matrix must be a 3x3 matrix containing only:
    - 0
    - +1
    - -1

    with exactly one nonzero element per row and column.
    """
    return remap_matrix @ raw_vec


# ============================================================
# EKF configuration
# ============================================================

class ImuEkfConfig:
    """
    Configuration values for startup behavior, thresholds, and optional
    future sensor calibration settings.
    """

    def __init__(self):
        # Startup mode:
        # "instant" or "stationary_calibration"
        self.startup_mode = "instant"

        # Optional user-defined initial pose
        self.use_user_initial_pose = False
        self.initial_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.initial_rpy_deg = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # Startup stationary calibration settings
        self.startup_sample_count = 100
        self.startup_sample_dt_sec = 0.01

        # Stillness thresholds
        self.still_gyro_max_rps = 0.08
        self.still_accel_norm_min_mps2 = 9.3
        self.still_accel_norm_max_mps2 = 10.4
        self.still_accel_std_max_mps2 = 0.25

        # Magnetometer placeholders for later
        self.use_magnetometer = False
        self.use_mag_calibration = False
        self.mag_bias = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.mag_scale_matrix = np.eye(3, dtype=np.float64)

        # Logging behavior
        self.print_interval_sec = 0.20

        # --------------------------------------------------------
        # Runtime stationary detection settings
        # --------------------------------------------------------
        # Window length for online stationary detection
        self.runtime_stationary_window_size = 40

        # Thresholds for deciding if the IMU is currently stationary
        self.runtime_stationary_gyro_norm_max_rps = 0.06
        self.runtime_stationary_accel_norm_error_max_mps2 = 0.20
        self.runtime_stationary_accel_std_max_mps2 = 0.05
        self.runtime_stationary_gyro_std_max_rps = 0.01

        # --------------------------------------------------------
        # Zero-velocity update (ZUPT)
        # --------------------------------------------------------
        self.use_zupt = True

        # Velocity measurement standard deviation [m/s]
        # Smaller means stronger pull toward zero during stationary periods.
        self.zupt_velocity_meas_std_mps = 0.02

        # --------------------------------------------------------
        # Stationary gyro-bias refinement
        # --------------------------------------------------------
        self.use_stationary_bias_refinement = True

        # Small blending factor for slow bias adaptation during stationary periods
        self.stationary_bias_blend = 0.01


# ============================================================
# EKF state
# ============================================================

class ImuEkfState:
    """
    Holds the nominal EKF state and its error covariance.

    Nominal state:
        pos        = position in world frame              shape (3,)
        vel        = velocity in world frame              shape (3,)
        quat       = orientation quaternion [w, x, y, z] shape (4,)
        bias_gyro  = gyroscope bias                       shape (3,)
        bias_acc   = accelerometer bias                   shape (3,)

    Error-state covariance matrix:
        cov   shape (15, 15)

    Error-state ordering:
        [dpos, dvel, dtheta, dbias_gyro, dbias_acc]
    """

    def __init__(self, config):
        # Store config for use in methods that need thresholds, etc.
        self.config = config

        #
        self.prev_gyro_unbiased_rps = None
        self.prev_accel_unbiased_mps2 = None
        
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
        self.gravity_mps2 = 9.80665 #9.82 # m/s^2
        self.gravity_world = np.array([0.0, 0.0, -self.gravity_mps2], dtype=np.float64)

        # Timing
        self.last_timestamp_sec = None

        # Accelerometer update settings
        self.accel_update_min_mps2 = 8.0
        self.accel_update_max_mps2 = 11.5

        # IMU process noise settings
            # These values are based on mesurementes from 60 min. test logged with imu_stationary_logger.py
            # and analysed with analyze_imu_noise_stats.py
        self.gyro_noise_std_rps = 0.00096
        self.accel_noise_std_mps2 = 0.0162
        self.accel_meas_std = 0.0020 #0.00110 is plausible from the stationary data, but with current filter structure it may still be too aggressive

        # These values are based on output from interpret_imu_allan_for_ekf.py which is based on the same 60 min. test as above 
        # and uses the output from analyze_imu_allan.py.
        self.gyro_bias_random_walk_std = 4.07e-06
        self.accel_bias_random_walk_std = 1.05e-04

        # --------------------------------------------------------
        # Runtime stationary detection buffers
        # --------------------------------------------------------
        self.stationary_accel_buffer = deque(
            maxlen=self.config.runtime_stationary_window_size
        )
        self.stationary_gyro_buffer = deque(
            maxlen=self.config.runtime_stationary_window_size
        )

        self.is_currently_stationary = False

        # Debug option:
        # If True, compare analytic vs numerical accel Jacobian occasionally.
        self.debug_compare_accel_jacobian = False
        self.debug_compare_accel_jacobian_every_n_updates = 200
        self.accel_update_counter = 0

    def predict_covariance(self, gyro_unbiased_rps, accel_unbiased_mps2, dt):
        """
        Propagate the 15x15 error-state covariance through one IMU prediction step.

        Error-state ordering:
            0:3   dpos
            3:6   dvel
            6:9   dtheta
            9:12  dbias_gyro
            12:15 dbias_acc
        """
        rotation_world_from_body = quat_to_rotmat(self.quat)

        F = np.zeros((15, 15), dtype=np.float64)

        # dpos_dot = dvel
        F[0:3, 3:6] = np.eye(3, dtype=np.float64)

        # dvel_dot = -R * skew(accel_body) * dtheta - R * dbias_acc + R * accel_noise
        F[3:6, 6:9] = -rotation_world_from_body @ skew_symmetric(accel_unbiased_mps2)
        F[3:6, 12:15] = -rotation_world_from_body

        # dtheta_dot = -skew(gyro_body) * dtheta - dbias_gyro - gyro_noise
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

    def update_runtime_stationary_detector(self, accel_mps2, gyro_rps):
        """
        Update rolling-window stationary detection using recent IMU samples.

        Logic:
        - gyro norm must be small
        - accel norm must stay close to gravity magnitude
        - accel variation must be small
        - gyro variation must be small
        """
        self.stationary_accel_buffer.append(np.array(accel_mps2, dtype=np.float64))
        self.stationary_gyro_buffer.append(np.array(gyro_rps, dtype=np.float64))

        if len(self.stationary_accel_buffer) < self.config.runtime_stationary_window_size:
            self.is_currently_stationary = False
            return self.is_currently_stationary

        accel_array = np.array(self.stationary_accel_buffer, dtype=np.float64)
        gyro_array = np.array(self.stationary_gyro_buffer, dtype=np.float64)

        accel_norm_array = np.linalg.norm(accel_array, axis=1)
        gyro_norm_array = np.linalg.norm(gyro_array, axis=1)

        accel_norm_mean = float(np.mean(accel_norm_array))
        accel_norm_std = float(np.std(accel_norm_array))
        gyro_norm_mean = float(np.mean(gyro_norm_array))

        gyro_std_vec = np.std(gyro_array, axis=0)
        gyro_std_norm = float(np.linalg.norm(gyro_std_vec))

        accel_norm_error = abs(accel_norm_mean - self.gravity_mps2)

        gyro_ok = gyro_norm_mean <= self.config.runtime_stationary_gyro_norm_max_rps
        accel_mean_ok = accel_norm_error <= self.config.runtime_stationary_accel_norm_error_max_mps2
        accel_std_ok = accel_norm_std <= self.config.runtime_stationary_accel_std_max_mps2
        gyro_std_ok = gyro_std_norm <= self.config.runtime_stationary_gyro_std_max_rps

        self.is_currently_stationary = gyro_ok and accel_mean_ok and accel_std_ok and gyro_std_ok
        return self.is_currently_stationary


    def refine_gyro_bias_if_stationary(self, gyro_rps):
        """
        Slowly refine gyro bias during confidently stationary periods.

        Since true angular rate should be near zero when stationary,
        the measured gyro mainly reflects bias plus noise.
        """
        if not self.config.use_stationary_bias_refinement:
            return

        blend = self.config.stationary_bias_blend
        self.bias_gyro = (1.0 - blend) * self.bias_gyro + blend * gyro_rps


    def update_zero_velocity(self):
        """
        Zero-velocity update (ZUPT).

        When the IMU is stationary, world-frame velocity should be near zero.
        This directly constrains the velocity states and indirectly helps
        reduce drift through state correlations.

        Measurement model:
            z = 0
            h(x) = vel
            innovation = z - vel = -vel
        """
        if not self.config.use_zupt:
            return

        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 3:6] = np.eye(3, dtype=np.float64)

        innovation = -self.vel

        R_meas = np.eye(3, dtype=np.float64) * (
            self.config.zupt_velocity_meas_std_mps ** 2
        )

        S = H @ self.cov @ H.T + R_meas
        K = self.cov @ H.T @ np.linalg.inv(S)

        error_state = K @ innovation

        identity = np.eye(15, dtype=np.float64)
        temp = identity - K @ H
        self.cov = temp @ self.cov @ temp.T + K @ R_meas @ K.T
        self.cov = 0.5 * (self.cov + self.cov.T)

        self.inject_error_state(error_state)


    def predict_from_imu(self, imu_sample):
        """
        Prediction step using one IMU sample.

        Uses midpoint / trapezoidal integration in the nominal state:
        - midpoint gyro for quaternion propagation
        - average of world-frame linear acceleration for vel/pos propagation

        This is a better nominal-state integrator than simple first-order Euler.
        """
        timestamp_sec = imu_sample["timestamp_sec"]
        gyro_rps = imu_sample["gyro_rps"]
        accel_mps2 = imu_sample["accel_mps2"]

        if self.last_timestamp_sec is None:
            self.last_timestamp_sec = timestamp_sec
            self.update_runtime_stationary_detector(accel_mps2, gyro_rps)

            # Initialize previous unbiased signals on first sample
            gyro_unbiased_rps = gyro_rps - self.bias_gyro
            accel_unbiased_mps2 = accel_mps2 - self.bias_acc
            self.prev_gyro_unbiased_rps = gyro_unbiased_rps.copy()
            self.prev_accel_unbiased_mps2 = accel_unbiased_mps2.copy()
            return

        dt = timestamp_sec - self.last_timestamp_sec
        self.last_timestamp_sec = timestamp_sec

        if dt <= 0.0 or dt > 0.1:
            self.update_runtime_stationary_detector(accel_mps2, gyro_rps)

            gyro_unbiased_rps = gyro_rps - self.bias_gyro
            accel_unbiased_mps2 = accel_mps2 - self.bias_acc
            self.prev_gyro_unbiased_rps = gyro_unbiased_rps.copy()
            self.prev_accel_unbiased_mps2 = accel_unbiased_mps2.copy()
            return

        # --------------------------------------------------------
        # Runtime stationary detection
        # --------------------------------------------------------
        stationary_now = self.update_runtime_stationary_detector(accel_mps2, gyro_rps)

        # Optional slow gyro-bias refinement during stationary periods
        if stationary_now:
            self.refine_gyro_bias_if_stationary(gyro_rps)

        # --------------------------------------------------------
        # Current bias-corrected IMU
        # --------------------------------------------------------
        gyro_unbiased_rps = gyro_rps - self.bias_gyro
        accel_unbiased_mps2 = accel_mps2 - self.bias_acc

        # If previous values are not yet initialized, fall back gracefully
        if self.prev_gyro_unbiased_rps is None:
            self.prev_gyro_unbiased_rps = gyro_unbiased_rps.copy()
        if self.prev_accel_unbiased_mps2 is None:
            self.prev_accel_unbiased_mps2 = accel_unbiased_mps2.copy()

        # Save old nominal state for trapezoidal integration
        quat_old = self.quat.copy()
        vel_old = self.vel.copy()
        pos_old = self.pos.copy()

        # --------------------------------------------------------
        # 1) Midpoint gyro integration for orientation
        # --------------------------------------------------------
        gyro_mid_rps = 0.5 * (self.prev_gyro_unbiased_rps + gyro_unbiased_rps)
        delta_theta = gyro_mid_rps * dt
        delta_quat = quat_from_small_angle(delta_theta)
        quat_new = quat_normalize(quat_mul(quat_old, delta_quat))
        self.quat = quat_new

        # --------------------------------------------------------
        # 2) Compute linear acceleration at old and new attitude
        # --------------------------------------------------------
        R_old = quat_to_rotmat(quat_old)
        R_new = quat_to_rotmat(quat_new)

        accel_world_old_mps2 = R_old @ self.prev_accel_unbiased_mps2
        accel_world_new_mps2 = R_new @ accel_unbiased_mps2

        linear_accel_world_old_mps2 = accel_world_old_mps2 + self.gravity_world
        linear_accel_world_new_mps2 = accel_world_new_mps2 + self.gravity_world

        linear_accel_world_mid_mps2 = 0.5 * (
            linear_accel_world_old_mps2 + linear_accel_world_new_mps2
        )

        # --------------------------------------------------------
        # 3) Trapezoidal / midpoint integration for vel and pos
        # --------------------------------------------------------
        vel_new = vel_old + linear_accel_world_mid_mps2 * dt
        pos_new = pos_old + 0.5 * (vel_old + vel_new) * dt

        self.vel = vel_new
        self.pos = pos_new

        # --------------------------------------------------------
        # 4) Covariance propagation
        #
        # Keep covariance model first-order for now, but drive it using
        # midpoint inputs so it is more consistent with nominal propagation.
        # --------------------------------------------------------
        accel_unbiased_mid_mps2 = 0.5 * (
            self.prev_accel_unbiased_mps2 + accel_unbiased_mps2
        )

        self.predict_covariance(
            gyro_unbiased_rps=gyro_mid_rps,
            accel_unbiased_mps2=accel_unbiased_mid_mps2,
            dt=dt
        )

        # --------------------------------------------------------
        # 5) Accelerometer gravity-direction update
        # --------------------------------------------------------
        self.update_from_accel(imu_sample)

        # --------------------------------------------------------
        # 6) Zero-velocity update if stationary
        # --------------------------------------------------------
        if stationary_now:
            self.update_zero_velocity()

        # --------------------------------------------------------
        # 7) Store current unbiased IMU for next midpoint step
        # --------------------------------------------------------
        self.prev_gyro_unbiased_rps = gyro_unbiased_rps.copy()
        self.prev_accel_unbiased_mps2 = accel_unbiased_mps2.copy()

    def inject_error_state(self, error_state):
        """
        Inject a 15D error-state correction into the nominal state.

        Error-state ordering:
            0:3   -> dpos
            3:6   -> dvel
            6:9   -> dtheta
            9:12  -> dbias_gyro
            12:15 -> dbias_acc
        """
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

    def update_from_accel(self, imu_sample):
        """
        Correct orientation using the accelerometer as a gravity-direction measurement.

        This update is only valid when the measured acceleration magnitude is
        reasonably close to 1 g, meaning the sensor is not undergoing large
        linear acceleration.

        What it corrects well:
        - roll
        - pitch

        What it does NOT directly correct:
        - yaw
        - position
        - velocity
        - accelerometer bias

        Notes
        -----
        We use an observability-consistent update:
        - gravity direction only constrains tilt
        - the correction component around the gravity axis is projected out
        - position, velocity, and accelerometer bias are not directly corrected
        by this gravity-direction measurement

        Jacobian note
        -------------
        The attitude Jacobian below is a first-order small-angle linearization.
        This is the standard choice in EKF / error-state EKF implementations.
        For the right-multiplicative quaternion error convention used here
        (q <- q * dq), the correct sign for the rotated body-frame gravity
        vector is +skew(expected_dir_body).
        """
        accel_mps2 = imu_sample["accel_mps2"] - self.bias_acc

        accel_norm = np.linalg.norm(accel_mps2)
        if accel_norm < 1e-12:
            return

        # Only trust accel as gravity direction when close to 1 g
        if not (self.accel_update_min_mps2 <= accel_norm <= self.accel_update_max_mps2):
            return

        # Measured gravity direction in body frame
        measured_dir_body = accel_mps2 / accel_norm

        # Expected gravity direction in body frame from current orientation
        rotation_world_from_body = quat_to_rotmat(self.quat)
        rotation_body_from_world = rotation_world_from_body.T

        gravity_dir_world = -self.gravity_world / np.linalg.norm(self.gravity_world)
        expected_dir_body = rotation_body_from_world @ gravity_dir_world
        expected_dir_body = expected_dir_body / np.linalg.norm(expected_dir_body)

        # --------------------------------------------------------
        # Optional debug: compare analytic vs numerical attitude Jacobian
        # --------------------------------------------------------
        self.accel_update_counter += 1
        if (
            self.debug_compare_accel_jacobian
            and self.accel_update_counter % self.debug_compare_accel_jacobian_every_n_updates == 0
        ):
            H_theta_num = compute_accel_measurement_jacobian_numerical(
                self.quat,
                self.gravity_world,
                eps=1e-6
            )
            H_theta_analytic = compute_accel_measurement_jacobian_analytic(
                self.quat,
                self.gravity_world
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

        # Innovation
        innovation = measured_dir_body - expected_dir_body

        # --------------------------------------------------------
        # Analytic measurement Jacobian
        #
        # First-order small-angle linearization for the chosen
        # right-multiplicative attitude error convention.
        #
        # First-order small-angle linearization of the gravity-direction measurement.
        # This is the standard EKF / error-state EKF approximation.
        # The sign depends on the chosen attitude-error convention.
        # For this implementation we use right-multiplicative correction:
        #     q <- q * dq
        # so the attitude Jacobian is +skew(expected_dir_body).
        # --------------------------------------------------------
        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 6:9] = skew_symmetric(expected_dir_body)

        R_meas = np.eye(3, dtype=np.float64) * (self.accel_meas_std ** 2)

        S = H @ self.cov @ H.T + R_meas
        K = self.cov @ H.T @ np.linalg.inv(S)

        # --------------------------------------------------------
        # Observability-consistent correction projection
        #
        # Gravity-direction update should not directly correct:
        # - position
        # - velocity
        # - accelerometer bias
        #
        # Also, rotation around the gravity direction is unobservable
        # from gravity alone, so we project that component out of:
        # - attitude correction
        # - gyro bias correction
        # --------------------------------------------------------
        gravity_axis_body = expected_dir_body / np.linalg.norm(expected_dir_body)
        tilt_projector = np.eye(3, dtype=np.float64) - np.outer(gravity_axis_body, gravity_axis_body)

        correction_projector = np.zeros((15, 15), dtype=np.float64)
        correction_projector[6:9, 6:9] = tilt_projector
        correction_projector[9:12, 9:12] = tilt_projector

        K_eff = correction_projector @ K
        error_state = K_eff @ innovation

        # --------------------------------------------------------
        # Joseph covariance update using the same effective gain
        # --------------------------------------------------------
        identity = np.eye(15, dtype=np.float64)
        temp = identity - K_eff @ H
        self.cov = temp @ self.cov @ temp.T + K_eff @ R_meas @ K_eff.T
        self.cov = 0.5 * (self.cov + self.cov.T)

        self.inject_error_state(error_state)


# ============================================================
# MPU-9250 register definitions
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
    """
    Combine two 8-bit bytes into one signed 16-bit integer.
    """
    value = (msb << 8) | lsb
    if value & 0x8000:
        value -= 65536
    return value


def compute_startup_imu_metrics(startup_samples):
    """
    Compute simple sanity metrics from a list of startup IMU samples.
    """
    gyro_array = np.array([sample["gyro_rps"] for sample in startup_samples], dtype=np.float64)
    accel_array = np.array([sample["accel_mps2"] for sample in startup_samples], dtype=np.float64)

    gyro_norm_array = np.linalg.norm(gyro_array, axis=1)
    accel_norm_array = np.linalg.norm(accel_array, axis=1)

    metrics = {
        "gyro_mean_rps": np.mean(gyro_array, axis=0),
        "accel_mean_mps2": np.mean(accel_array, axis=0),
        "gyro_norm_mean_rps": float(np.mean(gyro_norm_array)),
        "accel_norm_mean_mps2": float(np.mean(accel_norm_array)),
        "accel_norm_std_mps2": float(np.std(accel_norm_array)),
    }
    return metrics


def is_imu_stationary(metrics, config):
    """
    Decide whether startup data looks stationary enough for calibration.
    """
    gyro_ok = metrics["gyro_norm_mean_rps"] <= config.still_gyro_max_rps
    accel_mean_ok = (
        config.still_accel_norm_min_mps2
        <= metrics["accel_norm_mean_mps2"]
        <= config.still_accel_norm_max_mps2
    )
    accel_std_ok = metrics["accel_norm_std_mps2"] <= config.still_accel_std_max_mps2

    return gyro_ok and accel_mean_ok and accel_std_ok


def initialize_ekf_from_startup(ekf, config, startup_samples, startup_metrics):
    """
    Initialize EKF state from startup mode and startup IMU data.
    """
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
# MPU-9250 reader
# ============================================================

class Mpu9250Reader:
    """
    Minimal MPU-9250 reader for accelerometer, gyroscope, and temperature.

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
            # made with imu_calibration_logger.py (120 s for the 6 axis aligend poses) 
            # and analyzed with analyze_accel_six_pose_calibration.py
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
        Read WHO_AM_I from the MPU-9250.

        For MPU-9250 this is typically 0x71.
        """
        MPU_WHO_AM_I = 0x75
        return self.read_u8(MPU_WHO_AM_I)

    def read_block(self, register_addr, length):
        return self.i2c_bus.read_i2c_block_data(self.i2c_addr, register_addr, length)

    def initialize(self):
        """
        Initialize the MPU-9250.

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
        """
        Read one IMU sample and convert it to physical units.

        Returns a dictionary with:
            timestamp_sec
            accel_mps2
            gyro_rps
            temperature_c
        """
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

        # Use standard gravity here for raw unit conversion from g to m/s^2.
        # Do not replace this with local gravity unless the full calibration
        # pipeline is redone consistently.
        #
        # The EKF later estimates/uses the effective local gravity magnitude
        # from stationary startup data.
        g0 = 9.80665

        accel_mps2_raw = np.array([
            accel_x_g * g0,
            accel_y_g * g0,
            accel_z_g * g0
        ], dtype=np.float64)

        # Apply accelerometer bias and per-axis scale calibration
        accel_mps2 = (
            accel_mps2_raw - self.accel_bias_vec_mps2
        ) / self.accel_scale_vec

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
            print("Failed to communicate with MPU-9250 before initialization.")
            print("Check power, wiring, and run: sudo i2cdetect -y -r 7")
            raise

        imu_reader.initialize()

        # --------------------------------------------------------
        # Startup initialization
        # --------------------------------------------------------
        if config.startup_mode == "stationary_calibration":
            print("\nCollecting startup samples for stationary calibration...")
            for sample_index in range(config.startup_sample_count):
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

        # --------------------------------------------------------
        # Live EKF loop
        # --------------------------------------------------------
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
                        #f"pos m: {ekf.pos} | "
                        f"pos cm: {ekf.pos*100} | "
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