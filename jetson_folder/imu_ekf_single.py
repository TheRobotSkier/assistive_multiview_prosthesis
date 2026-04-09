#!/usr/bin/env python3
"""
Single-IMU EKF prototype (plain Python, direct I2C read)

Purpose
-------
This script will estimate the IMU pose relative to its start pose using:
- gyroscope
- accelerometer
- optional magnetometer later

Development plan
----------------
Step 1: file skeleton and imports
Step 2: math helper functions
Step 3: EKF state definition
Step 4: IMU read function
Step 5: prediction step
Step 6: accelerometer update
Step 7: optional magnetometer update
Step 8: main loop and logging

Notes
-----
- This version is a normal Python script, not a ROS 2 node.
- IMU data is read directly over I2C.
- Orientation will be stored internally as a quaternion.
- Roll, pitch, yaw can still be computed for debugging and printing.
"""

from logging import config
import time
import math
import numpy as np
from smbus2 import SMBus

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

    This will later be used for:
    - gyro integration
    - small EKF correction updates
    """
    dtheta = np.asarray(dtheta, dtype=np.float64)
    angle = np.linalg.norm(dtheta)

    if angle < 1e-12:
        # First-order approximation for very small angles
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

    We will use this later when we need to:
    - rotate acceleration from body frame to world frame
    - compute expected gravity direction in body frame
    - compute expected magnetic field direction in body frame
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

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w*y - z*x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def rpy_to_quat(roll_rad, pitch_rad, yaw_rad):
    """
    Convert roll, pitch, yaw [rad] into quaternion [w, x, y, z].

    This is used if the user wants to specify an initial start pose.
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

def apply_small_angle_quat_correction(quat, dtheta):
    """
    Apply a small-angle correction dtheta to an existing quaternion.

    Inputs:
        quat   : current quaternion [w, x, y, z]
        dtheta : small rotation vector [rad]

    Returns:
        corrected normalized quaternion

    Why:
    In the EKF, attitude (orientation in 3D) error is represented as a small 3D angle.
    We convert that small angle into a small quaternion and apply it.
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

    We will use this more later for EKF Jacobians.
    """
    x, y, z = vec3
    return np.array([
        [0.0, -z,  y],
        [z,   0.0, -x],
        [-y,  x,   0.0]
    ], dtype=np.float64)


class ImuEkfConfig:
    """
    Configuration values for startup behavior, thresholds, and optional
    future sensor calibration settings.

    This keeps 'tuning numbers' out of the EKF logic itself.
    """

    def __init__(self):
        # --------------------------------------------------------
        # Startup mode
        # --------------------------------------------------------
        # "instant":
        #   Start immediately using either:
        #   - a user-defined initial pose, or
        #   - identity orientation (startup pose becomes world frame)
        #
        # "stationary_calibration":
        #   Collect startup samples, check whether the IMU is still,
        #   estimate gyro bias, and estimate initial tilt from gravity.
        self.startup_mode = "instant"

        # --------------------------------------------------------
        # Optional user-defined initial pose
        # --------------------------------------------------------
        # If True, we use the user-defined initial pose below.
        # If False, startup orientation becomes the world reference pose.
        self.use_user_initial_pose = False

        # Initial position [m]
        self.initial_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # Initial orientation as roll, pitch, yaw [deg]
        # Only used if use_user_initial_pose = True
        self.initial_rpy_deg = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # --------------------------------------------------------
        # Startup stationary calibration settings
        # --------------------------------------------------------
        # Number of startup samples to collect if stationary calibration is used.
        self.startup_sample_count = 100

        # Delay between startup samples [s]
        self.startup_sample_dt_sec = 0.01

        # Stillness thresholds
        #
        # If the IMU is stationary:
        # - gyro magnitude should be small
        # - accel magnitude should be close to gravity
        # - accel variation should be small
        self.still_gyro_max_rps = 0.08
        self.still_accel_norm_min_mps2 = 9.3
        self.still_accel_norm_max_mps2 = 10.4
        self.still_accel_std_max_mps2 = 0.25

        # --------------------------------------------------------
        # Magnetometer configuration placeholders
        # --------------------------------------------------------
        # We are not using mag fusion yet, but we prepare for it here.
        self.use_magnetometer = False

        # If True, apply manual calibration values to mag data later.
        self.use_mag_calibration = False

        # Hard-iron offset (bias)
        self.mag_bias = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # Soft-iron scale matrix
        self.mag_scale_matrix = np.eye(3, dtype=np.float64)

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

    def __init__(self):
        # --------------------------------------------------------
        # Nominal state
        # --------------------------------------------------------

        # Position in world frame [m]
        # Start pose is defined as the origin.
        self.pos = np.zeros(3, dtype=np.float64)

        # Velocity in world frame [m/s]
        self.vel = np.zeros(3, dtype=np.float64)

        # Orientation quaternion [w, x, y, z]
        # Identity quaternion means "no rotation" at startup.
        self.quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

        # Gyroscope bias [rad/s]
        self.bias_gyro = np.zeros(3, dtype=np.float64)

        # Accelerometer bias [m/s^2]
        self.bias_acc = np.zeros(3, dtype=np.float64)

        # --------------------------------------------------------
        # Error-state covariance matrix
        # --------------------------------------------------------
        # Error-state ordering:
        #   0:3   -> position error            dpos
        #   3:6   -> velocity error            dvel
        #   6:9   -> attitude error            dtheta
        #   9:12  -> gyro bias error           dbias_gyro
        #   12:15 -> accelerometer bias error  dbias_acc
        self.cov = np.zeros((15, 15), dtype=np.float64)

        # Initial uncertainty values.
        # These are just reasonable starting values for now.
        # Later we can tune them using stationary measurements.

        # Position uncertainty [m^2]
        self.cov[0:3, 0:3] = np.eye(3) * 1e-6

        # Velocity uncertainty [(m/s)^2]
        self.cov[3:6, 3:6] = np.eye(3) * 1e-3

        # Attitude uncertainty [rad^2]
        self.cov[6:9, 6:9] = np.eye(3) * 1e-2

        # Gyro bias uncertainty [(rad/s)^2]
        self.cov[9:12, 9:12] = np.eye(3) * 1e-3

        # Accelerometer bias uncertainty [(m/s^2)^2]
        self.cov[12:15, 12:15] = np.eye(3) * 1e-2

        
        # Gravity magnitude used by the prediction model [m/s^2]
        self.gravity_mps2 = 9.80665

        # Gravity vector in world frame.
        # World frame is chosen with +Z up, so gravity points downward.
        self.gravity_world = np.array([0.0, 0.0, -self.gravity_mps2], dtype=np.float64)

        # Last timestamp used for prediction
        self.last_timestamp_sec = None

        
        # Accelerometer update settings
        # We only trust the accelerometer as a gravity measurement when
        # its magnitude is reasonably close to 1 g.
        self.accel_update_min_mps2 = 8.0
        self.accel_update_max_mps2 = 11.5

        # Accelerometer direction measurement noise
        self.accel_meas_std = 0.08
    

    def predict_from_imu(self, imu_sample):
        """
        Prediction step using one IMU sample.

        Inputs from imu_sample:
            imu_sample["timestamp_sec"]
            imu_sample["gyro_rps"]
            imu_sample["accel_mps2"]

        This method updates:
            self.quat
            self.vel
            self.pos

        Notes
        -----
        - Gyro is used to update orientation.
        - Acceleration is rotated from body frame to world frame.
        - Gravity is added in world frame to recover linear acceleration.
        - This is prediction only: no correction/update is applied yet.
        """

        timestamp_sec = imu_sample["timestamp_sec"]
        gyro_rps = imu_sample["gyro_rps"]
        accel_mps2 = imu_sample["accel_mps2"]

        # --------------------------------------------------------
        # Compute dt
        # --------------------------------------------------------
        if self.last_timestamp_sec is None:
            self.last_timestamp_sec = timestamp_sec
            return

        dt = timestamp_sec - self.last_timestamp_sec
        self.last_timestamp_sec = timestamp_sec

        # Reject clearly bad dt values (e.g. first sample, or if the IMU read is very delayed).
        if dt <= 0.0 or dt > 0.1:
            return

        # --------------------------------------------------------
        # 1) Bias-correct gyro and accel
        # --------------------------------------------------------
        gyro_unbiased_rps = gyro_rps - self.bias_gyro       # rps=[rad/s]
        accel_unbiased_mps2 = accel_mps2 - self.bias_acc    # mps2=[m/s^2]

        # --------------------------------------------------------
        # 2) Update orientation quaternion from gyro
        # --------------------------------------------------------
        delta_theta = gyro_unbiased_rps * dt
        delta_quat = quat_from_small_angle(delta_theta)

        # Apply the incremental rotation to the current orientation
        self.quat = quat_normalize(quat_mul(self.quat, delta_quat))

        # --------------------------------------------------------
        # 3) Rotate body-frame acceleration into world frame
        # --------------------------------------------------------
        rotation_world_from_body = quat_to_rotmat(self.quat)
        accel_world_mps2 = rotation_world_from_body @ accel_unbiased_mps2

        # --------------------------------------------------------
        # 4) Add gravity in world frame to get linear acceleration
        # --------------------------------------------------------
        # Accelerometers measure specific force, not direct world-frame
        # linear acceleration. For this simple inertial model:
        #
        #   linear_accel_world = rotated_accel + gravity_world
        #
        linear_accel_world_mps2 = accel_world_mps2 + self.gravity_world # mps2=[m/s^2]

        # --------------------------------------------------------
        # 5) Integrate to velocity and position
        # --------------------------------------------------------
        old_vel = self.vel.copy()

        self.vel = self.vel + linear_accel_world_mps2 * dt
        self.pos = self.pos + old_vel * dt + 0.5 * linear_accel_world_mps2 * dt * dt

        # --------------------------------------------------------
        # 6) Accelerometer correction update
        # --------------------------------------------------------
        self.update_from_accel(imu_sample)


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

        What it does NOT correct:
        - yaw
        """
        accel_mps2 = imu_sample["accel_mps2"]

        accel_norm = np.linalg.norm(accel_mps2)
        if accel_norm < 1e-12:
            return

        # Only use the accelerometer as a gravity-direction measurement
        # when its magnitude is close to gravity.
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

        # Innovation: measured minus expected gravity direction
        innovation = measured_dir_body - expected_dir_body

        # Measurement Jacobian H:
        # For this first readable version, we only model sensitivity to attitude error.
        H = np.zeros((3, 15), dtype=np.float64)

        eps = 1e-6
        for axis_index in range(3):
            dtheta = np.zeros(3, dtype=np.float64)
            dtheta[axis_index] = eps

            perturbed_quat = apply_small_angle_quat_correction(self.quat, dtheta)
            perturbed_rot_world_from_body = quat_to_rotmat(perturbed_quat)
            perturbed_rot_body_from_world = perturbed_rot_world_from_body.T

            perturbed_expected_dir_body = perturbed_rot_body_from_world @ gravity_dir_world
            perturbed_expected_dir_body = (
                perturbed_expected_dir_body / np.linalg.norm(perturbed_expected_dir_body)
            )

            H[:, 6 + axis_index] = (
                perturbed_expected_dir_body - expected_dir_body
            ) / eps

        R_meas = np.eye(3, dtype=np.float64) * (self.accel_meas_std ** 2)

        S = H @ self.cov @ H.T + R_meas
        K = self.cov @ H.T @ np.linalg.inv(S)

        error_state = K @ innovation

        identity = np.eye(15, dtype=np.float64)
        self.cov = (identity - K @ H) @ self.cov
        self.cov = 0.5 * (self.cov + self.cov.T) # just a numerical cleanup step, because covariance should stay symmetric.

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

    The MPU-9250 stores many sensor values as:
    - high byte (MSB)
    - low byte (LSB)

    This function converts those two bytes into a normal Python int.
    """
    value = (msb << 8) | lsb
    if value & 0x8000:
        value -= 65536
    return value

def compute_startup_imu_metrics(startup_samples):
    """
    Compute simple sanity metrics from a list of startup IMU samples.

    Returns a dictionary containing:
    - mean gyro
    - mean accel
    - mean accel norm
    - std accel norm
    - mean gyro norm
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

class Mpu9250Reader:
    """
    Minimal MPU-9250 reader for accelerometer, gyroscope, and temperature.

    This class:
    - opens the sensor through an already-open SMBus object
    - configures the IMU
    - reads one sample at a time
    - converts raw values into physical units

    Output units:
    - acceleration: m/s^2
    - angular velocity: rad/s
    - temperature: deg C
    """

    def __init__(self, i2c_bus, i2c_addr=MPU9250_I2C_ADDR):
        self.i2c_bus = i2c_bus
        self.i2c_addr = i2c_addr

    def write_u8(self, register_addr, value):
        """Write one byte to one IMU register."""
        self.i2c_bus.write_byte_data(self.i2c_addr, register_addr, value)

    def read_block(self, register_addr, length):
        """Read multiple bytes starting from one IMU register."""
        return self.i2c_bus.read_i2c_block_data(self.i2c_addr, register_addr, length)
    
    def initialize(self):
        """
        Initialize the MPU-9250.

        Configuration chosen for now:
        - wake up the sensor
        - sample rate divider = 4
        - low-pass filter enabled
        - gyro range = ±250 deg/s
        - accel range = ±2 g

        These settings match the scale factors we use below:
        - gyro sensitivity = 131 LSB/(deg/s)
        - accel sensitivity = 16384 LSB/g
        """
        # Wake up the sensor
        self.write_u8(MPU_PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Sample rate divider
        # With DLPF enabled, internal rate is typically 1 kHz,
        # so sample_rate = 1000 / (1 + 4) = 200 Hz
        self.write_u8(MPU_SMPLRT_DIV, 0x04)

        # Low-pass filter configuration
        self.write_u8(MPU_CONFIG, 0x03)

        # Gyro full-scale range = ±250 deg/s
        self.write_u8(MPU_GYRO_CONFIG, 0x00)

        # Accelerometer full-scale range = ±2 g
        self.write_u8(MPU_ACCEL_CONFIG, 0x00)

        # Additional accelerometer filtering
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

        Notes on conversions
        --------------------
        Accel:
            ±2 g  => 16384 LSB/g
            accel_g = raw / 16384
            accel_mps2 = accel_g * 9.80665

        Gyro:
            ±250 deg/s => 131 LSB/(deg/s)
            gyro_dps = raw / 131
            gyro_rps = gyro_dps * pi / 180

        Temperature:
            temp_c = (temp_raw / 333.87) + 21.0
        """
        timestamp_sec = time.time()

        # Read 14 bytes:
        # accel xyz (6), temp (2), gyro xyz (6)
        data = self.read_block(MPU_ACCEL_XOUT_H, 14)

        raw_accel_x = int16_from_bytes(data[0], data[1])
        raw_accel_y = int16_from_bytes(data[2], data[3])
        raw_accel_z = int16_from_bytes(data[4], data[5])

        raw_temp = int16_from_bytes(data[6], data[7])

        raw_gyro_x = int16_from_bytes(data[8], data[9])
        raw_gyro_y = int16_from_bytes(data[10], data[11])
        raw_gyro_z = int16_from_bytes(data[12], data[13])

        # ----------------------------------------
        # Convert accelerometer to m/s^2
        # ----------------------------------------
        accel_x_g = raw_accel_x / 16384.0
        accel_y_g = raw_accel_y / 16384.0
        accel_z_g = raw_accel_z / 16384.0

        g0 = 9.80665  # standard gravity [m/s^2]

        accel_mps2 = np.array([
            accel_x_g * g0,
            accel_y_g * g0,
            accel_z_g * g0
        ], dtype=np.float64)

        # ----------------------------------------
        # Convert gyroscope to rad/s
        # ----------------------------------------
        gyro_x_dps = raw_gyro_x / 131.0
        gyro_y_dps = raw_gyro_y / 131.0
        gyro_z_dps = raw_gyro_z / 131.0

        deg_to_rad = math.pi / 180.0

        gyro_rps = np.array([
            gyro_x_dps * deg_to_rad,
            gyro_y_dps * deg_to_rad,
            gyro_z_dps * deg_to_rad
        ], dtype=np.float64)

        # ----------------------------------------
        # Convert temperature to deg C
        # ----------------------------------------
        temperature_c = (raw_temp / 333.87) + 21.0

        return {
            "timestamp_sec": timestamp_sec,
            "accel_mps2": accel_mps2,
            "gyro_rps": gyro_rps,
            "temperature_c": temperature_c,
            "raw_accel": np.array([raw_accel_x, raw_accel_y, raw_accel_z], dtype=np.int32),
            "raw_gyro": np.array([raw_gyro_x, raw_gyro_y, raw_gyro_z], dtype=np.int32),
        }


def main():
    print("Single-IMU EKF prototype started")

    config = ImuEkfConfig()
    #config.startup_mode = "stationary_calibration"
    ekf = ImuEkfState()

    print("\nStartup configuration:")
    print("startup_mode:", config.startup_mode)
    print("use_user_initial_pose:", config.use_user_initial_pose)
    print("initial_pos:", config.initial_pos)
    print("initial_rpy_deg:", config.initial_rpy_deg)
    print("use_magnetometer:", config.use_magnetometer)
    print("use_mag_calibration:", config.use_mag_calibration)

    bus_num = 7

    with SMBus(bus_num) as i2c_bus:
        imu_reader = Mpu9250Reader(i2c_bus)
        imu_reader.initialize()

        startup_samples = []
        for sample_index in range(config.startup_sample_count):
            sample = imu_reader.read_sample()
            startup_samples.append(sample)
            time.sleep(config.startup_sample_dt_sec)

    metrics = compute_startup_imu_metrics(startup_samples)
    stationary = is_imu_stationary(metrics, config)

    print("\nStartup sanity metrics:")
    print("gyro_mean_rps:", metrics["gyro_mean_rps"])
    print("accel_mean_mps2:", metrics["accel_mean_mps2"])
    print("gyro_norm_mean_rps:", metrics["gyro_norm_mean_rps"])
    print("accel_norm_mean_mps2:", metrics["accel_norm_mean_mps2"])
    print("accel_norm_std_mps2:", metrics["accel_norm_std_mps2"])
    print("stationary_detected:", stationary)

    print("\nStep 7A complete: startup config and sanity metrics are ready")

if __name__ == "__main__":
    main()