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

    ekf = ImuEkfState()

    print("\nInitial nominal state:")
    print("Position pos:", ekf.pos)
    print("Velocity vel:", ekf.vel)
    print("Quaternion quat:", ekf.quat)
    print("Gyro bias bias_gyro:", ekf.bias_gyro)
    print("Accel bias bias_acc:", ekf.bias_acc)

    print("\nInitial covariance diagonal:")
    print(np.diag(ekf.cov))

    bus_num = 7

    print("\nOpening I2C bus and running prediction-only loop...")

    with SMBus(bus_num) as i2c_bus:
        imu_reader = Mpu9250Reader(i2c_bus)
        imu_reader.initialize()

        for sample_index in range(50):
            imu_sample = imu_reader.read_sample()
            ekf.predict_from_imu(imu_sample)

            roll, pitch, yaw = quat_to_rpy(ekf.quat)

            print(f"\nSample {sample_index + 1}")
            print("accel_mps2:", imu_sample["accel_mps2"])
            print("gyro_rps:", imu_sample["gyro_rps"])
            print("temperature_c:", imu_sample["temperature_c"])

            print("Estimated position pos [m]:", ekf.pos)
            print("Estimated velocity vel [m/s]:", ekf.vel)
            print("Estimated quaternion quat:", ekf.quat)
            print("Estimated roll, pitch, yaw [rad]:", (roll, pitch, yaw))

            time.sleep(0.02)

    print("\nStep 5 complete: prediction-only loop is working")

if __name__ == "__main__":
    main()