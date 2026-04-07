#!/usr/bin/env python3
"""
single_imu_ekf_node.py

Single-IMU EKF node for ROS 2.

Purpose
-------
Fuse one IMU's gyroscope + accelerometer, with optional magnetometer,
to estimate pose relative to the start frame.

This node:
- predicts state on every IMU message
- optionally corrects orientation using accelerometer (gravity direction)
- optionally corrects yaw/orientation using magnetometer
- publishes pose/velocity at a lower fixed rate (default: 15 Hz)

State
-----
Nominal state:
    p  : position in world frame              (3,)
    v  : velocity in world frame              (3,)
    q  : orientation world_from_body quat     (4,) as [w, x, y, z]
    bg : gyro bias                            (3,)
    ba : accel bias                           (3,)

Error-state covariance:
    15 x 15  for [dp, dv, dtheta, dbg, dba]

Important limitations
---------------------
- IMU-only position will drift over time.
- Without magnetometer or vision, yaw will drift.
- The accelerometer update assumes that, on average, gravity is the dominant
  measured acceleration direction. During aggressive motion this is less true.

ROS interfaces
--------------
Subscriptions:
- /imu/data_raw    sensor_msgs/Imu
- /imu/mag         sensor_msgs/MagneticField  (optional)

Publications:
- /imu/ekf/odom    nav_msgs/Odometry
- /imu/ekf/pose    geometry_msgs/PoseStamped
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def skew(v: np.ndarray) -> np.ndarray:
    """Return the 3x3 skew-symmetric matrix of a 3-vector."""
    x, y, z = v
    return np.array([
        [0.0, -z,  y],
        [z,   0.0, -x],
        [-y,  x,   0.0]
    ], dtype=np.float64)


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion [w, x, y, z]."""
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=np.float64)


def quat_from_small_angle(dtheta: np.ndarray) -> np.ndarray:
    """
    Quaternion from a small rotation vector.
    Uses first-order approximation when angle is tiny.
    """
    angle = np.linalg.norm(dtheta)
    if angle < 1e-12:
        return quat_normalize(np.array([1.0, 0.5*dtheta[0], 0.5*dtheta[1], 0.5*dtheta[2]], dtype=np.float64))

    axis = dtheta / angle
    half = 0.5 * angle
    s = math.sin(half)
    return np.array([math.cos(half), axis[0]*s, axis[1]*s, axis[2]*s], dtype=np.float64)


def quat_from_omega_dt(omega: np.ndarray, dt: float) -> np.ndarray:
    """Quaternion increment from angular velocity * dt."""
    return quat_from_small_angle(omega * dt)


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """Rotation matrix world_from_body from quaternion [w, x, y, z]."""
    q = quat_normalize(q)
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion [w, x, y, z]."""
    trace = np.trace(R)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return quat_normalize(np.array([w, x, y, z], dtype=np.float64))


def quat_to_msg_xyzw(q: np.ndarray):
    """
    Convert internal [w, x, y, z] to ROS order (x, y, z, w).
    """
    return q[1], q[2], q[3], q[0]


class SingleImuEkfNode(Node):
    def __init__(self):
        super().__init__('single_imu_ekf_node')

        # -------------------------------
        # Parameters
        # -------------------------------
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('use_magnetometer', False)

        self.declare_parameter('world_frame', 'imu_start')
        self.declare_parameter('body_frame', 'imu_link')

        self.declare_parameter('publish_rate_hz', 15.0)

        # Gravity in world frame (ENU-style world: +Z up => gravity = [0,0,-9.81])
        self.declare_parameter('gravity_mps2', 9.81)

        # Initial magnetic field reference in world frame.
        # This is only a direction reference for the prototype.
        # A simple default is +X in world.
        self.declare_parameter('mag_world_x', 1.0)
        self.declare_parameter('mag_world_y', 0.0)
        self.declare_parameter('mag_world_z', 0.0)

        # Process noise parameters
        self.declare_parameter('gyro_noise_std', 0.02)      # rad/s / sqrt(Hz) approx prototype value
        self.declare_parameter('accel_noise_std', 0.30)     # m/s^2 / sqrt(Hz)
        self.declare_parameter('gyro_bias_rw_std', 0.001)   # rad/s^2-ish random walk
        self.declare_parameter('accel_bias_rw_std', 0.01)   # m/s^3-ish random walk

        # Measurement noise parameters
        self.declare_parameter('accel_meas_std', 0.08)      # normalized gravity direction residual
        self.declare_parameter('mag_meas_std', 0.10)        # normalized magnetic direction residual

        # Accelerometer update gating
        self.declare_parameter('accel_update_min_g', 8.0)
        self.declare_parameter('accel_update_max_g', 11.5)

        # Initial covariance scalars
        self.declare_parameter('init_pos_cov', 1e-6)
        self.declare_parameter('init_vel_cov', 1e-3)
        self.declare_parameter('init_att_cov', 1e-2)
        self.declare_parameter('init_gyro_bias_cov', 1e-3)
        self.declare_parameter('init_accel_bias_cov', 1e-2)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.mag_topic = self.get_parameter('mag_topic').value
        self.use_magnetometer = bool(self.get_parameter('use_magnetometer').value)

        self.world_frame = self.get_parameter('world_frame').value
        self.body_frame = self.get_parameter('body_frame').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        g = float(self.get_parameter('gravity_mps2').value)
        self.g_world = np.array([0.0, 0.0, -g], dtype=np.float64)

        mref = np.array([
            float(self.get_parameter('mag_world_x').value),
            float(self.get_parameter('mag_world_y').value),
            float(self.get_parameter('mag_world_z').value),
        ], dtype=np.float64)
        self.mag_world = mref / max(np.linalg.norm(mref), 1e-12)

        self.gyro_noise_std = float(self.get_parameter('gyro_noise_std').value)
        self.accel_noise_std = float(self.get_parameter('accel_noise_std').value)
        self.gyro_bias_rw_std = float(self.get_parameter('gyro_bias_rw_std').value)
        self.accel_bias_rw_std = float(self.get_parameter('accel_bias_rw_std').value)

        self.accel_meas_std = float(self.get_parameter('accel_meas_std').value)
        self.mag_meas_std = float(self.get_parameter('mag_meas_std').value)

        self.accel_update_min_g = float(self.get_parameter('accel_update_min_g').value)
        self.accel_update_max_g = float(self.get_parameter('accel_update_max_g').value)

        # -------------------------------
        # Nominal state
        # -------------------------------
        self.p = np.zeros(3, dtype=np.float64)  # world position
        self.v = np.zeros(3, dtype=np.float64)  # world velocity
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # world_from_body quaternion
        self.bg = np.zeros(3, dtype=np.float64)
        self.ba = np.zeros(3, dtype=np.float64)

        # 15x15 error-state covariance:
        # [dp, dv, dtheta, dbg, dba]
        self.P = np.zeros((15, 15), dtype=np.float64)
        self.P[0:3, 0:3] = np.eye(3) * float(self.get_parameter('init_pos_cov').value)
        self.P[3:6, 3:6] = np.eye(3) * float(self.get_parameter('init_vel_cov').value)
        self.P[6:9, 6:9] = np.eye(3) * float(self.get_parameter('init_att_cov').value)
        self.P[9:12, 9:12] = np.eye(3) * float(self.get_parameter('init_gyro_bias_cov').value)
        self.P[12:15, 12:15] = np.eye(3) * float(self.get_parameter('init_accel_bias_cov').value)

        # -------------------------------
        # Runtime state
        # -------------------------------
        self.last_imu_time = None
        self.have_imu = False
        self.have_mag = False
        self.latest_mag_body = None

        # Publishers / subscribers
        self.odom_pub = self.create_publisher(Odometry, '/imu/ekf/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/imu/ekf/pose', 10)

        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_cb, 100)
        if self.use_magnetometer:
            self.mag_sub = self.create_subscription(MagneticField, self.mag_topic, self.mag_cb, 100)
        else:
            self.mag_sub = None

        self.pub_timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_outputs)

        self.get_logger().info(f'Single IMU EKF started. IMU topic={self.imu_topic}, mag enabled={self.use_magnetometer}')

    def mag_cb(self, msg: MagneticField):
        """
        Store the latest magnetometer sample in body frame.

        MagneticField contains magnetic field vector components.
        For this prototype we use only the direction, not magnitude.
        """
        m = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        ], dtype=np.float64)

        n = np.linalg.norm(m)
        if n > 1e-12:
            self.latest_mag_body = m / n
            self.have_mag = True

    def imu_cb(self, msg: Imu):
        """
        Predict on every IMU sample, then apply accel/mag updates.
        """
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        omega_m = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=np.float64)

        accel_m = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=np.float64)

        if self.last_imu_time is None:
            self.last_imu_time = t
            self.have_imu = True
            return

        dt = t - self.last_imu_time
        self.last_imu_time = t

        if dt <= 0.0 or dt > 0.1:
            # Reject obviously bad dt values
            return

        # 1) Prediction
        self.predict(omega_m, accel_m, dt)

        # 2) Accelerometer correction as gravity-direction measurement
        accel_norm = np.linalg.norm(accel_m)
        if self.accel_update_min_g <= accel_norm <= self.accel_update_max_g:
            z_acc = accel_m / max(accel_norm, 1e-12)
            self.update_accel(z_acc)

        # 3) Optional magnetometer correction
        if self.use_magnetometer and self.have_mag and self.latest_mag_body is not None:
            self.update_mag(self.latest_mag_body)

    def predict(self, omega_m: np.ndarray, accel_m: np.ndarray, dt: float):
        """
        IMU process model.

        Uses:
            omega = omega_m - bg
            accel = accel_m - ba

        State propagation:
            q <- q * dq
            a_world = R(q) * accel + g_world
            v <- v + a_world * dt
            p <- p + v * dt + 0.5 * a_world * dt^2
        """
        omega = omega_m - self.bg
        accel = accel_m - self.ba

        # Orientation update
        dq = quat_from_omega_dt(omega, dt)
        q_prev = self.q.copy()
        self.q = quat_normalize(quat_mul(self.q, dq))

        # Acceleration to world frame
        Rwb = quat_to_rotmat(q_prev)  # world_from_body
        a_world = Rwb @ accel + self.g_world

        # Position / velocity propagation
        self.p = self.p + self.v * dt + 0.5 * a_world * dt * dt
        self.v = self.v + a_world * dt

        # Linearized covariance propagation (simplified ESKF form)
        F = np.eye(15, dtype=np.float64)

        # dp/dv
        F[0:3, 3:6] = np.eye(3) * dt

        # dv/dtheta  approx -R[a]_x dt
        F[3:6, 6:9] = -Rwb @ skew(accel) * dt

        # dv/dba
        F[3:6, 12:15] = -Rwb * dt

        # dtheta/dtheta  approx I - [omega]x dt
        F[6:9, 6:9] = np.eye(3) - skew(omega) * dt

        # dtheta/dbg
        F[6:9, 9:12] = -np.eye(3) * dt

        # Process noise
        Q = np.zeros((15, 15), dtype=np.float64)

        gyro_var = (self.gyro_noise_std ** 2) * dt
        accel_var = (self.accel_noise_std ** 2) * dt
        gyro_bias_var = (self.gyro_bias_rw_std ** 2) * dt
        accel_bias_var = (self.accel_bias_rw_std ** 2) * dt

        # Simple injected process noise
        Q[3:6, 3:6] += np.eye(3) * accel_var
        Q[6:9, 6:9] += np.eye(3) * gyro_var
        Q[9:12, 9:12] += np.eye(3) * gyro_bias_var
        Q[12:15, 12:15] += np.eye(3) * accel_bias_var

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    def update_accel(self, z_acc_body_unit: np.ndarray):
        """
        Accelerometer update using gravity direction.

        Measurement model:
            expected body-frame gravity direction = R_bw * ( -g_world / |g| )

        Because the accelerometer measures proper acceleration, when motion is slow,
        it approximately points opposite to gravity direction in body frame.
        """
        g_ref_world_unit = -self.g_world / max(np.linalg.norm(self.g_world), 1e-12)

        def h_of_q(q: np.ndarray) -> np.ndarray:
            Rwb = quat_to_rotmat(q)
            Rbw = Rwb.T
            pred = Rbw @ g_ref_world_unit
            return pred / max(np.linalg.norm(pred), 1e-12)

        self._direction_update(
            z=z_acc_body_unit,
            h_func=h_of_q,
            R_meas=np.eye(3) * (self.accel_meas_std ** 2)
        )

    def update_mag(self, z_mag_body_unit: np.ndarray):
        """
        Magnetometer update using magnetic field direction.

        This prototype assumes a fixed world-frame magnetic direction vector.
        It improves heading/yaw observability if the field is reasonably undisturbed.
        """
        def h_of_q(q: np.ndarray) -> np.ndarray:
            Rwb = quat_to_rotmat(q)
            Rbw = Rwb.T
            pred = Rbw @ self.mag_world
            return pred / max(np.linalg.norm(pred), 1e-12)

        self._direction_update(
            z=z_mag_body_unit,
            h_func=h_of_q,
            R_meas=np.eye(3) * (self.mag_meas_std ** 2)
        )

    def _direction_update(self, z: np.ndarray, h_func, R_meas: np.ndarray):
        """
        Generic EKF update for a 3D unit-direction measurement.

        We numerically differentiate only wrt orientation error dtheta,
        which keeps the implementation compact and practical for a prototype.
        """
        z = z / max(np.linalg.norm(z), 1e-12)
        h0 = h_func(self.q)
        y = z - h0

        # Measurement Jacobian H is 3 x 15
        H = np.zeros((3, 15), dtype=np.float64)

        # Numerical derivative wrt dtheta only
        eps = 1e-6
        for i in range(3):
            dtheta = np.zeros(3, dtype=np.float64)
            dtheta[i] = eps
            dq = quat_from_small_angle(dtheta)
            q_pert = quat_normalize(quat_mul(self.q, dq))
            hi = h_func(q_pert)
            H[:, 6 + i] = (hi - h0) / eps

        S = H @ self.P @ H.T + R_meas
        K = self.P @ H.T @ np.linalg.inv(S)

        dx = K @ y

        I = np.eye(15, dtype=np.float64)
        self.P = (I - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)

        self.inject_error(dx)

    def inject_error(self, dx: np.ndarray):
        """
        Inject 15D error-state correction into nominal state.
        dx = [dp, dv, dtheta, dbg, dba]
        """
        self.p += dx[0:3]
        self.v += dx[3:6]

        dtheta = dx[6:9]
        dq = quat_from_small_angle(dtheta)
        self.q = quat_normalize(quat_mul(self.q, dq))

        self.bg += dx[9:12]
        self.ba += dx[12:15]

    def publish_outputs(self):
        """Publish pose + odometry at a lower fixed rate."""
        if not self.have_imu:
            return

        now = self.get_clock().now().to_msg()

        # PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.world_frame

        pose_msg.pose.position.x = float(self.p[0])
        pose_msg.pose.position.y = float(self.p[1])
        pose_msg.pose.position.z = float(self.p[2])

        qx, qy, qz, qw = quat_to_msg_xyzw(self.q)
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)

        self.pose_pub.publish(pose_msg)

        # Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.body_frame

        odom.pose.pose.position.x = float(self.p[0])
        odom.pose.pose.position.y = float(self.p[1])
        odom.pose.pose.position.z = float(self.p[2])

        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)

        odom.twist.twist.linear.x = float(self.v[0])
        odom.twist.twist.linear.y = float(self.v[1])
        odom.twist.twist.linear.z = float(self.v[2])

        # Store a simple covariance mapping
        # pose covariance indices: xyz at [0,7,14], orientation at [21,28,35]
        odom.pose.covariance[0] = float(self.P[0, 0])
        odom.pose.covariance[7] = float(self.P[1, 1])
        odom.pose.covariance[14] = float(self.P[2, 2])

        odom.pose.covariance[21] = float(self.P[6, 6])
        odom.pose.covariance[28] = float(self.P[7, 7])
        odom.pose.covariance[35] = float(self.P[8, 8])

        odom.twist.covariance[0] = float(self.P[3, 3])
        odom.twist.covariance[7] = float(self.P[4, 4])
        odom.twist.covariance[14] = float(self.P[5, 5])

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SingleImuEkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()