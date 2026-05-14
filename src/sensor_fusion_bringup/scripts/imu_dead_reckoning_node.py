#!/usr/bin/env python3
"""IMU-only dead reckoning node for RealSense cameras.

Subscribes to IMU topics, runs a per-camera CALIBRATING→TRACKING state
machine, integrates gyro+accel, and publishes TF imu_test_world→camN_imu.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


# ── Pure numpy quaternion helpers ──────────────────────────────────────────

def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of two quaternions in [x, y, z, w] form."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ], dtype=float)


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion; return identity if near-zero norm."""
    n = np.linalg.norm(q)
    if n < 1e-10:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def rotate_vec(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate 3-vector v by unit quaternion q (active rotation, [x,y,z,w])."""
    x, y, z, w = q
    R = np.array([
        [1.0 - 2*(y*y + z*z),  2*(x*y - w*z),       2*(x*z + w*y)],
        [2*(x*y + w*z),        1.0 - 2*(x*x + z*z),  2*(y*z - w*x)],
        [2*(x*z - w*y),        2*(y*z + w*x),         1.0 - 2*(x*x + y*y)],
    ], dtype=float)
    return R @ np.asarray(v, dtype=float)


def integrate_gyro(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
    """First-order body-frame gyro integration.

    Rotates q by the angular displacement omega*dt expressed in body frame.
    Returns normalized result.
    """
    angle = np.linalg.norm(omega) * dt
    if angle < 1e-10:
        return q
    axis = omega / np.linalg.norm(omega)
    s = math.sin(angle / 2.0)
    dq = np.array([axis[0]*s, axis[1]*s, axis[2]*s, math.cos(angle / 2.0)], dtype=float)
    return quat_normalize(quat_mult(q, dq))


# ── Per-camera state ────────────────────────────────────────────────────────

class CalibState(Enum):
    CALIBRATING = "CALIBRATING"
    TRACKING = "TRACKING"


@dataclass
class CameraState:
    name: str
    imu_topic: str
    calib_duration: float

    state: CalibState = CalibState.CALIBRATING
    accel_samples: list = field(default_factory=list)
    gyro_samples: list = field(default_factory=list)
    calib_start_sec: float = -1.0

    # Calibration results
    g_world: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))

    # Integration state
    q: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    v: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    p: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    last_stamp_sec: float = -1.0


def finish_calibration(state: CameraState, logger) -> None:
    """Compute g_world and gyro_bias from collected samples; switch to TRACKING."""
    if not state.accel_samples or not state.gyro_samples:
        logger.warning(f"[{state.name}] Calibration ended with no samples — staying in CALIBRATING")
        return
    state.g_world = np.mean(state.accel_samples, axis=0)
    state.gyro_bias = np.mean(state.gyro_samples, axis=0)
    state.state = CalibState.TRACKING
    g_mag = float(np.linalg.norm(state.g_world))
    logger.info(
        f"[{state.name}] Calibration done: "
        f"g_world={state.g_world.tolist()}, "
        f"gyro_bias={state.gyro_bias.tolist()}, "
        f"|g|={g_mag:.3f} m/s² (expected ~9.81)"
    )


def integration_step(state: CameraState, accel: np.ndarray, gyro: np.ndarray,
                     stamp_sec: float) -> None:
    """One IMU integration step. Mutates state in-place."""
    if state.last_stamp_sec < 0:
        state.last_stamp_sec = stamp_sec
        return
    dt = stamp_sec - state.last_stamp_sec
    state.last_stamp_sec = stamp_sec
    if dt <= 0.0 or dt > 0.5:
        return

    omega = gyro - state.gyro_bias
    state.q = integrate_gyro(state.q, omega, dt)

    a_world = rotate_vec(state.q, accel)
    a_lin = a_world - state.g_world

    state.v += a_lin * dt
    state.p += state.v * dt


# ── ROS2 node ───────────────────────────────────────────────────────────────

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class ImuDeadReckoningNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_dead_reckoning_node")

        self.declare_parameter("camera_configs", "[]")
        self.declare_parameter("calibration_duration", 2.0)
        self.declare_parameter("config_dir", "")

        configs_json: str = self.get_parameter("camera_configs").value
        calib_dur: float = float(self.get_parameter("calibration_duration").value)
        config_dir: str = str(self.get_parameter("config_dir").value)

        try:
            configs: list = json.loads(configs_json)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"camera_configs is not valid JSON: {exc}") from exc

        self.tf_broadcaster = TransformBroadcaster(self)
        self._camera_states: dict = {}

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        for cfg in configs:
            name: str = cfg["name"]
            topic: str = cfg["imu_topic"]
            state = CameraState(name=name, imu_topic=topic, calib_duration=calib_dur)
            self._camera_states[name] = state
            self.create_subscription(
                Imu, topic,
                lambda msg, n=name: self._imu_cb(n, msg),
                sensor_qos,
            )
            self.get_logger().info(
                f"[{name}] Subscribed to IMU topic: {topic}"
            )

        if config_dir:
            self._log_calib_matches(configs, config_dir)

        self.get_logger().info(
            f"IMU dead reckoning node ready. "
            f"Cameras: {[c['name'] for c in configs]}. "
            f"Calibration duration: {calib_dur} s (hold cameras still)."
        )

    def _imu_cb(self, camera_name: str, msg: Imu) -> None:
        state = self._camera_states[camera_name]
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ], dtype=float)
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ], dtype=float)

        if state.state == CalibState.CALIBRATING:
            if state.calib_start_sec < 0.0:
                state.calib_start_sec = stamp_sec
                self.get_logger().info(
                    f"[{camera_name}] Calibration started — hold camera still for "
                    f"{state.calib_duration:.1f} s"
                )
            state.accel_samples.append(accel)
            state.gyro_samples.append(gyro)
            if stamp_sec - state.calib_start_sec >= state.calib_duration:
                finish_calibration(state, self.get_logger())
            return

        # TRACKING
        integration_step(state, accel, gyro, stamp_sec)
        self._publish_tf(camera_name, state, msg.header.stamp)

    def _publish_tf(self, camera_name: str, state: CameraState, stamp) -> None:
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "imu_test_world"
        tf.child_frame_id = f"{camera_name}_imu"
        tf.transform.translation.x = float(state.p[0])
        tf.transform.translation.y = float(state.p[1])
        tf.transform.translation.z = float(state.p[2])
        tf.transform.rotation.x = float(state.q[0])
        tf.transform.rotation.y = float(state.q[1])
        tf.transform.rotation.z = float(state.q[2])
        tf.transform.rotation.w = float(state.q[3])
        self.tf_broadcaster.sendTransform(tf)

    def _log_calib_matches(self, configs: list, config_dir: str) -> None:
        try:
            entries = [
                e for e in os.listdir(config_dir)
                if os.path.isdir(os.path.join(config_dir, e))
            ]
        except OSError:
            self.get_logger().warning(f"Cannot list config_dir: {config_dir}")
            return
        for cfg in configs:
            serial: str = cfg.get("serial", "")
            matches = [e for e in entries if serial and serial in e]
            if matches:
                self.get_logger().info(
                    f"[{cfg['name']}] Calibration dir matched: {matches[0]} (informational)"
                )
            else:
                self.get_logger().warning(
                    f"[{cfg['name']}] No calib dir for serial={serial!r} "
                    "(using runtime gravity estimation)"
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuDeadReckoningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
