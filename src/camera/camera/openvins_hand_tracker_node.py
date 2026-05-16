#!/usr/bin/env python3
"""Publish world->wrist_link from the OpenVINS hand-camera TF.

OpenVINS owns the camera pose. The digital twin hand is rendered by applying the
fixed wrist->camera mounting transform in reverse.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def _rpy_to_matrix(roll, pitch, yaw, tx, ty, tz):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rot
    out[:3, 3] = [tx, ty, tz]
    return out


def _invert_matrix(mat):
    out = np.eye(4, dtype=np.float64)
    rot = mat[:3, :3]
    trans = mat[:3, 3]
    out[:3, :3] = rot.T
    out[:3, 3] = -rot.T @ trans
    return out


def _msg_to_matrix(msg: TransformStamped):
    t = msg.transform.translation
    q = msg.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = np.array([
        [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
    ], dtype=np.float64)
    out[:3, 3] = [t.x, t.y, t.z]
    return out


def _matrix_to_tf(mat, parent, child, stamp):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(mat[0, 3])
    msg.transform.translation.y = float(mat[1, 3])
    msg.transform.translation.z = float(mat[2, 3])

    rot = mat[:3, :3]
    trace = np.trace(rot)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx = (rot[2, 1] - rot[1, 2]) / s
        qy = (rot[0, 2] - rot[2, 0]) / s
        qz = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2
        qw = (rot[2, 1] - rot[1, 2]) / s
        qx = 0.25 * s
        qy = (rot[0, 1] + rot[1, 0]) / s
        qz = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2
        qw = (rot[0, 2] - rot[2, 0]) / s
        qx = (rot[0, 1] + rot[1, 0]) / s
        qy = 0.25 * s
        qz = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2
        qw = (rot[1, 0] - rot[0, 1]) / s
        qx = (rot[0, 2] + rot[2, 0]) / s
        qy = (rot[1, 2] + rot[2, 1]) / s
        qz = 0.25 * s

    msg.transform.rotation.x = float(qx)
    msg.transform.rotation.y = float(qy)
    msg.transform.rotation.z = float(qz)
    msg.transform.rotation.w = float(qw)
    return msg


class OpenVinsHandTracker(Node):
    def __init__(self):
        super().__init__("openvins_hand_tracker")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("camera_frame", "arm_d435i_arm_link")
        self.declare_parameter("wrist_frame", "wrist_link")
        self.declare_parameter("wrist_cam_tx", -0.04)
        self.declare_parameter("wrist_cam_ty", -0.01)
        self.declare_parameter("wrist_cam_tz", 0.20)
        self.declare_parameter("wrist_cam_roll", 1.57)
        self.declare_parameter("wrist_cam_pitch", 0.0)
        self.declare_parameter("wrist_cam_yaw", 1.57)
        self.declare_parameter("publish_rate", 30.0)

        self._world = self.get_parameter("world_frame").value
        self._camera = self.get_parameter("camera_frame").value
        self._wrist = self.get_parameter("wrist_frame").value
        wrist_to_camera = _rpy_to_matrix(
            self.get_parameter("wrist_cam_roll").value,
            self.get_parameter("wrist_cam_pitch").value,
            self.get_parameter("wrist_cam_yaw").value,
            self.get_parameter("wrist_cam_tx").value,
            self.get_parameter("wrist_cam_ty").value,
            self.get_parameter("wrist_cam_tz").value,
        )
        self._camera_to_wrist = _invert_matrix(wrist_to_camera)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._had_tf = False

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"OpenVINS hand tracker: {self._world}->{self._camera} -> {self._wrist}"
        )

    def _tick(self):
        try:
            tf_world_camera = self._tf_buffer.lookup_transform(
                self._world,
                self._camera,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            if self._had_tf:
                self.get_logger().warn("OpenVINS camera TF lost", throttle_duration_sec=5.0)
                self._had_tf = False
            return

        world_to_camera = _msg_to_matrix(tf_world_camera)
        world_to_wrist = world_to_camera @ self._camera_to_wrist
        msg = _matrix_to_tf(
            world_to_wrist,
            self._world,
            self._wrist,
            self.get_clock().now().to_msg(),
        )
        self._tf_broadcaster.sendTransform(msg)
        if not self._had_tf:
            self.get_logger().info("OpenVINS camera TF connected — hand follows arm camera")
            self._had_tf = True


def main(args=None):
    rclpy.init(args=args)
    node = OpenVinsHandTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
