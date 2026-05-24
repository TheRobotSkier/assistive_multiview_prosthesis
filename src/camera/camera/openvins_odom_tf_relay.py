#!/usr/bin/env python3
"""Relay OpenVINS odometry to TF transforms on the host.

Publishes the dynamic ``marker_map -> *_imu`` TF edges from OpenVINS odometry
messages, and optionally the static ``*_imu -> *_cam0`` extrinsic.  This makes
the host self-sufficient for the TF chain that the pointcloud fusion node
needs, bypassing unreliable DDS ``/tf`` delivery from the Jetson.

The odometry messages arrive at ~200 Hz on separate topics (one per camera).
Each message carries ``T(marker_map -> imu)`` in its pose field.  We extract
that pose and publish it as a local TF transform stamped with the **host clock**
to avoid the "extrapolation into the past" errors caused by Jetson-host clock
skew.

Parameters
----------
head_odom_topic          str   input Odometry topic for head camera
                                (default: /ov_msckf/odomimu)
arm_odom_topic           str   input Odometry topic for arm camera
                                (default: /ov_msckf_arm/odomimu)
target_frame             str   parent frame of the dynamic TF (default: marker_map)
head_imu_frame           str   child frame for head (default: head_imu)
arm_imu_frame            str   child frame for arm  (default: arm_imu)
head_cam_frame           str   head camera frame from OpenVINS (default: head_cam0)
arm_cam_frame            str   arm camera frame from OpenVINS  (default: arm_cam0)
publish_imu_to_cam_tf    bool  publish static *_imu -> *_cam0 edges (default: True)
imu_to_cam_extrinsics_head  dict  nominal head imu->cam0 {x,y,z,roll,pitch,yaw}
imu_to_cam_extrinsics_arm   dict  nominal arm  imu->cam0 {x,y,z,roll,pitch,yaw}

Publishes
---------
/tf    (multiple TransformStamped) — marker_map -> *_imu at odom rate
/tf    (once on startup)           — *_imu -> *_cam0 static edge
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert roll-pitch-yaw (ZYX / Rz*Ry*Rx) to unit quaternion (x,y,z,w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def _make_transform(
    parent: str, child: str,
    x: float, y: float, z: float,
    qx: float, qy: float, qz: float, qw: float,
    stamp,
) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw
    return t


def _extract_quaternion_from_odom(msg: Odometry) -> tuple[float, float, float, float]:
    q = msg.pose.pose.orientation
    return (q.x, q.y, q.z, q.w)


def _extract_translation_from_odom(msg: Odometry) -> tuple[float, float, float]:
    p = msg.pose.pose.position
    return (p.x, p.y, p.z)


class OpenVINSOdomTFRelay(Node):
    """Relay OpenVINS odometry poses to local TF transforms."""

    def __init__(self):
        super().__init__("openvins_odom_tf_relay")

        # ── Declare parameters ────────────────────────────────────────────
        self.declare_parameter("head_odom_topic", "/ov_msckf/odomimu")
        self.declare_parameter("arm_odom_topic", "/ov_msckf_arm/odomimu")
        self.declare_parameter("target_frame", "marker_map")
        self.declare_parameter("head_imu_frame", "head_imu")
        self.declare_parameter("arm_imu_frame", "arm_imu")
        self.declare_parameter("head_cam_frame", "head_cam0")
        self.declare_parameter("arm_cam_frame", "arm_cam0")
        self.declare_parameter("publish_imu_to_cam_tf", True)
        self.declare_parameter(
            "imu_to_cam_extrinsics_head",
            {"x": 0.0, "y": 0.015, "z": 0.0,
             "roll": -1.57079632679, "pitch": 0.0, "yaw": -1.57079632679},
        )
        self.declare_parameter(
            "imu_to_cam_extrinsics_arm",
            {"x": 0.0, "y": 0.015, "z": 0.0,
             "roll": -1.57079632679, "pitch": 0.0, "yaw": -1.57079632679},
        )

        # ── Read parameters ───────────────────────────────────────────────
        head_odom_topic = self.get_parameter("head_odom_topic").value
        arm_odom_topic = self.get_parameter("arm_odom_topic").value
        self._target_frame = self.get_parameter("target_frame").value
        self._head_imu = self.get_parameter("head_imu_frame").value
        self._arm_imu = self.get_parameter("arm_imu_frame").value
        self._head_cam = self.get_parameter("head_cam_frame").value
        self._arm_cam = self.get_parameter("arm_cam_frame").value
        publish_imu_to_cam = self.get_parameter("publish_imu_to_cam_tf").value
        head_extrinsics = self.get_parameter("imu_to_cam_extrinsics_head").value
        arm_extrinsics = self.get_parameter("imu_to_cam_extrinsics_arm").value

        # ── QoS — OpenVINS publishes odom with BEST_EFFORT ────────────────
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Odometry, head_odom_topic, self._on_head_odom, best_effort)
        self.create_subscription(
            Odometry, arm_odom_topic, self._on_arm_odom, best_effort)

        # ── Counters for throttled logging ─────────────────────────────────
        self._head_count = 0
        self._arm_count = 0

        # ── Publish static *_imu -> *_cam0 edges ──────────────────────────
        if publish_imu_to_cam:
            static_tfs: list[TransformStamped] = []
            stamp = self.get_clock().now().to_msg()

            for imu_frame, cam_frame, extrinsics, name in [
                (self._head_imu, self._head_cam, head_extrinsics, "head"),
                (self._arm_imu, self._arm_cam, arm_extrinsics, "arm"),
            ]:
                qx, qy, qz, qw = _quaternion_from_rpy(
                    extrinsics["roll"], extrinsics["pitch"], extrinsics["yaw"])
                static_tfs.append(_make_transform(
                    imu_frame, cam_frame,
                    extrinsics["x"], extrinsics["y"], extrinsics["z"],
                    qx, qy, qz, qw, stamp,
                ))
                self.get_logger().info(
                    f"{name}: publishing static {imu_frame} -> {cam_frame} "
                    f"from nominal extrinsics "
                    f"(t=({extrinsics['x']:.3f}, {extrinsics['y']:.3f}, "
                    f"{extrinsics['z']:.3f}), "
                    f"rpy=({extrinsics['roll']:.3f}, {extrinsics['pitch']:.3f}, "
                    f"{extrinsics['yaw']:.3f}))"
                )

            if static_tfs:
                self._static_tf_broadcaster.sendTransform(static_tfs)

        self.get_logger().info(
            f"OpenVINS odometry TF relay active: "
            f"{self._target_frame} -> {self._head_imu} "
            f"(via {head_odom_topic}), "
            f"{self._target_frame} -> {self._arm_imu} "
            f"(via {arm_odom_topic})"
        )

    def _on_head_odom(self, msg: Odometry):
        self._on_odom(msg, self._target_frame, self._head_imu, "head")

    def _on_arm_odom(self, msg: Odometry):
        self._on_odom(msg, self._target_frame, self._arm_imu, "arm")

    def _on_odom(self, msg: Odometry, parent: str, child: str, name: str):
        """Extract pose from odom and publish as TF with host clock stamp."""
        x, y, z = _extract_translation_from_odom(msg)
        qx, qy, qz, qw = _extract_quaternion_from_odom(msg)

        # Use host clock stamp — avoids extrapolation-into-the-past errors
        # that occur when the Jetson clock and host clock differ.
        tf_msg = _make_transform(
            parent, child, x, y, z, qx, qy, qz, qw,
            self.get_clock().now().to_msg(),
        )
        self._tf_broadcaster.sendTransform(tf_msg)

        # Throttled log every 100th message
        count = getattr(self, f"_{name}_count")
        count += 1
        setattr(self, f"_{name}_count", count)
        if count % 100 == 1:
            self.get_logger().info(
                f"Relay #{count} ({name}): "
                f"{parent} -> {child} "
                f"t=({x:.3f}, {y:.3f}, {z:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = OpenVINSOdomTFRelay()
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