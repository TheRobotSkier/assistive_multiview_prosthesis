#!/usr/bin/env python3
"""Relay OpenVINS odometry to TF transforms on the host.

Publishes the dynamic ``marker_map -> *_imu`` TF edges from OpenVINS odometry
messages, and optionally the static ``*_imu -> *_cam0`` extrinsic.  This makes
the host self-sufficient for the TF chain that the pointcloud fusion node
needs, bypassing unreliable DDS ``/tf`` delivery from the Jetson.

The odometry messages arrive at ~200 Hz on separate topics (one per camera).
Each message carries ``T(marker_map -> imu)`` in its pose field.  We extract
that pose and publish it as a local TF transform stamped with the **odom
message timestamp** so that TF timestamps share the same time domain as the
point clouds arriving from the Jetson.

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
imu_to_cam_{x,y,z,roll,pitch,yaw}_head  double  nominal head imu->cam0 extrinsic components
imu_to_cam_{x,y,z,roll,pitch,yaw}_arm   double  nominal arm  imu->cam0 extrinsic components

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
        # Head IMU->cam0 extrinsic from OpenVINS live output (v12 log).
        # OpenVINS' IMU frame is already in optical convention, so T_cam_imu
        # is near-identity rotation with a small translation.
        #   head (marker-6): T_cam_imu t=[0.015,0.011, -0.066]
        # T_imu_cam = inv(T_cam_imu) ≈ (-0.015, -0.011,0.066)
        self.declare_parameter("imu_to_cam_x_head", -0.015)
        self.declare_parameter("imu_to_cam_y_head", -0.011)
        self.declare_parameter("imu_to_cam_z_head",0.066)
        self.declare_parameter("imu_to_cam_roll_head",0.0)
        self.declare_parameter("imu_to_cam_pitch_head",0.0)
        self.declare_parameter("imu_to_cam_yaw_head",0.0)
        # Arm IMU->cam0 extrinsic from OpenVINS live output (v12 log).
        #   arm (marker-7): T_cam_imu t=[0.002,0.003,0.007]
        # T_imu_cam = inv(T_cam_imu) ≈ (-0.002, -0.003, -0.007)
        self.declare_parameter("imu_to_cam_x_arm", -0.002)
        self.declare_parameter("imu_to_cam_y_arm", -0.003)
        self.declare_parameter("imu_to_cam_z_arm", -0.007)
        self.declare_parameter("imu_to_cam_roll_arm",0.0)
        self.declare_parameter("imu_to_cam_pitch_arm",0.0)
        self.declare_parameter("imu_to_cam_yaw_arm",0.0)

        # ── Read parameters ───────────────────────────────────────────────
        head_odom_topic = self.get_parameter("head_odom_topic").value
        arm_odom_topic = self.get_parameter("arm_odom_topic").value
        self._target_frame = self.get_parameter("target_frame").value
        self._head_imu = self.get_parameter("head_imu_frame").value
        self._arm_imu = self.get_parameter("arm_imu_frame").value
        self._head_cam = self.get_parameter("head_cam_frame").value
        self._arm_cam = self.get_parameter("arm_cam_frame").value
        publish_imu_to_cam = self.get_parameter("publish_imu_to_cam_tf").value
        head_extrinsics = {
            "x": self.get_parameter("imu_to_cam_x_head").value,
            "y": self.get_parameter("imu_to_cam_y_head").value,
            "z": self.get_parameter("imu_to_cam_z_head").value,
            "roll": self.get_parameter("imu_to_cam_roll_head").value,
            "pitch": self.get_parameter("imu_to_cam_pitch_head").value,
            "yaw": self.get_parameter("imu_to_cam_yaw_head").value,
        }
        arm_extrinsics = {
            "x": self.get_parameter("imu_to_cam_x_arm").value,
            "y": self.get_parameter("imu_to_cam_y_arm").value,
            "z": self.get_parameter("imu_to_cam_z_arm").value,
            "roll": self.get_parameter("imu_to_cam_roll_arm").value,
            "pitch": self.get_parameter("imu_to_cam_pitch_arm").value,
            "yaw": self.get_parameter("imu_to_cam_yaw_arm").value,
        }

        # ── QoS — OpenVINS publishes odom with BEST_EFFORT ────────────────
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ────────────────────────────────────────────────────
        # -- Publishers ---------------------------------------------------
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        # -- Subscribers ---------------------------------------------------
        self.create_subscription(
            Odometry, head_odom_topic, self._on_head_odom, best_effort)
        self.create_subscription(
            Odometry, arm_odom_topic, self._on_arm_odom, best_effort)

        # -- Counters for throttled logging ---------------------------------
        self._head_count = 0
        self._arm_count = 0

        # -- Publish static *_imu -> *_cam0 edges --------------------------
        # Stored for liveness re-send on /tf, matching the same pattern
        # used by openvins_realsense_tf_bridge_node for DDS reliability.
        self._imu_to_cam_tfs: list[TransformStamped] = []
        if publish_imu_to_cam:
            stamp = self.get_clock().now().to_msg()

            for imu_frame, cam_frame, extrinsics, name in [
                (self._head_imu, self._head_cam, head_extrinsics, "head"),
                (self._arm_imu, self._arm_cam, arm_extrinsics, "arm"),
            ]:
                qx, qy, qz, qw = _quaternion_from_rpy(
                    extrinsics["roll"], extrinsics["pitch"], extrinsics["yaw"])
                tf_msg = _make_transform(
                    imu_frame, cam_frame,
                    extrinsics["x"], extrinsics["y"], extrinsics["z"],
                    qx, qy, qz, qw, stamp,
                )
                self._imu_to_cam_tfs.append(tf_msg)
                self.get_logger().info(
                    f"{name}: publishing static {imu_frame} -> {cam_frame} "
                    f"from nominal extrinsics "
                    f"(t=({extrinsics['x']:.3f}, {extrinsics['y']:.3f}, "
                    f"{extrinsics['z']:.3f}), "
                    f"rpy=({extrinsics['roll']:.3f}, {extrinsics['pitch']:.3f}, "
                    f"{extrinsics['yaw']:.3f}))"
                )

            if self._imu_to_cam_tfs:
                # Publish on /tf_static for persistent storage.
                self._static_tf_broadcaster.sendTransform(self._imu_to_cam_tfs)
                # Also on /tf immediately so late-joining nodes don't miss it
                # (CycloneDDS TRANSIENT_LOCAL latch is unreliable).
                self._tf_broadcaster.sendTransform(self._imu_to_cam_tfs)
                # Liveness timer: re-send on /tf at 2 Hz so nodes that join
                # after startup can still pick up the chain.
                self._liveness_timer = self.create_timer(0.5, self._liveness_tick)
            else:
                self._liveness_timer = None
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
        """Extract pose from odom and publish as TF with odom's timestamp."""
        x, y, z = _extract_translation_from_odom(msg)
        qx, qy, qz, qw = _extract_quaternion_from_odom(msg)

        # Use the odom message's timestamp so TF stamps share the same
        # time domain as the point clouds from the Jetson.  The fusion
        # node re-stamps the output cloud with the host clock, so
        # downstream consumers are unaffected.
        tf_msg = _make_transform(
            parent, child, x, y, z, qx, qy, qz, qw,
            msg.header.stamp,
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

    def _liveness_tick(self):
        """Re-send imu->cam0 static transforms on /tf periodically.

        This is a workaround for CycloneDDS /tf_static latch unreliability,
        matching the pattern used by openvins_realsense_tf_bridge_node.
        """
        if not self._imu_to_cam_tfs or self._liveness_timer is None:
            return
        stamp = self.get_clock().now().to_msg()
        for tf_msg in self._imu_to_cam_tfs:
            tf_msg.header.stamp = stamp
        self._tf_broadcaster.sendTransform(self._imu_to_cam_tfs)


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