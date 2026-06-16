#!/usr/bin/env python3
"""Relay OpenVINS odometry to TF transforms on the host.

Publishes the dynamic ``marker_map -> *_imu`` TF edges from OpenVINS odometry
messages, and optionally the static ``*_imu -> *_cam0`` extrinsic.  This makes
the host self-sufficient for the TF chain that the pointcloud fusion node
needs, bypassing unreliable DDS ``/tf`` delivery from the Jetson.

The odometry messages arrive at ~200 Hz on separate topics (one per camera).
Each message carries ``T(marker_map -> imu)`` in its pose field.  We extract
that pose and publish it as a local TF transform stamped with the **host
clock** so that TF timestamps share the same time domain as the other
host-side TF publishers (bridge, camera mounts).  This ensures the
multi-edge TF chain composes correctly for bbox removal lookups.

Parameters
----------
head_odom_topic          str   input Odometry topic for head camera
                                (default: /jetson/head/odom)
arm_odom_topic           str   input Odometry topic for arm camera
                                (default: /jetson/arm/odom)
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
from tf2_ros import Buffer, TransformBroadcaster, StaticTransformBroadcaster, TransformListener


# Minimum covariance trace required to consider OpenVINS initialized.
# Uninitialized OpenVINS publishes odometry with all-zero covariance.
# A trace > 0 means the EKF has produced at least one update.
_MIN_COV_TRACE = 1e-12

# If the init guard hasn't passed after this many seconds, force-publish
# anyway.  This prevents the relay from being permanently blocked if
# OpenVINS never publishes non-zero covariance.
_INIT_GUARD_FALLBACK_TIMEOUT_S = 30.0

# Minimum position norm (meters) to consider the pose non-trivial.
# Identity pose is near origin; a converged VIO will have moved.
_MIN_POSE_NORM_M = 0.001

# Maximum translation magnitude (m) for the imu->cam0 extrinsic.
# If self-calibration produces a translation larger than this, it is
# rejected and the hardcoded fallback is kept.
_MAX_VALID_EXTRINSIC_TRANSLATION_M = 0.20


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
        self.declare_parameter("head_odom_topic", "/jetson/head/odom")
        self.declare_parameter("arm_odom_topic", "/jetson/arm/odom")
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
        #
        # These are FALLBACK values used when self_calibrate_extrinsics is
        # False or when self-calibration fails.  When self-calibration
        # succeeds, the computed live values override these.
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
        # ── Self-calibration parameters ───────────────────────────────────
        self.declare_parameter("self_calibrate_extrinsics", False)
        self.declare_parameter("self_calibration_max_retries", 100)
        # ── Future Jetson-side extrinsics topic ───────────────────────────
        self.declare_parameter("extrinsics_topic", "")
        self.declare_parameter("max_pose_norm_m", 2.0)
        self.declare_parameter("max_pose_jump_m", 0.50)
        # When True (default), the relay broadcasts the dynamic
        # marker_map -> *_imu TF edges from raw VIO.  When False, only the
        # static *_imu -> *_cam0 edges are published and the dynamic edges
        # are left to another publisher (e.g. the GTSAM tracker, which
        # broadcasts the smoothed trajectory).  Set this to False when
        # running the V6 GTSAM tracker with broadcast_tf=true so the two
        # nodes don't fight over the same TF edges (V6 §6.3).
        self.declare_parameter("publish_dynamic_tf", True)

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

        # ── Self-calibration config ──────────────────────────────────────
        self._self_calibrate = self.get_parameter(
            "self_calibrate_extrinsics").value
        self._self_calibration_max_retries = self.get_parameter(
            "self_calibration_max_retries").value
        self._extrinsics_topic = self.get_parameter("extrinsics_topic").value
        self._max_pose_norm_m = self.get_parameter("max_pose_norm_m").value
        self._max_pose_jump_m = self.get_parameter("max_pose_jump_m").value
        self._publish_dynamic_tf = self.get_parameter("publish_dynamic_tf").value

        # ── TF2 buffer for self-calibration lookups ───────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

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

        # Optional extrinsics topic for future Jetson-side support.
        if self._extrinsics_topic:
            from std_msgs.msg import Float64MultiArray
            self.create_subscription(
                Float64MultiArray,
                self._extrinsics_topic,
                self._on_extrinsics,
                10,
            )
            self.get_logger().info(
                f"Subscribing to extrinsics topic: {self._extrinsics_topic}"
            )

        # -- Counters for throttled logging ---------------------------------
        self._head_count = 0
        self._arm_count = 0

        # -- Startup timestamp for fallback timeout --------------------------
        self._startup_time = self.get_clock().now()

        # -- Periodic status timer (visible at warn level) --------------------
        self._status_timer = self.create_timer(10.0, self._status_tick)

        # -- Per-camera initialization tracking --------------------------------
        self._head_initialized = False
        self._arm_initialized = False

        # -- Per-camera self-calibration state ---------------------------------
        self._head_self_calibrated = False
        self._arm_self_calibrated = False
        self._head_self_calibration_attempts = 0
        self._arm_self_calibration_attempts = 0
        # Track the actual IMU frame name from the odom message for
        # self-calibration lookups through the RealSense TF tree.
        self._head_actual_imu_frame: str | None = None
        self._arm_actual_imu_frame: str | None = None
        self._last_head_pos: tuple[float, float, float] | None = None
        self._last_arm_pos: tuple[float, float, float] | None = None

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
        self.get_logger().warn(
            f"OpenVINS odometry TF relay active: "
            f"{self._target_frame} -> {self._head_imu} "
            f"(via {head_odom_topic}), "
            f"{self._target_frame} -> {self._arm_imu} "
            f"(via {arm_odom_topic}), "
            f"publish_dynamic_tf={self._publish_dynamic_tf}"
        )

    def _on_head_odom(self, msg: Odometry):
        self._on_odom(msg, self._target_frame, self._head_imu,
                      self._head_cam, "head",
                      "_head_self_calibrated",
                      "_head_actual_imu_frame")

    def _on_arm_odom(self, msg: Odometry):
        self._on_odom(msg, self._target_frame, self._arm_imu,
                      self._arm_cam, "arm",
                      "_arm_self_calibrated",
                      "_arm_actual_imu_frame")

    def _on_odom(self, msg: Odometry, parent: str, child: str,
                 cam_frame: str, name: str,
                 calibrated_attr: str, actual_imu_attr: str):
        """Extract pose from odom and publish as TF with odom's timestamp.

        Skips messages from uninitialized OpenVINS (zero covariance or
        non-finite pose).  This prevents garbage TFs from flooding the
        system during the first 40-80 seconds while VIO converges.

        After ``_INIT_GUARD_FALLBACK_TIMEOUT_S`` seconds, the guard is
        bypassed and messages are published regardless of covariance, to
        prevent the relay from being permanently blocked.
        """
        x, y, z = _extract_translation_from_odom(msg)
        qx, qy, qz, qw = _extract_quaternion_from_odom(msg)

        # ── Initialization guard ─────────────────────────────────────────
        # Uninitialized OpenVINS publishes identity pose with zero
        # covariance.  Wait until the EKF has actually converged before
        # publishing TFs to avoid polluting the TF buffer.
        init_attr = f"_{name}_initialized"
        if not getattr(self, init_attr):
            # Check 1: all pose values must be finite.
            if not all(math.isfinite(v) for v in [x, y, z, qx, qy, qz, qw]):
                return

            # Check 2: covariance trace must be > 0 (EKF has updated).
            cov = msg.pose.covariance
            cov_trace = sum(c * c for c in cov)
            pos_norm = math.sqrt(x * x + y * y + z * z)

            # Check 3: fallback timeout — if we've waited long enough,
            # accept the message if the pose is non-trivial (not identity).
            elapsed = (self.get_clock().now() - self._startup_time).nanoseconds / 1e9
            fallback_triggered = (
                elapsed >= _INIT_GUARD_FALLBACK_TIMEOUT_S
                and pos_norm > _MIN_POSE_NORM_M
            )

            if cov_trace < _MIN_COV_TRACE and not fallback_triggered:
                return  # still waiting for convergence

            # First valid message — mark initialized and log.
            setattr(self, init_attr, True)
            # Capture the actual IMU frame name from the odom message.
            actual_imu = msg.child_frame_id
            setattr(self, actual_imu_attr, actual_imu)
            reason = "covariance" if cov_trace >= _MIN_COV_TRACE else "fallback-timeout"
            # Use warn level so this is visible even when the relay
            # is launched with --log-level warn.
            self.get_logger().warn(
                f"{name}: OpenVINS initialized ({reason}), publishing TF "
                f"{parent} -> {child} "
                f"(p=({x:.3f}, {y:.3f}, {z:.3f}), "
                f"cov_trace={cov_trace:.2e}, "
                f"pos_norm={pos_norm:.3f}m, "
                f"elapsed={elapsed:.1f}s, "
                f"actual_imu_frame={actual_imu!r})"
            )

        # ── Post-initialization outlier suppression ────────────────────────
        pos_norm = math.sqrt(x * x + y * y + z * z)
        if pos_norm > self._max_pose_norm_m:
            self.get_logger().warn(
                f'{name} odom pose norm {pos_norm:.2f}m exceeds max '
                f'{self._max_pose_norm_m}m — suppressing TF',
                throttle_duration_sec=2.0)
            return

        last_pos = self._last_head_pos if name == 'head' else self._last_arm_pos
        if last_pos is not None:
            jump = math.sqrt(
                (x - last_pos[0]) ** 2
                + (y - last_pos[1]) ** 2
                + (z - last_pos[2]) ** 2
            )
            if jump > self._max_pose_jump_m:
                self.get_logger().warn(
                    f'{name} odom pose jumped {jump:.3f}m '
                    f'(> {self._max_pose_jump_m}m) — suppressing TF',
                    throttle_duration_sec=2.0)
                return

        # Update last position
        if name == 'head':
            self._last_head_pos = (x, y, z)
        else:
            self._last_arm_pos = (x, y, z)

        # ── Dynamic TF broadcast ──────────────────────────────────────────
        # When ``publish_dynamic_tf`` is False (e.g. when the V6 GTSAM tracker
        # owns the dynamic TF tree via broadcast_tf=true), skip broadcasting
        # the marker_map -> *_imu edge.  The relay still runs its init guard,
        # outlier suppression, and self-calibration so that the odometry is
        # validated and the static *_imu -> *_cam0 extrinsics are maintained,
        # but the smoothed trajectory from GTSAM is what feeds downstream TF
        # consumers (V6 §6.3).
        if not self._publish_dynamic_tf:
            # Throttled log so the operator knows the relay is alive but
            # deferring to GTSAM for the dynamic TF.
            count_attr = f"_{name}_count"
            count = getattr(self, count_attr)
            count += 1
            setattr(self, count_attr, count)
            if count % 200 == 1:
                self.get_logger().info(
                    f"Relay #{count} ({name}): publish_dynamic_tf=False — "
                    f"deferring {parent} -> {child} to GTSAM tracker"
                )
        else:
            # Stamp with the host clock so all edges in the TF chain
            # (relay, bridge, camera mounts) share the same time domain.
            # Using the Jetson odom timestamp created a ~10s clock gap that
            # prevented TF2 from composing the multi-edge chain for bbox
            # removal lookups ("extrapolation into the past" errors).
            host_stamp = self.get_clock().now().to_msg()
            tf_msg = _make_transform(
                parent, child, x, y, z, qx, qy, qz, qw,
                host_stamp,
            )
            self._tf_broadcaster.sendTransform(tf_msg)

        # ── Self-calibration attempt (once per camera) ────────────────────
        if (self._self_calibrate and not getattr(self, calibrated_attr)
                and getattr(self, init_attr)):
            self._try_self_calibrate(name, child, cam_frame,
                                     calibrated_attr, actual_imu_attr)

        # Throttled log every 100th message
        count_attr = f"_{name}_count"
        count = getattr(self, count_attr)
        count += 1
        setattr(self, count_attr, count)
        if count % 100 == 1:
            self.get_logger().info(
                f"Relay #{count} ({name}): "
                f"{parent} -> {child} "
                f"t=({x:.3f}, {y:.3f}, {z:.3f})"
            )

    # ── Periodic status (visible at warn level) ───────────────────────────

    def _status_tick(self):
        """Log a one-line status every 10 seconds, visible at warn level."""
        elapsed = (self.get_clock().now() - self._startup_time).nanoseconds / 1e9
        head_status = "OK" if self._head_initialized else "WAITING"
        arm_status = "OK" if self._arm_initialized else "WAITING"
        if self._head_initialized and self._arm_initialized:
            # Both initialized — stop the status timer.
            self.destroy_timer(self._status_timer)
            return
        self.get_logger().warn(
            f"[RELAY-STATUS] {elapsed:.0f}s elapsed — "
            f"head: {head_status} ({self._head_count} msgs), "
            f"arm: {arm_status} ({self._arm_count} msgs)"
        )

    # ── Self-calibration ──────────────────────────────────────────────────

    def _try_self_calibrate(self, name: str, imu_frame: str, cam_frame: str,
                            calibrated_attr: str, actual_imu_attr: str):
        """Attempt to compute live imu->cam0 extrinsic from the TF tree.

        Uses the actual IMU frame name (captured from the first valid odom
        message's child_frame_id) to look up T(imu, depth_optical) through
        the RealSense static chain.  This path does NOT go through the
        hardcoded static TF, so the result reflects the true live extrinsic.
        """
        actual_imu = getattr(self, actual_imu_attr, None)
        if not actual_imu:
            return

        count_attr = f"_{name}_self_calibration_attempts"
        attempts = getattr(self, count_attr)
        attempts += 1
        setattr(self, count_attr, attempts)

        # Only attempt on the first message, then every 10th, up to max.
        if attempts > self._self_calibration_max_retries:
            if attempts == self._self_calibration_max_retries + 1:
                self.get_logger().warn(
                    f"{name}: self-calibration exceeded max retries "
                    f"({self._self_calibration_max_retries}) — "
                    f"using hardcoded extrinsics"
                )
            setattr(self, calibrated_attr, True)
            return
        if attempts > 1 and attempts % 10 != 0:
            return

        # Derive the depth optical frame name from the actual IMU frame.
        # Typical RealSense naming: *_imu_optical_frame -> *_depth_optical_frame
        depth_optical = self._derive_depth_optical_frame(actual_imu)
        if depth_optical is None:
            self.get_logger().debug(
                f"{name}: cannot derive depth optical frame from "
                f"actual IMU frame {actual_imu!r} — skipping self-calibration"
            )
            setattr(self, calibrated_attr, True)
            return

        color_optical = self._derive_color_optical_frame(actual_imu)

        # Look up T(actual_imu_frame, depth_optical_frame) through the
        # RealSense static chain (not through our hardcoded imu->cam0).
        try:
            t_imu_depth = self._tf_buffer.lookup_transform(
                actual_imu, depth_optical, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as e:
            self.get_logger().debug(
                f"{name}: self-calibration lookup "
                f"{actual_imu!r} -> {depth_optical!r} failed: {e}"
            )
            return

        # Look up T(depth_optical, color_optical) — known RealSense extrinsic.
        T_imu_cam0 = self._extract_transform_matrix(t_imu_depth)
        if color_optical and color_optical != depth_optical:
            try:
                t_depth_color = self._tf_buffer.lookup_transform(
                    depth_optical, color_optical, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                T_depth_color = self._extract_transform_matrix(t_depth_color)
                T_imu_cam0 = T_imu_cam0 @ T_depth_color
            except Exception:
                # color_optical may not be available; use imu->depth directly
                # (depth and color have the same origin, only a small baseline).
                pass

        # ── Sanity check: reject translations > 20cm ────────────────────
        tx, ty, tz = T_imu_cam0[0, 3], T_imu_cam0[1, 3], T_imu_cam0[2, 3]
        trans_norm = math.sqrt(tx * tx + ty * ty + tz * tz)
        if trans_norm > _MAX_VALID_EXTRINSIC_TRANSLATION_M:
            self.get_logger().warn(
                f"{name}: self-calibration produced implausible translation "
                f"({trans_norm:.3f}m > {_MAX_VALID_EXTRINSIC_TRANSLATION_M}m) "
                f"— rejecting, using hardcoded extrinsics"
            )
            setattr(self, calibrated_attr, True)
            return

        # ── Extract rotation as quaternion ──────────────────────────────
        q = self._matrix_to_quaternion(T_imu_cam0[:3, :3])

        # ── Update the static TF for this camera ─────────────────────────
        self._update_imu_to_cam_tf(name, imu_frame, cam_frame,
                                    tx, ty, tz, q[0], q[1], q[2], q[3])
        setattr(self, calibrated_attr, True)

        # ── Log the result ──────────────────────────────────────────────
        self.get_logger().info(
            f"{name}: self-calibrated extrinsics "
            f"{imu_frame} -> {cam_frame} "
            f"t=({tx:.4f}, {ty:.4f}, {tz:.4f}) "
            f"q=({q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f}) "
            f"(trans_norm={trans_norm:.4f}m, attempts={attempts})"
        )

    @staticmethod
    def _derive_depth_optical_frame(imu_frame: str) -> str | None:
        """Derive the depth optical frame name from an IMU frame name.

        Typical RealSense conventions:
          head_d435i_head_imu_optical_frame -> head_d435i_head_depth_optical_frame
        """
        for suffix in ("_imu_optical_frame", "_accel_optical_frame",
                        "_gyro_optical_frame", "_imu_frame",
                        "_accel_frame", "_gyro_frame"):
            if imu_frame.endswith(suffix):
                prefix = imu_frame[: -len(suffix)]
                return prefix + "_depth_optical_frame"
        return None

    @staticmethod
    def _derive_color_optical_frame(imu_frame: str) -> str | None:
        """Derive the color optical frame name from an IMU frame name."""
        for suffix in ("_imu_optical_frame", "_accel_optical_frame",
                        "_gyro_optical_frame", "_imu_frame",
                        "_accel_frame", "_gyro_frame"):
            if imu_frame.endswith(suffix):
                prefix = imu_frame[: -len(suffix)]
                return prefix + "_color_optical_frame"
        return None

    @staticmethod
    def _extract_transform_matrix(t: TransformStamped):
        """Extract a 4x4 homogeneous transform matrix from a TransformStamped."""
        import numpy as np
        q = t.transform.rotation
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z

        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        r00 = 1 - 2 * (qy * qy + qz * qz)
        r01 = 2 * (qx * qy - qz * qw)
        r02 = 2 * (qx * qz + qy * qw)
        r10 = 2 * (qx * qy + qz * qw)
        r11 = 1 - 2 * (qx * qx + qz * qz)
        r12 = 2 * (qy * qz - qx * qw)
        r20 = 2 * (qx * qz - qy * qw)
        r21 = 2 * (qy * qz + qx * qw)
        r22 = 1 - 2 * (qx * qx + qy * qy)

        return np.array([
            [r00, r01, r02, tx],
            [r10, r11, r12, ty],
            [r20, r21, r22, tz],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=np.float64)

    @staticmethod
    def _matrix_to_quaternion(R):
        """Convert a 3x3 rotation matrix to quaternion (x, y, z, w)."""
        import numpy as np
        trace = float(np.trace(R))
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0.0:
            return (0.0, 0.0, 0.0, 1.0)
        return (qx / norm, qy / norm, qz / norm, qw / norm)

    def _update_imu_to_cam_tf(self, name: str, imu_frame: str, cam_frame: str,
                               x: float, y: float, z: float,
                               qx: float, qy: float, qz: float, qw: float):
        """Update the stored imu->cam0 static TF with live values.

        Re-publishes on /tf_static (persistent) and /tf (for late joiners).
        Also updates the liveness timer's stored list.
        """
        stamp = self.get_clock().now().to_msg()
        new_tf = _make_transform(
            imu_frame, cam_frame, x, y, z, qx, qy, qz, qw, stamp,
        )

        # Replace the old entry for this camera in _imu_to_cam_tfs.
        updated = False
        for i, tf_msg in enumerate(self._imu_to_cam_tfs):
            if tf_msg.header.frame_id == imu_frame and tf_msg.child_frame_id == cam_frame:
                self._imu_to_cam_tfs[i] = new_tf
                updated = True
                break
        if not updated:
            self._imu_to_cam_tfs.append(new_tf)

        # Re-publish static (persistent) and dynamic (for late joiners).
        self._static_tf_broadcaster.sendTransform(new_tf)
        self._tf_broadcaster.sendTransform(new_tf)

    # ── Extrinsics topic callback (future Jetson-side support) ────────────

    def _on_extrinsics(self, msg):
        """Handle incoming extrinsics from a custom topic.

        Expected format: Float64MultiArray with layout.dim[0].label = "head" or
        "arm", and data = [x, y, z, qx, qy, qz, qw] (imu->cam0).
        """
        if len(msg.data) != 7:
            self.get_logger().warn(
                f"extrinsics message has {len(msg.data)} values, expected 7"
            )
            return

        # Determine which camera this is for.
        label = ""
        if msg.layout.dim and len(msg.layout.dim) > 0:
            label = msg.layout.dim[0].label.lower()
        if not label:
            self.get_logger().warn(
                "extrinsics message has no label — cannot determine camera"
            )
            return

        if "head" in label:
            name, imu_frame, cam_frame = "head", self._head_imu, self._head_cam
        elif "arm" in label:
            name, imu_frame, cam_frame = "arm", self._arm_imu, self._arm_cam
        else:
            self.get_logger().warn(
                f"extrinsics message label {label!r} not recognised"
            )
            return

        x, y, z, qx, qy, qz, qw = msg.data
        self._update_imu_to_cam_tf(name, imu_frame, cam_frame,
                                    x, y, z, qx, qy, qz, qw)
        self.get_logger().info(
            f"{name}: received live extrinsics via topic "
            f"{imu_frame} -> {cam_frame} "
            f"t=({x:.4f}, {y:.4f}, {z:.4f}) "
            f"q=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})"
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