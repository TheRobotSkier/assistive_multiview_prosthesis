#!/usr/bin/env python3
"""gtsam_tracker_node — ROS 2 wrapper for the GTSAM trajectory factor graph.

Subscribes to head + arm OpenVINS odometry, ArUco marker observations, and
cross-camera visual factors, feeds them into a :class:`TrajectoryFactorGraph`,
and publishes optimised head/arm poses at a fixed rate.

Graceful degradation (V6 §9):
- If ArUco topics are absent, the graph runs on odometry + visual factors only.
- If ``head_pose_source == "tf"``, head pose is read from TF instead of odom.

Reference: V6 plan §6.3, §6.9.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np

# Pure-logic helpers (ROS-free, done in Phase 1).
from gtsam_tracker.factor_graph import (
    TrajectoryFactorGraph,
    noise_from_covariance_diag,
    default_odom_noise,
)
from gtsam_tracker.se3_helpers import (
    pose_to_matrix,
    matrix_to_pose,
    inverse_se3,
    relative_transform,
)


__all__ = ["create_node", "main", "DEFAULT_PARAMS"]


# ---------------------------------------------------------------------------
# Default parameters (V6 §6.9)
# ---------------------------------------------------------------------------

DEFAULT_PARAMS = {
    # Odometry topics
    "head_odom_topic": "/jetson/head/odom",
    "arm_odom_topic": "/jetson/arm/odom",
    # Head pose source: "odom" or "tf"
    "head_pose_source": "odom",
    # TF frames for the TF fallback
    "head_tf_target_frame": "marker_map",
    "head_tf_source_frame": "head_imu",
    # ArUco topics
    "aruco_marker_topic": "/aruco/marker_pose",
    "aruco_dynamic_topic": "/aruco/dynamic_marker",
    "aruco_arm_pose_topic": "/aruco/arm_pose",
    # Visual factor topic (from cross_camera_features)
    "visual_factor_topic": "/vis/head_arm_pose",
    # Graph parameters
    "graph_rate_hz": 15.0,
    "smoother_lag_s": 15.0,
    # Kinematic range factor
    "kinematic_range_m": 1.0,
    "kinematic_range_sigma_m": 0.05,
    "kinematic_check_interval": 10,
    # Noise sigmas
    "odom_translation_sigma_m": 0.02,
    "odom_rotation_sigma_rad": 0.02,
    # Sanity gate: reject odometry deltas larger than this (metres) to protect
    # the factor graph from diverging VIO (V6 §9 fault tolerance).
    "max_odom_delta_m": 0.5,
    # Output topics
    "head_pose_topic": "/gtsam/head_pose",
    "arm_pose_topic": "/gtsam/arm_pose",
    # Output frame
    "world_frame": "marker_map",
    # TF broadcasting: when enabled, the optimised head/arm poses are
    # broadcast as dynamic TF edges (world_frame -> head_child_frame /
    # arm_child_frame) so downstream nodes (pointcloud_fusion, rviz) consume
    # the smoothed trajectory instead of the raw, diverging VIO relay.
    # See V6 §6.3 — GTSAM owns the dynamic TF tree when active.
    "broadcast_tf": True,
    "head_child_frame": "head_imu",
    "arm_child_frame": "arm_imu",
}


# ---------------------------------------------------------------------------
# Odometry → SE(3) helpers
# ---------------------------------------------------------------------------

def _odom_to_matrix(msg) -> np.ndarray:
    """Convert a ``nav_msgs/Odometry`` pose to a ``(4, 4)`` matrix."""
    return pose_to_matrix(msg.pose.pose)


def _odom_covariance_diag(msg, sigma_t: float, sigma_r: float) -> list:
    """Extract a 6-element covariance diagonal from an odom message.

    Falls back to fixed sigmas if the covariance is zero/invalid.
    """
    cov = list(msg.pose.covariance)  # 36 elements, row-major 6×6
    diag = [cov[0], cov[7], cov[14], cov[21], cov[28], cov[35]]
    if all(d <= 1e-12 for d in diag):
        # No covariance provided — use defaults.
        return [sigma_t ** 2] * 3 + [sigma_r ** 2] * 3
    return diag


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _float_to_stamp(stamp_f: float):
    """Convert a float timestamp (seconds) back to a ``builtin_interfaces/Time``.

    Inverse of :func:`_stamp_to_float`.  Used to stamp published poses with
    the sensor time so downstream consumers stay temporally aligned.
    """
    from builtin_interfaces.msg import Time
    sec = int(stamp_f)
    nanosec = int(round((stamp_f - sec) * 1e9))
    # Clamp nanosec to [0, 1e9) to avoid rollover edge cases.
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    t = Time()
    t.sec = sec
    t.nanosec = nanosec
    return t


# ---------------------------------------------------------------------------
# ROS 2 node factory
# ---------------------------------------------------------------------------

def _import_ros():
    """Import ROS 2 modules lazily."""
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import Header
    return rclpy, Node, Time, QoSProfile, ReliabilityPolicy, HistoryPolicy, Odometry, PoseWithCovarianceStamped, Header


def create_node():
    """Build and return the ``GtsamTrackerNode`` (ROS 2 Node subclass)."""
    (rclpy, Node, Time, QoSProfile, ReliabilityPolicy, HistoryPolicy, Odometry, PoseWithCovarianceStamped, Header) = _import_ros()

    # Optional TF2 (for head_pose_source="tf" and for broadcasting the
    # optimised poses back into the TF tree — V6 §6.3 / §6.9).
    try:
        from tf2_ros import Buffer, TransformListener, TransformBroadcaster
        _HAS_TF2 = True
    except ImportError:
        Buffer = None
        TransformListener = None
        TransformBroadcaster = None
        _HAS_TF2 = False

    # Optional sensor_fusion_msgs (ArUco observations)
    try:
        from sensor_fusion_msgs.msg import (
            MarkerPoseObservation,
            DynamicMarkerObservation,
            DynamicArmPoseObservation,
        )
        _HAS_ARUCO_MSGS = True
    except ImportError:
        MarkerPoseObservation = None
        DynamicMarkerObservation = None
        DynamicArmPoseObservation = None
        _HAS_ARUCO_MSGS = False

    class GtsamTrackerNode(Node):
        """ROS 2 node wrapping :class:`TrajectoryFactorGraph`."""

        def __init__(self):
            super().__init__("gtsam_tracker")

            # ── Declare parameters (V6 §6.9) ────────────────────────────
            for key, default in DEFAULT_PARAMS.items():
                self.declare_parameter(key, default)

            p = lambda k: self.get_parameter(k).value  # noqa: E731

            self._head_odom_topic = str(p("head_odom_topic"))
            self._arm_odom_topic = str(p("arm_odom_topic"))
            self._head_pose_source = str(p("head_pose_source"))
            self._head_tf_target = str(p("head_tf_target_frame"))
            self._head_tf_source = str(p("head_tf_source_frame"))
            self._visual_topic = str(p("visual_factor_topic"))
            self._graph_rate = float(p("graph_rate_hz"))
            self._lag_s = float(p("smoother_lag_s"))
            self._kinematic_range = float(p("kinematic_range_m"))
            self._kinematic_sigma = float(p("kinematic_range_sigma_m"))
            self._kinematic_check_interval = int(p("kinematic_check_interval"))
            self._sigma_t = float(p("odom_translation_sigma_m"))
            self._sigma_r = float(p("odom_rotation_sigma_rad"))
            self._max_delta = float(p("max_odom_delta_m"))
            self._head_pose_topic = str(p("head_pose_topic"))
            self._arm_pose_topic = str(p("arm_pose_topic"))
            self._world_frame = str(p("world_frame"))
            self._broadcast_tf = bool(p("broadcast_tf"))
            self._head_child_frame = str(p("head_child_frame"))
            self._arm_child_frame = str(p("arm_child_frame"))

            # ── Factor graph ─────────────────────────────────────────────
            self._graph = TrajectoryFactorGraph(lag_s=self._lag_s)

            # Key counters (advance together for synchronized chains)
            self._key_idx = 0

            # Recovery bookkeeping: consecutive graph-update failures trigger a
            # full graph reset so the node self-heals after ISAM2 corruption.
            self._consecutive_failures = 0
            self._reset_count = 0

            # Latest odom caches
            self._head_pose: Optional[np.ndarray] = None
            self._arm_pose: Optional[np.ndarray] = None
            self._head_stamp: float = 0.0
            self._arm_stamp: float = 0.0
            self._head_prev: Optional[np.ndarray] = None
            self._arm_prev: Optional[np.ndarray] = None

            # Default odom noise (when covariance is unavailable)
            self._default_noise = default_odom_noise(
                self._sigma_t, self._sigma_r)

            # ArUco graceful-degradation tracking
            self._aruco_warned = False
            self._aruco_received = False

            # ── TF2 (optional, for head_pose_source="tf" and for broadcasting
            #    the optimised poses back into the TF tree) ────────────────
            self._tf_buffer = None
            self._tf_listener = None
            self._tf_broadcaster = None
            if self._head_pose_source == "tf":
                if _HAS_TF2:
                    self._tf_buffer = Buffer()
                    self._tf_listener = TransformListener(
                        self._tf_buffer, self)
                    self.get_logger().info(
                        "Head pose source: TF "
                        f"({self._head_tf_target} → {self._head_tf_source})")
                else:
                    self.get_logger().warn(
                        "head_pose_source='tf' but tf2_ros not available — "
                        "falling back to odom")
                    self._head_pose_source = "odom"

            # TF broadcaster: publishes the smoothed head/arm poses as dynamic
            # TF edges so downstream nodes consume the FGO trajectory rather
            # than the raw VIO relay (V6 §6.3).
            if self._broadcast_tf and _HAS_TF2:
                self._tf_broadcaster = TransformBroadcaster(self)
                self.get_logger().info(
                    "TF broadcasting enabled "
                    f"({self._world_frame} → {self._head_child_frame}, "
                    f"{self._arm_child_frame})")
            elif self._broadcast_tf and not _HAS_TF2:
                self.get_logger().warn(
                    "broadcast_tf=true but tf2_ros not available — "
                    "smoothed poses will be published as topics only")

            # ── Subscriptions ────────────────────────────────────────────
            # Use BEST_EFFORT QoS — Jetson relay publishes with BEST_EFFORT
            best_effort = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )

            if self._head_pose_source == "odom":
                self.create_subscription(
                    Odometry, self._head_odom_topic,
                    self._on_head_odom, best_effort)

            # Arm odom is always from the odom topic
            self.create_subscription(
                Odometry, self._arm_odom_topic,
                self._on_arm_odom, best_effort)

            # ArUco subscriptions (graceful degradation)
            if _HAS_ARUCO_MSGS:
                aruco_marker_topic = str(p("aruco_marker_topic"))
                aruco_dynamic_topic = str(p("aruco_dynamic_topic"))
                aruco_arm_topic = str(p("aruco_arm_pose_topic"))
                self.create_subscription(
                    MarkerPoseObservation, aruco_marker_topic,
                    self._on_aruco_marker, best_effort)
                self.create_subscription(
                    DynamicMarkerObservation, aruco_dynamic_topic,
                    self._on_aruco_dynamic, best_effort)
                self.create_subscription(
                    DynamicArmPoseObservation, aruco_arm_topic,
                    self._on_aruco_arm, best_effort)
            else:
                self.get_logger().warn(
                    "sensor_fusion_msgs not available — ArUco factors disabled")

            # Visual factor subscription (from cross_camera_features)
            self.create_subscription(
                PoseWithCovarianceStamped, self._visual_topic,
                self._on_visual_factor, best_effort)

            # ── Publishers ───────────────────────────────────────────────
            self._head_pub = self.create_publisher(
                PoseWithCovarianceStamped, self._head_pose_topic, 10)
            self._arm_pub = self.create_publisher(
                PoseWithCovarianceStamped, self._arm_pose_topic, 10)

            # ── Graph update timer ───────────────────────────────────────
            period = 1.0 / self._graph_rate if self._graph_rate > 0 else 0.1
            self._update_count = 0
            self._publish_count = 0
            self._odom_head_count = 0
            self._odom_arm_count = 0
            self._visual_count = 0
            self._delta_reject_count = 0
            self._timer = self.create_timer(period, self._graph_update)

            # ── Periodic stats timer ─────────────────────────────────────
            # Every 10s, log a one-line health summary so the run log
            # surfaces divergence / clock-skew / stall issues at a glance
            # without needing to grep through raw odom messages.
            self.create_timer(10.0, self._log_stats)

            self.get_logger().info(
                f"GtsamTrackerNode ready "
                f"(rate={self._graph_rate}Hz, lag={self._lag_s}s, "
                f"head_source={self._head_pose_source}, "
                f"broadcast_tf={self._broadcast_tf and _HAS_TF2})")

        # ------------------------------------------------------------------
        # Odometry callbacks
        # ------------------------------------------------------------------

        def _on_head_odom(self, msg: Odometry):
            """Cache the latest head odometry pose."""
            self._head_pose = _odom_to_matrix(msg)
            self._head_stamp = _stamp_to_float(msg.header.stamp)
            self._odom_head_count += 1

        def _on_arm_odom(self, msg: Odometry):
            """Cache the latest arm odometry pose."""
            self._arm_pose = _odom_to_matrix(msg)
            self._arm_stamp = _stamp_to_float(msg.header.stamp)
            self._odom_arm_count += 1
            # Extract covariance for arm noise
            self._arm_cov_diag = _odom_covariance_diag(
                msg, self._sigma_t, self._sigma_r)

        def _lookup_head_tf(self) -> bool:
            """Look up head pose from TF.  Returns True on success."""
            if self._tf_buffer is None:
                return False
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._head_tf_target, self._head_tf_source,
                    Time(), timeout=rclpy.duration.Duration(seconds=0.05))
                T = np.eye(4)
                t = tf.transform.translation
                q = tf.transform.rotation
                # Build matrix from quaternion + translation
                from gtsam_tracker.se3_helpers import quaternion_to_rotation_matrix
                T[:3, :3] = quaternion_to_rotation_matrix(
                    np.array([q.x, q.y, q.z, q.w]))
                T[:3, 3] = [t.x, t.y, t.z]
                self._head_pose = T
                self._head_stamp = time.time()
                return True
            except Exception:
                return False

        # ------------------------------------------------------------------
        # ArUco callbacks (graceful degradation)
        # ------------------------------------------------------------------

        def _on_aruco_marker(self, msg):
            """Add an ArUco prior factor (absolute pose constraint)."""
            if not self._aruco_received:
                self._aruco_received = True
            T = pose_to_matrix(msg.pose.pose)
            cov_diag = list(msg.pose.covariance)
            diag = [cov_diag[0], cov_diag[7], cov_diag[14],
                    cov_diag[21], cov_diag[28], cov_diag[35]]
            if all(d <= 1e-12 for d in diag):
                diag = [0.01 ** 2] * 3 + [0.01 ** 2] * 3

            # Use the current head key for the prior
            key = self._graph._make_key("h", self._key_idx)
            self._graph.add_aruco_prior(key, T, diag)

        def _on_aruco_dynamic(self, msg):
            """Add a dynamic ArUco between-factor (relative marker obs)."""
            if not self._aruco_received:
                self._aruco_received = True
            T_cam_marker = pose_to_matrix(msg.pose.pose)
            cov_diag = list(msg.pose.covariance)
            diag = [cov_diag[0], cov_diag[7], cov_diag[14],
                    cov_diag[21], cov_diag[28], cov_diag[35]]
            if all(d <= 1e-12 for d in diag):
                diag = [0.02 ** 2] * 3 + [0.02 ** 2] * 3

            # Add as a between-factor on the current head key (self-constraint)
            key = self._graph._make_key("h", self._key_idx)
            self._graph.add_visual_between(
                key, key, np.eye(4), diag)

        def _on_aruco_arm(self, msg):
            """Add an ArUco arm pose observation as a prior on the arm chain."""
            if not self._aruco_received:
                self._aruco_received = True
            T = pose_to_matrix(msg.pose.pose)
            cov_diag = list(msg.pose.covariance)
            diag = [cov_diag[0], cov_diag[7], cov_diag[14],
                    cov_diag[21], cov_diag[28], cov_diag[35]]
            if all(d <= 1e-12 for d in diag):
                diag = [0.01 ** 2] * 3 + [0.01 ** 2] * 3

            key = self._graph._make_key("a", self._key_idx)
            self._graph.add_aruco_prior(key, T, diag)

        # ------------------------------------------------------------------
        # Visual factor callback
        # ------------------------------------------------------------------

        def _on_visual_factor(self, msg: PoseWithCovarianceStamped):
            """Add a cross-chain visual between-factor."""
            self._visual_count += 1
            T_head_arm = pose_to_matrix(msg.pose.pose)
            cov_diag = list(msg.pose.covariance)
            diag = [cov_diag[0], cov_diag[7], cov_diag[14],
                    cov_diag[21], cov_diag[28], cov_diag[35]]
            if all(d <= 1e-12 for d in diag):
                diag = [0.05 ** 2] * 3 + [0.05 ** 2] * 3

            kh = self._graph._make_key("h", self._key_idx)
            ka = self._graph._make_key("a", self._key_idx)
            self._graph.add_visual_between(kh, ka, T_head_arm, diag)

        # ------------------------------------------------------------------
        # Graph update loop
        # ------------------------------------------------------------------

        def _graph_update(self):
            """Run one graph update cycle at graph_rate_hz."""
            # If head source is TF, look up the latest transform
            if self._head_pose_source == "tf":
                self._lookup_head_tf()

            # Check if we have new data for either chain
            head_new = (self._head_pose is not None and
                        (self._head_prev is None or
                         not np.allclose(self._head_pose, self._head_prev,
                                         atol=1e-9)))
            arm_new = (self._arm_pose is not None and
                       (self._arm_prev is None or
                        not np.allclose(self._arm_pose, self._arm_prev,
                                        atol=1e-9)))

            if not head_new and not arm_new:
                # No new data — skip this update
                return

            # Compute deltas
            if self._head_prev is not None and self._head_pose is not None:
                delta_head = relative_transform(
                    self._head_prev, self._head_pose)
            elif self._head_pose is not None:
                delta_head = self._head_pose.copy()
            else:
                delta_head = np.eye(4)

            if self._arm_prev is not None and self._arm_pose is not None:
                delta_arm = relative_transform(
                    self._arm_prev, self._arm_pose)
            elif self._arm_pose is not None:
                delta_arm = self._arm_pose.copy()
            else:
                delta_arm = np.eye(4)

            # ── Sanity gate: reject absurd deltas from diverging VIO ─────
            # If the translation component of either delta exceeds
            # ``max_odom_delta_m``, the odometry source is almost certainly
            # diverging (e.g. OpenVINS integrating garbage when static).
            # Injecting a 900m between-factor would poison the graph, so we
            # skip this update entirely and re-sync ``_head_prev`` /
            # ``_arm_prev`` to the current poses so the next healthy delta is
            # measured from here.
            head_jump = float(np.linalg.norm(delta_head[:3, 3]))
            arm_jump = float(np.linalg.norm(delta_arm[:3, 3]))
            if (self._max_delta > 0.0 and
                    (head_jump > self._max_delta or
                     arm_jump > self._max_delta)):
                self.get_logger().warn(
                    f"Rejecting odometry delta — head jump {head_jump:.3f}m, "
                    f"arm jump {arm_jump:.3f}m exceed max "
                    f"{self._max_delta}m. Re-syncing pose baseline "
                    f"(diverging VIO suspected).",
                    throttle_duration_sec=2.0)
                self._delta_reject_count += 1
                if self._head_pose is not None:
                    self._head_prev = self._head_pose.copy()
                if self._arm_pose is not None:
                    self._arm_prev = self._arm_pose.copy()
                return

            # Noise models
            noise_head = self._default_noise
            noise_arm = self._default_noise
            if hasattr(self, "_arm_cov_diag"):
                noise_arm = noise_from_covariance_diag(self._arm_cov_diag)

            # Add odometry factors (both chains advance together)
            stamp = max(self._head_stamp, self._arm_stamp, time.time())
            try:
                self._graph.add_odometry_factor(
                    self._key_idx, self._key_idx, stamp,
                    delta_head, delta_arm,
                    noise_head, noise_arm)
            except Exception as exc:
                self._consecutive_failures += 1
                self.get_logger().error(
                    f"Failed to add odometry factor "
                    f"({self._consecutive_failures}x): {exc}",
                    throttle_duration_sec=5.0)
                # Recovery: add_odometry_factor calls get_pose() internally,
                # which hits the corrupted ISAM2 state.  After repeated
                # failures, reset the graph so the node self-heals.
                if self._consecutive_failures >= 3:
                    self._reset_graph()
                return

            # Update previous poses
            if self._head_pose is not None:
                self._head_prev = self._head_pose.copy()
            if self._arm_pose is not None:
                self._arm_prev = self._arm_pose.copy()

            # Run ISAM2 update
            try:
                self._graph.update()
                self._consecutive_failures = 0
            except Exception as exc:
                self._consecutive_failures += 1
                self.get_logger().error(
                    f"Graph update failed ({self._consecutive_failures}x): "
                    f"{exc}", throttle_duration_sec=5.0)
                # Recovery: after repeated failures the ISAM2 internal state
                # is corrupt (e.g. a BetweenFactor references a variable that
                # was never inserted).  Reset the graph and re-seed from the
                # current poses so the node self-heals instead of looping on
                # the same broken VectorValues forever.
                if self._consecutive_failures >= 3:
                    self._reset_graph()
                    return

            # Marginalise old keys
            try:
                self._graph.marginalize_old_keys(stamp)
            except Exception:
                pass

            # Kinematic range factor (every N updates)
            self._update_count += 1
            if (self._kinematic_check_interval > 0 and
                    self._update_count % self._kinematic_check_interval == 0):
                self._add_kinematic_range_factor()

            # Publish poses
            self._publish_poses()
            self._publish_count += 1

            # ArUco graceful degradation warning
            if not self._aruco_received and not self._aruco_warned:
                if self._update_count > int(self._graph_rate * 5):
                    self.get_logger().warn(
                        "No ArUco messages received after 5s — running on "
                        "odometry + visual factors only")
                    self._aruco_warned = True

            self._key_idx += 1

        # ------------------------------------------------------------------
        # Kinematic range factor
        # ------------------------------------------------------------------

        def _add_kinematic_range_factor(self):
            """Add a soft range constraint between head and arm."""
            kh = self._graph._make_key("h", max(self._key_idx - 1, 0))
            ka = self._graph._make_key("a", max(self._key_idx - 1, 0))
            try:
                self._graph.add_range_factor(
                    kh, ka, self._kinematic_range,
                    self._kinematic_sigma)
            except Exception:
                pass  # Keys may not exist yet — non-fatal

        # ------------------------------------------------------------------
        # Graph recovery (self-heal after ISAM2 corruption)
        # ------------------------------------------------------------------

        def _reset_graph(self):
            """Reinitialize the factor graph and re-seed from current poses.

            Called when repeated ISAM2 updates fail (e.g.
            ``IndeterminantLinearSystemException`` from diverging VIO).  After
            reset, the next ``_graph_update`` cycle treats the current poses as
            the origin of a fresh trajectory, so downstream consumers see a
            small discontinuity rather than a permanent error loop.

            Reference: V6 §9 fault tolerance / graceful degradation.
            """
            self._graph.reset()
            self._key_idx = 0
            self._head_prev = None
            self._arm_prev = None
            self._consecutive_failures = 0
            self._reset_count += 1
            self.get_logger().warn(
                f"Factor graph reset (#{self._reset_count}). Re-seeding from "
                f"current poses — expect a small pose discontinuity.")

        # ------------------------------------------------------------------
        # Periodic health stats
        # ------------------------------------------------------------------

        def _log_stats(self):
            """Log a one-line health summary every 10s.

            Surfaces the key signals that indicate problems at a glance:
              - Pose norms: if these climb past a few metres, OpenVINS is
                diverging (the delta gate catches the jumps, but the norm
                shows the accumulated drift).
              - Clock offset: sensor stamp vs host clock.  Under chrony
                sync this should be < 0.1s; a large value means time sync
                is broken and timestamp-based TF lookups will fail.
              - Odom rates: if head or arm odom stops arriving, the graph
                stalls.  The counts let you spot a dead publisher.
              - Delta rejects / resets: non-zero values mean the delta
                gate or graph recovery is actively working.
            """
            head_norm = (float(np.linalg.norm(self._head_pose[:3, 3]))
                         if self._head_pose is not None else -1.0)
            arm_norm = (float(np.linalg.norm(self._arm_pose[:3, 3]))
                        if self._arm_pose is not None else -1.0)

            # Clock offset: how far behind the host clock is the latest
            # sensor stamp?  Positive = sensor is in the past (expected
            # due to network transit); large positive = sync broken.
            host_now = time.time()
            head_offset = (host_now - self._head_stamp
                           if self._head_stamp > 0 else -1.0)
            arm_offset = (host_now - self._arm_stamp
                          if self._arm_stamp > 0 else -1.0)

            self.get_logger().info(
                f"Stats: published={self._publish_count} "
                f"odom(head={self._odom_head_count}, arm={self._odom_arm_count}) "
                f"visual={self._visual_count} "
                f"delta_rejects={self._delta_reject_count} "
                f"resets={self._reset_count} "
                f"pose_norm(head={head_norm:.3f}m, arm={arm_norm:.3f}m) "
                f"clock_offset(head={head_offset:.3f}s, arm={arm_offset:.3f}s)"
            )

            # Reset per-interval counters (keep cumulative odom/reset counts)
            self._publish_count = 0
            self._visual_count = 0
            self._delta_reject_count = 0

        # ------------------------------------------------------------------
        # Pose publishing
        # ------------------------------------------------------------------

        def _publish_poses(self):
            """Query the graph and publish head/arm poses.

            Poses are stamped with the latest sensor time (not host publish
            time) so downstream consumers (keyframe_buffer, pointcloud_fusion)
            can align them with sensor data under chrony time sync.
            """
            kh = self._graph._make_key("h", max(self._key_idx - 1, 0))
            ka = self._graph._make_key("a", max(self._key_idx - 1, 0))

            # Use the most recent sensor stamp available so the published pose
            # is temporally consistent with the sensor data that produced it.
            sensor_stamp_f = max(self._head_stamp, self._arm_stamp)
            if sensor_stamp_f <= 0.0:
                sensor_stamp_f = time.time()

            try:
                head_pose3 = self._graph.get_pose(kh)
                self._publish_pose(
                    head_pose3, self._head_pub, sensor_stamp_f,
                    child_frame=self._head_child_frame)
            except Exception:
                pass  # Key may not exist yet

            try:
                arm_pose3 = self._graph.get_pose(ka)
                self._publish_pose(
                    arm_pose3, self._arm_pub, sensor_stamp_f,
                    child_frame=self._arm_child_frame)
            except Exception:
                pass

        def _publish_pose(self, pose3, publisher, stamp_f: float,
                          child_frame: Optional[str] = None):
            """Convert a gtsam.Pose3 to PoseWithCovarianceStamped and publish.

            Optionally also broadcast a dynamic TF edge
            (world_frame → child_frame) when ``self._tf_broadcaster`` is set.

            Parameters
            ----------
            pose3 : gtsam.Pose3
                The optimised pose.
            publisher : rclpy Publisher
                Topic publisher for the pose.
            stamp_f : float
                Sensor timestamp (seconds) to stamp the message with.  Using
                the sensor time (rather than ``get_clock().now()``) keeps the
                pose temporally aligned with the sensor data under chrony
                time sync.
            child_frame : str, optional
                If given and TF broadcasting is enabled, broadcast a dynamic
                TF edge (world_frame → child_frame).
            """
            from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

            T = np.eye(4)
            T[:3, :3] = pose3.rotation().matrix()
            T[:3, 3] = pose3.translation()

            t_tuple, q_tuple = matrix_to_pose(T)

            # Convert the float sensor stamp back to a ROS Time message.
            stamp_msg = _float_to_stamp(stamp_f)

            msg = PoseWithCovarianceStamped()
            msg.header = Header()
            msg.header.stamp = stamp_msg
            msg.header.frame_id = self._world_frame
            msg.pose.pose.position.x = t_tuple[0]
            msg.pose.pose.position.y = t_tuple[1]
            msg.pose.pose.position.z = t_tuple[2]
            msg.pose.pose.orientation.x = q_tuple[0]
            msg.pose.pose.orientation.y = q_tuple[1]
            msg.pose.pose.orientation.z = q_tuple[2]
            msg.pose.pose.orientation.w = q_tuple[3]
            # Identity covariance (ISAM2 marginal covariance extraction
            # would require calculateEstimateCovariance — deferred).
            msg.pose.covariance = [0.0] * 36
            publisher.publish(msg)

            # Broadcast dynamic TF edge (world_frame → child_frame) so the
            # smoothed trajectory feeds the TF tree consumed by
            # pointcloud_fusion, rviz, etc. (V6 §6.3).
            if child_frame is not None and self._tf_broadcaster is not None:
                from tf2_ros import TransformStamped
                tf_msg = TransformStamped()
                tf_msg.header.stamp = stamp_msg
                tf_msg.header.frame_id = self._world_frame
                tf_msg.child_frame_id = child_frame
                tf_msg.transform.translation.x = t_tuple[0]
                tf_msg.transform.translation.y = t_tuple[1]
                tf_msg.transform.translation.z = t_tuple[2]
                tf_msg.transform.rotation.x = q_tuple[0]
                tf_msg.transform.rotation.y = q_tuple[1]
                tf_msg.transform.rotation.z = q_tuple[2]
                tf_msg.transform.rotation.w = q_tuple[3]
                self._tf_broadcaster.sendTransform(tf_msg)

    return GtsamTrackerNode


def main(args=None):
    """Entry point for the ``gtsam_tracker_node`` console script."""
    (rclpy, *_rest) = _import_ros()
    rclpy.init(args=args)
    NodeClass = create_node()
    node = NodeClass()
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
