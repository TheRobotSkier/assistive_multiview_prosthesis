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
    "head_odom_topic": "/ov_msckf/odomimu",
    "arm_odom_topic": "/ov_msckf_arm/odomimu",
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
    # Output topics
    "head_pose_topic": "/gtsam/head_pose",
    "arm_pose_topic": "/gtsam/arm_pose",
    # Output frame
    "world_frame": "marker_map",
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


# ---------------------------------------------------------------------------
# ROS 2 node factory
# ---------------------------------------------------------------------------

def _import_ros():
    """Import ROS 2 modules lazily."""
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import Header
    return rclpy, Node, Time, Odometry, PoseWithCovarianceStamped, Header


def create_node():
    """Build and return the ``GtsamTrackerNode`` (ROS 2 Node subclass)."""
    (rclpy, Node, Time, Odometry, PoseWithCovarianceStamped, Header) = _import_ros()

    # Optional TF2 (for head_pose_source="tf")
    try:
        from tf2_ros import Buffer, TransformListener
        _HAS_TF2 = True
    except ImportError:
        Buffer = None
        TransformListener = None
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
            self._head_pose_topic = str(p("head_pose_topic"))
            self._arm_pose_topic = str(p("arm_pose_topic"))
            self._world_frame = str(p("world_frame"))

            # ── Factor graph ─────────────────────────────────────────────
            self._graph = TrajectoryFactorGraph(lag_s=self._lag_s)

            # Key counters (advance together for synchronized chains)
            self._key_idx = 0

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

            # ── TF2 (optional, for head_pose_source="tf") ───────────────
            self._tf_buffer = None
            self._tf_listener = None
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

            # ── Subscriptions ────────────────────────────────────────────
            if self._head_pose_source == "odom":
                self.create_subscription(
                    Odometry, self._head_odom_topic,
                    self._on_head_odom, 10)

            # Arm odom is always from the odom topic
            self.create_subscription(
                Odometry, self._arm_odom_topic,
                self._on_arm_odom, 10)

            # ArUco subscriptions (graceful degradation)
            if _HAS_ARUCO_MSGS:
                aruco_marker_topic = str(p("aruco_marker_topic"))
                aruco_dynamic_topic = str(p("aruco_dynamic_topic"))
                aruco_arm_topic = str(p("aruco_arm_pose_topic"))
                self.create_subscription(
                    MarkerPoseObservation, aruco_marker_topic,
                    self._on_aruco_marker, 10)
                self.create_subscription(
                    DynamicMarkerObservation, aruco_dynamic_topic,
                    self._on_aruco_dynamic, 10)
                self.create_subscription(
                    DynamicArmPoseObservation, aruco_arm_topic,
                    self._on_aruco_arm, 10)
            else:
                self.get_logger().warn(
                    "sensor_fusion_msgs not available — ArUco factors disabled")

            # Visual factor subscription (from cross_camera_features)
            self.create_subscription(
                PoseWithCovarianceStamped, self._visual_topic,
                self._on_visual_factor, 10)

            # ── Publishers ───────────────────────────────────────────────
            self._head_pub = self.create_publisher(
                PoseWithCovarianceStamped, self._head_pose_topic, 10)
            self._arm_pub = self.create_publisher(
                PoseWithCovarianceStamped, self._arm_pose_topic, 10)

            # ── Graph update timer ───────────────────────────────────────
            period = 1.0 / self._graph_rate if self._graph_rate > 0 else 0.1
            self._update_count = 0
            self._timer = self.create_timer(period, self._graph_update)

            self.get_logger().info(
                f"GtsamTrackerNode ready "
                f"(rate={self._graph_rate}Hz, lag={self._lag_s}s, "
                f"head_source={self._head_pose_source})")

        # ------------------------------------------------------------------
        # Odometry callbacks
        # ------------------------------------------------------------------

        def _on_head_odom(self, msg: Odometry):
            """Cache the latest head odometry pose."""
            self._head_pose = _odom_to_matrix(msg)
            self._head_stamp = _stamp_to_float(msg.header.stamp)

        def _on_arm_odom(self, msg: Odometry):
            """Cache the latest arm odometry pose."""
            self._arm_pose = _odom_to_matrix(msg)
            self._arm_stamp = _stamp_to_float(msg.header.stamp)
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
            T_head_arm = pose_to_matrix(msg.pose)
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
                self.get_logger().error(
                    f"Failed to add odometry factor: {exc}", throttle_duration_sec=5.0)
                return

            # Update previous poses
            if self._head_pose is not None:
                self._head_prev = self._head_pose.copy()
            if self._arm_pose is not None:
                self._arm_prev = self._arm_pose.copy()

            # Run ISAM2 update
            try:
                self._graph.update()
            except Exception as exc:
                self.get_logger().error(
                    f"Graph update failed: {exc}", throttle_duration_sec=5.0)

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
        # Pose publishing
        # ------------------------------------------------------------------

        def _publish_poses(self):
            """Query the graph and publish head/arm poses."""
            kh = self._graph._make_key("h", max(self._key_idx - 1, 0))
            ka = self._graph._make_key("a", max(self._key_idx - 1, 0))

            try:
                head_pose3 = self._graph.get_pose(kh)
                self._publish_pose(head_pose3, self._head_pub)
            except Exception:
                pass  # Key may not exist yet

            try:
                arm_pose3 = self._graph.get_pose(ka)
                self._publish_pose(arm_pose3, self._arm_pub)
            except Exception:
                pass

        def _publish_pose(self, pose3, publisher):
            """Convert a gtsam.Pose3 to PoseWithCovarianceStamped and publish."""
            from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

            T = np.eye(4)
            T[:3, :3] = pose3.rotation().matrix()
            T[:3, 3] = pose3.translation()

            t_tuple, q_tuple = matrix_to_pose(T)

            msg = PoseWithCovarianceStamped()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
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
