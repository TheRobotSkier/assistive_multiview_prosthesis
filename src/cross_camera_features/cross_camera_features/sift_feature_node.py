#!/usr/bin/env python3
"""sift_feature_node — SIFT-based cross-camera 3D-3D alignment.

Subscribes to head + arm image/cloud/camera_info topics, synchronises head+arm
image pairs with ``ApproximateTimeSynchronizer`` (50 ms slop), extracts SIFT
features, matches them, looks up 3-D depth at each keypoint (organisation-aware),
runs Umeyama alignment, and publishes ``/vis/head_arm_pose``
(``geometry_msgs/PoseWithCovariance``) consumed by the GTSAM tracker as a visual
between-factor.

The pure-logic core (:func:`match_and_align`) has ZERO ROS imports so it can be
unit-tested on the host with synthetic images.

Reference: V6 plan §6.6, §5.8.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

# Pure-logic helpers (Phase 1 Task B + C).
from keyframe_buffer.cloud_utils import lookup_depth_3d
from gtsam_tracker.umeyama import umeyama


__all__ = [
    "match_and_align",
    "AlignResult",
    "create_node",
    "main",
]


# ---------------------------------------------------------------------------
# Pure-logic core — unit-tested without ROS
# ---------------------------------------------------------------------------

class AlignResult(Tuple):
    """Named-tuple-like result of :func:`match_and_align`.

    Fields
    ------
    T : ndarray (4, 4) or None
        Estimated ``T_head_arm`` (maps arm-frame points to head frame) or
        ``None`` if alignment failed.
    covariance : ndarray (6, 6) or None
        6×6 covariance of the transform, or ``None``.
    num_matches : int
        Number of valid 3-D-3-D correspondences used.
    num_raw_matches : int
        Number of raw descriptor matches before depth filtering.
    message : str
        Human-readable status.
    """

    __slots__ = ()

    def __new__(cls, T, covariance, num_matches, num_raw_matches, message):
        return tuple.__new__(
            cls, (T, covariance, num_matches, num_raw_matches, message))

    @property
    def T(self):
        return self[0]

    @property
    def covariance(self):
        return self[1]

    @property
    def num_matches(self) -> int:
        return self[2]

    @property
    def num_raw_matches(self) -> int:
        return self[3]

    @property
    def message(self) -> str:
        return self[4]

    def __repr__(self) -> str:
        return (f"AlignResult(matches={self.num_matches}, "
                f"raw={self.num_raw_matches}, message={self.message!r})")


def extract_sift(sift, image: np.ndarray):
    """Detect + compute SIFT on a greyscale image.

    Parameters
    ----------
    sift : cv2.SIFT
        A SIFT detector instance.
    image : ndarray (H, W, 3) or (H, W)
        RGB or greyscale image.

    Returns
    -------
    keypoints, descriptors
    """
    import cv2  # lazy

    if image.ndim == 3:
        grey = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    else:
        grey = image
    return sift.detectAndCompute(grey, None)


def match_descriptors(matcher, des_a: np.ndarray, des_b: np.ndarray,
                      top_k: int = 100,
                      ratio_threshold: float = 0.8) -> List[Tuple[int, int]]:
    """Match two SIFT descriptor sets with Lowe's ratio test.

    Returns a list of ``(query_idx, train_idx)`` correspondences sorted by
    distance.
    """
    if des_a is None or des_b is None:
        return []
    if des_a.shape[0] == 0 or des_b.shape[0] == 0:
        return []

    # knnMatch with k=2 for Lowe's ratio test.
    knn = matcher.knnMatch(des_a, des_b, k=2)

    good = []
    for pair in knn:
        if len(pair) < 2:
            continue
        m, n = pair[0], pair[1]
        if m.distance < ratio_threshold * n.distance:
            good.append((m.queryIdx, m.trainIdx, m.distance))

    # If ratio test yields too few, fall back to plain top-k matching.
    if len(good) < 5:
        matches = matcher.match(des_a, des_b)
        matches = sorted(matches, key=lambda m: m.distance)[:top_k]
        good = [(m.queryIdx, m.trainIdx, m.distance) for m in matches]
    else:
        good.sort(key=lambda t: t[2])
        good = good[:top_k]

    return [(q, t) for q, t, _ in good]


def match_and_align(
    sift,
    matcher,
    head_image: np.ndarray,
    arm_image: np.ndarray,
    head_cloud: np.ndarray,
    arm_cloud: np.ndarray,
    head_organized: bool,
    arm_organized: bool,
    head_K: np.ndarray,
    arm_K: np.ndarray,
    head_pose: np.ndarray,
    arm_pose: np.ndarray,
    min_matches: int = 5,
    top_k: int = 100,
    ratio_threshold: float = 0.8,
    depth_search_radius_m: float = 0.02,
) -> AlignResult:
    """Pure-logic SIFT match + depth lookup + Umeyama alignment.  No ROS.

    Parameters
    ----------
    sift : cv2.SIFT
    matcher : cv2.BFMatcher
    head_image, arm_image : ndarray (H, W, 3) uint8
    head_cloud, arm_cloud : ndarray
        ``(H, W, 3)`` if organised, else ``(N, 3)``.  World frame.
    head_organized, arm_organized : bool
    head_K, arm_K : ndarray (3, 3)
    head_pose, arm_pose : ndarray (4, 4)
        ``T_world_camera`` for each camera.
    min_matches : int
        Minimum valid 3-D correspondences to publish.
    top_k : int
        Maximum number of matches to keep.
    ratio_threshold : float
        Lowe's ratio test threshold.
    depth_search_radius_m : float
        Ray search radius for unorganised depth lookup.

    Returns
    -------
    AlignResult
    """
    # 1. Extract SIFT on both images.
    kp_head, des_head = extract_sift(sift, head_image)
    kp_arm, des_arm = extract_sift(sift, arm_image)

    if des_head is None or des_arm is None:
        return AlignResult(None, None, 0, 0, "no descriptors extracted")
    if des_head.shape[0] == 0 or des_arm.shape[0] == 0:
        return AlignResult(
            None, None, 0, 0,
            f"empty descriptors (head={des_head.shape[0]}, "
            f"arm={des_arm.shape[0]})")

    # 2. Match descriptors.
    correspondences = match_descriptors(
        matcher, des_head, des_arm, top_k=top_k,
        ratio_threshold=ratio_threshold)

    if len(correspondences) == 0:
        return AlignResult(None, None, 0, 0, "no descriptor matches")

    # 3. Depth lookup — organisation-aware.
    p_head_list: List[np.ndarray] = []
    p_arm_list: List[np.ndarray] = []

    for q_idx, t_idx in correspondences:
        u_h = int(round(kp_head[q_idx].pt[0]))
        v_h = int(round(kp_head[q_idx].pt[1]))
        u_a = int(round(kp_arm[t_idx].pt[0]))
        v_a = int(round(kp_arm[t_idx].pt[1]))

        p3d_h = lookup_depth_3d(
            head_cloud, head_organized, u_h, v_h,
            K=head_K if not head_organized else None,
            pose=head_pose if not head_organized else None,
            search_radius_m=depth_search_radius_m)
        p3d_a = lookup_depth_3d(
            arm_cloud, arm_organized, u_a, v_a,
            K=arm_K if not arm_organized else None,
            pose=arm_pose if not arm_organized else None,
            search_radius_m=depth_search_radius_m)

        if p3d_h is not None and p3d_a is not None:
            p_head_list.append(np.asarray(p3d_h, dtype=np.float64))
            p_arm_list.append(np.asarray(p3d_a, dtype=np.float64))

    num_valid = len(p_head_list)

    if num_valid < min_matches:
        return AlignResult(
            None, None, num_valid, len(correspondences),
            f"only {num_valid} valid 3-D matches (need {min_matches}); "
            f"consider SuperPoint upgrade")

    # 4. Umeyama alignment.
    src = np.array(p_arm_list)   # arm points (source)
    dst = np.array(p_head_list)  # head points (destination)

    try:
        T, cov = umeyama(src, dst)
    except ValueError as exc:
        return AlignResult(
            None, None, num_valid, len(correspondences),
            f"umeyama failed: {exc}")

    return AlignResult(T, cov, num_valid, len(correspondences),
                       f"aligned {num_valid} matches")


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

DEFAULT_PARAMS = {
    "head_image_topic": "/jetson/head/image",
    "arm_image_topic": "/jetson/arm/image",
    "head_cloud_topic": "/jetson/head/points",
    "arm_cloud_topic": "/jetson/arm/points",
    "head_info_topic": "/jetson/head/camera_info",
    "arm_info_topic": "/jetson/arm/camera_info",
    "head_pose_topic": "/gtsam/head_pose",
    "arm_pose_topic": "/gtsam/arm_pose",
    "output_topic": "/vis/head_arm_pose",
    "process_rate_hz": 5.0,
    "sync_slop_s": 0.05,
    "min_matches": 5,
    "top_k_matches": 100,
    "ratio_threshold": 0.8,
    "depth_search_radius_m": 0.02,
    "backend": "sift",
    "world_frame": "marker_map",
}


def _import_ros():
    """Import ROS 2 modules lazily (host-testable core without ROS)."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    import message_filters
    from sensor_msgs.msg import Image, PointCloud2, CameraInfo
    from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
    from std_msgs.msg import Header
    return (rclpy, Node, QoSProfile, ReliabilityPolicy, HistoryPolicy,
            message_filters,
            Image, PointCloud2, CameraInfo,
            PoseWithCovariance, PoseWithCovarianceStamped, Header)


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _parse_cloud2_xyzrgb(msg) -> Tuple[np.ndarray, np.ndarray, bool]:
    """Parse a PointCloud2 into ``(xyz (N,3), rgb (N,3), organized)``."""
    n = msg.width * msg.height
    organized = msg.height > 1
    if n == 0:
        return (np.zeros((0, 3), dtype=np.float32),
                np.zeros((0, 3), dtype=np.uint8), organized)

    step = msg.point_step
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)
    fields = {f.name: f for f in msg.fields}

    def _col_f32(name: str) -> np.ndarray:
        off = fields[name].offset
        return np.frombuffer(
            raw[:, off:off + 4].copy().tobytes(), dtype=np.float32)

    xyz = np.column_stack([_col_f32("x"), _col_f32("y"), _col_f32("z")])

    rgb = np.zeros((n, 3), dtype=np.uint8)
    if "rgb" in fields:
        packed = _col_f32("rgb").view(np.uint32)
        rgb[:, 0] = (packed >> 16) & 0xFF
        rgb[:, 1] = (packed >> 8) & 0xFF
        rgb[:, 2] = packed & 0xFF

    if organized:
        H, W = int(msg.height), int(msg.width)
        xyz = xyz.reshape(H, W, 3)
        rgb = rgb.reshape(H, W, 3)

    return xyz, rgb, organized


def _image_to_rgb(msg) -> np.ndarray:
    """Decode a sensor_msgs/Image to ``(H, W, 3)`` uint8 RGB."""
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = msg.encoding
    if enc in ("rgb8", "bgr8"):
        arr = raw.reshape(msg.height, msg.width, 3)
        if enc == "bgr8":
            arr = arr[:, :, ::-1]
    elif enc in ("rgba8", "bgra8"):
        arr = raw.reshape(msg.height, msg.width, 4)[:, :, :3]
        if enc == "bgra8":
            arr = arr[:, :, ::-1]
    else:
        arr = raw.reshape(msg.height, msg.width, 3)
    return np.ascontiguousarray(arr)


def _camera_info_to_K(msg) -> np.ndarray:
    return np.array(msg.k, dtype=np.float64).reshape(3, 3)


def _pose_to_matrix(msg) -> np.ndarray:
    """Convert a PoseWithCovarianceStamped to a ``(4, 4)`` matrix."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T = np.eye(4, dtype=np.float64)
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-15:
        T[:3, :3] = np.eye(3)
    else:
        s = 2.0 / n
        xs, ys, zs = qx * s, qy * s, qz * s
        wx, wy, wz = qw * xs, qw * ys, qw * zs
        xx, xy, xz = qx * xs, qx * ys, qx * zs
        yy, yz, zz = qy * ys, qy * zs, qz * zs
        T[:3, :3] = np.array([
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ])
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _matrix_to_pose_with_cov(T: np.ndarray, cov: np.ndarray,
                             header) -> object:
    """Build a ``PoseWithCovarianceStamped`` from a transform + covariance."""
    from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
    from std_msgs.msg import Header as Hdr

    # Quaternion from rotation matrix (Shepperd).
    R = np.asarray(T, dtype=np.float64)[:3, :3]
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    msg = PoseWithCovarianceStamped()
    msg.header = header
    msg.pose.pose.position.x = float(T[0, 3])
    msg.pose.pose.position.y = float(T[1, 3])
    msg.pose.pose.position.z = float(T[2, 3])
    msg.pose.pose.orientation.x = float(x)
    msg.pose.pose.orientation.y = float(y)
    msg.pose.pose.orientation.z = float(z)
    msg.pose.pose.orientation.w = float(w)

    # Row-major 6×6 covariance.
    cov6 = np.asarray(cov, dtype=np.float64).reshape(6, 6)
    msg.pose.covariance = cov6.flatten().tolist()
    return msg


def create_node():
    """Build and return the ``SiftFeatureNode`` (ROS 2 Node subclass)."""
    (rclpy, Node, QoSProfile, ReliabilityPolicy, HistoryPolicy,
     message_filters,
     Image, PointCloud2, CameraInfo,
     PoseWithCovariance, PoseWithCovarianceStamped, Header) = _import_ros()

    import cv2  # lazy

    class SiftFeatureNode(Node):
        """ROS 2 node for cross-camera SIFT alignment."""

        def __init__(self):
            super().__init__("cross_camera_features")

            # ── Declare parameters (V6 §6.9) ────────────────────────────
            for key, default in DEFAULT_PARAMS.items():
                self.declare_parameter(key, default)

            p = lambda k: self.get_parameter(k).value  # noqa: E731

            self._min_matches = int(p("min_matches"))
            self._top_k = int(p("top_k_matches"))
            self._ratio = float(p("ratio_threshold"))
            self._depth_radius = float(p("depth_search_radius_m"))
            self._world_frame = str(p("world_frame"))

            # ── SIFT + matcher ──────────────────────────────────────────
            backend = str(p("backend")).lower()
            if backend == "sift":
                self._sift = cv2.SIFT_create()
                self._matcher = cv2.BFMatcher(cv2.NORM_L2)
            else:
                # Future: SuperPoint.  For now fall back to SIFT.
                self.get_logger().warn(
                    f"Unknown backend {backend!r}, falling back to SIFT.")
                self._sift = cv2.SIFT_create()
                self._matcher = cv2.BFMatcher(cv2.NORM_L2)

            # ── Latest cloud / info / pose caches (independent subs) ────
            self._head_cloud = None
            self._head_cloud_org = None
            self._arm_cloud = None
            self._arm_cloud_org = None
            self._head_K = None
            self._arm_K = None
            self._head_pose = None
            self._arm_pose = None

            # Cloud / info / pose subscriptions (independent, like the
            # keyframe buffer).  BEST_EFFORT — Jetson relay publishes BEST_EFFORT.
            best_effort = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            for cam in ("head", "arm"):
                self.create_subscription(
                    PointCloud2, str(p(f"{cam}_cloud_topic")),
                    lambda msg, c=cam: self._on_cloud(msg, c), best_effort)
                self.create_subscription(
                    CameraInfo, str(p(f"{cam}_info_topic")),
                    lambda msg, c=cam: self._on_info(msg, c), best_effort)
                self.create_subscription(
                    PoseWithCovarianceStamped, str(p(f"{cam}_pose_topic")),
                    lambda msg, c=cam: self._on_pose(msg, c), best_effort)

            # ── Synced image pair (ApproximateTimeSynchronizer) ─────────
            sync_slop = float(p("sync_slop_s"))
            self._head_img_sub = message_filters.Subscriber(
                self, Image, str(p("head_image_topic")),
                qos_profile=best_effort)
            self._arm_img_sub = message_filters.Subscriber(
                self, Image, str(p("arm_image_topic")),
                qos_profile=best_effort)
            self._sync = message_filters.ApproximateTimeSynchronizer(
                [self._head_img_sub, self._arm_img_sub],
                queue_size=10, slop=sync_slop)
            self._sync.registerCallback(self._on_synced_images)

            # ── Publisher ───────────────────────────────────────────────
            self._pub = self.create_publisher(
                PoseWithCovarianceStamped, str(p("output_topic")), 10)

            # ── Diagnostics ─────────────────────────────────────────────
            self._frame_count = 0
            self._publish_count = 0
            self._low_match_warned = False
            self.create_timer(10.0, self._diag)

            self.get_logger().info(
                f"SiftFeatureNode ready (backend={backend}, "
                f"min_matches={self._min_matches}, slop={sync_slop}s)")

        # ── Independent callbacks (cloud / info / pose) ────────────────

        def _on_cloud(self, msg: PointCloud2, cam: str):
            xyz, rgb, org = _parse_cloud2_xyzrgb(msg)
            if cam == "head":
                self._head_cloud = xyz
                self._head_cloud_org = org
            else:
                self._arm_cloud = xyz
                self._arm_cloud_org = org

        def _on_info(self, msg: CameraInfo, cam: str):
            K = _camera_info_to_K(msg)
            if cam == "head":
                self._head_K = K
            else:
                self._arm_K = K

        def _on_pose(self, msg: PoseWithCovarianceStamped, cam: str):
            T = _pose_to_matrix(msg)
            if cam == "head":
                self._head_pose = T
            else:
                self._arm_pose = T

        # ── Synced image callback ──────────────────────────────────────

        def _on_synced_images(self, head_img_msg: Image, arm_img_msg: Image):
            self._frame_count += 1

            # Need cloud + K + pose for both cameras before we can align.
            if (self._head_cloud is None or self._arm_cloud is None
                    or self._head_K is None or self._arm_K is None
                    or self._head_pose is None or self._arm_pose is None):
                return

            try:
                head_img = _image_to_rgb(head_img_msg)
                arm_img = _image_to_rgb(arm_img_msg)
            except Exception as exc:
                self.get_logger().warn(
                    f"Image decode failed: {exc}", throttle_duration_sec=10.0)
                return

            result = match_and_align(
                self._sift, self._matcher,
                head_img, arm_img,
                self._head_cloud, self._arm_cloud,
                self._head_cloud_org, self._arm_cloud_org,
                self._head_K, self._arm_K,
                self._head_pose, self._arm_pose,
                min_matches=self._min_matches,
                top_k=self._top_k,
                ratio_threshold=self._ratio,
                depth_search_radius_m=self._depth_radius,
            )

            if result.T is not None and result.num_matches >= self._min_matches:
                header = Header()
                header.stamp = head_img_msg.header.stamp
                header.frame_id = self._world_frame
                msg = _matrix_to_pose_with_cov(
                    result.T, result.covariance, header)
                self._pub.publish(msg)
                self._publish_count += 1
                self.get_logger().debug(
                    f"Published head_arm_pose ({result.num_matches} matches)")
            else:
                # Diagnostics: warn if consistently low.
                if (result.num_matches < self._min_matches
                        and not self._low_match_warned
                        and self._frame_count > 10):
                    self.get_logger().warn(
                        f"Consistently < {self._min_matches} SIFT matches "
                        f"({result.num_matches}). Consider SuperPoint upgrade.")
                    self._low_match_warned = True

        def _diag(self):
            self.get_logger().info(
                f"frames={self._frame_count}, published={self._publish_count}")

    return SiftFeatureNode


def main(args=None):
    """Entry point for the ``sift_feature_node`` console script."""
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
