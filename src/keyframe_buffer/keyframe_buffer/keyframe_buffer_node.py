#!/usr/bin/env python3
"""keyframe_buffer_node — spatial-gated keyframe storage ROS 2 node.

Subscribes to each camera's cloud, image, camera_info and pose **independently**
(no ``ApproximateTimeSynchronizer``) so that keyframes are never dropped during
rapid arm movement.  When a new cloud arrives, the node checks whether enough
motion has occurred since the last stored keyframe (spatial gate) and, if so,
creates a ``Keyframe`` and appends it to a per-camera ring buffer.

A ``GetKeyframesInROI`` service returns all keyframes whose camera translation
falls within a spherical region of interest — this is the data backbone for TSDF
fusion.

Design (V6 plan §6.2, §5.5):
- The cloud is treated as **unorganised by default** (``height == 1``); the
  organisation is auto-detected once at runtime and logged.
- The pure-logic core (``KeyframeBufferCore``) is ROS-free and unit-tested.
- The ROS node is a thin wrapper that wires topics/params to the core.

Reference: V6 plan §6.2, §5.5, §6.9.
"""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# Pure-logic helpers (ROS-free) — imported lazily-safe because these modules
# have zero ROS dependencies.
from keyframe_buffer.cloud_utils import detect_organization
from keyframe_buffer.keyframe import Keyframe

# SE(3) helpers live in the gtsam_tracker package (Phase 1 Task C).  They are
# pure-numpy with zero ROS imports, so importing them here is safe on the host.
try:
    from gtsam_tracker.se3_helpers import angle_between_quaternions
    from gtsam_tracker.se3_helpers import rotation_matrix_to_quaternion
    _HAS_SE3_HELPERS = True
except Exception:  # pragma: no cover — gtsam_tracker may not be on path on host
    _HAS_SE3_HELPERS = False

    def angle_between_quaternions(q1: np.ndarray, q2: np.ndarray) -> float:
        q1 = np.asarray(q1, dtype=np.float64).reshape(4)
        q2 = np.asarray(q2, dtype=np.float64).reshape(4)
        n1 = np.linalg.norm(q1)
        n2 = np.linalg.norm(q2)
        if n1 < 1e-15 or n2 < 1e-15:
            return 0.0
        dot = abs(np.dot(q1 / n1, q2 / n2))
        dot = min(dot, 1.0)
        return float(2.0 * np.arccos(dot))

    def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
        # Fallback Shepperd method (identical to se3_helpers).
        R = np.asarray(R, dtype=np.float64).reshape(3, 3)
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
        q = np.array([x, y, z, w])
        q /= np.linalg.norm(q)
        return q


__all__ = [
    "KeyframeBufferCore",
    "CAMERAS",
    "DEFAULT_PARAMS",
]


CAMERAS = ("head", "arm")

DEFAULT_PARAMS = {
    "head_image_topic": "/jetson/head/image",
    "arm_image_topic": "/jetson/arm/image",
    "head_cloud_topic": "/jetson/head/points",
    "arm_cloud_topic": "/jetson/arm/points",
    "head_info_topic": "/jetson/head/camera_info",
    "arm_info_topic": "/jetson/arm/camera_info",
    "head_pose_topic": "/gtsam/head_pose",
    "arm_pose_topic": "/gtsam/arm_pose",
    "max_keyframes_per_camera": 50,
    "spatial_gate_translation_m": 0.10,
    "spatial_gate_rotation_deg": 15.0,
    "pose_max_age_s": 0.10,
    "cloud_timestamp_source": "header",  # "header" or "receive_time"
}


# ---------------------------------------------------------------------------
# ROS-free core — unit-tested without rclpy
# ---------------------------------------------------------------------------

@dataclass
class _CameraState:
    """Per-camera "latest" buffers and spatial-gate bookkeeping."""

    image: Optional[object] = None       # latest image msg / array
    image_stamp: Optional[float] = None
    cloud: Optional[object] = None       # latest cloud msg
    cloud_stamp: Optional[float] = None
    info: Optional[object] = None        # latest CameraInfo msg
    info_stamp: Optional[float] = None
    pose: Optional[object] = None        # latest pose (4, 4) matrix
    pose_stamp: Optional[float] = None
    cloud_organized: Optional[bool] = None
    last_keyframe_pose: Optional[np.ndarray] = None  # (4, 4)


class KeyframeBufferCore:
    """ROS-free keyframe buffer with spatial gating and ROI queries.

    This is the pure-logic heart of the node.  The ROS node (below) feeds it
    decoded messages and reads back ``Keyframe`` objects.
    """

    def __init__(
        self,
        max_keyframes_per_camera: int = 50,
        spatial_gate_translation_m: float = 0.10,
        spatial_gate_rotation_deg: float = 15.0,
        pose_max_age_s: float = 0.05,
        cameras: tuple[str, ...] = CAMERAS,
    ):
        self.max_keyframes_per_camera = int(max_keyframes_per_camera)
        self.spatial_gate_translation_m = float(spatial_gate_translation_m)
        self.spatial_gate_rotation_rad = float(
            np.deg2rad(spatial_gate_rotation_deg))
        self.pose_max_age_s = float(pose_max_age_s)
        self.cameras = tuple(cameras)

        self._buffers: dict[str, deque] = {
            cam: deque(maxlen=self.max_keyframes_per_camera) for cam in self.cameras
        }
        self._state: dict[str, _CameraState] = {
            cam: _CameraState() for cam in self.cameras
        }
        self._lock = threading.Lock()

        # Counters for diagnostics
        self._accepted = {cam: 0 for cam in self.cameras}
        self._rejected_gate = {cam: 0 for cam in self.cameras}
        self._rejected_no_pose = {cam: 0 for cam in self.cameras}

    # ------------------------------------------------------------------
    # State updates (called by the ROS node callbacks)
    # ------------------------------------------------------------------

    def update_image(self, camera_id: str, image, stamp: Optional[float] = None):
        with self._lock:
            st = self._state[camera_id]
            st.image = image
            st.image_stamp = stamp

    def update_info(self, camera_id: str, info, stamp: Optional[float] = None):
        with self._lock:
            st = self._state[camera_id]
            st.info = info
            st.info_stamp = stamp

    def update_pose(self, camera_id: str, pose: np.ndarray,
                    stamp: Optional[float] = None):
        with self._lock:
            st = self._state[camera_id]
            st.pose = np.asarray(pose, dtype=np.float64).reshape(4, 4)
            st.pose_stamp = stamp

    def detect_cloud_organization(self, camera_id: str, height: int) -> bool:
        """Detect and cache cloud organisation for *camera_id*.

        Returns the organisation flag (``True`` = organised).
        """
        organized = detect_organization(height)
        with self._lock:
            st = self._state[camera_id]
            if st.cloud_organized is None:
                st.cloud_organized = organized
            return st.cloud_organized

    # ------------------------------------------------------------------
    # Spatial gate
    # ------------------------------------------------------------------

    def _spatial_gate_passes(
        self,
        camera_id: str,
        new_pose: np.ndarray,
    ) -> bool:
        """Return ``True`` if enough motion has occurred since the last keyframe.

        Accepts only when BOTH translation AND rotation thresholds are exceeded.
        Always accepts the first keyframe for a camera.
        """
        st = self._state[camera_id]
        last = st.last_keyframe_pose
        if last is None:
            return True  # first keyframe

        # Translation distance
        dt = new_pose[:3, 3] - last[:3, 3]
        trans_moved = float(np.linalg.norm(dt))
        if trans_moved < self.spatial_gate_translation_m:
            return False

        # Rotation angular distance
        q_new = rotation_matrix_to_quaternion(new_pose[:3, :3])
        q_last = rotation_matrix_to_quaternion(last[:3, :3])
        angle_rad = angle_between_quaternions(q_new, q_last)
        if angle_rad < self.spatial_gate_rotation_rad:
            return False

        return True

    # ------------------------------------------------------------------
    # Keyframe creation (triggered on cloud arrival)
    # ------------------------------------------------------------------

    def try_create_keyframe(
        self,
        camera_id: str,
        cloud_xyz: np.ndarray,
        cloud_rgb: np.ndarray,
        image: Optional[np.ndarray],
        K: Optional[np.ndarray],
        cloud_stamp: Optional[float] = None,
    ) -> Optional[Keyframe]:
        """Attempt to create and store a keyframe for *camera_id*.

        Returns the created ``Keyframe`` if accepted, or ``None`` if rejected
        by the spatial gate or if no fresh pose/image is available.
        """
        with self._lock:
            st = self._state[camera_id]

            # Need a pose within pose_max_age_s of the cloud.
            if st.pose is None:
                self._rejected_no_pose[camera_id] += 1
                return None
            if (cloud_stamp is not None and st.pose_stamp is not None):
                age = abs(cloud_stamp - st.pose_stamp)
                if age > self.pose_max_age_s:
                    self._rejected_no_pose[camera_id] += 1
                    return None

            pose = st.pose.copy()

            # Spatial gate
            if not self._spatial_gate_passes(camera_id, pose):
                self._rejected_gate[camera_id] += 1
                return None

            # Need at least the cloud + pose to make a keyframe.
            organized = st.cloud_organized if st.cloud_organized is not None else False
            ts = cloud_stamp if cloud_stamp is not None else st.pose_stamp

            # Fallbacks for image / K if not yet received.
            img = image if image is not None else st.image
            intrinsics = K if K is not None else st.info

            kf = Keyframe(
                timestamp=float(ts) if ts is not None else 0.0,
                camera_id=camera_id,
                cloud_xyz=np.asarray(cloud_xyz),
                cloud_rgb=np.asarray(cloud_rgb),
                image=np.asarray(img) if img is not None
                else np.zeros((1, 1, 3), dtype=np.uint8),
                K=np.asarray(intrinsics) if intrinsics is not None
                else np.eye(3),
                pose=pose,
                organized=bool(organized),
            )

            self._buffers[camera_id].append(kf)
            st.last_keyframe_pose = pose
            self._accepted[camera_id] += 1
            return kf

    # ------------------------------------------------------------------
    # ROI query (pure logic — the service handler serialises the result)
    # ------------------------------------------------------------------

    def get_in_roi(
        self,
        center: np.ndarray,
        radius: float,
    ) -> list[Keyframe]:
        """Return all keyframes whose camera translation is within *radius*."""
        center = np.asarray(center, dtype=np.float64).reshape(3)
        radius_sq = float(radius) * float(radius)
        result: list[Keyframe] = []
        with self._lock:
            for cam in self.cameras:
                for kf in self._buffers[cam]:
                    t = kf.translation
                    if float(np.sum((t - center) ** 2)) <= radius_sq:
                        result.append(kf)
        return result

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def diagnostics(self) -> dict:
        """Return a diagnostics snapshot (counts, memory, organisation)."""
        with self._lock:
            per_camera = {}
            total_mem_mb = 0.0
            total_count = 0
            for cam in self.cameras:
                buf = self._buffers[cam]
                count = len(buf)
                mem = sum(kf.memory_mb for kf in buf)
                total_mem_mb += mem
                total_count += count
                per_camera[cam] = {
                    "count": count,
                    "memory_mb": round(mem, 2),
                    "cloud_organized": self._state[cam].cloud_organized,
                    "accepted": self._accepted[cam],
                    "rejected_gate": self._rejected_gate[cam],
                    "rejected_no_pose": self._rejected_no_pose[cam],
                }
            return {
                "total_count": total_count,
                "total_memory_mb": round(total_mem_mb, 2),
                "max_keyframes_per_camera": self.max_keyframes_per_camera,
                "per_camera": per_camera,
            }

    def all_keyframes(self, camera_id: Optional[str] = None) -> list[Keyframe]:
        """Return a flat list of all stored keyframes (optionally one camera)."""
        with self._lock:
            cams = (camera_id,) if camera_id else self.cameras
            return [kf for cam in cams for kf in self._buffers[cam]]


# ---------------------------------------------------------------------------
# ROS 2 node — thin wrapper around KeyframeBufferCore
# ---------------------------------------------------------------------------

def _import_ros():
    """Import ROS 2 modules lazily so this file can be imported on the host
    without ROS installed (for the pure-logic core)."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2, Image, CameraInfo
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    return (rclpy, Node, QoSProfile, ReliabilityPolicy, HistoryPolicy,
            PointCloud2, Image, CameraInfo, PoseWithCovarianceStamped,
            DiagnosticArray, DiagnosticStatus, KeyValue)


def _parse_cloud2_xyzrgb(msg) -> tuple[np.ndarray, np.ndarray]:
    """Parse a ``sensor_msgs/PointCloud2`` into ``(N, 3)`` xyz + ``(N, 3)`` rgb.

    Reuses the struct-unpacking pattern from ``pointcloud_fusion_node``.
    """
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)

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
        rgb[:, 0] = (packed >> 16) & 0xFF  # R
        rgb[:, 1] = (packed >> 8) & 0xFF   # G
        rgb[:, 2] = packed & 0xFF          # B

    return xyz, rgb


def _camera_info_to_K(msg) -> np.ndarray:
    """Extract the ``(3, 3)`` intrinsics matrix from a ``CameraInfo`` message."""
    K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
    return K


def _pose_to_matrix(msg) -> np.ndarray:
    """Convert a ``Pose`` (inside PoseWithCovarianceStamped) to ``(4, 4)``."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T = np.eye(4, dtype=np.float64)
    # Use the se3_helpers quaternion→rotation if available, else inline.
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-15:
        R = np.eye(3)
    else:
        s = 2.0 / n
        xs, ys, zs = qx * s, qy * s, qz * s
        wx, wy, wz = qw * xs, qw * ys, qw * zs
        xx, xy, xz = qx * xs, qx * ys, qx * zs
        yy, yz, zz = qy * ys, qy * zs, qz * zs
        R = np.array([
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ])
    T[:3, :3] = R
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _stamp_to_float(stamp) -> float:
    """Convert a ``builtin_interfaces/Time`` to seconds (float)."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _keyframe_to_msg(kf, KeyframeMsg, PointCloud2, PointField, Image,
                     CameraInfo, Pose, Time):
    """Serialise a ``Keyframe`` dataclass into a ``sensor_fusion_msgs/Keyframe``."""
    msg = KeyframeMsg()
    ts = Time()
    sec = int(kf.timestamp)
    ts.sec = sec
    ts.nanosec = int((kf.timestamp - sec) * 1e9)
    msg.timestamp = ts
    msg.camera_id = kf.camera_id

    # Cloud → PointCloud2 (xyz + rgb)
    n = kf.num_points
    pc2 = PointCloud2()
    pc2.height = 1
    pc2.width = n
    pc2.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    pc2.is_bigendian = False
    pc2.point_step = 16
    pc2.row_step = 16 * n
    pc2.is_dense = True
    xyz = kf.cloud_xyz.reshape(n, 3).astype(np.float32)
    # Pack RGB into a single float32 (same layout as sensor_msgs PointCloud2).
    # Use explicit dtype=np.uint32 in sum() — numpy's default accumulator
    # upcasts to uint64, which would break the .view(np.float32) reinterpret.
    rgb_u32 = kf.cloud_rgb.reshape(n, 3).astype(np.uint32)
    rgb_packed = (
        rgb_u32[:, 0] * np.uint32(1 << 16)
        + rgb_u32[:, 1] * np.uint32(1 << 8)
        + rgb_u32[:, 2]
    ).astype(np.uint32)
    buf = np.zeros((n, 4), dtype=np.float32)
    buf[:, :3] = xyz
    buf[:, 3] = rgb_packed.view(np.float32)
    pc2.data = np.ascontiguousarray(buf).tobytes()
    msg.cloud = pc2

    # Image
    img = Image()
    img.height = int(kf.image.shape[0])
    img.width = int(kf.image.shape[1])
    img.encoding = "rgb8"
    img.step = img.width * 3
    img.data = np.ascontiguousarray(kf.image.astype(np.uint8)).tobytes()
    msg.image = img

    # Pose
    pose = Pose()
    pose.position.x = float(kf.pose[0, 3])
    pose.position.y = float(kf.pose[1, 3])
    pose.position.z = float(kf.pose[2, 3])
    q = rotation_matrix_to_quaternion(kf.pose[:3, :3])
    pose.orientation.x = float(q[0])
    pose.orientation.y = float(q[1])
    pose.orientation.z = float(q[2])
    pose.orientation.w = float(q[3])
    msg.pose = pose

    # CameraInfo
    info = CameraInfo()
    info.k = kf.K.reshape(-1).tolist()
    msg.camera_info = info

    msg.organized = bool(kf.organized)
    return msg


def create_node():
    """Build and return the ``KeyframeBufferNode`` (ROS 2 Node subclass).

    Kept as a factory so the module can be imported without ROS installed.
    """
    (rclpy, Node, QoSProfile, ReliabilityPolicy, HistoryPolicy,
     PointCloud2, Image, CameraInfo, PoseWithCovarianceStamped,
     DiagnosticArray, DiagnosticStatus, KeyValue) = _import_ros()

    from sensor_msgs.msg import PointField
    try:
        from sensor_fusion_msgs.msg import Keyframe as KeyframeMsg
        from sensor_fusion_msgs.srv import GetKeyframesInROI
        from sensor_fusion_msgs.srv import GetAllKeyframes
    except ImportError:
        KeyframeMsg = None  # type: ignore
        GetKeyframesInROI = None  # type: ignore
        GetAllKeyframes = None  # type: ignore
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import Pose

    class KeyframeBufferNode(Node):
        """ROS 2 node wrapping :class:`KeyframeBufferCore`."""

        def __init__(self):
            super().__init__("keyframe_buffer")

            # ── Declare parameters (defaults from V6 §6.9) ────────────────
            for key, default in DEFAULT_PARAMS.items():
                self.declare_parameter(key, default)

            p = lambda k: self.get_parameter(k).value  # noqa: E731

            self._core = KeyframeBufferCore(
                max_keyframes_per_camera=int(p("max_keyframes_per_camera")),
                spatial_gate_translation_m=float(
                    p("spatial_gate_translation_m")),
                spatial_gate_rotation_deg=float(
                    p("spatial_gate_rotation_deg")),
                pose_max_age_s=float(p("pose_max_age_s")),
            )

            # ── QoS (BEST_EFFORT — Jetson relay publishes with BEST_EFFORT) ──
            best_effort = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )

            # ── Independent subscriptions per camera ──────────────────────
            for cam in CAMERAS:
                cloud_topic = p(f"{cam}_cloud_topic")
                image_topic = p(f"{cam}_image_topic")
                info_topic = p(f"{cam}_info_topic")
                pose_topic = p(f"{cam}_pose_topic")

                self.create_subscription(
                    PointCloud2, cloud_topic,
                    lambda msg, c=cam: self._on_cloud(msg, c), best_effort)
                self.create_subscription(
                    Image, image_topic,
                    lambda msg, c=cam: self._on_image(msg, c), best_effort)
                self.create_subscription(
                    CameraInfo, info_topic,
                    lambda msg, c=cam: self._on_info(msg, c), best_effort)
                self.create_subscription(
                    PoseWithCovarianceStamped, pose_topic,
                    lambda msg, c=cam: self._on_pose(msg, c), 10)

            # ── Service: GetKeyframesInROI ────────────────────────────
            if GetKeyframesInROI is not None:
                self._srv = self.create_service(
                    GetKeyframesInROI,
                    "~/get_in_roi",
                    self._handle_get_in_roi,
                )
                self.get_logger().info(
                    "Service advertised: /keyframe_buffer/get_in_roi")
            else:
                self._srv = None
                self.get_logger().warn(
                    "sensor_fusion_msgs not available — GetKeyframesInROI "
                    "service disabled (build sensor_fusion_msgs first)")

            # ── Service: GetAllKeyframes (scene-preview fusion) ─────────
            if GetAllKeyframes is not None:
                self._srv_all = self.create_service(
                    GetAllKeyframes,
                    "~/get_all",
                    self._handle_get_all_keyframes,
                )
                self.get_logger().info(
                    "Service advertised: /keyframe_buffer/get_all")
            else:
                self._srv_all = None
                self.get_logger().warn(
                    "sensor_fusion_msgs not available — GetAllKeyframes "
                    "service disabled (build sensor_fusion_msgs first)")

            # ── Diagnostics publisher ────────────────────────────────────
            self._diag_pub = self.create_publisher(
                DiagnosticArray, "~/diagnostics", 10)
            self.create_timer(5.0, self._publish_diagnostics)

            self._use_receive_time = (
                str(p("cloud_timestamp_source")).strip().lower() == "receive_time"
            )

            self.get_logger().info(
                f"KeyframeBufferNode ready "
                f"(max_kf/cam={self._core.max_keyframes_per_camera}, "
                f"gate_t={self._core.spatial_gate_translation_m}m, "
                f"gate_r={np.rad2deg(self._core.spatial_gate_rotation_rad):.1f}deg, "
                f"cloud_ts={'RECEIVE_TIME' if self._use_receive_time else 'header'})")

        # ── Callbacks ──────────────────────────────────────────────────

        def _on_cloud(self, msg: PointCloud2, camera_id: str):
            # Cloud organisation auto-detection (V6 §5.5 — mandatory)
            organized = self._core.detect_cloud_organization(
                camera_id, msg.height)
            state = self._core._state[camera_id]
            if state.cloud_organized and organized and \
                    state.cloud is None and msg.height > 1:
                # First organised cloud — log once.
                pass
            if state.cloud is None:
                self.get_logger().info(
                    f"{camera_id} cloud: height={msg.height} -> "
                    f"{'ORGANIZED' if organized else 'UNORGANIZED'}")

            stamp = (
                time.time() if self._use_receive_time
                else _stamp_to_float(msg.header.stamp)
            )
            self._core._state[camera_id].cloud = msg
            self._core._state[camera_id].cloud_stamp = stamp

            # Parse + attempt keyframe creation
            xyz, rgb = _parse_cloud2_xyzrgb(msg)
            image = self._core._state[camera_id].image
            # st.info is already a (3,3) K matrix (converted in _on_info)
            K = self._core._state[camera_id].info

            kf = self._core.try_create_keyframe(
                camera_id, xyz, rgb, image, K, cloud_stamp=stamp)
            if kf is not None:
                self.get_logger().debug(
                    f"[{camera_id}] keyframe accepted @ t={stamp:.3f} "
                    f"({kf.num_points} pts, {kf.memory_mb:.2f} MB)")

        def _on_image(self, msg: Image, camera_id: str):
            stamp = _stamp_to_float(msg.header.stamp)
            # Decode to (H, W, 3) uint8 RGB
            try:
                enc = msg.encoding
                raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
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
                self._core.update_image(camera_id, arr.copy(), stamp)
            except Exception as exc:
                self.get_logger().warn(
                    f"[{camera_id}] image decode failed ({enc}): {exc}",
                    throttle_duration_sec=10.0)

        def _on_info(self, msg: CameraInfo, camera_id: str):
            stamp = _stamp_to_float(msg.header.stamp)
            K = _camera_info_to_K(msg)
            self._core.update_info(camera_id, K, stamp)

        def _on_pose(self, msg: PoseWithCovarianceStamped, camera_id: str):
            stamp = _stamp_to_float(msg.header.stamp)
            T = _pose_to_matrix(msg)
            self._core.update_pose(camera_id, T, stamp)

        # ── Service handler ────────────────────────────────────────────

        def _handle_get_in_roi(self, request, response):
            center = np.array([
                request.center.x, request.center.y, request.center.z],
                dtype=np.float64)
            radius = float(request.radius)
            keyframes = self._core.get_in_roi(center, radius)

            if KeyframeMsg is not None:
                response.keyframes = [
                    _keyframe_to_msg(
                        kf, KeyframeMsg, PointCloud2, PointField, Image,
                        CameraInfo, Pose, Time)
                    for kf in keyframes
                ]
            response.count = len(keyframes)
            self.get_logger().info(
                f"GetKeyframesInROI(center={center.tolist()}, "
                f"r={radius:.3f}) -> {response.count} keyframes")
            return response

        def _handle_get_all_keyframes(self, request, response):
            """Return all stored keyframes (optionally one camera).

            Used by the TSDF scene-preview node, which integrates the full
            workspace without a hit point.
            """
            camera_id = str(request.camera_id).strip() if request.camera_id else None
            keyframes = self._core.all_keyframes(camera_id=camera_id)

            if KeyframeMsg is not None:
                response.keyframes = [
                    _keyframe_to_msg(
                        kf, KeyframeMsg, PointCloud2, PointField, Image,
                        CameraInfo, Pose, Time)
                    for kf in keyframes
                ]
            response.count = len(keyframes)
            self.get_logger().info(
                f"GetAllKeyframes(camera_id={camera_id!r}) -> "
                f"{response.count} keyframes")
            return response

        # ── Diagnostics ────────────────────────────────────────────────

        def _publish_diagnostics(self):
            diag = self._core.diagnostics()
            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "keyframe_buffer"
            status.hardware_id = "keyframe_buffer"
            total_mb = diag["total_memory_mb"]
            total_count = diag["total_count"]
            # OK if under the 350 MB budget, WARN if approaching, ERROR if over.
            if total_mb < 350.0:
                status.level = DiagnosticStatus.OK
            elif total_mb < 450.0:
                status.level = DiagnosticStatus.WARN
            else:
                status.level = DiagnosticStatus.ERROR
            status.message = (
                f"{total_count} keyframes, {total_mb:.1f} MB")
            status.values = [
                KeyValue(key="total_count", value=str(total_count)),
                KeyValue(key="total_memory_mb", value=f"{total_mb:.2f}"),
                KeyValue(
                    key="max_keyframes_per_camera",
                    value=str(diag["max_keyframes_per_camera"])),
            ]
            for cam in CAMERAS:
                pc = diag["per_camera"].get(cam, {})
                status.values.append(KeyValue(
                    key=f"{cam}_count", value=str(pc.get("count", 0))))
                status.values.append(KeyValue(
                    key=f"{cam}_memory_mb",
                    value=f"{pc.get('memory_mb', 0.0):.2f}"))
                status.values.append(KeyValue(
                    key=f"{cam}_organized",
                    value=str(pc.get("cloud_organized"))))
                status.values.append(KeyValue(
                    key=f"{cam}_accepted",
                    value=str(pc.get("accepted", 0))))
                status.values.append(KeyValue(
                    key=f"{cam}_rejected_gate",
                    value=str(pc.get("rejected_gate", 0))))
            msg.status = [status]
            self._diag_pub.publish(msg)

    return KeyframeBufferNode


def main(args=None):
    """Entry point for the ``keyframe_buffer_node`` console script."""
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
