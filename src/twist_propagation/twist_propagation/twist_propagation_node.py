#!/usr/bin/env python3
"""Twist Propagation Target Selector Node.

Estimates the hand twist from sequential pose messages, propagates it forward
in time against the live point cloud, and when a propagated position
intersects a cluster of points within a configurable threshold, triggers
segmentation and then grasp preshaping.

Internal state machine:
  IDLE                     -- active, running propagation each cycle
  WAITING_FOR_SEGMENTATION -- click published, waiting for segmented cloud
  WAITING_FOR_PRESHAPING   -- segmented cloud received, preshaping called

Topics
------
Subscribe:
  /hand_pose                geometry_msgs/PoseStamped
      Current hand pose (from external hand tracking).  Frame is taken from
      the message header and used for TF2 transforms.
  /camera/depth/color/points  sensor_msgs/PointCloud2
      Raw scene point cloud for propagation intersection checks.
  /segmentation/object_cloud  sensor_msgs/PointCloud2
      Segmented cloud output -- watched to know when segmentation completes.

Publish:
  /hand_twist               geometry_msgs/TwistStamped
      Estimated hand twist (linear + angular velocity).
  /segmentation/click_positive  geometry_msgs/PointStamped
      Hit point sent to the segmentation node to trigger inference.
  /twist_propagation/status    std_msgs/String
      JSON status string for monitoring / debugging.
  /twist_propagation/predicted_path  nav_msgs/Path
      Predicted future trajectory as a path for RViz visualization.
  /twist_propagation/current_pose   geometry_msgs/PoseStamped
      Current hand pose in the cloud frame.
  /twist_propagation/collision_spheres  visualization_msgs/MarkerArray
      Semi-transparent spheres along the predicted path showing collision geometry.
  /twist_propagation/hit_marker  visualization_msgs/Marker
      Persistent green sphere at the predicted collision point.
  /twist_propagation/trajectory_line  visualization_msgs/Marker
      Line strip connecting predicted positions, color-coded by collision state.

Services:
  /twist_propagation/activate    std_srvs/Trigger
  /twist_propagation/deactivate  std_srvs/Trigger

Service clients:
  /grasp_preshaping/compute_grasp  std_srvs/Trigger
"""

from __future__ import annotations

import enum
import json
import math
import threading
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    TwistStamped,
    Vector3,
)
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String, ColorRGBA, Empty
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from scipy.spatial import KDTree

try:
    from tf2_ros import Buffer, TransformListener
    import tf2_geometry_msgs  # noqa: F401
    _HAS_TF2 = True
except ImportError:
    _HAS_TF2 = False


# ---------------------------------------------------------------------------
# Helpers -- pure functions, testable without rclpy
# ---------------------------------------------------------------------------

def _parse_xyz(cloud_msg: PointCloud2) -> np.ndarray | None:
    """Extract an (N, 3) float32 XYZ array from a PointCloud2 message."""
    n = cloud_msg.width * cloud_msg.height
    if n == 0:
        return None
    step = cloud_msg.point_step
    raw = np.frombuffer(bytes(cloud_msg.data), dtype=np.uint8).reshape(n, step)

    fields = {f.name: f for f in cloud_msg.fields}
    for name in ("x", "y", "z"):
        if name not in fields:
            return None

    def _col(name: str) -> np.ndarray:
        off = fields[name].offset
        return np.frombuffer(raw[:, off:off + 4].copy().tobytes(), dtype=np.float32)

    xyz = np.column_stack([_col("x"), _col("y"), _col("z")])
    # Filter NaN / Inf
    mask = np.isfinite(xyz).all(axis=1)
    return xyz[mask]


def _voxel_downsample(xyz: np.ndarray, leaf_m: float) -> np.ndarray:
    """Downsample an (N, 3) point array by keeping one point per voxel cube.

    Each voxel is a cube of side ``leaf_m`` metres.  Points falling in the
    same voxel are collapsed to the first point encountered (deterministic
    because ``np.unique`` with ``return_index=True`` is used).
    """
    if leaf_m <= 0.0 or len(xyz) == 0:
        return xyz
    inv_leaf = 1.0 / leaf_m
    voxel_indices = np.floor(xyz * inv_leaf).astype(np.int64)
    _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
    return xyz[unique_idx]


def _quat_multiply(q0: tuple, q1: tuple) -> tuple:
    """Hamilton product of two quaternions (x, y, z, w)."""
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return (
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
    )


def _quat_to_rotation_vec(q: tuple) -> tuple:
    """Convert quaternion (x, y, z, w) to rotation vector (axis * angle)."""
    x, y, z, w = q
    # Ensure w >= 0 for canonical form
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    angle = 2.0 * math.acos(min(w, 1.0))
    s = math.sin(angle / 2.0)
    if s < 1e-10:
        return (0.0, 0.0, 0.0)
    return (x / s * angle, y / s * angle, z / s * angle)


def _rotation_vec_to_quat(rv: tuple) -> tuple:
    """Convert rotation vector (axis * angle) to quaternion (x, y, z, w)."""
    angle = math.sqrt(rv[0] ** 2 + rv[1] ** 2 + rv[2] ** 2)
    if angle < 1e-10:
        return (0.0, 0.0, 0.0, 1.0)
    ha = angle / 2.0
    s = math.sin(ha) / angle
    return (rv[0] * s, rv[1] * s, rv[2] * s, math.cos(ha))


def _quat_diff(q1: tuple, q0: tuple) -> tuple:
    """Rotation vector from q0 to q1: q_diff = q0^{-1} * q1."""
    # Inverse of q0
    ix, iy, iz, iw = -q0[0], -q0[1], -q0[2], q0[3]
    q_diff = _quat_multiply((ix, iy, iz, iw), q1)
    return _quat_to_rotation_vec(q_diff)


def _propagate_pose(
    px: float, py: float, pz: float,
    qx: float, qy: float, qz: float, qw: float,
    vx: float, vy: float, vz: float,
    wx: float, wy: float, wz: float,
    dt: float,
) -> tuple:
    """Propagate a pose by a twist for time dt.

    Returns (px, py, pz, qx, qy, qz, qw) after propagation.
    """
    # Linear part
    npx = px + vx * dt
    npy = py + vy * dt
    npz = pz + vz * dt

    # Angular part: small rotation by omega * dt
    half_angle = math.sqrt((wx * dt) ** 2 + (wy * dt) ** 2 + (wz * dt) ** 2) / 2.0
    if half_angle > 1e-10:
        s = math.sin(half_angle) / (2.0 * half_angle)
        dq = (wx * dt * s, wy * dt * s, wz * dt * s, math.cos(half_angle))
    else:
        dq = (0.0, 0.0, 0.0, 1.0)

    nqx, nqy, nqz, nqw = _quat_multiply((qx, qy, qz, qw), dq)
    # Normalize
    norm = math.sqrt(nqx ** 2 + nqy ** 2 + nqz ** 2 + nqw ** 2)
    if norm > 1e-10:
        nqx /= norm
        nqy /= norm
        nqz /= norm
        nqw /= norm

    return (npx, npy, npz, nqx, nqy, nqz, nqw)


# -- Covariance helpers (ported from future_pose_prediction_collision_node) --

def _build_initial_covariance_from_pose_buf(
    pose_buf: deque,
    process_noise_linear_mps2_per_s: float,
    process_noise_angular_radps2_per_s: float,
) -> np.ndarray:
    """Build a 12x12 initial state covariance from a pose buffer.

    The state vector is [px, py, pz, qx, qy, qz, vx, vy, vz, wx, wy, wz]
    (position + orientation euler + linear vel + angular vel).

    Position covariance is estimated from the scatter of the pose buffer.
    Velocity covariance uses the process noise as a floor.
    Orientation covariance is set to a conservative default since we don't
    track full orientation uncertainty from PoseStamped alone.
    """
    P = np.zeros((12, 12), dtype=np.float64)

    if len(pose_buf) >= 3:
        positions = np.array([(b[1], b[2], b[3]) for b in pose_buf])
        pos_cov = np.cov(positions.T) if len(positions) > 1 else np.eye(3) * 0.01
        P[0:3, 0:3] = pos_cov
    else:
        P[0:3, 0:3] = np.eye(3) * 0.01

    # Conservative orientation uncertainty
    P[3:6, 3:6] = np.eye(3) * 0.01

    # Velocity covariance: use process noise as floor
    sigma_v_sq = process_noise_linear_mps2_per_s
    sigma_w_sq = process_noise_angular_radps2_per_s
    P[6:9, 6:9] = np.eye(3) * sigma_v_sq
    P[9:12, 9:12] = np.eye(3) * sigma_w_sq

    return P


def _build_initial_covariance_from_odom(
    odom_msg: Odometry,
    process_noise_linear_mps2_per_s: float,
    process_noise_angular_radps2_per_s: float,
) -> np.ndarray:
    """Build a 12x12 initial state covariance from an Odometry message.

    Uses the actual pose and twist covariance from the odometry, with the
    process noise as a floor for the twist covariance.
    """
    pose_cov = np.array(odom_msg.pose.covariance, dtype=np.float64).reshape(6, 6)
    twist_cov_raw = np.array(odom_msg.twist.covariance, dtype=np.float64).reshape(6, 6)

    sigma_v_sq = process_noise_linear_mps2_per_s
    sigma_w_sq = process_noise_angular_radps2_per_s
    noise_floor = np.diag([sigma_v_sq, sigma_v_sq, sigma_v_sq,
                           sigma_w_sq, sigma_w_sq, sigma_w_sq])
    twist_cov = np.maximum(twist_cov_raw, noise_floor)

    P = np.zeros((12, 12), dtype=np.float64)
    P[0:3, 0:3] = pose_cov[0:3, 0:3]
    P[3:6, 3:6] = pose_cov[3:6, 3:6]
    P[6:9, 6:9] = twist_cov[0:3, 0:3]
    P[9:12, 9:12] = twist_cov[3:6, 3:6]
    return P


def _propagate_covariance(
    P: np.ndarray,
    dt: float,
    sigma_v_sq: float,
    sigma_w_sq: float,
) -> np.ndarray:
    """Propagate a 12x12 state covariance forward by dt.

    State: [px, py, pz, qx, qy, qz, vx, vy, vz, wx, wy, wz]
    Uses a linearised constant-velocity model.
    """
    F = np.eye(12, dtype=np.float64)
    F[0, 6] = dt
    F[1, 7] = dt
    F[2, 8] = dt
    F[3, 9] = dt
    F[4, 10] = dt
    F[5, 11] = dt

    Q = np.zeros((12, 12), dtype=np.float64)
    Q[6, 6] = dt * sigma_v_sq
    Q[7, 7] = dt * sigma_v_sq
    Q[8, 8] = dt * sigma_v_sq
    Q[9, 9] = dt * sigma_w_sq
    Q[10, 10] = dt * sigma_w_sq
    Q[11, 11] = dt * sigma_w_sq

    return F @ P @ F.T + Q


def _is_odom_initialized(odom_msg: Odometry) -> bool:
    """Check whether an Odometry message has valid, initialized data."""
    p = odom_msg.pose.pose.position
    o = odom_msg.pose.pose.orientation
    t = odom_msg.twist.twist
    for v in [p.x, p.y, p.z,
              o.x, o.y, o.z, o.w,
              t.linear.x, t.linear.y, t.linear.z,
              t.angular.x, t.angular.y, t.angular.z]:
        if not math.isfinite(v):
            return False
    cov = odom_msg.pose.covariance
    pose_cov_trace = sum(cov[i * 6 + i] for i in range(6))
    if pose_cov_trace <= 0.0:
        return False
    return True


def _should_retarget(
    hit: tuple[float, float, float],
    current_target: tuple[float, float, float] | None,
    retarget_distance_m: float,
) -> tuple[bool, bool]:
    """Decide whether to publish a click for the given hit.

    Returns
    -------
    (publish_click, publish_reset_first)
    """
    if current_target is None:
        return True, False
    dist = math.sqrt(sum((h - c) ** 2 for h, c in zip(hit, current_target)))
    if dist > retarget_distance_m:
        return True, True
    return False, False


def _sample_spherical_shell_clicks(
    centre: tuple[float, float, float],
    r_min: float,
    r_max: float,
    count: int,
    rng: np.random.Generator,
) -> list[tuple[float, float, float]]:
    """Sample N points uniformly from a spherical shell [r_min, r_max].

    Directions are sampled uniformly on the sphere. Radii are sampled
    uniformly by volume within the shell (i.e. P(r) ~ r^2, so we sample
    u ~ Uniform(0,1) and r = (u*r_max^3 + (1-u)*r_min^3)^(1/3)).

    Returns a list of (x, y, z) points in the same frame as centre.
    """
    if count <= 0 or r_max <= r_min:
        return []

    cx, cy, cz = centre
    # Sample directions uniformly on the sphere
    # Method: sample u,v ~ Uniform, then spherical coordinates
    # theta = arccos(2*v - 1), phi = 2*pi*u
    u = rng.random(count)
    v = rng.random(count)
    theta = np.arccos(2.0 * v - 1.0)  # polar angle from z-axis
    phi = 2.0 * np.pi * u              # azimuthal angle

    # Sample radii uniformly by volume in [r_min, r_max]
    # P(r) ~ r^2  =>  CDF: F(r) = (r^3 - r_min^3) / (r_max^3 - r_min^3)
    # Inverse: r = (r_min^3 + u*(r_max^3 - r_min^3))^(1/3)
    w = rng.random(count)
    r_min3 = r_min ** 3
    r_max3 = r_max ** 3
    radii = np.cbrt(r_min3 + w * (r_max3 - r_min3))

    # Convert to Cartesian offsets
    dx = radii * np.sin(theta) * np.cos(phi)
    dy = radii * np.sin(theta) * np.sin(phi)
    dz = radii * np.cos(theta)

    points = []
    for i in range(count):
        points.append((float(cx + dx[i]), float(cy + dy[i]), float(cz + dz[i])))
    return points


# ---------------------------------------------------------------------------
# Internal state enum
# ---------------------------------------------------------------------------

class CycleState(enum.Enum):
    IDLE = "IDLE"
    WAITING_FOR_SEGMENTATION = "WAITING_FOR_SEGMENTATION"
    WAITING_FOR_PRESHAPING = "WAITING_FOR_PRESHAPING"


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class TwistPropagationNode(Node):

    def __init__(self):
        super().__init__("twist_propagation")

        # ── Parameters ─────────────────────────────────────────────────────
        # Core propagation
        self.declare_parameter("cycle_delay_s", 0.1)
        self.declare_parameter("propagation_time_horizon_s", 2.0)
        self.declare_parameter("propagation_dt_s", 0.02)

        # Collision geometry
        self.declare_parameter("hit_threshold_m", 0.05)
        self.declare_parameter("min_points_near_hit", 3)
        self.declare_parameter("collision_geometry_radius_m", 0.05)

        # Twist estimation
        self.declare_parameter("twist_estimation_window", 5)

        # Activation
        self.declare_parameter("active", False)

        # Topic names
        self.declare_parameter("input_cloud_topic", "/camera/depth/color/points")
        self.declare_parameter("segmented_cloud_topic", "/segmentation/object_cloud")
        self.declare_parameter("hand_pose_topic", "/hand_pose")
        self.declare_parameter("odom_topic", "")  # optional: nav_msgs/Odometry
        self.declare_parameter("click_positive_topic", "/segmentation/click_positive")
        self.declare_parameter("hand_twist_topic", "/hand_twist")
        self.declare_parameter("segmentation_reset_topic", "/segmentation/reset")

        # Segmentation retarget policy
        self.declare_parameter("segmentation_retarget_distance_m", 0.10)

        # Service names
        self.declare_parameter("compute_grasp_service", "/grasp_preshaping/compute_grasp")
        self.declare_parameter("activate_service", "/twist_propagation/activate")
        self.declare_parameter("deactivate_service", "/twist_propagation/deactivate")

        # Timeouts / staleness
        self.declare_parameter("segmentation_timeout_s", 5.0)
        self.declare_parameter("cloud_max_age_s", 2.0)
        self.declare_parameter("pose_max_age_s", 1.0)

        # Voxel downsampling
        self.declare_parameter("voxel_leaf_m", 0.02)

        # Covariance / uncertainty
        self.declare_parameter("max_prediction_covariance_trace", 100.0)
        self.declare_parameter("process_noise_linear_mps2_per_s", 0.1)
        self.declare_parameter("process_noise_angular_radps2_per_s", 0.5)
        self.declare_parameter("enable_covariance_propagation", True)

        # Propagation origin offset (in the pose's local frame)
        # Shifts the propagation start point from the tracked pose origin
        # (camera) to the grasp contact point (fingertips).  Applied by
        # rotating the offset by the pose orientation and adding to the
        # position before propagation.
        self.declare_parameter("propagation_origin_offset", [0.0, 0.0, 0.0])

        # Multi-click segmentation seeding
        self.declare_parameter("click_count", 0)
        self.declare_parameter("click_radius_m", 0.03)
        self.declare_parameter("click_min_radius_m", 0.005)
        self.declare_parameter("click_random_seed", 42)

        # ── Read parameters ────────────────────────────────────────────────
        self._cycle_delay = self.get_parameter("cycle_delay_s").value
        self._horizon = self.get_parameter("propagation_time_horizon_s").value
        self._dt = self.get_parameter("propagation_dt_s").value
        self._hit_thresh = self.get_parameter("hit_threshold_m").value
        self._min_points = self.get_parameter("min_points_near_hit").value
        self._collision_radius = self.get_parameter("collision_geometry_radius_m").value
        self._twist_window = self.get_parameter("twist_estimation_window").value
        self._seg_timeout = self.get_parameter("segmentation_timeout_s").value
        self._cloud_max_age = self.get_parameter("cloud_max_age_s").value
        self._pose_max_age = self.get_parameter("pose_max_age_s").value
        self._voxel_leaf = self.get_parameter("voxel_leaf_m").value
        self._max_cov_trace = self.get_parameter("max_prediction_covariance_trace").value
        self._sigma_v_sq = self.get_parameter("process_noise_linear_mps2_per_s").value
        self._sigma_w_sq = self.get_parameter("process_noise_angular_radps2_per_s").value
        self._enable_cov = self.get_parameter("enable_covariance_propagation").value
        self._seg_reset_topic = self.get_parameter("segmentation_reset_topic").value
        self._seg_retarget_distance = self.get_parameter("segmentation_retarget_distance_m").value

        # Propagation origin offset (3D vector in pose local frame)
        offset_raw = self.get_parameter("propagation_origin_offset").value
        self._propagation_offset = tuple(float(v) for v in offset_raw)
        if any(abs(v) > 1e-6 for v in self._propagation_offset):
            self.get_logger().info(
                f"Propagation origin offset: {self._propagation_offset} "
                f"(shifts start point in pose local frame)"
            )

        # Multi-click seeding parameters
        self._click_count = int(self.get_parameter("click_count").value)
        self._click_radius = float(self.get_parameter("click_radius_m").value)
        self._click_min_radius = float(self.get_parameter("click_min_radius_m").value)
        self._click_random_seed = int(self.get_parameter("click_random_seed").value)

        # Validation
        if self._click_count < 0:
            raise ValueError(f"click_count must be >= 0, got {self._click_count}")
        if self._click_radius < 0:
            raise ValueError(f"click_radius_m must be >= 0, got {self._click_radius}")
        if self._click_min_radius < 0:
            raise ValueError(f"click_min_radius_m must be >= 0, got {self._click_min_radius}")
        if self._click_count > 0 and self._click_min_radius >= self._click_radius:
            raise ValueError(
                f"click_min_radius_m ({self._click_min_radius}) must be < click_radius_m "
                f"({self._click_radius}) when click_count > 0"
            )

        # Effective collision threshold: hit_threshold + collision_radius
        self._effective_hit_thresh = self._hit_thresh + self._collision_radius

        # ── State ──────────────────────────────────────────────────────────
        self._active: bool = self.get_parameter("active").value
        self._cycle_state: CycleState = CycleState.IDLE
        self._lock = threading.RLock()  # RLock: _run_idle_cycle -> _propagate_and_find_hit nests

        # Pose buffer: deque of (timestamp_s, px, py, pz, qx, qy, qz, qw, frame_id)
        self._pose_buf: deque[tuple] = deque(maxlen=max(self._twist_window + 2, 2))

        # Latest estimated twist (linear vx,vy,vz; angular wx,wy,wz)
        self._twist: tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # Latest raw cloud
        self._cloud_xyz: np.ndarray | None = None
        self._cloud_stamp: float = 0.0
        self._cloud_frame: str = ""
        self._cloud_kdtree: KDTree | None = None

        # Odometry (optional, for covariance)
        self._latest_odom: Odometry | None = None

        # Segmented cloud tracking
        self._seg_cloud_stamp: float = 0.0
        self._seg_cloud_stamp_at_trigger: float = 0.0
        self._seg_trigger_time: float = 0.0

        # Current accepted segmentation target (cloud frame)
        self._current_segmentation_target: tuple[float, float, float] | None = None

        # Marker ID counters for RViz visualization
        self._sphere_marker_ns = "collision_spheres"
        self._hit_marker_ns = "hit_marker"
        self._trajectory_line_ns = "trajectory_line"

        # ── TF2 ────────────────────────────────────────────────────────────
        self._tf_buffer = None
        self._tf_listener = None
        if _HAS_TF2:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Subscriptions ──────────────────────────────────────────────────
        hand_pose_topic = self.get_parameter("hand_pose_topic").value
        input_cloud_topic = self.get_parameter("input_cloud_topic").value
        seg_cloud_topic = self.get_parameter("segmented_cloud_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self.create_subscription(PoseStamped, hand_pose_topic, self._on_hand_pose, 10)

        # RELIABLE — the fusion node publishes with RELIABLE QoS; the
        # RealSense publishers on the Jetson also use RELIABLE, so this
        # matches both paths.
        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(PointCloud2, input_cloud_topic, self._on_input_cloud, cloud_qos)
        self.create_subscription(PointCloud2, seg_cloud_topic, self._on_segmented_cloud, cloud_qos)

        # Optional odometry subscription for covariance data
        if odom_topic:
            self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        # ── Publishers ─────────────────────────────────────────────────────
        twist_topic = self.get_parameter("hand_twist_topic").value
        click_topic = self.get_parameter("click_positive_topic").value

        self._twist_pub = self.create_publisher(
            TwistStamped, twist_topic, 10)
        self._click_pub = self.create_publisher(
            PointStamped, click_topic, 10)
        self._reset_pub = self.create_publisher(
            Empty, self._seg_reset_topic, 10)
        self._status_pub = self.create_publisher(
            String, "/twist_propagation/status", 10)

        # Visualization publishers
        self._path_pub = self.create_publisher(
            Path, "/twist_propagation/predicted_path", 10)
        self._current_pose_pub = self.create_publisher(
            PoseStamped, "/twist_propagation/current_pose", 10)
        self._sphere_markers_pub = self.create_publisher(
            MarkerArray, "/twist_propagation/collision_spheres", 10)
        self._hit_marker_pub = self.create_publisher(
            MarkerArray, "/twist_propagation/hit_marker", 10)
        self._trajectory_line_pub = self.create_publisher(
            MarkerArray, "/twist_propagation/trajectory_line", 10)

        # ── Service servers ────────────────────────────────────────────────
        self.create_service(
            Trigger,
            self.get_parameter("activate_service").value,
            self._on_activate,
        )
        self.create_service(
            Trigger,
            self.get_parameter("deactivate_service").value,
            self._on_deactivate,
        )

        # ── Service client ─────────────────────────────────────────────────
        self._compute_client = self.create_client(
            Trigger,
            self.get_parameter("compute_grasp_service").value,
        )

        # ── Cycle timer ────────────────────────────────────────────────────
        self.create_timer(self._cycle_delay, self._cycle_callback)

        self.get_logger().info(
            f"Twist propagation node started -- active={self._active}, "
            f"horizon={self._horizon}s, dt={self._dt}s, "
            f"hit_thresh={self._hit_thresh}m, "
            f"collision_radius={self._collision_radius}m, "
            f"effective_thresh={self._effective_hit_thresh}m, "
            f"multi_click={self._click_count} (r={self._click_radius}m, r_min={self._click_min_radius}m)"
        )

    # ── Service callbacks ──────────────────────────────────────────────────

    def _on_activate(self, _req, resp):
        with self._lock:
            self._active = True
            self._cycle_state = CycleState.IDLE
            self._pose_buf.clear()
            self._seg_trigger_time = 0.0
        self.get_logger().info("Activated")
        resp.success = True
        resp.message = "twist_propagation activated"
        return resp

    def _on_deactivate(self, _req, resp):
        with self._lock:
            self._active = False
            self._cycle_state = CycleState.IDLE
            self._pose_buf.clear()
            self._twist = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self._seg_trigger_time = 0.0
            self._current_segmentation_target = None
        # Clear visualization markers
        self._clear_all_markers()
        self.get_logger().info("Deactivated")
        resp.success = True
        resp.message = "twist_propagation deactivated"
        return resp

    # ── Subscription callbacks ─────────────────────────────────────────────

    def _on_hand_pose(self, msg: PoseStamped):
        q = msg.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frame_id = msg.header.frame_id
        with self._lock:
            self._pose_buf.append((
                stamp,
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                q.x, q.y, q.z, q.w,
                frame_id,
            ))

    def _on_odom(self, msg: Odometry):
        """Store the latest odometry for covariance initialization."""
        with self._lock:
            self._latest_odom = msg

    def _on_input_cloud(self, msg: PointCloud2):
        xyz = _parse_xyz(msg)
        if xyz is None or len(xyz) == 0:
            return

        # Voxel downsample before storing
        if self._voxel_leaf > 0.0:
            xyz = _voxel_downsample(xyz, self._voxel_leaf)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._cloud_xyz = xyz
            self._cloud_stamp = stamp
            self._cloud_frame = msg.header.frame_id
            # Invalidate cached KDTree
            self._cloud_kdtree = None

    def _on_segmented_cloud(self, msg: PointCloud2):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._seg_cloud_stamp = stamp

    # ── Twist estimation ───────────────────────────────────────────────────

    def _estimate_twist(self) -> tuple:
        """Estimate twist from the pose buffer using finite differences.

        Uses the last two poses for a simple estimate, or a least-squares
        linear fit over the full window if enough poses are available.

        Returns (vx, vy, vz, wx, wy, wz).
        """
        buf = list(self._pose_buf)
        if len(buf) < 2:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # Simple finite difference using last two poses
        t0, px0, py0, pz0, qx0, qy0, qz0, qw0, _ = buf[-2]
        t1, px1, py1, pz1, qx1, qy1, qz1, qw1, _ = buf[-1]

        dt = t1 - t0
        if dt < 1e-6:
            return self._twist  # return previous estimate

        # Linear velocity
        vx = (px1 - px0) / dt
        vy = (py1 - py0) / dt
        vz = (pz1 - pz0) / dt

        # Angular velocity via rotation vector difference
        rv = _quat_diff(
            (qx1, qy1, qz1, qw1),
            (qx0, qy0, qz0, qw0),
        )
        wx, wy, wz = rv[0] / dt, rv[1] / dt, rv[2] / dt

        # If we have enough poses, use least-squares over the window for
        # linear velocity (smoother estimate)
        if len(buf) >= 3:
            ts = np.array([b[0] for b in buf])
            px = np.array([b[1] for b in buf])
            py = np.array([b[2] for b in buf])
            pz = np.array([b[3] for b in buf])

            # Fit linear model: p = a + v*t
            # Use relative times for numerical stability
            t_rel = ts - ts[0]
            A = np.column_stack([np.ones_like(t_rel), t_rel])
            try:
                coeffs_x, _, _, _ = np.linalg.lstsq(A, px, rcond=None)
                coeffs_y, _, _, _ = np.linalg.lstsq(A, py, rcond=None)
                coeffs_z, _, _, _ = np.linalg.lstsq(A, pz, rcond=None)
                vx = float(coeffs_x[1])
                vy = float(coeffs_y[1])
                vz = float(coeffs_z[1])
            except np.linalg.LinAlgError:
                pass  # keep finite-diff values

        # Exponential moving average for smoothing
        alpha = 0.4
        prev = self._twist
        vx = alpha * vx + (1 - alpha) * prev[0]
        vy = alpha * vy + (1 - alpha) * prev[1]
        vz = alpha * vz + (1 - alpha) * prev[2]
        wx = alpha * wx + (1 - alpha) * prev[3]
        wy = alpha * wy + (1 - alpha) * prev[4]
        wz = alpha * wz + (1 - alpha) * prev[5]

        return (vx, vy, vz, wx, wy, wz)

    # ── Twist publishing ───────────────────────────────────────────────────

    def _publish_twist(self, twist: tuple, frame_id: str):
        vx, vy, vz, wx, wy, wz = twist
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.twist.linear = Vector3(x=vx, y=vy, z=vz)
        msg.twist.angular = Vector3(x=wx, y=wy, z=wz)
        self._twist_pub.publish(msg)

    # ── KDTree management ──────────────────────────────────────────────────

    def _get_kdtree(self) -> KDTree | None:
        """Return a KDTree for the current cloud, building it if needed."""
        if self._cloud_xyz is None:
            return None
        if self._cloud_kdtree is not None:
            return self._cloud_kdtree
        try:
            self._cloud_kdtree = KDTree(self._cloud_xyz)
            return self._cloud_kdtree
        except Exception as exc:
            self.get_logger().warn(f"Failed to build KDTree: {exc}")
            return None

    # ── Propagation ────────────────────────────────────────────────────────

    def _propagate_and_find_hit(
        self, pose: tuple, twist: tuple
    ) -> tuple | None:
        """Propagate the pose forward and check for cloud intersection.

        Returns (hit_x, hit_y, hit_z) in the cloud frame, or None.
        Also populates self._last_predicted_positions for visualization.
        """
        tree = self._get_kdtree()
        if tree is None:
            return None

        px, py, pz = pose[0], pose[1], pose[2]
        qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
        vx, vy, vz, wx, wy, wz = twist

        # Build initial covariance if enabled
        P = None
        if self._enable_cov:
            with self._lock:
                odom = self._latest_odom
            if odom is not None and _is_odom_initialized(odom):
                P = _build_initial_covariance_from_odom(
                    odom, self._sigma_v_sq, self._sigma_w_sq)
            else:
                P = _build_initial_covariance_from_pose_buf(
                    self._pose_buf, self._sigma_v_sq, self._sigma_w_sq)

        # Collect predicted positions for visualization
        positions: list[tuple[float, float, float]] = []

        t = 0.0
        while t < self._horizon:
            t += self._dt
            result = _propagate_pose(
                px, py, pz, qx, qy, qz, qw,
                vx, vy, vz, wx, wy, wz,
                self._dt,
            )
            px, py, pz = result[0], result[1], result[2]
            qx, qy, qz, qw = result[3], result[4], result[5], result[6]

            # Propagate covariance if enabled
            if P is not None:
                P = _propagate_covariance(P, self._dt, self._sigma_v_sq, self._sigma_w_sq)
                cov_trace = float(np.trace(P))
                if cov_trace > self._max_cov_trace:
                    self.get_logger().debug(
                        f"Covariance trace {cov_trace:.2f} exceeds max "
                        f"{self._max_cov_trace} at t={t:.3f}s, truncating horizon"
                    )
                    break

            positions.append((px, py, pz))

            # Check distance to nearest cloud points (with collision radius)
            dists, _ = tree.query([px, py, pz], k=max(self._min_points, 1))
            if np.max(dists[:self._min_points]) <= self._effective_hit_thresh:
                self._last_predicted_positions = positions
                # Return the nearest surface point, not the sphere center
                _, nearest_idx = tree.query([px, py, pz], k=1)
                idx = int(nearest_idx) if np.ndim(nearest_idx) == 0 else int(nearest_idx[0])
                return tuple(self._cloud_xyz[idx].tolist())

        self._last_predicted_positions = positions
        return None

    # ── TF transform helper ────────────────────────────────────────────────

    def _transform_pose_to_cloud_frame(
        self, px: float, py: float, pz: float, frame_id: str
    ) -> tuple | None:
        """Transform a point from the given frame to the cloud frame via TF2.

        Returns (x, y, z) in cloud frame, or the original coordinates on
        failure.
        """
        if self._tf_buffer is None or not self._cloud_frame:
            return (px, py, pz)

        if frame_id == self._cloud_frame:
            return (px, py, pz)

        try:
            t = self._tf_buffer.lookup_transform(
                self._cloud_frame, frame_id, Time()
            )
            # Apply the transform manually to avoid blocking.
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tz = t.transform.translation.z
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w

            # Rotate the point: p_out = R * p_in + t
            # Quaternion rotation matrix (row-major).
            r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
            r01 = 2.0 * (qx * qy - qz * qw)
            r02 = 2.0 * (qx * qz + qy * qw)
            r10 = 2.0 * (qx * qy + qz * qw)
            r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
            r12 = 2.0 * (qy * qz - qx * qw)
            r20 = 2.0 * (qx * qz - qy * qw)
            r21 = 2.0 * (qy * qz + qx * qw)
            r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

            rx = r00 * px + r01 * py + r02 * pz + tx
            ry = r10 * px + r11 * py + r12 * pz + ty
            rz = r20 * px + r21 * py + r22 * pz + tz
            return (rx, ry, rz)
        except Exception as exc:
            self.get_logger().warn(
                f"TF transform from '{frame_id}' to '{self._cloud_frame}' "
                f"failed: {exc}. Using raw coordinates."
            )
            return (px, py, pz)

    # ── Visualization helpers ──────────────────────────────────────────────

    def _publish_predicted_path(self, positions: list[tuple[float, float, float]]):
        """Publish predicted positions as a nav_msgs/Path for RViz."""
        if not positions:
            return
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = self._cloud_frame
        for (px, py, pz) in positions:
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = self._cloud_frame
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.position.z = pz
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._path_pub.publish(path)

    def _publish_current_pose(self, px: float, py: float, pz: float,
                              qx: float, qy: float, qz: float, qw: float):
        """Publish the current hand pose in the cloud frame."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._cloud_frame
        msg.pose.position.x = px
        msg.pose.position.y = py
        msg.pose.position.z = pz
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._current_pose_pub.publish(msg)

    def _publish_collision_spheres(self, positions: list[tuple[float, float, float]]):
        """Publish semi-transparent red spheres along the predicted path."""
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        # Delete previous markers
        delete_marker = Marker()
        delete_marker.header.stamp = now
        delete_marker.header.frame_id = self._cloud_frame
        delete_marker.ns = self._sphere_marker_ns
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        # Publish new spheres (subsample for performance: max 50 spheres)
        step = max(1, len(positions) // 50)
        for i in range(0, len(positions), step):
            px, py, pz = positions[i]
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self._cloud_frame
            m.ns = self._sphere_marker_ns
            m.id = i // step
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = pz
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self._collision_radius * 2.0
            m.color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.2)
            m.lifetime.sec = 0
            m.lifetime.nanosec = int(self._cycle_delay * 1e9)
            markers.markers.append(m)

        self._sphere_markers_pub.publish(markers)

    def _publish_hit_marker(self, hit_x: float, hit_y: float, hit_z: float):
        """Publish a persistent green sphere at the collision point."""
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._cloud_frame
        m.ns = self._hit_marker_ns
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = hit_x
        m.pose.position.y = hit_y
        m.pose.position.z = hit_z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.06  # 6cm sphere
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.9)
        m.lifetime.sec = 2  # persist for 2 seconds
        ma = MarkerArray()
        ma.markers.append(m)
        self._hit_marker_pub.publish(ma)

    def _publish_trajectory_line(
        self, positions: list[tuple[float, float, float]], hit_found: bool
    ):
        """Publish a LINE_STRIP connecting predicted positions.

        Color-coded: green if no hit, red if collision detected.
        """
        if not positions:
            return
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._cloud_frame
        m.ns = self._trajectory_line_ns
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.01  # 1cm line width

        if hit_found:
            m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
        else:
            m.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.7)

        m.lifetime.sec = 0
        m.lifetime.nanosec = int(self._cycle_delay * 1e9)

        for (px, py, pz) in positions:
            from geometry_msgs.msg import Point
            p = Point()
            p.x = px
            p.y = py
            p.z = pz
            m.points.append(p)

        ma = MarkerArray()
        ma.markers.append(m)
        self._trajectory_line_pub.publish(ma)

    def _clear_all_markers(self):
        """Clear all visualization markers."""
        now = self.get_clock().now().to_msg()

        # Clear collision spheres
        ma = MarkerArray()
        m = Marker()
        m.header.stamp = now
        m.header.frame_id = self._cloud_frame or "world"
        m.ns = self._sphere_marker_ns
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self._sphere_markers_pub.publish(ma)

        # Clear hit marker
        m2 = Marker()
        m2.header.stamp = now
        m2.header.frame_id = self._cloud_frame or "world"
        m2.ns = self._hit_marker_ns
        m2.action = Marker.DELETEALL
        ma2 = MarkerArray()
        ma2.markers.append(m2)
        self._hit_marker_pub.publish(ma2)

        # Clear trajectory line
        m3 = Marker()
        m3.header.stamp = now
        m3.header.frame_id = self._cloud_frame or "world"
        m3.ns = self._trajectory_line_ns
        m3.action = Marker.DELETEALL
        ma3 = MarkerArray()
        ma3.markers.append(m3)
        self._trajectory_line_pub.publish(ma3)

    # ── Status publishing ──────────────────────────────────────────────────

    def _publish_status(self, **kwargs):
        status = {
            "active": self._active,
            "state": self._cycle_state.value,
            **kwargs,
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    # ── Main cycle callback ────────────────────────────────────────────────

    def _cycle_callback(self):
        # Snapshot state under lock, then decide what to do.
        # The lock is released before any blocking operations (service calls).
        should_call_preshaping = False

        try:
            with self._lock:
                active = self._active
                state = self._cycle_state

                if not active:
                    self._publish_status()
                    return

                # -- State: WAITING_FOR_SEGMENTATION ----------------------------
                if state == CycleState.WAITING_FOR_SEGMENTATION:
                    elapsed = time.time() - self._seg_trigger_time
                    if self._seg_cloud_stamp > self._seg_cloud_stamp_at_trigger:
                        # New segmented cloud arrived -- transition and call preshaping
                        self.get_logger().info(
                            "Segmented cloud received, calling preshaping service"
                        )
                        self._cycle_state = CycleState.WAITING_FOR_PRESHAPING
                        should_call_preshaping = True
                    elif elapsed > self._seg_timeout:
                        self.get_logger().warn(
                            f"Segmentation timeout ({elapsed:.1f}s), "
                            "returning to IDLE"
                        )
                        self._cycle_state = CycleState.IDLE
                        self._seg_trigger_time = 0.0
                        self._current_segmentation_target = None
                    else:
                        self.get_logger().debug(
                            f"Waiting for segmentation ({elapsed:.1f}s)"
                        )
                    self._publish_status()

                # -- State: WAITING_FOR_PRESHAPING ------------------------------
                elif state == CycleState.WAITING_FOR_PRESHAPING:
                    # Just wait -- the future callback will transition back to IDLE
                    self._publish_status()

                # -- State: IDLE -- run propagation -----------------------------
                else:
                    self._run_idle_cycle()

            # -- Outside the lock: call preshaping service if needed -------------
            if should_call_preshaping:
                self._call_preshaping_service()
        except Exception as exc:
            import traceback
            self.get_logger().error(
                f"EXCEPTION in _cycle_callback: {exc}\n{traceback.format_exc()}"
            )

    def _run_idle_cycle(self):
        """Run one propagation cycle.  Called under self._lock."""
        # Need at least 2 poses and a cloud
        if len(self._pose_buf) < 2:
            self._publish_status(reason="waiting_for_poses")
            return
        if self._cloud_xyz is None:
            self._publish_status(reason="waiting_for_cloud")
            return

        # Check cloud age using ROS time
        now_s = self.get_clock().now().nanoseconds / 1e9
        cloud_age = now_s - self._cloud_stamp
        if cloud_age > self._cloud_max_age:
            self._publish_status(
                reason="cloud_too_old",
                cloud_age_s=round(cloud_age, 2),
            )
            return

        # Check pose staleness
        latest_pose_time = self._pose_buf[-1][0]
        pose_age = now_s - latest_pose_time
        if pose_age > self._pose_max_age:
            self._publish_status(
                reason="pose_too_old",
                pose_age_s=round(pose_age, 2),
            )
            return

        # Estimate twist
        self._twist = self._estimate_twist()

        # Get latest pose and its frame
        latest = self._pose_buf[-1]
        _, px, py, pz, qx, qy, qz, qw, pose_frame = latest

        # Apply propagation origin offset (rotate offset by pose orientation,
        # then add to position).  This shifts the propagation start from the
        # tracked camera origin to the grasp contact point (fingertips).
        ox, oy, oz = self._propagation_offset
        if any(abs(v) > 1e-6 for v in (ox, oy, oz)):
            # Quaternion rotation of offset vector
            r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
            r01 = 2.0 * (qx * qy - qz * qw)
            r02 = 2.0 * (qx * qz + qy * qw)
            r10 = 2.0 * (qx * qy + qz * qw)
            r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
            r12 = 2.0 * (qy * qz - qx * qw)
            r20 = 2.0 * (qx * qz - qy * qw)
            r21 = 2.0 * (qy * qz + qx * qw)
            r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
            px += r00 * ox + r01 * oy + r02 * oz
            py += r10 * ox + r11 * oy + r12 * oz
            pz += r20 * ox + r21 * oy + r22 * oz

        # Transform hand pose position to the cloud frame via TF2
        transformed = self._transform_pose_to_cloud_frame(px, py, pz, pose_frame)
        px_cloud, py_cloud, pz_cloud = transformed

        # Publish current pose in cloud frame
        self._publish_current_pose(px_cloud, py_cloud, pz_cloud, qx, qy, qz, qw)

        # Publish twist in the hand pose's frame
        self._publish_twist(self._twist, pose_frame)

        # Propagate and find hit (in cloud frame)
        # Initialize the positions list that _propagate_and_find_hit will fill
        self._last_predicted_positions: list[tuple[float, float, float]] = []
        hit = self._propagate_and_find_hit(
            (px_cloud, py_cloud, pz_cloud, qx, qy, qz, qw),
            self._twist,
        )

        positions = self._last_predicted_positions

        # Always publish visualization (even when no hit)
        self._publish_predicted_path(positions)
        self._publish_collision_spheres(positions)
        self._publish_trajectory_line(positions, hit_found=hit is not None)

        if hit is not None:
            hit_x, hit_y, hit_z = hit
            publish_click, publish_reset = _should_retarget(
                hit, self._current_segmentation_target, self._seg_retarget_distance
            )

            if not publish_click:
                self.get_logger().debug(
                    f"Hit at ({hit_x:.3f}, {hit_y:.3f}, {hit_z:.3f}) "
                    f"discarded (within {self._seg_retarget_distance}m of current target)"
                )
                self._publish_status(
                    reason="near_existing_target_discarded",
                    hit_point=[round(hit_x, 4), round(hit_y, 4), round(hit_z, 4)],
                    current_target=[round(c, 4) for c in self._current_segmentation_target],
                    num_predicted_poses=len(positions),
                )
            else:
                self.get_logger().info(
                    f"Hit found at ({hit_x:.3f}, {hit_y:.3f}, {hit_z:.3f}) "
                    f"in frame '{self._cloud_frame}'"
                )

                if publish_reset:
                    self._reset_pub.publish(Empty())
                    self.get_logger().info("Published segmentation reset (new object)")

                # Publish hit marker
                self._publish_hit_marker(hit_x, hit_y, hit_z)

                # Publish click cluster (original hit + synthetic clicks)
                rng = np.random.default_rng(self._click_random_seed)
                synthetic = _sample_spherical_shell_clicks(
                    hit, self._click_min_radius, self._click_radius, self._click_count, rng
                )
                total_clicks = 1 + len(synthetic)

                # Publish original hit first
                click = PointStamped()
                click.header.stamp = self.get_clock().now().to_msg()
                click.header.frame_id = self._cloud_frame
                click.point.x = hit_x
                click.point.y = hit_y
                click.point.z = hit_z
                self._click_pub.publish(click)

                # Publish synthetic clicks
                for sx, sy, sz in synthetic:
                    sclick = PointStamped()
                    sclick.header.stamp = self.get_clock().now().to_msg()
                    sclick.header.frame_id = self._cloud_frame
                    sclick.point.x = sx
                    sclick.point.y = sy
                    sclick.point.z = sz
                    self._click_pub.publish(sclick)

                self.get_logger().info(
                    f"Published click cluster: {total_clicks} clicks "
                    f"(1 original + {len(synthetic)} synthetic) around hit "
                    f"({hit_x:.3f}, {hit_y:.3f}, {hit_z:.3f}), "
                    f"r=[{self._click_min_radius:.4f}, {self._click_radius:.4f}]m"
                )

                # Update current target and transition
                self._current_segmentation_target = hit
                self._seg_cloud_stamp_at_trigger = self._seg_cloud_stamp
                self._seg_trigger_time = time.time()
                self._cycle_state = CycleState.WAITING_FOR_SEGMENTATION

                self._publish_status(
                    hit_point=[round(hit_x, 4), round(hit_y, 4), round(hit_z, 4)],
                    num_predicted_poses=len(positions),
                    reset_before_click=publish_reset,
                    total_positive_clicks=total_clicks,
                    click_count=self._click_count,
                    click_radius_m=self._click_radius,
                    click_min_radius_m=self._click_min_radius,
                )
        else:
            # No hit -- publish status with twist info
            vx, vy, vz, wx, wy, wz = self._twist
            lin_mag = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
            self._publish_status(
                reason="no_hit",
                twist_linear_mag=round(lin_mag, 4),
                num_predicted_poses=len(positions),
            )

    # ── Preshaping service call ────────────────────────────────────────────

    def _call_preshaping_service(self):
        """Call the preshaping service.  NOT called under self._lock."""
        if not self._compute_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Preshaping service not available")
            with self._lock:
                self._cycle_state = CycleState.IDLE
            return

        future = self._compute_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_preshaping_response)

    def _on_preshaping_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Preshaping succeeded: {response.message[:100]}"
                )
            else:
                self.get_logger().warn(
                    f"Preshaping failed: {response.message[:100]}"
                )
        except Exception as exc:
            self.get_logger().error(f"Preshaping service error: {exc}")

        with self._lock:
            self._cycle_state = CycleState.IDLE
            self._seg_trigger_time = 0.0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = TwistPropagationNode()
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
