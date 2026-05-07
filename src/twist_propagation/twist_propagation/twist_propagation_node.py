#!/usr/bin/env python3
"""Twist Propagation Target Selector Node.

Estimates the hand twist from sequential pose messages, propagates it forward
in time against the live point cloud, and when a propagated position
intersects a cluster of points within a configurable threshold, triggers
segmentation and then grasp preshaping.

Internal state machine:
  IDLE                     – active, running propagation each cycle
  WAITING_FOR_SEGMENTATION – click published, waiting for segmented cloud
  WAITING_FOR_PRESHAPING   – segmented cloud received, preshaping called

Topics
------
Subscribe:
  /hand_pose                geometry_msgs/PoseStamped
      Current hand pose (from external hand tracking).  Frame is taken from
      the message header and used for TF2 transforms.
  /camera/depth/color/points  sensor_msgs/PointCloud2
      Raw scene point cloud for propagation intersection checks.
  /segmentation/object_cloud  sensor_msgs/PointCloud2
      Segmented cloud output — watched to know when segmentation completes.

Publish:
  /hand_twist               geometry_msgs/TwistStamped
      Estimated hand twist (linear + angular velocity).
  /segmentation/click_positive  geometry_msgs/PointStamped
      Hit point sent to the segmentation node to trigger inference.
  /twist_propagation/status    std_msgs/String
      JSON status string for monitoring / debugging.

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
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    TwistStamped,
    Vector3,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger

from scipy.spatial import KDTree

try:
    from tf2_ros import Buffer, TransformListener
    import tf2_geometry_msgs  # noqa: F401
    _HAS_TF2 = True
except ImportError:
    _HAS_TF2 = False


# ---------------------------------------------------------------------------
# Helpers
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
        self.declare_parameter("cycle_delay_s", 0.1)
        self.declare_parameter("propagation_time_horizon_s", 2.0)
        self.declare_parameter("propagation_dt_s", 0.02)
        self.declare_parameter("hit_threshold_m", 0.05)
        self.declare_parameter("min_points_near_hit", 3)
        self.declare_parameter("twist_estimation_window", 5)
        self.declare_parameter("active", False)
        self.declare_parameter("input_cloud_topic", "/camera/depth/color/points")
        self.declare_parameter("segmented_cloud_topic", "/segmentation/object_cloud")
        self.declare_parameter("hand_pose_topic", "/hand_pose")
        self.declare_parameter("click_positive_topic", "/segmentation/click_positive")
        self.declare_parameter("hand_twist_topic", "/hand_twist")
        self.declare_parameter("compute_grasp_service", "/grasp_preshaping/compute_grasp")
        self.declare_parameter("activate_service", "/twist_propagation/activate")
        self.declare_parameter("deactivate_service", "/twist_propagation/deactivate")
        self.declare_parameter("segmentation_timeout_s", 5.0)
        self.declare_parameter("cloud_max_age_s", 2.0)

        self._cycle_delay = self.get_parameter("cycle_delay_s").value
        self._horizon = self.get_parameter("propagation_time_horizon_s").value
        self._dt = self.get_parameter("propagation_dt_s").value
        self._hit_thresh = self.get_parameter("hit_threshold_m").value
        self._min_points = self.get_parameter("min_points_near_hit").value
        self._twist_window = self.get_parameter("twist_estimation_window").value
        self._seg_timeout = self.get_parameter("segmentation_timeout_s").value
        self._cloud_max_age = self.get_parameter("cloud_max_age_s").value

        # ── State ──────────────────────────────────────────────────────────
        self._active: bool = self.get_parameter("active").value
        self._cycle_state: CycleState = CycleState.IDLE
        self._lock = threading.Lock()

        # Pose buffer: deque of (timestamp_s, px, py, pz, qx, qy, qz, qw, frame_id)
        self._pose_buf: deque[tuple] = deque(maxlen=max(self._twist_window + 1, 2))

        # Latest estimated twist (linear vx,vy,vz; angular wx,wy,wz)
        self._twist: tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # Latest raw cloud
        self._cloud_xyz: np.ndarray | None = None
        self._cloud_stamp: float = 0.0
        self._cloud_frame: str = ""
        self._cloud_kdtree: KDTree | None = None

        # Segmented cloud tracking
        self._seg_cloud_stamp: float = 0.0
        self._seg_cloud_stamp_at_trigger: float = 0.0
        self._seg_trigger_time: float = 0.0

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

        self.create_subscription(PoseStamped, hand_pose_topic, self._on_hand_pose, 10)
        self.create_subscription(PointCloud2, input_cloud_topic, self._on_input_cloud, 10)
        self.create_subscription(PointCloud2, seg_cloud_topic, self._on_segmented_cloud, 10)

        # ── Publishers ─────────────────────────────────────────────────────
        twist_topic = self.get_parameter("hand_twist_topic").value
        click_topic = self.get_parameter("click_positive_topic").value

        self._twist_pub = self.create_publisher(
            TwistStamped, twist_topic, 10)
        self._click_pub = self.create_publisher(
            PointStamped, click_topic, 10)
        self._status_pub = self.create_publisher(
            String, "/twist_propagation/status", 10)

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
            f"Twist propagation node started — active={self._active}, "
            f"horizon={self._horizon}s, dt={self._dt}s, "
            f"hit_thresh={self._hit_thresh}m"
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

    def _on_input_cloud(self, msg: PointCloud2):
        xyz = _parse_xyz(msg)
        if xyz is None or len(xyz) == 0:
            return
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

    def _propagate_and_find_hit(self, pose: tuple, twist: tuple) -> tuple | None:
        """Propagate the pose forward and check for cloud intersection.

        Returns (hit_x, hit_y, hit_z) in the cloud frame, or None.
        """
        tree = self._get_kdtree()
        if tree is None:
            return None

        px, py, pz = pose[0], pose[1], pose[2]
        qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
        vx, vy, vz, wx, wy, wz = twist

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

            # Check distance to nearest cloud points
            dists, _ = tree.query([px, py, pz], k=max(self._min_points, 1))
            if np.max(dists[:self._min_points]) <= self._hit_thresh:
                return (px, py, pz)

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

        ps = PointStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.point.x = px
        ps.point.y = py
        ps.point.z = pz

        try:
            transformed = self._tf_buffer.transform(
                ps, self._cloud_frame,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            return (transformed.point.x, transformed.point.y, transformed.point.z)
        except Exception as exc:
            self.get_logger().warn(
                f"TF transform from '{frame_id}' to '{self._cloud_frame}' "
                f"failed: {exc}. Using raw coordinates."
            )
            return (px, py, pz)

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

        with self._lock:
            active = self._active
            state = self._cycle_state

            if not active:
                self._publish_status()
                return

            # ── State: WAITING_FOR_SEGMENTATION ────────────────────────────
            if state == CycleState.WAITING_FOR_SEGMENTATION:
                elapsed = time.time() - self._seg_trigger_time
                if self._seg_cloud_stamp > self._seg_cloud_stamp_at_trigger:
                    # New segmented cloud arrived — transition and call preshaping
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
                else:
                    self.get_logger().debug(
                        f"Waiting for segmentation ({elapsed:.1f}s)"
                    )
                self._publish_status()

            # ── State: WAITING_FOR_PRESHAPING ──────────────────────────────
            elif state == CycleState.WAITING_FOR_PRESHAPING:
                # Just wait — the future callback will transition back to IDLE
                self._publish_status()

            # ── State: IDLE — run propagation ──────────────────────────────
            else:
                self._run_idle_cycle()

        # ── Outside the lock: call preshaping service if needed ─────────────
        if should_call_preshaping:
            self._call_preshaping_service()

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

        # Estimate twist
        self._twist = self._estimate_twist()

        # Get latest pose and its frame
        latest = self._pose_buf[-1]
        _, px, py, pz, qx, qy, qz, qw, pose_frame = latest

        # Transform hand pose position to the cloud frame via TF2
        transformed = self._transform_pose_to_cloud_frame(px, py, pz, pose_frame)
        px_cloud, py_cloud, pz_cloud = transformed

        # Publish twist in the hand pose's frame
        self._publish_twist(self._twist, pose_frame)

        # Propagate and find hit (in cloud frame)
        hit = self._propagate_and_find_hit(
            (px_cloud, py_cloud, pz_cloud, qx, qy, qz, qw),
            self._twist,
        )

        if hit is not None:
            hit_x, hit_y, hit_z = hit
            self.get_logger().info(
                f"Hit found at ({hit_x:.3f}, {hit_y:.3f}, {hit_z:.3f}) "
                f"in frame '{self._cloud_frame}'"
            )

            # Publish click to trigger segmentation
            click = PointStamped()
            click.header.stamp = self.get_clock().now().to_msg()
            click.header.frame_id = self._cloud_frame
            click.point.x = hit_x
            click.point.y = hit_y
            click.point.z = hit_z
            self._click_pub.publish(click)

            # Transition to waiting for segmentation
            self._seg_cloud_stamp_at_trigger = self._seg_cloud_stamp
            self._seg_trigger_time = time.time()
            self._cycle_state = CycleState.WAITING_FOR_SEGMENTATION

            self._publish_status(
                hit_point=[round(hit_x, 4), round(hit_y, 4), round(hit_z, 4)],
            )
        else:
            # No hit — publish status with twist info
            vx, vy, vz, wx, wy, wz = self._twist
            lin_mag = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
            self._publish_status(
                reason="no_hit",
                twist_linear_mag=round(lin_mag, 4),
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
