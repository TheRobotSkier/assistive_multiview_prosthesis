#!/usr/bin/env python3
"""Future Pose Prediction and Pointcloud Collision/Proximity Checker Node.

Predicts the future trajectory of the prosthetic arm using a constant-twist
motion model and checks for proximity/collision against live pointcloud data.

Architecture
------------
- Subscribes to odometry (nav_msgs/Odometry, ~200 Hz) and stores the latest.
- Subscribes to pointcloud (PointCloud2, ~8 Hz).  Every accepted pointcloud
  message triggers one prediction + collision check cycle.
- A separate status timer publishes periodic health telemetry only (never
  triggers prediction).

Topics
------
Subscribe:
  /ov_msckf_arm/odomimu              nav_msgs/Odometry
      Arm pose + twist from OpenVINS (~200 Hz).
  /arm/d435i_arm/points_marker_map   sensor_msgs/PointCloud2
      Scene pointcloud in marker_map frame (~8 Hz).  This is the sole
      trigger for prediction/click output.

Publish:
  /prediction/future_trajectory      sensor_fusion_msgs/FuturePoseTrajectory
  /prediction/future_trajectory/status  std_msgs/String (JSON)
  /segmentation/click_positive       geometry_msgs/PointStamped
  /segmentation/click_positive/status   std_msgs/String (JSON)
"""

from __future__ import annotations

import json
import math
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from sensor_fusion_msgs.msg import FuturePoseTrajectory


# ---------------------------------------------------------------------------
# Pure helpers — testable without rclpy
# ---------------------------------------------------------------------------

def parse_xyz_from_cloud(cloud_msg: PointCloud2) -> np.ndarray | None:
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
    mask = np.isfinite(xyz).all(axis=1)
    return xyz[mask]


def voxel_downsample(xyz: np.ndarray, leaf_m: float) -> np.ndarray:
    if leaf_m <= 0.0 or len(xyz) == 0:
        return xyz
    inv_leaf = 1.0 / leaf_m
    voxel_indices = np.floor(xyz * inv_leaf).astype(np.int64)
    _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
    return xyz[unique_idx]


def is_odom_initialized(odom_msg: Odometry) -> bool:
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


def quat_multiply(q0: tuple, q1: tuple) -> tuple:
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return (
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
    )


def propagate_pose(
    px: float, py: float, pz: float,
    qx: float, qy: float, qz: float, qw: float,
    vx: float, vy: float, vz: float,
    wx: float, wy: float, wz: float,
    dt: float,
) -> tuple:
    npx = px + vx * dt
    npy = py + vy * dt
    npz = pz + vz * dt

    ang = math.sqrt((wx * dt) ** 2 + (wy * dt) ** 2 + (wz * dt) ** 2)
    if ang > 0.0:
        axis_x = wx * dt / ang
        axis_y = wy * dt / ang
        axis_z = wz * dt / ang
        half = ang / 2.0
        s = math.sin(half)
        dq = (axis_x * s, axis_y * s, axis_z * s, math.cos(half))
    else:
        dq = (0.0, 0.0, 0.0, 1.0)

    nqx, nqy, nqz, nqw = quat_multiply((qx, qy, qz, qw), dq)
    norm = math.sqrt(nqx ** 2 + nqy ** 2 + nqz ** 2 + nqw ** 2)
    nqx /= norm
    nqy /= norm
    nqz /= norm
    nqw /= norm

    return (npx, npy, npz, nqx, nqy, nqz, nqw)


def build_initial_covariance(
    odom_msg: Odometry,
    process_noise_linear_mps2_per_s: float,
    process_noise_angular_radps2_per_s: float,
) -> np.ndarray:
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


def propagate_covariance(
    P: np.ndarray,
    dt: float,
    sigma_v_sq: float,
    sigma_w_sq: float,
) -> np.ndarray:
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


def check_proximity_hit(
    positions: np.ndarray,
    kdtree,
    hit_threshold_m: float,
    min_points_near_hit: int,
    collision_radius_m: float = 0.0,
) -> np.ndarray | None:
    effective_threshold = hit_threshold_m + collision_radius_m
    for i in range(len(positions)):
        pos = positions[i]
        dists, _ = kdtree.query(pos, k=min_points_near_hit)
        if np.max(dists) <= effective_threshold:
            return pos
    return None


def extract_covariance_diag(P: np.ndarray) -> list:
    return [
        float(P[0, 0]), float(P[1, 1]), float(P[2, 2]),
        float(P[3, 3]), float(P[4, 4]), float(P[5, 5]),
    ]


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class FuturePosePredictionCollisionNode(Node):

    def __init__(self):
        super().__init__("future_pose_prediction_collision")

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter("enable_prediction", False)
        self.declare_parameter("enable_collision_check", False)
        self.declare_parameter("horizon_s", 1.0)
        self.declare_parameter("dt_s", 0.02)
        self.declare_parameter("hit_threshold_m", 0.05)
        self.declare_parameter("min_points_near_hit", 3)
        self.declare_parameter("odom_max_age_s", 0.1)
        self.declare_parameter("cloud_max_age_s", 2.0)
        self.declare_parameter("max_prediction_covariance_trace", 100.0)
        self.declare_parameter("process_noise_linear_mps2_per_s", 0.1)
        self.declare_parameter("process_noise_angular_radps2_per_s", 0.5)
        self.declare_parameter("collision_geometry_type", "sphere")
        self.declare_parameter("collision_geometry_radius_m", 0.10)
        self.declare_parameter("collision_geometry_frame", "arm_imu")
        self.declare_parameter("odom_topic", "/ov_msckf_arm/odomimu")
        self.declare_parameter("cloud_topic", "/arm/d435i_arm/points_marker_map")
        self.declare_parameter("trajectory_topic", "/prediction/future_trajectory")
        self.declare_parameter("click_topic", "/segmentation/click_positive")
        self.declare_parameter("voxel_leaf_m", 0.02)
        self.declare_parameter("status_period_s", 1.0)

        self._enable_prediction = self.get_parameter("enable_prediction").value
        self._enable_collision_check = self.get_parameter("enable_collision_check").value
        self._horizon_s = self.get_parameter("horizon_s").value
        self._dt_s = self.get_parameter("dt_s").value
        self._hit_threshold_m = self.get_parameter("hit_threshold_m").value
        self._min_points_near_hit = self.get_parameter("min_points_near_hit").value
        self._odom_max_age_s = self.get_parameter("odom_max_age_s").value
        self._cloud_max_age_s = self.get_parameter("cloud_max_age_s").value
        self._max_cov_trace = self.get_parameter("max_prediction_covariance_trace").value
        self._sigma_v_sq = self.get_parameter("process_noise_linear_mps2_per_s").value
        self._sigma_w_sq = self.get_parameter("process_noise_angular_radps2_per_s").value
        self._odom_topic = self.get_parameter("odom_topic").value
        self._cloud_topic = self.get_parameter("cloud_topic").value
        self._trajectory_topic = self.get_parameter("trajectory_topic").value
        self._click_topic = self.get_parameter("click_topic").value
        self._voxel_leaf_m = self.get_parameter("voxel_leaf_m").value
        self._status_period_s = self.get_parameter("status_period_s").value
        self._collision_radius = self.get_parameter("collision_geometry_radius_m").value

        # ── State ──────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._latest_odom: Odometry | None = None
        self._cloud_xyz: np.ndarray | None = None
        self._cloud_stamp_sec: float = 0.0
        self._cloud_frame_id: str = ""
        self._last_trigger_time_s: float = 0.0
        self._last_status: dict = {"accepted": False, "reason": "waiting_for_cloud"}

        # ── Subscriptions ──────────────────────────────────────────────────
        self.create_subscription(
            Odometry, self._odom_topic, self._on_odom, 10)

        self.create_subscription(
            PointCloud2, self._cloud_topic, self._on_cloud, 10)

        # ── Publishers ─────────────────────────────────────────────────────
        self._trajectory_pub = self.create_publisher(
            FuturePoseTrajectory, self._trajectory_topic, 10)
        self._trajectory_status_pub = self.create_publisher(
            String, f"{self._trajectory_topic}/status", 10)
        self._click_pub = self.create_publisher(
            PointStamped, self._click_topic, 10)
        self._click_status_pub = self.create_publisher(
            String, f"{self._click_topic}/status", 10)
        self._path_pub = self.create_publisher(
            Path, f"{self._trajectory_topic}/path", 10)

        # ── Status timer (health telemetry only — never triggers prediction)
        status_period_ns = self._status_period_s
        self.create_timer(status_period_ns, self._publish_health_status)

        self.get_logger().info(
            f"Future pose prediction node started — "
            f"prediction={self._enable_prediction}, "
            f"collision={self._enable_collision_check}, "
            f"horizon={self._horizon_s}s, dt={self._dt_s}s"
        )

    # ── Subscription callbacks ─────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._latest_odom = msg

    def _on_cloud(self, msg: PointCloud2):
        if not self._enable_prediction and not self._enable_collision_check:
            return

        xyz = parse_xyz_from_cloud(msg)
        if xyz is None or len(xyz) == 0:
            self._last_status = {"accepted": False, "reason": "empty_cloud"}
            return

        cloud_stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        with self._lock:
            odom = self._latest_odom

        # ── Staleness / init checks ─────────────────────────────────────
        now_s = self.get_clock().now().nanoseconds / 1e9

        cloud_age = now_s - cloud_stamp_sec
        if cloud_age > self._cloud_max_age_s:
            self._last_status = {
                "accepted": False,
                "reason": "cloud_too_old",
                "cloud_age_s": round(cloud_age, 4),
            }
            return

        if odom is None:
            self._last_status = {"accepted": False, "reason": "waiting_for_odom"}
            return

        odom_stamp_sec = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        odom_age = cloud_stamp_sec - odom_stamp_sec
        if abs(odom_age) > self._odom_max_age_s:
            self._last_status = {
                "accepted": False,
                "reason": "odom_too_old",
                "odom_age_s": round(odom_age, 4),
            }
            return

        if not is_odom_initialized(odom):
            self._last_status = {"accepted": False, "reason": "odom_uninitialized"}
            return

        # ── Downsample and build KDTree ─────────────────────────────────
        if self._voxel_leaf_m > 0.0:
            xyz = voxel_downsample(xyz, self._voxel_leaf_m)

        self._cloud_xyz = xyz
        self._cloud_stamp_sec = cloud_stamp_sec
        self._cloud_frame_id = msg.header.frame_id

        from scipy.spatial import KDTree
        try:
            kdtree = KDTree(xyz)
        except Exception as exc:
            self.get_logger().warn(f"Failed to build KDTree: {exc}")
            self._last_status = {"accepted": False, "reason": "kdtree_build_failed"}
            return

        # ── Extract current state ───────────────────────────────────────
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        t = odom.twist.twist

        px, py, pz = p.x, p.y, p.z
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        vx, vy, vz = t.linear.x, t.linear.y, t.linear.z
        wx, wy, wz = t.angular.x, t.angular.y, t.angular.z

        # ── Build initial covariance (with twist floor) ─────────────────
        P = build_initial_covariance(
            odom, self._sigma_v_sq, self._sigma_w_sq)

        # ── Propagate ───────────────────────────────────────────────────
        num_steps = max(1, int(self._horizon_s / self._dt_s))
        dt = self._dt_s

        poses = []
        cov_diags = []
        positions = np.zeros((num_steps, 3), dtype=np.float64)
        hit_found = False
        hit_point = None
        hit_step = -1

        cur_px, cur_py, cur_pz = px, py, pz
        cur_qx, cur_qy, cur_qz, cur_qw = qx, qy, qz, qw
        cur_P = P.copy()

        for step in range(num_steps):
            cur_px, cur_py, cur_pz, cur_qx, cur_qy, cur_qz, cur_qw = propagate_pose(
                cur_px, cur_py, cur_pz,
                cur_qx, cur_qy, cur_qz, cur_qw,
                vx, vy, vz, wx, wy, wz,
                dt,
            )
            cur_P = propagate_covariance(
                cur_P, dt, self._sigma_v_sq, self._sigma_w_sq)

            cov_trace = float(np.trace(cur_P))
            if cov_trace > self._max_cov_trace:
                self.get_logger().warn(
                    f"Covariance trace {cov_trace:.2f} exceeds max "
                    f"{self._max_cov_trace} at step {step}, truncating horizon"
                )
                break

            from geometry_msgs.msg import Pose
            pose = Pose()
            pose.position.x = float(cur_px)
            pose.position.y = float(cur_py)
            pose.position.z = float(cur_pz)
            pose.orientation.x = float(cur_qx)
            pose.orientation.y = float(cur_qy)
            pose.orientation.z = float(cur_qz)
            pose.orientation.w = float(cur_qw)
            poses.append(pose)

            d = extract_covariance_diag(cur_P)
            cov_diags.extend(d)

            positions[step, 0] = cur_px
            positions[step, 1] = cur_py
            positions[step, 2] = cur_pz

            if self._enable_collision_check and not hit_found:
                result = check_proximity_hit(
                    np.array([[cur_px, cur_py, cur_pz]]),
                    kdtree,
                    self._hit_threshold_m,
                    self._min_points_near_hit,
                    self._collision_radius,
                )
                if result is not None:
                    hit_found = True
                    hit_point = result
                    hit_step = step

        # ── Publish trajectory ──────────────────────────────────────────
        if self._enable_prediction and len(poses) > 0:
            traj = FuturePoseTrajectory()
            traj.header.stamp = msg.header.stamp
            traj.header.frame_id = msg.header.frame_id
            traj.poses = poses
            traj.covariance_diag = cov_diags
            traj.horizon_s = float(self._horizon_s)
            traj.dt_s = float(self._dt_s)
            traj.odom_source_topic = self._odom_topic
            traj.cloud_source_topic = self._cloud_topic
            traj.odom_age_s = float(odom_age)
            self._trajectory_pub.publish(traj)

            path = Path()
            path.header.stamp = msg.header.stamp
            path.header.frame_id = msg.header.frame_id
            path.poses = [
                PoseStamped(header=path.header, pose=p) for p in poses
            ]
            self._path_pub.publish(path)

        # ── Publish click ───────────────────────────────────────────────
        if hit_found and hit_point is not None:
            click = PointStamped()
            click.header.stamp = msg.header.stamp
            click.header.frame_id = msg.header.frame_id
            click.point.x = float(hit_point[0])
            click.point.y = float(hit_point[1])
            click.point.z = float(hit_point[2])
            self._click_pub.publish(click)

            self._last_status = {
                "accepted": True,
                "reason": "hit_found",
                "odom_age_s": round(odom_age, 4),
                "cloud_age_s": round(cloud_age, 4),
                "hit_step": hit_step,
                "hit_point": [round(float(hit_point[0]), 4),
                              round(float(hit_point[1]), 4),
                              round(float(hit_point[2]), 4)],
                "num_poses": len(poses),
                "num_cloud_points": int(len(xyz)),
            }
        else:
            self._last_status = {
                "accepted": True,
                "reason": "no_hit" if self._enable_collision_check else "prediction_only",
                "odom_age_s": round(odom_age, 4),
                "cloud_age_s": round(cloud_age, 4),
                "num_poses": len(poses),
                "num_cloud_points": int(len(xyz)),
            }

        self._last_trigger_time_s = time.time()

        self._publish_trajectory_status(self._last_status)

        if self._enable_collision_check:
            click_status = {
                "click_published": hit_found,
                "reason": self._last_status.get("reason", ""),
                "odom_age_s": round(odom_age, 4),
            }
            self._publish_click_status(click_status)

    # ── Status publishers ─────────────────────────────────────────────────

    def _publish_trajectory_status(self, status: dict):
        msg = String()
        msg.data = json.dumps(status)
        self._trajectory_status_pub.publish(msg)

    def _publish_click_status(self, status: dict):
        msg = String()
        msg.data = json.dumps(status)
        self._click_status_pub.publish(msg)

    def _publish_health_status(self):
        with self._lock:
            odom = self._latest_odom

        now_s = time.time()
        health = {
            "enable_prediction": self._enable_prediction,
            "enable_collision_check": self._enable_collision_check,
            "last_trigger_age_s": round(now_s - self._last_trigger_time_s, 3)
            if self._last_trigger_time_s > 0.0 else -1.0,
            "last_status": self._last_status,
        }

        if odom is not None:
            odom_stamp = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
            health["odom_age_s"] = round(now_s - odom_stamp, 4)
        else:
            health["odom_age_s"] = -1.0

        if self._cloud_xyz is not None:
            health["cloud_age_s"] = round(now_s - self._cloud_stamp_sec, 4)
            health["cloud_points"] = int(len(self._cloud_xyz))
        else:
            health["cloud_age_s"] = -1.0
            health["cloud_points"] = 0

        msg = String()
        msg.data = json.dumps(health)
        self._trajectory_status_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = FuturePosePredictionCollisionNode()
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
