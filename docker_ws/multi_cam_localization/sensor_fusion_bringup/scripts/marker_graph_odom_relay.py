#!/usr/bin/env python3

"""ROS2 node: marker-graph-corrected odometry relay.

Subscribes to the marker graph head-to-arm constraint, head and arm OpenVINS
odometry, and publishes a corrected arm odometry topic.  Falls back to raw arm
odom pass-through when the graph has no valid constraint.

--- Architecture decision ---
TF-corrected odometry relay (non-invasive):
  - The marker graph provides T_head_arm through marker co-observation chains.
  - The relay applies T_head_arm to head odometry to produce a corrected arm
    pose in the head's odometry frame.
  - Baseline OpenVINS topics remain untouched; this is a downstream consumer.
  - Covariances are combined conservatively (element-wise max of diagonals).

--- Topics ---
Subscriptions:
  /marker_graph_estimator/head_to_arm (PoseWithCovarianceStamped)
  /marker_graph_estimator/status      (String, JSON)
  /ov_msckf/odomimu                   (Odometry) – head
  /ov_msckf_arm/odomimu               (Odometry) – arm

Publications:
  /ov_msckf_arm/odomimu_corrected     (Odometry) – corrected arm odom
  /marker_graph/correction_status     (String, JSON) – diagnostics

--- Monitoring ---
  ros2 topic echo /marker_graph/correction_status
  (fields: stamp, graph_found, graph_chain_quality, correction_active,
   correction_mode, correction_translation_m, ages_s, ...)
"""

from __future__ import annotations

import json
import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def _odom_pose_to_T(odom: Odometry) -> np.ndarray:
    """Extract 4x4 rigid transform from an Odometry message."""
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    R = _quat_xyzw_to_R(q.x, q.y, q.z, q.w)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _T_to_pose(T: np.ndarray, pose_msg):
    """Fill a ROS Pose message from a 4x4 transform."""
    qx, qy, qz, qw = _rot_to_quat_xyzw(T[:3, :3])
    pose_msg.position.x = float(T[0, 3])
    pose_msg.position.y = float(T[1, 3])
    pose_msg.position.z = float(T[2, 3])
    pose_msg.orientation.x = float(qx)
    pose_msg.orientation.y = float(qy)
    pose_msg.orientation.z = float(qz)
    pose_msg.orientation.w = float(qw)


def _quat_xyzw_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n <= 0.0 or not math.isfinite(n):
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=float,
    )


def _rot_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    tr = np.trace(R)
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
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

    q = np.array([qx, qy, qz, qw], dtype=float)
    norm = np.linalg.norm(q)
    if norm <= 0.0 or not math.isfinite(norm):
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


def _odom_cov_diag(odom: Odometry) -> np.ndarray:
    return np.maximum(np.diag(np.array(odom.pose.covariance).reshape(6, 6)), 0.0)


def _graph_cov_diag(msg: PoseWithCovarianceStamped) -> np.ndarray:
    return np.maximum(np.diag(np.array(msg.pose.covariance).reshape(6, 6)), 0.0)


def _diag_to_cov(diag6: np.ndarray) -> list[float]:
    cov = [0.0] * 36
    values = np.asarray(diag6, dtype=float).reshape(6)
    for i in range(6):
        cov[6 * i + i] = float(max(values[i], 0.0))
    return cov


def _rotation_angle_deg(R: np.ndarray) -> float:
    cos_angle = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) * 0.5))
    return math.degrees(math.acos(cos_angle))


class MarkerGraphOdomRelay(Node):
    def __init__(self):
        super().__init__("marker_graph_odom_relay")

        self.declare_parameter("head_to_arm_topic", "/marker_graph_estimator/head_to_arm")
        self.declare_parameter("graph_status_topic", "/marker_graph_estimator/status")
        self.declare_parameter("head_odom_topic", "/ov_msckf/odomimu")
        self.declare_parameter("arm_odom_topic", "/ov_msckf_arm/odomimu")
        self.declare_parameter("corrected_odom_topic", "/ov_msckf_arm/odomimu_corrected")
        self.declare_parameter("correction_status_topic", "/marker_graph/correction_status")
        self.declare_parameter("corrected_child_frame_id", "arm_imu_corrected")
        self.declare_parameter("min_graph_chain_quality", 0.2)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_head_odom_age_s", 0.5)
        self.declare_parameter("max_arm_odom_age_s", 0.5)
        self.declare_parameter("max_graph_age_s", 1.0)

        self._head_to_arm_topic = str(self.get_parameter("head_to_arm_topic").value)
        self._graph_status_topic = str(self.get_parameter("graph_status_topic").value)
        self._head_odom_topic = str(self.get_parameter("head_odom_topic").value)
        self._arm_odom_topic = str(self.get_parameter("arm_odom_topic").value)
        self._corrected_odom_topic = str(self.get_parameter("corrected_odom_topic").value)
        self._correction_status_topic = str(self.get_parameter("correction_status_topic").value)
        self._corrected_child_frame_id = str(self.get_parameter("corrected_child_frame_id").value)
        self._min_quality = float(self.get_parameter("min_graph_chain_quality").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._max_head_age = float(self.get_parameter("max_head_odom_age_s").value)
        self._max_arm_age = float(self.get_parameter("max_arm_odom_age_s").value)
        self._max_graph_age = float(self.get_parameter("max_graph_age_s").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._graph_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self._head_to_arm_topic,
            self._graph_cb,
            qos_reliable,
        )
        self._graph_status_sub = self.create_subscription(
            String,
            self._graph_status_topic,
            self._graph_status_cb,
            qos_reliable,
        )
        self._head_odom_sub = self.create_subscription(
            Odometry,
            self._head_odom_topic,
            self._head_odom_cb,
            qos,
        )
        self._arm_odom_sub = self.create_subscription(
            Odometry,
            self._arm_odom_topic,
            self._arm_odom_cb,
            qos,
        )

        self._corrected_pub = self.create_publisher(Odometry, self._corrected_odom_topic, 10)
        self._status_pub = self.create_publisher(String, self._correction_status_topic, 10)

        period = 1.0 / max(self._publish_rate_hz, 0.1)
        self._timer = self.create_timer(period, self._publish_cb)

        self._latest_graph_msg: Optional[PoseWithCovarianceStamped] = None
        self._latest_graph_stamp: float = 0.0
        self._graph_found: bool = False
        self._graph_chain_quality: float = 0.0
        self._graph_hops: int = 0
        self._graph_path: list[int] = []
        self._latest_head_odom: Optional[Odometry] = None
        self._head_stamp: float = 0.0
        self._latest_arm_odom: Optional[Odometry] = None
        self._arm_stamp: float = 0.0

        self.get_logger().info("Marker graph odometry relay started")
        self.get_logger().info(f"  Graph topic: {self._head_to_arm_topic}")
        self.get_logger().info(f"  Head odom:   {self._head_odom_topic}")
        self.get_logger().info(f"  Arm odom:    {self._arm_odom_topic}")
        self.get_logger().info(f"  Corrected:   {self._corrected_odom_topic}")

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _graph_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_graph_msg = msg
        self._latest_graph_stamp = stamp_to_sec(msg.header.stamp)
        cov_diag = _graph_cov_diag(msg)
        self._graph_found = float(np.sum(cov_diag)) < 500.0

    def _graph_status_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            hta = data.get("head_to_arm", {})
            self._graph_found = bool(hta.get("found", False))
            self._graph_chain_quality = float(hta.get("chain_quality", 0.0))
            self._graph_hops = int(hta.get("hops", 0))
            self._graph_path = list(hta.get("path", []))
        except (json.JSONDecodeError, TypeError, ValueError):
            pass

    def _head_odom_cb(self, msg: Odometry) -> None:
        self._latest_head_odom = msg
        self._head_stamp = stamp_to_sec(msg.header.stamp)

    def _arm_odom_cb(self, msg: Odometry) -> None:
        self._latest_arm_odom = msg
        self._arm_stamp = stamp_to_sec(msg.header.stamp)

    def _publish_cb(self) -> None:
        now = self._now_sec()
        graph_age = now - self._latest_graph_stamp if self._latest_graph_msg else float("inf")
        head_age = now - self._head_stamp if self._latest_head_odom else float("inf")
        arm_age = now - self._arm_stamp if self._latest_arm_odom else float("inf")

        correction_mode = "none"
        correction_active = False
        correction_t = [0.0, 0.0, 0.0]

        out_msg: Optional[Odometry] = None

        graph_valid = (
            self._graph_found
            and self._graph_chain_quality >= self._min_quality
            and self._latest_graph_msg is not None
            and graph_age <= self._max_graph_age
        )

        if graph_valid and self._latest_head_odom is not None and head_age <= self._max_head_age:
            out_msg = self._build_corrected_odom(self._latest_head_odom, self._latest_graph_msg)
            correction_mode = "corrected"
            correction_active = True
            if out_msg is not None:
                T = _odom_pose_to_T(out_msg)
                correction_t = [float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]

        if out_msg is None and self._latest_arm_odom is not None and arm_age <= self._max_arm_age:
            out_msg = self._build_pass_through_odom(self._latest_arm_odom)
            correction_mode = "pass_through"
            correction_active = False

        if out_msg is not None:
            self._corrected_pub.publish(out_msg)

        self._publish_diagnostics(now, correction_mode, correction_active, correction_t,
                                  head_age, arm_age, graph_age)

    def _build_corrected_odom(
        self, head_odom: Odometry, graph_msg: PoseWithCovarianceStamped
    ) -> Optional[Odometry]:
        try:
            T_odom_head = _odom_pose_to_T(head_odom)
            T_head_arm = _graph_pose_to_T(graph_msg)
        except (ValueError, IndexError):
            return None

        T_odom_arm = T_odom_head @ T_head_arm

        out = Odometry()
        out.header.stamp = head_odom.header.stamp
        out.header.frame_id = head_odom.header.frame_id
        out.child_frame_id = self._corrected_child_frame_id
        _T_to_pose(T_odom_arm, out.pose.pose)

        head_cov = _odom_cov_diag(head_odom)
        graph_cov = _graph_cov_diag(graph_msg)
        combined_cov = np.maximum(head_cov, graph_cov)
        out.pose.covariance = _diag_to_cov(combined_cov)

        out.twist = head_odom.twist
        twist_cov = np.full(6, 10.0)
        out.twist.covariance = _diag_to_cov(twist_cov)

        return out

    def _build_pass_through_odom(self, arm_odom: Odometry) -> Odometry:
        out = Odometry()
        out.header = arm_odom.header
        out.child_frame_id = self._corrected_child_frame_id
        out.pose = arm_odom.pose
        out.twist = arm_odom.twist
        return out

    def _publish_diagnostics(
        self,
        now: float,
        mode: str,
        active: bool,
        trans: list[float],
        head_age: float,
        arm_age: float,
        graph_age: float,
    ) -> None:
        payload = {
            "stamp": round(now, 6),
            "graph_found": self._graph_found,
            "graph_chain_quality": round(self._graph_chain_quality, 4),
            "graph_hops": self._graph_hops,
            "graph_path": self._graph_path,
            "correction_active": active,
            "correction_mode": mode,
            "correction_translation_m": [round(v, 4) for v in trans],
            "ages_s": {
                "head_odom": round(head_age, 3) if head_age < float("inf") else None,
                "arm_odom": round(arm_age, 3) if arm_age < float("inf") else None,
                "graph": round(graph_age, 3) if graph_age < float("inf") else None,
            },
            "min_quality_threshold": self._min_quality,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _graph_pose_to_T(msg: PoseWithCovarianceStamped) -> np.ndarray:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    R = _quat_xyzw_to_R(q.x, q.y, q.z, q.w)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def main():
    rclpy.init()
    node = MarkerGraphOdomRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
