#!/usr/bin/env python3

"""ROS2 node wrapping the marker co-observation graph (marker_graph_core.py).

Subscribes to head and arm MarkerPoseObservation topics, maintains a graph of
co-observed marker pairs, and publishes the inferred head-to-arm IMU transform
through multi-hop chains of co-observed markers.
"""

from __future__ import annotations

from collections import defaultdict, deque
import json
import math
from typing import Any, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_fusion_msgs.msg import MarkerPoseObservation
from std_msgs.msg import String

from marker_graph_core import (
    CameraObservation,
    MarkerGraph,
    PathResult,
    covariance_from_diag,
    diag_from_covariance,
    fill_pose,
    pose_to_T,
    stamp_to_sec,
)


class MarkerGraphEstimator(Node):
    def __init__(self):
        super().__init__("marker_graph_estimator")

        self.declare_parameter("head_observation_topic", "/head/marker_pose/observation")
        self.declare_parameter("arm_observation_topic", "/arm/marker_pose/observation")
        self.declare_parameter("max_edge_age_s", 30.0)
        self.declare_parameter("coobservation_time_window_s", 0.05)
        self.declare_parameter("min_edge_quality", 0.2)
        self.declare_parameter("max_hops", 10)
        self.declare_parameter("max_observation_age_s", 5.0)
        self.declare_parameter("covariance_growth_xyz_m", 0.01)
        self.declare_parameter("covariance_growth_rpy_rad", 0.0174533)
        self.declare_parameter("head_source_name", "head")
        self.declare_parameter("arm_source_name", "arm")
        self.declare_parameter("head_imu_frame", "head_imu")
        self.declare_parameter("arm_imu_frame", "arm_imu")
        self.declare_parameter("map_frame", "marker_map")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("observation_buffer_s", 0.5)

        self._head_topic = str(self.get_parameter("head_observation_topic").value)
        self._arm_topic = str(self.get_parameter("arm_observation_topic").value)
        self._max_edge_age_s = float(self.get_parameter("max_edge_age_s").value)
        self._co_window_s = float(self.get_parameter("coobservation_time_window_s").value)
        self._min_edge_quality = float(self.get_parameter("min_edge_quality").value)
        self._max_hops = int(self.get_parameter("max_hops").value)
        self._max_obs_age_s = float(self.get_parameter("max_observation_age_s").value)
        self._head_source = str(self.get_parameter("head_source_name").value)
        self._arm_source = str(self.get_parameter("arm_source_name").value)
        self._head_imu_frame = str(self.get_parameter("head_imu_frame").value)
        self._arm_imu_frame = str(self.get_parameter("arm_imu_frame").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._obs_buffer_s = float(self.get_parameter("observation_buffer_s").value)

        cov_xyz = float(self.get_parameter("covariance_growth_xyz_m").value)
        cov_rpy = float(self.get_parameter("covariance_growth_rpy_rad").value)
        self._cov_growth_diag = np.array(
            [cov_xyz**2, cov_xyz**2, cov_xyz**2, cov_rpy**2, cov_rpy**2, cov_rpy**2],
            dtype=float,
        )

        self._graph = MarkerGraph(
            max_edge_age_s=self._max_edge_age_s,
            min_edge_quality=self._min_edge_quality,
        )

        self._obs_buffers: dict[str, deque[CameraObservation]] = defaultdict(
            lambda: deque()
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self._head_sub = self.create_subscription(
            MarkerPoseObservation,
            self._head_topic,
            lambda msg: self._observation_cb(self._head_source, msg),
            qos,
        )
        self._arm_sub = self.create_subscription(
            MarkerPoseObservation,
            self._arm_topic,
            lambda msg: self._observation_cb(self._arm_source, msg),
            qos,
        )

        self._head_to_arm_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "~/head_to_arm",
            10,
        )
        self._status_pub = self.create_publisher(String, "~/status", 10)
        self._edges_pub = self.create_publisher(String, "~/edges", 10)

        period = 1.0 / max(self._publish_rate_hz, 0.1)
        self._publish_timer = self.create_timer(period, self._publish_cb)
        self._prune_timer = self.create_timer(1.0, self._prune_cb)

        self._last_path_result: Optional[PathResult] = None

        self.get_logger().info(f"Head observation topic: {self._head_topic}")
        self.get_logger().info(f"Arm observation topic: {self._arm_topic}")
        self.get_logger().info(f"Head source name: {self._head_source}")
        self.get_logger().info(f"Arm source name: {self._arm_source}")
        self.get_logger().info(f"Max edge age: {self._max_edge_age_s:.1f}s")
        self.get_logger().info(
            f"Co-observation window: {self._co_window_s * 1e3:.0f}ms"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _msg_to_observation(
        self, source: str, msg: MarkerPoseObservation
    ) -> CameraObservation:
        stamp_sec = stamp_to_sec(msg.header.stamp)
        position_err = float(msg.reprojection_error_px)
        geo_score = float(msg.geometry_score)
        stable = bool(msg.stable)
        quality = geo_score * (1.0 / max(1.0 + position_err, 1.0))
        if stable:
            quality = min(1.0, quality * 1.2)
        else:
            quality *= 0.7
        quality = max(0.0, min(1.0, quality))

        return CameraObservation(
            stamp_sec=stamp_sec,
            marker_id=int(msg.marker_id),
            marker_frame=str(msg.marker_frame),
            target_frame=str(msg.target_frame),
            T_map_imu=pose_to_T(msg.pose.pose),
            covariance_diag=diag_from_covariance(msg.pose.covariance),
            geometry_score=geo_score,
            reprojection_error_px=position_err,
            stable=stable,
            quality=quality,
        )

    def _observation_cb(self, source: str, msg: MarkerPoseObservation) -> None:
        if not msg.hard_gate_passed:
            return

        obs = self._msg_to_observation(source, msg)
        self._graph.update_camera_observation(source, obs)

        buffer = self._obs_buffers[source]
        buffer.append(obs)

        co_obs = self._find_co_observations(source, obs.stamp_sec)
        if len(co_obs) >= 2:
            new_edges = self._graph.add_co_observations(source, co_obs)
            if new_edges:
                self.get_logger().debug(
                    f"Added {len(new_edges)} edges from {source}: {new_edges}"
                )

        self._prune_buffers()

    def _find_co_observations(
        self, source: str, stamp_sec: float
    ) -> list[CameraObservation]:
        buffer = self._obs_buffers.get(source)
        if not buffer:
            return []
        co_list = []
        for obs in buffer:
            if abs(obs.stamp_sec - stamp_sec) < self._co_window_s:
                if not any(o.marker_id == obs.marker_id for o in co_list):
                    co_list.append(obs)
        return co_list

    def _prune_buffers(self) -> None:
        now = self._now_sec()
        cutoff = now - self._obs_buffer_s
        for source in list(self._obs_buffers.keys()):
            buf = self._obs_buffers[source]
            while buf and buf[0].stamp_sec < cutoff:
                buf.popleft()
            if not buf:
                del self._obs_buffers[source]

    def _prune_cb(self) -> None:
        now = self._now_sec()
        removed = self._graph.prune_stale_edges(now)
        if removed:
            self.get_logger().debug(
                f"Pruned {len(removed)} stale edges: {removed}"
            )

    def _publish_cb(self) -> None:
        now = self._now_sec()

        result = self._graph.query_head_to_arm(
            head_source=self._head_source,
            arm_source=self._arm_source,
            cov_growth_diag=self._cov_growth_diag,
            max_obs_age_s=self._max_obs_age_s,
            max_hops=self._max_hops,
            now_sec=now,
        )
        self._last_path_result = result

        self._publish_head_to_arm(result)
        self._publish_status(now, result)
        self._publish_edges(now)

    def _publish_head_to_arm(self, result: PathResult) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        if result.found and result.T_head_arm is not None:
            fill_pose(msg.pose.pose, result.T_head_arm)
            diag = (
                result.covariance_diag
                if result.covariance_diag is not None
                else np.zeros(6)
            )
            msg.pose.covariance = covariance_from_diag(np.maximum(diag, 1e-9))
        else:
            msg.pose.pose.orientation.w = 1.0
            msg.pose.covariance = covariance_from_diag(np.full(6, 999.0))
        self._head_to_arm_pub.publish(msg)

    def _publish_status(self, now_sec: float, result: PathResult) -> None:
        edges = self._graph.get_edges()
        components = self._graph.connected_components()
        markers = self._graph.get_markers()

        head_obs = self._graph.get_all_observations(self._head_source)
        arm_obs = self._graph.get_all_observations(self._arm_source)

        payload: dict[str, Any] = {
            "stamp": round(now_sec, 6),
            "num_markers": len(markers),
            "num_edges": len(edges),
            "num_components": len(components),
            "component_sizes": sorted([len(c) for c in components], reverse=True),
            "head_markers_seen": sorted(head_obs.keys()),
            "arm_markers_seen": sorted(arm_obs.keys()),
            "shared_markers": sorted(set(head_obs.keys()) & set(arm_obs.keys())),
            "head_to_arm": {
                "found": result.found,
                "hops": result.hops,
                "chain_quality": round(result.chain_quality, 4),
                "path": result.path,
                "head_marker": result.head_marker,
                "arm_marker": result.arm_marker,
                "head_stamp_sec": round(result.head_stamp_sec, 6),
                "arm_stamp_sec": round(result.arm_stamp_sec, 6),
            },
        }

        if result.found and result.T_head_arm is not None:
            T = result.T_head_arm
            t = T[:3, 3]
            R = T[:3, :3]
            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
            yaw = math.atan2(R[1, 0], R[0, 0])
            cov = (
                result.covariance_diag
                if result.covariance_diag is not None
                else np.zeros(6)
            )
            std = np.sqrt(np.maximum(cov, 0.0))
            payload["head_to_arm"]["translation_m"] = [
                round(float(t[0]), 4),
                round(float(t[1]), 4),
                round(float(t[2]), 4),
            ]
            payload["head_to_arm"]["rotation_deg"] = [
                round(math.degrees(roll), 2),
                round(math.degrees(pitch), 2),
                round(math.degrees(yaw), 2),
            ]
            payload["head_to_arm"]["std_xyz_m"] = [
                round(float(std[0]), 4),
                round(float(std[1]), 4),
                round(float(std[2]), 4),
            ]
            payload["head_to_arm"]["std_rpy_deg"] = [
                round(math.degrees(float(std[3])), 2),
                round(math.degrees(float(std[4])), 2),
                round(math.degrees(float(std[5])), 2),
            ]

        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)

    def _publish_edges(self, now_sec: float) -> None:
        edges = self._graph.get_edges()
        edge_list = []
        for (a, b), edge in sorted(edges.items()):
            age = max(0.0, now_sec - edge.last_stamp_sec)
            edge_list.append(
                {
                    "marker_a": a,
                    "marker_b": b,
                    "quality": round(edge.quality, 4),
                    "num_observations": edge.num_observations,
                    "age_s": round(age, 3),
                }
            )

        payload = {
            "stamp": round(now_sec, 6),
            "num_edges": len(edge_list),
            "edges": edge_list,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._edges_pub.publish(msg)


def main():
    rclpy.init()
    node = MarkerGraphEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
