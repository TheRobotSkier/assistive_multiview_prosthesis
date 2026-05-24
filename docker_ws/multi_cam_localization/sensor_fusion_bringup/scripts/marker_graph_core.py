#!/usr/bin/env python3

"""Pure graph data structures and algorithms for marker co-observation graphs.

No ROS2 dependency - testable with standard Python.
Imported by marker_graph_estimator.py (the ROS2 node).
"""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
import math
from typing import Optional

import numpy as np


def T_inv(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Tout = np.eye(4)
    Tout[:3, :3] = R.T
    Tout[:3, 3] = -R.T @ t
    return Tout


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_to_R_xyzw(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n <= 0.0 or not math.isfinite(n):
        return np.eye(3)
    s = 2.0 / n
    xx = x * x * s
    yy = y * y * s
    zz = z * z * s
    xy = x * y * s
    xz = x * z * s
    yz = y * z * s
    wx = w * x * s
    wy = w * y * s
    wz = w * z * s
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=float,
    )


def rot_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
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


def pose_to_T(pose_msg) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_to_R_xyzw(pose_msg.orientation)
    T[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
    return T


def fill_pose(pose_msg, T: np.ndarray) -> None:
    qx, qy, qz, qw = rot_to_quat_xyzw(T[:3, :3])
    pose_msg.position.x = float(T[0, 3])
    pose_msg.position.y = float(T[1, 3])
    pose_msg.position.z = float(T[2, 3])
    pose_msg.orientation.x = float(qx)
    pose_msg.orientation.y = float(qy)
    pose_msg.orientation.z = float(qz)
    pose_msg.orientation.w = float(qw)


def diag_from_covariance(cov_36) -> np.ndarray:
    values = np.asarray(cov_36, dtype=float).reshape(6, 6)
    return np.maximum(np.diag(values), 0.0)


def covariance_from_diag(diag6: np.ndarray) -> list[float]:
    cov = [0.0] * 36
    values = np.asarray(diag6, dtype=float).reshape(6)
    for i in range(6):
        cov[6 * i + i] = float(max(values[i], 0.0))
    return cov


def rotation_angle_deg(R: np.ndarray) -> float:
    cos_angle = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) * 0.5))
    return math.degrees(math.acos(cos_angle))


@dataclass
class CameraObservation:
    stamp_sec: float
    marker_id: int
    marker_frame: str
    target_frame: str
    T_map_imu: np.ndarray
    covariance_diag: np.ndarray
    geometry_score: float
    reprojection_error_px: float
    stable: bool
    quality: float


@dataclass
class MarkerEdge:
    marker_a: int
    marker_b: int
    last_stamp_sec: float
    num_observations: int
    quality: float


@dataclass
class PathResult:
    found: bool
    path: list[int]
    hops: int
    chain_quality: float
    T_head_arm: Optional[np.ndarray]
    covariance_diag: Optional[np.ndarray]
    head_marker: int
    arm_marker: int
    head_stamp_sec: float
    arm_stamp_sec: float


class MarkerGraph:
    def __init__(self, max_edge_age_s: float = 30.0, min_edge_quality: float = 0.0):
        self._adjacency: dict[int, set[int]] = defaultdict(set)
        self._edges: dict[tuple[int, int], MarkerEdge] = {}
        self._max_edge_age_s = max_edge_age_s
        self._min_edge_quality = min_edge_quality
        self._camera_latest: dict[str, dict[int, CameraObservation]] = defaultdict(dict)

    def _edge_key(self, a: int, b: int) -> tuple[int, int]:
        return (min(a, b), max(a, b))

    def has_edge(self, a: int, b: int) -> bool:
        key = self._edge_key(a, b)
        return key in self._edges and self._edges[key].quality >= self._min_edge_quality

    def get_edge(self, a: int, b: int) -> Optional[MarkerEdge]:
        return self._edges.get(self._edge_key(a, b))

    def get_edges(self) -> dict[tuple[int, int], MarkerEdge]:
        return dict(self._edges)

    def get_markers(self) -> set[int]:
        markers = set()
        markers.update(self._adjacency.keys())
        for source, obs_dict in self._camera_latest.items():
            markers.update(obs_dict.keys())
        return markers

    def update_camera_observation(self, source: str, obs: CameraObservation) -> None:
        existing = self._camera_latest[source].get(obs.marker_id)
        if existing is None or obs.stamp_sec >= existing.stamp_sec:
            self._camera_latest[source][obs.marker_id] = obs

    def get_latest_observation(self, source: str, marker_id: int) -> Optional[CameraObservation]:
        return self._camera_latest.get(source, {}).get(marker_id)

    def get_all_observations(self, source: str) -> dict[int, CameraObservation]:
        return dict(self._camera_latest.get(source, {}))

    def add_co_observations(
        self,
        source: str,
        observations: list[CameraObservation],
    ) -> list[tuple[int, int]]:
        new_edges = []
        ids = sorted(set(obs.marker_id for obs in observations))
        if len(ids) < 2:
            return new_edges

        stamp_sec = max(obs.stamp_sec for obs in observations)

        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                a, b = ids[i], ids[j]
                obs_a = next(o for o in observations if o.marker_id == a)
                obs_b = next(o for o in observations if o.marker_id == b)
                avg_quality = 0.5 * (obs_a.quality + obs_b.quality)

                key = self._edge_key(a, b)
                existing = self._edges.get(key)
                if existing is None:
                    edge = MarkerEdge(
                        marker_a=a,
                        marker_b=b,
                        last_stamp_sec=stamp_sec,
                        num_observations=1,
                        quality=avg_quality,
                    )
                    self._edges[key] = edge
                    self._adjacency[a].add(b)
                    self._adjacency[b].add(a)
                    new_edges.append((a, b))
                else:
                    alpha = 0.3
                    existing.quality = (1.0 - alpha) * existing.quality + alpha * avg_quality
                    existing.last_stamp_sec = max(existing.last_stamp_sec, stamp_sec)
                    existing.num_observations += 1

        return new_edges

    def prune_stale_edges(self, now_sec: float) -> list[tuple[int, int]]:
        removed = []
        stale_keys = []
        cutoff = now_sec - self._max_edge_age_s
        for key, edge in self._edges.items():
            if edge.last_stamp_sec < cutoff:
                stale_keys.append(key)
        for key in stale_keys:
            a, b = key
            self._adjacency[a].discard(b)
            self._adjacency[b].discard(a)
            if not self._adjacency[a]:
                del self._adjacency[a]
            if not self._adjacency[b]:
                del self._adjacency[b]
            del self._edges[key]
            removed.append(key)
        return removed

    def connected_components(self) -> list[set[int]]:
        visited = set()
        components = []
        all_markers = self.get_markers()
        for marker in sorted(all_markers):
            if marker not in visited:
                component = self._bfs_component(marker)
                visited.update(component)
                components.append(component)
        return components

    def _bfs_component(self, start: int) -> set[int]:
        visited = set()
        queue = deque([start])
        while queue:
            node = queue.popleft()
            if node in visited:
                continue
            visited.add(node)
            for neighbor in self._adjacency.get(node, set()):
                if neighbor not in visited:
                    queue.append(neighbor)
        return visited

    def shortest_path(self, a: int, b: int) -> list[int]:
        if a == b:
            return [a]
        if a not in self._adjacency and b not in self._adjacency:
            return []

        visited = {a: None}
        queue = deque([a])
        while queue:
            node = queue.popleft()
            if node == b:
                path = []
                cur = b
                while cur is not None:
                    path.append(cur)
                    cur = visited[cur]
                path.reverse()
                return path
            for neighbor in self._adjacency.get(node, set()):
                if neighbor not in visited:
                    visited[neighbor] = node
                    queue.append(neighbor)
        return []

    def compute_ranked_path(
        self,
        source_marker: int,
        target_marker: int,
        max_hops: int = 10,
    ) -> Optional[list[int]]:
        if source_marker == target_marker:
            return [source_marker]
        if self._adjacency.get(source_marker) is None and self._adjacency.get(target_marker) is None:
            return None

        predecessor: dict[int, Optional[int]] = {source_marker: None}
        quality: dict[int, float] = {source_marker: 1.0}
        visited: set[int] = set()
        pending: list[tuple[float, int]] = [(1.0, source_marker)]

        while pending:
            pending.sort(key=lambda x: (-x[0], x[1]))
            node_quality, node = pending.pop(0)
            if node in visited:
                continue
            visited.add(node)
            if node == target_marker:
                path = []
                cur: Optional[int] = target_marker
                while cur is not None:
                    path.append(cur)
                    cur = predecessor[cur]
                path.reverse()
                return path
            for neighbor in self._adjacency.get(node, set()):
                if neighbor in visited:
                    continue
                edge = self._edges.get(self._edge_key(node, neighbor))
                edge_q = edge.quality if edge else 1.0
                new_q = node_quality * edge_q
                if neighbor not in quality or new_q > quality[neighbor]:
                    quality[neighbor] = new_q
                    predecessor[neighbor] = node
                    pending.append((new_q, neighbor))
        return None

    def query_head_to_arm(
        self,
        head_source: str,
        arm_source: str,
        cov_growth_diag: np.ndarray,
        max_obs_age_s: float = 5.0,
        max_hops: int = 10,
        now_sec: Optional[float] = None,
    ) -> PathResult:
        empty_result = PathResult(
            found=False,
            path=[],
            hops=0,
            chain_quality=0.0,
            T_head_arm=None,
            covariance_diag=None,
            head_marker=-1,
            arm_marker=-1,
            head_stamp_sec=0.0,
            arm_stamp_sec=0.0,
        )

        head_obs_all = self._camera_latest.get(head_source, {})
        arm_obs_all = self._camera_latest.get(arm_source, {})
        if not head_obs_all or not arm_obs_all:
            return empty_result

        if now_sec is None:
            now_sec = max(
                max((obs.stamp_sec for obs in head_obs_all.values()), default=0.0),
                max((obs.stamp_sec for obs in arm_obs_all.values()), default=0.0),
                0.0,
            )
            now_sec += 0.001  # slight padding so recent obs are not filtered
        age_cutoff = now_sec - max_obs_age_s

        best_score = -1.0
        best_result = empty_result

        for head_marker, head_obs in head_obs_all.items():
            if head_obs.stamp_sec < age_cutoff:
                continue

            for arm_marker, arm_obs in arm_obs_all.items():
                if arm_obs.stamp_sec < age_cutoff:
                    continue

                path = self.compute_ranked_path(head_marker, arm_marker, max_hops)
                if path is None or len(path) < 1:
                    continue

                hops = len(path) - 1
                chain_q = self._path_quality(path)
                head_q = head_obs.quality
                arm_q = arm_obs.quality
                score = chain_q * head_q * arm_q / max(1.0, float(hops))

                if score > best_score:
                    T = T_inv(head_obs.T_map_imu) @ arm_obs.T_map_imu
                    hops_penalty = float(hops) * np.asarray(cov_growth_diag, dtype=float).reshape(6)
                    chain_cov = (
                        head_obs.covariance_diag + arm_obs.covariance_diag + np.maximum(hops_penalty, 0.0)
                    )
                    best_score = score
                    best_result = PathResult(
                        found=True,
                        path=path,
                        hops=hops,
                        chain_quality=chain_q,
                        T_head_arm=T,
                        covariance_diag=chain_cov,
                        head_marker=head_marker,
                        arm_marker=arm_marker,
                        head_stamp_sec=head_obs.stamp_sec,
                        arm_stamp_sec=arm_obs.stamp_sec,
                    )

        return best_result

    def _path_quality(self, path: list[int]) -> float:
        if len(path) < 2:
            return 1.0
        q = 1.0
        for i in range(len(path) - 1):
            edge = self._edges.get(self._edge_key(path[i], path[i + 1]))
            if edge is None:
                return 0.0
            q *= max(edge.quality, 1e-6)
        return q
