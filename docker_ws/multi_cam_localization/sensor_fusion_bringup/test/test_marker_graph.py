from pathlib import Path
import sys
import math

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from marker_graph_core import (  # noqa: E402
    CameraObservation,
    MarkerEdge,
    MarkerGraph,
    PathResult,
    T_inv,
    stamp_to_sec,
    pose_to_T,
    fill_pose,
    diag_from_covariance,
    covariance_from_diag,
    rotation_angle_deg,
)


def make_obs(
    marker_id: int,
    T_map_imu: np.ndarray,
    stamp_sec: float = 0.0,
    covariance_diag: np.ndarray = None,
    geometry_score: float = 0.9,
    reprojection_error_px: float = 0.5,
    stable: bool = True,
) -> CameraObservation:
    if covariance_diag is None:
        covariance_diag = np.array(
            [0.01**2] * 3 + [math.radians(1.0) ** 2] * 3, dtype=float
        )
    quality = geometry_score * (1.0 / max(1.0 + reprojection_error_px, 1.0))
    if stable:
        quality = min(1.0, quality * 1.2)
    else:
        quality *= 0.7
    quality = max(0.0, min(1.0, quality))
    return CameraObservation(
        stamp_sec=stamp_sec,
        marker_id=marker_id,
        marker_frame=f"marker_{marker_id}",
        target_frame="head_imu",
        T_map_imu=T_map_imu.copy(),
        covariance_diag=covariance_diag.copy(),
        geometry_score=geometry_score,
        reprojection_error_px=reprojection_error_px,
        stable=stable,
        quality=quality,
    )


def T_identity() -> np.ndarray:
    return np.eye(4, dtype=float)


def T_translate(x: float = 1.0, y: float = 0.0, z: float = 0.0) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def T_rotate_z(deg: float) -> np.ndarray:
    rad = math.radians(deg)
    c = math.cos(rad)
    s = math.sin(rad)
    T = np.eye(4, dtype=float)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    return T


class TestMarkerGraphBasic:
    def test_single_marker_observation_adds_node(self):
        g = MarkerGraph()
        obs = make_obs(5, T_identity(), stamp_sec=1.0)
        g.update_camera_observation("head", obs)
        markers = g.get_markers()
        assert 5 in markers

    def test_two_markers_same_frame_creates_edge(self):
        g = MarkerGraph()
        obs_a = make_obs(1, T_identity(), stamp_sec=2.0)
        obs_b = make_obs(2, T_identity(), stamp_sec=2.0)

        edges = g.add_co_observations("head", [obs_a, obs_b])
        assert len(edges) == 1
        assert (edges[0] == (1, 2)) or (edges[0] == (2, 1))
        assert g.has_edge(1, 2)
        assert g.has_edge(2, 1)

    def test_edge_stored_symmetrically(self):
        g = MarkerGraph()
        g.add_co_observations(
            "head",
            [
                make_obs(3, T_identity(), stamp_sec=3.0),
                make_obs(7, T_identity(), stamp_sec=3.0),
            ],
        )
        edge_37 = g.get_edge(3, 7)
        edge_73 = g.get_edge(7, 3)
        assert edge_37 is not None
        assert edge_73 is not None
        assert edge_37.marker_a == edge_73.marker_a
        assert edge_37.quality == edge_73.quality

    def test_two_markers_different_time_no_edge(self):
        g = MarkerGraph()
        obs_a = make_obs(1, T_identity(), stamp_sec=4.0)
        obs_b = make_obs(2, T_identity(), stamp_sec=5.0)

        edges = g.add_co_observations("head", [obs_a])
        assert len(edges) == 0
        edges2 = g.add_co_observations("head", [obs_b])
        assert len(edges2) == 0

    def test_three_markers_same_frame_three_edges(self):
        g = MarkerGraph()
        obs = [
            make_obs(1, T_identity(), stamp_sec=6.0),
            make_obs(2, T_identity(), stamp_sec=6.0),
            make_obs(3, T_identity(), stamp_sec=6.0),
        ]
        edges = g.add_co_observations("head", obs)
        assert len(edges) == 3

    def test_edge_accumulates_quality_with_repeated_observations(self):
        g = MarkerGraph()

        def mk(n, q):
            return CameraObservation(
                stamp_sec=10.0,
                marker_id=n,
                marker_frame=f"marker_{n}",
                target_frame="head_imu",
                T_map_imu=T_identity(),
                covariance_diag=np.full(6, 0.0001, dtype=float),
                geometry_score=q,
                reprojection_error_px=0.1,
                stable=True,
                quality=q,
            )

        g.add_co_observations("head", [mk(1, 0.5), mk(2, 0.5)])
        edge1 = g.get_edge(1, 2)
        q1 = edge1.quality
        assert edge1.num_observations == 1
        assert q1 > 0.4

        g.add_co_observations("head", [mk(1, 0.8), mk(2, 0.8)])
        edge2 = g.get_edge(1, 2)
        assert edge2.num_observations == 2
        assert edge2.quality > q1


class TestPathQuery:
    def test_direct_path_same_marker(self):
        g = MarkerGraph()
        path = g.shortest_path(5, 5)
        assert path == [5]

    def test_no_path_disconnected(self):
        g = MarkerGraph()
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=1.0),
                make_obs(2, T_identity(), stamp_sec=1.0),
            ],
        )
        path = g.shortest_path(1, 3)
        assert path == []

    def test_one_hop_path(self):
        g = MarkerGraph()
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=1.0),
                make_obs(2, T_identity(), stamp_sec=1.0),
            ],
        )
        path = g.shortest_path(1, 2)
        assert path == [1, 2]

    def test_three_hop_chain_path(self):
        g = MarkerGraph()
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=1.0),
                make_obs(2, T_identity(), stamp_sec=1.0),
            ],
        )
        g.add_co_observations(
            "head",
            [
                make_obs(2, T_identity(), stamp_sec=2.0),
                make_obs(3, T_identity(), stamp_sec=2.0),
            ],
        )
        g.add_co_observations(
            "arm",
            [
                make_obs(3, T_identity(), stamp_sec=3.0),
                make_obs(4, T_identity(), stamp_sec=3.0),
            ],
        )
        path = g.shortest_path(1, 4)
        assert path == [1, 2, 3, 4]

    def test_connected_components(self):
        g = MarkerGraph()
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=1.0),
                make_obs(2, T_identity(), stamp_sec=1.0),
            ],
        )
        g.add_co_observations(
            "arm",
            [
                make_obs(3, T_identity(), stamp_sec=2.0),
                make_obs(4, T_identity(), stamp_sec=2.0),
            ],
        )
        comps = g.connected_components()
        sizes = sorted([len(c) for c in comps])
        assert sizes == [2, 2]
        c1, c2 = comps
        assert (1 in c1 and 2 in c1) or (1 in c2 and 2 in c2)

    def test_connected_components_merged(self):
        g = MarkerGraph()
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=1.0),
                make_obs(2, T_identity(), stamp_sec=1.0),
            ],
        )
        g.add_co_observations(
            "head",
            [
                make_obs(2, T_identity(), stamp_sec=2.0),
                make_obs(3, T_identity(), stamp_sec=2.0),
            ],
        )
        comps = g.connected_components()
        assert len(comps) == 1
        assert comps[0] == {1, 2, 3}


class TestStaleEdgeExpiration:
    def test_stale_edge_removed_after_timeout(self):
        g = MarkerGraph(max_edge_age_s=10.0)
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=1.0),
                make_obs(2, T_identity(), stamp_sec=1.0),
            ],
        )
        assert g.has_edge(1, 2)

        removed = g.prune_stale_edges(now_sec=5.0)
        assert len(removed) == 0
        assert g.has_edge(1, 2)

        removed = g.prune_stale_edges(now_sec=20.0)
        assert len(removed) == 1
        assert not g.has_edge(1, 2)

    def test_fresh_edge_not_removed(self):
        g = MarkerGraph(max_edge_age_s=10.0)
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=15.0),
                make_obs(2, T_identity(), stamp_sec=15.0),
            ],
        )
        removed = g.prune_stale_edges(now_sec=20.0)
        assert len(removed) == 0
        assert g.has_edge(1, 2)

    def test_reobserved_edge_resets_age(self):
        g = MarkerGraph(max_edge_age_s=10.0)
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=1.0),
                make_obs(2, T_identity(), stamp_sec=1.0),
            ],
        )
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=9.0),
                make_obs(2, T_identity(), stamp_sec=9.0),
            ],
        )
        removed = g.prune_stale_edges(now_sec=15.0)
        assert len(removed) == 0
        assert g.has_edge(1, 2)


class TestEdgeInconsistency:
    def test_low_quality_edge(self):
        g = MarkerGraph()
        obs_bad_a = CameraObservation(
            stamp_sec=1.0,
            marker_id=1,
            marker_frame="marker_1",
            target_frame="head_imu",
            T_map_imu=T_identity(),
            covariance_diag=np.full(6, 0.01, dtype=float),
            geometry_score=0.3,
            reprojection_error_px=5.0,
            stable=False,
            quality=0.0,
        )
        obs_bad_b = CameraObservation(
            stamp_sec=1.0,
            marker_id=2,
            marker_frame="marker_2",
            target_frame="head_imu",
            T_map_imu=T_identity(),
            covariance_diag=np.full(6, 0.01, dtype=float),
            geometry_score=0.3,
            reprojection_error_px=5.0,
            stable=False,
            quality=0.0,
        )
        g.add_co_observations("head", [obs_bad_a, obs_bad_b])
        edge = g.get_edge(1, 2)
        assert edge is not None
        assert edge.quality < 0.3

    def test_inconsistent_observation_updates_edge(self):
        g = MarkerGraph()
        obs_a = make_obs(1, T_identity(), stamp_sec=1.0, geometry_score=0.9, stable=True)
        obs_b = make_obs(2, T_identity(), stamp_sec=1.0, geometry_score=0.9, stable=True)
        g.add_co_observations("head", [obs_a, obs_b])
        q1 = g.get_edge(1, 2).quality

        obs_a2 = make_obs(
            1, T_identity(), stamp_sec=2.0, geometry_score=0.5, stable=False
        )
        obs_b2 = make_obs(
            2, T_identity(), stamp_sec=2.0, geometry_score=0.5, stable=False
        )
        g.add_co_observations("head", [obs_a2, obs_b2])
        q2 = g.get_edge(1, 2).quality

        assert q2 < q1


class TestHeadToArm:
    def test_head_to_arm_via_shared_marker(self):
        g = MarkerGraph()
        T_head = T_translate(1.0, 0.0, 0.0)
        T_arm = T_translate(2.0, 0.0, 0.0)

        g.update_camera_observation("head", make_obs(5, T_head, stamp_sec=1.0))
        g.update_camera_observation("arm", make_obs(5, T_arm, stamp_sec=1.0))

        cov_growth = np.zeros(6, dtype=float)
        result = g.query_head_to_arm("head", "arm", cov_growth)
        assert result.found
        assert result.hops == 0
        assert result.head_marker == 5
        assert result.arm_marker == 5
        assert result.T_head_arm is not None
        assert abs(result.T_head_arm[0, 3] - 1.0) < 1e-6
        expected = T_inv(T_head) @ T_arm
        assert np.allclose(result.T_head_arm, expected)

    def test_head_to_arm_via_two_hop_chain(self):
        g = MarkerGraph()
        T_head = T_translate(1.0, 0.0, 0.0)
        T_arm = T_translate(3.0, 0.0, 0.0)

        g.update_camera_observation("head", make_obs(1, T_head, stamp_sec=1.0))
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_translate(1.0, 0.0, 0.0), stamp_sec=2.0),
                make_obs(2, T_translate(1.0, 0.0, 0.0), stamp_sec=2.0),
            ],
        )
        g.add_co_observations(
            "arm",
            [
                make_obs(2, T_translate(2.5, 0.0, 0.0), stamp_sec=3.0),
                make_obs(3, T_translate(2.5, 0.0, 0.0), stamp_sec=3.0),
            ],
        )
        g.update_camera_observation("arm", make_obs(3, T_arm, stamp_sec=4.0))

        cov_growth = np.zeros(6, dtype=float)
        result = g.query_head_to_arm("head", "arm", cov_growth)
        assert result.found
        assert result.hops == 2
        assert result.path == [1, 2, 3]

    def test_head_to_arm_via_multi_marker_options_picks_best(self):
        g = MarkerGraph()
        T_head = T_translate(0.0, 0.0, 0.0)
        T_arm = T_translate(5.0, 0.0, 0.0)

        g.update_camera_observation("head", make_obs(1, T_head, stamp_sec=1.0))
        g.update_camera_observation("head", make_obs(10, T_head, stamp_sec=1.0))
        g.update_camera_observation("arm", make_obs(5, T_arm, stamp_sec=1.0))
        g.update_camera_observation("arm", make_obs(20, T_arm, stamp_sec=1.0))

        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=2.0, geometry_score=0.5),
                make_obs(2, T_identity(), stamp_sec=2.0, geometry_score=0.5),
            ],
        )
        g.add_co_observations(
            "head",
            [
                make_obs(2, T_identity(), stamp_sec=3.0, geometry_score=0.5),
                make_obs(3, T_identity(), stamp_sec=3.0, geometry_score=0.5),
            ],
        )
        g.add_co_observations(
            "arm",
            [
                make_obs(3, T_identity(), stamp_sec=4.0, geometry_score=0.5),
                make_obs(4, T_identity(), stamp_sec=4.0, geometry_score=0.5),
            ],
        )
        g.add_co_observations(
            "arm",
            [
                make_obs(4, T_identity(), stamp_sec=5.0, geometry_score=0.5),
                make_obs(5, T_identity(), stamp_sec=5.0, geometry_score=0.5),
            ],
        )

        g.add_co_observations(
            "head",
            [
                make_obs(10, T_identity(), stamp_sec=2.0, geometry_score=0.95),
                make_obs(20, T_identity(), stamp_sec=2.0, geometry_score=0.95),
            ],
        )

        cov_growth = np.zeros(6, dtype=float)
        result = g.query_head_to_arm("head", "arm", cov_growth)
        assert result.found
        assert result.path[0] in (1, 10)
        assert result.path[-1] in (5, 20)
        assert result.hops >= 1

    def test_disconnected_components_no_path(self):
        g = MarkerGraph()
        g.update_camera_observation("head", make_obs(1, T_identity(), stamp_sec=1.0))
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=1.0),
                make_obs(2, T_identity(), stamp_sec=1.0),
            ],
        )
        g.update_camera_observation("arm", make_obs(3, T_identity(), stamp_sec=1.0))
        g.add_co_observations(
            "arm",
            [
                make_obs(3, T_identity(), stamp_sec=1.0),
                make_obs(4, T_identity(), stamp_sec=1.0),
            ],
        )
        cov_growth = np.zeros(6, dtype=float)
        result = g.query_head_to_arm("head", "arm", cov_growth)
        assert not result.found

    def test_stale_observation_rejected(self):
        g = MarkerGraph()
        g.update_camera_observation(
            "head", make_obs(1, T_identity(), stamp_sec=100.0)
        )
        g.update_camera_observation(
            "arm", make_obs(1, T_identity(), stamp_sec=100.0)
        )
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=100.0),
                make_obs(2, T_identity(), stamp_sec=100.0),
            ],
        )
        g.update_camera_observation(
            "arm", make_obs(2, T_identity(), stamp_sec=100.0)
        )

        cov_growth = np.zeros(6, dtype=float)
        result = g.query_head_to_arm("head", "arm", cov_growth, max_obs_age_s=1.0, now_sec=110.0)
        assert not result.found


class TestCovarianceAccumulation:
    def test_covariance_grows_with_chain_length(self):
        g = MarkerGraph()
        obs_cov = np.array([0.0001, 0.0001, 0.0001, 1e-6, 1e-6, 1e-6], dtype=float)
        growth_cov = np.array([0.0004, 0.0004, 0.0004, 4e-6, 4e-6, 4e-6], dtype=float)

        g.update_camera_observation(
            "head", make_obs(1, T_identity(), stamp_sec=1.0, covariance_diag=obs_cov)
        )
        g.update_camera_observation(
            "arm", make_obs(3, T_identity(), stamp_sec=1.0, covariance_diag=obs_cov)
        )
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=1.0),
                make_obs(2, T_identity(), stamp_sec=1.0),
            ],
        )
        g.add_co_observations(
            "arm",
            [
                make_obs(2, T_identity(), stamp_sec=1.0),
                make_obs(3, T_identity(), stamp_sec=1.0),
            ],
        )

        result = g.query_head_to_arm("head", "arm", growth_cov)
        assert result.found
        assert result.covariance_diag is not None
        assert result.hops == 2

        for i in range(6):
            expected = obs_cov[i] + obs_cov[i] + 2.0 * growth_cov[i]
            assert result.covariance_diag[i] >= expected * 0.99

    def test_zero_covariance_growth_for_direct_link(self):
        g = MarkerGraph()
        obs_cov = np.array([0.0001, 0.0001, 0.0001, 1e-6, 1e-6, 1e-6], dtype=float)
        g.update_camera_observation(
            "head", make_obs(1, T_identity(), stamp_sec=1.0, covariance_diag=obs_cov)
        )
        g.update_camera_observation(
            "arm", make_obs(1, T_identity(), stamp_sec=1.0, covariance_diag=obs_cov)
        )
        growth = np.zeros(6, dtype=float)
        result = g.query_head_to_arm("head", "arm", growth)
        assert result.found
        assert result.hops == 0
        for i in range(6):
            assert abs(result.covariance_diag[i] - 2.0 * obs_cov[i]) < 1e-12


class TestTransformHelpers:
    def test_T_inv(self):
        T = T_translate(1.0, 2.0, 3.0)
        T[:3, :3] = T_rotate_z(90.0)[:3, :3]
        Tinv = T_inv(T)
        I = T @ Tinv
        assert np.allclose(I, np.eye(4), atol=1e-10)

    def test_diag_from_covariance(self):
        cov_list = [0.0] * 36
        cov_list[0] = 0.04
        cov_list[7] = 0.09
        cov_list[14] = 0.16
        cov_list[21] = 0.25
        cov_list[28] = 0.36
        cov_list[35] = 0.49
        diag = diag_from_covariance(cov_list)
        expected = np.array([0.04, 0.09, 0.16, 0.25, 0.36, 0.49], dtype=float)
        assert np.allclose(diag, expected)

    def test_covariance_from_diag(self):
        diag = np.array([0.01, 0.02, 0.03, 0.04, 0.05, 0.06], dtype=float)
        cov = covariance_from_diag(diag)
        assert len(cov) == 36
        for i in range(6):
            assert cov[6 * i + i] == diag[i]
        non_diag_sum = sum(
            cov[i] for i in range(36) if i not in [0, 7, 14, 21, 28, 35]
        )
        assert non_diag_sum == 0.0


class TestRankedPath:
    def test_ranked_path_prefers_high_quality(self):
        g = MarkerGraph()
        g.update_camera_observation(
            "head", make_obs(1, T_identity(), stamp_sec=1.0)
        )
        g.update_camera_observation(
            "arm", make_obs(4, T_identity(), stamp_sec=1.0)
        )

        obs_low = [
            CameraObservation(
                stamp_sec=1.0,
                marker_id=1,
                marker_frame="marker_1",
                target_frame="head_imu",
                T_map_imu=T_identity(),
                covariance_diag=np.full(6, 0.0001, dtype=float),
                geometry_score=0.3,
                reprojection_error_px=0.5,
                stable=False,
                quality=0.1,
            ),
            CameraObservation(
                stamp_sec=1.0,
                marker_id=2,
                marker_frame="marker_2",
                target_frame="head_imu",
                T_map_imu=T_identity(),
                covariance_diag=np.full(6, 0.0001, dtype=float),
                geometry_score=0.3,
                reprojection_error_px=0.5,
                stable=False,
                quality=0.1,
            ),
        ]
        obs_high = [
            CameraObservation(
                stamp_sec=1.0,
                marker_id=1,
                marker_frame="marker_1",
                target_frame="head_imu",
                T_map_imu=T_identity(),
                covariance_diag=np.full(6, 0.0001, dtype=float),
                geometry_score=0.95,
                reprojection_error_px=0.1,
                stable=True,
                quality=0.9,
            ),
            CameraObservation(
                stamp_sec=1.0,
                marker_id=3,
                marker_frame="marker_3",
                target_frame="head_imu",
                T_map_imu=T_identity(),
                covariance_diag=np.full(6, 0.0001, dtype=float),
                geometry_score=0.95,
                reprojection_error_px=0.1,
                stable=True,
                quality=0.9,
            ),
        ]
        obs_high2 = [
            CameraObservation(
                stamp_sec=1.0,
                marker_id=3,
                marker_frame="marker_3",
                target_frame="head_imu",
                T_map_imu=T_identity(),
                covariance_diag=np.full(6, 0.0001, dtype=float),
                geometry_score=0.95,
                reprojection_error_px=0.1,
                stable=True,
                quality=0.9,
            ),
            CameraObservation(
                stamp_sec=1.0,
                marker_id=4,
                marker_frame="marker_4",
                target_frame="head_imu",
                T_map_imu=T_identity(),
                covariance_diag=np.full(6, 0.0001, dtype=float),
                geometry_score=0.95,
                reprojection_error_px=0.1,
                stable=True,
                quality=0.9,
            ),
        ]
        g.add_co_observations("head", obs_low)
        g.add_co_observations("head", obs_high)
        g.add_co_observations("head", obs_high2)

        cov_growth = np.zeros(6, dtype=float)
        result = g.query_head_to_arm("head", "arm", cov_growth)
        assert result.found
        assert result.path == [1, 3, 4]

    def test_ranked_path_returns_none_when_no_path(self):
        g = MarkerGraph()
        g.update_camera_observation(
            "head", make_obs(1, T_identity(), stamp_sec=1.0)
        )
        g.update_camera_observation(
            "arm", make_obs(5, T_identity(), stamp_sec=1.0)
        )
        path = g.compute_ranked_path(1, 5)
        assert path is None


class TestPruneObservationAge:
    def test_old_observations_excluded_from_query(self):
        g = MarkerGraph()
        g.update_camera_observation(
            "head", make_obs(1, T_identity(), stamp_sec=1.0)
        )
        g.update_camera_observation(
            "arm", make_obs(1, T_identity(), stamp_sec=1.0)
        )

        cov_growth = np.zeros(6, dtype=float)
        result_fresh = g.query_head_to_arm(
            "head", "arm", cov_growth, max_obs_age_s=10.0
        )
        assert result_fresh.found

        result_stale = g.query_head_to_arm(
            "head", "arm", cov_growth, max_obs_age_s=0.001, now_sec=10.0
        )
        assert not result_stale.found

    def test_only_recent_observation_used(self):
        g = MarkerGraph()
        g.update_camera_observation(
            "head", make_obs(1, T_identity(), stamp_sec=1.0)
        )
        g.update_camera_observation(
            "head", make_obs(1, T_identity(), stamp_sec=100.0)
        )
        g.update_camera_observation(
            "arm", make_obs(1, T_identity(), stamp_sec=100.0)
        )
        g.add_co_observations(
            "head",
            [
                make_obs(1, T_identity(), stamp_sec=100.0),
                make_obs(2, T_identity(), stamp_sec=100.0),
            ],
        )
        g.update_camera_observation(
            "arm", make_obs(2, T_identity(), stamp_sec=100.0)
        )

        cov_growth = np.zeros(6, dtype=float)
        result = g.query_head_to_arm("head", "arm", cov_growth, max_obs_age_s=5.0)
        assert result.found
        assert result.head_stamp_sec >= 99.0
        assert result.arm_stamp_sec >= 99.0
