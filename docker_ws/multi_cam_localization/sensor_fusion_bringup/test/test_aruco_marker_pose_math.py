from pathlib import Path
import sys

import math
import numpy as np
import yaml


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from aruco_marker_pose_node import (  # noqa: E402
    MarkerCovarianceModelConfig,
    MarkerMeasurement,
    MarkerQualityMetrics,
    MarkerTemporalStats,
    ArucoMarkerPoseNode,
    compute_marker_map_poses,
    estimate_marker_covariance_v2,
    load_kalibr_imucam,
    marker_quality_metrics,
    marker_view_angle_deg,
    rotvec_to_R,
    should_process_marker_frame,
    T_cam_body_display,
    T_inv,
)
from marker_quality_monitor import marker_summary_line, parse_marker_id_filter  # noqa: E402


def default_temporal(stable_frames=8, stable=True, stability_factor=0.0, correction_delta=None):
    return MarkerTemporalStats(
        stable_frames=stable_frames,
        stable=stable,
        stability_factor=stability_factor,
        detection_translation_delta_m=None,
        detection_rotation_delta_deg=None,
        correction_translation_delta_m=correction_delta,
        correction_rotation_delta_deg=None,
        odom_match_dt=None,
    )


def covariance_estimate(
    *,
    reprojection_error_px=0.5,
    side_mean_px=100.0,
    distance_m=1.0,
    view_angle_deg=0.0,
    temporal=None,
    cfg=None,
    T_map_cam=None,
):
    metrics = MarkerQualityMetrics(
        area_px2=side_mean_px * side_mean_px,
        sqrt_area_px=side_mean_px,
        side_mean_px=side_mean_px,
        side_min_px=side_mean_px,
        distance_m=distance_m,
        reprojection_error_px=reprojection_error_px,
        view_angle_deg=view_angle_deg,
    )
    return estimate_marker_covariance_v2(
        metrics=metrics,
        T_map_cam=np.eye(4) if T_map_cam is None else T_map_cam,
        f_avg_px=600.0,
        temporal_stats=default_temporal() if temporal is None else temporal,
        geometry_score=1.0,
        cfg=MarkerCovarianceModelConfig.from_mapping({}) if cfg is None else cfg,
    )


def test_marker_frame_throttle_disabled_processes_every_frame():
    assert should_process_marker_frame(10.0, None, 0.0)
    assert should_process_marker_frame(10.01, 10.0, 0.0)
    assert should_process_marker_frame(10.01, 10.0, -1.0)


def test_marker_frame_throttle_uses_image_timestamps():
    assert should_process_marker_frame(10.0, None, 15.0)
    assert not should_process_marker_frame(10.03, 10.0, 15.0)
    assert should_process_marker_frame(10.067, 10.0, 15.0)
    assert should_process_marker_frame(9.5, 10.0, 15.0)


def test_solvepnp_pose_direction_uses_inverse_for_camera_pose():
    T_map_marker = np.eye(4)
    T_cam_marker = np.eye(4)
    T_cam_marker[:3, 3] = [0.0, 0.0, 1.0]
    T_cam_imu = np.eye(4)

    T_map_cam, T_map_imu = compute_marker_map_poses(T_map_marker, T_cam_marker, T_cam_imu)

    np.testing.assert_allclose(T_map_cam[:3, 3], [0.0, 0.0, -1.0])
    np.testing.assert_allclose(T_map_imu, T_map_cam)


def test_camera_body_display_frame_is_forward_left_up_from_optical_frame():
    T_cam_body = T_cam_body_display()
    R_cam_body = T_cam_body[:3, :3]

    # Columns are body axes expressed in the camera optical frame.
    # Optical frame: +X right, +Y down, +Z forward through the lens.
    # Display body: +X forward, +Y left, +Z up.
    np.testing.assert_allclose(R_cam_body @ np.array([1.0, 0.0, 0.0]), [0.0, 0.0, 1.0])
    np.testing.assert_allclose(R_cam_body @ np.array([0.0, 1.0, 0.0]), [-1.0, 0.0, 0.0])
    np.testing.assert_allclose(R_cam_body @ np.array([0.0, 0.0, 1.0]), [0.0, -1.0, 0.0])


def test_marker_map_axes_apply_live_empirical_navigation_remap():
    root = Path(__file__).resolve().parents[1]
    marker_map = yaml.safe_load((root / "config" / "markers" / "head_aruco_map.yaml").read_text())
    T_map_marker = np.array(marker_map["markers"][0]["T_map_marker"], dtype=float)

    old_R_map_marker = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )[:3, :3]
    R_new_old = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=float,
    )

    # Live validation of the previous marker map showed:
    #   old x = physical right, old y = physical down, old z = marker depth.
    # The desired navigation remap is:
    #   new_x = old_x, new_y = old_z, new_z = -old_y.
    np.testing.assert_allclose(R_new_old @ np.array([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0])
    np.testing.assert_allclose(R_new_old @ np.array([0.0, -1.0, 0.0]), [0.0, 0.0, 1.0])
    np.testing.assert_allclose(R_new_old @ np.array([0.0, 0.0, 1.0]), [0.0, 1.0, 0.0])

    expected_R_map_marker = R_new_old @ old_R_map_marker
    np.testing.assert_allclose(
        T_map_marker,
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    )
    np.testing.assert_allclose(T_map_marker[:3, :3], expected_R_map_marker)


def test_head_marker_configs_use_explicit_physical_marker_sizes():
    root = Path(__file__).resolve().parents[1]
    config_dir = root / "config" / "markers"
    expected_sizes = {
        "head_aruco_map.yaml": 0.100,
        "head_aruco_map_replay_1533mm.yaml": 0.1533,
        "head_aruco_map_large_160mm.yaml": 0.160,
    }

    for filename, expected_size_m in expected_sizes.items():
        marker_map = yaml.safe_load((config_dir / filename).read_text())
        marker_ids = sorted(int(marker_id) for marker_id in marker_map["markers"].keys())
        T_map_marker = np.array(marker_map["markers"][0]["T_map_marker"], dtype=float)

        assert marker_ids == [0]
        assert marker_map["aruco"]["default_marker_size_m"] == expected_size_m
        assert marker_map["markers"][0]["size_m"] == expected_size_m
        assert T_map_marker.shape == (4, 4)


def test_marker_quality_monitor_marker_id_filter_and_summary_line():
    assert parse_marker_id_filter("0,1,2") == {0, 1, 2}
    assert parse_marker_id_filter("all") is None
    assert parse_marker_id_filter("") is None

    line = marker_summary_line(
        {
            "marker_id": 2,
            "distance_m": 0.7421,
            "view_angle_deg": 28.63,
            "side_mean_px": 91.24,
            "reprojection_error_px": 0.4123,
        }
    )

    assert "id=2" in line
    assert "dist=0.742m" in line
    assert "angle=28.6deg" in line


def test_load_kalibr_inverts_t_imu_cam_when_t_cam_imu_is_absent(tmp_path):
    yaml_path = tmp_path / "kalibr_imucam_chain.yaml"
    yaml_path.write_text(
        """%YAML:1.0
cam0:
  T_imu_cam:
    - [1.0, 0.0, 0.0, 0.10]
    - [0.0, 1.0, 0.0, 0.00]
    - [0.0, 0.0, 1.0, 0.00]
    - [0.0, 0.0, 0.0, 1.00]
  camera_model: pinhole
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  distortion_model: radtan
  intrinsics: [100.0, 100.0, 50.0, 50.0]
  resolution: [100, 100]
  rostopic: /camera/image_raw
  timeshift_cam_imu: 0.01
""",
        encoding="utf-8",
    )

    _K, _D, T_cam_imu, timeshift = load_kalibr_imucam(str(yaml_path))

    expected = T_inv(
        np.array(
            [
                [1.0, 0.0, 0.0, 0.10],
                [0.0, 1.0, 0.0, 0.00],
                [0.0, 0.0, 1.0, 0.00],
                [0.0, 0.0, 0.0, 1.00],
            ],
            dtype=float,
        )
    )
    np.testing.assert_allclose(T_cam_imu, expected)
    assert timeshift == 0.01


def test_marker_quality_metrics_include_pixel_size_distance_and_view_angle():
    image_points = np.array(
        [
            [10.0, 20.0],
            [110.0, 20.0],
            [110.0, 120.0],
            [10.0, 120.0],
        ],
        dtype=np.float32,
    )
    T_cam_marker = np.eye(4)
    T_cam_marker[:3, 3] = [0.0, 0.0, 1.2]

    metrics = marker_quality_metrics(image_points, T_cam_marker, reprojection_error_px=0.75)

    assert metrics.area_px2 == 10000.0
    assert metrics.sqrt_area_px == 100.0
    assert metrics.side_mean_px == 100.0
    assert metrics.side_min_px == 100.0
    assert metrics.distance_m == 1.2
    assert metrics.reprojection_error_px == 0.75
    assert metrics.view_angle_deg == 0.0


def test_marker_view_angle_increases_for_oblique_marker():
    T_cam_marker = np.eye(4)
    T_cam_marker[:3, :3] = rotvec_to_R(np.array([0.0, math.radians(60.0), 0.0]))

    assert marker_view_angle_deg(T_cam_marker) > 59.0


def test_covariance_increases_with_reprojection_error_pixel_size_distance_view_and_temporal_instability():
    baseline = covariance_estimate()
    worse_reprojection = covariance_estimate(reprojection_error_px=2.0)
    smaller_marker = covariance_estimate(side_mean_px=45.0)
    farther_marker = covariance_estimate(distance_m=1.8)
    oblique_marker = covariance_estimate(view_angle_deg=65.0)
    unstable_marker = covariance_estimate(
        temporal=default_temporal(stable_frames=2, stable=False, stability_factor=0.75)
    )
    jittery_marker = covariance_estimate(
        temporal=default_temporal(correction_delta=0.08)
    )

    assert np.all(worse_reprojection.std_diag > baseline.std_diag)
    assert smaller_marker.std_diag[2] > baseline.std_diag[2]
    assert smaller_marker.std_diag[3] > baseline.std_diag[3]
    assert farther_marker.std_diag[0] > baseline.std_diag[0]
    assert farther_marker.std_diag[2] > baseline.std_diag[2]
    assert oblique_marker.std_diag[2] > baseline.std_diag[2]
    assert oblique_marker.std_diag[3] > baseline.std_diag[3]
    assert np.all(unstable_marker.std_diag > baseline.std_diag)
    assert np.all(jittery_marker.std_diag[:3] > baseline.std_diag[:3])


def test_covariance_respects_floors_and_caps():
    cfg = MarkerCovarianceModelConfig.from_mapping(
        {
            "min_marker_xy_std_m": 0.02,
            "min_marker_z_std_m": 0.03,
            "max_marker_xy_std_m": 0.04,
            "max_marker_z_std_m": 0.05,
            "min_marker_roll_pitch_std_deg": 2.0,
            "min_marker_yaw_std_deg": 3.0,
            "max_marker_roll_pitch_std_deg": 5.0,
            "max_marker_yaw_std_deg": 6.0,
            "marker_xy_std_px_gain": 1000.0,
            "marker_z_std_px_gain": 1000.0,
            "marker_roll_pitch_std_px_gain": 1000.0,
            "marker_yaw_std_px_gain": 1000.0,
        }
    )

    capped = covariance_estimate(
        reprojection_error_px=10.0,
        side_mean_px=5.0,
        distance_m=5.0,
        view_angle_deg=80.0,
        cfg=cfg,
    )

    assert np.all(capped.std_diag[:3] <= 0.05 + 1e-12)
    assert np.all(np.degrees(capped.std_diag[3:]) <= 6.0 + 1e-12)
    assert np.all(capped.std_diag[:3] >= 0.02 - 1e-12)
    assert np.all(np.degrees(capped.std_diag[3:]) >= 2.0 - 1e-12)


def test_covariance_rotation_moves_depth_uncertainty_into_marker_map_axis():
    T_map_cam = np.eye(4)
    T_map_cam[:3, :3] = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ],
        dtype=float,
    )

    estimate = covariance_estimate(side_mean_px=45.0, distance_m=1.5, T_map_cam=T_map_cam)

    assert estimate.std_diag[0] > estimate.std_diag[1]
    assert estimate.std_diag[0] > estimate.std_diag[2]


def test_marker_quality_payload_contains_flat_covariance_and_temporal_fields():
    measurement = MarkerMeasurement(
        stamp=None,
        stamp_sec=123.0,
        marker_id=7,
        marker_frame="marker_7",
        T_cam_marker=np.eye(4),
        T_marker_cam=np.eye(4),
        T_map_cam=np.eye(4),
        T_map_imu=np.eye(4),
        T_map_marker=np.eye(4),
        area_px2=10000.0,
        sqrt_area_px=100.0,
        side_mean_px=100.0,
        side_min_px=95.0,
        distance_m=1.0,
        reprojection_error_px=0.5,
        view_angle_deg=10.0,
        view_penalty=1.1,
        covariance_diag=np.array([0.01, 0.01, 0.04, 0.001, 0.001, 0.002], dtype=float),
        covariance_std_diag=np.sqrt(np.array([0.01, 0.01, 0.04, 0.001, 0.001, 0.002], dtype=float)),
        covariance_camera_std_diag=np.array([0.1, 0.1, 0.2, 0.03, 0.03, 0.04], dtype=float),
        covariance_sigma_px=0.5,
        stable_frames=8,
        stable=True,
        stability_factor=0.0,
        temporal_detection_translation_m=0.01,
        temporal_detection_rotation_deg=0.5,
        temporal_correction_translation_m=0.002,
        temporal_correction_rotation_deg=0.1,
        temporal_odom_match_dt=0.003,
        geometry_score=0.95,
        image_width=640,
        image_height=480,
    )

    payload = ArucoMarkerPoseNode.marker_quality_payload(None, measurement, hard_gate_status="accepted")

    assert payload["marker_id"] == 7
    assert payload["hard_gate_passed"] is True
    assert payload["marker_side_mean_px"] == 100.0
    assert payload["marker_temporal_correction_translation_m"] == 0.002
    assert payload["marker_covariance_std_z_m"] == 0.2
    assert payload["marker_covariance_camera_std_yaw_deg"] > 0.0


def test_corner_geometry_still_rejects_bad_corners_before_covariance_weighting():
    node = ArucoMarkerPoseNode.__new__(ArucoMarkerPoseNode)
    node.border_margin_px = 8.0
    node.min_corner_angle_deg = 25.0
    node.max_corner_angle_deg = 155.0
    node.min_border_geometry_score = 0.35

    bad_points = np.array(
        [
            [10.0, 10.0],
            [11.0, 10.0],
            [12.0, 10.0],
            [10.0, 11.0],
        ],
        dtype=np.float32,
    )

    geometry_ok, _score, reason = node.check_corner_geometry(bad_points, 640, 480)

    assert geometry_ok is False
    assert reason in {
        "marker_corners_not_convex",
        "marker_corner_angles_bad",
        "marker_degenerate_corner",
        "marker_near_border_poor_geometry",
    }
