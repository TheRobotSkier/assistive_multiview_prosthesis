from pathlib import Path
import sys

import math
import numpy as np
import yaml


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from aruco_marker_pose_node import (  # noqa: E402
    DynamicMarkerMeasurement,
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
from calibrate_arm_marker_extrinsic import (  # noqa: E402
    RESIDUAL_ORDER,
    compute_armcam_marker_sample,
    robust_se3_estimate,
    se3_exp,
    se3_residual,
)
from head_derived_arm_pose_preview_node import (  # noqa: E402
    HeadPoseSample,
    compose_head_derived_arm_pose,
    load_arm_marker_extrinsic,
    match_head_pose,
    propagate_candidate_covariance,
)
from dynamic_arm_pose_measurement_node import (  # noqa: E402
    DynamicArmPoseMeasurementNode,
    HeadPoseMeasurementSample,
    compose_dynamic_arm_imu_pose,
    match_head_pose_measurement,
    propagate_dynamic_arm_imu_covariance,
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


def test_dynamic_marker_id2_is_separate_from_fixed_marker_maps():
    root = Path(__file__).resolve().parents[1]
    config_dir = root / "config" / "markers"
    head_map = yaml.safe_load((config_dir / "head_aruco_map.yaml").read_text())
    arm_map = yaml.safe_load((config_dir / "arm_aruco_map.yaml").read_text())

    assert sorted(int(marker_id) for marker_id in head_map["markers"].keys()) == [0]
    assert sorted(int(marker_id) for marker_id in arm_map["markers"].keys()) == [0]
    assert sorted(int(marker_id) for marker_id in head_map["dynamic_markers"].keys()) == [2]
    assert "T_map_marker" not in head_map["dynamic_markers"][2]
    assert "dynamic_markers" not in arm_map
    assert head_map["topics"]["dynamic_observation_topic_suffix"] == "dynamic_observation"


def test_dynamic_marker_publisher_uses_camera_frame_pose_not_marker_map_pose():
    from builtin_interfaces.msg import Time

    class CapturePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    node = ArucoMarkerPoseNode.__new__(ArucoMarkerPoseNode)
    node.dynamic_marker_observation_pub = CapturePublisher()

    T_cam_marker = np.eye(4)
    T_cam_marker[:3, 3] = [0.10, -0.02, 0.55]
    measurement = DynamicMarkerMeasurement(
        stamp=Time(sec=123, nanosec=456),
        stamp_sec=123.000000456,
        marker_id=2,
        marker_frame="arm_marker_2",
        camera_frame="head_d435i_head_color_optical_frame",
        T_cam_marker=T_cam_marker,
        area_px2=10000.0,
        sqrt_area_px=100.0,
        side_mean_px=100.0,
        side_min_px=98.0,
        distance_m=0.56,
        reprojection_error_px=0.4,
        view_angle_deg=12.0,
        view_penalty=1.0,
        covariance_diag=np.array([0.01, 0.01, 0.02, 0.001, 0.001, 0.002], dtype=float),
        covariance_std_diag=np.sqrt(np.array([0.01, 0.01, 0.02, 0.001, 0.001, 0.002], dtype=float)),
        covariance_sigma_px=0.4,
        stable_frames=8,
        stable=True,
        stability_factor=0.0,
        temporal_detection_translation_m=None,
        temporal_detection_rotation_deg=None,
        geometry_score=0.9,
        image_width=640,
        image_height=480,
    )

    ArucoMarkerPoseNode.publish_dynamic_marker_observation(node, measurement)

    assert len(node.dynamic_marker_observation_pub.messages) == 1
    msg = node.dynamic_marker_observation_pub.messages[0]
    assert msg.header.frame_id == "head_d435i_head_color_optical_frame"
    assert msg.camera_frame == "head_d435i_head_color_optical_frame"
    assert msg.marker_frame == "arm_marker_2"
    assert msg.marker_id == 2
    assert msg.pose.pose.position.x == 0.10
    assert msg.pose.pose.position.y == -0.02
    assert msg.pose.pose.position.z == 0.55


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


def test_calibration_transform_composes_head_dynamic_marker_into_arm_camera():
    T_map_headcam = se3_exp([0.2, -0.1, 0.5, 0.02, -0.03, 0.04])
    T_headcam_marker = se3_exp([0.4, 0.1, 0.8, -0.01, 0.02, 0.03])
    T_map_armcam = se3_exp([-0.3, 0.2, 0.4, 0.03, 0.01, -0.02])

    expected = T_inv(T_map_armcam) @ T_map_headcam @ T_headcam_marker

    np.testing.assert_allclose(
        compute_armcam_marker_sample(T_map_headcam, T_headcam_marker, T_map_armcam),
        expected,
        atol=1e-12,
    )


def test_head_derived_preview_composes_head_pose_dynamic_marker_and_extrinsic():
    T_map_headimu = se3_exp([0.2, -0.1, 0.5, 0.02, -0.03, 0.04])
    T_headcam_headimu = se3_exp([0.01, 0.02, -0.03, -0.01, 0.02, 0.0])
    T_headcam_marker = se3_exp([0.4, 0.1, 0.8, -0.01, 0.02, 0.03])
    T_armcam_marker = se3_exp([-0.05, 0.02, -0.09, 0.03, -0.01, 0.02])

    T_map_headcam, T_map_marker, T_map_armcam = compose_head_derived_arm_pose(
        T_map_headimu,
        T_headcam_headimu,
        T_headcam_marker,
        T_armcam_marker,
    )

    expected_headcam = T_map_headimu @ T_inv(T_headcam_headimu)
    expected_marker = expected_headcam @ T_headcam_marker
    expected_armcam = expected_marker @ T_inv(T_armcam_marker)
    np.testing.assert_allclose(T_map_headcam, expected_headcam, atol=1e-12)
    np.testing.assert_allclose(T_map_marker, expected_marker, atol=1e-12)
    np.testing.assert_allclose(T_map_armcam, expected_armcam, atol=1e-12)


def test_head_derived_preview_covariance_is_finite_and_scales_with_inputs():
    T_map_headimu = se3_exp([0.2, -0.1, 0.5, 0.02, -0.03, 0.04])
    T_headcam_headimu = se3_exp([0.01, 0.02, -0.03, -0.01, 0.02, 0.0])
    T_headcam_marker = se3_exp([0.4, 0.1, 0.8, -0.01, 0.02, 0.03])
    T_armcam_marker = se3_exp([-0.05, 0.02, -0.09, 0.03, -0.01, 0.02])
    min_diag = np.array([1e-8, 1e-8, 1e-8, 1e-10, 1e-10, 1e-10], dtype=float)
    P_head = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])
    P_dynamic = np.diag([0.002, 0.002, 0.004, 0.0005, 0.0005, 0.0008])
    P_extrinsic = np.diag([0.0004, 0.0004, 0.0009, 0.0002, 0.0002, 0.0003])

    baseline = propagate_candidate_covariance(
        T_map_headimu,
        P_head,
        T_headcam_headimu,
        T_headcam_marker,
        P_dynamic,
        T_armcam_marker,
        P_extrinsic,
        min_diag,
    )
    larger = propagate_candidate_covariance(
        T_map_headimu,
        4.0 * P_head,
        T_headcam_headimu,
        T_headcam_marker,
        4.0 * P_dynamic,
        T_armcam_marker,
        4.0 * P_extrinsic,
        min_diag,
    )

    assert baseline.shape == (6, 6)
    assert np.all(np.isfinite(baseline))
    assert np.all(np.diag(baseline) > 0.0)
    assert np.all(np.diag(baseline)[3:] > min_diag[3:])
    assert np.trace(larger) > np.trace(baseline)


def test_head_derived_preview_loads_id2_extrinsic_with_robust_covariance():
    root = Path(__file__).resolve().parents[1]
    fallback = np.array([0.01, 0.01, 0.01, 0.001, 0.001, 0.001], dtype=float)

    extrinsic = load_arm_marker_extrinsic(
        root / "config" / "markers" / "arm_marker_extrinsics.yaml",
        marker_id=2,
        covariance_source="robust_diag_covariance_se3",
        fallback_covariance_diag=fallback,
    )

    assert extrinsic.marker_id == 2
    assert extrinsic.marker_frame == "arm_marker_2"
    assert extrinsic.parent_camera_frame == "arm_d435i_arm_color_optical_frame"
    assert extrinsic.covariance_source == "robust_diag_covariance_se3"
    assert extrinsic.T_armcam_marker.shape == (4, 4)
    assert extrinsic.covariance.shape == (6, 6)
    assert np.all(np.diag(extrinsic.covariance) > 0.0)


def test_head_derived_preview_interpolates_or_rejects_head_pose_matches():
    samples = [
        HeadPoseSample(stamp=None, stamp_sec=10.0, T_map_headimu=np.eye(4), covariance=np.eye(6)),
        HeadPoseSample(
            stamp=None,
            stamp_sec=10.1,
            T_map_headimu=se3_exp([0.1, 0.0, 0.0, 0.0, 0.0, 0.1]),
            covariance=2.0 * np.eye(6),
        ),
    ]

    matched = match_head_pose(samples, 10.05, max_dt_s=0.06)
    assert matched is not None
    assert matched.mode == "interpolated"
    np.testing.assert_allclose(matched.T_map_headimu[:3, 3], [0.05, 0.0, 0.0], atol=1e-12)

    assert match_head_pose(samples, 10.3, max_dt_s=0.06) is None


def test_dynamic_arm_measurement_composes_arm_imu_pose():
    T_map_headimu = se3_exp([0.2, -0.1, 0.5, 0.02, -0.03, 0.04])
    T_headcam_headimu = se3_exp([0.01, 0.02, -0.03, -0.01, 0.02, 0.0])
    T_headcam_marker = se3_exp([0.4, 0.1, 0.8, -0.01, 0.02, 0.03])
    T_armcam_marker = se3_exp([-0.05, 0.02, -0.09, 0.03, -0.01, 0.02])
    T_armcam_armimu = se3_exp([0.02, 0.0, -0.01, 0.01, 0.02, -0.01])

    T_map_headcam, T_map_marker, T_map_armcam, T_map_armimu = compose_dynamic_arm_imu_pose(
        T_map_headimu,
        T_headcam_headimu,
        T_headcam_marker,
        T_armcam_marker,
        T_armcam_armimu,
    )

    expected_headcam = T_map_headimu @ T_inv(T_headcam_headimu)
    expected_marker = expected_headcam @ T_headcam_marker
    expected_armcam = expected_marker @ T_inv(T_armcam_marker)
    expected_armimu = expected_armcam @ T_armcam_armimu
    np.testing.assert_allclose(T_map_headcam, expected_headcam, atol=1e-12)
    np.testing.assert_allclose(T_map_marker, expected_marker, atol=1e-12)
    np.testing.assert_allclose(T_map_armcam, expected_armcam, atol=1e-12)
    np.testing.assert_allclose(T_map_armimu, expected_armimu, atol=1e-12)


def test_dynamic_arm_measurement_covariance_is_finite_and_scales_with_inputs():
    T_map_headimu = se3_exp([0.2, -0.1, 0.5, 0.02, -0.03, 0.04])
    T_headcam_headimu = se3_exp([0.01, 0.02, -0.03, -0.01, 0.02, 0.0])
    T_headcam_marker = se3_exp([0.4, 0.1, 0.8, -0.01, 0.02, 0.03])
    T_armcam_marker = se3_exp([-0.05, 0.02, -0.09, 0.03, -0.01, 0.02])
    T_armcam_armimu = se3_exp([0.02, 0.0, -0.01, 0.01, 0.02, -0.01])
    min_diag = np.array([1e-8, 1e-8, 1e-8, 1e-10, 1e-10, 1e-10], dtype=float)
    P_head = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])
    P_dynamic = np.diag([0.002, 0.002, 0.004, 0.0005, 0.0005, 0.0008])
    P_marker = np.diag([0.0004, 0.0004, 0.0009, 0.0002, 0.0002, 0.0003])
    P_armcamimu = np.diag([0.0001, 0.0001, 0.0001, 0.00002, 0.00002, 0.00002])

    baseline = propagate_dynamic_arm_imu_covariance(
        T_map_headimu,
        P_head,
        T_headcam_headimu,
        T_headcam_marker,
        P_dynamic,
        T_armcam_marker,
        P_marker,
        T_armcam_armimu,
        P_armcamimu,
        min_diag,
    )
    larger = propagate_dynamic_arm_imu_covariance(
        T_map_headimu,
        4.0 * P_head,
        T_headcam_headimu,
        T_headcam_marker,
        4.0 * P_dynamic,
        T_armcam_marker,
        4.0 * P_marker,
        T_armcam_armimu,
        4.0 * P_armcamimu,
        min_diag,
    )

    assert baseline.shape == (6, 6)
    assert np.all(np.isfinite(baseline))
    assert np.all(np.diag(baseline) > 0.0)
    assert np.trace(larger) > np.trace(baseline)


def test_dynamic_arm_measurement_head_sync_modes_and_fallback_flags():
    samples = [
        HeadPoseMeasurementSample(
            stamp=None,
            stamp_sec=10.0,
            T_map_headimu=np.eye(4),
            covariance=np.eye(6),
            covariance_fallback=False,
        ),
        HeadPoseMeasurementSample(
            stamp=None,
            stamp_sec=10.1,
            T_map_headimu=se3_exp([0.1, 0.0, 0.0, 0.0, 0.0, 0.1]),
            covariance=2.0 * np.eye(6),
            covariance_fallback=True,
        ),
    ]

    matched = match_head_pose_measurement(samples, 10.05, max_dt_s=0.06)
    assert matched is not None
    assert matched.mode == "interpolated"
    assert matched.covariance_fallback is True
    np.testing.assert_allclose(matched.T_map_headimu[:3, 3], [0.05, 0.0, 0.0], atol=1e-12)

    nearest = match_head_pose_measurement(samples, 10.0, max_dt_s=0.01)
    assert nearest is not None
    assert nearest.mode in {"nearest", "interpolated"}

    assert match_head_pose_measurement(samples, 10.3, max_dt_s=0.06) is None


def test_dynamic_arm_measurement_gate_reports_bad_id2_quality():
    class FakeMsg:
        pass

    node = DynamicArmPoseMeasurementNode.__new__(DynamicArmPoseMeasurementNode)
    node.marker_id = 2
    node.head_camera_frame = "head_d435i_head_color_optical_frame"
    node.extrinsic = type("Extrinsic", (), {"marker_frame": "arm_marker_2"})()
    node.require_stable_dynamic_marker = True
    node.max_reprojection_error_px = 3.0
    node.max_marker_distance_m = 2.0
    node.max_view_angle_deg = 75.0
    node.min_marker_area_px2 = 800.0
    node.min_geometry_score = 0.35

    msg = FakeMsg()
    msg.marker_id = 2
    msg.header = type("Header", (), {"frame_id": "head_d435i_head_color_optical_frame"})()
    msg.camera_frame = "head_d435i_head_color_optical_frame"
    msg.marker_frame = "arm_marker_2"
    msg.hard_gate_passed = True
    msg.hard_gate_status = "accepted"
    msg.stable = True
    msg.reprojection_error_px = 3.5
    msg.distance_m = 0.5
    msg.view_angle_deg = 12.0
    msg.area_px2 = 10000.0
    msg.geometry_score = 0.9

    assert DynamicArmPoseMeasurementNode.dynamic_gate_reason(node, msg) == "dynamic_reprojection_error_too_high"


def test_robust_se3_estimate_recovers_known_transform_with_outliers():
    base = se3_exp([0.12, -0.04, 0.35, math.radians(2.0), math.radians(-1.0), math.radians(3.0)])
    samples = [
        base @ se3_exp([0.001 * i, -0.0005 * i, 0.0008 * i, 0.0002 * i, -0.0001 * i, 0.00015 * i])
        for i in range(-5, 6)
    ]
    samples.extend(
        [
            base @ se3_exp([0.25, 0.10, -0.12, math.radians(25.0), 0.0, 0.0]),
            base @ se3_exp([-0.20, -0.10, 0.15, 0.0, math.radians(-30.0), 0.0]),
        ]
    )

    estimate = robust_se3_estimate(samples)
    residual = se3_residual(base, estimate.T_estimate)

    assert RESIDUAL_ORDER == ["x", "y", "z", "roll", "pitch", "yaw"]
    assert len(estimate.inlier_indices) == 11
    assert len(estimate.outlier_indices) == 2
    assert estimate.residual_covariance.shape == (6, 6)
    assert estimate.estimate_covariance.shape == (6, 6)
    assert np.linalg.norm(residual[:3]) < 0.01
    assert math.degrees(np.linalg.norm(residual[3:])) < 1.0
