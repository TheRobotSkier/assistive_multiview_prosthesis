from pathlib import Path
import sys

import numpy as np
import yaml


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from aruco_marker_pose_node import compute_marker_map_poses, load_kalibr_imucam, T_cam_body_display, T_inv  # noqa: E402


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
