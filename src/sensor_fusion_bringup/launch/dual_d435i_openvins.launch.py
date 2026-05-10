"""
Dual D435i + OpenVINS monocular VIO launch file.

Launches two Intel RealSense D435i cameras (head + arm) with their built-in IMUs,
then starts two independent OpenVINS monocular VIO instances — one per camera.

Camera config is loaded from d435i_cameras.yaml.
Each OpenVINS instance uses its own estimator config in config/openvins/<camera>/.

Topic mapping per camera:
  head:  /head/d435i_head/color/image_raw  +  /head/d435i_head/imu
  arm:   /arm/d435i_arm/color/image_raw    +  /arm/d435i_arm/imu

OpenVINS output odometry:
  head:  /ov_msckf_head/odomimu
  arm:   /ov_msckf_arm/odomimu
"""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _str(value) -> str:
    """Convert bool -> 'true'/'false', everything else -> str."""
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _load_camera_config():
    """Load d435i_cameras.yaml from the sensor_fusion_bringup package."""
    # Using a direct Path approach since FindPackageShare needs a context.
    # This file is installed to share/sensor_fusion_bringup/config/
    pkg_share = Path(__file__).resolve().parents[1] / "share" / "sensor_fusion_bringup"
    config_path = pkg_share / "config" / "d435i_cameras.yaml"
    # Fallback: try the source tree location
    if not config_path.exists():
        config_path = (
            Path(__file__).resolve().parents[1] / "config" / "d435i_cameras.yaml"
        )
    with open(config_path) as f:
        return yaml.safe_load(f)


def _camera_launch(cam, common):
    """Generate a RealSense camera launch action for one camera."""
    rs_launch_path = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"),
        "launch",
        "rs_launch.py",
    ])

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments={
            "camera_namespace": cam["namespace"],
            "camera_name": cam["name"],
            "serial_no": cam["serial_no"],
            # RGB for VIO feature tracking
            "enable_color": "true",
            "rgb_camera.color_profile": common.get("color_profile", "640x480x30"),
            # IMU for VIO
            "enable_gyro": _str(common.get("enable_gyro", True)),
            "enable_accel": _str(common.get("enable_accel", True)),
            "unite_imu_method": str(common.get("unite_imu_method", 2)),
            "gyro_fps": str(common.get("gyro_fps", 200)),
            "accel_fps": str(common.get("accel_fps", 200)),
            # Depth/pointcloud optional (can be disabled for lightweight VIO)
            "enable_depth": _str(common.get("enable_depth", False)),
            "pointcloud.enable": _str(common.get("pointcloud_enable", False)),
            "align_depth.enable": _str(common.get("align_depth_enable", False)),
        }.items(),
    )


def _openvins_launch(namespace: str, config_subpath: str, rviz: str, verbosity: str):
    """Generate an OpenVINS monocular VIO launch action."""
    ov_launch_path = PathJoinSubstitution([
        FindPackageShare("ov_msckf"),
        "launch",
        "subscribe.launch.py",
    ])

    ov_config_path = PathJoinSubstitution([
        FindPackageShare("sensor_fusion_bringup"),
        "config",
        "openvins",
        config_subpath,
        "estimator_config.yaml",
    ])

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ov_launch_path),
        launch_arguments={
            "namespace": namespace,
            "config_path": ov_config_path,
            "use_stereo": "false",
            "max_cameras": "1",
            "rviz_enable": rviz,
            "verbosity": verbosity,
        }.items(),
    )


def generate_launch_description():
    rviz_arg = LaunchConfiguration("rviz_enable")
    verbosity_arg = LaunchConfiguration("verbosity")

    # Load camera definitions
    data = _load_camera_config()
    common = data.get("common", {})
    cameras = data.get("cameras", {})

    head_cam = cameras.get("head")
    arm_cam = cameras.get("arm")

    actions = []

    # ── Camera launches ──────────────────────────────────────────────────
    if head_cam:
        actions.append(_camera_launch(head_cam, common))
    if arm_cam:
        actions.append(_camera_launch(arm_cam, common))

    # ── OpenVINS launches (delayed to let camera topics appear) ──────────
    if head_cam:
        ov_head = _openvins_launch(
            namespace="ov_msckf_head",
            config_subpath="head_d435i_336222071386",
            rviz=rviz_arg,
            verbosity=verbosity_arg,
        )
        actions.append(TimerAction(period=5.0, actions=[ov_head]))

    if arm_cam:
        ov_arm = _openvins_launch(
            namespace="ov_msckf_arm",
            config_subpath="arm_d435i_310622071850",
            rviz="false",  # Only one RViz instance
            verbosity=verbosity_arg,
        )
        actions.append(TimerAction(period=6.0, actions=[ov_arm]))

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_enable",
            default_value="false",
            description="Enable RViz visualization for head camera OpenVINS.",
        ),
        DeclareLaunchArgument(
            "verbosity",
            default_value="INFO",
            description="OpenVINS logging verbosity (DEBUG, INFO, WARNING, ERROR).",
        ),
        *actions,
    ])
