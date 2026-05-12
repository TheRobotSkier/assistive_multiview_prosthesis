"""Unified bringup for Robotlab Jetson Orin Nano.

Launches both D435 cameras via IncludeLaunchDescription + rs_launch.py
(for proper pointcloud support), both ICM-20948 IMUs, and EKF odometry.

Camera/IMU/pointcloud parameters are read from d435_cameras.yaml.
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_IMU_CONFIGS = {
    "cam0": "/miahand_ws/src/multi_cam_localization/imu_driver/config/imu_cam0.yaml",
    "cam1": "/miahand_ws/src/multi_cam_localization/imu_driver/config/imu_cam1.yaml",
}


def _load_config():
    """Load d435_cameras.yaml and return parsed dict."""
    config_path = None

    # Try ament_index first (ROS2 runtime)
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory("sensor_fusion_bringup")
        config_path = os.path.join(pkg_share, "config", "d435_cameras.yaml")
    except Exception:
        pass

    # Fallback: relative to this launch file
    if config_path is None or not os.path.exists(config_path):
        launch_dir = os.path.dirname(os.path.realpath(__file__))
        config_path = os.path.join(launch_dir, "..", "config", "d435_cameras.yaml")

    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    cfg = _load_config()

    # Camera entries
    cameras = cfg["cameras"]

    # New-style keys added in E1.1-E1.3
    profiles = cfg.get("profiles", {})
    pointcloud_cfg = cfg.get("pointcloud", {})
    align_depth_cfg = cfg.get("align_depth", {})

    # Common section (legacy fallback values)
    common = cfg.get("common", {})

    # Read from YAML, with sensible defaults
    depth_profile = profiles.get("depth", common.get("depth_profile", "424x240x15"))
    color_profile = profiles.get("color", common.get("color_profile", "640x480x15"))
    pc_enable = str(pointcloud_cfg.get("enable", common.get("pointcloud_enable", True))).lower()
    pc_stream_filter = str(pointcloud_cfg.get("stream_filter", common.get("pointcloud_stream_filter", 2)))
    align_enable = str(align_depth_cfg.get("enable", common.get("align_depth_enable", True))).lower()
    enable_gyro = str(common.get("enable_gyro", False)).lower()
    enable_accel = str(common.get("enable_accel", False)).lower()
    unite_imu = str(common.get("unite_imu_method", 0))

    actions = []
    rs_launch_path = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"), "launch", "rs_launch.py",
    ])

    # ---- Cameras via rs_launch.py (params from d435_cameras.yaml) ----
    for label, cam in cameras.items():
        actions.append(LogInfo(msg=f"Starting {label} camera: {cam[serial_no]}"))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_path),
            launch_arguments={
                "camera_namespace": cam["namespace"],
                "camera_name":      cam["name"],
                "serial_no":        f"\"{cam[serial_no]}\"",
                "pointcloud.enable":          pc_enable,
                "pointcloud.stream_filter":   pc_stream_filter,
                "align_depth.enable":         align_enable,
                "enable_color":               "true",
                "enable_gyro":                enable_gyro,
                "enable_accel":               enable_accel,
                "unite_imu_method":           unite_imu,
                "enable_infra1":              "false",
                "enable_infra2":              "false",
                "initial_reset":              "true",
                "depth_module.depth_profile": depth_profile,
                "rgb_camera.color_profile":   color_profile,
            }.items(),
        ))

    # Jetson NEON pointcloud fix (delayed, after cameras init)
    apply_fix = LaunchConfiguration("enable_pointcloud_neon_fix")
    neon_param = common.get("pointcloud_neon_param", "pointcloud__neon_.enable")
    neon_val = str(common.get("pointcloud_neon_value", True)).lower()
    neon_delay = float(common.get("pointcloud_neon_delay_sec", 6.0))
    for label, cam in cameras.items():
        param_path = f"/{cam[namespace]}/{cam[name]}"
        actions.append(TimerAction(
            period=neon_delay,
            actions=[ExecuteProcess(
                cmd=["ros2", "param", "set", param_path, neon_param, neon_val],
                output="screen",
                condition=IfCondition(apply_fix),
            )],
        ))

    # ---- IMU drivers ----
    for ns, cfg_path in _IMU_CONFIGS.items():
        actions.append(Node(
            package="imu_driver", executable="imu_node", name="imu_node",
            namespace=ns, output="screen", parameters=[cfg_path],
        ))

    # ---- EKF ----
    ekf_path = PathJoinSubstitution([
        FindPackageShare("sensor_fusion_bringup"), "launch", "two_imus_ekf.launch.py",
    ])
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_path),
    ))

    return LaunchDescription([
        DeclareLaunchArgument("enable_pointcloud_neon_fix", default_value="true",
            description="Apply Jetson pointcloud__neon_.enable fix after startup."),
        LogInfo(msg="=== Robotlab bringup: cameras + IMUs + EKF ==="),
        *actions,
    ])
