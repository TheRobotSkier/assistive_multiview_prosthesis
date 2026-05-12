"""Unified bringup for Robotlab Jetson Orin Nano.

Launches both D435 cameras via IncludeLaunchDescription + rs_launch.py
(for proper pointcloud support), both ICM-20948 IMUs, and EKF odometry.
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_CAMERA_CONFIG = {
    "head": {"serial": "827112072033", "namespace": "head", "name": "d435_head"},
    "arm":  {"serial": "829212072207", "namespace": "arm",  "name": "d435_arm"},
}

_IMU_CONFIGS = {
    "cam0": "/miahand_ws/src/multi_cam_localization/imu_driver/config/imu_cam0.yaml",
    "cam1": "/miahand_ws/src/multi_cam_localization/imu_driver/config/imu_cam1.yaml",
}


def _bool_str(val):
    return "true" if val else "false"


def generate_launch_description():
    actions = []
    rs_launch_path = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"), "launch", "rs_launch.py",
    ])

    # ---- Cameras via rs_launch.py (proper YAML param handling for pointclouds) ----
    for label, cam in _CAMERA_CONFIG.items():
        actions.append(LogInfo(msg=f"Starting {label} camera: {cam['serial']}"))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_path),
            launch_arguments={
                "camera_namespace": cam["namespace"],
                "camera_name":      cam["name"],
                "serial_no":        f"'{cam['serial']}'",
                "pointcloud.enable":          "true",
                "pointcloud.stream_filter":   "2",
                "align_depth.enable":         "true",
                "enable_color":               "true",
                "enable_gyro":                "false",
                "enable_accel":               "false",
                "unite_imu_method":           "0",
                "enable_infra1":              "false",
                "enable_infra2":              "false",
                "initial_reset":              "true",
                "depth_module.depth_profile": "424x240x15",
                "rgb_camera.color_profile":   "640x480x15",
            }.items(),
        ))

    # Jetson NEON pointcloud fix (delayed, after cameras init)
    apply_fix = LaunchConfiguration("enable_pointcloud_neon_fix")
    for label, cam in _CAMERA_CONFIG.items():
        param_path = f"/{cam['namespace']}/{cam['name']}"
        actions.append(TimerAction(
            period=8.0,
            actions=[ExecuteProcess(
                cmd=["ros2", "param", "set", param_path, "pointcloud__neon_.enable", "true"],
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
