"""Launch both D435i cameras (head + arm) from the sensor_fusion_bringup config.

Uses the official realsense2_camera rs_launch.py for each camera and applies
the Jetson NEON pointcloud fix via TimerAction after camera initialization.

Launch arguments:
    enable_pointcloud_neon_fix  Apply Jetson NEON fix after startup (default: true)
"""
from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _bool_str(value) -> str:
    return "true" if bool(value) else "false"


def _load_config(context):
    package_dir = Path(FindPackageShare("sensor_fusion_bringup").perform(context))
    config_path = package_dir / "config" / "d435i_cameras.yaml"
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _camera_actions(cam, common):
    rs_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"),
        "launch",
        "rs_launch.py",
    ])

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            "camera_namespace": cam["namespace"],
            "camera_name": cam["name"],
            "serial_no": cam["serial_no"],
            "pointcloud.enable": _bool_str(common["pointcloud_enable"]),
            "align_depth.enable": _bool_str(common["align_depth_enable"]),
            "enable_gyro": _bool_str(common["enable_gyro"]),
            "enable_accel": _bool_str(common["enable_accel"]),
            "unite_imu_method": str(common["unite_imu_method"]),
            "depth_module.depth_profile": common["depth_profile"],
            "rgb_camera.color_profile": common["color_profile"],
        }.items(),
    )

    neon_fix = TimerAction(
        period=float(common["pointcloud_neon_delay_sec"]),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "param", "set",
                    f"/{cam['namespace']}/{cam['name']}",
                    common["pointcloud_neon_param"],
                    _bool_str(common["pointcloud_neon_value"]),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_pointcloud_neon_fix")),
            )
        ],
    )

    return [camera, neon_fix]


def _setup_launch(context, *args, **kwargs):
    data = _load_config(context)
    common = data["common"]
    actions = []
    actions.extend(_camera_actions(data["cameras"]["head"], common))
    actions.extend(_camera_actions(data["cameras"]["arm"], common))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_pointcloud_neon_fix",
            default_value="true",
            description="Apply Jetson pointcloud__neon_.enable fix after startup.",
        ),
        OpaqueFunction(function=_setup_launch),
    ])
