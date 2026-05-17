"""Launch both D435i cameras (head + arm) from the sensor_fusion_bringup config.

Uses the official realsense2_camera rs_launch.py for each camera and applies
the Jetson NEON pointcloud fix via TimerAction after camera initialization.

Launch arguments:
    camera_config              Config file in sensor_fusion_bringup/config
                               (default: d435i_cameras.yaml)
    enable_pointcloud_neon_fix  Apply Jetson NEON fix after startup (default: true)
"""
from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _bool_str(value) -> str:
    return "true" if bool(value) else "false"


def _load_config(context):
    package_dir = Path(FindPackageShare("sensor_fusion_bringup").perform(context))
    config_name = LaunchConfiguration("camera_config").perform(context)
    config_path = Path(config_name)
    if not config_path.is_absolute():
        config_path = package_dir / "config" / config_name
    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    data["_config_path"] = str(config_path)
    return data


def _camera_actions(cam, common):
    rs_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"),
        "launch",
        "rs_launch.py",
    ])

    has_builtin_imu = cam.get("has_builtin_imu", True)
    camera_args = {
        "camera_namespace": cam["namespace"],
        "camera_name": cam["name"],
        "serial_no": cam["serial_no"],
        "enable_depth": "true",
        "enable_color": "true",
        "enable_sync": _bool_str(common.get("enable_sync", True)),
        "pointcloud.enable": _bool_str(common["pointcloud_enable"]),
        "pointcloud.allow_no_texture_points": _bool_str(
            common.get("pointcloud_allow_no_texture_points", True)
        ),
        "pointcloud.stream_filter": str(common.get("pointcloud_stream_filter", 0)),
        "pointcloud.stream_index_filter": str(common.get("pointcloud_stream_index_filter", 0)),
        "decimation_filter.enable": _bool_str(common.get("decimation_filter_enable", True)),
        "decimation_filter.filter_magnitude": str(common.get("decimation_filter_magnitude", 2)),
        "align_depth.enable": _bool_str(common["align_depth_enable"]),
        "depth_module.depth_profile": common["depth_profile"],
        "rgb_camera.color_profile": common["color_profile"],
    }
    if has_builtin_imu:
        camera_args.update({
            "enable_gyro": _bool_str(common["enable_gyro"]),
            "enable_accel": _bool_str(common["enable_accel"]),
            "unite_imu_method": str(common["unite_imu_method"]),
            "gyro_fps": str(common.get("gyro_fps", "200")),
            "accel_fps": str(common.get("accel_fps", "200")),
        })
    else:
        camera_args.update({
            "enable_gyro": "false",
            "enable_accel": "false",
        })

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments=camera_args.items(),
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
            ),
            ExecuteProcess(
                cmd=[
                    "ros2", "param", "set",
                    f"/{cam['namespace']}/{cam['name']}",
                    common["pointcloud_neon_stream_filter_param"],
                    str(common["pointcloud_neon_stream_filter_value"]),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_pointcloud_neon_fix")),
            ),
        ],
    )

    actions = [camera, neon_fix]

    external_imu = cam.get("external_imu", {})
    if external_imu.get("enabled", False):
        actions.append(
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="sensor_fusion_bringup",
                        executable="i2c_mpu9250_imu_node.py",
                        name=f"{cam['name']}_external_imu",
                        output="screen",
                        parameters=[{
                            "bus": int(external_imu.get("i2c_bus", 7)),
                            "address": int(external_imu.get("i2c_address", 0x68)),
                            "frame_id": external_imu.get("frame_id", "head_imu"),
                            "topic": external_imu.get("topic", f"/{cam['namespace']}/{cam['name']}/imu"),
                            "publish_rate_hz": float(external_imu.get("publish_rate_hz", 200.0)),
                            "accel_noise_std": float(external_imu.get("accel_noise_std", 0.25)),
                            "gyro_noise_std": float(external_imu.get("gyro_noise_std", 0.03)),
                        }],
                    )
                ],
            )
        )

    return actions


def _setup_launch(context, *args, **kwargs):
    data = _load_config(context)
    common = data["common"]
    actions = [LogInfo(msg=f"Using camera config: {data['_config_path']}")]
    actions.extend(_camera_actions(data["cameras"]["head"], common))
    actions.extend(_camera_actions(data["cameras"]["arm"], common))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_config",
            default_value="d435i_cameras.yaml",
            description="Camera YAML file under sensor_fusion_bringup/config or absolute path.",
        ),
        DeclareLaunchArgument(
            "enable_pointcloud_neon_fix",
            default_value="true",
            description="Apply Jetson pointcloud__neon_.enable fix after startup.",
        ),
        OpaqueFunction(function=_setup_launch),
    ])
