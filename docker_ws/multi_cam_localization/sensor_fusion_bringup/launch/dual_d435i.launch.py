from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _bool_str(value) -> str:
    return "true" if _as_bool(value) else "false"


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


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
            "tf_prefix": f"{cam['namespace']}_",
            "enable_color": "true",
            "enable_depth": _bool_str(common.get("enable_depth", common["pointcloud_enable"])),
            "pointcloud.enable": _bool_str(common["pointcloud_enable"]),
            "pointcloud.stream_filter": str(common.get("pointcloud_stream_filter", 2)),
            "pointcloud.stream_index_filter": str(common.get("pointcloud_stream_index_filter", 0)),
            "pointcloud.ordered_pc": _bool_str(common.get("pointcloud_ordered_pc", False)),
            "pointcloud.allow_no_texture_points": _bool_str(common.get("pointcloud_allow_no_texture_points", False)),
            "align_depth.enable": _bool_str(common["align_depth_enable"]),
            "decimation_filter.enable": _bool_str(common.get("decimation_filter_enable", False) and common["pointcloud_enable"]),
            "decimation_filter.filter_magnitude": str(common.get("decimation_filter_magnitude", 2)),
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

    actions = [camera]
    if _as_bool(common["pointcloud_enable"]):
        actions.append(neon_fix)
    return actions


def _setup_launch(context, *args, **kwargs):
    data = _load_config(context)
    common = dict(data["common"])
    enable_pointclouds = LaunchConfiguration("enable_pointclouds").perform(context)
    if enable_pointclouds != "":
        common["pointcloud_enable"] = _as_bool(enable_pointclouds)
        common["enable_depth"] = common["pointcloud_enable"]
    actions = []
    actions.extend(_camera_actions(data["cameras"]["head"], common))
    actions.extend(_camera_actions(data["cameras"]["arm"], common))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_pointclouds",
            default_value="",
            description="Override config/d435i_cameras.yaml pointcloud_enable. Empty uses config.",
        ),
        DeclareLaunchArgument(
            "enable_pointcloud_neon_fix",
            default_value="true",
            description="Apply Jetson pointcloud__neon_.enable fix after startup.",
        ),
        OpaqueFunction(function=_setup_launch),
    ])
