from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    pointcloud_enabled = _as_bool(common["pointcloud_enable"])
    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace=cam["namespace"],
        name=cam["name"],
        output="screen",
        emulate_tty=True,
        parameters=[{
            "camera_name": cam["name"],
            "serial_no": cam["serial_no"],
            "tf_prefix": f"{cam['namespace']}_",
            "enable_color": True,
            "enable_depth": _as_bool(common.get("enable_depth", common["pointcloud_enable"])),
            "enable_infra": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "clip_distance": float(common.get("pointcloud_max_range_m", -2.0))
            if pointcloud_enabled and float(common.get("pointcloud_max_range_m", -2.0)) > 0.0
            else -2.0,
            "pointcloud.enable": pointcloud_enabled,
            "pointcloud.stream_filter": int(common.get("pointcloud_stream_filter", 2)),
            "pointcloud.stream_index_filter": int(common.get("pointcloud_stream_index_filter", 0)),
            "pointcloud.ordered_pc": _as_bool(common.get("pointcloud_ordered_pc", False)),
            "pointcloud.allow_no_texture_points": _as_bool(common.get("pointcloud_allow_no_texture_points", False)),
            "pointcloud__neon_.enable": pointcloud_enabled,
            "pointcloud__neon_.stream_filter": int(common.get("pointcloud_stream_filter", 2)),
            "pointcloud__neon_.stream_index_filter": int(common.get("pointcloud_stream_index_filter", 0)),
            "pointcloud__neon_.ordered_pc": _as_bool(common.get("pointcloud_ordered_pc", False)),
            "pointcloud__neon_.allow_no_texture_points": _as_bool(common.get("pointcloud_allow_no_texture_points", False)),
            "align_depth.enable": _as_bool(common["align_depth_enable"]),
            "decimation_filter.enable": _as_bool(common.get("decimation_filter_enable", False)) and pointcloud_enabled,
            "decimation_filter.filter_magnitude": int(common.get("decimation_filter_magnitude", 2)),
            "enable_gyro": _as_bool(common["enable_gyro"]),
            "enable_accel": _as_bool(common["enable_accel"]),
            "unite_imu_method": int(common["unite_imu_method"]),
            "hold_back_imu_for_frames": _as_bool(common.get("hold_back_imu_for_frames", True)),
            "depth_module.depth_profile": common["depth_profile"],
            "rgb_camera.color_profile": common["color_profile"],
        }],
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
    hold_back_imu_for_frames = LaunchConfiguration("hold_back_imu_for_frames").perform(context)
    if hold_back_imu_for_frames != "":
        common["hold_back_imu_for_frames"] = _as_bool(hold_back_imu_for_frames)
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
            default_value="false",
            description="Legacy delayed Jetson pointcloud__neon_.enable fix. Startup parameters normally handle this.",
        ),
        DeclareLaunchArgument(
            "hold_back_imu_for_frames",
            default_value="",
            description="Override config/d435i_cameras.yaml hold_back_imu_for_frames. Empty uses config.",
        ),
        OpaqueFunction(function=_setup_launch),
    ])
