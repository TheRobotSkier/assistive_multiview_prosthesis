from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import openvins_profile


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _create_openvins_node(context, *args, **kwargs):
    package_dir = Path(FindPackageShare("sensor_fusion_bringup").perform(context))
    profile_name = LaunchConfiguration("openvins_experiment_profile").perform(context).strip()
    overrides = openvins_profile.get_profile_overrides(package_dir, profile_name)

    head_config = overrides.get("head_config")
    if head_config:
        ov_config = str(package_dir / "config" / "openvins" / head_config)
    else:
        ov_config = str(
            package_dir / "config" / "openvins" / "head_d435i_336222071386" / "estimator_config.yaml"
        )

    marker_overrides = overrides.get("head_marker", {})

    params = {
        "use_sim_time": _as_bool(LaunchConfiguration("use_sim_time").perform(context)),
        "verbosity": LaunchConfiguration("verbosity").perform(context),
        "use_stereo": False,
        "max_cameras": 1,
        "config_path": ov_config,
        "global_frame_id": "marker_map",
        "imu_frame_id": "head_imu",
        "camera_frame_prefix": "head_cam",
        "publish_global_to_imu_tf": True,
        "publish_calibration_tf": True,
        "use_marker_pose_updates": True,
        "marker_pose_topic": "/head/marker_pose/observation",
        "marker_status_topic": "/ov_msckf/marker_update/status",
        "marker_global_frame_id": "marker_map",
        "marker_target_frame": "head_imu",
        "marker_fixed_ids": "0",
        "marker_time_tolerance_s": 0.05,
        "marker_chi2_gate": 16.81,
        "marker_noise_multiplier": 1.0,
        "marker_max_update_translation_m": 0.25,
        "marker_max_update_rotation_deg": 25.0,
        "marker_reset_translation_m": 0.50,
        "marker_reset_rotation_deg": 20.0,
        "marker_reset_min_samples": 5,
        "marker_reset_window_s": 0.50,
        "marker_reset_min_sample_dt_s": 0.10,
        "marker_reset_max_velocity_mps": 2.0,
        "marker_reset_min_velocity_std_mps": 0.05,
        "marker_reset_bias_gyro_std": 0.02,
        "marker_reset_bias_accel_std": 0.20,
    }
    params.update(marker_overrides)

    return Node(
        package="ov_msckf",
        executable="run_subscribe_msckf_marker",
        namespace="ov_msckf",
        name="run_subscribe_msckf_marker",
        output="screen",
        parameters=[params],
    )


def generate_launch_description():
    pointcloud_marker_map_enabled = PythonExpression([
        "'",
        LaunchConfiguration("enable_pointclouds"),
        "'.lower() in ['true', '1', 'yes', 'on'] and '",
        LaunchConfiguration("enable_marker_map_pointclouds"),
        "'.lower() in ['true', '1', 'yes', 'on']",
    ])

    pointcloud_decimation_enabled = PythonExpression([
        "'",
        LaunchConfiguration("enable_pointclouds"),
        "'.lower() in ['true', '1', 'yes', 'on'] and '",
        LaunchConfiguration("pointcloud_decimation_enable"),
        "'.lower() in ['true', '1', 'yes', 'on']",
    ])

    head_camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="head",
        name="d435i_head",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_camera")),
        parameters=[{
            "camera_name": "d435i_head",
            "serial_no": "_336222071386",
            "tf_prefix": "head_",
            "enable_color": True,
            "rgb_camera.color_profile": "640x480x30",
            "enable_gyro": True,
            "enable_accel": True,
            "unite_imu_method": 2,
            "hold_back_imu_for_frames": ParameterValue(LaunchConfiguration("hold_back_imu_for_frames"), value_type=bool),
            "gyro_fps": 200,
            "accel_fps": 200,
            "enable_depth": ParameterValue(LaunchConfiguration("enable_pointclouds"), value_type=bool),
            "enable_infra": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "depth_module.depth_profile": "640x480x15",
            "clip_distance": ParameterValue(
                PythonExpression([
                    "'",
                    LaunchConfiguration("pointcloud_max_range_m"),
                    "' if '",
                    LaunchConfiguration("enable_pointclouds"),
                    "'.lower() in ['true', '1', 'yes', 'on'] and float('",
                    LaunchConfiguration("pointcloud_max_range_m"),
                    "') > 0.0 else '-2.0'",
                ]),
                value_type=float,
            ),
            "pointcloud.enable": ParameterValue(LaunchConfiguration("enable_pointclouds"), value_type=bool),
            "pointcloud.stream_filter": 2,
            "pointcloud.stream_index_filter": 0,
            "pointcloud.ordered_pc": False,
            "pointcloud.allow_no_texture_points": False,
            "pointcloud__neon_.enable": ParameterValue(LaunchConfiguration("enable_pointclouds"), value_type=bool),
            "pointcloud__neon_.stream_filter": 2,
            "pointcloud__neon_.stream_index_filter": 0,
            "pointcloud__neon_.ordered_pc": False,
            "pointcloud__neon_.allow_no_texture_points": False,
            "align_depth.enable": False,
            "decimation_filter.enable": ParameterValue(pointcloud_decimation_enabled, value_type=bool),
            "decimation_filter.filter_magnitude": ParameterValue(LaunchConfiguration("pointcloud_decimation_magnitude"), value_type=int),
        }],
    )

    pointcloud_neon_fix = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "param",
                    "set",
                    "/head/d435i_head",
                    "pointcloud__neon_.enable",
                    "true",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_pointcloud_neon_fix")),
            )
        ],
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration("enable_pointclouds"),
                "'.lower() in ['true', '1', 'yes', 'on'] and '",
                LaunchConfiguration("start_camera"),
                "'.lower() in ['true', '1', 'yes', 'on']",
            ])
        ),
    )

    pointcloud_marker_map = Node(
        package="sensor_fusion_bringup",
        executable="pointcloud_to_frame_node",
        name="head_d435i_points_to_marker_map",
        output="screen",
        condition=IfCondition(pointcloud_marker_map_enabled),
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"input_topic": "/head/d435i_head/depth/color/points"},
            {"output_topic": "/head/d435i_head/points_marker_map"},
            {"target_frame": "marker_map"},
            {"camera_pose_frame": "head_cam0"},
            {"camera_color_optical_frame": "head_d435i_head_color_optical_frame"},
            {"marker_map_locked_topic": "/ov_msckf/marker_map_locked"},
            {"require_marker_map_locked": ParameterValue(LaunchConfiguration("pointcloud_require_marker_map_locked"), value_type=bool)},
            {"max_rate_hz": ParameterValue(LaunchConfiguration("pointcloud_max_rate_hz"), value_type=float)},
            {"voxel_leaf_m": ParameterValue(LaunchConfiguration("pointcloud_voxel_leaf_m"), value_type=float)},
            {"max_source_range_m": ParameterValue(LaunchConfiguration("pointcloud_max_range_m"), value_type=float)},
            {"transform_timeout_s": ParameterValue(LaunchConfiguration("pointcloud_transform_timeout_s"), value_type=float)},
            {"max_tf_age_s": ParameterValue(LaunchConfiguration("pointcloud_max_tf_age_s"), value_type=float)},
        ],
    )


    return LaunchDescription([
        DeclareLaunchArgument("verbosity", default_value="INFO"),
        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            description="Start the live head RealSense camera. Set false for raw bag replay.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock, typically true during rosbag replay.",
        ),
        DeclareLaunchArgument(
            "hold_back_imu_for_frames",
            default_value="true",
            description="Ask RealSense to publish IMU/image messages in timestamp order for OpenVINS.",
        ),
        DeclareLaunchArgument(
            "enable_pointclouds",
            default_value="false",
            description="Enable RealSense depth/color pointcloud generation. Marker-map republishing has a separate gate.",
        ),
        DeclareLaunchArgument(
            "enable_marker_map_pointclouds",
            default_value="true",
            description="Start the Jetson marker_map pointcloud republisher when raw pointclouds are enabled.",
        ),
        DeclareLaunchArgument("pointcloud_max_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("pointcloud_voxel_leaf_m", default_value="0.02"),
        DeclareLaunchArgument(
            "pointcloud_max_range_m",
            default_value="2.0",
            description="Clip/filter pointcloud points farther than this source-frame range. Set <=0 to disable.",
        ),
        DeclareLaunchArgument(
            "pointcloud_require_marker_map_locked",
            default_value="false",
            description="Drop transformed pointclouds until OpenVINS reports marker-map lock. False publishes whenever TF is available.",
        ),
        DeclareLaunchArgument("pointcloud_transform_timeout_s", default_value="0.02"),
        DeclareLaunchArgument("pointcloud_max_tf_age_s", default_value="0.50"),
        DeclareLaunchArgument(
            "pointcloud_decimation_enable",
            default_value="true",
            description="Enable the RealSense decimation filter when raw pointclouds are enabled.",
        ),
        DeclareLaunchArgument("pointcloud_decimation_magnitude", default_value="3"),
        DeclareLaunchArgument(
            "enable_pointcloud_neon_fix",
            default_value="false",
            description="Legacy delayed Jetson pointcloud__neon_.enable fix. Startup parameters normally handle this.",
        ),
        DeclareLaunchArgument(
            "openvins_experiment_profile",
            default_value="baseline",
            description=(
                "Named experiment profile from config/openvins_experiment_profiles.yaml. "
                "'baseline' = unchanged defaults. "
                "Available: baseline, marker_strong_ekf, marker_easier_initial_lock, "
                "reset_bias_policy, calib_extrinsics, zupt, imu_frame_variant, all_changes"
            ),
        ),
        head_camera,
        pointcloud_neon_fix,
        pointcloud_marker_map,
        TimerAction(period=5.0, actions=[OpaqueFunction(function=_create_openvins_node)]),
    ])
