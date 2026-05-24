from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ov_config = PathJoinSubstitution([
        FindPackageShare("sensor_fusion_bringup"),
        "config",
        "openvins",
        "arm_d435i_310622071850",
        "estimator_config.yaml",
    ])

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

    arm_camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="arm",
        name="d435i_arm",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_camera")),
        parameters=[{
            "camera_name": "d435i_arm",
            "serial_no": "_310622071850",
            "tf_prefix": "arm_",
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
                    "/arm/d435i_arm",
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
        name="arm_d435i_points_to_marker_map",
        output="screen",
        condition=IfCondition(pointcloud_marker_map_enabled),
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"input_topic": "/arm/d435i_arm/depth/color/points"},
            {"output_topic": "/arm/d435i_arm/points_marker_map"},
            {"target_frame": "marker_map"},
            {"camera_pose_frame": "arm_cam0"},
            {"camera_color_optical_frame": "arm_d435i_arm_color_optical_frame"},
            {"marker_map_locked_topic": "/ov_msckf_arm/marker_map_locked"},
            {"require_marker_map_locked": ParameterValue(LaunchConfiguration("pointcloud_require_marker_map_locked"), value_type=bool)},
            {"max_rate_hz": ParameterValue(LaunchConfiguration("pointcloud_max_rate_hz"), value_type=float)},
            {"voxel_leaf_m": ParameterValue(LaunchConfiguration("pointcloud_voxel_leaf_m"), value_type=float)},
            {"max_source_range_m": ParameterValue(LaunchConfiguration("pointcloud_max_range_m"), value_type=float)},
            {"transform_timeout_s": ParameterValue(LaunchConfiguration("pointcloud_transform_timeout_s"), value_type=float)},
            {"max_tf_age_s": ParameterValue(LaunchConfiguration("pointcloud_max_tf_age_s"), value_type=float)},
        ],
    )

    openvins_phase2 = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf_marker",
        namespace="ov_msckf_arm",
        name="run_subscribe_msckf_marker",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": False},
            {"max_cameras": 1},
            {"config_path": ov_config},
            {"global_frame_id": "marker_map"},
            {"imu_frame_id": "arm_imu"},
            {"camera_frame_prefix": "arm_cam"},
            {"publish_global_to_imu_tf": True},
            {"publish_calibration_tf": True},
            {"use_marker_pose_updates": True},
            {"marker_pose_topic": "/arm/marker_pose/observation"},
            {"marker_status_topic": "/ov_msckf_arm/marker_update/status"},
            {"marker_global_frame_id": "marker_map"},
            {"marker_target_frame": "arm_imu"},
            {"marker_fixed_ids": "0"},
            {"marker_time_tolerance_s": 0.05},
            {"marker_chi2_gate": 16.81},
            {"marker_noise_multiplier": 1.0},
            {"marker_max_update_translation_m": 0.25},
            {"marker_max_update_rotation_deg": 25.0},
            {"marker_reset_translation_m": 0.50},
            {"marker_reset_rotation_deg": 20.0},
            {"marker_reset_min_samples": 5},
            {"marker_reset_window_s": 0.50},
            {"marker_reset_min_sample_dt_s": 0.10},
            {"marker_reset_max_velocity_mps": 2.0},
            {"marker_reset_min_velocity_std_mps": 0.05},
            {"marker_reset_bias_gyro_std": 0.02},
            {"marker_reset_bias_accel_std": 0.20},
            {
                "use_dynamic_arm_pose_updates": ParameterValue(
                    LaunchConfiguration("use_dynamic_arm_pose_updates"),
                    value_type=bool,
                )
            },
            {
                "dynamic_arm_measurement_only": ParameterValue(
                    LaunchConfiguration("dynamic_arm_measurement_only"),
                    value_type=bool,
                )
            },
            {"dynamic_arm_pose_topic": LaunchConfiguration("dynamic_arm_pose_topic")},
            {"dynamic_arm_status_topic": LaunchConfiguration("dynamic_arm_status_topic")},
            {"dynamic_arm_global_frame_id": LaunchConfiguration("dynamic_arm_global_frame_id")},
            {"dynamic_arm_target_frame": LaunchConfiguration("dynamic_arm_target_frame")},
            {"dynamic_arm_source_camera_frame": LaunchConfiguration("dynamic_arm_source_camera_frame")},
            {"dynamic_arm_marker_frame": LaunchConfiguration("dynamic_arm_marker_frame")},
            {"dynamic_arm_marker_id": ParameterValue(LaunchConfiguration("dynamic_arm_marker_id"), value_type=int)},
            {
                "dynamic_arm_time_tolerance_s": ParameterValue(
                    LaunchConfiguration("dynamic_arm_time_tolerance_s"),
                    value_type=float,
                )
            },
            {
                "dynamic_arm_noise_multiplier": ParameterValue(
                    LaunchConfiguration("dynamic_arm_noise_multiplier"),
                    value_type=float,
                )
            },
            {"dynamic_arm_chi2_gate": ParameterValue(LaunchConfiguration("dynamic_arm_chi2_gate"), value_type=float)},
            {
                "dynamic_arm_max_update_translation_m": ParameterValue(
                    LaunchConfiguration("dynamic_arm_max_update_translation_m"),
                    value_type=float,
                )
            },
            {
                "dynamic_arm_max_update_rotation_deg": ParameterValue(
                    LaunchConfiguration("dynamic_arm_max_update_rotation_deg"),
                    value_type=float,
                )
            },
            {
                "dynamic_arm_min_update_interval_s": ParameterValue(
                    LaunchConfiguration("dynamic_arm_min_update_interval_s"),
                    value_type=float,
                )
            },
            {
                "dynamic_arm_skip_after_fixed_marker_s": ParameterValue(
                    LaunchConfiguration("dynamic_arm_skip_after_fixed_marker_s"),
                    value_type=float,
                )
            },
            {"dynamic_arm_allow_initial_lock": ParameterValue(LaunchConfiguration("dynamic_arm_allow_initial_lock"), value_type=bool)},
            {"dynamic_arm_allow_reanchor": ParameterValue(LaunchConfiguration("dynamic_arm_allow_reanchor"), value_type=bool)},
            {
                "dynamic_arm_reanchor_measurement_only": ParameterValue(
                    LaunchConfiguration("dynamic_arm_reanchor_measurement_only"),
                    value_type=bool,
                )
            },
            {"dynamic_arm_reanchor_min_samples": ParameterValue(LaunchConfiguration("dynamic_arm_reanchor_min_samples"), value_type=int)},
            {"dynamic_arm_reanchor_window_s": ParameterValue(LaunchConfiguration("dynamic_arm_reanchor_window_s"), value_type=float)},
            {
                "dynamic_arm_reanchor_min_sample_dt_s": ParameterValue(
                    LaunchConfiguration("dynamic_arm_reanchor_min_sample_dt_s"),
                    value_type=float,
                )
            },
            {
                "dynamic_arm_reanchor_max_velocity_mps": ParameterValue(
                    LaunchConfiguration("dynamic_arm_reanchor_max_velocity_mps"),
                    value_type=float,
                )
            },
            {
                "dynamic_arm_reanchor_max_sample_translation_std_m": ParameterValue(
                    LaunchConfiguration("dynamic_arm_reanchor_max_sample_translation_std_m"),
                    value_type=float,
                )
            },
            {
                "dynamic_arm_reanchor_max_sample_rotation_std_deg": ParameterValue(
                    LaunchConfiguration("dynamic_arm_reanchor_max_sample_rotation_std_deg"),
                    value_type=float,
                )
            },
            {
                "dynamic_arm_reanchor_trigger_translation_m": ParameterValue(
                    LaunchConfiguration("dynamic_arm_reanchor_trigger_translation_m"),
                    value_type=float,
                )
            },
            {
                "dynamic_arm_reanchor_trigger_rotation_deg": ParameterValue(
                    LaunchConfiguration("dynamic_arm_reanchor_trigger_rotation_deg"),
                    value_type=float,
                )
            },
            {
                "dynamic_arm_reanchor_cooldown_s": ParameterValue(
                    LaunchConfiguration("dynamic_arm_reanchor_cooldown_s"),
                    value_type=float,
                )
            },
            {
                "dynamic_arm_reanchor_skip_after_fixed_marker_s": ParameterValue(
                    LaunchConfiguration("dynamic_arm_reanchor_skip_after_fixed_marker_s"),
                    value_type=float,
                )
            },
            {
                "dynamic_arm_reanchor_covariance_multiplier": ParameterValue(
                    LaunchConfiguration("dynamic_arm_reanchor_covariance_multiplier"),
                    value_type=float,
                )
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("verbosity", default_value="INFO"),
        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            description="Start the live arm RealSense camera. Set false for raw bag replay.",
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
            "use_dynamic_arm_pose_updates",
            default_value="false",
            description="Enable conservative dynamic ID2 arm pose update consumption in arm OpenVINS.",
        ),
        DeclareLaunchArgument(
            "dynamic_arm_measurement_only",
            default_value="true",
            description="Log and gate dynamic arm measurements without mutating the OpenVINS state.",
        ),
        DeclareLaunchArgument(
            "dynamic_arm_pose_topic",
            default_value="/arm/marker_pose/dynamic_arm_pose_observation",
            description="OpenVINS-facing dynamic arm pose observation topic.",
        ),
        DeclareLaunchArgument(
            "dynamic_arm_status_topic",
            default_value="/ov_msckf_arm/dynamic_arm_update/status",
            description="Status topic for dynamic arm pose update decisions.",
        ),
        DeclareLaunchArgument("dynamic_arm_global_frame_id", default_value="marker_map"),
        DeclareLaunchArgument("dynamic_arm_target_frame", default_value="arm_imu"),
        DeclareLaunchArgument("dynamic_arm_source_camera_frame", default_value="head_d435i_head_color_optical_frame"),
        DeclareLaunchArgument("dynamic_arm_marker_frame", default_value="arm_marker_2"),
        DeclareLaunchArgument("dynamic_arm_marker_id", default_value="2"),
        DeclareLaunchArgument("dynamic_arm_time_tolerance_s", default_value="0.05"),
        DeclareLaunchArgument("dynamic_arm_noise_multiplier", default_value="4.0"),
        DeclareLaunchArgument("dynamic_arm_chi2_gate", default_value="16.81"),
        DeclareLaunchArgument("dynamic_arm_max_update_translation_m", default_value="0.35"),
        DeclareLaunchArgument("dynamic_arm_max_update_rotation_deg", default_value="15.0"),
        DeclareLaunchArgument("dynamic_arm_min_update_interval_s", default_value="0.10"),
        DeclareLaunchArgument("dynamic_arm_skip_after_fixed_marker_s", default_value="0.50"),
        DeclareLaunchArgument("dynamic_arm_allow_initial_lock", default_value="false"),
        DeclareLaunchArgument("dynamic_arm_allow_reanchor", default_value="false"),
        DeclareLaunchArgument("dynamic_arm_reanchor_measurement_only", default_value="true"),
        DeclareLaunchArgument("dynamic_arm_reanchor_min_samples", default_value="5"),
        DeclareLaunchArgument("dynamic_arm_reanchor_window_s", default_value="2.0"),
        DeclareLaunchArgument("dynamic_arm_reanchor_min_sample_dt_s", default_value="0.50"),
        DeclareLaunchArgument("dynamic_arm_reanchor_max_velocity_mps", default_value="2.0"),
        DeclareLaunchArgument("dynamic_arm_reanchor_max_sample_translation_std_m", default_value="0.12"),
        DeclareLaunchArgument("dynamic_arm_reanchor_max_sample_rotation_std_deg", default_value="8.0"),
        DeclareLaunchArgument("dynamic_arm_reanchor_trigger_translation_m", default_value="0.75"),
        DeclareLaunchArgument("dynamic_arm_reanchor_trigger_rotation_deg", default_value="20.0"),
        DeclareLaunchArgument("dynamic_arm_reanchor_cooldown_s", default_value="5.0"),
        DeclareLaunchArgument("dynamic_arm_reanchor_skip_after_fixed_marker_s", default_value="3.0"),
        DeclareLaunchArgument("dynamic_arm_reanchor_covariance_multiplier", default_value="2.0"),
        arm_camera,
        pointcloud_neon_fix,
        pointcloud_marker_map,
        TimerAction(period=5.0, actions=[openvins_phase2]),
    ])
