from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rs_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"),
        "launch",
        "rs_launch.py",
    ])

    ov_config = PathJoinSubstitution([
        FindPackageShare("sensor_fusion_bringup"),
        "config",
        "openvins",
        "arm_d435i_310622071850",
        "estimator_config.yaml",
    ])

    arm_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        condition=IfCondition(LaunchConfiguration("start_camera")),
        launch_arguments={
            "camera_namespace": "arm",
            "camera_name": "d435i_arm",
            "serial_no": "_310622071850",
            "enable_color": "true",
            "rgb_camera.color_profile": "640x480x30",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "gyro_fps": "200",
            "accel_fps": "200",
            "enable_depth": "false",
            "pointcloud.enable": "false",
            "align_depth.enable": "false",
        }.items(),
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
            {"dynamic_arm_global_frame_id": "marker_map"},
            {"dynamic_arm_target_frame": "arm_imu"},
            {"dynamic_arm_source_camera_frame": "head_d435i_head_color_optical_frame"},
            {"dynamic_arm_marker_frame": "arm_marker_2"},
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
        DeclareLaunchArgument("dynamic_arm_marker_id", default_value="2"),
        DeclareLaunchArgument("dynamic_arm_time_tolerance_s", default_value="0.05"),
        DeclareLaunchArgument("dynamic_arm_noise_multiplier", default_value="4.0"),
        DeclareLaunchArgument("dynamic_arm_chi2_gate", default_value="16.81"),
        DeclareLaunchArgument("dynamic_arm_max_update_translation_m", default_value="0.35"),
        DeclareLaunchArgument("dynamic_arm_max_update_rotation_deg", default_value="15.0"),
        DeclareLaunchArgument("dynamic_arm_min_update_interval_s", default_value="0.10"),
        DeclareLaunchArgument("dynamic_arm_skip_after_fixed_marker_s", default_value="0.50"),
        arm_camera,
        TimerAction(period=5.0, actions=[openvins_phase2]),
    ])
