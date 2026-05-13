"""OpenVINS Phase 2 launch for ARM D435i camera (serial 841612071768).

Camera: Intel RealSense D435i (built-in IMU) at arm mount.
IMU:    D435i built-in, united at 200 Hz on /arm/d435i_arm/imu.
Camera name convention matches cameras container: namespace=arm, name=d435i_arm.

Calibration is a placeholder from arm_d435i_310622071850 (same model, different unit).
Replace config/openvins/arm_d435i_841612071768/ with a proper Kalibr calibration run
once available.

When start_camera:=false, only launches the OpenVINS estimator node (for use
alongside the cameras container which already publishes the camera topics).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
        "arm_d435i_841612071768",
        "estimator_config.yaml",
    ])

    arm_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        condition=IfCondition(LaunchConfiguration("start_camera")),
        launch_arguments={
            "camera_namespace": "arm",
            "camera_name": "d435i_arm",
            "serial_no": "_841612071768",
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
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("verbosity", default_value="INFO"),
        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            description="Start the live arm RealSense camera. Set false when cameras container is running.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock, typically true during rosbag replay.",
        ),
        arm_camera,
        TimerAction(period=5.0, actions=[openvins_phase2]),
    ])
