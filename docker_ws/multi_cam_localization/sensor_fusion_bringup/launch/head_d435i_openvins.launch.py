from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rs_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"),
        "launch",
        "rs_launch.py",
    ])

    ov_launch = PathJoinSubstitution([
        FindPackageShare("ov_msckf"),
        "launch",
        "subscribe.launch.py",
    ])

    ov_config = PathJoinSubstitution([
        FindPackageShare("sensor_fusion_bringup"),
        "config",
        "openvins",
        "head_d435i_336222071386",
        "estimator_config.yaml",
    ])

    head_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            "camera_namespace": "head",
            "camera_name": "d435i_head",
            "serial_no": "_336222071386",

            # RGB monocular + IMU VIO input
            "enable_color": "true",
            "rgb_camera.color_profile": "640x480x30",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "gyro_fps": "200",
            "accel_fps": "200",

            # Keep first OpenVINS bringup lightweight
            "enable_depth": "false",
            "pointcloud.enable": "false",
            "align_depth.enable": "false",
        }.items(),
    )

    openvins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ov_launch),
        launch_arguments={
            "namespace": "ov_msckf",
            "config_path": ov_config,
            "use_stereo": "false",
            "max_cameras": "1",
            "rviz_enable": LaunchConfiguration("rviz_enable"),
            "verbosity": LaunchConfiguration("verbosity"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz_enable", default_value="false"),
        DeclareLaunchArgument("verbosity", default_value="INFO"),

        head_camera,

        # Give RealSense topics a few seconds to appear before OpenVINS subscribes.
        TimerAction(period=5.0, actions=[openvins]),
    ])
