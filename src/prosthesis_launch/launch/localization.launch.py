"""Single-machine localization producer launch.

Starts the full dual-D435i + OpenVINS + ArUco localization pipeline on the
same x86 machine that runs the consumer pipeline.  This is the producer side;
the consumer side (pointcloud_fusion, twist_propagation, etc.) is launched
separately via pipeline.launch.py.

Brings up:
  1. Head D435i camera + head OpenVINS + head pointcloud→marker_map node
  2. Arm D435i camera + arm OpenVINS + arm pointcloud→marker_map node
  3. Head ArUco marker pose node (publishes /head/marker_pose/observation)
  4. Arm ArUco marker pose node (publishes /arm/marker_pose/observation)

The OpenVINS odom topics are remapped to the /jetson/{head,arm}/odom names
the host pipeline expects (free DDS alias — see merge plan §5.4).

Usage:
  ros2 launch prosthesis_launch localization.launch.py
  ros2 launch prosthesis_launch localization.launch.py enable_pointclouds:=true
  ros2 launch prosthesis_launch localization.launch.py head_only:=true

For bag replay (cameras off):
  ros2 launch prosthesis_launch localization.launch.py start_cameras:=false use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    head_only_expr = PythonExpression([
        "'", LaunchConfiguration("head_only"), "'.lower() in ['true', '1', 'yes', 'on']"
    ])

    head_openvins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("sensor_fusion_bringup"),
            "/launch/head_d435i_openvins_phase2.launch.py",
        ]),
        launch_arguments={
            "verbosity": LaunchConfiguration("verbosity"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "start_camera": LaunchConfiguration("start_cameras"),
            "hold_back_imu_for_frames": LaunchConfiguration("hold_back_imu_for_frames"),
            "enable_pointclouds": LaunchConfiguration("enable_pointclouds"),
            "enable_marker_map_pointclouds": LaunchConfiguration("enable_marker_map_pointclouds"),
            "odom_output_topic": LaunchConfiguration("head_odom_topic"),
            "pointcloud_max_rate_hz": LaunchConfiguration("pointcloud_max_rate_hz"),
            "pointcloud_max_range_m": LaunchConfiguration("pointcloud_max_range_m"),
            "pointcloud_decimation_enable": LaunchConfiguration("pointcloud_decimation_enable"),
            "pointcloud_decimation_magnitude": LaunchConfiguration("pointcloud_decimation_magnitude"),
        }.items(),
    )

    arm_openvins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("sensor_fusion_bringup"),
            "/launch/arm_d435i_openvins_phase2.launch.py",
        ]),
        condition=UnlessCondition(head_only_expr),
        launch_arguments={
            "verbosity": LaunchConfiguration("verbosity"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "start_camera": LaunchConfiguration("start_cameras"),
            "hold_back_imu_for_frames": LaunchConfiguration("hold_back_imu_for_frames"),
            "enable_pointclouds": LaunchConfiguration("enable_pointclouds"),
            "enable_marker_map_pointclouds": LaunchConfiguration("enable_marker_map_pointclouds"),
            "odom_output_topic": LaunchConfiguration("arm_odom_topic"),
            "pointcloud_max_rate_hz": LaunchConfiguration("pointcloud_max_rate_hz"),
            "pointcloud_max_range_m": LaunchConfiguration("pointcloud_max_range_m"),
            "pointcloud_decimation_enable": LaunchConfiguration("pointcloud_decimation_enable"),
            "pointcloud_decimation_magnitude": LaunchConfiguration("pointcloud_decimation_magnitude"),
        }.items(),
    )

    head_aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("sensor_fusion_bringup"),
            "/launch/head_marker_pose_phase2.launch.py",
        ]),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "marker_detection_rate_hz": LaunchConfiguration("marker_detection_rate_hz"),
        }.items(),
    )

    arm_aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("sensor_fusion_bringup"),
            "/launch/arm_marker_pose_phase2.launch.py",
        ]),
        condition=UnlessCondition(head_only_expr),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "marker_detection_rate_hz": LaunchConfiguration("marker_detection_rate_hz"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "verbosity",
            default_value="INFO",
            description="OpenVINS log verbosity (INFO/DEBUG/WARNING).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock (true during rosbag replay).",
        ),
        DeclareLaunchArgument(
            "start_cameras",
            default_value="true",
            description="Start live RealSense cameras. Set false for bag replay.",
        ),
        DeclareLaunchArgument(
            "head_only",
            default_value="false",
            description="Run only the head camera/OpenVINS/ArUco (skip arm). "
                        "Useful for single-camera bring-up and debugging.",
        ),
        DeclareLaunchArgument(
            "hold_back_imu_for_frames",
            default_value="true",
            description="Publish IMU/image in timestamp order for OpenVINS.",
        ),
        DeclareLaunchArgument(
            "enable_pointclouds",
            default_value="false",
            description="Enable RealSense depth/color pointcloud generation.",
        ),
        DeclareLaunchArgument(
            "enable_marker_map_pointclouds",
            default_value="true",
            description="Start the marker_map pointcloud republisher when "
                        "raw pointclouds are enabled.",
        ),
        DeclareLaunchArgument(
            "head_odom_topic",
            default_value="/jetson/head/odom",
            description="Output topic for head OpenVINS odometry.",
        ),
        DeclareLaunchArgument(
            "arm_odom_topic",
            default_value="/jetson/arm/odom",
            description="Output topic for arm OpenVINS odometry.",
        ),
        DeclareLaunchArgument(
            "marker_detection_rate_hz",
            default_value="15.0",
            description="Maximum ArUco detection rate (0.0 = every frame).",
        ),
        DeclareLaunchArgument(
            "pointcloud_max_rate_hz",
            default_value="10.0",
            description="Max rate for the marker_map pointcloud republisher.",
        ),
        DeclareLaunchArgument(
            "pointcloud_max_range_m",
            default_value="2.0",
            description="Clip pointcloud points farther than this range (m).",
        ),
        DeclareLaunchArgument(
            "pointcloud_decimation_enable",
            default_value="true",
            description="Enable RealSense decimation filter.",
        ),
        DeclareLaunchArgument(
            "pointcloud_decimation_magnitude",
            default_value="3",
            description="RealSense decimation filter magnitude.",
        ),
        head_openvins,
        arm_openvins,
        head_aruco,
        arm_aruco,
    ])
