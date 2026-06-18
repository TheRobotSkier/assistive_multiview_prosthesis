from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("sensor_fusion_bringup")

    default_config = os.path.join(
        pkg_share,
        "config",
        "markers",
        "arm_aruco_map.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to marker map YAML",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock, typically true during rosbag replay.",
        ),
        DeclareLaunchArgument(
            "marker_detection_rate_hz",
            default_value="15.0",
            description="Maximum ArUco detection rate. Set 0.0 to process every image frame.",
        ),

        Node(
            package="sensor_fusion_bringup",
            executable="aruco_marker_pose_node.py",
            name="aruco_marker_pose_node_arm_phase2",
            output="screen",
            remappings=[
                ("/tf", "/tf_raw"),
            ],
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"config_file": LaunchConfiguration("config_file")},
                {"correction_enabled_override": "true"},
                {"marker_detection_rate_hz": ParameterValue(LaunchConfiguration("marker_detection_rate_hz"), value_type=float)},
            ],
        ),
    ])
