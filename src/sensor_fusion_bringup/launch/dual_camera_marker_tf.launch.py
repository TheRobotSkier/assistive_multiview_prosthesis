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
        "camera_tf_markers.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to dual-camera marker TF YAML.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock, typically true during rosbag replay.",
        ),
        DeclareLaunchArgument(
            "marker_detection_rate_hz",
            default_value="15.0",
            description="Default per-camera marker detection rate. Per-camera YAML values override this.",
        ),
        DeclareLaunchArgument(
            "smoothing_alpha",
            default_value="-1.0",
            description="Override YAML smoothing alpha. Use -1.0 to keep YAML value.",
        ),

        Node(
            package="sensor_fusion_bringup",
            executable="camera_marker_tf_node.py",
            name="camera_marker_tf_node",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"config_file": LaunchConfiguration("config_file")},
                {
                    "marker_detection_rate_hz": ParameterValue(
                        LaunchConfiguration("marker_detection_rate_hz"),
                        value_type=float,
                    )
                },
                {
                    "smoothing_alpha": ParameterValue(
                        LaunchConfiguration("smoothing_alpha"),
                        value_type=float,
                    )
                },
            ],
        ),
    ])
