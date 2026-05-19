"""Launch the twist propagation node with visualization.

Loads parameters from config/twist_propagation.yaml and overlays
CLI arguments.

Launch arguments:
    active                  Start the node in active mode (default: false)
    input_cloud_topic       Input pointcloud topic
    hand_pose_topic         Hand pose topic
    odom_topic              Optional odometry topic for covariance (default: empty)
    rviz                    Launch RViz with the twist_propagation config (default: true)
    use_sim_time            Use simulation time (default: false)
"""
from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup_launch(context, *args, **kwargs):
    # Resolve config path relative to the package share directory
    config_path = Path(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "twist_propagation.yaml",
        )
    ).resolve()

    params = {}
    if config_path.exists():
        import yaml
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        params = dict(cfg.get("twist_propagation", {}).get("ros__parameters", {}))

    # Overlay launch arguments
    params["active"] = LaunchConfiguration("active")
    params["input_cloud_topic"] = LaunchConfiguration("input_cloud_topic")
    params["hand_pose_topic"] = LaunchConfiguration("hand_pose_topic")
    params["odom_topic"] = LaunchConfiguration("odom_topic")
    params["use_sim_time"] = LaunchConfiguration("use_sim_time")

    nodes = []

    # Twist propagation node
    twist_node = Node(
        package="twist_propagation",
        executable="twist_propagation_node",
        name="twist_propagation",
        output="screen",
        parameters=[params],
    )
    nodes.append(twist_node)

    # Optional RViz
    rviz_enabled = LaunchConfiguration("rviz").perform(context)
    if rviz_enabled.lower() in ("true", "1", "yes"):
        rviz_config = Path(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "..",
                "..",
                "rviz",
                "twist_propagation.rviz",
            )
        ).resolve()

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", str(rviz_config)],
            output="screen",
        )
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "active",
            default_value="false",
            description="Start the node in active mode.",
        ),
        DeclareLaunchArgument(
            "input_cloud_topic",
            default_value="/camera/depth/color/points",
            description="Input pointcloud topic.",
        ),
        DeclareLaunchArgument(
            "hand_pose_topic",
            default_value="/hand_pose",
            description="Hand pose topic.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="",
            description="Optional odometry topic for covariance initialization.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz with the twist_propagation config.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (bag) time.",
        ),
        OpaqueFunction(function=_setup_launch),
    ])
