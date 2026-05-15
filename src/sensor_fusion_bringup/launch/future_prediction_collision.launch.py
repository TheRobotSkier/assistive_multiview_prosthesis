"""Launch the future pose prediction and collision check node.

Loads parameters from config/future_prediction_collision.yaml and overlays
CLI arguments.  Both enable_prediction and enable_collision_check default
to false for safety — explicit opt-in is required.

Launch arguments:
    enable_prediction       Master enable for trajectory prediction (default: false)
    enable_collision_check  Master enable for proximity/collision checks (default: false)
    use_sim_time            Use simulation time (default: false)
"""
from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_config(context):
    pkg_dir = Path(FindPackageShare("sensor_fusion_bringup").perform(context))
    config_path = pkg_dir / "config" / "future_prediction_collision.yaml"
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _setup_launch(context, *args, **kwargs):
    config = _load_config(context)
    params = dict(config.get("prediction", {}))

    params["enable_prediction"] = LaunchConfiguration("enable_prediction")
    params["enable_collision_check"] = LaunchConfiguration("enable_collision_check")
    params["use_sim_time"] = LaunchConfiguration("use_sim_time")

    node = Node(
        package="sensor_fusion_bringup",
        executable="future_pose_prediction_collision_node.py",
        name="future_pose_prediction_collision",
        output="screen",
        parameters=[params],
    )

    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_prediction",
            default_value="false",
            description="Master enable for future trajectory prediction.",
        ),
        DeclareLaunchArgument(
            "enable_collision_check",
            default_value="false",
            description="Master enable for proximity/collision checks.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (bag) time.",
        ),
        OpaqueFunction(function=_setup_launch),
    ])
