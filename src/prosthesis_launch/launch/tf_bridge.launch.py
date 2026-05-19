"""Launch only the OpenVINS to RealSense TF bridge."""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_CONFIG = os.environ.get(
    "PROSTHESIS_CONFIG",
    "/prosthesis_ws/config/prosthesis_config.yaml",
)


def _load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _node_params(config: dict, node_name: str) -> dict:
    section = config.get(node_name, {})
    if not isinstance(section, dict):
        return {}
    ros_params = section.get("ros__parameters")
    if isinstance(ros_params, dict):
        return dict(ros_params)
    return dict(section)


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    config = _load_config(config_file)

    return [
        Node(
            package="camera",
            executable="openvins_realsense_tf_bridge_node",
            name="openvins_realsense_tf_bridge",
            parameters=[_node_params(config, "openvins_realsense_tf_bridge")],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=DEFAULT_CONFIG,
                description="Path to prosthesis_config.yaml",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
