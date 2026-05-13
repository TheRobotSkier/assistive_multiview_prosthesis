"""Minimal single-camera launch for RealSense D435."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_arg = DeclareLaunchArgument(
        "serial_no",
        default_value="",
        description="Serial number of the D435 camera (empty = first found)",
    )
    enable_color_arg = DeclareLaunchArgument(
        "enable_color",
        default_value="false",
        description="Enable RGB color stream (requires USB 3.0+)",
    )
    depth_profile_arg = DeclareLaunchArgument(
        "depth_profile",
        default_value="640x480x6",
        description="Depth stream profile (widthxheightxfps)",
    )
    namespace_arg = DeclareLaunchArgument(
        "camera_namespace",
        default_value="camera",
        description="ROS namespace for the camera node",
    )

    cam = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace=LaunchConfiguration("camera_namespace"),
        parameters=[
            {
                "serial_no": LaunchConfiguration("serial_no"),
                "initial_reset": True,
                "enable_sync": False,
                "align_depth.enable": False,
                "enable_color": LaunchConfiguration("enable_color"),
                "enable_infra1": False,
                "enable_infra2": False,
                "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
            }
        ],
        output="screen",
    )

    # Pointcloud must be enabled after camera initializes (~6s)
    enable_pc = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "param",
                    "set",
                    "/camera/camera",
                    "pointcloud.enable",
                    "true",
                ]
            )
        ],
    )

    return LaunchDescription(
        [serial_arg, enable_color_arg, depth_profile_arg, namespace_arg, cam, enable_pc]
    )
