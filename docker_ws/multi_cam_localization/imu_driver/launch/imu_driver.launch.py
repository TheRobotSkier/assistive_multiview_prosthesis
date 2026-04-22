from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="cam0",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="/miahand_ws/src/multi_cam_localization/imu_driver/config/imu_cam0.yaml",
    )

    return LaunchDescription([
        namespace_arg,
        params_file_arg,
        Node(
            package="imu_driver",
            executable="imu_node",
            name="imu_node",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])