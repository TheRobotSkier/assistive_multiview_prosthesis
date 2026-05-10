from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    prefix = LaunchConfiguration('prefix')

    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value = '',
        description = 'Prefix to be added before Mia Hand link and joint names.' 
                        'Useful for multi-robot scenarios.'
    )
    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("mia_hand_ros2_control"),
            "config",
            "test_joint_trajectory_controller.yaml",
        ]
    )

    return LaunchDescription(
        [   prefix_arg,
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                parameters=[ParameterFile(position_goals, allow_substs=True)],
                output="both",
            )
        ]
    )