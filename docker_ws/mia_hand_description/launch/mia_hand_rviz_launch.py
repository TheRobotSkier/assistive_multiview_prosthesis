from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, 
                                  PathJoinSubstitution, TextSubstitution)

from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from os.path import exists

def launch_fun(context, *args, **kwargs):
    use_joints_gui = LaunchConfiguration('use_joints_gui')
    laterality = LaunchConfiguration('laterality').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    load_ur_flange = LaunchConfiguration('load_ur_flange').perform(context)
    robot_ns = LaunchConfiguration('robot_ns').perform(context)

    joint_limits_config_file_path = PathJoinSubstitution([
        FindPackageShare('mia_hand_description'), 'calibration',
        'joint_limits.yaml']).perform(context)

    transmissions_config_file_path = PathJoinSubstitution([
        FindPackageShare('mia_hand_description'), 'calibration',
        'transmission_config.yaml']).perform(context)

    if exists(joint_limits_config_file_path):
        joint_limits_config_file = 'joint_limits.yaml'
    else:
        joint_limits_config_file = 'joint_limits_default.yaml'

    if exists(transmissions_config_file_path):
        transmissions_config_file = 'transmission_config.yaml'
    else:
        transmissions_config_file = 'transmission_config_default.yaml'

    if 'true' == load_ur_flange:
        robot_description = ParameterValue(
            value = Command([
                FindExecutable(name = 'xacro'),
                ' ',
                PathJoinSubstitution([
                  FindPackageShare('mia_hand_description'), 'urdf', 
                  'mia_hand_on_ur_flange.urdf.xacro'
                ]),
                ' prefix:=', 
                TextSubstitution(text = prefix),
                ' joint_limits_config_file:=', 
                TextSubstitution(text = joint_limits_config_file)
            ]),
            value_type = str
        )
    else:
        robot_description = ParameterValue(
            value = Command([
                FindExecutable(name = 'xacro'),
                ' ',
                PathJoinSubstitution([
                  FindPackageShare('mia_hand_description'), 'urdf', 
                  'mia_hand_description.urdf.xacro'
                ]),
                ' laterality:=', 
                TextSubstitution(text = laterality),
                ' prefix:=', 
                TextSubstitution(text = prefix),
                ' joint_limits_config_file:=', 
                TextSubstitution(text = joint_limits_config_file)
            ]),
            value_type = str
        )

    actions = []

    actions.append(
        Node(
            name = 'robot_state_publisher',
            package = 'robot_state_publisher',
            namespace = TextSubstitution(text = robot_ns),
            executable = 'robot_state_publisher',
            parameters = [{'robot_description': robot_description}],
            remappings = [('joint_states', 'remapped_joint_states')]
        )
    )

    actions.append(
        Node(
            condition = IfCondition(use_joints_gui),
            name = 'joint_state_publisher_gui',
            package = 'joint_state_publisher_gui',
            namespace = TextSubstitution(text = robot_ns),
            executable = 'joint_state_publisher_gui',
        )
    )

    actions.append(
        Node(
            name = 'rviz2_joint_state_publisher',
            package = 'mia_hand_description',
            namespace = TextSubstitution(text = robot_ns),
            executable = 'rviz2_joint_state_publisher_node',
            parameters = [
                {'robot_description': robot_description},
                PathJoinSubstitution([
                    FindPackageShare('mia_hand_description'),
                    'calibration',
                    TextSubstitution(text = transmissions_config_file)
                ]),
                {'prefix': prefix}
            ]
        )
    )

    rviz2_config_file = PathJoinSubstitution([
        FindPackageShare('mia_hand_description'), 'rviz', 'mia_hand_config.rviz'
    ])

    actions.append(
        Node(
            name = 'rviz2',
            package = 'rviz2',
            executable = 'rviz2',
            arguments = ['-d', rviz2_config_file]
        )
    )

    return actions

def generate_launch_description():

    use_joints_gui_arg = DeclareLaunchArgument(
        'use_joints_gui',
        default_value = 'true',
        description = 'True if joint states publisher should use a GUI for setting joint states.'
    )

    laterality_arg = DeclareLaunchArgument(
        'laterality',
        default_value = 'right',
        description = 'Parameter for loading a right or left Mia Hand.'
    )

    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value = '',
        description = 'Prefix to be added before Mia Hand link and joint names. Useful for multi-robot scenarios.'
    )

    load_ur_flange_arg = DeclareLaunchArgument(
        'load_ur_flange',
        default_value = 'false',
        description = 'True if Mia Hand should be loaded together with the UR flange.'
    )

    robot_ns_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value = 'mia_hand'
    )

    return LaunchDescription([
        use_joints_gui_arg,
        laterality_arg,
        prefix_arg,
        load_ur_flange_arg,
        robot_ns_arg,
        OpaqueFunction(function = launch_fun)
    ])
