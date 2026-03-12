from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, 
                                  PathJoinSubstitution, TextSubstitution)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare

from os.path import exists

def launch_fun(context, *args, **kwargs):
    xml_model_path = LaunchConfiguration('xml_model_path').perform(context)
    laterality = LaunchConfiguration('laterality').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
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

    robot_description = ParameterValue(
        value = Command([
            FindExecutable(name = 'xacro'),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('mia_hand_mujoco'), 'description', 
                'urdf', 'mia_hand_system_interface.urdf.xacro'
            ]),
            ' xml_model_path:=',
            TextSubstitution(text = xml_model_path),
            ' laterality:=',
            TextSubstitution(text = laterality),
            ' prefix:=',
            TextSubstitution(text = prefix),
            ' joint_limits_config_file:=',
            TextSubstitution(text = joint_limits_config_file)
        ]),
        value_type = str
    )

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('mia_hand_mujoco'),
        'config',
        'mia_hand_controllers.yaml'
    ])

    ros2_control_node = Node(
        package = 'controller_manager',
        executable = 'ros2_control_node',
        parameters = [ParameterFile(robot_controllers, allow_substs=True)],
        output = 'both',
        remappings = [(
            "robot_description", 
            PathJoinSubstitution([
                TextSubstitution(text = robot_ns), 'robot_description'
            ])
        )]
    )

    robot_state_publisher = Node(
        name = 'robot_state_publisher',
        package = 'robot_state_publisher',
        namespace = TextSubstitution(text = robot_ns),
        executable = 'robot_state_publisher',
        parameters = [{'robot_description': robot_description}],
        remappings = [('joint_states', '/remapped_joint_states')]
    )

    joint_state_broadcaster_spawner = Node(
        name = 'joint_state_broadcaster_spawner',
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_state_broadcaster', '-c', '/controller_manager',
                      '-n', TextSubstitution(text = robot_ns)]
    )

    position_controllers_spawner = Node(
        name = 'position_controllers_spawner',
        package = 'controller_manager',
        executable = 'spawner',
        arguments = [
            'thumb_pos_ff_controller',
            'index_pos_ff_controller',
            'mrl_pos_ff_controller',
            '-c', '/controller_manager'
        ]
    )

    position_controllers_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action = joint_state_broadcaster_spawner,
            on_exit = [position_controllers_spawner]
        )
    )

    return [
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        position_controllers_spawner_after_joint_state_broadcaster_spawner
    ]

def generate_launch_description():
    
    xml_model_path_arg = DeclareLaunchArgument(
        'xml_model_path',
        default_value = 
            PathJoinSubstitution([
              FindPackageShare('mia_hand_mujoco'),
              'mia_hand',
              'scene_right.xml'
            ]),
        description = 'Path to Mia Hand XML MuJoCo model.'
    )

    laterality_arg = DeclareLaunchArgument(
        'laterality',
        default_value = 'right',
        description = 'Parameter for loading a right or left hand.'
    )

    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value = '',
        description = 'Prefix to be added before Mia Hand link and joint names.' 
                        'Useful for multi-robot scenarios.'
    )

    robot_ns_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value = 'mia_hand'
    )

    return LaunchDescription([
        xml_model_path_arg,
        laterality_arg,
        prefix_arg,
        robot_ns_arg,
        OpaqueFunction(function = launch_fun)
    ])
