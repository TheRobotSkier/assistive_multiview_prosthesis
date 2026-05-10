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
    serial_port = LaunchConfiguration('serial_port').perform(context)
    rviz2_gui = LaunchConfiguration('rviz2_gui')
    laterality = LaunchConfiguration('laterality').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    robot_ns = LaunchConfiguration('robot_ns').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    use_mock_hardware = LaunchConfiguration('use_mock_hardware').perform(context)

    joint_limits_config_file_path = PathJoinSubstitution([
        FindPackageShare('mia_hand_description'), 'calibration',
        'joint_limits.yaml']).perform(context)
    
    transmissions_config_file_path = PathJoinSubstitution([
        FindPackageShare('mia_hand_description'), 'calibration',
        'transmission_config.yaml']).perform(context)
    
    if exists(transmissions_config_file_path):
        transmissions_config_file = 'transmission_config.yaml'
    else:
        transmissions_config_file = 'transmission_config_default.yaml'


    if exists(joint_limits_config_file_path):
        joint_limits_config_file = 'joint_limits.yaml'
    else:
        joint_limits_config_file = 'joint_limits_default.yaml'

    robot_description = ParameterValue(
        value = Command([
            FindExecutable(name = 'xacro'),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('mia_hand_ros2_control'), 'description', 
                'urdf', 'mia_hand_system_interface.urdf.xacro'
            ]),
            ' serial_port:=',
            TextSubstitution(text = serial_port),
            ' laterality:=',
            TextSubstitution(text = laterality),
            ' prefix:=',
            TextSubstitution(text = prefix),
            ' joint_limits_config_file:=',
            TextSubstitution(text = joint_limits_config_file),
            ' use_mock_hardware:=',
            TextSubstitution(text = use_mock_hardware)
        ]),
        value_type = str
    )

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('mia_hand_ros2_control'),
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

    rviz2_config_file = PathJoinSubstitution([
        FindPackageShare('mia_hand_description'), 'rviz', 'mia_hand_config.rviz'
    ])

    rviz2 = Node(
        condition = IfCondition(rviz2_gui),
        name = 'rviz2',
        package = 'rviz2',
        executable = 'rviz2',
        arguments = ['-d', rviz2_config_file]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments = ['joint_state_broadcaster', '-c', '/controller_manager',
                      '-n', TextSubstitution(text = robot_ns)]
    )

    rviz2_joint_state_publisher = Node(
        condition = IfCondition(rviz2_gui),
        name = 'rviz2_joint_state_publisher',
        package = 'mia_hand_description',
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

    controller_spawner = Node(
        name = 'controller_spawner',
        package = 'controller_manager',
        executable = 'spawner',
        arguments = [
            controller,
            '-c', '/controller_manager',
            '-p', robot_controllers
        ]
    )

    controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action = joint_state_broadcaster_spawner,
            on_exit = [controller_spawner]
        )
    )
    
    rviz2_joint_state_publisher_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action = joint_state_broadcaster_spawner,
            on_exit = [rviz2_joint_state_publisher]
        )
    )

    rviz2_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action = joint_state_broadcaster_spawner,
            on_exit = [rviz2]
        )
    )

    return [
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        controller_spawner_after_joint_state_broadcaster_spawner,
        rviz2_joint_state_publisher_after_joint_state_broadcaster_spawner,
        rviz2_after_joint_state_broadcaster_spawner
    ]

def generate_launch_description():
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value = '/dev/ttyUSB0',
        description = 'Serial port to which Mia Hand is connected.'
    )

    rviz2_gui_arg = DeclareLaunchArgument(
        'rviz2_gui',
        default_value = 'true',
        description = 'True if Rviz2 should be started automatically.'
    )

    laterality_arg = DeclareLaunchArgument(
        'laterality',
        default_value = 'right',
        description = 'Parameter for loading a right or left hand in RViz2.'
                        'Ignored if rviz2_gui:=false.'
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

    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value = 'group_pos_vel_controller'
    )

    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value = 'false',
        description="Start robot with mock hardware mirroring command to its states.",
    )

    return LaunchDescription([
        serial_port_arg,
        rviz2_gui_arg,
        laterality_arg,
        prefix_arg,
        robot_ns_arg,
        controller_arg,
        use_mock_hardware_arg,
        OpaqueFunction(function = launch_fun)
    ])
