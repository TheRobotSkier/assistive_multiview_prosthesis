import os

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
from moveit_configs_utils import MoveItConfigsBuilder

from os.path import exists

def launch_fun(context, *args, **kwargs):
    serial_port = LaunchConfiguration('serial_port').perform(context)
    rviz2_gui = LaunchConfiguration('rviz2_gui')
    laterality = LaunchConfiguration('laterality').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    robot_ns = LaunchConfiguration('robot_ns').perform(context)
    use_mock_hardware = LaunchConfiguration('use_mock_hardware').perform(context)
    use_mujoco_sim = LaunchConfiguration('use_mujoco_sim').perform(context)
    
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
    
    robot_description_path = None
    launch_arguments = {}

    launch_arguments = {
        "serial_port": serial_port,
        "laterality": laterality,
        "prefix": prefix,
        "robot_ns": robot_ns,
        "use_mock_hardware": use_mock_hardware,
        "joint_limits_config_file": joint_limits_config_file,
    }

    if(use_mujoco_sim == 'true'):
        robot_description_path = "config/mia_hand_mujoco.urdf.xacro"
        launch_arguments = {
            "laterality": laterality,
            "prefix": prefix,
            "joint_limits_config_file": joint_limits_config_file,
        }
    else:
        robot_description_path = "config/mia_hand_robot.urdf.xacro"
        launch_arguments = {
            "serial_port": serial_port,
            "laterality": laterality,
            "prefix": prefix,
            "robot_ns": robot_ns,
            "use_mock_hardware": use_mock_hardware,
            "joint_limits_config_file": joint_limits_config_file,
        }
    
    moveit_config = (
        MoveItConfigsBuilder(robot_name="mia_hand_robot", package_name="mia_hand_moveit_config")
        .robot_description(file_path=robot_description_path, mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace = TextSubstitution(text = robot_ns),
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
        remappings = [('joint_states', '/remapped_joint_states')]
    )

    if (use_mujoco_sim == 'true'):
      ros2_controllers_path = PathJoinSubstitution([
          FindPackageShare('mia_hand_mujoco'),
          'config',
          'mia_hand_controllers.yaml'
      ])
    else:
      ros2_controllers_path = PathJoinSubstitution([
          FindPackageShare('mia_hand_ros2_control'),
          'config',
          'mia_hand_controllers.yaml'
      ])

    ros2_control_node = Node(
        package = 'controller_manager',
        executable = 'ros2_control_node',
        parameters = [ParameterFile(ros2_controllers_path, allow_substs=True)],
        output = 'both',
        remappings = [(
            "robot_description", 
            PathJoinSubstitution([
                TextSubstitution(text = robot_ns), 'robot_description'
            ])
        )]
    )

    rviz2_config_file = PathJoinSubstitution([
        FindPackageShare('mia_hand_moveit_config'), 'rviz', 'mia_hand_moveit.rviz'
    ])

    rviz2 = Node(
        condition = IfCondition(rviz2_gui),
        name = 'rviz2',
        package = 'rviz2',
        executable = 'rviz2',
        arguments = ['-d', rviz2_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments = ['joint_state_broadcaster', '-c', '/controller_manager',
                      '-n', TextSubstitution(text = robot_ns)]
    )

    thumb_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["thumb_trajectory_controller", "-c", "/controller_manager"],
    )

    index_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["index_trajectory_controller", "-c", "/controller_manager"],
    )

    mrl_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mrl_trajectory_controller", "-c", "/controller_manager"],
    )

    rviz2_joint_state_publisher = Node(
        condition = IfCondition(rviz2_gui),
        name = 'rviz2_joint_state_publisher',
        package = 'mia_hand_description',
        executable = 'rviz2_joint_state_publisher_node',
        parameters=[
            moveit_config.robot_description,
            PathJoinSubstitution([
               FindPackageShare('mia_hand_description'),
              'calibration',
              TextSubstitution(text = transmissions_config_file)
            ]),
            {'prefix': prefix}
        ],
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
        move_group_node,
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        thumb_trajectory_controller,
        index_trajectory_controller, 
        mrl_trajectory_controller,
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

    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value = 'false',
        description="Start robot with mock hardware mirroring command to its states.",
    )

    use_mujoco_sim_arg = DeclareLaunchArgument(
        'use_mujoco_sim',
        default_value = 'false',
        description="Start Moveit with MuJoCo simulator",
    )

    return LaunchDescription([
        serial_port_arg,
        rviz2_gui_arg,
        laterality_arg,
        prefix_arg,
        robot_ns_arg,
        use_mock_hardware_arg,
        use_mujoco_sim_arg,
        OpaqueFunction(function = launch_fun)
    ])
