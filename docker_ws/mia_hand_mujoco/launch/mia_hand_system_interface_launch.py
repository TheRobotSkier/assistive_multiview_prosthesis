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
    scene          = LaunchConfiguration('scene').perform(context)
    laterality     = LaunchConfiguration('laterality').perform(context)
    prefix         = LaunchConfiguration('prefix').perform(context)
    robot_ns       = LaunchConfiguration('robot_ns').perform(context)
    enable_depth_publisher = LaunchConfiguration('enable_depth_publisher')
    depth_camera_name = LaunchConfiguration('depth_camera_name')
    depth_frame_id = LaunchConfiguration('depth_frame_id')
    depth_world_frame_id = LaunchConfiguration('depth_world_frame_id')
    depth_width = LaunchConfiguration('depth_width')
    depth_height = LaunchConfiguration('depth_height')
    depth_publish_hz = LaunchConfiguration('depth_publish_hz')
    depth_max_depth = LaunchConfiguration('depth_max_depth')
    depth_pointcloud_stride = LaunchConfiguration('depth_pointcloud_stride')
    depth_joint_state_topic = LaunchConfiguration('depth_joint_state_topic')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    depth_camera_info_topic = LaunchConfiguration('depth_camera_info_topic')
    depth_camera_pointcloud_topic = LaunchConfiguration('depth_camera_pointcloud_topic')
    depth_segmented_pointcloud_topic = LaunchConfiguration('depth_segmented_pointcloud_topic')
    depth_target_geom_name = LaunchConfiguration('depth_target_geom_name')
    depth_output_dir = LaunchConfiguration('depth_output_dir')

    # Resolve xml_model_path from scene arg when no explicit path was given
    if not xml_model_path:
        scene_file_map = {
            'default': 'scene_right.xml',
            'custom':  'scene_right_custom.xml',
        }
        scene_filename = scene_file_map.get(scene, 'scene_right.xml')
        xml_model_path = PathJoinSubstitution([
            FindPackageShare('mia_hand_mujoco'),
            'mia_hand',
            scene_filename
        ]).perform(context)

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

    depth_publisher_node = Node(
        package = 'mia_hand_mujoco',
        executable = 'mujoco_depth_publisher_node.py',
        name = 'mujoco_depth_publisher',
        output = 'screen',
        parameters = [{
            'xml_model_path': xml_model_path,
            'output_dir': depth_output_dir,
            'camera_name': depth_camera_name,
            'camera_frame_id': depth_frame_id,
            'world_frame_id': depth_world_frame_id,
            'width': depth_width,
            'height': depth_height,
            'publish_hz': depth_publish_hz,
            'max_depth': depth_max_depth,
            'pointcloud_stride': depth_pointcloud_stride,
            'joint_state_topic': depth_joint_state_topic,
            'depth_image_topic': depth_image_topic,
            'depth_camera_info_topic': depth_camera_info_topic,
            'depth_camera_pointcloud_topic': depth_camera_pointcloud_topic,
            'segmented_pointcloud_topic': depth_segmented_pointcloud_topic,
            'target_geom_name': depth_target_geom_name,
            'laterality': laterality,
            'prefix': prefix,
        }],
        condition = IfCondition(enable_depth_publisher),
    )

    return [
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        position_controllers_spawner_after_joint_state_broadcaster_spawner,
        depth_publisher_node,
    ]

def generate_launch_description():

    scene_arg = DeclareLaunchArgument(
        'scene',
        default_value='custom',
        choices=['default', 'custom'],
        description='MuJoCo scene to load. "default" uses scene_right.xml, '
                    '"custom" uses scene_right_custom.xml with a red origin marker.'
    )

    xml_model_path_arg = DeclareLaunchArgument(
        'xml_model_path',
        default_value='',
        description='Absolute path to a MuJoCo XML scene file. '
                    'Overrides the "scene" argument when non-empty.'
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

    enable_depth_publisher_arg = DeclareLaunchArgument(
        'enable_depth_publisher',
        default_value='false',
        description='Start the simulated depth publisher that renders the custom MuJoCo camera and publishes an object-only world-frame point cloud.'
    )

    depth_camera_name_arg = DeclareLaunchArgument(
        'depth_camera_name',
        default_value='front_depth_cam'
    )

    depth_frame_id_arg = DeclareLaunchArgument(
        'depth_frame_id',
        default_value='mujoco_front_depth_cam'
    )

    depth_world_frame_id_arg = DeclareLaunchArgument(
        'depth_world_frame_id',
        default_value='world'
    )

    depth_width_arg = DeclareLaunchArgument(
        'depth_width',
        default_value='640'
    )

    depth_height_arg = DeclareLaunchArgument(
        'depth_height',
        default_value='480'
    )

    depth_publish_hz_arg = DeclareLaunchArgument(
        'depth_publish_hz',
        default_value='5.0'
    )

    depth_max_depth_arg = DeclareLaunchArgument(
        'depth_max_depth',
        default_value='1.5'
    )

    depth_pointcloud_stride_arg = DeclareLaunchArgument(
        'depth_pointcloud_stride',
        default_value='2'
    )

    depth_joint_state_topic_arg = DeclareLaunchArgument(
        'depth_joint_state_topic',
        default_value='/joint_states'
    )

    depth_image_topic_arg = DeclareLaunchArgument(
        'depth_image_topic',
        default_value='/mujoco/depth/image'
    )

    depth_camera_info_topic_arg = DeclareLaunchArgument(
        'depth_camera_info_topic',
        default_value='/mujoco/depth/camera_info'
    )

    depth_camera_pointcloud_topic_arg = DeclareLaunchArgument(
        'depth_camera_pointcloud_topic',
        default_value='/mujoco/depth/points_camera'
    )

    depth_segmented_pointcloud_topic_arg = DeclareLaunchArgument(
        'depth_segmented_pointcloud_topic',
        default_value='/segmented_object_cloud'
    )

    depth_target_geom_name_arg = DeclareLaunchArgument(
        'depth_target_geom_name',
        default_value='target_sphere'
    )

    depth_output_dir_arg = DeclareLaunchArgument(
        'depth_output_dir',
        default_value='/tmp/mia_hand_mujoco_depth'
    )

    return LaunchDescription([
        scene_arg,
        xml_model_path_arg,
        laterality_arg,
        prefix_arg,
        robot_ns_arg,
        enable_depth_publisher_arg,
        depth_camera_name_arg,
        depth_frame_id_arg,
        depth_world_frame_id_arg,
        depth_width_arg,
        depth_height_arg,
        depth_publish_hz_arg,
        depth_max_depth_arg,
        depth_pointcloud_stride_arg,
        depth_joint_state_topic_arg,
        depth_image_topic_arg,
        depth_camera_info_topic_arg,
        depth_camera_pointcloud_topic_arg,
        depth_segmented_pointcloud_topic_arg,
        depth_target_geom_name_arg,
        depth_output_dir_arg,
        OpaqueFunction(function = launch_fun)
    ])
