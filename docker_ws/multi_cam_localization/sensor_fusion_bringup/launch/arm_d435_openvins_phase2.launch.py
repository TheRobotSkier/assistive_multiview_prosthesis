from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ov_config = PathJoinSubstitution([
        FindPackageShare('sensor_fusion_bringup'), 'config', 'openvins',
        'arm_d435_829212072207', 'estimator_config.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('verbosity', default_value='WARNING'),
        Node(
            package='ov_msckf', executable='run_subscribe_msckf_marker',
            namespace='ov_msckf_arm', name='run_subscribe_msckf_marker',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'verbosity': LaunchConfiguration('verbosity'),
                'use_stereo': False, 'max_cameras': 1,
                'config_path': ov_config,
                'global_frame_id': 'marker_map',
                'imu_frame_id': 'cam1_imu_link',
                'camera_frame_prefix': 'arm_cam',
                'publish_global_to_imu_tf': True,
            }],
        ),
    ])
