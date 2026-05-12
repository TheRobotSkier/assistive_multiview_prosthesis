from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ov_config = PathJoinSubstitution([
        FindPackageShare('sensor_fusion_bringup'), 'config', 'openvins',
        'head_d435_827112072033', 'estimator_config.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('verbosity', default_value='WARNING'),
        Node(
            package='ov_msckf', executable='run_subscribe_msckf_marker',
            namespace='ov_msckf', name='run_subscribe_msckf_marker',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'verbosity': LaunchConfiguration('verbosity'),
                'use_stereo': False, 'max_cameras': 1,
                'config_path': ov_config,
                'global_frame_id': 'marker_map',
                'imu_frame_id': 'cam0_imu_link',
                'camera_frame_prefix': 'head_cam',
                'publish_global_to_imu_tf': True,
            }],
        ),
    ])
