"""Launch a single D435 camera with pointcloud enabled after startup."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


CAM1_SERIAL = "_829212072207"


def generate_launch_description():
    serial_arg = DeclareLaunchArgument(
        'serial_no',
        default_value=CAM1_SERIAL,
        description='Serial number of the D435 camera to launch',
    )

    cam = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='cam1/d435_1',
        parameters=[{
            'serial_no': LaunchConfiguration('serial_no'),
            'enable_sync': True,
            'align_depth.enable': True,
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
        }],
        output='screen',
    )

    enable_pc = TimerAction(period=6.0, actions=[
        ExecuteProcess(cmd=[
            'ros2', 'param', 'set', '/cam1/d435_1', 'pointcloud.enable', 'true'
        ])
    ])

    return LaunchDescription([serial_arg, cam, enable_pc])
