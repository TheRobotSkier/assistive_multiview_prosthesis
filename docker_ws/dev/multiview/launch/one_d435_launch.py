"""Launch a single D435 camera with pointcloud enabled after startup.

USB 2.0 note: The D435 connected over USB 2.0/2.1 cannot stream depth + color
simultaneously. Color is disabled by default; enable it only on USB 3.0+ ports.
Override with: ros2 launch ... enable_color:=true
"""
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
    enable_color_arg = DeclareLaunchArgument(
        'enable_color',
        default_value='false',
        description='Enable RGB color stream (requires USB 3.0+)',
    )

    cam = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='cam1/d435_1',
        parameters=[{
            'serial_no': LaunchConfiguration('serial_no'),
            'initial_reset': True,
            'enable_sync': False,
            'align_depth.enable': False,
            'enable_color': LaunchConfiguration('enable_color'),
            'enable_infra1': False,
            'enable_infra2': False,
            'depth_module.depth_profile': '640x480x6',
        }],
        output='screen',
    )

    enable_pc = TimerAction(period=6.0, actions=[
        ExecuteProcess(cmd=[
            'ros2', 'param', 'set', '/cam1/d435_1', 'pointcloud.enable', 'true'
        ])
    ])

    return LaunchDescription([serial_arg, enable_color_arg, cam, enable_pc])
