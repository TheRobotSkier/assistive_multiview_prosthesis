"""Launch both D435 cameras with pointcloud enabled after startup."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess


CAM1_SERIAL = "_829212072207"
CAM2_SERIAL = "_827112072033"


def generate_launch_description():
    cam1 = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='cam1/d435_1',
        parameters=[{
            'serial_no': CAM1_SERIAL,
            'enable_sync': True,
            'align_depth.enable': True,
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
        }],
        output='screen',
    )
    cam2 = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='cam2/d435_2',
        parameters=[{
            'serial_no': CAM2_SERIAL,
            'enable_sync': True,
            'align_depth.enable': True,
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
        }],
        output='screen',
    )
    # Enable pointcloud output after cameras have started
    enable_pc1 = TimerAction(period=6.0, actions=[
        ExecuteProcess(cmd=[
            'ros2', 'param', 'set', '/cam1/d435_1', 'pointcloud.enable', 'true'
        ])
    ])
    enable_pc2 = TimerAction(period=6.0, actions=[
        ExecuteProcess(cmd=[
            'ros2', 'param', 'set', '/cam2/d435_2', 'pointcloud.enable', 'true'
        ])
    ])
    return LaunchDescription([cam1, cam2, enable_pc1, enable_pc2])
