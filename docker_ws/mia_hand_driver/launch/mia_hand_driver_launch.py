from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value = '/dev/ttyUSB0',
        description = 'Mia Hand serial port device.'
    )

    mia_hand_driver_node = Node(
        name = 'driver',
        package = 'mia_hand_driver',
        namespace = 'mia_hand',
        executable = 'mia_hand_driver_node',
        parameters = [{'serial_port': LaunchConfiguration('serial_port')}]
    )

    return LaunchDescription([
        serial_port_arg,
        mia_hand_driver_node
    ])
