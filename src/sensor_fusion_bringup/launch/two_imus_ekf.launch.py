from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cam0_imu = Node(
        package="imu_driver",
        executable="imu_node",
        name="imu_node",
        namespace="cam0",
        output="screen",
        parameters=[
            "/miahand_ws/src/multi_cam_localization/imu_driver/config/imu_cam0.yaml"
        ],
    )

    cam1_imu = Node(
        package="imu_driver",
        executable="imu_node",
        name="imu_node",
        namespace="cam1",
        output="screen",
        parameters=[
            "/miahand_ws/src/multi_cam_localization/imu_driver/config/imu_cam1.yaml"
        ],
    )

    return LaunchDescription([
        cam0_imu,
        cam1_imu,
    ])