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
"""from launch import LaunchDescription
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

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            "/miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/ekf_imu_only.yaml"
        ],
        remappings=[
            ("imu0", "/cam0/data_raw"),
            ("odometry/filtered", "/cam0/odometry/filtered"),
        ],
    )

    return LaunchDescription([
        cam0_imu,
        ekf,
    ])"""