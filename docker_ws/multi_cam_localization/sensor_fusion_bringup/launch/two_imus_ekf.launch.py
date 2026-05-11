"""Launch both ICM-20948 IMUs and the robot_localization EKF filter.

Produces /odometry/filtered by fusing /cam0/data_raw and /cam1/data_raw.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ---- IMU drivers ----

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

    # ---- EKF filter (robot_localization) ----

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            "/miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/ekf_dual_imu.yaml"
        ],
    )

    return LaunchDescription([
        cam0_imu,
        cam1_imu,
        ekf_node,
    ])
