"""Launch robot_localization EKF for dual ICM-20948 IMU fusion.

Produces /odometry/filtered by fusing /cam0/data_raw and /cam1/data_raw.
IMU drivers are launched separately by robotlab_bringup.launch.py.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            "/miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/ekf_dual_imu.yaml"
        ],
    )

    return LaunchDescription([ekf_node])
