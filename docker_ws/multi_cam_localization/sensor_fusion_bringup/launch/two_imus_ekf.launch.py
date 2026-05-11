"""Launch dual-IMU EKF node using robot_localization.

Maps two Realsense IMU streams into a single ekf_node instance:
  - /cam0/data_raw -> imu0 (head camera, bus 7, cam0_imu_link)
  - /cam1/data_raw -> imu1 (arm camera,  bus 1, cam1_imu_link)

Config: config/ekf_dual_imu.yaml (calibration values from P3.1)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_dual_imu = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            [FindPackageShare("sensor_fusion_bringup"), "/config/ekf_dual_imu.yaml"]
        ],
    )

    return LaunchDescription([ekf_dual_imu])
