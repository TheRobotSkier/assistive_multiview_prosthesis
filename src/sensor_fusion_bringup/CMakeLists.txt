cmake_minimum_required(VERSION 3.8)
project(sensor_fusion_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  PROGRAMS
    scripts/analyze_marker_covariance_bags.py
    scripts/aruco_marker_pose_node.py
    scripts/i2c_mpu9250_imu_node.py
    scripts/imu_dead_reckoning_node.py
    scripts/marker_pose_odometry_bridge.py
    scripts/marker_quality_monitor.py
    scripts/future_pose_prediction_collision_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  install(
    DIRECTORY test/
    DESTINATION share/${PROJECT_NAME}/test
  )
endif()

ament_package()
