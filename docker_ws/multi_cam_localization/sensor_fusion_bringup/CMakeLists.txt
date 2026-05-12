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
  DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

install(
  PROGRAMS
    scripts/analyze_marker_covariance_bags.py
    scripts/aruco_marker_pose_node.py
    scripts/calibrate_arm_marker_extrinsic.py
    scripts/dynamic_arm_pose_measurement_node.py
    scripts/head_derived_arm_pose_preview_node.py
    scripts/marker_quality_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_aruco_marker_pose_math test/test_aruco_marker_pose_math.py)
endif()

ament_package()
