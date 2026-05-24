cmake_minimum_required(VERSION 3.8)
project(sensor_fusion_bringup)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_fusion_msgs REQUIRED)

add_executable(pointcloud_to_frame_node src/pointcloud_to_frame_node.cpp)
ament_target_dependencies(
  pointcloud_to_frame_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

add_executable(aruco_marker_pose_cpp_node src/aruco_marker_pose_node.cpp)
ament_target_dependencies(
  aruco_marker_pose_cpp_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  sensor_fusion_msgs
  OpenCV
)

install(
  TARGETS pointcloud_to_frame_node aruco_marker_pose_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

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
  ament_add_pytest_test(test_openvins_profiles test/test_openvins_profiles.py)
endif()

ament_package()
