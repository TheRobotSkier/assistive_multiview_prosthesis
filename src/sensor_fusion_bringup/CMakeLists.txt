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

install(
  TARGETS pointcloud_to_frame_node
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

# Install scripts as executables. NOTE: jetson_relay.py is intentionally
# excluded — it is obsolete on a single machine (see merge plan §5.5).
# Host-only scripts (publish_camera_mounts.py, tf_pipeline_diagnostics.py)
# are preserved here alongside the Jetson-derived scripts.
install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

# Install the ROS-node scripts that are launched as executable nodes.
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
