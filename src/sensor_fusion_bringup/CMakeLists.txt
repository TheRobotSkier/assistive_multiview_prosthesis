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
