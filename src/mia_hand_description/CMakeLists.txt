cmake_minimum_required(VERSION 3.8)
project(mia_hand_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(joint_limits REQUIRED)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/thumb_joint_opposition.cpp
  src/${PROJECT_NAME}/rviz2_joint_state_publisher.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  urdf
  joint_limits
  sensor_msgs
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

add_executable(rviz2_joint_state_publisher_node 
  src/rviz2_joint_state_publisher_node.cpp
)

target_link_libraries(rviz2_joint_state_publisher_node ${PROJECT_NAME})

# Installing executables.
install(TARGETS
  rviz2_joint_state_publisher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Installing libraries.
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Installing header files.
install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

# Installing launch, URDF and config files.
install(DIRECTORY
  launch
  urdf
  materials
  meshes
  calibration
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp urdf joint_limits sensor_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
