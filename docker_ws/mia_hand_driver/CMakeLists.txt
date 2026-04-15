cmake_minimum_required(VERSION 3.8)
project(mia_hand_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mia_hand_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libserial REQUIRED IMPORTED_TARGET libserial>=1.0.0)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/cpp_driver.cpp
  src/${PROJECT_NAME}/ros_driver.cpp
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  rclcpp_action
  std_srvs
  mia_hand_msgs
)
  
target_link_libraries(${PROJECT_NAME} PUBLIC ${libserial_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

# Setting PIC on, to allow for linking to shared libraries.
set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

add_executable(${PROJECT_NAME}_node src/mia_hand_driver_node.cpp)

target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME})

# Installing executables.
install(TARGETS
  ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Installing libraries.
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
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

# Installing launch files.
install(DIRECTORY
  launch
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action std_srvs mia_hand_msgs)
ament_package()
