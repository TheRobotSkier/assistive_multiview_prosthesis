cmake_minimum_required(VERSION 3.15)
project(mia_hand_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(glfw3 REQUIRED)

if(DEFINED ENV{MUJOCO_DIR})

  find_library(MUJOCO_LIBRARIES mujoco HINTS $ENV{MUJOCO_DIR}/lib)

  if (NOT MUJOCO_LIBRARIES)
    message(FATAL_ERROR "No library found in $ENV{MUJOCO_DIR}.")
  endif()

  set(MUJOCO_INCLUDE_DIR $ENV{MUJOCO_DIR}/include)

else()

  message(FATAL_ERROR "MUJOCO_DIR environment variable not defined.")

endif()

set(PLUGIN_DIR mia_hand/plugin)

set(PLUGIN_SRCS
  ${PLUGIN_DIR}/src/index_thumb_actuator.cc
  ${PLUGIN_DIR}/src/register.cc
)

set(PLUGIN_COMPILE_OPTIONS
  ${AVX_COMPILE_OPTIONS}
  ${MUJOCO_MACOS_COMPILE_OPTIONS}
  ${EXTRA_COMPILE_OPTIONS}
  ${MUJOCO_CXX_FLAGS}
)

set(PLUGIN_LINK_OPTIONS
  ${MACOS_LINK_OPTIONS}
  ${EXTRA_LINK_OPTIONS}
)

add_library(miahanditactuator SHARED)

target_sources(miahanditactuator PRIVATE ${PLUGIN_SRCS})
target_include_directories(miahanditactuator PUBLIC ${PLUGIN_DIR}/include ${MUJOCO_INCLUDE_DIR})
target_link_libraries(miahanditactuator PRIVATE ${MUJOCO_LIBRARIES})
target_compile_options(miahanditactuator PRIVATE ${PLUGIN_COMPILE_OPTIONS})
target_link_options(miahanditactuator PRIVATE ${PLUGIN_LINK_OPTIONS})

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/simulator.cpp
  src/${PROJECT_NAME}/system_interface.cpp
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  glfw3
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${MUJOCO_LIBRARIES}
    miahanditactuator
    glfw
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${MUJOCO_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(hardware_interface
  mia_hand_mujoco.xml)

# Installing libraries.
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Installing header files.
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

# Installing config, description and launch files.
install(DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

# Installing MuJoCo model files
install(DIRECTORY mia_hand
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.xml" PATTERN "*.obj"
  PATTERN "src" EXCLUDE
  PATTERN "include" EXCLUDE
)

# Installing custom MuJoCo scene files from dev directory
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/scenes/scene_right_custom.xml
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/scenes/mia_hand_right_grasp_frame.xml
  DESTINATION share/${PROJECT_NAME}/mia_hand
)

# Installing Python helper scripts
install(PROGRAMS
  scripts/mujoco_depth_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installing MuJoCo plugins
install(TARGETS miahanditactuator
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
