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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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

add_library(${PROJECT_NAME}_planner_gui SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/gui_simulator/planner_gui_simulator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/gui_simulator/planner_gui_system_interface.cpp
)

set(INTERACTIVE_SIM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/interactive_simulator)

add_library(${PROJECT_NAME}_interactive SHARED
  ${INTERACTIVE_SIM_DIR}/interactive_simulator.cpp
  ${INTERACTIVE_SIM_DIR}/interactive_system_interface.cpp
  ${INTERACTIVE_SIM_DIR}/simulate_src/simulate.cc
  ${INTERACTIVE_SIM_DIR}/simulate_src/glfw_adapter.cc
  ${INTERACTIVE_SIM_DIR}/simulate_src/glfw_dispatch.cc
  ${INTERACTIVE_SIM_DIR}/simulate_src/platform_ui_adapter.cc
  ${INTERACTIVE_SIM_DIR}/simulate_src/lodepng.cpp
)

# Suppress -Wmissing-field-initializers for the entire interactive target.
# simulate.h (vendored) defines mjuiDef_ arrays with partial initializers; the
# warning fires in every TU that includes simulate.h, not only simulate.cc.
target_compile_options(${PROJECT_NAME}_interactive PRIVATE
  -Wno-missing-field-initializers
)

# Suppress additional warnings that are pre-existing in the vendored MuJoCo
# simulate sources and are not actionable. Applied only to the vendored files
# so our own code (interactive_simulator.cpp, interactive_system_interface.cpp)
# retains these diagnostics.
set_source_files_properties(
  ${INTERACTIVE_SIM_DIR}/simulate_src/simulate.cc
  ${INTERACTIVE_SIM_DIR}/simulate_src/glfw_adapter.cc
  ${INTERACTIVE_SIM_DIR}/simulate_src/glfw_dispatch.cc
  ${INTERACTIVE_SIM_DIR}/simulate_src/platform_ui_adapter.cc
  ${INTERACTIVE_SIM_DIR}/simulate_src/lodepng.cpp
  PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter -Wno-sign-compare"
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  glfw3
)

ament_target_dependencies(${PROJECT_NAME}_planner_gui PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  glfw3
  geometry_msgs
  std_msgs
  std_srvs
)

ament_target_dependencies(${PROJECT_NAME}_interactive PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  glfw3
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${MUJOCO_LIBRARIES}
    miahanditactuator
    glfw
)

target_link_libraries(${PROJECT_NAME}_planner_gui PUBLIC
  ${MUJOCO_LIBRARIES}
  miahanditactuator
  glfw
)

target_link_libraries(${PROJECT_NAME}_interactive PUBLIC
  ${MUJOCO_LIBRARIES}
  miahanditactuator
  glfw
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${MUJOCO_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}_planner_gui PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/gui_simulator>"
  ${MUJOCO_INCLUDE_DIR}
)

target_include_directories(${PROJECT_NAME}_interactive PUBLIC
  "$<BUILD_INTERFACE:${INTERACTIVE_SIM_DIR}>"
  "$<BUILD_INTERFACE:${INTERACTIVE_SIM_DIR}/simulate_src>"
  ${MUJOCO_INCLUDE_DIR}
)

pluginlib_export_plugin_description_file(hardware_interface
  mia_hand_mujoco.xml)

pluginlib_export_plugin_description_file(hardware_interface
  ../dev/mujoco/gui_simulator/mia_hand_mujoco_planner_gui.xml)

pluginlib_export_plugin_description_file(hardware_interface
  ../dev/mujoco/interactive_simulator/mia_hand_mujoco_interactive.xml)

# Installing libraries.
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_planner_gui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_interactive
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
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/scenes/scene_right_static.xml
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/scenes/scene_right_dynamic.xml
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/scenes/scene_right_cylinder.xml
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/scenes/scene_right_approach.xml
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/scenes/mia_hand_right_grasp_frame.xml
  DESTINATION share/${PROJECT_NAME}/mia_hand
)

# Installing RealSense D435 mesh asset
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/assets/RealSenseD435/d435.stl
  DESTINATION share/${PROJECT_NAME}/mia_hand/assets
)

# Installing Python helper scripts
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/nodes/mujoco_scene_state_publisher_node.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/nodes/mujoco_depth_publisher_node.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/nodes/mujoco_tf_publisher_node.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../dev/mujoco/nodes/mujoco_hand_trajectory_node.py
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
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_planner_gui ${PROJECT_NAME}_interactive)
ament_package()
