cmake_minimum_required(VERSION 3.15)
project(grasp_preshaping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# ── Pre-built Rust cdylib ──────────────────────────────────────────────────
# The .so is pre-compiled locally (cargo build --release --lib) and committed
# to lib/.  No Rust toolchain is needed in the Docker image.
set(PREBUILT_SO ${CMAKE_CURRENT_SOURCE_DIR}/lib/libgrasp_preshaping.so)

if(EXISTS ${PREBUILT_SO})
  message(STATUS "grasp_preshaping: installing pre-built .so from ${PREBUILT_SO}")
  install(FILES ${PREBUILT_SO} DESTINATION lib)
else()
  message(WARNING "grasp_preshaping: no pre-built .so at ${PREBUILT_SO}. "
    "Build locally with 'cargo build --release --lib' and copy to lib/.")
endif()

# Install data directory (finger_contact_lut.npz etc.)
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

# ── C++ preshaping service bridge node ─────────────────────────────────────
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(preshaping_service_bridge_node
  nodes/preshaping_service_bridge_node.cpp
)

target_include_directories(preshaping_service_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(preshaping_service_bridge_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

target_link_libraries(preshaping_service_bridge_node
  dl
)

install(TARGETS preshaping_service_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/grasp_proximity_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
