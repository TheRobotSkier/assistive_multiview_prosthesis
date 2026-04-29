cmake_minimum_required(VERSION 3.15)
project(grasp_preshaping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# ── Rust cdylib ────────────────────────────────────────────────────────────
# cargo is optional: the .so is normally pre-built by the rust_build Docker
# service and available at target/release/ via the workspace bind mount.
# If cargo IS present (e.g. local dev), build it here as usual.
# If neither cargo nor a pre-built .so is found the bridge node will halt at
# startup with RCLCPP_FATAL — this is intentional.
find_program(CARGO_EXECUTABLE cargo)

if(CARGO_EXECUTABLE)
  set(RUST_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/cargo-target)
  set(RUST_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/rust)
  set(RUST_LIB_PATH ${RUST_OUT_DIR}/libgrasp_preshaping.so)

  file(GLOB_RECURSE RUST_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.rs
  )

  add_custom_command(
    OUTPUT ${RUST_LIB_PATH}
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${RUST_TARGET_DIR}
            ${CARGO_EXECUTABLE} build --release --lib
            --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
    COMMAND ${CMAKE_COMMAND} -E make_directory ${RUST_OUT_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${RUST_TARGET_DIR}/release/libgrasp_preshaping.so
            ${RUST_LIB_PATH}
    DEPENDS
      ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
      ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.lock
      ${RUST_SOURCES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building Rust preshaping cdylib"
    VERBATIM
  )

  add_custom_target(grasp_preshaping_rustlib ALL
    DEPENDS ${RUST_LIB_PATH}
  )

  install(FILES ${RUST_LIB_PATH}
    DESTINATION lib
  )
else()
  set(PREBUILT_SO ${CMAKE_CURRENT_SOURCE_DIR}/target/release/libgrasp_preshaping.so)
  if(EXISTS ${PREBUILT_SO})
    message(STATUS "grasp_preshaping: installing pre-built .so from ${PREBUILT_SO}")
    install(FILES ${PREBUILT_SO} DESTINATION lib)
  else()
    message(STATUS "grasp_preshaping: cargo not found and no pre-built .so at ${PREBUILT_SO}. "
      "Run the rust_build service first. The bridge node will halt at startup if "
      "libgrasp_preshaping.so is not present at runtime.")
  endif()
endif()

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
