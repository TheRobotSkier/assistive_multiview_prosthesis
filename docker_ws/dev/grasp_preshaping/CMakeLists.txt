cmake_minimum_required(VERSION 3.15)
project(grasp_preshaping)

find_package(ament_cmake REQUIRED)

find_program(CARGO_EXECUTABLE cargo REQUIRED)

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

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
