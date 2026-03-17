cmake_minimum_required(VERSION 3.15)
project(mia_hand_mujoco_plugin)

set(PLUGIN_SRCS
  src/index_thumb_actuator.cc
  src/register.cc
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

if(DEFINED ENV{MUJOCO_DIR})

  set(MUJOCO_LIB_DIR $ENV{MUJOCO_DIR}/lib)

  find_library(MUJOCO_LIBRARIES mujoco HINTS ${MUJOCO_LIB_DIR})

  if(NOT MUJOCO_LIBRARIES)
    message(FATAL_ERROR "No library found in $ENV{MUJOCO_DIR}.")
  endif()

  set(MUJOCO_INCLUDE_DIR $ENV{MUJOCO_DIR}/include)
  set(MUJOCO_BIN_DIR $ENV{MUJOCO_DIR}/bin)
  set(MUJOCO_PLUGIN_DIR ${MUJOCO_BIN_DIR}/mujoco_plugin)

else()

  message(FATAL_ERROR "MUJOCO_DIR environment variable not defined.")

endif()

add_library(miahanditactuator SHARED)

target_sources(miahanditactuator PRIVATE ${PLUGIN_SRCS})
target_include_directories(miahanditactuator PUBLIC include ${MUJOCO_INCLUDE_DIR})
target_link_libraries(miahanditactuator PRIVATE ${MUJOCO_LIBRARIES})
target_compile_options(miahanditactuator PRIVATE ${PLUGIN_COMPILE_OPTIONS})
target_link_options(miahanditactuator PRIVATE ${PLUGIN_LINK_OPTIONS})

# Installing library.
install(
  TARGETS miahanditactuator
  ARCHIVE DESTINATION ${MUJOCO_LIB_DIR}
  LIBRARY DESTINATION ${MUJOCO_PLUGIN_DIR}
  RUNTIME DESTINATION ${MUJOCO_BIN_DIR}
)

# Installing header files.
install(
  DIRECTORY include/
  DESTINATION ${MUJOCO_INCLUDE_DIR}
  FILES_MATCHING PATTERN ".h"
)
