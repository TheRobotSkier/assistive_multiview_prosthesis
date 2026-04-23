cmake_minimum_required(VERSION 3.16)
project(mindrove_get_data CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# SDK_RELEASE_DIR must be provided at configure time (-DSDK_RELEASE_DIR=/sdk)
if(NOT DEFINED SDK_RELEASE_DIR)
    message(FATAL_ERROR "SDK_RELEASE_DIR is not set. Pass -DSDK_RELEASE_DIR=<path>")
endif()

set(SDK_INC "${SDK_RELEASE_DIR}/inc")
set(SDK_LIB "${SDK_RELEASE_DIR}/lib")

# --- get_data executable ---
add_executable(get_data get_data.cpp)

target_include_directories(get_data PRIVATE "${SDK_INC}")

target_link_libraries(get_data PRIVATE
    "${SDK_LIB}/libMindrove.a"
    "${SDK_LIB}/libBoardController.so"
    "${SDK_LIB}/libDataHandler.so"
    "${SDK_LIB}/libMLModule.so"
)

# Binary finds .so files next to itself at runtime
set_target_properties(get_data PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# Install binary and shared libs side by side
install(TARGETS get_data DESTINATION bin)
file(GLOB SDK_SHARED_LIBS "${SDK_LIB}/*.so")
install(FILES ${SDK_SHARED_LIBS} DESTINATION bin)
