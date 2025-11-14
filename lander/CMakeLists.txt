set(CMAKE_BUILD_TYPE "Release")
cmake_minimum_required(VERSION 2.8.12)
project(lander)

# Find required libraries
find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

# Paths
set(MAVLINK_DIR "${CMAKE_CURRENT_LIST_DIR}/third_party/mavlink")
message(STATUS "MAVLINK_DIR = ${MAVLINK_DIR}")

# Sanity check: require mavlink/common/mavlink.h to exist
if(NOT EXISTS "${MAVLINK_DIR}/common/mavlink.h")
    message(FATAL_ERROR "Expected header not found: ${MAVLINK_DIR}/common/mavlink.h")
endif()

# Add the executable
add_executable(lander lander.cpp)

# Include dirs
target_include_directories(lander PRIVATE
    "${MAVLINK_DIR}"
    /usr/include                       # ZBar include
)

# Link libraries
target_link_libraries(lander
    ${OpenCV_LIBS}
    depthai::core
    /usr/lib/aarch64-linux-gnu/libzbar.so
)

# Optional warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(lander PRIVATE -Wall -Wextra -Wpedantic)
endif()