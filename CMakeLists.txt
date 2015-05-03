cmake_minimum_required (VERSION 2.6)
project (fw-test-harness)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

set(CMAKE_CXX_FLAGS "-Wall -Werror -std=c++11")

set(FIRMWARE_DIR ${CMAKE_SOURCE_DIR}/external/Firmware/)

# some hacks to get of NuttX/PX4 specialities
add_definitions(-D__EXPORT=)
add_definitions(-DM_PI_2_F=\(float\)M_PI_2)
add_definitions(-D_GLIBCXX_USE_C99_MATH)

find_package(Eigen REQUIRED)

# include dirs, order is relevant, our customizations need to be in 1st place
include_directories(./Firmware_custom/)
include_directories(${FIRMWARE_DIR}src/modules)
include_directories(${FIRMWARE_DIR}src/lib)
include_directories(${FIRMWARE_DIR}src)
include_directories(${EIGEN_INCLUDE_DIR})

# libraries
add_library(pitch_controller SHARED
  ${FIRMWARE_DIR}src/lib/ecl/attitude_fw/ecl_pitch_controller.cpp
  ${FIRMWARE_DIR}src/lib/ecl/attitude_fw/ecl_controller.cpp
  ${FIRMWARE_DIR}src/platforms/ros/perf_counter.cpp
  ${FIRMWARE_DIR}src/lib/mathlib/math/Limits.cpp
)

message(info ${FIRMWARE_DIR})
