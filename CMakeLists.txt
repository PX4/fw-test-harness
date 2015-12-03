cmake_minimum_required (VERSION 2.6)
project (fw-test-harness)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/")

set(CMAKE_CXX_FLAGS "-Wall -Werror -std=c++11")

set(FIRMWARE_DIR ${CMAKE_SOURCE_DIR}/external/Firmware/)
set(FIRMWARE_CUSTOM_DIR ${CMAKE_SOURCE_DIR}/Firmware_custom/)

# some hacks to get of NuttX/PX4 specialities
add_definitions(-D__EXPORT=)
add_definitions(-DM_PI_2_F=\(float\)M_PI_2)
add_definitions(-D_GLIBCXX_USE_C99_MATH)
add_definitions(-D__PX4_POSIX=)

find_package(Eigen REQUIRED)

# include dirs, order is relevant, our customizations need to be in 1st place
include_directories(${FIRMWARE_CUSTOM_DIR})
include_directories(${FIRMWARE_CUSTOM_DIR}/platforms)
include_directories(${FIRMWARE_DIR}src/platforms)
include_directories(${FIRMWARE_DIR}src/platforms/posix/include)
include_directories(${FIRMWARE_DIR}src/modules)
include_directories(${FIRMWARE_DIR}src/modules/fw_pos_control_l1/mtecs)
include_directories(${FIRMWARE_DIR}src/lib)
include_directories(${FIRMWARE_DIR}src/lib/matrix)
include_directories(${FIRMWARE_DIR}src/lib/mathlib/math/filter)
include_directories(${FIRMWARE_DIR}src/include)
include_directories(${FIRMWARE_DIR}src)
include_directories(${EIGEN_INCLUDE_DIR})

# libraries
add_library(attitude_controller SHARED
  ${FIRMWARE_DIR}src/lib/ecl/attitude_fw/ecl_controller.cpp
  ${FIRMWARE_DIR}src/lib/ecl/attitude_fw/ecl_pitch_controller.cpp
  ${FIRMWARE_DIR}src/lib/ecl/attitude_fw/ecl_roll_controller.cpp
  ${FIRMWARE_DIR}src/lib/ecl/attitude_fw/ecl_yaw_controller.cpp
  ${FIRMWARE_DIR}src/platforms/ros/perf_counter.cpp
  ${FIRMWARE_DIR}src/lib/mathlib/math/Limits.cpp
  ${FIRMWARE_DIR}src/platforms/posix/px4_layer/px4_log.c
  ${FIRMWARE_CUSTOM_DIR}platforms/drv_hrt.c
  )

add_library(mtecs SHARED
  ${FIRMWARE_DIR}src/modules/fw_pos_control_l1/mtecs/mTecs.cpp
  ${FIRMWARE_DIR}src/modules/fw_pos_control_l1/mtecs/limitoverride.cpp
  ${FIRMWARE_DIR}src/modules/controllib/block/Block.cpp
  ${FIRMWARE_DIR}src/modules/controllib/block/BlockParam.cpp
  ${FIRMWARE_DIR}src/modules/controllib/blocks.cpp
  ${FIRMWARE_CUSTOM_DIR}platforms/drv_hrt.c
  ${FIRMWARE_CUSTOM_DIR}param/param.cpp
  # ${FIRMWARE_CUSTOM_DIR}param/param_load.cpp
  ${FIRMWARE_DIR}src/lib/mathlib/math/test/test.cpp # needed by blocks
  ${FIRMWARE_DIR}src/lib/mathlib/math/filter/LowPassFilter2p.cpp
  )

# add python module
set(PACKAGE_VERSION 0.1)
add_subdirectory(fw_test_harness)
