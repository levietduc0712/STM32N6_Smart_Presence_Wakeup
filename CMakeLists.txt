# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(vl53l5cx_smart_presence_wakeup)

# Path to the ST VL53L5CX ULD driver
set(VL53L5CX_ULD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/drivers/sensor/vl53l5cx/uld)
set(VL53L5CX_PLATFORM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/drivers/sensor/vl53l5cx)

# Include headers: ULD API headers + our custom platform.h
target_include_directories(app PRIVATE
    ${VL53L5CX_ULD_DIR}/inc
    ${VL53L5CX_PLATFORM_DIR}
)

# Build ULD API source + plugins + platform adaptation + application
target_sources(app PRIVATE
    ${VL53L5CX_ULD_DIR}/src/vl53l5cx_api.c
    ${VL53L5CX_ULD_DIR}/src/vl53l5cx_plugin_motion_indicator.c
    ${VL53L5CX_PLATFORM_DIR}/platform.c
    src/main.c
)

# Reduce VL53L5CX compile warnings
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  set_source_files_properties(
    ${VL53L5CX_ULD_DIR}/src/vl53l5cx_api.c
    PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter"
  )
endif()
