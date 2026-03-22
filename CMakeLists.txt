cmake_minimum_required(VERSION 3.20)
project(adsim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(CompilerOptions)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_library(adsim_core STATIC
    src/math/pid.cpp
    src/dynamics/flight_model.cpp
    src/control/controller.cpp
    src/sensors/imu.cpp
    src/estimation/complementary_filter.cpp
    src/navigation/waypoint_navigator.cpp
    src/failsafe/failsafe.cpp
    src/simulation/simulator.cpp
    src/logging/csv_logger.cpp
    src/config/config.cpp
    src/network/telemetry_server.cpp
)

if(WIN32)
    target_link_libraries(adsim_core PRIVATE ws2_32)
endif()

target_include_directories(adsim_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

adsim_set_compile_options(adsim_core)

add_executable(adsim apps/adsim/main.cpp)
target_link_libraries(adsim PRIVATE adsim_core)
adsim_set_compile_options(adsim)

option(ADSIM_BUILD_TESTS "Build test executables" ON)

if(ADSIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
