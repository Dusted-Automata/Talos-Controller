cmake_minimum_required(VERSION 3.20)
project(Controller)

cmake_policy(SET CMP0167 NEW)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
    set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*" OR "${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm64.*")
    set(ARCH arm64)
endif()


find_package(raylib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_library(common_settings INTERFACE)
target_compile_options(common_settings INTERFACE
    -Wall -Wextra -Wpedantic
    -fsanitize=undefined
    -fno-omit-frame-pointer
    -O0 -fPIC -ggdb
)
target_link_options(common_settings INTERFACE
    -fsanitize=undefined
)
target_include_directories(common_settings INTERFACE
    include
    src
    src/utils
    src/robots
    src/sensors
    src/planning
    src/control
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${raylib_INCLUDE_DIRS}
)
target_link_directories(common_settings INTERFACE
    lib/cpp/${ARCH}
)

set(COMMON_SOURCES
    src/robots/robot.cpp
    src/control/pid.cpp
    # src/control/mppi.cpp
    src/control/linear_controller.cpp
    src/planning/robot_path.cpp
    src/planning/motion_profile.cpp
    src/planning/path_planner.cpp
    src/sensors/ublox.cpp
    src/utils/frames.cpp
    src/utils/logger.cpp
    src/utils/socket.cpp
    src/utils/sim.cpp
    src/utils/EGM96.cpp
)

add_library(controller_core STATIC ${COMMON_SOURCES})
target_link_libraries(controller_core PUBLIC common_settings)

function(add_controller_executable target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} PRIVATE controller_core raylib)
endfunction()

add_controller_executable(sim src/robots/sim_bot.cpp)
# add_controller_executable(go1 src/robots/go1.cpp)
add_controller_executable(wheelchair src/robots/wheelchair.cpp)
# target_link_libraries(go1 PRIVATE pthread unitree_legged_sdk)

message(STATUS "=== Configuration Summary ===")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Architecture: ${ARCH}")
message(STATUS "Raylib: ${raylib_FOUND}")
message(STATUS "Boost: ${Boost_FOUND}")
message(STATUS "Eigen3: ${Eigen3_FOUND}")
message(STATUS "=============================")
