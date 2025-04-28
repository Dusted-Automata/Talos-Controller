cmake_minimum_required(VERSION 3.30)
project(Controller)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined -fno-omit-frame-pointer")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fno-omit-frame-pointer")


# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm64.*")
  set(ARCH arm64)
endif()

include_directories(include)
include_directories(src)
link_directories(unitree_legged_sdk/lib/cpp/${ARCH})

set(EXTRA_LIBS -pthread libunitree_legged_sdk.a)
set(CMAKE_CXX_FLAGS "-O3 -fPIC -ggdb")
# set(CMAKE_CXX_FLAGS "-O3 -fPIC -Wall -Wextra -ggdb")
set(CMAKE_CXX_STANDARD 20)

find_package(catkin QUIET)
if(${catkin_FOUND})
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_SOURCE_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.a
  )
endif()
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# one pc one process
add_executable(controller src/go1.cpp src/linear_controller.cpp src/robot.cpp src/pid.cpp src/mppi.cpp src/linear_controller.cpp src/pid.cpp src/logger.cpp src/frame_controller.cpp src/sensor_manager.cpp src/socket.cpp src/ublox.cpp src/path_controller.cpp)
add_executable(sim tools/sim.cpp src/robot.cpp src/mppi.cpp src/linear_controller.cpp src/pid.cpp src/logger.cpp src/frame_controller.cpp src/sensor_manager.cpp src/socket.cpp src/ublox.cpp src/path_controller.cpp)
add_executable(trajectory_run src/trajectoryRun.cpp src/linear_controller.cpp src/robot.cpp src/pid.cpp src/logger.cpp src/frame_controller.cpp src/sensor_manager.cpp src/socket.cpp src/ublox.cpp src/path_controller.cpp)
target_link_libraries(controller ${EXTRA_LIBS})
target_include_directories(controller PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(sim -lraylib -lGL -lm -lpthread -ldl -lrt )
target_include_directories(sim PRIVATE ${EIGEN3_INCLUDE_DIR})

target_link_libraries(trajectory_run)
target_include_directories(trajectory_run PRIVATE ${EIGEN3_INCLUDE_DIR})

# install
# install(TARGETS
#   controller
#   DESTINATION bin/)
install(DIRECTORY lib/cpp/${ARCH}/
  DESTINATION lib/
  USE_SOURCE_PERMISSIONS)
