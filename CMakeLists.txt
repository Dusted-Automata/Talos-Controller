cmake_minimum_required(VERSION 3.30)
project(Controller)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

include_directories(include)
link_directories(lib/cpp/${ARCH})

set(EXTRA_LIBS -pthread libunitree_legged_sdk.a)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin QUIET)
if(${catkin_FOUND})
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_SOURCE_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.a
  )
endif()
find_package(Eigen3 REQUIRED)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# one pc one process
add_executable(controller go1.cpp robot.cpp pid.cpp trajectory.cpp)
add_executable(sim ./tools/sim.cpp)
target_link_libraries(controller ${EXTRA_LIBS})
target_include_directories(controller PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(sim -lraylib -lGL -lm)


# install
install(TARGETS
  controller
  DESTINATION bin/)
install(DIRECTORY lib/cpp/${ARCH}/
  DESTINATION lib/
  USE_SOURCE_PERMISSIONS)
