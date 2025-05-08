cmake_minimum_required(VERSION 3.20)
project(Controller)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined -fno-omit-frame-pointer -O0 -fPIC -ggdb")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=undefined -fno-omit-frame-pointer -O0 -fPIC -ggdb")
set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic -fsanitize=undefined)

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
    set(RAYLIB_LIB_DIR ${CMAKE_BINARY_DIR}/raylib/lib)
    set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*" OR "${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm64.*")
    set(RAYLIB_LIB_DIR ${CMAKE_BINARY_DIR}/raylib/lib64)
    set(ARCH arm64)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(raylib QUIET)


find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
cmake_policy(SET CMP0167 NEW)

include_directories(include src src/utils src/robots src/sensors src/controllers ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(unitree_legged_sdk/lib/cpp/${ARCH})
set(EXTRA_LIBS -pthread libunitree_legged_sdk.a)

# =========== RAYLIB ==============
if(raylib_FOUND)
    message(STATUS "Found system-installed Raylib")
    set(RAYLIB_INCLUDE_DIR ${raylib_INCLUDE_DIRS})
    set(RAYLIB_LIBRARY ${raylib_LIBRARIES})
    set(RAYLIB_EXTERNAL OFF)
else()
    message(STATUS "System-installed Raylib not found")
    set(RAYLIB_INSTALL_DIR ${CMAKE_BINARY_DIR}/raylib)
    set(RAYLIB_INCLUDE_DIR ${CMAKE_BINARY_DIR}/raylib/include)
    set(RAYLIB_LIBRARY ${RAYLIB_LIB_DIR}/libraylib.a)
    set(RAYLIB_EXTERNAL ON)
endif()

if (RAYLIB_EXTERNAL)
    if(NOT EXISTS ${RAYLIB_LIBRARY})
        message(STATUS "Raylib not found, will download and build it")

        include(ExternalProject)
        ExternalProject_Add(
            raylib_ext
            GIT_REPOSITORY https://github.com/raysan5/raylib.git
            GIT_TAG master
            CMAKE_ARGS 
            -DCMAKE_INSTALL_PREFIX=${RAYLIB_INSTALL_DIR}
            -DBUILD_SHARED_LIBS=OFF
            BUILD_BYPRODUCTS ${RAYLIB_LIBRARY}
            INSTALL_DIR ${RAYLIB_INSTALL_DIR}
        )

        add_custom_target(raylib_target DEPENDS raylib_ext)
    else()
        message(STATUS "Raylib already built, skipping build")
        add_custom_target(raylib_target)
    endif()
else()
    add_custom_target(raylib_target)  
endif()




# === Main Executables ===
add_executable(controller
    src/robots/go1.cpp
    src/robots/robot.cpp
    src/controllers/mppi.cpp
    src/controllers/path_controller.cpp
    src/controllers/linear_controller.cpp
    src/sensors/sensor_manager.cpp
    src/sensors/ublox.cpp
    src/utils/pid.cpp
    src/utils/frames.cpp
    src/utils/logger.cpp
    src/utils/socket.cpp
    src/utils/sim.cpp
)
add_executable(sim
    src/run_sim.cpp
    src/robots/robot.cpp
    src/controllers/mppi.cpp
    src/controllers/path_controller.cpp
    src/controllers/linear_controller.cpp
    src/sensors/sensor_manager.cpp
    src/sensors/ublox.cpp
    src/utils/pid.cpp
    src/utils/frames.cpp
    src/utils/logger.cpp
    src/utils/socket.cpp
    src/utils/sim.cpp
)

# Link raylib after it's built
add_dependencies(sim raylib_target)
add_dependencies(controller raylib_target)

target_compile_options(sim PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(controller PRIVATE -Wall -Wextra -Wpedantic)

# Link libraries for raylib
target_link_libraries(sim PRIVATE ${RAYLIB_LIBRARY} -lGL -lm -lpthread -ldl -lrt)
target_link_libraries(controller PRIVATE ${EXTRA_LIBS} ${RAYLIB_LIBRARY} -lGL -lm -lpthread -ldl -lrt)

target_include_directories(sim PRIVATE ${RAYLIB_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})
target_include_directories(controller PRIVATE ${RAYLIB_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

# install
install(DIRECTORY lib/cpp/${ARCH}/
  DESTINATION lib/
  USE_SOURCE_PERMISSIONS)


