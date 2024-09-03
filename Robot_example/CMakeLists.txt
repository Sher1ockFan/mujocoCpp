cmake_minimum_required(VERSION 3.16)
project(pnd_mujoco)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(mujoco REQUIRED)
include_directories(include()
                    src/
                    )

FILE (GLOB SIM_SRC
        src/src/*.cc)

set(SIM_DEPENDENCIES 
    pthread
    mujoco::mujoco
    glfw
    yaml-cpp)


add_executable(pnd_mujoco ${SIM_SRC} src/main.cc src/mujoco_robot.cpp)
target_link_libraries(pnd_mujoco  ${SIM_DEPENDENCIES})

SET(CMAKE_BUILD_TYPE Release)