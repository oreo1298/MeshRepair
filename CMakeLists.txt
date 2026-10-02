cmake_minimum_required(VERSION 3.13)
project(MeshRepair VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(MESHREPAIR_BUILD_CLI   "Build the meshrepair command line tool" ON)
option(MESHREPAIR_BUILD_GUI   "Build the meshrepair-gui viewer (needs GLFW + OpenGL)" ON)
option(MESHREPAIR_BUILD_TESTS "Build the test suite" ON)

include(GNUInstallDirs)

add_subdirectory(libmeshrepair)

if(MESHREPAIR_BUILD_CLI)
    add_subdirectory(apps/cli)
endif()

if(MESHREPAIR_BUILD_GUI)
    add_subdirectory(apps/gui)
endif()

if(MESHREPAIR_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
