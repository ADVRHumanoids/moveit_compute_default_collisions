# Copyright: (C) 2014 Walkman Consortium
# Authors: Alessio Rocchi
# CopyPolicy: Released under the terms of the GNU LGPL v2.1.

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
PROJECT(moveit_compute_default_collisions)
set (moveit_compute_default_collisions_VERSION_MAJOR 0)
set (moveit_compute_default_collisions_VERSION_MINOR 1)

configure_file (
  "${PROJECT_SOURCE_DIR}/include/version.h.in"
  "${PROJECT_BINARY_DIR}/include/version.h"
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type " )
endif(CMAKE_BUILD_TYPE STREQUAL "Debug")

FIND_PACKAGE(Boost COMPONENTS filesystem program_options REQUIRED)
FIND_PACKAGE(moveit_setup_assistant REQUIRED)
FIND_PACKAGE(srdfdom REQUIRED)
FIND_PACKAGE(urdf REQUIRED)

INCLUDE_DIRECTORIES(include ${moveit_setup_assistant_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIRS})

file(GLOB_RECURSE proj_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)
ADD_EXECUTABLE(moveit_compute_default_collisions main.cpp
                                                 src/moveit_compute_default_collisions.cpp
                                                 ${proj_INCLUDES})

TARGET_LINK_LIBRARIES(moveit_compute_default_collisions ${moveit_setup_assistant_LIBRARIES}
                                                        ${Boost_LIBRARIES})
install(TARGETS moveit_compute_default_collisions
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

