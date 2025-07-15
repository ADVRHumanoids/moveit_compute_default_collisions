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

# Detect Linux Distribution 
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     if(EXISTS "/etc/lsb-release")
         execute_process(COMMAND cat /etc/lsb-release
                         COMMAND grep DISTRIB_CODENAME
                         COMMAND awk -F= "{ print $2 }"
                         COMMAND tr "\n" " "
                         COMMAND sed "s/ //"
                         OUTPUT_VARIABLE UBUNTU_VERSION
                         RESULT_VARIABLE UBUNTU_VERSION_RESULT
         )
     endif(EXISTS "/etc/lsb-release")
     
     message(STATUS "Configuring idynutils to be built on Ubuntu ${UBUNTU_VERSION}")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

if(${UBUNTU_VERSION} MATCHES "xenial")
    message(STATUS "Adding CXX support")
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
    check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
    if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    else()
        message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
    endif()
    set (moveit_compute_default_collisions_use_xenial 1)
else()
    set (moveit_compute_default_collisions_use_xenial 0)
endif(${UBUNTU_VERSION} MATCHES "xenial")

configure_file (
  "${PROJECT_SOURCE_DIR}/include/version.h.in"
  "${PROJECT_BINARY_DIR}/include/version.h"
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type " )
endif(CMAKE_BUILD_TYPE STREQUAL "Debug")

FIND_PACKAGE(Boost COMPONENTS filesystem program_options REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(moveit_setup_assistant REQUIRED)
FIND_PACKAGE(srdfdom REQUIRED)
FIND_PACKAGE(urdf REQUIRED)
find_package(catkin REQUIRED)

INCLUDE_DIRECTORIES(include ${orocos_kdl_INCLUDE_DIRS}
                            ${moveit_setup_assistant_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIRS})

file(GLOB_RECURSE proj_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)
ADD_EXECUTABLE(moveit_compute_default_collisions main.cpp
                                                 src/moveit_compute_default_collisions.cpp
                                                 ${proj_INCLUDES})

TARGET_LINK_LIBRARIES(moveit_compute_default_collisions ${orocos_kdl_LIBRARIES}
                                                        ${moveit_setup_assistant_LIBRARIES}
                                                        ${Boost_LIBRARIES})

install(TARGETS moveit_compute_default_collisions
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

catkin_package(
   INCLUDE_DIRS include
)


ADD_LIBRARY(mcdc src/moveit_compute_default_collisions.cpp ${proj_INCLUDES})

TARGET_LINK_LIBRARIES(mcdc ${orocos_kdl_LIBRARIES} ${moveit_setup_assistant_LIBRARIES}
                                                        ${Boost_LIBRARIES})

install(TARGETS mcdc
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)


add_subdirectory(bindings/python/)

