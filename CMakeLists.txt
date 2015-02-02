set(PROJECT_NAME ttrack)
project(${PROJECT_NAME})
set(${PROJECT_NAME}_VERSION_MAJOR 0)
set(${PROJECT_NAME}_VERSION_MINOR 1)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR})

cmake_minimum_required(VERSION 2.7)

#needed for cinder
add_definitions(-D_UNICODE)
add_definitions(-DUNICODE)

# Overide with cmake -DCMAKE_BUILD_TYPE=Debug
if(NOT CMAKE_BUILD_TYPE)
   message("Build type not set (defaults to release)")
   message("-DCMAKE_BUILD_TYPE=Debug for debug")
   set( CMAKE_BUILD_TYPE Release )
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_VERBOSE_MAKEFILE true)

option(BUILD_SHARED_LIBS "Build Shared Library" ON)

################################################################################
# Add local path for finding packages, set the local version first
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CmakeModules")

# Platform configuration vars
include(${CMAKE_MODULE_PATH}/SetPlatformVars.cmake)

#set(CMAKE_DEBUG_POSTFIX "_debug")
string(TOLOWER ${PROJECT_NAME} LIBRARY_NAME)

# Build source files
add_subdirectory(src)




