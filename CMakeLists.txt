# Test CMake version
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project
PROJECT(PRM)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})

# Verbose mode prints more info when doing make
set(CMAKE_VERBOSE_MAKEFILE false)

# Set build type to release
SET(CMAKE_BUILD_TYPE Release)

# Use RobWork
SET(RobWork_DIR ${RW_ROOT}/cmake) 
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# Set the output dir for generated libraries and binaries
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${ROOT}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${ROOT}/libs)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ROOT}/libs)

# Add the source files to build
SET(SrcFiles ${ROOT}/src/PRM.cpp)

# The shared library to build:
ADD_EXECUTABLE(PRM ${SrcFiles})
TARGET_LINK_LIBRARIES(PRM ${ROBWORK_LIBRARIES})

# For compile with the last version of C++
list(APPEND CMAKE_CXX_FLAGS "-std=c++0x  -g -O3 -ftest-coverage -fprofile-arcs")
