# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealSense)

find_package( OpenCV REQUIRED )
include_directories (
        ${OpenCV_INCLUDE_DIRS}
)
link_directories(
        ${OpenCV_LIB_DIR}
)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

include_directories(rs-hello-realsense)

add_executable(rs-hello-realsense rs-hello-realsense.cpp)
target_link_libraries(rs-hello-realsense realsense2)

add_executable(depth_colorRegister depth_colorRegister.cpp)
target_link_libraries(depth_colorRegister realsense2 ${OpenCV_LIBS})


