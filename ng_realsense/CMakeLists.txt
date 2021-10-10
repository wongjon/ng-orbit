# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.12.0)

project(ng_realsense_project)

# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# Find librealsense2 installed package
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
# Find installed Open3D, which exports Open3D::Open3D
find_package(Open3D REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Add the application sources to the target
# add_executable(${PROJECT_NAME} multiway-registration.cpp)
# add_executable(${PROJECT_NAME} pointcloud_test.cpp)
add_executable(${PROJECT_NAME} icp_implementation.cpp)

# Link librealsense2 to the target
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} Open3D::Open3D)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# On Windows if BUILD_SHARED_LIBS is enabled, copy .dll files to the executable directory
if(WIN32)
    get_target_property(open3d_type Open3D::Open3D TYPE)
    if(open3d_type STREQUAL "SHARED_LIBRARY")
        message(STATUS "Copying Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
        add_custom_command(TARGET Draw POST_BUILD
                           COMMAND ${CMAKE_COMMAND} -E copy
                                   ${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll
                                   ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>)
    endif()
endif()
