cmake_minimum_required(VERSION 3.0)
project(pointcloud_example)

# Find nanoflannConfig.cmake:
find_package(nanoflann REQUIRED)

add_executable(${PROJECT_NAME} pointcloud_example.cpp)

# Make sure the include path is used:
target_link_libraries(${PROJECT_NAME} nanoflann::nanoflann)

# for this example to find "../utils.h"
target_include_directories(${PROJECT_NAME} PRIVATE ".")


