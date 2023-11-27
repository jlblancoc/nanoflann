cmake_minimum_required(VERSION 3.0)
project(nanoflann_gui_example_R3)

find_package(nanoflann REQUIRED)
find_package(mrpt-gui REQUIRED)

add_executable(${PROJECT_NAME}_radius ${PROJECT_NAME}.cpp)
add_executable(${PROJECT_NAME}_knn ${PROJECT_NAME}.cpp)
add_executable(${PROJECT_NAME}_rknn ${PROJECT_NAME}.cpp)

target_compile_definitions(${PROJECT_NAME}_radius PRIVATE USE_RADIUS_SEARCH)
target_compile_definitions(${PROJECT_NAME}_knn PRIVATE USE_KNN_SEARCH)
target_compile_definitions(${PROJECT_NAME}_rknn PRIVATE USE_RKNN_SEARCH)

# optimized build:
if (CMAKE_COMPILER_IS_GNUCXX)
	target_compile_options(${PROJECT_NAME}_radius PRIVATE -O2 -mtune=native)
	target_compile_options(${PROJECT_NAME}_knn PRIVATE -O2 -mtune=native)
	target_compile_options(${PROJECT_NAME}_rknn PRIVATE -O2 -mtune=native)
endif()

# Make sure the include path is used:
target_link_libraries(${PROJECT_NAME}_radius nanoflann::nanoflann mrpt::gui)
target_link_libraries(${PROJECT_NAME}_knn nanoflann::nanoflann mrpt::gui)
target_link_libraries(${PROJECT_NAME}_rknn nanoflann::nanoflann mrpt::gui)

# for this example to find "../utils.h"
target_include_directories(${PROJECT_NAME}_radius PRIVATE ".")
target_include_directories(${PROJECT_NAME}_knn PRIVATE ".")
target_include_directories(${PROJECT_NAME}_rknn PRIVATE ".")
