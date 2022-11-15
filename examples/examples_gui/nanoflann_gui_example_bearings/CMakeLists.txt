cmake_minimum_required(VERSION 3.0)
project(nanoflann_gui_example_bearings)

find_package(nanoflann REQUIRED)
find_package(mrpt-gui REQUIRED)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)

# optimized build:
if (CMAKE_COMPILER_IS_GNUCXX)
	target_compile_options(${PROJECT_NAME} PRIVATE -O2 -mtune=native)
endif()

# Make sure the include path is used:
target_link_libraries(${PROJECT_NAME} nanoflann::nanoflann mrpt::gui)

