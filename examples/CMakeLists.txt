find_package(Eigen3 QUIET) # optional, for examples only

option(NANOFLANN_INSTALL_EXAMPLES "Install the example binaries" OFF)
set(INSTALL_EXAMPLES_DIR "${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/examples" CACHE STRING "Example install folder")

# examples:
macro(DefineExample _NAME)
	add_executable(${_NAME} ${_NAME}.cpp)
	set_target_properties(${_NAME} PROPERTIES FOLDER "Examples")
	target_link_libraries(${_NAME} nanoflann::nanoflann)  # adds the "#include" directory.
	if(NOT MSVC)
		target_compile_options(${_NAME} PRIVATE
			-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
	else()
		target_compile_options(${_NAME} PRIVATE /W4)
	endif()
	if(NANOFLANN_INSTALL_EXAMPLES)
		install(TARGETS ${_NAME}
			DESTINATION ${INSTALL_EXAMPLES_DIR})
	endif()
endmacro()

DefineExample(dynamic_pointcloud_example)
DefineExample(pointcloud_example)
DefineExample(pointcloud_kdd_radius)
DefineExample(pointcloud_adaptor_example)
DefineExample(pointcloud_custom_metric)
DefineExample(pointcloud_custom_resultset)
DefineExample(SO3_adaptor_example)
DefineExample(SO2_adaptor_example)
DefineExample(manifold_se3_example)
DefineExample(manifold_s2_example)
DefineExample(manifold_se3_incremental_example)
DefineExample(saveload_example)
DefineExample(vector_of_vectors_example)

if(EIGEN3_FOUND)
	DefineExample(matrix_example)
	target_include_directories (matrix_example PRIVATE ${EIGEN3_INCLUDE_DIR})
endif()

add_subdirectory(examples_gui)
