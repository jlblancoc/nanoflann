find_package(Eigen3 QUIET) # optional, for examples only

# examples:
macro(DefineExample _NAME)
	add_executable(${_NAME} ${_NAME}.cpp)
	set_target_properties(${_NAME} PROPERTIES FOLDER "Examples")
	target_link_libraries(${_NAME} nanoflann::nanoflann)  # adds the "#include" directory.
endmacro()

DefineExample(dynamic_pointcloud_example)
DefineExample(pointcloud_example)
DefineExample(pointcloud_kdd_radius)
DefineExample(pointcloud_adaptor_example)
DefineExample(pointcloud_custom_metric)
DefineExample(pointcloud_custom_resultset)
DefineExample(SO3_adaptor_example)
DefineExample(SO2_adaptor_example)
DefineExample(saveload_example)
DefineExample(vector_of_vectors_example)

if(EIGEN3_FOUND)
	DefineExample(matrix_example)
	target_include_directories (matrix_example PRIVATE ${EIGEN3_INCLUDE_DIR})
endif()

# optional for gui examples:
find_package(mrpt-gui QUIET)
if (mrpt-gui_FOUND)
	add_subdirectory(examples_gui/nanoflann_gui_example_R3)
	add_subdirectory(examples_gui/nanoflann_gui_example_bearings)
endif()
