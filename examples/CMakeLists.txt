find_package(Eigen3 QUIET)

# examples:
MACRO(DEFINE_EXAMPLE _NAME)
	ADD_EXECUTABLE(${_NAME} ${_NAME}.cpp)
	set_target_properties(${_NAME} PROPERTIES FOLDER "Examples")
	TARGET_LINK_LIBRARIES(${_NAME} nanoflann::nanoflann)  # adds the "#include" directory.
  install(TARGETS ${_NAME}
          DESTINATION examples)
ENDMACRO()

DEFINE_EXAMPLE(dynamic_pointcloud_example)
DEFINE_EXAMPLE(pointcloud_example)
DEFINE_EXAMPLE(pointcloud_kdd_radius)
DEFINE_EXAMPLE(pointcloud_adaptor_example)
DEFINE_EXAMPLE(SO3_adaptor_example)
DEFINE_EXAMPLE(SO2_adaptor_example)
DEFINE_EXAMPLE(saveload_example)
DEFINE_EXAMPLE(vector_of_vectors_example)

IF(EIGEN3_FOUND)
	DEFINE_EXAMPLE(matrix_example)
    target_include_directories (matrix_example PRIVATE ${EIGEN3_INCLUDE_DIR})
ENDIF()
