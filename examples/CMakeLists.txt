# Try to find Eigen3 for use in the examples
FIND_PACKAGE(PkgConfig QUIET)
MESSAGE(STATUS "Check for Eigen3")
IF(PKG_CONFIG_FOUND)
  PKG_CHECK_MODULES(PKG_EIGEN3 QUIET eigen3) # Try to find eigen3 with pkg-config
ENDIF()
SET(EIGEN3_FOUND FALSE)
FIND_PATH(EIGEN3_INCLUDE NAMES signature_of_eigen3_matrix_library
  PATHS ${CMAKE_INSTALL_PREFIX}/include ${PKG_EIGEN3_INCLUDE_DIRS}
  PATH_SUFFIXES eigen3
)
IF (NOT EXISTS ${EIGEN3_INCLUDE})
  MESSAGE(STATUS "Eigen3 not found in standard location. Try passing -DEIGEN3_INCLUDE=/path/to/eigen3")
ELSE()
  MESSAGE(STATUS "Using Eigen3 in: ${EIGEN3_INCLUDE}")
  INCLUDE_DIRECTORIES("${EIGEN3_INCLUDE}")
  SET(EIGEN3_FOUND TRUE)
ENDIF()

# examples:
MACRO(DEFINE_EXAMPLE _NAME)
	ADD_EXECUTABLE(${_NAME} ${_NAME}.cpp)
	set_target_properties(${_NAME} PROPERTIES FOLDER "Examples")
	TARGET_LINK_LIBRARIES(${_NAME} nanoflann)  # adds the "#include" directory.
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
ENDIF()

