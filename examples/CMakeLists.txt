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
ADD_EXECUTABLE(pointcloud_example pointcloud_example.cpp)
TARGET_LINK_LIBRARIES(pointcloud_example nanoflann)

ADD_EXECUTABLE(pointcloud_kdd_radius pointcloud_kdd_radius.cpp)
TARGET_LINK_LIBRARIES(pointcloud_kdd_radius nanoflann)

ADD_EXECUTABLE(pointcloud_adaptor_example pointcloud_adaptor_example.cpp)
TARGET_LINK_LIBRARIES(pointcloud_adaptor_example nanoflann)

ADD_EXECUTABLE(saveload_example saveload_example.cpp)
TARGET_LINK_LIBRARIES(saveload_example nanoflann)

ADD_EXECUTABLE(vector_of_vectors_example vector_of_vectors_example.cpp KDTreeVectorOfVectorsAdaptor.h)
TARGET_LINK_LIBRARIES(vector_of_vectors_example nanoflann)

IF(EIGEN3_FOUND)
	ADD_EXECUTABLE(matrix_example matrix_example.cpp)
	TARGET_LINK_LIBRARIES(matrix_example nanoflann)
ENDIF()

