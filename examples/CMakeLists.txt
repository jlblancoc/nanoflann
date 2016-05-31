
# examples:
ADD_EXECUTABLE(pointcloud_example pointcloud_example.cpp)
ADD_EXECUTABLE(pointcloud_kdd_radius pointcloud_kdd_radius.cpp)
ADD_EXECUTABLE(pointcloud_adaptor_example pointcloud_adaptor_example.cpp)
ADD_EXECUTABLE(saveload_example saveload_example.cpp)
ADD_EXECUTABLE(vector_of_vectors_example vector_of_vectors_example.cpp KDTreeVectorOfVectorsAdaptor.h)

IF(EIGEN3_FOUND)
	ADD_EXECUTABLE(matrix_example matrix_example.cpp)
ENDIF()

