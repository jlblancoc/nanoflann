
# examples:
ADD_EXECUTABLE(pointcloud_example pointcloud_example.cpp)
ADD_EXECUTABLE(pointcloud_kdd_radius pointcloud_kdd_radius.cpp)
ADD_EXECUTABLE(pointcloud_adaptor_example pointcloud_adaptor_example.cpp)

IF(HAS_EIGEN3)
	ADD_EXECUTABLE(matrix_example matrix_example.cpp)
	ADD_EXECUTABLE(vector_of_vectors_example vector_of_vectors_example.cpp)
ENDIF(HAS_EIGEN3)

