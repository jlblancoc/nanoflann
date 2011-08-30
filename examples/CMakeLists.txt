
# example: "pointcloud_example"
ADD_EXECUTABLE(pointcloud_example pointcloud_example.cpp)

# example: "pointcloud_adaptor_example"
ADD_EXECUTABLE(pointcloud_adaptor_example pointcloud_adaptor_example.cpp)

# example: "matrix_example"
IF(HAS_EIGEN3)
	ADD_EXECUTABLE(matrix_example matrix_example.cpp)
ENDIF(HAS_EIGEN3)

