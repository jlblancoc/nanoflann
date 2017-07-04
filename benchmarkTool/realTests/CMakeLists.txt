set(flann_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/flann/src/cpp/")
include_directories(${flann_INCLUDE_DIRS})

# examples:
ADD_EXECUTABLE(nanoflann_test nanoflann_test.cpp)
TARGET_LINK_LIBRARIES(nanoflann_test nanoflann)

ADD_EXECUTABLE(flann_test flann_test.cpp)
TARGET_LINK_LIBRARIES(flann_test nanoflann)
