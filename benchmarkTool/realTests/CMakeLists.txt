ADD_EXECUTABLE(nanoflann_test nanoflann_test.cpp)
TARGET_LINK_LIBRARIES(nanoflann_test nanoflann)

set(flann_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/flann/src/cpp")
include_directories(${flann_INCLUDE_DIRS})
set(flann_LIBRARIES "${CMAKE_SOURCE_DIR}/build/flann-prefix/src/flann-build/lib/libflann_cpp.so")
ADD_EXECUTABLE(flann_test flann_test.cpp)
TARGET_LINK_LIBRARIES(flann_test nanoflann ${flann_LIBRARIES})

set(fastann_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/fastann")
include_directories(${fastann_INCLUDE_DIRS})
set(fastann_LIBRARIES "${CMAKE_SOURCE_DIR}/build/fastann-prefix/src/fastann-build/libfastann.so")
ADD_EXECUTABLE(fastann_test fastann_test.cpp)
TARGET_LINK_LIBRARIES(fastann_test nanoflann ${fastann_LIBRARIES})

set(libkdtree_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/libkdtree")
include_directories(${libkdtree_INCLUDE_DIRS})
ADD_EXECUTABLE(libkdtree_test libkdtree_test.cpp)
