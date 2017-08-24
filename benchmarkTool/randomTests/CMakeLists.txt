ADD_EXECUTABLE(nanoflann_testRandom nanoflann_testRandom.cpp)
TARGET_LINK_LIBRARIES(nanoflann_testRandom nanoflann)

set(flann_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/flann/src/cpp")
include_directories(${flann_INCLUDE_DIRS})
set(flann_LIBRARIES "${CMAKE_SOURCE_DIR}/build/flann-prefix/src/flann-build/lib/libflann_cpp.so")
ADD_EXECUTABLE(flann_testRandom flann_testRandom.cpp)
TARGET_LINK_LIBRARIES(flann_testRandom nanoflann ${flann_LIBRARIES})

set(fastann_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/fastann")
include_directories(${fastann_INCLUDE_DIRS})
set(fastann_LIBRARIES "${CMAKE_SOURCE_DIR}/build/fastann-prefix/src/fastann-build/libfastann.so")
ADD_EXECUTABLE(fastann_testRandom fastann_testRandom.cpp)
TARGET_LINK_LIBRARIES(fastann_testRandom nanoflann ${fastann_LIBRARIES})

set(libkdtree_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/libkdtree")
include_directories(${libkdtree_INCLUDE_DIRS})
ADD_EXECUTABLE(libkdtree_testRandom libkdtree_testRandom.cpp)

set_target_properties(nanoflann_testRandom flann_testRandom fastann_testRandom libkdtree_testRandom PROPERTIES FOLDER "RandomTests")
