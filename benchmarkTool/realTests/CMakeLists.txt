ADD_EXECUTABLE(nanoflann_testReal nanoflann_testReal.cpp)
TARGET_LINK_LIBRARIES(nanoflann_testReal nanoflann)

set(flann_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/flann/src/cpp")
include_directories(${flann_INCLUDE_DIRS})
set(flann_LIBRARIES "${CMAKE_SOURCE_DIR}/build/flann-prefix/src/flann-build/lib/libflann_cpp.so")
ADD_EXECUTABLE(flann_testReal flann_testReal.cpp)
TARGET_LINK_LIBRARIES(flann_testReal nanoflann ${flann_LIBRARIES})

set(fastann_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/fastann")
include_directories(${fastann_INCLUDE_DIRS})
set(fastann_LIBRARIES "${CMAKE_SOURCE_DIR}/build/fastann-prefix/src/fastann-build/libfastann.so")
ADD_EXECUTABLE(fastann_testReal fastann_testReal.cpp)
TARGET_LINK_LIBRARIES(fastann_testReal nanoflann ${fastann_LIBRARIES})

set(libkdtree_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/libkdtree")
include_directories(${libkdtree_INCLUDE_DIRS})
ADD_EXECUTABLE(libkdtree_testReal libkdtree_testReal.cpp)

set_target_properties(nanoflann_testReal flann_testReal fastann_testReal libkdtree_testReal PROPERTIES FOLDER "RealTests")
