ADD_EXECUTABLE(benchmark_nanoflann_real benchmark_nanoflann_real.cpp)
TARGET_LINK_LIBRARIES(benchmark_nanoflann_real nanoflann::nanoflann)
ADD_DEPENDENCIES(benchmarks benchmark_nanoflann_real)


# Since flann 1.8.5, LZ4 is vendored. Building against this version fails.
# This explicitly states to use the system's version of LZ4 instead.
# See: https://github.com/mariusmuja/flann/issues/384#issuecomment-668782051
MESSAGE(STATUS "Finding LZ4 libraries")
FIND_LIBRARY(LZ4_LIBRARIES NAMES liblz4.so HINTS "lz4/lib")
IF(LZ4_LIBRARIES)
    MESSAGE(STATUS "Found: ${LZ4_LIBRARIES}")
    ELSE()
    MESSAGE(STATUS "Not found: ${LZ4_LIBRARIES}")
    MESSAGE(FATAL_ERROR "Cannot find required LZ4 libraries")
ENDIF()

set(flann_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/flann/src/cpp")
include_directories(${flann_INCLUDE_DIRS})
set(flann_LIBRARIES "${CMAKE_SOURCE_DIR}/build/flann-prefix/src/flann-build/lib/libflann_cpp.so")
ADD_EXECUTABLE(benchmark_flann_real benchmark_flann_real.cpp)
TARGET_LINK_LIBRARIES(benchmark_flann_real nanoflann::nanoflann ${flann_LIBRARIES} ${LZ4_LIBRARIES})
ADD_DEPENDENCIES(benchmark_flann_real flann)
ADD_DEPENDENCIES(benchmarks benchmark_flann_real)

set(fastann_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/fastann")
include_directories(${fastann_INCLUDE_DIRS})
set(fastann_LIBRARIES "${CMAKE_SOURCE_DIR}/build/fastann-prefix/src/fastann-build/libfastann.so")
ADD_EXECUTABLE(benchmark_fastann_real benchmark_fastann_real.cpp)
TARGET_LINK_LIBRARIES(benchmark_fastann_real nanoflann::nanoflann ${fastann_LIBRARIES})
ADD_DEPENDENCIES(benchmark_fastann_real fastann)
ADD_DEPENDENCIES(benchmarks benchmark_fastann_real)

set(libkdtree_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/libkdtree")
include_directories(${libkdtree_INCLUDE_DIRS})
ADD_EXECUTABLE(benchmark_libkdtree_real benchmark_libkdtree_real.cpp)
set_target_properties(benchmark_nanoflann_real benchmark_flann_real benchmark_fastann_real benchmark_libkdtree_real PROPERTIES FOLDER "RealTests")
ADD_DEPENDENCIES(benchmark_libkdtree_real libkdtree)
ADD_DEPENDENCIES(benchmarks benchmark_libkdtree_real)
