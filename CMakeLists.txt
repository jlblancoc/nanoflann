# ----------------------------------------------------------------------------
# Root CMake file for nanoflann
#
#  2011 - Jose Luis Blanco 
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.6)

PROJECT(nanoflann)

# --------------------------------------------------------------
# Indicate CMake 2.7 and above that we don't want to mix relative
#  and absolute paths in lib lists. Run "cmake --help-policy CMP0003" for info
# --------------------------------------------------------------
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

SET( EXECUTABLE_OUTPUT_PATH ${nanoflann_BINARY_DIR}/bin CACHE PATH "Output directory for programs" )


# Compiler options:
IF(CMAKE_COMPILER_IS_GNUCXX)
	 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -mtune=native ")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)



# Include dir:
INCLUDE_DIRECTORIES(${nanoflann_SOURCE_DIR}/include)

# Examples
add_subdirectory(examples)

# Tests
#add_subdirectory(tests)


