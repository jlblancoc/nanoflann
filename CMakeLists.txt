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


SET(HAS_EIGEN3 0)

INCLUDE(FindPkgConfig OPTIONAL)
IF(PKG_CONFIG_FOUND)
	INCLUDE(FindPkgConfig)
	PKG_CHECK_MODULES(PKG_EIGEN3 eigen3)	# Find eigen3 with pkg-config
	IF(PKG_EIGEN3_FOUND)
		MESSAGE(STATUS "Using eigen3 in: ${PKG_EIGEN3_INCLUDE_DIRS}")
		INCLUDE_DIRECTORIES(${PKG_EIGEN3_INCLUDE_DIRS})
		SET(HAS_EIGEN3 1)
	ENDIF(PKG_EIGEN3_FOUND)
ENDIF(PKG_CONFIG_FOUND)




# Include dir:
INCLUDE_DIRECTORIES(${nanoflann_SOURCE_DIR}/include)

# Examples
add_subdirectory(examples)

# Tests
#add_subdirectory(tests)


