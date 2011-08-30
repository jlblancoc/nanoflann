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


# Extract library version into "NANOFLANN_VERSION"
# -----------------------------------------------------
# Look for: "#define NANOFLANN_VERSION 0xABC"
file(READ "${nanoflann_SOURCE_DIR}/include/nanoflann.hpp" STR_HPP)
STRING(REGEX MATCHALL "NANOFLANN_VERSION.*0x[0-9,A-F]+" CMAKE_VERSION_LINE "${STR_HPP}")
STRING(REGEX MATCHALL "0x[0-9,A-F]+" NANOFLANN_VERSION_HEX "${CMAKE_VERSION_LINE}")

STRING(REGEX REPLACE "0x(.).*" "\\1" NANOFLANN_VERSION_MAJOR "${NANOFLANN_VERSION_HEX}" )
STRING(REGEX REPLACE "0x.(.).*" "\\1" NANOFLANN_VERSION_MINOR "${NANOFLANN_VERSION_HEX}" )
STRING(REGEX REPLACE "0x..(.).*" "\\1" NANOFLANN_VERSION_PATCH "${NANOFLANN_VERSION_HEX}" )

MESSAGE(STATUS "nanoflann version: ${NANOFLANN_VERSION_MAJOR}.${NANOFLANN_VERSION_MINOR}.${NANOFLANN_VERSION_PATCH}")

# Generate the pkg-config file:
CONFIGURE_FILE(
	"${nanoflann_SOURCE_DIR}/nanoflann.pc.in" 
	"${nanoflann_BINARY_DIR}/nanoflann.pc" @ONLY IMMEDIATE )

# Compiler options:
IF(CMAKE_COMPILER_IS_GNUCXX)
	 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -mtune=native ")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)


# Use eigen3 for examples?
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
SET( BUILD_TESTING ON CACHE BOOL "Build unit tests")
SET(HAVE_GTEST 0)

IF( BUILD_TESTING)
	# Try using libgtest (Google testing library) from the system, if available
	IF(UNIX)
		FIND_FILE(GTEST_CONFIG_FILE gtest-config)
		IF(GTEST_CONFIG_FILE)
			MARK_AS_ADVANCED(GTEST_CONFIG_FILE)

			SET(HAVE_GTEST 1)

			# Get the config params:
			EXECUTE_PROCESS(COMMAND ${GTEST_CONFIG_FILE} --libs
				RESULT_VARIABLE CMAKE_GTEST_CONFIG_RES
				OUTPUT_VARIABLE CMAKE_GTEST_LIBS
				OUTPUT_STRIP_TRAILING_WHITESPACE
				)
			IF(${CMAKE_GTEST_CONFIG_RES})
				MESSAGE("Error invoking GTEST config file:\n ${GTEST_CONFIG_FILE} --libs")
			ENDIF(${CMAKE_GTEST_CONFIG_RES})

			EXECUTE_PROCESS(COMMAND ${GTEST_CONFIG_FILE} --cxxflags
				RESULT_VARIABLE CMAKE_GTEST_CONFIG_RES
				OUTPUT_VARIABLE CMAKE_GTEST_CFLAGS
				OUTPUT_STRIP_TRAILING_WHITESPACE
				)
			IF(${CMAKE_GTEST_CONFIG_RES})
				MESSAGE("Error invoking GTEST config file:\n ${GTEST_CONFIG_FILE} --cxxflags")
			ENDIF(${CMAKE_GTEST_CONFIG_RES})

		ELSE(GTEST_CONFIG_FILE)
			SET(HAVE_GTEST 0)
		ENDIF(GTEST_CONFIG_FILE)
	ENDIF(UNIX)

	IF(HAVE_GTEST)
		add_subdirectory(tests)
	ELSE(HAVE_GTEST)
		MESSAGE(STATUS "Warning: Not building unit test since gtest-config was not found")
	ENDIF(HAVE_GTEST)

ENDIF( BUILD_TESTING)


