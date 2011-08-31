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
file(WRITE "${nanoflann_BINARY_DIR}/version" "${NANOFLANN_VERSION_MAJOR}.${NANOFLANN_VERSION_MINOR}.${NANOFLANN_VERSION_PATCH}")
file(WRITE "${nanoflann_SOURCE_DIR}/version" "${NANOFLANN_VERSION_MAJOR}.${NANOFLANN_VERSION_MINOR}.${NANOFLANN_VERSION_PATCH}")

# Compiler options:
IF(CMAKE_COMPILER_IS_GNUCXX)
	 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -mtune=native -Wall -pedantic")
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
	FIND_FILE(GTEST_HEADER_FILE gtest/gtest.h)
	IF(GTEST_HEADER_FILE)
		MARK_AS_ADVANCED(GTEST_HEADER_FILE)

		SET(HAVE_GTEST 1)
	ENDIF(GTEST_HEADER_FILE)

	IF(HAVE_GTEST)
		add_subdirectory(tests)
	ELSE(HAVE_GTEST)
		MESSAGE(STATUS "Warning: Not building unit test since gtest/gtest.h was not found")
	ENDIF(HAVE_GTEST)
ENDIF( BUILD_TESTING)

# --------------------------------------------------------------------
# Install/uninstall targets
# --------------------------------------------------------------------

#--------------------------------------------------------------
# If we are building the final step of the Debian package,
#  save each library files in the corresponding directories:
#--------------------------------------------------------------
IF(CMAKE_USE_DEB_POSTFIXS)
	# Values when building a Debian package ---------------
	MESSAGE(STATUS "** Using Debian post-fix for install directories **")
	SET(libnanoflann_dev_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/libnanoflann-dev/usr/")
	SET(nanoflann_pkgconfig_INSTALL_PREFIX "/usr")	# Values when building a Debian package
ELSE(CMAKE_USE_DEB_POSTFIXS)
	# Values under normal conditions -----------------------
	SET(libnanoflann_dev_INSTALL_PREFIX "")
	SET(nanoflann_pkgconfig_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}") 	# Values under normal conditions
ENDIF(CMAKE_USE_DEB_POSTFIXS)

# Generate the pkg-config file:
CONFIGURE_FILE(
	"${nanoflann_SOURCE_DIR}/nanoflann.pc.in" 
	"${nanoflann_BINARY_DIR}/nanoflann.pc" @ONLY IMMEDIATE )


# Uninstall target, for "make uninstall"
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  @ONLY IMMEDIATE)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")


INSTALL(
	FILES "${nanoflann_BINARY_DIR}/nanoflann.pc"
	DESTINATION ${libnanoflann_dev_INSTALL_PREFIX}lib${LIB_SUFFIX}/pkgconfig )

INSTALL(
	FILES "${nanoflann_SOURCE_DIR}/include/nanoflann.hpp"
	DESTINATION ${libnanoflann_dev_INSTALL_PREFIX}include/ )

IF(CMAKE_USE_DEB_POSTFIXS)
	INSTALL(
		FILES "${nanoflann_SOURCE_DIR}/copyright"
		DESTINATION ${libnanoflann_dev_INSTALL_PREFIX}share/doc/libnanoflann-dev/ )
ENDIF(CMAKE_USE_DEB_POSTFIXS)

