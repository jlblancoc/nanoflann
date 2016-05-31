# ----------------------------------------------------------------------------
# Root CMake file for nanoflann
#
#  2011-2016 - Jose Luis Blanco 
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.6)

PROJECT(nanoflann)

# --------------------------------------------------------------
# Indicate CMake 2.7 and above that we don't want to mix relative
#  and absolute paths in lib lists. Run "cmake --help-policy CMP0003" for info
# --------------------------------------------------------------
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
      if (POLICY CMP0037)
		cmake_policy(SET CMP0037 OLD) # allow target "test"
	  endif()
      if (POLICY CMP0026)
		cmake_policy(SET CMP0026 OLD) # allow get target exe location 
	  endif()
endif()
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
ENDIF()

IF(MSVC)
	 add_definitions( "/W3 /D_CRT_SECURE_NO_WARNINGS /wd4267 /wd4244 /nologo" )
ENDIF()

# Try to find Eigen3 for use in the examples
FIND_PACKAGE(PkgConfig QUIET)
MESSAGE(STATUS "Check for Eigen3")
IF(PKG_CONFIG_FOUND)
  PKG_CHECK_MODULES(PKG_EIGEN3 QUIET eigen3) # Try to find eigen3 with pkg-config
ENDIF()
SET(EIGEN3_FOUND FALSE)
FIND_PATH(EIGEN3_INCLUDE NAMES signature_of_eigen3_matrix_library
  PATHS ${CMAKE_INSTALL_PREFIX}/include ${PKG_EIGEN3_INCLUDE_DIRS}
  PATH_SUFFIXES eigen3
)
IF (NOT EXISTS ${EIGEN3_INCLUDE})
  MESSAGE(STATUS "Eigen3 not found in standard location. Try passing -DEIGEN3_INCLUDE=/path/to/eigen3")
ELSE()
  MESSAGE(STATUS "Using Eigen3 in: ${EIGEN3_INCLUDE}")
  INCLUDE_DIRECTORIES("${EIGEN3_INCLUDE}")
  SET(EIGEN3_FOUND TRUE)
ENDIF()

# Include dir:
INCLUDE_DIRECTORIES(${nanoflann_SOURCE_DIR}/include)

# Examples
add_subdirectory(examples)

# Tests
SET( BUILD_TESTING ON CACHE BOOL "Build unit tests")

IF( BUILD_TESTING)
	add_subdirectory(tests)
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
	"${nanoflann_SOURCE_DIR}/scripts/nanoflann.pc.in" 
	"${nanoflann_BINARY_DIR}/nanoflann.pc" @ONLY IMMEDIATE )


# Uninstall target, for "make uninstall"
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/cmake_uninstall.cmake.in"
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

