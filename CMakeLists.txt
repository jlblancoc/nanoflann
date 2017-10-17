# ----------------------------------------------------------------------------
# Root CMake file for nanoflann
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.8.12)

PROJECT(nanoflann)

cmake_policy(SET CMP0003 NEW) # dont mix relative and absolute paths in lib lists
if (POLICY CMP0037)
	cmake_policy(SET CMP0037 OLD) # allow target "test"
endif()

# Enable a high level of warnings.
if (CMAKE_COMPILER_IS_GNUCXX)
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
	add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
	# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
	if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
		add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
	endif()
endif()

# Solution Folder options:
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
      if (POLICY CMP0037)
		cmake_policy(SET CMP0037 OLD) # allow target "test"
	  endif()
endif()
SET( EXECUTABLE_OUTPUT_PATH ${nanoflann_BINARY_DIR}/bin CACHE PATH "Output directory for programs" )
add_definitions ( -DNANOFLANN_PATH="${CMAKE_SOURCE_DIR}" )

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

# Compiler options:
IF(CMAKE_COMPILER_IS_GNUCXX)
	 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -mtune=native -Wall")
ENDIF()

IF(MSVC)
	 add_definitions( "/W3 /D_CRT_SECURE_NO_WARNINGS /wd4267 /wd4244 /nologo" )
ENDIF()

# Define nanoflann lib (header-only)
add_library(nanoflann INTERFACE)

target_include_directories(nanoflann
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>/include)

# Examples
OPTION(BUILD_EXAMPLES "Build examples" ON)
IF(BUILD_EXAMPLES)
	add_subdirectory(examples)
ENDIF()

# Benchmarks
OPTION(BUILD_BENCHMARKS "Build benchmarks" OFF)
IF(BUILD_BENCHMARKS)
        # 3rdparty Libraries
        include(3rdparty/CMakeLists-flann.txt)
        include(3rdparty/CMakeLists-fastann.txt)
        include(3rdparty/CMakeLists-libkdtree.txt)
	add_subdirectory(benchmarkTool/realTests)
	add_subdirectory(benchmarkTool/randomTests)
ENDIF()

# Tests
OPTION(BUILD_TESTS "Build unit tests" ON)
IF(BUILD_TESTS)
	add_subdirectory(tests)
ENDIF()

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



# Set relative install directories
set(INSTALL_INCLUDE_DIR "include")
set(INSTALL_PKGCONFIG_DIR "lib${LIB_SUFFIX}/pkgconfig")
set(INSTALL_CMAKE_DIR "lib${LIB_SUFFIX}/cmake/nanoflann")
set(INSTALL_COPYRIGHT_DIR "share/doc/libnanoflann-dev")


# Generate the cmake config and cmake config-version file:
SET(RELATIVE_INCLUDE_DIR "../../../include")
SET(CONFIG_INCLUDE_DIRS "\${nanoflann_CMAKE_DIR}/${RELATIVE_INCLUDE_DIR}")

CONFIGURE_FILE(
    "${nanoflann_SOURCE_DIR}/scripts/nanoflannConfig.cmake.in"
    "${nanoflann_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/nanoflannConfig.cmake" @ONLY IMMEDIATE )

CONFIGURE_FILE(
    "${nanoflann_SOURCE_DIR}/scripts/nanoflannConfigVersion.cmake.in"
    "${nanoflann_BINARY_DIR}/nanoflannConfigVersion.cmake" @ONLY IMMEDIATE )


# Uninstall target, for "make uninstall"
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  @ONLY IMMEDIATE)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")


INSTALL(
	FILES "${nanoflann_BINARY_DIR}/nanoflann.pc"
	DESTINATION "${libnanoflann_dev_INSTALL_PREFIX}${INSTALL_PKGCONFIG_DIR}" )

INSTALL(
    FILES "${nanoflann_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/nanoflannConfig.cmake"
          "${nanoflann_BINARY_DIR}/nanoflannConfigVersion.cmake"
    DESTINATION "${libnanoflann_dev_INSTALL_PREFIX}${INSTALL_CMAKE_DIR}" )

INSTALL(
	FILES "${nanoflann_SOURCE_DIR}/include/nanoflann.hpp"
	DESTINATION "${libnanoflann_dev_INSTALL_PREFIX}${INSTALL_INCLUDE_DIR}" )

IF(CMAKE_USE_DEB_POSTFIXS)
	INSTALL(
		FILES "${nanoflann_SOURCE_DIR}/copyright"
		DESTINATION "${libnanoflann_dev_INSTALL_PREFIX}${INSTALL_COPYRIGHT_DIR}" )
ENDIF(CMAKE_USE_DEB_POSTFIXS)
