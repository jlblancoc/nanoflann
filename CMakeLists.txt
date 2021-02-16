# ----------------------------------------------------------------------------
# Root CMake file for nanoflann
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.1)

# Extract library version into "NANOFLANN_VERSION"
# -----------------------------------------------------
# Look for: "#define NANOFLANN_VERSION 0xABC"
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/include/nanoflann.hpp" STR_HPP)
string(REGEX MATCHALL "NANOFLANN_VERSION.*0x[0-9,A-F]+" CMAKE_VERSION_LINE "${STR_HPP}")
string(REGEX MATCHALL "0x[0-9,A-F]+" NANOFLANN_VERSION_HEX "${CMAKE_VERSION_LINE}")

string(REGEX REPLACE "0x(.).*" "\\1" NANOFLANN_VERSION_MAJOR "${NANOFLANN_VERSION_HEX}" )
string(REGEX REPLACE "0x.(.).*" "\\1" NANOFLANN_VERSION_MINOR "${NANOFLANN_VERSION_HEX}" )
string(REGEX REPLACE "0x..(.).*" "\\1" NANOFLANN_VERSION_PATCH "${NANOFLANN_VERSION_HEX}" )
mark_as_advanced(STR_HPP CMAKE_VERSION_LINE NANOFLANN_VERSION_HEX NANOFLANN_VERSION_MAJOR NANOFLANN_VERSION_MINOR NANOFLANN_VERSION_PATCH)

project(nanoflann VERSION "${NANOFLANN_VERSION_MAJOR}.${NANOFLANN_VERSION_MINOR}.${NANOFLANN_VERSION_PATCH}")

message(STATUS "nanoflann version: ${NANOFLANN_VERSION_MAJOR}.${NANOFLANN_VERSION_MINOR}.${NANOFLANN_VERSION_PATCH}")
file(WRITE "${nanoflann_BINARY_DIR}/version" "${NANOFLANN_VERSION_MAJOR}.${NANOFLANN_VERSION_MINOR}.${NANOFLANN_VERSION_PATCH}")

# Enable a high level of warnings.
if (CMAKE_COMPILER_IS_GNUCXX)
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
	add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros -O2 -mtune=native)
	# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
	if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
		add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
	endif()
endif()

if(MSVC)
	 add_definitions( "/W3 /D_CRT_SECURE_NO_WARNINGS /nologo" )
endif()

# Solution Folder options:
if (${CMAKE_SOURCE_DIR} STREQUAL ${PROJECT_SOURCE_DIR})
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
	set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
endif()

add_definitions ( -DNANOFLANN_PATH="${CMAKE_SOURCE_DIR}" )

include(GNUInstallDirs)
# Set relative install directories
set(INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_INCLUDEDIR}")
set(INSTALL_PKGCONFIG_DIR "${CMAKE_INSTALL_DATADIR}/pkgconfig")
set(INSTALL_CMAKE_DIR "${CMAKE_INSTALL_DATADIR}/cmake/${PROJECT_NAME}")
set(INSTALL_COPYRIGHT_DIR "${CMAKE_INSTALL_DOCDIR}")


# Define nanoflann lib (header-only)
add_library(nanoflann INTERFACE)

# Tell CMake which C++ features we need
target_compile_features(nanoflann
	INTERFACE
		cxx_auto_type
		cxx_decltype
		cxx_deleted_functions
)
target_include_directories(nanoflann
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>)

install(TARGETS nanoflann
        EXPORT nanoflannTargets)

add_library(nanoflann::nanoflann ALIAS nanoflann)

# Examples
option(NANOFLANN_BUILD_EXAMPLES "Build examples" ON)
if(NANOFLANN_BUILD_EXAMPLES)
	add_subdirectory(examples)
endif()

# Benchmarks
option(NANOFLANN_BUILD_BENCHMARKS "Build benchmarks" OFF)
if(NANOFLANN_BUILD_BENCHMARKS)
        # 3rdparty Libraries
        include(3rdparty/CMakeLists-flann.txt)
        include(3rdparty/CMakeLists-fastann.txt)
        include(3rdparty/CMakeLists-libkdtree.txt)
	add_subdirectory(benchmarkTool)
endif()

# Tests
option(NANOFLANN_BUILD_TESTS "Build unit tests" ON)
if(NANOFLANN_BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif()

# --------------------------------------------------------------------
# Install/uninstall targets
# --------------------------------------------------------------------

#--------------------------------------------------------------
# If we are building the final step of the Debian package,
#  save each library files in the corresponding directories:
#--------------------------------------------------------------
if(CMAKE_USE_DEB_POSTFIXS)
	# Values when building a Debian package ---------------
	message(STATUS "** Using Debian post-fix for install directories **")
	set(libnanoflann_dev_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/libnanoflann-dev/usr/")
	set(nanoflann_pkgconfig_INSTALL_PREFIX "/usr")	# Values when building a Debian package
else()
	# Values under normal conditions -----------------------
	set(libnanoflann_dev_INSTALL_PREFIX "")
	set(nanoflann_pkgconfig_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}") 	# Values under normal conditions
endif()

# Generate the pkg-config file:
configure_file(
	"${nanoflann_SOURCE_DIR}/scripts/nanoflann.pc.in"
	"${nanoflann_BINARY_DIR}/nanoflann.pc" @ONLY IMMEDIATE )



# Generate the cmake config and cmake config-version file:
include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${nanoflann_SOURCE_DIR}/scripts/nanoflannConfig.cmake.in"
    "${nanoflann_BINARY_DIR}/nanoflannConfig.cmake"
    INSTALL_DESTINATION ${INSTALL_CMAKE_DIR}
    PATH_VARS INSTALL_INCLUDE_DIR)

# Setting CMAKE_SIZEOF_VOID_P to the empty string has the same
# effect as the ARCH_INDEPENDENT option of
# write_basic_package_version_file(), but works with older CMake
# versions before 3.14
set(backup_of_CMAKE_SIZEOF_VOID_P "${CMAKE_SIZEOF_VOID_P}")
set(CMAKE_SIZEOF_VOID_P "")

write_basic_package_version_file(
    "${nanoflann_BINARY_DIR}/nanoflannConfigVersion.cmake"
    VERSION ${nanoflann_VERSION}
    COMPATIBILITY AnyNewerVersion)

set(CMAKE_SIZEOF_VOID_P "${backup_of_CMAKE_SIZEOF_VOID_P}")

# Uninstall target, for "make uninstall"
configure_file(
  "${nanoflann_SOURCE_DIR}/scripts/cmake_uninstall.cmake.in"
  "${nanoflann_BINARY_DIR}/cmake_uninstall.cmake"
  @ONLY IMMEDIATE)

if (NOT TARGET uninstall)
  add_custom_target(uninstall
    "${CMAKE_COMMAND}" -P "${nanoflann_BINARY_DIR}/cmake_uninstall.cmake")
else()
  add_custom_target(nanoflann_uninstall
    "${CMAKE_COMMAND}" -P "${nanoflann_BINARY_DIR}/cmake_uninstall.cmake")
  add_dependencies(uninstall nanoflann_uninstall)
endif()

export(EXPORT nanoflannTargets
       NAMESPACE nanoflann::
       FILE "${nanoflann_BINARY_DIR}/nanoflannTargets.cmake")

export(PACKAGE nanoflann)

install(EXPORT nanoflannTargets
        NAMESPACE nanoflann::
        DESTINATION "${libnanoflann_dev_INSTALL_PREFIX}${INSTALL_CMAKE_DIR}")

install(
	FILES "${nanoflann_BINARY_DIR}/nanoflann.pc"
	DESTINATION "${libnanoflann_dev_INSTALL_PREFIX}${INSTALL_PKGCONFIG_DIR}" )

install(
    FILES "${nanoflann_BINARY_DIR}/nanoflannConfig.cmake"
          "${nanoflann_BINARY_DIR}/nanoflannConfigVersion.cmake"
    DESTINATION "${libnanoflann_dev_INSTALL_PREFIX}${INSTALL_CMAKE_DIR}" )

install(
	FILES "${nanoflann_SOURCE_DIR}/include/nanoflann.hpp"
	DESTINATION "${libnanoflann_dev_INSTALL_PREFIX}${INSTALL_INCLUDE_DIR}" )

if(CMAKE_USE_DEB_POSTFIXS)
	install(
		FILES "${nanoflann_SOURCE_DIR}/copyright"
		DESTINATION "${libnanoflann_dev_INSTALL_PREFIX}${INSTALL_COPYRIGHT_DIR}" )
endif()
