# ----------------------------------------------------------------------------
#   TESTS
# ----------------------------------------------------------------------------
PROJECT(tests)

# Fix a "bug" in VS11 (MSVC 2012):
if(MSVC)
	add_definitions(-D_VARIADIC_MAX=10)
endif(MSVC)

if(NOT TARGET gtest)
	# Treat this directory as "system" to ignore pedantic warnings.
	if(CMAKE_COMPILER_IS_GNUCXX)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem gtest-1.8.0/include")
	endif()

	# Compile gtest from sources:
	add_subdirectory(gtest-1.8.0)
	INCLUDE_DIRECTORIES("gtest-1.8.0/include/")
endif()

set(EXECUTABLE_OUTPUT_PATH "${${PROJECT_NAME}_BINARY_DIR}")

# Tests based on Google gtest:
# -----------------------------

# If using system library, add C++ flags:
if (UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

# Test project:
ADD_EXECUTABLE(unit_tests test_main.cpp)
set_target_properties(unit_tests PROPERTIES FOLDER "./")
ADD_TEST(unit_tests_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target unit_tests)
ADD_TEST(unit_tests_run ${EXECUTABLE_OUTPUT_PATH}/unit_tests)
set_tests_properties(unit_tests_run PROPERTIES DEPENDS unit_tests_build)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(unit_tests mygtest nanoflann)
