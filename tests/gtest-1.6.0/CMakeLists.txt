# ----------------------------------------------------------------------------
#  Auxiliary static library: gtest (Google test framework)
#   Used for Unit Testing
# ----------------------------------------------------------------------------

PROJECT(gtest)

INCLUDE_DIRECTORIES("fused-src")

ADD_LIBRARY(gtest 
	STATIC
	fused-src/gtest/gtest-all.cc 
	)

IF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)

