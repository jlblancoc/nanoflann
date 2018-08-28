# ----------------------------------------------------------------------------
#  Auxiliary static library: gtest (Google test framework)
# ----------------------------------------------------------------------------
project(mygtest)

# Fix a "bug" in VS11 (MSVC 2012):
if(MSVC)
	add_definitions(-D_VARIADIC_MAX=10)
endif(MSVC)

include_directories("include")
include_directories(".")

add_library(mygtest STATIC src/gtest-all.cc)

if(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
	add_compile_options("-fPIC")
endif()

set_target_properties(mygtest PROPERTIES FOLDER "3rd party")
