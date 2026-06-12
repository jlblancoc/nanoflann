# Fix a "bug" in VS11 (MSVC 2012):
if(MSVC)
    add_compile_definitions(_VARIADIC_MAX=10)
endif()

# Find or build GTest
if(NANOFLANN_USE_SYSTEM_GTEST)
    find_package(GTest REQUIRED)
    set(GTEST_TARGET GTest::gtest)
else()
    # EXCLUDE_FROM_ALL prevents gtest from being installed with your project
    add_subdirectory(gtest-1.8.0 EXCLUDE_FROM_ALL)

    set(GTEST_TARGET mygtest)

    # Mark local GTest headers as SYSTEM to suppress pedantic warnings from them
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_include_directories(${GTEST_TARGET} SYSTEM INTERFACE "gtest-1.8.0/include")
    else()
        target_include_directories(${GTEST_TARGET} INTERFACE "gtest-1.8.0/include")
    endif()
endif()

# Test executable
add_executable(nanoflann_unit_tests
  test_kdtree_basic.cpp
  test_kdtree_dynamic.cpp
  test_kdtree_incremental.cpp
  test_kdtree_manifold.cpp
  test_kdtree_incremental_manifold.cpp
  test_main.cpp
)
set_target_properties(nanoflann_unit_tests PROPERTIES FOLDER "./")

if(MSVC)
    target_compile_options(nanoflann_unit_tests PRIVATE /W4)
else()
    target_compile_options(nanoflann_unit_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wshadow
    )
endif()

find_package(Threads REQUIRED)

target_link_libraries(nanoflann_unit_tests
    PRIVATE
        ${GTEST_TARGET}
        nanoflann::nanoflann
        Threads::Threads
)

add_test(NAME nanoflann_unit_tests COMMAND nanoflann_unit_tests)