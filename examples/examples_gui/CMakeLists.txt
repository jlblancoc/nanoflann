# optional for gui examples:
find_package(mrpt-gui QUIET)
if (NOT mrpt-gui_FOUND)
    return()
endif()

add_subdirectory(nanoflann_gui_example_R3)
add_subdirectory(nanoflann_gui_example_SO2)
add_subdirectory(nanoflann_gui_example_bearings)
