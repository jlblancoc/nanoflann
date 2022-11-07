This directory contains example programs that depend on mrpt-gui to show 
a live GUI visualization of found matches by nanoflann.

Each subdirectory can be compiled as an independent project by setting CMake SOURCE_DIR to it,
but they will be also compiled as part of the default "all" target of the nanoflann
project, if all dependencies are found; otherwise, they will be silently skipped.
