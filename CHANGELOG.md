# nanoflann 1.5.3: Released Dec 7, 2023
 * **Other changes**:
   - Save one redundant call to `computeMinMax()` in `middleSplit_` ([PR#220](https://github.com/jlblancoc/nanoflann/pull/220) by [qq422216549](https://github.com/qq422216549)).
     This saves *a lot* of time, up to 20% faster in a benchmark with small (thousands) point clouds.

# nanoflann 1.5.2: Released Nov 29, 2023
 * **Other changes**:
   - Improve RKNN search efficiency ([PR#219](https://github.com/jlblancoc/nanoflann/pull/219) by [kya8](https://github.com/kya8)).

# nanoflann 1.5.1: Released Nov 27, 2023
 * **API changes:**
   - Add new search method `rknnSearch()` for knn searches with a maximum radius.
   - Add missing `SearchParameters` argument to `KDTreeSingleIndexDynamicAdaptor_::knnSearch()` ([PR#213](https://github.com/jlblancoc/nanoflann/pull/213) by [ManosPapadakis95](https://github.com/ManosPapadakis95)).
   - Add missing method `KNNResultSet::empty()` for consistency with the other result sets.
 * **Other changes**:
   - Add GUI examples for each search type:
     - `nanoflann_gui_example_R3_knn`
     - `nanoflann_gui_example_R3_radius`
     - `nanoflann_gui_example_R3_rknn`


# nanoflann 1.5.0: Released Jun 16, 2023
 * **API changes:**
   - Users of radius search should change their result placeholder type:
   `std::vector<std::pair<IndexType, DistanceType>>` => `std::vector<nanoflann::ResultItem<IndexType, DistanceType>>`. (See [#166](https://github.com/jlblancoc/nanoflann/issues/166) for the motivation of this change).
   - More concise auxiliary (internal) type name:
     `array_or_vector_selector` -> `array_or_vector`.
   - Remove obsolete parameter `nChecks_IGNORED`. Removed from `SearchParams`
     constructor too, so that structure has been renamed `SearchParameters` to
     enforce users to update the code and avoid mistakes with the order of its
     ctor parameters.
   - Added method RadiusResultSet::empty()
   - Template argument rename: `AccesorType` => `IndexType` (does not actually affect user code at all).
   - Added concurrent tree building support, refer to `KDTreeSingleIndexAdaptorParams::n_thread_build`.
 * **Other changes:**
   - Macros to avoid conflicts with X11 symbols.
   - Inline an auxiliary example function in case users want to use it and
    include the file in multiple translation units (Closes [#182](https://github.com/jlblancoc/nanoflann/issues/182)).
   - Move all benchmarking code, data, and scripts to [its own repository](https://github.com/MRPT/nanoflann-benchmark) to keep this repo as clean as possible.
   - Fix "potentially uninitialized" GCC warning.
   - Clarified, even more, in docs and examples, that L2 distances are **squared** distances.
   - Removed the (with modern compilers) now useless `inline` keyword in class members.
   - Add examples with GUI (requires [mrpt-gui](https://docs.mrpt.org/reference/latest/group_mrpt_gui_grp.html)):
     - nanoflann_gui_example_R3: Radius search on R³ Euclidean space.
     - nanoflann_gui_example_bearings: NN search on non-Euclidean spaces.
 * BUGFIXES:
     - Avoid segfault if saving an empty index (Closes [#205](https://github.com/jlblancoc/nanoflann/issues/205)).

# nanoflann 1.4.3: Released Jul 24, 2022
 * Added flag SkipInitialBuildIndex to allow not wasting time building a tree when it will be loaded from a file later on ([PR #171](https://github.com/jlblancoc/nanoflann/pull/171)).
 * Mark all constructors explicit, to avoid unintended creation of temporary objects ([Issue #179](https://github.com/jlblancoc/nanoflann/issues/179)).
 * BUGFIX: avoid potential index out of bounds in KDTreeSingleIndexDynamicAdaptor ([PR #173](https://github.com/jlblancoc/nanoflann/pull/173))

# nanoflann 1.4.2: Released Jan 11, 2022
 * Install pkg-config .pc file under lib directory (Closes [#161](https://github.com/jlblancoc/nanoflann/issues/161)).
 * Integrate AppVeyor CI.

# nanoflann 1.4.1: Released Jan 6, 2022
  * Fix incorrect install directory for cmake target & config files.
  * Do not install example binaries with `make install`.
  * Provide working examples for cmake and pkgconfig under `examples/example_*` directories.

# nanoflann 1.4.0: Released Jan 2, 2022
  * nanoflann::KDTreeSingleIndexAdaptor() ctor now forwards additional parameters to the metric class, enabling custom dynamic metrics.
  * Add and apply a `.clang-format` file (same one than used in MOLAorg/MOLA projects).
  * Examples: clean up and code modernization.
  * CMake variables prefixed now with `NANOFLANN_` for easier integration of nanoflann as a Git submodule.
  * Fixes for IndexType which are not of integral types [PR #154](https://github.com/jlblancoc/nanoflann/pull/154)
  * save/load API upgraded from C `FILE*` to C++ file streams (By Dominic Kempf, Heidelberg University, [PR](https://github.com/jlblancoc/nanoflann/pull/157)).

# nanoflann 1.3.2: Released Nov 5, 2020
  * Add optional argument for Eigen matrix layout [commit](https://github.com/jlblancoc/nanoflann/commit/40fa96badcfc4b1a2df38b40b8a368cf5521ace4).
  * Throw exception on malloc failure [PR #126](https://github.com/jlblancoc/nanoflann/pull/126).
  * Respect GNUInstallDirs in CMake install rules [PR #131](https://github.com/jlblancoc/nanoflann/pull/131).

# nanoflann 1.3.1: Released Oct 11, 2019
  * Fixed bug in KDTreeSingleIndexDynamicAdaptor. See: https://github.com/jlblancoc/nanoflann/commit/a066148517d16c173954dcde13c1527481b9fad3
  * Fix build in XCode.
  * Simplify CMakeLists for Eigen example (requires Eigen3Config.cmake now)
  * Avoid setting cmake global executable build path

# nanoflann 1.3.0: Released Aug 28, 2018
  * Instructions for `make install` for Linux and Windows (Closes #87).
  * Fix all (?) MSVC conversion warnings (Closes: #95).
  * Avoid need for _USE_MATH_DEFINES in MSVC (Closes: #96)
  * Eigen::Matrix datasets: now uses std::cref() to store a reference to matrix.
  * GSOC2017 contributions by Pranjal Kumar Rai:
    * Support for dynamic datasets.
    * Support for non-Euclidean spaces: SO(2), SO(3)

# nanoflann 1.2.3: Released Dec 20, 2016
  * Fixed: split plane now correctly chooses the dimensions with the largest span.
    Should lead to more optimal trees.

# nanoflann 1.2.2: Released Nov 10, 2016
  * knnSearch() now also returns the number of valid points found.

# nanoflann 1.2.1: Released Jun 1, 2016
  * Fix potential compiler warnings if `IndexType` is signed.
  * New unit tests comparing the results to those of brute force search.

# nanoflann 1.2.0: Released May 5, 2016
  * Fixed: many classes constructors get const ref arguments but stored const values.

# nanoflann 1.1.9: Released Oct 2, 2015
  * Added KDTreeSingleIndexAdaptor::radiusSearchCustomCallback() (Based on a suggestion by Yannick Morin-Rivest)
  * Better documentation in class headers.
  * Cleanup of unused code.
  * Parameter KDTreeSingleIndexAdaptorParams::dim has been removed since it was redundant.

# nanoflann 1.1.8: Released May 2, 2014
  * Created hidden constructors in nanoflann class, to disallow unintentional copies which will corrupt
    the internal pointers.
  * Fixed crash if trying to build an index of an empty dataset.

# nanoflann 1.1.7: Released Aug 24, 2013
  * Two internal containers are now automatically defined as fixed-size arrays if the
    problem dimension is known at compile time, improving efficiency.
    The new/modified datatypes are: KDTreeSingleIndexAdaptor::BoundingBox, KDTreeSingleIndexAdaptor::distance_vector_t
  * Fixed compilation with GCC 4.8 and C++11 enabled (Thanks to Simon Praetorius).

# nanoflann 1.1.6: Released May 14, 2013
  * Fixed warnings about unused parameters.
  * Fixed L1_adaptor.accum_dist(), which implemented L2 instead (Closes #1)
  * Fixed wrong typedef in KDTreeEigenMatrixAdaptor<> for IndexType!=size_t (Closes: #2)

# nanoflann 1.1.5: Released Mar 25, 2013
  * Fixed: Memory pool wasn't freed after each call to buildIndex()
  * GCC: Added -isystem flag to gtest headers to avoid pedantic warnings.

# nanoflann 1.1.4: Released Jan 11, 2013
  * Fixed compilation with Visual Studio 11 (MSVC 2012).
  * Fixed compilation of gtest with VS11 and its _VARIADIC_MAX "bug".
  * Added a security check to launch an exception if searches are attempted before buildIndex().
  * New example to demonstrate save/load the index to files.
  * save/load methods exposed as public.

# nanoflann 1.1.3: Released Jun 6, 2012
  * GTest sources are now embedded, due to the changes in newer Ubuntu packages which don't carry the precompiled libs.
  * Added asserts to detect whether the user passes NULL as query points.
  * New method RadiusResultSet::worst_item()
  * New method RadiusResultSet::set_radius_and_clear()
  * Avoid potential collision of min/max macros with <windows.h>
  * Removed unneeded #include's of std headers.
  * New sample code for vectors of vectors.
  * Fixed building of tests for MSVC in Windows.
  * Allow manually setting the path to Eigen3 (mainly for building examples under Windows).

# nanoflann 1.1.2: Released May 2, 2012
  * Better documentation and added graphs of a benchmarking for helping choosing "leaf_max_size".
  * Now KDTreeSingleIndexAdaptor::buildIndex() can be called several times
     even when the dataset size changes (Thanks to Rob McDonald for reporting!)

# nanoflann 1.1.1: Released Feb 1, 2012
  * Some fixes to kd_tree index and L1/L2 metrics to allow distinct types
     in data elements and in the distances. This is mainly to permit elements
	 being vectors of integers (e.g. uint8_t) but distances being real numbers.
  * Examples and unit tests have been corrected to use template arguments
     instead of being hard-wired to "float" data types (Thanks Thomas Vincent
	 for noticing!).

# nanoflann 1.1.0: Released Dec 15, 2011
  * Fixed warnings for MSVC and for GCC with "-Wall -pedantic"
  * Updated performance tests to work with the final nanoflann code (they were
     written for a very early version).
  * All main classes now have new template arguments for the type of indice,
     which now defaults to "size_t" instead of "int". In case this breaks
     backward compatibility in user code, especify "int" to override the default
     template arguments, although "size_t" it's recommended.

# nanoflann 1.0.0: Released Aug 30, 2011
  * Initial version
