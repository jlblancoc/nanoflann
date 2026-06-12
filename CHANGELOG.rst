^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nanoflann
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (in progress)
-------------------
* New feature: **compile-time product-manifold topology** for *exact* nearest-neighbor
  search on non-Euclidean state spaces. Declare the
  search space as a product of base manifolds at compile time:
  ``nanoflann::Rn<N>``, ``nanoflann::SO2``, ``nanoflann::SO3``, ``nanoflann::SE2``,
  ``nanoflann::SE3``, ``nanoflann::Torus<N>``, the unit sphere
  ``nanoflann::Sn<N>`` / ``nanoflann::S2`` (3D directions / bearings as unit
  vectors, chordal metric, no double cover), and arbitrary
  ``nanoflann::Product<...>`` (e.g. ``Product<Rn<3>, S2>`` for oriented points).
  The synthesized ``nanoflann::Manifold_Adaptor<Space, T, Dataset>`` metric keeps the
  KD-tree pruning admissible (wrap-aware for circular coordinates, antipodal
  double-cover-aware for unit-quaternion blocks), so results are identical to an
  exhaustive geodesic brute-force search. Helpers ``chord_sq_to_angle`` /
  ``angle_to_chord_sq`` convert between chordal-squared distances and geodesic
  angles for S^N and SO(3). Both the static ``KDTreeSingleIndexAdaptor`` and the
  incremental ``KDTreeSingleIndexIncrementalAdaptor`` (plus its multi-threaded
  wrapper) support these metrics; the incremental index stays exact under
  insert/remove churn and background rebuilds, reusing its per-subtree bounding
  boxes as the admissible region supersets. Box-region deletes
  (``removeBox`` / ``removeOutsideBox``) operate on raw coordinates and do not
  wrap. Requires C++17; the legacy C++11 metrics
  and the Euclidean code path are completely unaffected (opt out with
  ``NANOFLANN_NO_MANIFOLDS``). See ``examples/manifold_se3_example.cpp``,
  ``examples/manifold_s2_example.cpp``, and
  ``examples/manifold_se3_incremental_example.cpp``.
* **Removed (breaking)**: the legacy ``SO2_Adaptor`` / ``metric_SO2`` and
  ``SO3_Adaptor`` / ``metric_SO3`` metrics. They are superseded by, and were
  approximate/incorrect compared to, the new exact manifold metrics: the legacy
  SO(2) metric only adjusted the per-coordinate distance while the KD-tree
  pruning stayed Euclidean, so searches near the +/-pi seam could over-prune the
  true neighbor; the legacy SO(3) metric was plain L2 on 4 coordinates and
  ignored the antipodal double cover (treating q and -q, the same rotation, as
  far apart). Migration (requires C++17): ``metric_SO2`` ->
  ``metric_Manifold<SO2>`` (or ``Manifold_Adaptor<SO2, ...>``); ``metric_SO3`` ->
  ``metric_Manifold<SO3>`` (or ``Manifold_Adaptor<SO3, ...>``). Note SO(3)
  distances are now chordal-squared and double-cover-aware. C++11 users needing
  the old behavior should pin nanoflann 1.x. The ported examples are
  ``examples/manifold_so2_example.cpp`` and ``examples/manifold_so3_example.cpp``.

1.11.0 (2026-07-31)
-------------------
* Merge pull request `#309 <https://github.com/jlblancoc/nanoflann/issues/309>`_ from jlblancoc/feat/incremental-index-save-load
  Add saveIndex()/loadIndex() to the incremental k-d tree index
* Tests: cover loadIndex()'s remaining validation branches
  Version mismatch, type-size mismatch, dimensionality mismatch, and
  truncated/EOF node data, each isolated by corrupting exactly the field the
  corresponding check inspects. Addresses the patch-coverage gaps left by the
  previous commit.
* Add saveIndex()/loadIndex() to the incremental k-d tree index
  KDTreeSingleIndexIncrementalAdaptor could not previously be persisted:
  it never populates the base class's root_node\_/vAcc\_, so the inherited
  saveIndex()/loadIndex() were silent no-ops for it.
  Serialize the tree topology explicitly, field by field, instead of a
  raw struct byte-dump (safe regardless of DIM being compile-time fixed
  or runtime): per node, only ptIdx, divfeat, deleted, treeDeleted and
  child-presence bits are written. Bounding box, subtree_size,
  invalid_count, the coordinate cache and parent links are all
  structural invariants, so they are recomputed on load instead of
  trusted from the file. Uses a magic number distinct from the static
  index's SAVE_MAGIC so loading the wrong kind of file fails fast.
  The multithreaded KDTreeSingleIndexIncrementalAdaptorMT variant gets
  forwarding saveIndex()/loadIndex() that block on any in-flight
  background rebuild first.
* Merge pull request `#308 <https://github.com/jlblancoc/nanoflann/issues/308>`_ from icyveins7/master
  Fix: correct SearchParams->SearchParameters and other data types in examples/example_with_cmake
* Apply clang-format-14 to example_with_cmake fix
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
* fix: correct SearchParams->SearchParameters and other data types in examples/example_with_cmake
  was probably missed when they changed type names in a4fac39
* Update README with new build status badges
* Fix arm64 badge link for ROS 2 Lyrical
* docs: fix rolling URIs
* docs: extend badge tables
* Contributors: Jose Luis Blanco-Claraco, icyveins7

1.10.1 (2026-06-09)
-------------------
* Merge pull request `#303 <https://github.com/jlblancoc/nanoflann/issues/303>`_ from jlblancoc/refactor/tests-smaller-files
  refactor: unit tests into smaller files
* refactor: unit tests into smaller files
* CHANGELOG: ported to rst format for ROS tools compatibility
* doc: update outdated COPYING dates
* docs: add readme build badges
* docs: add ROS build farm badge table to README
* Add ROS package.xml and integrate BUILD_TESTING for ROS build farm
  - package.xml: plain cmake build_type, BSD-2-Clause, libgtest-dev test_depend
  - CMakeLists.txt: include(CTest), use BUILD_TESTING guard, auto-select
  system GTest when ROS_VERSION env is set
  - tests/CMakeLists.txt: remove redundant project(), rename test target to
  nanoflann_unit_tests to avoid collisions in workspaces
* Contributors: Jose Luis Blanco-Claraco

1.10.0 (2026-06-06)
--------------------
* New: Incremental self-balancing dynamic k-d tree index (``KDTreeSingleIndexIncrementalAdaptor``), both single-threaded and multithreaded variants (`#302 <https://github.com/jlblancoc/nanoflann/pull/302>`_).
* Performance: Hoist point-length computation out of search inner loops (`#301 <https://github.com/jlblancoc/nanoflann/pull/301>`_).
* Refactor: Deduplicate tree builders, harden Eigen adaptor ownership, centralize DIM dispatch with ``if constexpr`` (`#297 <https://github.com/jlblancoc/nanoflann/pull/297>`_, `#299 <https://github.com/jlblancoc/nanoflann/pull/299>`_, `#300 <https://github.com/jlblancoc/nanoflann/pull/300>`_).
* Fix: ``SO2_Adaptor`` now returns absolute angular distance instead of signed (`#298 <https://github.com/jlblancoc/nanoflann/pull/298>`_).
* Fix: ``saveIndex``/``loadIndex`` now include a magic+version+type-size header and throw on mismatch (`#295 <https://github.com/jlblancoc/nanoflann/pull/295>`_).
* Fix: ``KDTreeSingleIndexDynamicAdaptor_`` saveIndex/loadIndex infinite recursion (`#290 <https://github.com/jlblancoc/nanoflann/pull/290>`_).
* Fix: loadIndex reading past EOF for empty indices (`#287 <https://github.com/jlblancoc/nanoflann/pull/287>`_).
* Fix: duplicate-entry growth on removePoint/addPoints re-add cycles (`#285 <https://github.com/jlblancoc/nanoflann/pull/285>`_).
* Fix: divideTree assertion failure with dynamic adaptor removePoint/addPoints (`#283 <https://github.com/jlblancoc/nanoflann/pull/283>`_).
* CI: Added Windows (MSVC + clang-cl) and macOS (14+15) workflows; extended tested C++ standards.
* Docs: Consolidated macro list and documented threading guarantees.
* Contributors: Jose Luis Blanco-Claraco

1.8.0 (2025-11-16)
-------------------
* Performance: Minor performance optimization changes (`#271 <https://github.com/jlblancoc/nanoflann/issues/271>`_).
* New API: ``findWithinBox()`` (`#272 <https://github.com/jlblancoc/nanoflann/pull/272>`_).
* Contributors: Jose Luis Blanco-Claraco

1.7.1 (2025-03-15)
-------------------
* Fix: ``ResultSets::worstDist()``: fix a potential access to negative index in array (did not happen in practice, but static analysis tools correctly detected this possibility).
* Contributors: Jose Luis Blanco-Claraco

1.7.0 (2025-02-03)
-------------------
* API: ``ResultSets::worstDist()``: clarified the meaning of its return value; now returns the actual worst distance in the found set (only if set is full).
* Contributors: Jose Luis Blanco-Claraco

1.6.3 (2025-01-07)
-------------------
* cmake_required_version bumped to 3.10.
* clang-format version bumped to 14.
* Contributors: Jose Luis Blanco-Claraco

1.6.2 (2024-11-04)
-------------------
* Fix: ``middleSplit_`` for same points (`#250 <https://github.com/jlblancoc/nanoflann/pull/250>`_).
* Fix: Build warnings.
* Contributors: Jose Luis Blanco-Claraco, yzabalotski

1.6.1 (2024-08-24)
-------------------
* Add conan install instructions.
* Add multiple thread kdtree build support for ``KDTreeEigenMatrixAdaptor`` (`#246 <https://github.com/jlblancoc/nanoflann/pull/246>`_).
* Contributors: Jose Luis Blanco-Claraco

1.6.0 (2024-07-11)
-------------------
* Fix: ``nanoflann::SearchParameters::sorted`` was ignored for ``RadiusResultSet``.
* API: ``ResultSet`` classes now must implement a ``sort()`` method.
* API: Added type ``IndexType`` to ``nanoflann::KDTreeBaseClass``.
* Contributors: Jose Luis Blanco-Claraco

1.5.5 (2024-03-12)
-------------------
* Performance: Potentially more efficient scheduling of multi-thread index building (`#236 <https://github.com/jlblancoc/nanoflann/pull/236>`_).
* Bump minimum required cmake version to 3.5 (`#230 <https://github.com/jlblancoc/nanoflann/pull/230>`_).
* Contributors: Jose Luis Blanco-Claraco

1.5.4 (2024-01-10)
-------------------
* Fix outdated ``NANOFLANN_VERSION`` macro in header file.
* Fix pool-allocator alignment problems.
* Add ``NANOFLANN_USE_SYSTEM_GTEST`` option.
* Look for Threads dependency in CMake config script.
* Contributors: Jose Luis Blanco-Claraco

1.5.3 (2023-12-07)
-------------------
* Performance: Save one redundant call to ``computeMinMax()`` in ``middleSplit_`` (`#220 <https://github.com/jlblancoc/nanoflann/pull/220>`_). Saves up to 20% build time in benchmarks with small (thousands) point clouds.
* Contributors: Jose Luis Blanco-Claraco, qq422216549

1.5.2 (2023-11-29)
-------------------
* Performance: Improve RKNN search efficiency (`#219 <https://github.com/jlblancoc/nanoflann/pull/219>`_).
* Contributors: Jose Luis Blanco-Claraco, kya8

1.5.1 (2023-11-27)
-------------------
* New API: ``rknnSearch()`` for knn searches with a maximum radius.
* Fix: Add missing ``SearchParameters`` argument to ``KDTreeSingleIndexDynamicAdaptor_::knnSearch()`` (`#213 <https://github.com/jlblancoc/nanoflann/pull/213>`_).
* Fix: Add missing method ``KNNResultSet::empty()`` for consistency with other result sets.
* Add GUI examples for each search type: ``nanoflann_gui_example_R3_knn``, ``nanoflann_gui_example_R3_radius``, ``nanoflann_gui_example_R3_rknn``.
* Contributors: Jose Luis Blanco-Claraco, ManosPapadakis95

1.5.0 (2023-06-16)
-------------------
* API: Users of radius search should update result placeholder type from ``std::vector<std::pair<IndexType, DistanceType>>`` to ``std::vector<nanoflann::ResultItem<IndexType, DistanceType>>``.
* API: More concise auxiliary type name: ``array_or_vector_selector`` -> ``array_or_vector``.
* API: Remove obsolete parameter ``nChecks_IGNORED``; ``SearchParams`` renamed to ``SearchParameters``.
* API: Added method ``RadiusResultSet::empty()``.
* API: Template argument rename: ``AccesorType`` => ``IndexType``.
* API: Added concurrent tree building support via ``KDTreeSingleIndexAdaptorParams::n_thread_build``.
* Macros to avoid conflicts with X11 symbols.
* Inline auxiliary example function to support inclusion in multiple translation units.
* Move all benchmarking code and scripts to a `separate repository <https://github.com/MRPT/nanoflann-benchmark>`_.
* Fix: Potential uninitialized variable GCC warning.
* Docs: Clarified that L2 distances are squared distances.
* Add GUI examples: ``nanoflann_gui_example_R3`` and ``nanoflann_gui_example_bearings``.
* Fix: Avoid segfault when saving an empty index.
* Contributors: Jose Luis Blanco-Claraco

1.4.3 (2022-07-24)
-------------------
* Add flag ``SkipInitialBuildIndex`` to skip tree building when the index will be loaded from a file (`#171 <https://github.com/jlblancoc/nanoflann/pull/171>`_).
* Mark all constructors explicit to avoid unintended creation of temporary objects.
* Fix: Potential index out of bounds in ``KDTreeSingleIndexDynamicAdaptor`` (`#173 <https://github.com/jlblancoc/nanoflann/pull/173>`_).
* Contributors: Jose Luis Blanco-Claraco

1.4.2 (2022-01-11)
-------------------
* Install pkg-config ``.pc`` file under lib directory.
* Integrate AppVeyor CI.
* Contributors: Jose Luis Blanco-Claraco

1.4.1 (2022-01-06)
-------------------
* Fix incorrect install directory for cmake target & config files.
* Do not install example binaries with ``make install``.
* Provide working examples for cmake and pkgconfig under ``examples/example_*`` directories.
* Contributors: Jose Luis Blanco-Claraco

1.4.0 (2022-01-02)
-------------------
* API: ``nanoflann::KDTreeSingleIndexAdaptor()`` ctor now forwards additional parameters to the metric class, enabling custom dynamic metrics.
* Add and apply a ``.clang-format`` file.
* Examples: Clean up and code modernization.
* CMake variables now prefixed with ``NANOFLANN_`` for easier Git submodule integration.
* Fix: ``IndexType`` for non-integral types (`#154 <https://github.com/jlblancoc/nanoflann/pull/154>`_).
* API: save/load upgraded from C ``FILE*`` to C++ file streams (`#157 <https://github.com/jlblancoc/nanoflann/pull/157>`_).
* Contributors: Jose Luis Blanco-Claraco, Dominic Kempf

1.3.2 (2020-11-05)
-------------------
* Add optional argument for Eigen matrix layout.
* Throw exception on malloc failure (`#126 <https://github.com/jlblancoc/nanoflann/pull/126>`_).
* Respect GNUInstallDirs in CMake install rules (`#131 <https://github.com/jlblancoc/nanoflann/pull/131>`_).
* Contributors: Jose Luis Blanco-Claraco

1.3.1 (2019-10-11)
-------------------
* Fix bug in ``KDTreeSingleIndexDynamicAdaptor``.
* Fix build in XCode.
* Simplify CMakeLists for Eigen example (requires Eigen3Config.cmake now).
* Avoid setting cmake global executable build path.
* Contributors: Jose Luis Blanco-Claraco

1.3.0 (2018-08-28)
-------------------
* Add ``make install`` instructions for Linux and Windows.
* Fix all MSVC conversion warnings.
* Avoid need for ``_USE_MATH_DEFINES`` in MSVC.
* Eigen::Matrix datasets: now uses ``std::cref()`` to store a reference to matrix.
* GSOC2017: Support for dynamic datasets.
* GSOC2017: Support for non-Euclidean spaces: SO(2), SO(3).
* Contributors: Jose Luis Blanco-Claraco, Pranjal Kumar Rai

1.2.3 (2016-12-20)
-------------------
* Fix: Split plane now correctly chooses the dimension with the largest span, leading to more optimal trees.
* Contributors: Jose Luis Blanco-Claraco

1.2.2 (2016-11-10)
-------------------
* ``knnSearch()`` now also returns the number of valid points found.
* Contributors: Jose Luis Blanco-Claraco

1.2.1 (2016-06-01)
-------------------
* Fix potential compiler warnings if ``IndexType`` is signed.
* New unit tests comparing results against brute-force search.
* Contributors: Jose Luis Blanco-Claraco

1.2.0 (2016-05-05)
-------------------
* Fix: Many class constructors were accepting const ref arguments but storing const values.
* Contributors: Jose Luis Blanco-Claraco

1.1.9 (2015-10-02)
-------------------
* New: ``KDTreeSingleIndexAdaptor::radiusSearchCustomCallback()``.
* Better documentation in class headers.
* Cleanup of unused code.
* Remove redundant parameter ``KDTreeSingleIndexAdaptorParams::dim``.
* Contributors: Jose Luis Blanco-Claraco

1.1.8 (2014-05-02)
-------------------
* Create hidden constructors in nanoflann class to disallow unintentional copies that corrupt internal pointers.
* Fix crash when building an index from an empty dataset.
* Contributors: Jose Luis Blanco-Claraco

1.1.7 (2013-08-24)
-------------------
* Two internal containers now automatically use fixed-size arrays when problem dimension is known at compile time, improving efficiency (``KDTreeSingleIndexAdaptor::BoundingBox``, ``KDTreeSingleIndexAdaptor::distance_vector_t``).
* Fix compilation with GCC 4.8 and C++11 enabled.
* Contributors: Jose Luis Blanco-Claraco, Simon Praetorius

1.1.6 (2013-05-14)
-------------------
* Fix warnings about unused parameters.
* Fix ``L1_adaptor.accum_dist()``, which was implementing L2 instead of L1.
* Fix wrong typedef in ``KDTreeEigenMatrixAdaptor<>`` for ``IndexType!=size_t``.
* Contributors: Jose Luis Blanco-Claraco

1.1.5 (2013-03-25)
-------------------
* Fix: Memory pool was not freed after each call to ``buildIndex()``.
* GCC: Added ``-isystem`` flag to gtest headers to avoid pedantic warnings.
* Contributors: Jose Luis Blanco-Claraco

1.1.4 (2013-01-11)
-------------------
* Fix compilation with Visual Studio 11 (MSVC 2012).
* Fix compilation of gtest with VS11 and its ``_VARIADIC_MAX`` bug.
* Add security check: throw exception if searches are attempted before ``buildIndex()``.
* New example demonstrating save/load of the index to files.
* Expose save/load methods as public.
* Contributors: Jose Luis Blanco-Claraco

1.1.3 (2012-06-06)
-------------------
* Embed GTest sources due to changes in newer Ubuntu packages.
* Add asserts to detect NULL query points passed by the user.
* New method ``RadiusResultSet::worst_item()``.
* New method ``RadiusResultSet::set_radius_and_clear()``.
* Avoid potential collision of min/max macros with ``<windows.h>``.
* Remove unneeded ``#include`` of std headers.
* New sample code for vectors of vectors.
* Fix building of tests for MSVC on Windows.
* Allow manually setting path to Eigen3 (mainly for Windows example builds).
* Contributors: Jose Luis Blanco-Claraco

1.1.2 (2012-05-02)
-------------------
* Better documentation and benchmark graphs for choosing ``leaf_max_size``.
* ``KDTreeSingleIndexAdaptor::buildIndex()`` can now be called multiple times even when the dataset size changes.
* Contributors: Jose Luis Blanco-Claraco

1.1.1 (2012-02-01)
-------------------
* Fix kd-tree index and L1/L2 metrics to allow distinct types for data elements and distances (e.g. integer data with floating-point distances).
* Fix examples and unit tests to use template arguments instead of hard-coded ``float`` types.
* Contributors: Jose Luis Blanco-Claraco, Thomas Vincent

1.1.0 (2011-12-15)
-------------------
* Fix warnings for MSVC and GCC with ``-Wall -pedantic``.
* Update performance tests for current nanoflann code.
* All main classes now have a new template argument for index type, defaulting to ``size_t`` instead of ``int``.
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2011-08-30)
-------------------
* Initial release.
* Contributors: Jose Luis Blanco-Claraco
