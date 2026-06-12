/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2026 Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
 *
 * THE BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

#include <sstream>

#include "test_helpers.h"

TEST(kdtree, L2_dynamic_vs_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 10; i++)
    {
        L2_dynamic_vs_bruteforce_test<float>(100);
        L2_dynamic_vs_bruteforce_test<float>(100);
        L2_dynamic_vs_bruteforce_test<float>(100);

        L2_dynamic_vs_bruteforce_test<double>(100);
        L2_dynamic_vs_bruteforce_test<double>(100);
        L2_dynamic_vs_bruteforce_test<double>(100);
    }
}

TEST(kdtree, L2_static_vs_dynamic)
{
    for (int nResults = 0; nResults < 10; nResults++)
    {
        L2_dynamic_sorted_test<float>(100, nResults);
        L2_dynamic_sorted_test<double>(100, nResults);
    }
}

TEST(kdtree, robust_nonempty_tree)
{
    // Try to build a dynamic tree with some initial points
    PointCloud<double> cloud;
    const size_t       max_point_count = 1000;
    generateRandomPointCloud(cloud, max_point_count);

    const double query_pt[3] = {0.5, 0.5, 0.5};

    // construct a kd-tree index:
    typedef KDTreeSingleIndexDynamicAdaptor<
        L2_Simple_Adaptor<double, PointCloud<double>>, PointCloud<double>, 3>
        my_kd_tree_simple_t;

    my_kd_tree_simple_t index1(
        3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */), max_point_count);

    // Try a search and expect a neighbor to exist because the dynamic tree was
    // passed a non-empty cloud
    const size_t                    num_results = 1;
    std::vector<size_t>             ret_index(num_results);
    std::vector<double>             out_dist_sqr(num_results);
    nanoflann::KNNResultSet<double> resultSet(num_results);
    resultSet.init(&ret_index[0], &out_dist_sqr[0]);
    bool result = index1.findNeighbors(resultSet, &query_pt[0]);
    EXPECT_EQ(result, true);
}

TEST(kdtree, add_and_remove_points)
{
    PointCloud<double> cloud;
    cloud.pts = {{0.0, 0.0, 0.0}, {0.5, 0.5, 0.5}, {0.7, 0.7, 0.7}};

    typedef KDTreeSingleIndexDynamicAdaptor<
        L2_Simple_Adaptor<double, PointCloud<double>>, PointCloud<double>, 3>
        my_kd_tree_simple_t;

    my_kd_tree_simple_t index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

    const auto query = [&index]() -> size_t
    {
        const double                    query_pt[3] = {0.5, 0.5, 0.5};
        const size_t                    num_results = 1;
        std::vector<size_t>             ret_index(num_results);
        std::vector<double>             out_dist_sqr(num_results);
        nanoflann::KNNResultSet<double> resultSet(num_results);

        resultSet.init(&ret_index[0], &out_dist_sqr[0]);
        index.findNeighbors(resultSet, &query_pt[0]);

        return ret_index[0];
    };

    auto actual = query();
    EXPECT_EQ(actual, static_cast<size_t>(1));

    index.removePoint(1);
    actual = query();
    EXPECT_EQ(actual, static_cast<size_t>(2));

    index.addPoints(1, 1);
    actual = query();
    EXPECT_EQ(actual, static_cast<size_t>(1));

    index.removePoint(1);
    index.removePoint(2);
    actual = query();
    EXPECT_EQ(actual, static_cast<size_t>(0));
}

// Test that repeated removePoint/addPoints cycles work correctly.
// This exercises the case where the dynamic adaptor's internal pointCount_
// grows beyond kdtree_get_point_count() due to re-adding removed points,
// which must not cause assertion failures in divideTree.
TEST(kdtree, dynamic_repeated_remove_readd)
{
    PointCloud<double> cloud;
    // Create enough points that sub-trees accumulate entries exceeding N
    const size_t N = 20;
    for (size_t i = 0; i < N; i++)
    {
        cloud.pts.push_back({
            static_cast<double>(i),
            static_cast<double>(i * 2),
            static_cast<double>(i * 3),
        });
    }

    typedef KDTreeSingleIndexDynamicAdaptor<
        L2_Simple_Adaptor<double, PointCloud<double>>, PointCloud<double>, 3>
        my_kd_tree_t;

    my_kd_tree_t index(3, cloud, KDTreeSingleIndexAdaptorParams(10));

    // Repeatedly remove and re-add groups of points, simulating the pattern
    // of temporarily excluding points from queries then restoring them.
    for (size_t iteration = 0; iteration < 10; iteration++)
    {
        // Remove a group of points
        for (size_t i = 0; i < N / 2; i++)
        {
            index.removePoint(i);
        }

        // Query while points are removed
        const double                    query_pt[3] = {5.0, 10.0, 15.0};
        std::vector<size_t>             ret_index(1);
        std::vector<double>             out_dist_sqr(1);
        nanoflann::KNNResultSet<double> resultSet(1);
        resultSet.init(&ret_index[0], &out_dist_sqr[0]);
        index.findNeighbors(resultSet, &query_pt[0]);

        // The nearest point should be among the non-removed ones (indices N/2..N-1)
        EXPECT_GE(ret_index[0], N / 2);

        // Re-add the removed points
        for (size_t i = 0; i < N / 2; i++)
        {
            index.addPoints(static_cast<uint32_t>(i), static_cast<uint32_t>(i));
        }

        // Query again - nearest should be index 5 (closest to query point)
        nanoflann::KNNResultSet<double> resultSet2(1);
        resultSet2.init(&ret_index[0], &out_dist_sqr[0]);
        index.findNeighbors(resultSet2, &query_pt[0]);
        EXPECT_EQ(ret_index[0], static_cast<size_t>(5));
    }
}

// Re-adding a previously-removed point must reactivate the existing entry in
// place, not insert a duplicate. Otherwise repeated remove/add cycles make the
// internal index vectors grow without bound and cause the same point index to
// be reported multiple times in radius/knn searches.
TEST(kdtree, dynamic_readd_no_duplicates)
{
    PointCloud<double> cloud;
    const size_t       N = 20;
    for (size_t i = 0; i < N; i++)
    {
        cloud.pts.push_back({
            static_cast<double>(i),
            static_cast<double>(i * 2),
            static_cast<double>(i * 3),
        });
    }

    typedef KDTreeSingleIndexDynamicAdaptor<
        L2_Simple_Adaptor<double, PointCloud<double>>, PointCloud<double>, 3>
        my_kd_tree_t;

    my_kd_tree_t index(3, cloud, KDTreeSingleIndexAdaptorParams(10));

    // Helper: total number of entries physically stored across all sub-trees.
    const auto total_stored_entries = [&index]() -> size_t
    {
        size_t total = 0;
        for (const auto& subtree : index.getAllIndices()) total += subtree.vAcc_.size();
        return total;
    };

    // Right after construction every point is stored exactly once.
    EXPECT_EQ(total_stored_entries(), N);

    // Repeatedly remove and re-add the same group of points.
    for (size_t iteration = 0; iteration < 10; iteration++)
    {
        for (size_t i = 0; i < N / 2; i++) index.removePoint(i);
        for (size_t i = 0; i < N / 2; i++)
            index.addPoints(static_cast<uint32_t>(i), static_cast<uint32_t>(i));
    }

    // The number of stored entries must not have grown: re-adding a removed
    // point reuses its slot rather than appending a duplicate.
    EXPECT_EQ(total_stored_entries(), N);

    // A radius search large enough to cover every point must return each point
    // index exactly once (no duplicates).
    const double                                       query_pt[3] = {0.0, 0.0, 0.0};
    std::vector<nanoflann::ResultItem<size_t, double>> matches;
    nanoflann::RadiusResultSet<double, size_t>         resultSet(1e30, matches);
    index.findNeighbors(resultSet, &query_pt[0]);

    EXPECT_EQ(matches.size(), N);
    std::set<size_t> unique_indices;
    for (const auto& m : matches) unique_indices.insert(m.first);
    EXPECT_EQ(unique_indices.size(), N);
}

// Regression test: saveIndex/loadIndex for an empty dataset.
// saveIndex conditionally skips save_tree when root_node_==nullptr, but
// loadIndex used to call load_tree unconditionally, reading past EOF on the
// stream (setting its failbit) and leaving root_node_ as a non-null pointer to
// garbage memory.
TEST(kdtree, saveload_empty_index)
{
    using num_t    = double;
    using cloud_t  = PointCloud<num_t>;
    using kdtree_t = KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<num_t, cloud_t>, cloud_t, 3>;

    cloud_t emptyCloud;  // 0 points

    std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);
    {
        const kdtree_t index(3, emptyCloud, KDTreeSingleIndexAdaptorParams(10));
        index.saveIndex(ss);
    }

    {
        kdtree_t index(
            3, emptyCloud,
            KDTreeSingleIndexAdaptorParams(
                10, KDTreeSingleIndexAdaptorFlags::SkipInitialBuildIndex));
        index.loadIndex(ss);

        // Bug: load_tree always ran even for empty indices, reading sizeof(Node)
        // bytes past EOF and setting the stream's failbit.
        EXPECT_FALSE(ss.fail());

        num_t                          query_pt[3] = {0.5, 0.5, 0.5};
        nanoflann::KNNResultSet<num_t> resultSet(1);
        size_t                         ret_index;
        num_t                          out_dist_sqr;
        resultSet.init(&ret_index, &out_dist_sqr);
        EXPECT_FALSE(index.findNeighbors(resultSet, query_pt));
    }
}

// Verify that a non-empty index produces identical search results after a
// save/load round-trip.
TEST(kdtree, saveload_nonempty_index)
{
    srand(42);

    using num_t    = double;
    using cloud_t  = PointCloud<num_t>;
    using kdtree_t = KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<num_t, cloud_t>, cloud_t, 3>;

    cloud_t cloud;
    generateRandomPointCloud(cloud, 10000);

    const num_t query_pt[3] = {0.5, 0.5, 0.5};
    size_t      expected_idx;
    num_t       expected_dist;

    std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);
    {
        kdtree_t                       index(3, cloud, KDTreeSingleIndexAdaptorParams(10));
        nanoflann::KNNResultSet<num_t> rs(1);
        rs.init(&expected_idx, &expected_dist);
        index.findNeighbors(rs, query_pt);
        index.saveIndex(ss);
    }

    {
        kdtree_t index(
            3, cloud,
            KDTreeSingleIndexAdaptorParams(
                10, KDTreeSingleIndexAdaptorFlags::SkipInitialBuildIndex));
        index.loadIndex(ss);

        size_t                         loaded_idx;
        num_t                          loaded_dist;
        nanoflann::KNNResultSet<num_t> rs(1);
        rs.init(&loaded_idx, &loaded_dist);
        index.findNeighbors(rs, query_pt);

        EXPECT_EQ(loaded_idx, expected_idx);
        EXPECT_NEAR(loaded_dist, expected_dist, 1e-10);
    }
}

// Regression test: a truncated/corrupt stream whose stored vector size passes
// the header check must throw rather than trigger an unbounded resize() (which
// could attempt a multi-GB allocation / bad_alloc). See load_value(vector).
TEST(kdtree, loadindex_corrupt_vector_size_throws)
{
    srand(42);

    using num_t    = double;
    using cloud_t  = PointCloud<num_t>;
    using kdtree_t = KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<num_t, cloud_t>, cloud_t, 3>;

    cloud_t cloud;
    generateRandomPointCloud(cloud, 1000);

    std::string blob;
    {
        std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);
        kdtree_t          index(3, cloud, KDTreeSingleIndexAdaptorParams(10));
        index.saveIndex(ss);
        blob = ss.str();
    }

    // Truncate the serialized index to half its length: the header still parses,
    // but a vector size read later now claims more elements than bytes remain.
    ASSERT_GT(blob.size(), 64u);
    blob.resize(blob.size() / 2);

    std::stringstream ss(blob, std::ios::in | std::ios::out | std::ios::binary);
    kdtree_t          index(
                 3, cloud,
                 KDTreeSingleIndexAdaptorParams(10, KDTreeSingleIndexAdaptorFlags::SkipInitialBuildIndex));
    EXPECT_ANY_THROW(index.loadIndex(ss));
}

// Regression test: saveIndex/loadIndex on KDTreeSingleIndexDynamicAdaptor_ used
// to recurse infinitely because the 1-arg method name-hid Base::saveIndex and
// called itself instead of Base::saveIndex(*this, stream).
TEST(kdtree, dynamic_saveload_roundtrip)
{
    srand(42);

    using num_t    = double;
    using cloud_t  = PointCloud<num_t>;
    using dist_t   = nanoflann::L2_Simple_Adaptor<num_t, cloud_t>;
    using subidx_t = KDTreeSingleIndexDynamicAdaptor_<dist_t, cloud_t, 3>;

    cloud_t cloud;
    generateRandomPointCloud(cloud, 200);

    // Build a sub-tree directly with all points assigned to tree slot 0.
    std::vector<int> treeIndex(cloud.pts.size(), 0);
    subidx_t         orig(3, cloud, treeIndex);
    for (size_t i = 0; i < cloud.pts.size(); ++i)
    {
        orig.vAcc_.push_back(static_cast<uint32_t>(i));
    }
    orig.buildIndex();

    const num_t query_pt[3] = {0.5, 0.5, 0.5};

    size_t                         expected_idx;
    num_t                          expected_dist;
    nanoflann::KNNResultSet<num_t> rs_orig(1);
    rs_orig.init(&expected_idx, &expected_dist);
    orig.findNeighbors(rs_orig, query_pt, nanoflann::SearchParameters());

    // Save the sub-tree index.
    std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);
    orig.saveIndex(ss);
    EXPECT_FALSE(ss.fail());

    // Load into a fresh sub-tree and verify the search result matches.
    subidx_t loaded(3, cloud, treeIndex);
    loaded.loadIndex(ss);
    EXPECT_FALSE(ss.fail());

    size_t                         loaded_idx;
    num_t                          loaded_dist;
    nanoflann::KNNResultSet<num_t> rs_loaded(1);
    rs_loaded.init(&loaded_idx, &loaded_dist);
    loaded.findNeighbors(rs_loaded, query_pt, nanoflann::SearchParameters());

    EXPECT_EQ(loaded_idx, expected_idx);
    EXPECT_NEAR(loaded_dist, expected_dist, 1e-10);
}

// loadIndex() must throw when the stream does not start with the nanoflann magic.
TEST(kdtree, saveload_bad_magic)
{
    using num_t    = double;
    using cloud_t  = PointCloud<num_t>;
    using kdtree_t = KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<num_t, cloud_t>, cloud_t, 3>;

    cloud_t cloud;
    generateRandomPointCloud(cloud, 100);

    std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);
    {
        kdtree_t index(3, cloud, KDTreeSingleIndexAdaptorParams(10));
        index.saveIndex(ss);
    }

    // Overwrite the first four bytes (magic) with garbage.
    ss.seekp(0);
    const uint32_t bad_magic = 0xDEADBEEF;
    ss.write(reinterpret_cast<const char*>(&bad_magic), sizeof(bad_magic));
    ss.seekg(0);

    kdtree_t index(
        3, cloud,
        KDTreeSingleIndexAdaptorParams(10, KDTreeSingleIndexAdaptorFlags::SkipInitialBuildIndex));
    EXPECT_THROW(index.loadIndex(ss), std::runtime_error);
}

// loadIndex() must throw when the nanoflann version in the file differs.
TEST(kdtree, saveload_version_mismatch)
{
    using num_t    = double;
    using cloud_t  = PointCloud<num_t>;
    using kdtree_t = KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<num_t, cloud_t>, cloud_t, 3>;

    cloud_t cloud;
    generateRandomPointCloud(cloud, 100);

    std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);
    {
        kdtree_t index(3, cloud, KDTreeSingleIndexAdaptorParams(10));
        index.saveIndex(ss);
    }

    // Overwrite the version field (bytes 4-7) with a fake version.
    ss.seekp(4);
    const uint32_t bad_version = 0x000;
    ss.write(reinterpret_cast<const char*>(&bad_version), sizeof(bad_version));
    ss.seekg(0);

    kdtree_t index(
        3, cloud,
        KDTreeSingleIndexAdaptorParams(10, KDTreeSingleIndexAdaptorFlags::SkipInitialBuildIndex));
    EXPECT_THROW(index.loadIndex(ss), std::runtime_error);
}