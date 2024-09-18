/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2024 Jose Luis Blanco (joseluisblancoc@gmail.com).
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

#include <gtest/gtest.h>

#include <cmath>  // for abs()
#include <cstdlib>
#include <iostream>
#include <map>
#include <nanoflann.hpp>

#include "../examples/utils.h"

using namespace std;
using namespace nanoflann;

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

template <typename num_t, bool LOOSETREE>
void L2_vs_L2_simple_test(const size_t N, const size_t num_results)
{
    PointCloud<num_t> cloud;

    // Generate points:
    generateRandomPointCloud(cloud, N);

    const PointCloud<num_t>::Point query_pt{num_t(0.5), num_t(0.5), num_t(0.5)};

    // construct a kd-tree index:
    using my_kd_tree_simple_t = KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<num_t, PointCloud<num_t>>, PointCloud<num_t>,
        3 /* dim */,
        uint32_t /* index type*/,
        LOOSETREE
        >;

    using my_kd_tree_t = KDTreeSingleIndexAdaptor<
        L2_Adaptor<num_t, PointCloud<num_t>>, PointCloud<num_t>, 3 /* dim */, uint32_t /* index type */, LOOSETREE
        >;

    my_kd_tree_simple_t index1(
        3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

    my_kd_tree_t index2(
        3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

    // do a knn search
    std::vector<size_t>            ret_index(num_results);
    std::vector<num_t>             out_dist_sqr(num_results);
    nanoflann::KNNResultSet<num_t> resultSet(num_results);
    resultSet.init(&ret_index[0], &out_dist_sqr[0]);
    index1.findNeighbors(resultSet, query_pt);

    std::vector<size_t> ret_index1    = ret_index;
    std::vector<num_t>  out_dist_sqr1 = out_dist_sqr;

    resultSet.init(&ret_index[0], &out_dist_sqr[0]);

    index2.findNeighbors(resultSet, query_pt);

    for (size_t i = 0; i < num_results; i++)
    {
        EXPECT_EQ(ret_index1[i], ret_index[i]);
        EXPECT_DOUBLE_EQ(out_dist_sqr1[i], out_dist_sqr[i]);
    }
    // Ensure results are sorted:
    num_t lastDist = -1;
    for (size_t i = 0; i < out_dist_sqr.size(); i++)
    {
        const num_t newDist = out_dist_sqr[i];
        EXPECT_GE(newDist, lastDist);
        lastDist = newDist;
    }

    // Test "RadiusResultSet" too:
    const num_t maxRadiusSqrSearch = 10.0 * 10.0;

    std::vector<nanoflann::ResultItem<
        typename my_kd_tree_simple_t::IndexType,
        typename my_kd_tree_simple_t::DistanceType>>
        radiusIdxs;

    nanoflann::RadiusResultSet<num_t, typename my_kd_tree_simple_t::IndexType>
        radiusResults(maxRadiusSqrSearch, radiusIdxs);
    radiusResults.init();
    nanoflann::SearchParameters searchParams;
    searchParams.sorted = true;
    index1.findNeighbors(radiusResults, query_pt, searchParams);

    // Ensure results are sorted:
    lastDist = -1;
    for (const auto& r : radiusIdxs)
    {
        const num_t newDist = r.second;
        EXPECT_GE(newDist, lastDist);
        lastDist = newDist;
    }
}

using namespace nanoflann;
#include "../examples/KDTreeVectorOfVectorsAdaptor.h"

template <typename NUM>
void generateRandomPointCloud(
    std::vector<std::vector<NUM>>& samples, const size_t N, const size_t dim,
    const NUM max_range)
{
    samples.resize(N);
    for (size_t i = 0; i < N; i++)
    {
        samples[i].resize(dim);
        for (size_t d = 0; d < dim; d++)
            samples[i][d] = max_range * (rand() % 1000) / NUM(1000.0);
    }
}

template <typename NUM>
void L2_vs_bruteforce_test(
    const size_t nSamples, const size_t DIM, const size_t numToSearch)
{
    std::vector<std::vector<NUM>> samples;

    const NUM max_range = NUM(20.0);

    // Generate points:
    generateRandomPointCloud(samples, nSamples, DIM, max_range);

    // Query point:
    std::vector<NUM> query_pt(DIM);
    for (size_t d = 0; d < DIM; d++)
        query_pt[d] = static_cast<NUM>(max_range * (rand() % 1000) / (1000.0));

    // construct a kd-tree index:
    // Dimensionality set at run-time (default: L2)
    // ------------------------------------------------------------
    typedef KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<NUM>>, NUM>
        my_kd_tree_t;

    my_kd_tree_t mat_index(DIM /*dim*/, samples, 10 /* max leaf */);

    // do a knn search
    const size_t        num_results = numToSearch;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<NUM>    out_dists_sqr(num_results);

    nanoflann::KNNResultSet<NUM> resultSet(num_results);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    mat_index.index->findNeighbors(resultSet, my_kd_tree_t::PointType(query_pt));

    const auto nFound = resultSet.size();

    // Brute force neighbors:
    std::multimap<NUM /*dist*/, size_t /*idx*/> bf_nn;
    {
        for (size_t i = 0; i < nSamples; i++)
        {
            double dist = 0.0;
            for (size_t d = 0; d < DIM; d++)
                dist += (query_pt[d] - samples[i][d]) *
                        (query_pt[d] - samples[i][d]);
            bf_nn.emplace(dist, i);
        }
    }

    // Keep bruteforce solutions indexed by idx instead of distances,
    // to handle correctly almost or exactly coindicing distances for >=2 NN:
    std::map<size_t, NUM> bf_idx2dist;
    for (const auto& kv : bf_nn) bf_idx2dist[kv.second] = kv.first;

    // Compare:
    if (!bf_nn.empty())
    {
        auto it = bf_nn.begin();
        for (size_t i = 0; i < nFound; ++i, ++it)
        {
            // Distances must be in exact order:
            EXPECT_NEAR(it->first, out_dists_sqr[i], 1e-3);

            // indices may be not in the (rare) case of a tie:
            EXPECT_NEAR(bf_idx2dist.at(ret_indexes[i]), out_dists_sqr[i], 1e-3)
                << "For: numToSearch=" << numToSearch
                << " out_dists_sqr[i]=" << out_dists_sqr[i] << "\n";
        }
    }
}

template <typename NUM>
void rknn_L2_vs_bruteforce_test(
    const size_t nSamples, const size_t DIM, const size_t numToSearch,
    const NUM maxRadiusSqr)
{
    std::vector<std::vector<NUM>> samples;

    const NUM max_range = NUM(20.0);

    // Generate points:
    generateRandomPointCloud(samples, nSamples, DIM, max_range);

    // Query point:
    std::vector<NUM> query_pt(DIM);
    for (size_t d = 0; d < DIM; d++)
        query_pt[d] = static_cast<NUM>(max_range * (rand() % 1000) / (1000.0));

    // construct a kd-tree index:
    // Dimensionality set at run-time (default: L2)
    // ------------------------------------------------------------
    typedef KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<NUM>>, NUM>
        my_kd_tree_t;

    my_kd_tree_t mat_index(DIM /*dim*/, samples, 10 /* max leaf */);

    // do a knn search
    const size_t        num_results = numToSearch;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<NUM>    out_dists_sqr(num_results);

    nanoflann::RKNNResultSet<NUM> resultSet(num_results, maxRadiusSqr);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    mat_index.index->findNeighbors(resultSet, my_kd_tree_t::PointType(query_pt));

    const auto nFound = resultSet.size();

    // Brute force neighbors:
    std::multimap<NUM /*dist*/, size_t /*idx*/> bf_nn;
    {
        for (size_t i = 0; i < nSamples; i++)
        {
            double dist = 0.0;
            for (size_t d = 0; d < DIM; d++)
                dist += (query_pt[d] - samples[i][d]) *
                        (query_pt[d] - samples[i][d]);

            if (dist <= maxRadiusSqr) bf_nn.emplace(dist, i);
        }
    }

    // Keep bruteforce solutions indexed by idx instead of distances,
    // to handle correctly almost or exactly coindicing distances for >=2 NN:
    std::map<size_t, NUM> bf_idx2dist;
    for (const auto& kv : bf_nn) bf_idx2dist[kv.second] = kv.first;

    // Compare:
    if (!bf_nn.empty())
    {
        auto it = bf_nn.begin();
        for (size_t i = 0; i < nFound; ++i, ++it)
        {
            // Distances must be in exact order:
            EXPECT_NEAR(it->first, out_dists_sqr[i], 1e-3)
                << "For: numToSearch=" << numToSearch
                << " out_dists_sqr[i]=" << out_dists_sqr[i] << "\n";

            // indices may be not in the (rare) case of a tie:
            EXPECT_NEAR(bf_idx2dist.at(ret_indexes[i]), out_dists_sqr[i], 1e-3)
                << "For: numToSearch=" << numToSearch
                << " out_dists_sqr[i]=" << out_dists_sqr[i] << "\n";
        }
    }
}

template <typename NUM>
void SO3_vs_bruteforce_test(const size_t nSamples)
{
    PointCloud_Quat<NUM> cloud;

    // Generate points:
    generateRandomPointCloud_Quat(cloud, nSamples);

    const PointCloud_Quat<NUM>::PointType query_pt{0.5, 0.5, 0.5, 0.5};

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
        SO3_Adaptor<NUM, PointCloud_Quat<NUM>>, PointCloud_Quat<NUM>,
        4 /* dim */
        >
        my_kd_tree_t;

    my_kd_tree_t index(
        4 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    // do a knn search
    const size_t        num_results = 1;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<NUM>    out_dists_sqr(num_results);

    nanoflann::KNNResultSet<NUM> resultSet(num_results);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    index.findNeighbors(resultSet, query_pt);

    // Brute force:
    double min_dist_L2 = std::numeric_limits<double>::max();
    size_t min_idx     = std::numeric_limits<size_t>::max();
    {
        for (size_t i = 0; i < nSamples; i++)
        {
            double dist = 0.0;
            for (int d = 0; d < 4; d++)
                dist += (query_pt.get_component(d) - cloud.kdtree_get_pt(i).get_component(d)) *
                        (query_pt.get_component(d) - cloud.kdtree_get_pt(i).get_component(d));
            if (dist < min_dist_L2)
            {
                min_dist_L2 = dist;
                min_idx     = i;
            }
        }
        ASSERT_TRUE(min_idx != std::numeric_limits<size_t>::max());
    }

    // Compare:
    EXPECT_EQ(min_idx, ret_indexes[0]);
    EXPECT_NEAR(min_dist_L2, out_dists_sqr[0], 1e-3);
}

template <typename NUM>
void SO2_vs_bruteforce_test(const size_t nSamples)
{
    PointCloud_Orient<NUM> cloud;

    // Generate points:
    generateRandomPointCloud_Orient(cloud, nSamples);

    const PointCloud_Orient<NUM>::PointType query_pt{0.5};

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
        SO2_Adaptor<NUM, PointCloud_Orient<NUM>>, PointCloud_Orient<NUM>,
        1 /* dim */
        >
        my_kd_tree_t;

    my_kd_tree_t index(
        1 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    // do a knn search
    const size_t        num_results = 1;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<NUM>    out_dists_sqr(num_results);

    nanoflann::KNNResultSet<NUM> resultSet(num_results);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    index.findNeighbors(resultSet, query_pt);

    // Brute force:
    double min_dist_SO2 = std::numeric_limits<double>::max();
    size_t min_idx      = std::numeric_limits<size_t>::max();
    {
        for (size_t i = 0; i < nSamples; i++)
        {
            double dist = 0.0;
            dist        = cloud.kdtree_get_pt(i).get_component(0) - query_pt.get_component(0);
            if (dist > nanoflann::pi_const<double>())
                dist -= 2 * nanoflann::pi_const<double>();
            else if (dist < -nanoflann::pi_const<double>())
                dist += 2 * nanoflann::pi_const<double>();
            if (dist < min_dist_SO2)
            {
                min_dist_SO2 = dist;
                min_idx      = i;
            }
        }
        ASSERT_TRUE(min_idx != std::numeric_limits<size_t>::max());
    }
    // Compare:
    EXPECT_EQ(min_idx, ret_indexes[0]);
    EXPECT_NEAR(min_dist_SO2, out_dists_sqr[0], 1e-3);
}

// First add nSamples/2 points, find the closest point
// Then add remaining points and find closest point
// Compare both with closest point using brute force approach
template <typename NUM>
void L2_dynamic_vs_bruteforce_test(const size_t nSamples)
{
    PointCloud<NUM> cloud;

    const NUM max_range = NUM(20.0);

    // construct a kd-tree index:
    typedef KDTreeSingleIndexDynamicAdaptor<
        L2_Simple_Adaptor<NUM, PointCloud<NUM>>, PointCloud<NUM>, 3 /* dim */
        >
        my_kd_tree_t;

    my_kd_tree_t index(
        3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

    // Generate points:
    generateRandomPointCloud(cloud, nSamples, max_range);

    const PointCloud<NUM>::PointType query_pt{0.5, 0.5, 0.5};

    // add points in chunks at a time
    size_t chunk_size = 100;
    size_t end        = 0;
    for (size_t i = 0; i < nSamples / 2; i = i + chunk_size)
    {
        end = min(size_t(i + chunk_size), nSamples / 2 - 1);
        index.addPoints(i, end);
    }

    {
        // do a knn search
        const size_t        num_results = 1;
        std::vector<size_t> ret_indexes(num_results);
        std::vector<NUM>    out_dists_sqr(num_results);

        nanoflann::KNNResultSet<NUM> resultSet(num_results);

        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
        index.findNeighbors(resultSet, query_pt);

        // Brute force:
        double min_dist_L2 = std::numeric_limits<double>::max();
        size_t min_idx     = std::numeric_limits<size_t>::max();
        {
            for (size_t i = 0; i < nSamples / 2; i++)
            {
                double dist = 0.0;
                for (int d = 0; d < 3; d++)
                    dist += (query_pt.get_component(d) - cloud.kdtree_get_pt(i).get_component(d)) *
                            (query_pt.get_component(d) - cloud.kdtree_get_pt(i).get_component(d));
                if (dist < min_dist_L2)
                {
                    min_dist_L2 = dist;
                    min_idx     = i;
                }
            }
            ASSERT_TRUE(min_idx != std::numeric_limits<size_t>::max());
        }
        // Compare:
        EXPECT_EQ(min_idx, ret_indexes[0]);
        EXPECT_NEAR(min_dist_L2, out_dists_sqr[0], 1e-3);
    }
    for (size_t i = end + 1; i < nSamples; i = i + chunk_size)
    {
        end = min(size_t(i + chunk_size), nSamples - 1);
        index.addPoints(i, end);
    }

    {
        // do a knn search
        const size_t        num_results = 1;
        std::vector<size_t> ret_indexes(num_results);
        std::vector<NUM>    out_dists_sqr(num_results);

        nanoflann::KNNResultSet<NUM> resultSet(num_results);

        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
        index.findNeighbors(resultSet, query_pt);

        // Brute force:
        double min_dist_L2 = std::numeric_limits<double>::max();
        size_t min_idx     = std::numeric_limits<size_t>::max();
        {
            for (size_t i = 0; i < nSamples; i++)
            {
                double dist = 0.0;
                for (int d = 0; d < 3; d++)
                    dist += (query_pt.get_component(d) - cloud.kdtree_get_pt(i).get_component(d)) *
                            (query_pt.get_component(d) - cloud.kdtree_get_pt(i).get_component(d));
                if (dist < min_dist_L2)
                {
                    min_dist_L2 = dist;
                    min_idx     = i;
                }
            }
            ASSERT_TRUE(min_idx != std::numeric_limits<size_t>::max());
        }
        // Compare:
        EXPECT_EQ(min_idx, ret_indexes[0]);
        EXPECT_NEAR(min_dist_L2, out_dists_sqr[0], 1e-3);
    }
}

template <typename NUM>
void L2_concurrent_build_vs_bruteforce_test(
    const size_t nSamples, const size_t DIM)
{
    std::vector<std::vector<NUM>> samples;

    const NUM max_range = NUM(20.0);

    // Generate points:
    generateRandomPointCloud(samples, nSamples, DIM, max_range);

    // Query point:
    std::vector<NUM> query_pt(DIM);
    for (size_t d = 0; d < DIM; d++)
        query_pt[d] = static_cast<NUM>(max_range * (rand() % 1000) / (1000.0));

    // construct a kd-tree index:
    // Dimensionality set at run-time (default: L2)
    // ------------------------------------------------------------
    typedef KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<NUM>>, NUM>
        my_kd_tree_t;

    my_kd_tree_t mat_index(
        DIM /*dim*/, samples, 10 /* max leaf */, 0 /* concurrent build */);

    // do a knn search
    const size_t        num_results = 1;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<NUM>    out_dists_sqr(num_results);

    nanoflann::KNNResultSet<NUM> resultSet(num_results);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    mat_index.index->findNeighbors(resultSet, my_kd_tree_t::PointType(query_pt));

    // Brute force:
    double min_dist_L2 = std::numeric_limits<double>::max();
    size_t min_idx     = std::numeric_limits<size_t>::max();
    {
        for (size_t i = 0; i < nSamples; i++)
        {
            double dist = 0.0;
            for (size_t d = 0; d < DIM; d++)
                dist += (query_pt[d] - samples[i][d]) *
                        (query_pt[d] - samples[i][d]);
            if (dist < min_dist_L2)
            {
                min_dist_L2 = dist;
                min_idx     = i;
            }
        }
        ASSERT_TRUE(min_idx != std::numeric_limits<size_t>::max());
    }

    // Compare:
    EXPECT_EQ(min_idx, ret_indexes[0]);
    EXPECT_NEAR(min_dist_L2, out_dists_sqr[0], 1e-3);
}

template <typename NUM>
void L2_concurrent_build_vs_L2_test(const size_t nSamples, const size_t DIM)
{
    std::vector<std::vector<NUM>> samples;

    const NUM max_range = NUM(20.0);

    // Generate points:
    generateRandomPointCloud(samples, nSamples, DIM, max_range);

    // Query point:
    std::vector<NUM> query_pt(DIM);
    for (size_t d = 0; d < DIM; d++)
        query_pt[d] = static_cast<NUM>(max_range * (rand() % 1000) / (1000.0));

    // construct a kd-tree index:
    // Dimensionality set at run-time (default: L2)
    // ------------------------------------------------------------
    typedef KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<NUM>>, NUM>
        my_kd_tree_t;

    my_kd_tree_t mat_index_concurrent_build(
        DIM /*dim*/, samples, 10 /* max leaf */, 0 /* concurrent build */);
    my_kd_tree_t mat_index(DIM /*dim*/, samples, 10 /* max leaf */);

    // Compare:
    EXPECT_EQ(mat_index.index->vAcc_, mat_index_concurrent_build.index->vAcc_);
}

template <typename num_t, bool LOOSETREE>
void radius_search_test()
{
    PointCloud<num_t> cloud;
    generateGridPointCloud(cloud, 3, 4, 5);

    const PointCloud<num_t>::Point query_pt{num_t(1.22), num_t(2.12), num_t(3.47)};

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<num_t, PointCloud<num_t>>, PointCloud<num_t>,
        3 /* dim */, uint32_t, LOOSETREE>
        my_kd_tree_simple_t;

    my_kd_tree_simple_t index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    index.buildIndex();

    // do a radius search
    const num_t radius = num_t(1.88);
    std::vector<ResultItem<uint32_t, num_t>> matches;
    matches.reserve(30);
    const auto num_results = index.radiusSearch(query_pt, radius, matches);
    ASSERT_EQ(num_results, matches.size());

    // execute linear search through the points
    std::vector<std::pair<uint32_t, num_t>> checkMatches;
    checkMatches.reserve(30);
    for (size_t i = 0; i < cloud.kdtree_get_point_count(); ++i)
    {
        const auto dist = cloud.kdtree_get_pt(i).get_distance_to(query_pt);
        if (dist < radius)
            checkMatches.emplace_back(i, dist);
    }
    ASSERT_EQ(num_results, checkMatches.size());
    std::sort(checkMatches.begin(), checkMatches.end(), IndexDist_Sorter());

    for (size_t i = 0; i < num_results; i++)
    {
        EXPECT_EQ(matches[i].first, checkMatches[i].first);
        EXPECT_EQ(matches[i].second, checkMatches[i].second);
    }
}

template <typename num_t, bool LOOSETREE>
void radius_search_dynamic_test()
{
    PointCloud<num_t> cloud;
    generateGridPointCloud(cloud, 3, 4, 5);

    const PointCloud<num_t>::Point query_pt{
        num_t(1.22), num_t(2.12), num_t(3.47)};

    // construct a kd-tree index:
    typedef KDTreeSingleIndexDynamicAdaptor<
        L2_Simple_Adaptor<num_t, PointCloud<num_t>>, PointCloud<num_t>,
        3 /* dim */, uint32_t, LOOSETREE>
        my_kd_tree_simple_t;

    // the constructor also adds the points currently stored in cloud
    my_kd_tree_simple_t index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */), 1000 /* maximum points count */);

    // add some more points
    const size_t numPoints = cloud.pts.size();
    generateGridPointCloud(cloud, 3, 4, 5);
    for (size_t k = numPoints; k < cloud.pts.size(); ++k)
        cloud.pts[k].x += num_t(0.5);
    index.addPoints(numPoints, cloud.pts.size() - 1);

    // do a radius search
    const num_t radius = num_t(1.88);
    std::vector<ResultItem<uint32_t, num_t>> matches;
    matches.reserve(30);
    const auto num_results = index.radiusSearch(query_pt, radius, matches);
    ASSERT_EQ(num_results, matches.size());

    // execute linear search through the points
    std::vector<std::pair<uint32_t, num_t>> checkMatches;
    checkMatches.reserve(30);
    for (size_t i = 0; i < cloud.kdtree_get_point_count(); ++i)
    {
        const auto dist = cloud.kdtree_get_pt(i).get_distance_to(query_pt);
        if (dist < radius) checkMatches.emplace_back(i, dist);
    }
    ASSERT_EQ(num_results, checkMatches.size());
    std::sort(checkMatches.begin(), checkMatches.end(), IndexDist_Sorter());

    for (size_t i = 0; i < num_results; i++)
    {
        EXPECT_EQ(matches[i].first, checkMatches[i].first);
        EXPECT_EQ(matches[i].second, checkMatches[i].second);
    }
}

template <typename num_t, bool LOOSETREE>
void box_search_test()
{
    PointCloud<num_t> cloud;
    generateGridPointCloud(cloud, 6, 7, 8);

    // do a box search (using lineSeg and interpreting start/end points as
    // min/max points of a box)
    const PointCloud<num_t>::Point minPoint{num_t(1.22), num_t(2.12), num_t(3.47)};
    const PointCloud<num_t>::Point maxPoint{num_t(2.42), num_t(3.82), num_t(5.17)};

    // distance from the box (don't use 0.0)
    constexpr num_t cRadius = num_t(1e-6);

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<num_t, PointCloud<num_t>>, PointCloud<num_t>,
        3 /* dim */, uint32_t, LOOSETREE>
        my_kd_tree_simple_t;

    my_kd_tree_simple_t index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    index.buildIndex();

    // do the search
    std::vector<ResultItem<uint32_t, num_t>> matches;
    matches.reserve(30);
    const auto num_results = index.lineSegSearch(minPoint, maxPoint, cRadius, matches);
    ASSERT_EQ(num_results, matches.size());
    std::sort(matches.begin(), matches.end());

    // execute linear search through the points
    std::vector<ResultItem<uint32_t, num_t>> checkMatches;
    checkMatches.reserve(30);
    for (size_t i = 0; i < cloud.kdtree_get_point_count(); ++i)
    {
        const auto dist = cloud.kdtree_get_pt(i).get_distance_to(minPoint, maxPoint);
        if (dist < cRadius)
            checkMatches.emplace_back(i, dist);
    }
    ASSERT_EQ(num_results, checkMatches.size());
    std::sort(checkMatches.begin(), checkMatches.end());

    for (size_t i = 0; i < num_results; i++)
    {
        EXPECT_EQ(matches[i].first, checkMatches[i].first);
        EXPECT_EQ(matches[i].second, checkMatches[i].second);
    }
}

TEST(kdtree, L2_vs_L2_simple)
{
    for (int nResults = 1; nResults < 10; nResults++)
    {
        L2_vs_L2_simple_test<float, false>(100, nResults);
        L2_vs_L2_simple_test<double, false>(100, nResults);

        L2_vs_L2_simple_test<float, true>(100, nResults);
        L2_vs_L2_simple_test<double, true>(100, nResults);
    }
}

TEST(kdtree, robust_empty_tree)
{
    // Try to build a tree with 0 data points, to test
    // robustness against this situation:
    PointCloud<double> cloud;

    const PointCloud<double>::Point query_pt{0.5, 0.5, 0.5};

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<double, PointCloud<double>>, PointCloud<double>,
        3 /* dim */
        >
        my_kd_tree_simple_t;

    my_kd_tree_simple_t index1(
        3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

    // Now we will try to search in the tree, and WE EXPECT a result of
    // no neighbors found if the error detection works fine:
    const size_t                    num_results = 1;
    std::vector<size_t>             ret_index(num_results);
    std::vector<double>             out_dist_sqr(num_results);
    nanoflann::KNNResultSet<double> resultSet(num_results);
    resultSet.init(&ret_index[0], &out_dist_sqr[0]);
    bool result = index1.findNeighbors(resultSet, query_pt);
    EXPECT_EQ(result, false);
}

TEST(kdtree, L2_vs_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int knn = 1; knn < 20; knn += 3)
    {
        for (int i = 0; i < 500; i++)
        {
            L2_vs_bruteforce_test<float>(100, 2, knn);
            L2_vs_bruteforce_test<float>(100, 3, knn);
            L2_vs_bruteforce_test<float>(100, 7, knn);

            L2_vs_bruteforce_test<double>(100, 2, knn);
            L2_vs_bruteforce_test<double>(100, 3, knn);
            L2_vs_bruteforce_test<double>(100, 7, knn);
        }
    }
}

TEST(kdtree, L2_vs_bruteforce_rknn)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int knn = 1; knn < 20; knn += 3)
    {
        for (int r = 1; r < 5; r++)
        {
            for (int i = 0; i < 100; i++)
            {
                rknn_L2_vs_bruteforce_test<float>(100, 2, knn, 9.0f * r * r);
                rknn_L2_vs_bruteforce_test<float>(100, 3, knn, 9.0f * r * r);
                rknn_L2_vs_bruteforce_test<float>(100, 7, knn, 9.0f * r * r);

                rknn_L2_vs_bruteforce_test<double>(100, 2, knn, 9.0 * r * r);
                rknn_L2_vs_bruteforce_test<double>(100, 3, knn, 9.0 * r * r);
                rknn_L2_vs_bruteforce_test<double>(100, 7, knn, 9.0 * r * r);
            }
        }
    }
}

TEST(kdtree, SO3_vs_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 10; i++)
    {
        SO3_vs_bruteforce_test<float>(100);
        SO3_vs_bruteforce_test<float>(100);
        SO3_vs_bruteforce_test<float>(100);

        SO3_vs_bruteforce_test<double>(100);
        SO3_vs_bruteforce_test<double>(100);
        SO3_vs_bruteforce_test<double>(100);
    }
}

TEST(kdtree, SO2_vs_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 10; i++)
    {
        SO2_vs_bruteforce_test<float>(100);
        SO2_vs_bruteforce_test<float>(100);
        SO2_vs_bruteforce_test<float>(100);

        SO2_vs_bruteforce_test<double>(100);
        SO2_vs_bruteforce_test<double>(100);
        SO2_vs_bruteforce_test<double>(100);
    }
}

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

TEST(kdtree, robust_nonempty_tree)
{
    // Try to build a dynamic tree with some initial points
    PointCloud<double> cloud;
    const size_t       max_point_count = 1000;
    generateRandomPointCloud(cloud, max_point_count);

    const PointCloud<double>::Point query_pt{0.5, 0.5, 0.5};

    // construct a kd-tree index:
    typedef KDTreeSingleIndexDynamicAdaptor<
        L2_Simple_Adaptor<double, PointCloud<double>>, PointCloud<double>, 3>
        my_kd_tree_simple_t;

    my_kd_tree_simple_t index1(
        3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */),
        max_point_count);

    // Try a search and expect a neighbor to exist because the dynamic tree was
    // passed a non-empty cloud
    const size_t                    num_results = 1;
    std::vector<size_t>             ret_index(num_results);
    std::vector<double>             out_dist_sqr(num_results);
    nanoflann::KNNResultSet<double> resultSet(num_results);
    resultSet.init(&ret_index[0], &out_dist_sqr[0]);
    bool result = index1.findNeighbors(resultSet, query_pt);
    EXPECT_EQ(result, true);
}

TEST(kdtree, add_and_remove_points)
{
    PointCloud<double> cloud;
    cloud.pts = {{0.0, 0.0, 0.0}, {0.5, 0.5, 0.5}, {0.7, 0.7, 0.7}};

    typedef KDTreeSingleIndexDynamicAdaptor<
        L2_Simple_Adaptor<double, PointCloud<double>>, PointCloud<double>, 3>
        my_kd_tree_simple_t;

    my_kd_tree_simple_t index(
        3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

    const auto query = [&index]() -> size_t
    {
        const PointCloud<double>::Point query_pt{0.5, 0.5, 0.5};
        const size_t                    num_results = 1;
        std::vector<size_t>             ret_index(num_results);
        std::vector<double>             out_dist_sqr(num_results);
        nanoflann::KNNResultSet<double> resultSet(num_results);

        resultSet.init(&ret_index[0], &out_dist_sqr[0]);
        index.findNeighbors(resultSet, query_pt);

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

TEST(kdtree, L2_concurrent_build_vs_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 10; i++)
    {
        L2_concurrent_build_vs_bruteforce_test<float>(100, 2);
        L2_concurrent_build_vs_bruteforce_test<float>(100, 3);
        L2_concurrent_build_vs_bruteforce_test<float>(100, 7);

        L2_concurrent_build_vs_bruteforce_test<double>(100, 2);
        L2_concurrent_build_vs_bruteforce_test<double>(100, 3);
        L2_concurrent_build_vs_bruteforce_test<double>(100, 7);
    }
}

TEST(kdtree, L2_concurrent_build_vs_L2)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 10; i++)
    {
        L2_concurrent_build_vs_L2_test<float>(100, 2);
        L2_concurrent_build_vs_L2_test<float>(100, 3);
        L2_concurrent_build_vs_L2_test<float>(100, 7);

        L2_concurrent_build_vs_L2_test<double>(100, 2);
        L2_concurrent_build_vs_L2_test<double>(100, 3);
        L2_concurrent_build_vs_L2_test<double>(100, 7);
    }
}

TEST(kdtree, RadiusSearch)
{
    radius_search_test<float, false>();
    radius_search_test<double, false>();

    radius_search_test<float, true>();
    radius_search_test<double, true>();

    radius_search_dynamic_test<float, false>();
    radius_search_dynamic_test<double, false>();

    radius_search_dynamic_test<float, true>();
    radius_search_dynamic_test<double, true>();
}

TEST(kdtree, BoxSearch)
{
    box_search_test<float, false>();
    box_search_test<double, false>();

    box_search_test<float, true>();
    box_search_test<double, true>();
}

// The test is about verification of building kd-tree with identical points for which the function KDTreeBaseClass::planeSplit crashed before fixing it.
TEST(kdtree, OverlapWithLooseTree)
{
    using num_t = double;
    PointCloud<num_t> cloud;
    cloud.pts = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<num_t, PointCloud<num_t>>, PointCloud<num_t>,
        3 /* dim */, uint32_t, true>
        my_kd_tree_simple_t;

    my_kd_tree_simple_t index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(3 /* max leaf */, KDTreeSingleIndexAdaptorFlags::SkipInitialBuildIndex));
    index.buildIndex();
    EXPECT_TRUE(index.root_node_ != nullptr);
}
