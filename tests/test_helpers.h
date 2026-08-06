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

#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cmath>  // for abs()
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <nanoflann.hpp>
#include <random>
#include <set>
#include <vector>

#include "../examples/utils.h"

using namespace std;
using namespace nanoflann;

using namespace nanoflann;
#include "../examples/KDTreeVectorOfVectorsAdaptor.h"

// ---------------------------------------------------------------------------
// Template helpers shared across multiple translation units
// ---------------------------------------------------------------------------

template <typename NUM>
void generateRandomPointCloud(
    std::vector<std::vector<NUM>>& samples, const size_t N, const size_t dim, const NUM max_range)
{
    samples.resize(N);
    for (size_t i = 0; i < N; i++)
    {
        samples[i].resize(dim);
        for (size_t d = 0; d < dim; d++)
            samples[i][d] = max_range * NUM(rand() % 1000) / NUM(1000.0);
    }
}

template <typename num_t>
void L2_vs_L2_simple_test(const size_t N, const size_t num_results)
{
    PointCloud<num_t> cloud;

    // Generate points:
    generateRandomPointCloud(cloud, N);

    num_t query_pt[3] = {0.5, 0.5, 0.5};

    // construct a kd-tree index:
    using my_kd_tree_simple_t = KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<num_t, PointCloud<num_t>>, PointCloud<num_t>, 3 /* dim */
        >;

    using my_kd_tree_t = KDTreeSingleIndexAdaptor<
        L2_Adaptor<num_t, PointCloud<num_t>>, PointCloud<num_t>, 3 /* dim */
        >;

    my_kd_tree_simple_t index1(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

    my_kd_tree_t index2(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

    // do a knn search
    std::vector<size_t>            ret_index(num_results);
    std::vector<num_t>             out_dist_sqr(num_results);
    nanoflann::KNNResultSet<num_t> resultSet(num_results);
    resultSet.init(&ret_index[0], &out_dist_sqr[0]);
    index1.findNeighbors(resultSet, &query_pt[0]);

    std::vector<size_t> ret_index1    = ret_index;
    std::vector<num_t>  out_dist_sqr1 = out_dist_sqr;

    resultSet.init(&ret_index[0], &out_dist_sqr[0]);

    index2.findNeighbors(resultSet, &query_pt[0]);

    if (N >= num_results)
    {
        EXPECT_EQ(resultSet.size(), num_results);
    }
    else
    {
        EXPECT_EQ(resultSet.size(), N);
    }

    for (size_t i = 0; i < resultSet.size(); i++)
    {
        EXPECT_EQ(ret_index1[i], ret_index[i]);
        EXPECT_NEAR(out_dist_sqr1[i], out_dist_sqr[i], 1e-3);
    }
    // Ensure results are sorted:
    num_t lastDist = -1;
    for (size_t i = 0; i < resultSet.size(); i++)
    {
        const num_t newDist = out_dist_sqr[i];
        EXPECT_GE(newDist, lastDist);
        lastDist = newDist;
    }

    // Test "RadiusResultSet" too:
    const num_t maxRadiusSqrSearch = 10.0 * 10.0;

    std::vector<nanoflann::ResultItem<
        typename my_kd_tree_simple_t::IndexType, typename my_kd_tree_simple_t::DistanceType>>
        radiusIdxs;

    nanoflann::RadiusResultSet<num_t, typename my_kd_tree_simple_t::IndexType> radiusResults(
        maxRadiusSqrSearch, radiusIdxs);
    radiusResults.init();
    nanoflann::SearchParameters searchParams;
    searchParams.sorted = true;
    index1.findNeighbors(radiusResults, &query_pt[0], searchParams);

    // Ensure results are sorted:
    lastDist = -1;
    for (const auto& r : radiusIdxs)
    {
        const num_t newDist = r.second;
        EXPECT_GE(newDist, lastDist);
        lastDist = newDist;
    }
}

template <typename NUM>
void L1_vs_bruteforce_test(const size_t nSamples, const size_t DIM, const size_t numToSearch)
{
    std::vector<std::vector<NUM>> samples;

    const NUM max_range = NUM(20.0);

    // Generate points:
    generateRandomPointCloud(samples, nSamples, DIM, max_range);

    // Query point:
    std::vector<NUM> query_pt(DIM);
    for (size_t d = 0; d < DIM; d++) query_pt[d] = max_range * NUM(rand() % 1000) / NUM(1000.0);

    // construct a kd-tree index:
    // Dimensionality set at run-time
    // ------------------------------------------------------------
    typedef KDTreeVectorOfVectorsAdaptor<
        std::vector<std::vector<NUM>>, NUM, -1, nanoflann::metric_L1>
        my_kd_tree_t;

    my_kd_tree_t mat_index(DIM /*dim*/, samples, 10 /* max leaf */);

    // do a knn search
    const size_t        num_results = numToSearch;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<NUM>    out_dists_sqr(num_results);

    nanoflann::KNNResultSet<NUM> resultSet(num_results);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    mat_index.index->findNeighbors(resultSet, &query_pt[0]);

    const auto nFound = resultSet.size();

    EXPECT_TRUE(nFound > 0);
    if (resultSet.full())
    {
        EXPECT_EQ(resultSet.worstDist(), out_dists_sqr.at(nFound - 1));
    }

    // Brute force neighbors:
    std::multimap<NUM /*dist*/, size_t /*idx*/> bf_nn;
    {
        for (size_t i = 0; i < nSamples; i++)
        {
            double dist = 0.0;
            for (size_t d = 0; d < DIM; d++) dist += std::abs(query_pt[d] - samples[i][d]);
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
                << "For: numToSearch=" << numToSearch << " out_dists_sqr[i]=" << out_dists_sqr[i]
                << "\n";
        }
    }
}

template <typename NUM>
void rknn_L1_vs_bruteforce_test(
    const size_t nSamples, const size_t DIM, const size_t numToSearch, const NUM maxRadiusSqr)
{
    std::vector<std::vector<NUM>> samples;

    const NUM max_range = NUM(20.0);

    // Generate points:
    generateRandomPointCloud(samples, nSamples, DIM, max_range);

    // Query point:
    std::vector<NUM> query_pt(DIM);
    for (size_t d = 0; d < DIM; d++) query_pt[d] = max_range * NUM(rand() % 1000) / NUM(1000.0);

    // construct a kd-tree index:
    // Dimensionality set at run-time
    // ------------------------------------------------------------
    typedef KDTreeVectorOfVectorsAdaptor<
        std::vector<std::vector<NUM>>, NUM, -1, nanoflann::metric_L1>
        my_kd_tree_t;

    my_kd_tree_t mat_index(DIM /*dim*/, samples, 10 /* max leaf */);

    // do a knn search
    const size_t        num_results = numToSearch;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<NUM>    out_dists_sqr(num_results);

    nanoflann::RKNNResultSet<NUM> resultSet(num_results, maxRadiusSqr);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    mat_index.index->findNeighbors(resultSet, &query_pt[0]);

    const auto nFound = resultSet.size();

    if (resultSet.full())
    {
        EXPECT_EQ(resultSet.worstDist(), out_dists_sqr.at(nFound - 1));
    }

    // Brute force neighbors:
    std::multimap<NUM /*dist*/, size_t /*idx*/> bf_nn;
    {
        for (size_t i = 0; i < nSamples; i++)
        {
            NUM dist = NUM(0.0);
            for (size_t d = 0; d < DIM; d++)
                dist += static_cast<NUM>(std::abs(query_pt[d] - samples[i][d]));

            if (dist < maxRadiusSqr) bf_nn.emplace(dist, i);
        }
    }

    // Keep bruteforce solutions indexed by idx instead of distances,
    // to handle correctly almost or exactly coindicing distances for >=2 NN:
    std::map<size_t, NUM> bf_idx2dist;
    for (const auto& kv : bf_nn) bf_idx2dist[kv.second] = kv.first;

    // Compare:
    if (!bf_nn.empty())
    {
        const size_t compareCount = std::min<size_t>(nFound, bf_nn.size());
        auto         it           = bf_nn.begin();
        for (size_t i = 0; i < compareCount; ++i, ++it)
        {
            // Distances must be in exact order:
            EXPECT_NEAR(it->first, out_dists_sqr[i], 1e-3)
                << "For: numToSearch=" << numToSearch << " out_dists_sqr[i]=" << out_dists_sqr[i]
                << "\n";

            if (bf_idx2dist.find(ret_indexes[i]) != bf_idx2dist.end())
            {
                EXPECT_NEAR(bf_idx2dist[ret_indexes[i]], out_dists_sqr[i], 1e-3)
                    << "For: numToSearch=" << numToSearch
                    << " out_dists_sqr[i]=" << out_dists_sqr[i] << "\n";
            }
        }
    }
}

template <typename NUM>
void L2_vs_bruteforce_test(const size_t nSamples, const size_t DIM, const size_t numToSearch)
{
    std::vector<std::vector<NUM>> samples;

    const NUM max_range = NUM(20.0);

    // Generate points:
    generateRandomPointCloud(samples, nSamples, DIM, max_range);

    // Query point:
    std::vector<NUM> query_pt(DIM);
    for (size_t d = 0; d < DIM; d++) query_pt[d] = max_range * NUM(rand() % 1000) / NUM(1000.0);

    // construct a kd-tree index:
    // Dimensionality set at run-time (default: L2)
    // ------------------------------------------------------------
    typedef KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<NUM>>, NUM> my_kd_tree_t;

    my_kd_tree_t mat_index(DIM /*dim*/, samples, 10 /* max leaf */);

    // do a knn search
    const size_t        num_results = numToSearch;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<NUM>    out_dists_sqr(num_results);

    nanoflann::KNNResultSet<NUM> resultSet(num_results);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    mat_index.index->findNeighbors(resultSet, &query_pt[0]);

    const auto nFound = resultSet.size();

    EXPECT_TRUE(nFound > 0);
    if (resultSet.full())
    {
        EXPECT_EQ(resultSet.worstDist(), out_dists_sqr.at(nFound - 1));
    }

    // Brute force neighbors:
    std::multimap<NUM /*dist*/, size_t /*idx*/> bf_nn;
    {
        for (size_t i = 0; i < nSamples; i++)
        {
            double dist = 0.0;
            for (size_t d = 0; d < DIM; d++)
                dist += (query_pt[d] - samples[i][d]) * (query_pt[d] - samples[i][d]);
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
                << "For: numToSearch=" << numToSearch << " out_dists_sqr[i]=" << out_dists_sqr[i]
                << "\n";
        }
    }
}

template <typename NUM>
void box_L2_vs_bruteforce_test(const size_t nSamples, const size_t DIM)
{
    std::vector<std::vector<NUM>> samples;

    const NUM max_range = NUM(20.0);

    // Generate points:
    generateRandomPointCloud(samples, nSamples, DIM, max_range);

    typedef KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<NUM>>, NUM> my_kd_tree_t;

    // Query box:
    typename my_kd_tree_t::index_t::BoundingBox query_box(DIM);
    for (size_t d = 0; d < DIM; d++)
    {
        query_box[d].low  = max_range * NUM(rand() % 1000) / NUM(1000.0);
        query_box[d].high = max_range * NUM(rand() % 1000) / NUM(1000.0);
        if (query_box[d].low > query_box[d].high) std::swap(query_box[d].low, query_box[d].high);
    }

    // construct a kd-tree index:
    // Dimensionality set at run-time (default: L2)
    // ------------------------------------------------------------
    my_kd_tree_t mat_index(DIM /*dim*/, samples, 10 /* max leaf */);

    // do a knn search
    std::vector<size_t> ret_indexes(nSamples);
    std::vector<NUM>    out_dists_sqr(nSamples);

    nanoflann::KNNResultSet<NUM> resultSet(nSamples);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    const auto nFound = mat_index.index->findWithinBox(resultSet, query_box);

    // Brute force:
    std::set<size_t /*idx*/> bf_nn;
    for (size_t i = 0; i < nSamples; i++)
    {
        if (mat_index.index->contains(query_box, i)) bf_nn.insert(i);
    }

    // Compare:
    EXPECT_EQ(bf_nn.size(), nFound);

    for (size_t i = 0; i < nFound; ++i)
    {
        EXPECT_TRUE(bf_nn.find(ret_indexes[i]) != bf_nn.end());
    }
}

template <typename NUM>
void rknn_L2_vs_bruteforce_test(
    const size_t nSamples, const size_t DIM, const size_t numToSearch, const NUM maxRadiusSqr)
{
    std::vector<std::vector<NUM>> samples;

    const NUM max_range = NUM(20.0);

    // Generate points:
    generateRandomPointCloud(samples, nSamples, DIM, max_range);

    // Query point:
    std::vector<NUM> query_pt(DIM);
    for (size_t d = 0; d < DIM; d++) query_pt[d] = max_range * NUM(rand() % 1000) / NUM(1000.0);

    // construct a kd-tree index:
    // Dimensionality set at run-time (default: L2)
    // ------------------------------------------------------------
    typedef KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<NUM>>, NUM> my_kd_tree_t;

    my_kd_tree_t mat_index(DIM /*dim*/, samples, 10 /* max leaf */);

    // do a knn search
    const size_t        num_results = numToSearch;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<NUM>    out_dists_sqr(num_results);

    nanoflann::RKNNResultSet<NUM> resultSet(num_results, maxRadiusSqr);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    mat_index.index->findNeighbors(resultSet, &query_pt[0]);

    const auto nFound = resultSet.size();

    if (resultSet.full())
    {
        EXPECT_EQ(resultSet.worstDist(), out_dists_sqr.at(nFound - 1));
    }

    // Brute force neighbors:
    std::multimap<NUM /*dist*/, size_t /*idx*/> bf_nn;
    {
        for (size_t i = 0; i < nSamples; i++)
        {
            double dist = 0.0;
            for (size_t d = 0; d < DIM; d++)
                dist += (query_pt[d] - samples[i][d]) * (query_pt[d] - samples[i][d]);

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
                << "For: numToSearch=" << numToSearch << " out_dists_sqr[i]=" << out_dists_sqr[i]
                << "\n";

            // indices may be not in the (rare) case of a tie:
            EXPECT_NEAR(bf_idx2dist.at(ret_indexes[i]), out_dists_sqr[i], 1e-3)
                << "For: numToSearch=" << numToSearch << " out_dists_sqr[i]=" << out_dists_sqr[i]
                << "\n";
        }
    }
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

    my_kd_tree_t index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

    // Generate points:
    generateRandomPointCloud(cloud, nSamples, max_range);

    NUM query_pt[3] = {0.5, 0.5, 0.5};

    // add points in chunks at a time
    size_t chunk_size = 100;
    size_t end        = 0;
    for (size_t i = 0; i < nSamples / 2; i = i + chunk_size)
    {
        end = min(size_t(i + chunk_size), nSamples / 2 - 1);
        index.addPoints(static_cast<uint32_t>(i), static_cast<uint32_t>(end));
    }

    {
        // do a knn search
        const size_t        num_results = 1;
        std::vector<size_t> ret_indexes(num_results);
        std::vector<NUM>    out_dists_sqr(num_results);

        nanoflann::KNNResultSet<NUM> resultSet(num_results);

        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
        index.findNeighbors(resultSet, &query_pt[0]);

        // Brute force:
        double min_dist_L2 = std::numeric_limits<double>::max();
        size_t min_idx     = std::numeric_limits<size_t>::max();
        {
            for (size_t i = 0; i < nSamples / 2; i++)
            {
                double dist = 0.0;
                for (int d = 0; d < 3; d++)
                    dist += (query_pt[d] - cloud.kdtree_get_pt(i, d)) *
                            (query_pt[d] - cloud.kdtree_get_pt(i, d));
                if (dist < min_dist_L2)
                {
                    min_dist_L2 = dist;
                    min_idx     = i;
                }
            }
            ASSERT_TRUE(min_idx != std::numeric_limits<size_t>::max());
        }
        // Compare distances, not indices: near-ties between equidistant points
        // can make the tree return a different (but equally valid) index.
        EXPECT_NEAR(min_dist_L2, out_dists_sqr[0], 1e-3);
    }
    for (size_t i = end + 1; i < nSamples; i = i + chunk_size)
    {
        end = min(size_t(i + chunk_size), nSamples - 1);
        index.addPoints(static_cast<uint32_t>(i), static_cast<uint32_t>(end));
    }

    {
        // do a knn search
        const size_t        num_results = 1;
        std::vector<size_t> ret_indexes(num_results);
        std::vector<NUM>    out_dists_sqr(num_results);

        nanoflann::KNNResultSet<NUM> resultSet(num_results);

        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
        index.findNeighbors(resultSet, &query_pt[0]);

        // Brute force:
        double min_dist_L2 = std::numeric_limits<double>::max();
        size_t min_idx     = std::numeric_limits<size_t>::max();
        {
            for (size_t i = 0; i < nSamples; i++)
            {
                double dist = 0.0;
                for (int d = 0; d < 3; d++)
                    dist += (query_pt[d] - cloud.kdtree_get_pt(i, d)) *
                            (query_pt[d] - cloud.kdtree_get_pt(i, d));
                if (dist < min_dist_L2)
                {
                    min_dist_L2 = dist;
                    min_idx     = i;
                }
            }
            ASSERT_TRUE(min_idx != std::numeric_limits<size_t>::max());
        }
        // Compare distances, not indices: near-ties between equidistant points
        // can make the tree return a different (but equally valid) index.
        EXPECT_NEAR(min_dist_L2, out_dists_sqr[0], 1e-3);
    }
}

template <typename NUM>
void L2_concurrent_build_vs_bruteforce_test(const size_t nSamples, const size_t DIM)
{
    std::vector<std::vector<NUM>> samples;

    const NUM max_range = NUM(20.0);

    // Generate points:
    generateRandomPointCloud(samples, nSamples, DIM, max_range);

    // Query point:
    std::vector<NUM> query_pt(DIM);
    for (size_t d = 0; d < DIM; d++) query_pt[d] = max_range * NUM(rand() % 1000) / NUM(1000.0);

    // construct a kd-tree index:
    // Dimensionality set at run-time (default: L2)
    // ------------------------------------------------------------
    typedef KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<NUM>>, NUM> my_kd_tree_t;

    my_kd_tree_t mat_index(DIM /*dim*/, samples, 10 /* max leaf */, 0 /* concurrent build */);

    // do a knn search
    const size_t        num_results = 1;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<NUM>    out_dists_sqr(num_results);

    nanoflann::KNNResultSet<NUM> resultSet(num_results);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    mat_index.index->findNeighbors(resultSet, &query_pt[0]);

    // Brute force:
    double min_dist_L2 = std::numeric_limits<double>::max();
    size_t min_idx     = std::numeric_limits<size_t>::max();
    {
        for (size_t i = 0; i < nSamples; i++)
        {
            double dist = 0.0;
            for (size_t d = 0; d < DIM; d++)
                dist += (query_pt[d] - samples[i][d]) * (query_pt[d] - samples[i][d]);
            if (dist < min_dist_L2)
            {
                min_dist_L2 = dist;
                min_idx     = i;
            }
        }
        ASSERT_TRUE(min_idx != std::numeric_limits<size_t>::max());
    }

    // Compare distances, not indices: near-ties between equidistant points
    // can make the tree return a different (but equally valid) index.
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
    for (size_t d = 0; d < DIM; d++) query_pt[d] = max_range * NUM(rand() % 1000) / NUM(1000.0);

    // construct a kd-tree index:
    // Dimensionality set at run-time (default: L2)
    // ------------------------------------------------------------
    typedef KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<NUM>>, NUM> my_kd_tree_t;

    my_kd_tree_t mat_index_concurrent_build(
        DIM /*dim*/, samples, 10 /* max leaf */, 0 /* concurrent build */);
    my_kd_tree_t mat_index(DIM /*dim*/, samples, 10 /* max leaf */);

    // Compare:
    EXPECT_EQ(mat_index.index->vAcc_, mat_index_concurrent_build.index->vAcc_);
}

template <typename num_t>
void L2_dynamic_sorted_test(const size_t N, const size_t num_results)
{
    if (num_results == 0) return;

    PointCloud<num_t> cloud;
    generateRandomPointCloud(cloud, N);

    num_t query_pt[3] = {0.5, 0.5, 0.5};

    using DynamicKDTree = KDTreeSingleIndexDynamicAdaptor<
        L2_Adaptor<num_t, PointCloud<num_t>>, PointCloud<num_t>, 3 /* dim */
        >;

    DynamicKDTree dynamic_index(3, cloud);

    // Prepare result containers
    std::vector<size_t>                    dynamic_idx(num_results);
    std::vector<num_t>                     dynamic_dist(num_results);
    KNNResultSet<num_t>                    dynamic_knn_result(num_results);
    std::vector<ResultItem<size_t, num_t>> radius_results_vec;
    RadiusResultSet<num_t, size_t>         dynamic_radius_result(10.0 * 10.0, radius_results_vec);

    // Prepare search params to sort result
    const auto search_params = SearchParameters(0, true);

    dynamic_knn_result.init(&dynamic_idx[0], &dynamic_dist[0]);
    dynamic_radius_result.init();

    dynamic_index.findNeighbors(dynamic_knn_result, &query_pt[0], search_params);
    dynamic_index.findNeighbors(dynamic_radius_result, &query_pt[0], search_params);

    // Check size
    const size_t expected_size = std::min(N, num_results);
    ASSERT_EQ(dynamic_knn_result.size(), expected_size);

    // Ensure knn results are sorted
    num_t last_dist = -1;
    for (size_t i = 0; i < expected_size; i++)
    {
        EXPECT_GE(dynamic_dist[i], last_dist);
        last_dist = dynamic_dist[i];
    }

    // Ensure radius results are sorted
    num_t last = -1;
    for (const auto& r : radius_results_vec)
    {
        EXPECT_GE(r.second, last);
        last = r.second;
    }
}

// ---------------------------------------------------------------------------
// Shared types for kdtree_incremental tests
// ---------------------------------------------------------------------------
using inc_cloud_t = PointCloud<double>;
using inc_tree_t  = nanoflann::KDTreeSingleIndexIncrementalAdaptor<
    nanoflann::L2_Simple_Adaptor<double, inc_cloud_t>, inc_cloud_t, 3, uint32_t>;

// Brute-force nearest neighbor over an explicit set of "live" indices.
inline double bruteforce_nn(
    const inc_cloud_t& cloud, const std::set<uint32_t>& live, const double* q, uint32_t& bestIdx)
{
    double best = std::numeric_limits<double>::max();
    bestIdx     = 0;
    for (uint32_t i : live)
    {
        const double dx = q[0] - cloud.pts[i].x, dy = q[1] - cloud.pts[i].y,
                     dz = q[2] - cloud.pts[i].z;
        const double d  = dx * dx + dy * dy + dz * dz;
        if (d < best)
        {
            best    = d;
            bestIdx = i;
        }
    }
    return best;
}

inline bool inc_in_box(const inc_cloud_t& c, uint32_t i, const double lo[3], const double hi[3])
{
    return c.pts[i].x >= lo[0] && c.pts[i].x <= hi[0] && c.pts[i].y >= lo[1] &&
           c.pts[i].y <= hi[1] && c.pts[i].z >= lo[2] && c.pts[i].z <= hi[2];
}
