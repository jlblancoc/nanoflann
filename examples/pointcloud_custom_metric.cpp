/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2025 Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
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

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <nanoflann.hpp>

#include "utils.h"

using namespace std;
using namespace nanoflann;

// This example demonstrate how to embed a custom parameter "myParam" into
// the metric class My_Custom_Metric_Adaptor, whose constructor accepts
// arbitrary parameters:

template <
    class T, class DataSource, typename _DistanceType = T,
    typename IndexType = uint32_t>
struct My_Custom_Metric_Adaptor
{
    using ElementType  = T;
    using DistanceType = _DistanceType;

    const DataSource& data_source;

    double _myParam = 1.0;

    My_Custom_Metric_Adaptor(const DataSource& _data_source, double myParam)
        : data_source(_data_source), _myParam(myParam)
    {
    }

    inline DistanceType evalMetric(
        const T* a, const IndexType b_idx, size_t size) const
    {
        DistanceType result = DistanceType();
        for (size_t i = 0; i < size; ++i)
        {
            const DistanceType diff =
                a[i] - data_source.kdtree_get_pt(b_idx, i);
            result += std::pow(diff, _myParam);
        }
        return result;
    }

    template <typename U, typename V>
    inline DistanceType accum_dist(const U a, const V b, const size_t) const
    {
        return std::pow((a - b), _myParam);
    }
};

static void kdtree_custom_metric_demo(const size_t N)
{
    using num_t = double;

    PointCloud<num_t> cloud;

    // Generate points:
    generateRandomPointCloud(cloud, N);

    num_t query_pt[3] = {0.5, 0.5, 0.5};

    // construct a kd-tree index:
    using my_kd_tree_t = KDTreeSingleIndexAdaptor<
        My_Custom_Metric_Adaptor<num_t, PointCloud<num_t>>, PointCloud<num_t>,
        3 /* dim */
        >;

    dump_mem_usage();

    // This will be forwarded to the metric class:
    const double myMetricParam = 4.0;

    my_kd_tree_t index(3 /*dim*/, cloud, {10 /* max leaf */}, myMetricParam);

    dump_mem_usage();
    {
        // do a knn search
        const size_t                   num_results = 1;
        size_t                         ret_index;
        num_t                          out_dist_sqr;
        nanoflann::KNNResultSet<num_t> resultSet(num_results);
        resultSet.init(&ret_index, &out_dist_sqr);
        index.findNeighbors(resultSet, &query_pt[0]);

        std::cout << "knnSearch(nn=" << num_results << "\n";
        std::cout << "ret_index=" << ret_index
                  << " out_dist_sqr=" << out_dist_sqr << endl;
    }
    {
        // Unsorted radius search:
        const num_t                                       radius = 1;
        std::vector<nanoflann::ResultItem<size_t, num_t>> indices_dists;
        RadiusResultSet<num_t, size_t> resultSet(radius, indices_dists);

        index.findNeighbors(resultSet, query_pt);

        // Get worst (furthest) point, without sorting:
        nanoflann::ResultItem<size_t, num_t> worst_pair =
            resultSet.worst_item();
        cout << "Worst pair: idx=" << worst_pair.first
             << " dist=" << worst_pair.second << endl;
    }
}

int main()
{
    // Randomize Seed
    srand(static_cast<unsigned int>(time(nullptr)));
    kdtree_custom_metric_demo(10000);
    return 0;
}
