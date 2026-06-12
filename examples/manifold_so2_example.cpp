/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2026 Jose Luis Blanco (joseluisblancoc@gmail.com).
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

// Example: exact nearest-neighbor search on the circle SO(2) (one angular
// coordinate in [-pi, pi], wrapping at the +/-pi seam) using the compile-time
// product-manifold topology feature of nanoflann 2.0. This replaces the legacy
// nanoflann::SO2_Adaptor (removed in 2.0), whose Euclidean pruning could miss
// the true neighbor across the +/-pi seam.

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <nanoflann.hpp>

#include "utils.h"

#if !defined(NANOFLANN_HAS_MANIFOLDS)
int main()
{
    std::cerr << "This example requires C++17 (product-manifold topology).\n";
    return 0;
}
#else

namespace
{

template <typename num_t>
void kdtree_demo(const size_t N)
{
    PointCloud_Orient<num_t> cloud;

    // Generate points:
    generateRandomPointCloud_Orient(cloud, N);

    num_t query_pt[1] = {0.5};

    // Declare the state-space topology once, at compile time:
    using StateSpace   = nanoflann::SO2;  // 1 angular coordinate, wraps at +/-pi
    using metric_t     = nanoflann::Manifold_Adaptor<StateSpace, num_t, PointCloud_Orient<num_t>>;
    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        metric_t, PointCloud_Orient<num_t>, StateSpace::ambient /* = 1 */>;

    dump_mem_usage();

    my_kd_tree_t index(StateSpace::ambient, cloud, {10 /* max leaf */});

    dump_mem_usage();
    {
        // do a knn search
        const size_t                   num_results = 1;
        size_t                         ret_index;
        num_t                          out_dist_sqr;
        nanoflann::KNNResultSet<num_t> resultSet(num_results);
        resultSet.init(&ret_index, &out_dist_sqr);
        index.findNeighbors(resultSet, &query_pt[0]);

        std::cout << "knnSearch(nn=" << num_results << "): \n";
        std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << std::endl;
    }
}
}  // namespace

int main()
{
    try
    {
        // Randomize Seed
        srand(static_cast<unsigned int>(time(nullptr)));
        kdtree_demo<float>(1000000);
        kdtree_demo<double>(1000000);
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
#endif  // NANOFLANN_HAS_MANIFOLDS
