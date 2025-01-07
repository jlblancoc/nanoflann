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

template <typename num_t>
void kdtree_demo(const size_t N)
{
    using std::cout;
    using std::endl;

    PointCloud<num_t> cloud;

    // Generate points:
    generateRandomPointCloud(cloud, N);

    // construct a kd-tree index:
    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t>>,
        PointCloud<num_t>, 3 /* dim */
        >;

    my_kd_tree_t index(3 /*dim*/, cloud, {10 /* max leaf */});

#if 0
	// Test resize of dataset and rebuild of index:
	cloud.pts.resize(cloud.pts.size()*0.5);
	index.buildIndex();
#endif

    const num_t query_pt[3] = {0.5, 0.5, 0.5};

    // ----------------------------------------------------------------
    // knnSearch():  Perform a search for the N closest points
    // ----------------------------------------------------------------
    {
        size_t                num_results = 5;
        std::vector<uint32_t> ret_index(num_results);
        std::vector<num_t>    out_dist_sqr(num_results);

        num_results = index.knnSearch(
            &query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

        // In case of less points in the tree than requested:
        ret_index.resize(num_results);
        out_dist_sqr.resize(num_results);

        cout << "knnSearch(): num_results=" << num_results << "\n";
        for (size_t i = 0; i < num_results; i++)
            cout << "idx[" << i << "]=" << ret_index[i] << " dist[" << i
                 << "]=" << out_dist_sqr[i] << endl;
        cout << "\n";
    }

    // ----------------------------------------------------------------
    // radiusSearch(): Perform a search for the points within search_radius
    // ----------------------------------------------------------------
    {
        const num_t search_radius = static_cast<num_t>(0.1);
        std::vector<nanoflann::ResultItem<uint32_t, num_t>> ret_matches;

        // nanoflanSearchParamsameters params;
        // params.sorted = false;

        const size_t nMatches =
            index.radiusSearch(&query_pt[0], search_radius, ret_matches);

        cout << "radiusSearch(): radius=" << search_radius << " -> " << nMatches
             << " matches\n";
        for (size_t i = 0; i < nMatches; i++)
            cout << "idx[" << i << "]=" << ret_matches[i].first << " dist[" << i
                 << "]=" << ret_matches[i].second << endl;
        cout << "\n";
    }
}

int main()
{
    // Randomize Seed
    srand(static_cast<unsigned int>(time(nullptr)));
    kdtree_demo<float>(4);
    kdtree_demo<double>(100000);
    return 0;
}
