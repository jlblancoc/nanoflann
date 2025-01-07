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
#include <fstream>
#include <iostream>
#include <nanoflann.hpp>

#include "utils.h"

void kdtree_save_load_demo(const size_t N)
{
    PointCloud<double> cloud;

    // Generate points:
    generateRandomPointCloud(cloud, N);

    double query_pt[3] = {0.5, 0.5, 0.5};

    // construct a kd-tree index:
    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloud<double>>,
        PointCloud<double>, 3 /* dim */
        >;

    // Construct the index and save it:
    // --------------------------------------------
    {
        my_kd_tree_t index(
            3 /*dim*/, cloud,
            nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

        std::ofstream f("index.bin", std::ofstream::binary);

        if (f.bad()) throw std::runtime_error("Error writing index file!");

        index.saveIndex(f);
        f.close();
    }

    // Load the index from disk:
    // --------------------------------------------
    {
        // Important: construct the index associated to the same dataset, since
        // data points are NOT stored in the binary file.
        // Note - set KDTreeSingleIndexAdaptor::SkipInitialBuildIndex, otherwise
        // the tree builds an index that'll be overwritten via loadIndex
        my_kd_tree_t index(
            3 /*dim*/, cloud,
            nanoflann::KDTreeSingleIndexAdaptorParams(
                10 /* max leaf */, nanoflann::KDTreeSingleIndexAdaptorFlags::
                                       SkipInitialBuildIndex));

        std::ifstream f("index.bin", std::ofstream::binary);

        if (f.fail()) throw std::runtime_error("Error reading index file!");

        index.loadIndex(f);
        f.close();

        // do a knn search
        const size_t                    num_results = 1;
        size_t                          ret_index;
        double                          out_dist_sqr;
        nanoflann::KNNResultSet<double> resultSet(num_results);
        resultSet.init(&ret_index, &out_dist_sqr);
        index.findNeighbors(resultSet, &query_pt[0]);

        std::cout << "knnSearch(nn=" << num_results << "): \n";
        std::cout << "ret_index=" << ret_index
                  << " out_dist_sqr=" << out_dist_sqr << std::endl;
    }

    // Stress test: try to save an empty index
    {
        PointCloud<double> emptyCloud;
        my_kd_tree_t       index(3 /*dim*/, emptyCloud);
        std::ofstream      f("index2.bin", std::ofstream::binary);
        if (f.bad()) throw std::runtime_error("Error writing index file!");
        index.saveIndex(f);
        f.close();
    }
}

int main()
{
    // Randomize Seed
    srand(static_cast<unsigned int>(time(nullptr)));
    kdtree_save_load_demo(100000);
    return 0;
}
