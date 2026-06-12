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

#include "test_helpers.h"

TEST(kdtree, L1_vs_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int knn = 1; knn < 20; knn += 3)
    {
        for (int i = 0; i < 500; i++)
        {
            L1_vs_bruteforce_test<float>(10, 2, knn);

            L1_vs_bruteforce_test<float>(100, 2, knn);
            L1_vs_bruteforce_test<float>(100, 3, knn);
            L1_vs_bruteforce_test<float>(100, 7, knn);

            L1_vs_bruteforce_test<double>(100, 2, knn);
            L1_vs_bruteforce_test<double>(100, 3, knn);
            L1_vs_bruteforce_test<double>(100, 7, knn);
        }
    }
}

TEST(kdtree, L1_vs_bruteforce_rknn)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (size_t knn = 1; knn < 20; knn += 3)
    {
        for (int r = 1; r < 5; r++)
        {
            const float rf = static_cast<float>(r);
            for (int i = 0; i < 100; i++)
            {
                rknn_L1_vs_bruteforce_test<float>(100, 2, knn, 9.0f * rf * rf);
                rknn_L1_vs_bruteforce_test<float>(100, 3, knn, 9.0f * rf * rf);
                rknn_L1_vs_bruteforce_test<float>(100, 7, knn, 9.0f * rf * rf);

                rknn_L1_vs_bruteforce_test<double>(100, 2, knn, 9.0 * r * r);
                rknn_L1_vs_bruteforce_test<double>(100, 3, knn, 9.0 * r * r);
                rknn_L1_vs_bruteforce_test<double>(100, 7, knn, 9.0 * r * r);
            }
        }
    }
}

TEST(kdtree, L2_vs_L2_simple)
{
    for (int nResults = 1; nResults < 10; nResults++)
    {
        L2_vs_L2_simple_test<float>(5, nResults);

        L2_vs_L2_simple_test<float>(100, nResults);
        L2_vs_L2_simple_test<double>(100, nResults);
    }
}

TEST(kdtree, robust_empty_tree)
{
    // Try to build a tree with 0 data points, to test
    // robustness against this situation:
    PointCloud<double> cloud;

    double query_pt[3] = {0.5, 0.5, 0.5};

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<double, PointCloud<double>>, PointCloud<double>, 3 /* dim */
        >
        my_kd_tree_simple_t;

    my_kd_tree_simple_t index1(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));

    // Now we will try to search in the tree, and WE EXPECT a result of
    // no neighbors found if the error detection works fine:
    const size_t                    num_results = 1;
    std::vector<size_t>             ret_index(num_results);
    std::vector<double>             out_dist_sqr(num_results);
    nanoflann::KNNResultSet<double> resultSet(num_results);
    resultSet.init(&ret_index[0], &out_dist_sqr[0]);
    bool result = index1.findNeighbors(resultSet, &query_pt[0]);
    EXPECT_EQ(result, false);
}

TEST(kdtree, L2_vs_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int knn = 1; knn < 20; knn += 3)
    {
        for (int i = 0; i < 500; i++)
        {
            L2_vs_bruteforce_test<float>(10, 2, knn);

            L2_vs_bruteforce_test<float>(100, 2, knn);
            L2_vs_bruteforce_test<float>(100, 3, knn);
            L2_vs_bruteforce_test<float>(100, 7, knn);

            L2_vs_bruteforce_test<double>(100, 2, knn);
            L2_vs_bruteforce_test<double>(100, 3, knn);
            L2_vs_bruteforce_test<double>(100, 7, knn);
        }
    }
}

TEST(kdtree, box_L2_vs_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 500; i++)
    {
        box_L2_vs_bruteforce_test<float>(10, 2);

        box_L2_vs_bruteforce_test<float>(100, 2);
        box_L2_vs_bruteforce_test<float>(100, 3);
        box_L2_vs_bruteforce_test<float>(100, 7);

        box_L2_vs_bruteforce_test<double>(100, 2);
        box_L2_vs_bruteforce_test<double>(100, 3);
        box_L2_vs_bruteforce_test<double>(100, 7);
    }
}

TEST(kdtree, L2_vs_bruteforce_rknn)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (size_t knn = 1; knn < 20; knn += 3)
    {
        for (int r = 1; r < 5; r++)
        {
            const float rf = static_cast<float>(r);
            for (int i = 0; i < 100; i++)
            {
                rknn_L2_vs_bruteforce_test<float>(100, 2, knn, 9.0f * rf * rf);
                rknn_L2_vs_bruteforce_test<float>(100, 3, knn, 9.0f * rf * rf);
                rknn_L2_vs_bruteforce_test<float>(100, 7, knn, 9.0f * rf * rf);

                rknn_L2_vs_bruteforce_test<double>(100, 2, knn, 9.0 * r * r);
                rknn_L2_vs_bruteforce_test<double>(100, 3, knn, 9.0 * r * r);
                rknn_L2_vs_bruteforce_test<double>(100, 7, knn, 9.0 * r * r);
            }
        }
    }
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

TEST(kdtree, same_points)
{
    using num_t         = double;
    using point_cloud_t = PointCloud<num_t>;
    using kdtree_t      = KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<num_t, point_cloud_t>, point_cloud_t, 3 /* dim */>;

    point_cloud_t cloud;
    cloud.pts.resize(16);
    for (size_t i = 0; i < 16; ++i)
    {
        cloud.pts[i].x = -1.;
        cloud.pts[i].y = 0.;
        cloud.pts[i].z = 1.;
    }

    kdtree_t idx(3 /*dim*/, cloud);
}
