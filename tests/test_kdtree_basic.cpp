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

TEST(kdtree, SO3_vs_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 10; i++)
    {
        SO3_vs_bruteforce_test<float>(5);

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

// ---------------------------------------------------------------------------
// Regression tests for unsigned integral ElementTypes.
//
// Before the fix that introduced detail::signed_distance_type_for_t<T>,
// instantiating a KDTree with ElementType=uint8_t (or any unsigned integral
// type) segfaulted inside planeSplit() during buildIndex(). These tests
// exercise build + knn + radius for each of uint8_t / uint16_t / uint32_t
// across the L1, L2 and L2_Simple adaptors, plus the metric_L2 traits path
// and a degenerate all-equal-points case.
// ---------------------------------------------------------------------------

TEST(kdtree, unsigned_uint8_L2_Simple_builds_and_matches_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 50; ++i)
    {
        unsigned_kd_vs_bruteforce<uint8_t, L2_Simple_Adaptor, true>(50, 3, 5, 200);
        unsigned_kd_vs_bruteforce<uint8_t, L2_Simple_Adaptor, true>(200, 3, 7, 250);
    }
}

TEST(kdtree, unsigned_uint8_L2_Adaptor_builds_and_matches_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 50; ++i)
    {
        // L2_Adaptor uses the unrolled 4-at-a-time loop, so this exercises
        // the evalMetric fast path in addition to the build code.
        unsigned_kd_vs_bruteforce<uint8_t, L2_Adaptor, true>(50, 3, 5, 200);
    }
}

TEST(kdtree, unsigned_uint8_L1_Adaptor_builds_and_matches_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 50; ++i)
    {
        unsigned_kd_vs_bruteforce<uint8_t, L1_Adaptor, false>(50, 3, 5, 200);
    }
}

TEST(kdtree, unsigned_uint16_L2_Simple_builds_and_matches_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 30; ++i)
    {
        unsigned_kd_vs_bruteforce<uint16_t, L2_Simple_Adaptor, true>(100, 3, 5, 60000);
    }
}

TEST(kdtree, unsigned_uint32_L2_Simple_builds_and_matches_bruteforce)
{
    // Smaller iteration count: uint32_t -> DistanceType=double, and the
    // point of this test is just to exercise the metafunction's uint32_t
    // branch and Oracle's Gap 1 fix (split_val addition overflow).
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 10; ++i)
    {
        unsigned_kd_vs_bruteforce<uint32_t, L2_Simple_Adaptor, true>(50, 3, 3, 4000000000u);
    }
}

TEST(kdtree, unsigned_uint8_via_metric_L2_traits_builds_and_matches_bruteforce)
{
    // Verify the metric_L2 path (which is what KDTreeVectorOfVectorsAdaptor
    // uses internally) also picks up the metafunction-driven DistanceType.
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 30; ++i)
    {
        PointCloud<uint8_t> cloud;
        generateRandomIntegralPointCloud(cloud, 100, uint8_t(250));

        using adaptor_t =
            nanoflann::metric_L2::traits<uint8_t, PointCloud<uint8_t>>::distance_t;
        using tree_t = KDTreeSingleIndexAdaptor<adaptor_t, PointCloud<uint8_t>, 3, size_t>;

        static_assert(
            std::is_signed<typename tree_t::DistanceType>::value,
            "metric_L2::traits must produce a signed DistanceType for uint8_t.");

        tree_t index(3, cloud, KDTreeSingleIndexAdaptorParams(10));
        EXPECT_EQ(cloud.pts.size(), 100u);
    }
}

TEST(kdtree, unsigned_uint8_radius_search_matches_bruteforce)
{
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < 20; ++i)
    {
        unsigned_radius_smoke<uint8_t, L2_Simple_Adaptor>(100, uint8_t(200));
    }
}

TEST(kdtree, unsigned_uint8_all_points_equal)
{
    // Degenerate case that previously triggered the planeSplit underflow:
    // when every point has identical coordinates, all values are equal to
    // cutval, so the partition walks the "else { mid++ }" branch only and
    // never decrements `right`. This must still terminate cleanly and
    // return a valid neighbour.
    PointCloud<uint8_t> cloud;
    cloud.pts.resize(20);
    for (auto& p : cloud.pts)
    {
        p.x = 100;
        p.y = 150;
        p.z = 200;
    }

    using adaptor_t =
        L2_Simple_Adaptor<uint8_t, PointCloud<uint8_t>,
                          nanoflann::detail::signed_distance_type_for_t<uint8_t>, size_t>;
    using tree_t = KDTreeSingleIndexAdaptor<adaptor_t, PointCloud<uint8_t>, 3, size_t>;

    tree_t index(3, cloud, KDTreeSingleIndexAdaptorParams(4));

    uint8_t query[3] = {100, 150, 200};
    size_t idx = 0;
    typename tree_t::DistanceType dist = 0;
    nanoflann::KNNResultSet<typename tree_t::DistanceType> rs(1);
    rs.init(&idx, &dist);
    bool ok = index.findNeighbors(rs, &query[0]);

    EXPECT_TRUE(ok);
    EXPECT_EQ(dist, 0);
}

TEST(kdtree, unsigned_static_assert_blocks_unsigned_distancetype)
{
    // Compile-time check: the metafunction gives unsigned->signed promotion.
    static_assert(
        std::is_same<nanoflann::detail::signed_distance_type_for_t<uint8_t>, int64_t>::value,
        "uint8_t must map to int64_t");
    static_assert(
        std::is_same<nanoflann::detail::signed_distance_type_for_t<uint16_t>, int64_t>::value,
        "uint16_t must map to int64_t");
    static_assert(
        std::is_same<nanoflann::detail::signed_distance_type_for_t<uint32_t>, double>::value,
        "uint32_t must map to double (squared max 2^64 overflows int64_t)");
    static_assert(
        std::is_same<nanoflann::detail::signed_distance_type_for_t<uint64_t>, double>::value,
        "uint64_t must map to double");
    static_assert(
        std::is_same<nanoflann::detail::signed_distance_type_for_t<double>, double>::value,
        "double must map to itself");
    static_assert(
        std::is_same<nanoflann::detail::signed_distance_type_for_t<float>, float>::value,
        "float must map to itself");
    static_assert(
        std::is_same<nanoflann::detail::signed_distance_type_for_t<int32_t>, int32_t>::value,
        "int32_t must map to itself (signed small types are not widened)");
    SUCCEED();
}
