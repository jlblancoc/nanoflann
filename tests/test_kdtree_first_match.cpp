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

// ===========================================================================
//  Tie-breaking under NANOFLANN_FIRST_MATCH.
//
//  The macro changes inline code shared by every translation unit, so these
//  tests are built into their own executable with the macro defined (and also
//  run, with weaker checks, in the main suite without it).
// ===========================================================================

#include "test_helpers.h"

namespace
{
using cloud_t = PointCloud<double>;

// n points evenly spread on the unit circle (z = 0), stored in a shuffled
// index order, so ties at the query (the origin) span many leaves.
cloud_t makeCircleCloud(const size_t n, const unsigned seed)
{
    std::vector<size_t> order(n);
    for (size_t i = 0; i < n; ++i)
    {
        order[i] = i;
    }
    std::mt19937 rng(seed);
    std::shuffle(order.begin(), order.end(), rng);

    cloud_t cloud;
    cloud.pts.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
        // Angles that are exact multiples of pi/2 make the distances exactly 1.
        const double ang = 2 * nanoflann::pi_const<double>() * double(order[i] % 4) / 4.0;
        cloud.pts[i]     = {std::cos(ang), std::sin(ang), 0.0};
    }
    return cloud;
}

template <class INDEX>
void checkTies(const INDEX& index, const cloud_t& cloud, const size_t k)
{
    const double         q[3] = {0, 0, 0};
    std::vector<size_t>  idx(k);
    std::vector<double>  d(k);
    KNNResultSet<double> rs(k);
    rs.init(idx.data(), d.data());
    index.findNeighbors(rs, q);
    ASSERT_EQ(rs.size(), k);

    // All candidates are tied: every returned distance is the same.
    for (size_t i = 0; i < k; ++i)
    {
        EXPECT_NEAR(d[i], d[0], 1e-12);
    }
#ifdef NANOFLANN_FIRST_MATCH
    // The k smallest indices, in ascending order.
    for (size_t i = 0; i < k; ++i)
    {
        EXPECT_EQ(idx[i], i) << "k=" << k << " n=" << cloud.pts.size();
    }
#else
    (void)cloud;
#endif
}
}  // namespace

TEST(kdtree_first_match, static_index_ties)
{
    for (const size_t n : {8u, 64u, 1000u})
    {
        const cloud_t cloud = makeCircleCloud(n, 3);
        using index_t = KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, cloud_t>, cloud_t, 3>;
        const index_t index(3, cloud, KDTreeSingleIndexAdaptorParams(1));
        for (const size_t k : {1u, 3u, 7u})
        {
            checkTies(index, cloud, k);
        }
    }
}

TEST(kdtree_first_match, incremental_index_ties)
{
    for (const size_t n : {8u, 64u, 1000u})
    {
        const cloud_t cloud = makeCircleCloud(n, 5);
        using index_t       = KDTreeSingleIndexIncrementalAdaptor<
            L2_Simple_Adaptor<double, cloud_t>, cloud_t, 3, uint32_t>;
        index_t index(3, cloud);
        index.addPoints(0, static_cast<uint32_t>(n - 1));
        for (const size_t k : {1u, 3u, 7u})
        {
            checkTies(index, cloud, k);
        }
    }
}
