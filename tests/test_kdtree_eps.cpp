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
//  Tests for eps-approximate searches (SearchParameters::eps > 0).
//
//  A subtree is pruned when mindist * (1 + eps) > worstDist(), so the i-th
//  returned distance is within a factor (1 + eps) of the true i-th distance,
//  in the metric's own units (squared distances for L2). The same bound holds
//  for the manifold metrics, whose pruning bound is admissible too.
// ===========================================================================

#include <functional>

#include "test_helpers.h"

namespace
{
constexpr size_t kNumPoints  = 5000;
constexpr size_t kNumQueries = 200;
constexpr size_t kNumNN      = 5;

// Returns the k smallest distances found by the index under test (sorted).
using knn_fn_t = std::function<std::vector<double>(const double* q, size_t k, float eps)>;
// Exact distance from query q to dataset point i, in the metric's units.
using dist_fn_t = std::function<double(const double* q, size_t i)>;

// Checks the (1 + eps) guarantee for every neighbor of every query against
// brute force, and returns how many queries got a non-exact answer.
size_t checkEpsGuarantee(
    const std::vector<std::vector<double>>& queries, const size_t nPoints, const float eps,
    const knn_fn_t& knn, const dist_fn_t& dist)
{
    size_t inexact = 0;
    for (size_t qi = 0; qi < queries.size(); ++qi)
    {
        const double*       q = queries[qi].data();
        std::vector<double> exact(nPoints);
        for (size_t i = 0; i < nPoints; ++i)
        {
            exact[i] = dist(q, i);
        }
        std::sort(exact.begin(), exact.end());

        const std::vector<double> got = knn(q, kNumNN, eps);
        EXPECT_EQ(got.size(), kNumNN);

        bool isExact = true;
        for (size_t k = 0; k < got.size(); ++k)
        {
            const double tol = 1e-9 * (1.0 + exact[k]);
            EXPECT_GE(got[k], exact[k] - tol) << "query #" << qi << " neighbor #" << k;
            EXPECT_LE(got[k], (1.0 + eps) * exact[k] + tol)
                << "query #" << qi << " neighbor #" << k << " eps=" << eps;
            if (got[k] > exact[k] + tol)
            {
                isExact = false;
            }
        }
        if (!isExact)
        {
            inexact++;
        }
    }
    return inexact;
}

// eps = 0 must be exact; a large eps must still honor the bound and must
// actually change some answers (otherwise the parameter would be ignored).
void runEpsChecks(
    const std::vector<std::vector<double>>& queries, const size_t nPoints, const knn_fn_t& knn,
    const dist_fn_t& dist)
{
    EXPECT_EQ(checkEpsGuarantee(queries, nPoints, 0.0f, knn, dist), 0u);
    checkEpsGuarantee(queries, nPoints, 0.5f, knn, dist);
    EXPECT_GT(checkEpsGuarantee(queries, nPoints, 4.0f, knn, dist), 0u);
}

std::vector<std::vector<double>> randomPoints(
    std::mt19937& rng, const size_t n, const size_t dim, const double lo, const double hi)
{
    std::uniform_real_distribution<double> ud(lo, hi);
    std::vector<std::vector<double>>       pts(n, std::vector<double>(dim));
    for (auto& p : pts)
    {
        for (auto& v : p)
        {
            v = ud(rng);
        }
    }
    return pts;
}

using cloud_t  = PointCloud<double>;
using metric_t = L2_Simple_Adaptor<double, cloud_t>;

cloud_t makeCloud(std::mt19937& rng)
{
    cloud_t cloud;
    for (const auto& p : randomPoints(rng, kNumPoints, 3, 0.0, 10.0))
    {
        cloud.pts.push_back({p[0], p[1], p[2]});
    }
    return cloud;
}

template <class INDEX>
knn_fn_t makeKnnFn(const INDEX& index)
{
    return [&index](const double* q, size_t k, float eps)
    {
        std::vector<size_t>  idx(k);
        std::vector<double>  d(k);
        KNNResultSet<double> rs(k);
        rs.init(idx.data(), d.data());
        index.findNeighbors(rs, q, SearchParameters(eps));
        d.resize(rs.size());
        return d;
    };
}
}  // namespace

TEST(kdtree_eps, static_index)
{
    std::mt19937   rng(1234);
    const cloud_t  cloud   = makeCloud(rng);
    const auto     queries = randomPoints(rng, kNumQueries, 3, 0.0, 10.0);
    const metric_t metric(cloud);

    using index_t = KDTreeSingleIndexAdaptor<metric_t, cloud_t, 3>;
    const index_t index(3, cloud, KDTreeSingleIndexAdaptorParams(10));

    runEpsChecks(
        queries, cloud.pts.size(), makeKnnFn(index),
        [&](const double* q, size_t i) { return metric.evalMetric(q, i, 3); });
}

TEST(kdtree_eps, dynamic_index)
{
    std::mt19937   rng(5678);
    const cloud_t  cloud   = makeCloud(rng);
    const auto     queries = randomPoints(rng, kNumQueries, 3, 0.0, 10.0);
    const metric_t metric(cloud);

    using index_t = KDTreeSingleIndexDynamicAdaptor<metric_t, cloud_t, 3>;
    const index_t index(3, cloud, KDTreeSingleIndexAdaptorParams(10));

    runEpsChecks(
        queries, cloud.pts.size(), makeKnnFn(index),
        [&](const double* q, size_t i) { return metric.evalMetric(q, i, 3); });
}

TEST(kdtree_eps, incremental_index)
{
    std::mt19937   rng(91011);
    const cloud_t  cloud   = makeCloud(rng);
    const auto     queries = randomPoints(rng, kNumQueries, 3, 0.0, 10.0);
    const metric_t metric(cloud);

    using index_t = KDTreeSingleIndexIncrementalAdaptor<metric_t, cloud_t, 3, uint32_t>;
    index_t index(3, cloud);
    // Several batches, so the tree goes through partial rebuilds.
    const uint32_t n = static_cast<uint32_t>(cloud.pts.size());
    for (uint32_t start = 0; start < n; start += 1000)
    {
        index.addPoints(start, std::min(start + 1000, n) - 1);
    }

    runEpsChecks(
        queries, cloud.pts.size(), makeKnnFn(index),
        [&](const double* q, size_t i) { return metric.evalMetric(q, i, 3); });
}

#if defined(NANOFLANN_HAS_MANIFOLDS)
namespace
{
struct VecCloud
{
    std::vector<std::vector<double>> pts;

    size_t kdtree_get_point_count() const { return pts.size(); }
    double kdtree_get_pt(const size_t idx, const size_t dim) const { return pts[idx][dim]; }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const
    {
        return false;
    }
};

// SE(2) points: (x, y) in [0, 10)^2 and a heading in [-pi, pi).
VecCloud makeSE2Cloud(std::mt19937& rng, const size_t n)
{
    VecCloud                               cloud;
    std::uniform_real_distribution<double> ux(0.0, 10.0);
    std::uniform_real_distribution<double> ua(
        -nanoflann::pi_const<double>(), nanoflann::pi_const<double>());
    for (size_t i = 0; i < n; ++i)
    {
        cloud.pts.push_back({ux(rng), ux(rng), ua(rng)});
    }
    return cloud;
}

using se2_metric_t = Manifold_Adaptor<SE2, double, VecCloud>;
}  // namespace

TEST(kdtree_eps, manifold_static_index)
{
    std::mt19937       rng(1213);
    const VecCloud     cloud   = makeSE2Cloud(rng, kNumPoints);
    const VecCloud     queries = makeSE2Cloud(rng, kNumQueries);
    const se2_metric_t metric(cloud);

    using index_t = KDTreeSingleIndexAdaptor<se2_metric_t, VecCloud, 3>;
    const index_t index(3, cloud, KDTreeSingleIndexAdaptorParams(10));

    runEpsChecks(
        queries.pts, cloud.pts.size(), makeKnnFn(index),
        [&](const double* q, size_t i) { return metric.evalMetric(q, i, 3); });
}

TEST(kdtree_eps, manifold_incremental_index)
{
    std::mt19937       rng(1415);
    const VecCloud     cloud   = makeSE2Cloud(rng, kNumPoints);
    const VecCloud     queries = makeSE2Cloud(rng, kNumQueries);
    const se2_metric_t metric(cloud);

    using index_t = KDTreeSingleIndexIncrementalAdaptor<se2_metric_t, VecCloud, 3, uint32_t>;
    index_t index(3, cloud);
    index.addPoints(0, static_cast<uint32_t>(cloud.pts.size()) - 1);

    runEpsChecks(
        queries.pts, cloud.pts.size(), makeKnnFn(index),
        [&](const double* q, size_t i) { return metric.evalMetric(q, i, 3); });
}
#endif  // NANOFLANN_HAS_MANIFOLDS
