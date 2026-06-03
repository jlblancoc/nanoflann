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


#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <nanoflann.hpp>
#include <random>
#include <type_traits>
#include <vector>

using namespace nanoflann;

// ===========================================================================
//  Compile-time product-manifold topology tests (nanoflann 2.0, C++17)
//
//  Every test compares the topology-aware KD-tree against an exhaustive
//  brute-force search using the *same* geodesic product metric, asserting that
//  nanoflann returns the exact nearest neighbors (correct distances) on
//  non-Euclidean state spaces: R^n, SO(2), SO(3), SE(2), SE(3), the torus, and
//  arbitrary products such as SO(3) x SO(3).
// ===========================================================================
#if defined(NANOFLANN_HAS_MANIFOLDS)

namespace manifold_test
{
// A flat point cloud whose points are stored as contiguous scalar coordinates.
template <typename T>
struct ManifoldCloud
{
    std::vector<std::vector<T>> pts;

    inline size_t kdtree_get_point_count() const { return pts.size(); }
    inline T      kdtree_get_pt(const size_t idx, const size_t dim) const { return pts[idx][dim]; }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const
    {
        return false;
    }
};

template <typename T>
T wrap_angle(T d)
{
    const T PI = nanoflann::pi_const<T>();
    while (d > PI) d -= 2 * PI;
    while (d < -PI) d += 2 * PI;
    return d;
}

// Reference squared product-metric distance, independent of nanoflann internals.
template <class Space, typename T>
T reference_dist(const std::vector<T>& a, const std::vector<T>& b)
{
    T s = T(0);
    for (unsigned i = 0; i < Space::ambient;)
    {
        const nanoflann::CoordTopology topo = Space::topology(i);
        if (topo == nanoflann::CoordTopology::Linear)
        {
            const T d = a[i] - b[i];
            s += d * d;
            ++i;
        }
        else if (topo == nanoflann::CoordTopology::Circular)
        {
            const T d = wrap_angle<T>(a[i] - b[i]);
            s += d * d;
            ++i;
        }
        else  // QuaternionBlock (4 coords, double cover)
        {
            T sm = T(0);
            T sp = T(0);
            for (unsigned k = 0; k < 4; ++k)
            {
                const T dm = a[i + k] - b[i + k];
                const T dp = a[i + k] + b[i + k];
                sm += dm * dm;
                sp += dp * dp;
            }
            s += std::min(sm, sp);
            i += 4;
        }
    }
    return s;
}

template <class Space, typename T>
std::vector<T> random_point(std::mt19937& rng)
{
    std::uniform_real_distribution<T> ud(-nanoflann::pi_const<T>(), nanoflann::pi_const<T>());
    std::normal_distribution<T>       nd(T(0), T(1));
    std::vector<T>                    p(Space::ambient);
    for (unsigned i = 0; i < Space::ambient;)
    {
        const nanoflann::CoordTopology topo = Space::topology(i);
        if (topo == nanoflann::CoordTopology::Linear)
        {
            p[i] = ud(rng) * T(3);
            ++i;
        }
        else if (topo == nanoflann::CoordTopology::Circular)
        {
            p[i] = ud(rng);
            ++i;
        }
        else  // unit quaternion
        {
            T q[4];
            T n = T(0);
            for (unsigned k = 0; k < 4; ++k)
            {
                q[k] = nd(rng);
                n += q[k] * q[k];
            }
            n = std::sqrt(n);
            if (n < T(1e-9)) n = T(1);
            for (unsigned k = 0; k < 4; ++k) p[i + k] = q[k] / n;
            i += 4;
        }
    }
    return p;
}

template <class Space, typename T>
void run_bruteforce_check(const size_t nPoints, const size_t nQueries, const size_t kNN)
{
    constexpr int    Dim = static_cast<int>(Space::ambient);
    ManifoldCloud<T> cloud;
    std::mt19937     rng(0xC0FFEEu);
    for (size_t i = 0; i < nPoints; ++i) cloud.pts.push_back(random_point<Space, T>(rng));

    using metric_t = nanoflann::Manifold_Adaptor<Space, T, ManifoldCloud<T>>;
    using index_t  = nanoflann::KDTreeSingleIndexAdaptor<metric_t, ManifoldCloud<T>, Dim>;

    index_t index(Space::ambient, cloud, {10});

    for (size_t q = 0; q < nQueries; ++q)
    {
        const std::vector<T> query = random_point<Space, T>(rng);

        std::vector<size_t>        idx(kNN);
        std::vector<T>             d2(kNN);
        nanoflann::KNNResultSet<T> rs(kNN);
        rs.init(idx.data(), d2.data());
        index.findNeighbors(rs, query.data());

        // Brute-force sorted distances.
        std::vector<T> bf;
        bf.reserve(nPoints);
        for (size_t i = 0; i < nPoints; ++i)
            bf.push_back(reference_dist<Space, T>(query, cloud.pts[i]));
        std::sort(bf.begin(), bf.end());

        const T tol = std::is_same<T, float>::value ? T(1e-3) : T(1e-7);
        for (size_t k = 0; k < kNN; ++k)
        {
            // Tie-robust: compare the k-th returned distance to the k-th
            // brute-force distance (indices may differ on ties, distances cannot).
            ASSERT_NEAR(
                static_cast<double>(bf[k]), static_cast<double>(d2[k]), static_cast<double>(tol))
                << "Space dim=" << Dim << " query #" << q << " neighbor #" << k;
        }
    }
}
}  // namespace manifold_test

TEST(manifold, compile_time_layout)
{
    using namespace nanoflann;
    static_assert(SE2::ambient == 3, "SE2 = R^2 x SO2 -> 3 coords");
    static_assert(SE3::ambient == 7, "SE3 = R^3 x SO3 -> 7 coords");
    static_assert(Product<SO3, SO3>::ambient == 8, "SO3 x SO3 -> 8 coords");
    static_assert(Torus<4>::ambient == 4, "Torus<4> -> 4 coords");
    static_assert(
        is_manifold_metric<
            Manifold_Adaptor<SE3, double, manifold_test::ManifoldCloud<double>>>::value,
        "Manifold_Adaptor must be tagged as a manifold metric");
    static_assert(
        !is_manifold_metric<L2_Adaptor<double, manifold_test::ManifoldCloud<double>>>::value,
        "Euclidean adaptors must NOT be manifold metrics");
    EXPECT_EQ(SE3::topology(0), CoordTopology::Linear);
    EXPECT_EQ(SE3::topology(3), CoordTopology::QuaternionBlock);
}

// The historical SO(2) failure case: a query near +pi whose true nearest
// neighbor lives near -pi (the wrap-around side). The Euclidean pruning bound
// discards it; the topology-aware bound finds it.
TEST(manifold, SO2_wraparound_regression)
{
    using namespace nanoflann;
    using T = double;
    manifold_test::ManifoldCloud<T> cloud;
    cloud.pts.push_back({T(-3.1)});
    cloud.pts.push_back({T(1.0)});

    using metric_t = Manifold_Adaptor<SO2, T, manifold_test::ManifoldCloud<T>>;
    using index_t  = KDTreeSingleIndexAdaptor<metric_t, manifold_test::ManifoldCloud<T>, 1>;
    index_t index(1, cloud, {1});

    const T         query[1] = {T(3.14)};
    size_t          idx;
    T               d2;
    KNNResultSet<T> rs(1);
    rs.init(&idx, &d2);
    index.findNeighbors(rs, query);

    EXPECT_EQ(idx, 0u) << "must return the wrap-around neighbor near -pi";
    const T expected = manifold_test::wrap_angle<T>(T(3.14) - T(-3.1));
    EXPECT_NEAR(d2, expected * expected, 1e-9);
}

TEST(manifold, Rn_vs_bruteforce)
{
    manifold_test::run_bruteforce_check<nanoflann::Rn<3>, double>(3000, 500, 5);
    manifold_test::run_bruteforce_check<nanoflann::Rn<7>, float>(3000, 500, 5);
}
TEST(manifold, SO2_vs_bruteforce)
{
    manifold_test::run_bruteforce_check<nanoflann::SO2, double>(3000, 1000, 5);
    manifold_test::run_bruteforce_check<nanoflann::SO2, float>(3000, 1000, 5);
}
TEST(manifold, SO3_vs_bruteforce)
{
    manifold_test::run_bruteforce_check<nanoflann::SO3, double>(3000, 1000, 5);
    manifold_test::run_bruteforce_check<nanoflann::SO3, float>(3000, 1000, 5);
}
TEST(manifold, SE2_vs_bruteforce)
{
    manifold_test::run_bruteforce_check<nanoflann::SE2, double>(3000, 1000, 5);
}
TEST(manifold, SE3_vs_bruteforce)
{
    manifold_test::run_bruteforce_check<nanoflann::SE3, double>(3000, 1000, 5);
    manifold_test::run_bruteforce_check<nanoflann::SE3, float>(3000, 1000, 5);
}
TEST(manifold, SO3xSO3_vs_bruteforce)
{
    manifold_test::run_bruteforce_check<nanoflann::Product<nanoflann::SO3, nanoflann::SO3>, double>(
        3000, 1000, 5);
}
TEST(manifold, Torus_vs_bruteforce)
{
    manifold_test::run_bruteforce_check<nanoflann::Torus<3>, double>(3000, 1000, 5);
}
TEST(manifold, mixed_product_vs_bruteforce)
{
    using Space = nanoflann::Product<nanoflann::Rn<3>, nanoflann::SO2, nanoflann::SO2>;
    manifold_test::run_bruteforce_check<Space, double>(3000, 1000, 5);
}

#endif  // NANOFLANN_HAS_MANIFOLDS
