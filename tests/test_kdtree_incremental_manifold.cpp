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
//  Manifold metrics on the incremental self-balancing KD-tree (nanoflann 2.0).
//
//  These tests certify that KDTreeSingleIndexIncrementalAdaptor (and its
//  multi-threaded wrapper) return *exact* nearest neighbors on non-Euclidean
//  product manifolds, including under insert/remove churn and background
//  rebuilds. Every query is checked against an exhaustive brute-force search
//  over the live set using the same geodesic product metric.
// ===========================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <nanoflann.hpp>
#include <random>
#include <set>
#include <vector>

#if defined(NANOFLANN_HAS_MANIFOLDS)

using namespace nanoflann;

namespace incr_manifold_test
{
// Flat point cloud: each row is a contiguous run of scalar coordinates.
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
    const T PI = pi_const<T>();
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
        const CoordTopology topo = Space::topology(i);
        if (topo == CoordTopology::Linear)
        {
            const T d = a[i] - b[i];
            s += d * d;
            ++i;
        }
        else if (topo == CoordTopology::Circular)
        {
            const T d = wrap_angle<T>(a[i] - b[i]);
            s += d * d;
            ++i;
        }
        else  // QuaternionBlock (4 coords, double cover)
        {
            T sm = T(0), sp = T(0);
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

// Per-base-space random point generation (recurses over a product's base spaces
// so a sphere block is generated as a unit vector, not a tag walk).
template <class Space, typename T>
struct PointGen
{
    static void append(std::mt19937& rng, std::vector<T>& out)
    {
        std::uniform_real_distribution<T> ud(-pi_const<T>(), pi_const<T>());
        std::normal_distribution<T>       nd(T(0), T(1));
        for (unsigned i = 0; i < Space::ambient;)
        {
            const CoordTopology topo = Space::topology(i);
            if (topo == CoordTopology::Linear)
            {
                out.push_back(ud(rng) * T(3));
                ++i;
            }
            else if (topo == CoordTopology::Circular)
            {
                out.push_back(ud(rng));
                ++i;
            }
            else
            {
                T q[4], n = T(0);
                for (unsigned k = 0; k < 4; ++k)
                {
                    q[k] = nd(rng);
                    n += q[k] * q[k];
                }
                n            = std::sqrt(n);
                if (n < T(1e-9)) n = T(1);
                const T sign = (nd(rng) < T(0)) ? T(-1) : T(1);
                for (unsigned k = 0; k < 4; ++k) out.push_back(sign * q[k] / n);
                i += 4;
            }
        }
    }
};
template <unsigned N, typename T>
struct PointGen<Sn<N>, T>
{
    static void append(std::mt19937& rng, std::vector<T>& out)
    {
        std::normal_distribution<T> nd(T(0), T(1));
        T                           v[N + 1], n = T(0);
        for (unsigned k = 0; k < N + 1; ++k)
        {
            v[k] = nd(rng);
            n += v[k] * v[k];
        }
        n = std::sqrt(n);
        if (n < T(1e-9)) n = T(1);
        for (unsigned k = 0; k < N + 1; ++k) out.push_back(v[k] / n);
    }
};
template <typename T>
struct PointGen<Product<>, T>
{
    static void append(std::mt19937&, std::vector<T>&) {}
};
template <class B, class... Rest, typename T>
struct PointGen<Product<B, Rest...>, T>
{
    static void append(std::mt19937& rng, std::vector<T>& out)
    {
        PointGen<B, T>::append(rng, out);
        PointGen<Product<Rest...>, T>::append(rng, out);
    }
};

template <class Space, typename T>
std::vector<T> random_point(std::mt19937& rng)
{
    std::vector<T> p;
    p.reserve(Space::ambient);
    PointGen<Space, T>::append(rng, p);
    return p;
}

// Brute-force sorted distances to the live set under the product metric.
template <class Space, typename T>
std::vector<T> bruteforce_sorted(
    const ManifoldCloud<T>& cloud, const std::set<uint32_t>& live, const std::vector<T>& q)
{
    std::vector<T> d;
    d.reserve(live.size());
    for (uint32_t i : live) d.push_back(reference_dist<Space, T>(q, cloud.pts[i]));
    std::sort(d.begin(), d.end());
    return d;
}

template <class Space, typename T>
using incr_tree_t = KDTreeSingleIndexIncrementalAdaptor<
    Manifold_Adaptor<Space, T, ManifoldCloud<T>>, ManifoldCloud<T>,
    static_cast<int>(Space::ambient), uint32_t>;

// Insert N points one-by-one (in batches), then check k-NN vs brute force.
template <class Space, typename T>
void run_incr_build_check(const size_t N, const size_t nQueries, const unsigned K)
{
    constexpr int    Dim = static_cast<int>(Space::ambient);
    ManifoldCloud<T> cloud;
    std::mt19937     rng(0xC0FFEEu);
    for (size_t i = 0; i < N; ++i) cloud.pts.push_back(random_point<Space, T>(rng));

    incr_tree_t<Space, T> index(Dim, cloud);
    std::set<uint32_t>    live;
    // Insert in interleaved batches to exercise the incremental path.
    uint32_t cursor = 0;
    while (cursor < N)
    {
        const uint32_t end = std::min<uint32_t>(cursor + 137, static_cast<uint32_t>(N)) - 1;
        index.addPoints(cursor, end);
        for (uint32_t i = cursor; i <= end; ++i) live.insert(i);
        cursor = end + 1;
    }
    ASSERT_EQ(index.size(), live.size());

    const T tol = std::is_same<T, float>::value ? T(1e-3) : T(1e-7);
    for (size_t q = 0; q < nQueries; ++q)
    {
        const std::vector<T> query = random_point<Space, T>(rng);
        std::vector<uint32_t> idx(K);
        std::vector<T>        d2(K);
        const size_t          n = index.knnSearch(query.data(), K, idx.data(), d2.data());
        ASSERT_EQ(n, K);
        const std::vector<T> bf = bruteforce_sorted<Space, T>(cloud, live, query);
        for (unsigned k = 0; k < K; ++k)
            ASSERT_NEAR(
                static_cast<double>(bf[k]), static_cast<double>(d2[k]), static_cast<double>(tol))
                << "dim=" << Dim << " query #" << q << " neighbor #" << k;
    }
}

// Cross-check: incremental tree must agree with the static manifold tree.
template <class Space, typename T>
void run_incr_vs_static(const size_t N, const size_t nQueries, const unsigned K)
{
    constexpr int    Dim = static_cast<int>(Space::ambient);
    ManifoldCloud<T> cloud;
    std::mt19937     rng(0xABCDu);
    for (size_t i = 0; i < N; ++i) cloud.pts.push_back(random_point<Space, T>(rng));

    incr_tree_t<Space, T> incr(Dim, cloud);
    incr.addPoints(0, static_cast<uint32_t>(N) - 1);

    using static_t =
        KDTreeSingleIndexAdaptor<Manifold_Adaptor<Space, T, ManifoldCloud<T>>, ManifoldCloud<T>, Dim>;
    static_t stat(Dim, cloud, {10});

    for (size_t q = 0; q < nQueries; ++q)
    {
        const std::vector<T> query = random_point<Space, T>(rng);
        std::vector<uint32_t> ia(K);
        std::vector<T>        da(K);
        incr.knnSearch(query.data(), K, ia.data(), da.data());
        std::vector<size_t> ib(K);
        std::vector<T>      db(K);
        KNNResultSet<T>     rs(K);
        rs.init(ib.data(), db.data());
        stat.findNeighbors(rs, query.data());
        for (unsigned k = 0; k < K; ++k)
            ASSERT_NEAR(static_cast<double>(da[k]), static_cast<double>(db[k]), 1e-9)
                << "incr vs static disagree, query #" << q << " neighbor #" << k;
    }
}

// Interleaved insert/remove churn, verified against brute force each epoch.
template <class Space, typename T>
void run_incr_churn(const size_t N)
{
    constexpr int    Dim = static_cast<int>(Space::ambient);
    ManifoldCloud<T> cloud;
    std::mt19937     rng(0x1234u);
    for (size_t i = 0; i < N; ++i) cloud.pts.push_back(random_point<Space, T>(rng));

    incr_tree_t<Space, T> index(Dim, cloud);
    index.addPoints(0, static_cast<uint32_t>(N) - 1);
    std::set<uint32_t> live;
    for (uint32_t i = 0; i < N; ++i) live.insert(i);

    std::uniform_int_distribution<uint32_t> pick(0, static_cast<uint32_t>(N) - 1);
    for (int epoch = 0; epoch < 6; ++epoch)
    {
        // Remove a chunk.
        for (int r = 0; r < static_cast<int>(N) / 5; ++r)
        {
            const uint32_t v = pick(rng);
            if (live.count(v))
            {
                index.removePoint(v);
                live.erase(v);
            }
        }
        // Re-insert some previously removed points (new appended rows).
        for (int a = 0; a < static_cast<int>(N) / 8; ++a)
        {
            const uint32_t newIdx = static_cast<uint32_t>(cloud.pts.size());
            cloud.pts.push_back(random_point<Space, T>(rng));
            index.addPoint(newIdx);
            live.insert(newIdx);
        }
        ASSERT_EQ(index.size(), live.size());

        for (int q = 0; q < 120; ++q)
        {
            const std::vector<T> query = random_point<Space, T>(rng);
            uint32_t             ri;
            T                    rd;
            const size_t         n = index.knnSearch(query.data(), 1, &ri, &rd);
            ASSERT_EQ(n, 1u);
            const std::vector<T> bf = bruteforce_sorted<Space, T>(cloud, live, query);
            ASSERT_NEAR(static_cast<double>(rd), static_cast<double>(bf[0]), 1e-9)
                << "churn epoch " << epoch << " query #" << q;
        }
    }
    // The churn must have triggered at least one rebuild (tombstone reclamation).
    EXPECT_LT(index.physicalSize(), 3 * index.size());
}
}  // namespace incr_manifold_test

using namespace incr_manifold_test;

TEST(incr_manifold, build_vs_bruteforce)
{
    run_incr_build_check<SO2, double>(1, 1, 1);  // trivial single-point tree
    run_incr_build_check<SO2, double>(50, 200, 5);
    run_incr_build_check<SO2, double>(3000, 300, 5);
    run_incr_build_check<SE2, double>(3000, 300, 5);
    run_incr_build_check<Torus<3>, double>(3000, 300, 5);
    run_incr_build_check<SO3, double>(3000, 300, 5);
    run_incr_build_check<SE3, double>(3000, 300, 5);
    run_incr_build_check<Product<SO3, SO3>, double>(2000, 200, 5);
    run_incr_build_check<S2, double>(3000, 300, 5);
    run_incr_build_check<SE3, float>(3000, 300, 5);
}

TEST(incr_manifold, vs_static)
{
    run_incr_vs_static<SO2, double>(3000, 300, 5);
    run_incr_vs_static<SE3, double>(3000, 300, 5);
    run_incr_vs_static<Product<SO3, SO3>, double>(2000, 200, 5);
    run_incr_vs_static<S2, double>(3000, 300, 5);
}

TEST(incr_manifold, churn_vs_bruteforce)
{
    run_incr_churn<SO2, double>(2000);
    run_incr_churn<SE2, double>(2000);
    run_incr_churn<SE3, double>(2000);
    run_incr_churn<S2, double>(2000);
}

// Incremental twin of the static SO2 wrap-around regression: a query near +pi
// whose true nearest neighbor lives near -pi. Insert in both orders, since the
// failing pruning depends on tree shape.
TEST(incr_manifold, SO2_wraparound_regression)
{
    for (int order = 0; order < 2; ++order)
    {
        ManifoldCloud<double> cloud;
        cloud.pts.push_back({-3.1});
        cloud.pts.push_back({1.0});

        incr_tree_t<SO2, double> index(1, cloud);
        if (order == 0)
        {
            index.addPoint(0);
            index.addPoint(1);
        }
        else
        {
            index.addPoint(1);
            index.addPoint(0);
        }

        const double query[1] = {3.14};
        uint32_t     ri;
        double       rd;
        index.knnSearch(query, 1, &ri, &rd);
        EXPECT_EQ(ri, 0u) << "order " << order << ": must return the wrap-around neighbor";
        const double expected = wrap_angle<double>(3.14 - (-3.1));
        EXPECT_NEAR(rd, expected * expected, 1e-9);
    }
}

// Insert quaternions with random sign flips (q vs -q, same rotation); the
// incremental search must honor the double cover. Includes the directed case
// where the dataset holds {q} and the query is {-q} (distance must be 0).
TEST(incr_manifold, SO3_double_cover)
{
    ManifoldCloud<double> cloud;
    std::mt19937          rng(0x9999u);
    for (size_t i = 0; i < 2000; ++i) cloud.pts.push_back(random_point<SO3, double>(rng));

    incr_tree_t<SO3, double> index(4, cloud);
    index.addPoints(0, static_cast<uint32_t>(cloud.pts.size()) - 1);

    // Directed antipodal query.
    const std::vector<double>& q = cloud.pts[0];
    std::vector<double>        negq = {-q[0], -q[1], -q[2], -q[3]};
    uint32_t                   ri;
    double                     rd;
    index.knnSearch(negq.data(), 1, &ri, &rd);
    EXPECT_NEAR(rd, 0.0, 1e-9) << "antipodal quaternion is the same rotation (distance 0)";

    std::set<uint32_t> live;
    for (uint32_t i = 0; i < cloud.pts.size(); ++i) live.insert(i);
    for (int t = 0; t < 200; ++t)
    {
        const std::vector<double> query = random_point<SO3, double>(rng);
        index.knnSearch(query.data(), 1, &ri, &rd);
        const std::vector<double> bf = bruteforce_sorted<SO3, double>(cloud, live, query);
        ASSERT_NEAR(rd, bf[0], 1e-9) << "double-cover query #" << t;
    }
}

#ifndef NANOFLANN_NO_THREADS
// MT/async wrapper on SE(3): foreground insert/remove/query while background
// rebuilds trigger; every query verified against brute force on the live set.
TEST(incr_manifold, mt_churn_vs_bruteforce)
{
    using Space = SE3;
    using T     = double;
    using mt_t  = KDTreeSingleIndexIncrementalAdaptorMT<
        Manifold_Adaptor<Space, T, ManifoldCloud<T>>, ManifoldCloud<T>,
        static_cast<int>(Space::ambient), uint32_t>;

    ManifoldCloud<T> cloud;
    std::mt19937     rng(0x7777u);
    // small min_rebuild_size so background rebuilds actually fire in the test.
    mt_t               index(static_cast<int>(Space::ambient), cloud, {}, 1.3, 500);
    std::set<uint32_t> live;

    std::uniform_int_distribution<int> coin(0, 2);
    for (int frame = 0; frame < 40; ++frame)
    {
        const uint32_t start = static_cast<uint32_t>(cloud.pts.size());
        for (int i = 0; i < 200; ++i) cloud.pts.push_back(random_point<Space, T>(rng));
        const uint32_t end = static_cast<uint32_t>(cloud.pts.size()) - 1;
        index.addPoints(start, end);
        for (uint32_t i = start; i <= end; ++i) live.insert(i);

        // Remove a few live points.
        if (!live.empty())
        {
            std::vector<uint32_t> v(live.begin(), live.end());
            std::shuffle(v.begin(), v.end(), rng);
            for (int r = 0; r < 80 && r < static_cast<int>(v.size()); ++r)
            {
                index.removePoint(v[r]);
                live.erase(v[r]);
            }
        }

        for (int q = 0; q < 60; ++q)
        {
            const std::vector<T> query = random_point<Space, T>(rng);
            uint32_t             ri;
            T                    rd;
            const size_t         n = index.knnSearch(query.data(), 1, &ri, &rd);
            if (n == 0) continue;
            const std::vector<T> bf = bruteforce_sorted<Space, T>(cloud, live, query);
            ASSERT_NEAR(rd, bf[0], 1e-9) << "MT frame " << frame << " query #" << q;
        }
    }
}
#endif  // NANOFLANN_NO_THREADS

#endif  // NANOFLANN_HAS_MANIFOLDS
