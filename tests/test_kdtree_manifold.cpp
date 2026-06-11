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
#include <limits>
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

// Per-base-space random point generation. We recurse over the *base spaces* of
// a product rather than the per-coordinate topology tags, because a sphere block
// (S^N) is tagged `Linear` yet must be generated as a unit vector, which the tag
// walk cannot know. The default below handles any "leaf" space whose coordinates
// are fully described by their tags (Rn, SO2, SO3); Sn and Product are
// specialized.
template <class Space, typename T>
struct PointGen
{
    static void append(std::mt19937& rng, std::vector<T>& out)
    {
        std::uniform_real_distribution<T> ud(-nanoflann::pi_const<T>(), nanoflann::pi_const<T>());
        std::normal_distribution<T>       nd(T(0), T(1));
        for (unsigned i = 0; i < Space::ambient;)
        {
            const nanoflann::CoordTopology topo = Space::topology(i);
            if (topo == nanoflann::CoordTopology::Linear)
            {
                out.push_back(ud(rng) * T(3));
                ++i;
            }
            else if (topo == nanoflann::CoordTopology::Circular)
            {
                out.push_back(ud(rng));
                ++i;
            }
            else  // unit quaternion (4 coords, exercise the double cover by sign flip)
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
                const T sign = (nd(rng) < T(0)) ? T(-1) : T(1);
                for (unsigned k = 0; k < 4; ++k) out.push_back(sign * q[k] / n);
                i += 4;
            }
        }
    }
};

// Unit sphere S^N: N+1 Gaussian coords normalized to unit length. No sign flip
// (antipodal vectors are distinct directions; there is no double cover).
template <unsigned N, typename T>
struct PointGen<nanoflann::Sn<N>, T>
{
    static void append(std::mt19937& rng, std::vector<T>& out)
    {
        std::normal_distribution<T> nd(T(0), T(1));
        T                           v[N + 1];
        T                           n = T(0);
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
struct PointGen<nanoflann::Product<>, T>
{
    static void append(std::mt19937& /*rng*/, std::vector<T>& /*out*/) {}
};

// Product: concatenate the generators of the base spaces, so a sphere block is
// generated correctly wherever it sits in the layout.
template <class B, class... Rest, typename T>
struct PointGen<nanoflann::Product<B, Rest...>, T>
{
    static void append(std::mt19937& rng, std::vector<T>& out)
    {
        PointGen<B, T>::append(rng, out);
        PointGen<nanoflann::Product<Rest...>, T>::append(rng, out);
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

// ---------------------------------------------------------------------------
//  Unit sphere S^2 / S^N (chordal embedding, no double cover)
// ---------------------------------------------------------------------------
TEST(manifold, S2_vs_bruteforce)
{
    manifold_test::run_bruteforce_check<nanoflann::S2, double>(3000, 500, 5);
    manifold_test::run_bruteforce_check<nanoflann::S2, float>(3000, 500, 5);
    // Pure-leaf-scan small case and a larger one.
    manifold_test::run_bruteforce_check<nanoflann::S2, double>(8, 200, 3);
    manifold_test::run_bruteforce_check<nanoflann::S2, double>(10000, 300, 5);
    // Higher sphere.
    manifold_test::run_bruteforce_check<nanoflann::Sn<3>, double>(3000, 500, 5);
}

TEST(manifold, R3xS2_vs_bruteforce)
{
    using Space = nanoflann::Product<nanoflann::Rn<3>, nanoflann::S2>;
    manifold_test::run_bruteforce_check<Space, double>(3000, 500, 5);
    manifold_test::run_bruteforce_check<Space, float>(3000, 500, 5);
}

TEST(manifold, S2xS2_vs_bruteforce)
{
    using Space = nanoflann::Product<nanoflann::S2, nanoflann::S2>;
    manifold_test::run_bruteforce_check<Space, double>(3000, 500, 5);
}

namespace manifold_test
{
// Geodesic angle between two unit vectors (clamped acos of the dot product),
// independent of nanoflann internals.
template <typename T>
T geodesic_angle(const std::vector<T>& a, const std::vector<T>& b)
{
    T dot = T(0);
    for (size_t i = 0; i < a.size(); ++i) dot += a[i] * b[i];
    if (dot > T(1)) dot = T(1);
    if (dot < T(-1)) dot = T(-1);
    return std::acos(dot);
}
}  // namespace manifold_test

// The chordal metric must induce the same NN ranking as the geodesic angle.
TEST(manifold, S2_geodesic_ranking)
{
    using namespace nanoflann;
    using T            = double;
    using Space        = S2;
    constexpr size_t N = 2000;
    constexpr size_t Q = 300;
    constexpr size_t K = 5;

    manifold_test::ManifoldCloud<T> cloud;
    std::mt19937                    rng(0x5EEDu);
    for (size_t i = 0; i < N; ++i)
        cloud.pts.push_back(manifold_test::random_point<Space, T>(rng));

    using metric_t = Manifold_Adaptor<Space, T, manifold_test::ManifoldCloud<T>>;
    using index_t  = KDTreeSingleIndexAdaptor<metric_t, manifold_test::ManifoldCloud<T>, 3>;
    index_t index(3, cloud, {10});

    for (size_t q = 0; q < Q; ++q)
    {
        const std::vector<T> query = manifold_test::random_point<Space, T>(rng);

        std::vector<size_t> idx(K);
        std::vector<T>      d2(K);
        KNNResultSet<T>     rs(K);
        rs.init(idx.data(), d2.data());
        index.findNeighbors(rs, query.data());

        // Brute-force geodesic-sorted angles.
        std::vector<T> ang;
        ang.reserve(N);
        for (size_t i = 0; i < N; ++i)
            ang.push_back(manifold_test::geodesic_angle<T>(query, cloud.pts[i]));
        std::sort(ang.begin(), ang.end());

        for (size_t k = 0; k < K; ++k)
        {
            // The k-th tree neighbor's chordal distance, converted to an angle,
            // must equal the k-th smallest geodesic angle.
            const T theta = chord_sq_to_angle<T>(d2[k]);
            EXPECT_NEAR(theta, ang[k], 1e-7) << "query #" << q << " neighbor #" << k;
        }
    }
}

// Radius search with an angular radius converted via angle_to_chord_sq must
// return exactly the brute-force angular-filter set. Includes a near-antipodal
// radius (179 deg) where a wrong double-cover treatment would show.
TEST(manifold, S2_radius_angular)
{
    using namespace nanoflann;
    using T            = double;
    using Space        = S2;
    constexpr size_t N = 2000;

    manifold_test::ManifoldCloud<T> cloud;
    std::mt19937                    rng(0xA11CEu);
    for (size_t i = 0; i < N; ++i)
        cloud.pts.push_back(manifold_test::random_point<Space, T>(rng));

    using metric_t = Manifold_Adaptor<Space, T, manifold_test::ManifoldCloud<T>>;
    using index_t  = KDTreeSingleIndexAdaptor<metric_t, manifold_test::ManifoldCloud<T>, 3>;
    index_t index(3, cloud, {10});

    const T deg = pi_const<T>() / T(180);
    for (const T theta : {T(1) * deg, T(10) * deg, T(90) * deg, T(179) * deg})
    {
        const std::vector<T> query = manifold_test::random_point<Space, T>(rng);
        const T              r2     = angle_to_chord_sq<T>(theta);

        std::vector<nanoflann::ResultItem<uint32_t, T>> matches;
        nanoflann::SearchParameters                   sp;
        sp.sorted = false;
        index.radiusSearch(query.data(), r2, matches, sp);

        // Brute-force angular filter.
        size_t bf = 0;
        for (size_t i = 0; i < N; ++i)
            if (manifold_test::geodesic_angle<T>(query, cloud.pts[i]) <= theta + T(1e-12)) ++bf;

        EXPECT_EQ(matches.size(), bf) << "theta(deg)=" << theta / deg;
    }
}

// All points clustered near the north pole +Z: a latitude/longitude chart
// implementation fails here; the embedding route must stay exact.
TEST(manifold, S2_polar_cluster)
{
    using namespace nanoflann;
    using T = double;
    manifold_test::ManifoldCloud<T> cloud;
    std::mt19937                    rng(0xBEEFu);
    std::normal_distribution<T>     jitter(T(0), T(0.02));
    for (int sign : {+1, -1})
    {
        for (size_t i = 0; i < 500; ++i)
        {
            T v[3] = {jitter(rng), jitter(rng), T(sign) * (T(1) + jitter(rng))};
            T n    = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            cloud.pts.push_back({v[0] / n, v[1] / n, v[2] / n});
        }
    }
    using metric_t = Manifold_Adaptor<S2, T, manifold_test::ManifoldCloud<T>>;
    using index_t  = KDTreeSingleIndexAdaptor<metric_t, manifold_test::ManifoldCloud<T>, 3>;
    index_t index(3, cloud, {10});

    std::mt19937 qrng(0x1234u);
    for (size_t q = 0; q < 200; ++q)
    {
        const std::vector<T> query = manifold_test::random_point<S2, T>(qrng);
        size_t               idx;
        T                    d2;
        KNNResultSet<T>      rs(1);
        rs.init(&idx, &d2);
        index.findNeighbors(rs, query.data());

        T best = std::numeric_limits<T>::max();
        for (const auto& p : cloud.pts)
            best = std::min(best, manifold_test::reference_dist<S2, T>(query, p));
        EXPECT_NEAR(d2, best, 1e-9) << "polar query #" << q;
    }
}

// Dataset of {v, -v} antipodal pairs: a query near +v must return +v, never -v.
// This is the test that distinguishes S2 (no double cover) from a hypothetical
// RP2 (which would treat v and -v as identical).
TEST(manifold, S2_antipodal_pairs)
{
    using namespace nanoflann;
    using T = double;
    manifold_test::ManifoldCloud<T> cloud;
    std::mt19937                    rng(0xFEEDu);
    std::vector<std::vector<T>>     base;
    for (size_t i = 0; i < 500; ++i)
    {
        std::vector<T> v = manifold_test::random_point<S2, T>(rng);
        base.push_back(v);
        cloud.pts.push_back(v);
        cloud.pts.push_back({-v[0], -v[1], -v[2]});
    }
    using metric_t = Manifold_Adaptor<S2, T, manifold_test::ManifoldCloud<T>>;
    using index_t  = KDTreeSingleIndexAdaptor<metric_t, manifold_test::ManifoldCloud<T>, 3>;
    index_t index(3, cloud, {10});

    for (const auto& v : base)
    {
        // Query slightly perturbed toward +v.
        const std::vector<T> query = v;
        size_t               idx;
        T                    d2;
        KNNResultSet<T>      rs(1);
        rs.init(&idx, &d2);
        index.findNeighbors(rs, query.data());
        // Nearest must be +v itself (distance ~0), not its antipode (distance 4).
        EXPECT_NEAR(d2, T(0), 1e-9);
        EXPECT_LT(d2, T(2)) << "must not collapse antipodes (no double cover)";
    }
}

TEST(manifold, chord_angle_roundtrip)
{
    using namespace nanoflann;
    for (int i = 0; i <= 1000; ++i)
    {
        const double theta = (double(i) / 1000.0) * pi_const<double>();
        const double d2    = angle_to_chord_sq<double>(theta);
        const double back  = chord_sq_to_angle<double>(d2);
        EXPECT_NEAR(theta, back, 1e-12) << "theta=" << theta;
    }
    for (int i = 0; i <= 1000; ++i)
    {
        const float theta = (float(i) / 1000.0f) * pi_const<float>();
        const float d2    = angle_to_chord_sq<float>(theta);
        const float back  = chord_sq_to_angle<float>(d2);
        // The inverse is ill-conditioned near theta=pi (the chord saturates, so
        // asin' diverges); use a looser float tolerance there.
        const float tol = (theta > 3.0f) ? 1e-4f : 1e-5f;
        EXPECT_NEAR(theta, back, tol) << "theta=" << theta;
    }
}

#endif  // NANOFLANN_HAS_MANIFOLDS
