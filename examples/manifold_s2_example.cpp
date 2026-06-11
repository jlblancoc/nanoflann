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

// Example: exact nearest-neighbor search over 3D directions on the unit sphere
// S^2 using the compile-time product-manifold topology feature of nanoflann 2.0.
//
// A direction is stored as 3 unit-vector coordinates (x, y, z), ||p|| = 1. The
// synthesized metric uses the chordal squared distance ||p-q||^2 = 2 - 2<p,q>,
// a strictly increasing function of the geodesic angle, so the KD-tree returns
// the exact nearest direction (same neighbor as brute force). Reported squared
// distances convert to angles via nanoflann::chord_sq_to_angle(); an angular
// search radius converts via nanoflann::angle_to_chord_sq().
//
// Applications: surface-normal / bearing-vector retrieval (surfel maps, ICP
// normal-space sampling), gravity/sun-direction matching, and, beyond robotics,
// angular-separation searches over event arrival directions in astroparticle
// physics. A second index over Product<R^3, S^2> shows oriented points (surfels).

#include <array>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <nanoflann.hpp>
#include <vector>

#if !defined(NANOFLANN_HAS_MANIFOLDS)
int main()
{
    std::cerr << "This example requires C++17 (product-manifold topology).\n";
    return 0;
}
#else

namespace
{
// A cloud of unit-vector directions stored as flat 3-coordinate rows.
template <typename num_t>
struct DirectionCloud
{
    std::vector<std::array<num_t, 3>> dirs;

    inline size_t kdtree_get_point_count() const { return dirs.size(); }
    inline num_t  kdtree_get_pt(const size_t idx, const size_t dim) const { return dirs[idx][dim]; }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const
    {
        return false;
    }
};

template <typename num_t>
std::array<num_t, 3> random_direction()
{
    std::array<num_t, 3> v;
    num_t                n = 0;
    for (int k = 0; k < 3; ++k)
    {
        // Box-Muller-free crude Gaussian via central limit; good enough for a demo.
        num_t g = 0;
        for (int j = 0; j < 6; ++j) g += static_cast<num_t>(std::rand() / (RAND_MAX + 1.0));
        v[k] = g - num_t(3);
        n += v[k] * v[k];
    }
    n = std::sqrt(n);
    for (int k = 0; k < 3; ++k) v[k] /= n;
    return v;
}

template <typename num_t>
void s2_demo(const size_t N)
{
    DirectionCloud<num_t> cloud;
    cloud.dirs.reserve(N);
    for (size_t i = 0; i < N; ++i) cloud.dirs.push_back(random_direction<num_t>());

    // Declare the state-space topology once, at compile time:
    using StateSpace = nanoflann::S2;  // = Sn<2>, 3 unit-vector coords

    using metric_t = nanoflann::Manifold_Adaptor<StateSpace, num_t, DirectionCloud<num_t>>;
    using kdtree_t = nanoflann::
        KDTreeSingleIndexAdaptor<metric_t, DirectionCloud<num_t>, StateSpace::ambient /* = 3 */>;

    kdtree_t index(StateSpace::ambient, cloud, {10 /* max leaf */});

    const std::array<num_t, 3> query = random_direction<num_t>();

    // (1) k-NN: report neighbors as geodesic angles in degrees.
    const size_t                   k = 3;
    std::vector<size_t>            ret_index(k);
    std::vector<num_t>             out_dist_sqr(k);
    nanoflann::KNNResultSet<num_t> resultSet(k);
    resultSet.init(ret_index.data(), out_dist_sqr.data());
    index.findNeighbors(resultSet, query.data());

    const num_t rad2deg = num_t(180) / nanoflann::pi_const<num_t>();
    std::cout << "S^2 nearest directions to the query:\n";
    for (size_t i = 0; i < k; ++i)
    {
        const num_t ang = nanoflann::chord_sq_to_angle<num_t>(out_dist_sqr[i]) * rad2deg;
        std::cout << "  #" << i << ": idx=" << ret_index[i] << " angle=" << ang << " deg\n";
    }

    // (2) radius search within a 5-degree angular cap.
    const num_t theta = num_t(5) / rad2deg;
    const num_t r2    = nanoflann::angle_to_chord_sq<num_t>(theta);
    std::vector<nanoflann::ResultItem<uint32_t, num_t>> matches;
    const size_t nFound = index.radiusSearch(query.data(), r2, matches);
    std::cout << "Directions within 5 deg of the query: " << nFound << "\n";
}

// Oriented points (surfels): position in R^3 plus a unit normal on S^2, i.e.
// Product<R^3, S^2> (6 coordinates).
template <typename num_t>
struct SurfelCloud
{
    std::vector<std::array<num_t, 6>> rows;
    inline size_t kdtree_get_point_count() const { return rows.size(); }
    inline num_t  kdtree_get_pt(const size_t i, const size_t d) const { return rows[i][d]; }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const
    {
        return false;
    }
};

template <typename num_t>
void surfel_demo(const size_t N)
{
    using Space = nanoflann::Product<nanoflann::Rn<3>, nanoflann::S2>;  // 6 coords

    SurfelCloud<num_t> cloud;
    cloud.rows.reserve(N);
    for (size_t i = 0; i < N; ++i)
    {
        std::array<num_t, 6>       r;
        const std::array<num_t, 3> nrm = random_direction<num_t>();
        for (int j = 0; j < 3; ++j)
            r[j] = static_cast<num_t>(-10.0 + 20.0 * (std::rand() / (RAND_MAX + 1.0)));
        for (int j = 0; j < 3; ++j) r[3 + j] = nrm[j];
        cloud.rows.push_back(r);
    }

    using metric_t = nanoflann::Manifold_Adaptor<Space, num_t, SurfelCloud<num_t>>;
    using kdtree_t = nanoflann::KDTreeSingleIndexAdaptor<metric_t, SurfelCloud<num_t>, Space::ambient>;
    kdtree_t index(Space::ambient, cloud, {10});

    std::array<num_t, 6>       q;
    const std::array<num_t, 3> qn = random_direction<num_t>();
    for (int j = 0; j < 3; ++j) q[j] = 0;
    for (int j = 0; j < 3; ++j) q[3 + j] = qn[j];

    size_t                         idx;
    num_t                          d2;
    nanoflann::KNNResultSet<num_t> rs(1);
    rs.init(&idx, &d2);
    index.findNeighbors(rs, q.data());
    std::cout << "Nearest surfel (R^3 x S^2): idx=" << idx << " dist^2=" << d2 << "\n";
}
}  // namespace

int main()
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    s2_demo<double>(100000);
    surfel_demo<double>(100000);
    return 0;
}
#endif  // NANOFLANN_HAS_MANIFOLDS
