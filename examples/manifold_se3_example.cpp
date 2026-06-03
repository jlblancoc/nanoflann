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

// Example: exact nearest-neighbor search over SE(3) poses (R^3 x SO(3)) using
// the compile-time product-manifold topology feature of nanoflann 2.0.
//
// A pose is stored as 7 scalar coordinates: (x, y, z, qx, qy, qz, qw), the last
// four being a unit quaternion. The synthesized metric uses squared Euclidean
// distance on the translation block and the chordal quaternion distance
// (handling the antipodal double cover q ~ -q) on the rotation block. The
// KD-tree pruning stays *exact* (it returns the same neighbor as brute force).

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
// A cloud of SE(3) poses stored as flat 7-coordinate rows.
template <typename num_t>
struct PoseCloud
{
    std::vector<std::array<num_t, 7>> poses;

    inline size_t kdtree_get_point_count() const { return poses.size(); }
    inline num_t kdtree_get_pt(const size_t idx, const size_t dim) const { return poses[idx][dim]; }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const
    {
        return false;
    }
};

template <typename num_t>
std::array<num_t, 7> random_pose()
{
    std::array<num_t, 7> p;
    for (int i = 0; i < 3; ++i)
        p[i] = static_cast<num_t>(-10.0 + 20.0 * (std::rand() / (RAND_MAX + 1.0)));
    num_t q[4];
    num_t n = 0;
    for (int k = 0; k < 4; ++k)
    {
        q[k] = static_cast<num_t>(-1.0 + 2.0 * (std::rand() / (RAND_MAX + 1.0)));
        n += q[k] * q[k];
    }
    n = std::sqrt(n);
    for (int k = 0; k < 4; ++k) p[3 + k] = q[k] / n;
    return p;
}

template <typename num_t>
void kdtree_demo(const size_t N)
{
    PoseCloud<num_t> cloud;
    cloud.poses.reserve(N);
    for (size_t i = 0; i < N; ++i) cloud.poses.push_back(random_pose<num_t>());

    // Declare the state-space topology once, at compile time:
    using StateSpace = nanoflann::SE3;  // = Product<Rn<3>, SO3>, 7 coords

    using metric_t = nanoflann::Manifold_Adaptor<StateSpace, num_t, PoseCloud<num_t>>;
    using kdtree_t = nanoflann::KDTreeSingleIndexAdaptor<
        metric_t, PoseCloud<num_t>, StateSpace::ambient /* = 7 */>;

    kdtree_t index(StateSpace::ambient, cloud, {10 /* max leaf */});

    const std::array<num_t, 7> query = random_pose<num_t>();

    const size_t                   k = 3;
    std::vector<size_t>            ret_index(k);
    std::vector<num_t>             out_dist_sqr(k);
    nanoflann::KNNResultSet<num_t> resultSet(k);
    resultSet.init(ret_index.data(), out_dist_sqr.data());
    index.findNeighbors(resultSet, query.data());

    std::cout << "SE(3) nearest poses to the query:\n";
    for (size_t i = 0; i < k; ++i)
        std::cout << "  #" << i << ": idx=" << ret_index[i] << " dist^2=" << out_dist_sqr[i]
                  << "\n";
}
}  // namespace

int main()
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    kdtree_demo<double>(100000);
    return 0;
}
#endif  // NANOFLANN_HAS_MANIFOLDS
