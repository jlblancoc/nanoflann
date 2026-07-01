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

// Example: exact incremental nearest-neighbor search over SE(3) poses using the
// self-balancing incremental KD-tree of nanoflann 2.0 with a product-manifold
// metric. A growing keyframe database is the canonical use case:
//
//   - SE(3) loop-closure candidate retrieval: insert each new keyframe pose,
//     radius-search the database in pose space for nearby candidates. The tree
//     only ever grows (plus optional sliding-window trimming), so there is no
//     per-frame rebuild as with a static manifold tree.
//   - RRT / RRT* on SE(2)/SE(3): insert each sampled state, then knnSearch /
//     radiusSearch for the near set. Same growing-tree pattern.
//
// A pose is 7 coords (x, y, z, qx, qy, qz, qw). The metric mixes squared meters
// (translation) with the chordal quaternion distance (rotation). Scale the
// rotation block by a weight w_R if the units must be balanced (here we keep the
// search radius generous instead).

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
template <typename num_t>
struct KeyframeDB
{
    std::vector<std::array<num_t, 7>> poses;
    inline size_t                     kdtree_get_point_count() const { return poses.size(); }
    inline num_t kdtree_get_pt(const size_t idx, const size_t dim) const { return poses[idx][dim]; }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const
    {
        return false;
    }
};

template <typename num_t>
std::array<num_t, 7> random_pose()
{
    std::array<num_t, 7> p;
    for (int i = 0; i < 3; ++i)
        p[i] = static_cast<num_t>(-50.0 + 100.0 * (std::rand() / (RAND_MAX + 1.0)));
    num_t q[4], n = 0;
    for (int k = 0; k < 4; ++k)
    {
        q[k] = static_cast<num_t>(-1.0 + 2.0 * (std::rand() / (RAND_MAX + 1.0)));
        n += q[k] * q[k];
    }
    n = std::sqrt(n);
    for (int k = 0; k < 4; ++k) p[3 + k] = q[k] / n;
    return p;
}
}  // namespace

int main()
{
    using num_t = double;
    using Space = nanoflann::SE3;  // R^3 x SO(3), 7 coords
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    KeyframeDB<num_t> db;

    using metric_t = nanoflann::Manifold_Adaptor<Space, num_t, KeyframeDB<num_t>>;
    using kdtree_t = nanoflann::KDTreeSingleIndexIncrementalAdaptor<
        metric_t, KeyframeDB<num_t>, Space::ambient, uint32_t>;

    kdtree_t index(Space::ambient, db);

    // Stream keyframes in; for each, query the database built so far.
    const size_t nFrames         = 5000;
    const num_t  radius2         = num_t(25.0);  // chordal-squared search radius
    size_t       totalCandidates = 0;
    for (uint32_t f = 0; f < nFrames; ++f)
    {
        db.poses.push_back(random_pose<num_t>());

        if (f > 0)
        {
            const auto&                                         q = db.poses.back();
            std::vector<nanoflann::ResultItem<uint32_t, num_t>> matches;
            const size_t n = index.radiusSearch(q.data(), radius2, matches);
            totalCandidates += n;
        }
        index.addPoint(f);

        // Sliding-window map trim: keep poses within a translation cube, but the
        // whole rotation block (set quaternion dims to [-1, 1]). Boxes are raw
        // coordinates and do not wrap.
        if (f % 1000 == 999)
        {
            typename kdtree_t::BoundingBox keep;
            nanoflann::resize(keep, 7);
            for (int d = 0; d < 3; ++d)
            {
                keep[d].low  = -40;
                keep[d].high = 40;
            }
            for (int d = 3; d < 7; ++d)
            {
                keep[d].low  = -1;
                keep[d].high = 1;
            }
            index.removeOutsideBox(keep);
        }
    }

    std::cout << "Streamed " << nFrames << " keyframes; live=" << index.size()
              << " physical=" << index.physicalSize()
              << " loop-closure candidates found=" << totalCandidates << "\n";
    return 0;
}
#endif  // NANOFLANN_HAS_MANIFOLDS
