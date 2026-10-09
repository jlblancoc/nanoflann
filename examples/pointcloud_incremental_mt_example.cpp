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

// Example: the sliding-window map of pointcloud_incremental_example.cpp, now
// with KDTreeSingleIndexIncrementalAdaptorMT, which runs the large balancing
// rebuilds on a background thread to keep the per-frame update latency low.
//
// The background thread reads point coordinates through the dataset adaptor
// while the foreground thread keeps inserting, so the dataset storage must not
// move while a rebuild is in flight. Here the point vector is reserved up
// front, and evicted slots are recycled so it never outgrows that capacity.

#include <array>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <nanoflann.hpp>
#include <stdexcept>
#include <vector>

#ifndef NANOFLANN_NO_THREADS
namespace
{
template <typename num_t>
struct PointCloud
{
    std::vector<std::array<num_t, 3>> pts;
    std::vector<uint32_t>             freeSlots;

    explicit PointCloud(const size_t capacity) { pts.reserve(capacity); }

    inline size_t kdtree_get_point_count() const { return pts.size(); }
    inline num_t  kdtree_get_pt(const size_t idx, const size_t dim) const { return pts[idx][dim]; }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const
    {
        return false;
    }

    // Stores a point, reusing a recycled slot if available. Never grows past
    // the reserved capacity, so existing points are never moved.
    uint32_t store(const std::array<num_t, 3>& p)
    {
        if (!freeSlots.empty())
        {
            const uint32_t slot = freeSlots.back();
            freeSlots.pop_back();
            pts[slot] = p;
            return slot;
        }
        if (pts.size() == pts.capacity())
        {
            throw std::runtime_error("PointCloud: reserved capacity exceeded");
        }
        pts.push_back(p);
        return static_cast<uint32_t>(pts.size() - 1);
    }
};

template <typename num_t>
std::array<num_t, 3> random_point(const num_t center)
{
    std::array<num_t, 3> p;
    for (int i = 0; i < 3; ++i)
    {
        p[i] = center + static_cast<num_t>(-5.0 + 10.0 * (std::rand() / (RAND_MAX + 1.0)));
    }
    return p;
}
}  // namespace

int main()
{
    using num_t = double;
    std::srand(42);

    PointCloud<num_t> cloud(200000);

    using metric_t = nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t>>;
    using kdtree_t =
        nanoflann::KDTreeSingleIndexIncrementalAdaptorMT<metric_t, PointCloud<num_t>, 3, uint32_t>;

    // Start a background rebuild once the tree holds 1.3x the live points of
    // the last rebuild (and at least 5000 nodes).
    kdtree_t index(3, cloud, nanoflann::KDTreeIncrementalIndexParams(), 1.3, 5000);
    index.setCollectRemovedPoints(true);

    // Runs on the worker thread for each freshly built tree, before it replaces
    // the active one. It must only touch the tree it is given.
    std::atomic<int> rebuilds{0};
    index.setRebuildCallback([&rebuilds](kdtree_t::Inner&) { rebuilds++; });

    const size_t nFrames        = 200;
    const size_t ptsPerFrame    = 500;
    const num_t  queryRadius2   = num_t(4.0);  // squared meters
    size_t       totalNeighbors = 0;

    for (size_t f = 0; f < nFrames; ++f)
    {
        const num_t center = static_cast<num_t>(f);

        // 1) Insert this frame's points.
        for (size_t k = 0; k < ptsPerFrame; ++k)
        {
            index.addPoint(cloud.store(random_point<num_t>(center)));
        }

        // 2) Query the running map. Queries always see every mutation issued so
        //    far, even while a rebuild is running in the background.
        const std::array<num_t, 3>                          q = {center, num_t(0), num_t(0)};
        std::vector<nanoflann::ResultItem<uint32_t, num_t>> matches;
        totalNeighbors += index.radiusSearch(q.data(), queryRadius2, matches);

        // 3) Sliding-window trim along x.
        kdtree_t::BoundingBox keep;
        nanoflann::resize(keep, 3);
        keep[0].low  = center - 15;
        keep[0].high = center + 15;
        for (int d = 1; d < 3; ++d)
        {
            keep[d].low  = static_cast<num_t>(-1e6);
            keep[d].high = static_cast<num_t>(+1e6);
        }
        index.removeOutsideBox(keep);

        // 4) Recycle the slots no tree references anymore.
        for (const uint32_t evicted : index.acquireRemovedPoints())
        {
            cloud.freeSlots.push_back(evicted);
        }
    }

    // Wait for any in-flight rebuild before reporting (or saving the index).
    index.sync();

    std::cout << "Streamed " << nFrames << " frames; live points=" << index.size()
              << " physical nodes=" << index.physicalSize() << " dataset slots=" << cloud.pts.size()
              << " background rebuilds=" << rebuilds << " radius matches=" << totalNeighbors
              << "\n";
    return 0;
}
#else
int main()
{
    std::cout << "This example requires threads (NANOFLANN_NO_THREADS is defined).\n";
    return 0;
}
#endif  // NANOFLANN_NO_THREADS
