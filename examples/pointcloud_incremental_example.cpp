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

// Example: plain Euclidean (R^3) sliding-window LiDAR map with the
// self-balancing incremental KD-tree of nanoflann 2.0. This is the canonical
// use case the incremental index is designed for:
//
//   - insert each new scan's points (addPoints), without rebuilding the tree;
//   - query the running map (knnSearch / radiusSearch);
//   - trim points that fell outside a moving box (removeOutsideBox), keeping
//     memory bounded under churn (lazy tombstones + amortized partial rebuilds);
//   - optionally recycle the freed point slots (acquireRemovedPoints).
//
// A multithreaded variant (KDTreeSingleIndexIncrementalAdaptorMT) that performs
// the balancing rebuilds on a background thread is sketched at the end.

#include <array>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <nanoflann.hpp>
#include <vector>

namespace
{
template <typename num_t>
struct PointCloud
{
    // A simple growable point store. Removed slots are recycled in place so the
    // dataset indices handed to the tree stay stable.
    std::vector<std::array<num_t, 3>> pts;
    std::vector<uint32_t>             freeSlots;

    inline size_t kdtree_get_point_count() const { return pts.size(); }
    inline num_t  kdtree_get_pt(const size_t idx, const size_t dim) const { return pts[idx][dim]; }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const
    {
        return false;
    }

    // Returns the index of the newly stored point, reusing a recycled slot if
    // available.
    uint32_t store(const std::array<num_t, 3>& p)
    {
        if (!freeSlots.empty())
        {
            const uint32_t slot = freeSlots.back();
            freeSlots.pop_back();
            pts[slot] = p;
            return slot;
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
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    PointCloud<num_t> cloud;

    using metric_t = nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t>>;
    using kdtree_t =
        nanoflann::KDTreeSingleIndexIncrementalAdaptor<metric_t, PointCloud<num_t>, 3, uint32_t>;

    kdtree_t index(3, cloud);

    const size_t nFrames        = 200;
    const size_t ptsPerFrame    = 500;
    const num_t  queryRadius2   = num_t(4.0);  // squared meters
    size_t       totalNeighbors = 0;

    for (size_t f = 0; f < nFrames; ++f)
    {
        // The "sensor" sweeps along +x, one meter per frame.
        const num_t center = static_cast<num_t>(f);

        // 1) Insert this frame's points.
        for (size_t k = 0; k < ptsPerFrame; ++k)
        {
            const uint32_t slot = cloud.store(random_point<num_t>(center));
            index.addPoint(slot);
        }

        // 2) Query the running map near the sensor position.
        const std::array<num_t, 3>                          q = {center, num_t(0), num_t(0)};
        std::vector<nanoflann::ResultItem<uint32_t, num_t>> matches;
        totalNeighbors += index.radiusSearch(q.data(), queryRadius2, matches);

        // 3) Sliding-window trim: keep only points within +/-15 m of the sensor
        //    in x (boxes are raw axis-aligned coordinates; here y, z are kept
        //    unbounded with a generous range).
        typename kdtree_t::BoundingBox keep;
        nanoflann::resize(keep, 3);
        keep[0].low  = center - 15;
        keep[0].high = center + 15;
        for (int d = 1; d < 3; ++d)
        {
            keep[d].low  = static_cast<num_t>(-1e6);
            keep[d].high = static_cast<num_t>(+1e6);
        }
        index.setCollectRemovedPoints(true);
        index.removeOutsideBox(keep);

        // 4) Recycle the slots of physically-evicted points back into the cloud.
        for (const uint32_t evicted : index.acquireRemovedPoints())
        {
            cloud.freeSlots.push_back(evicted);
        }
    }

    std::cout << "Streamed " << nFrames << " frames; live points=" << index.size()
              << " physical nodes=" << index.physicalSize()
              << " recyclable slots=" << cloud.freeSlots.size()
              << " radius matches=" << totalNeighbors << "\n";

    // --- Multithreaded variant (background-thread rebuilds) -----------------
    // The wrapper has the same insert/remove/query API; rebuilds run on a worker
    // thread so foreground update latency stays low. Use sync() before any read
    // that must observe all pending mutations, and setRebuildCallback() to be
    // notified when a background rebuild swaps in a fresh tree, e.g.:
    //
    //   using kdtree_mt_t = nanoflann::KDTreeSingleIndexIncrementalAdaptorMT<
    //       metric_t, PointCloud<num_t>, 3, uint32_t>;
    //   kdtree_mt_t mt(3, cloud);
    //   mt.setRebuildCallback([](kdtree_t& fresh) { /* warm caches, log... */ });
    //   mt.addPoints(0, N);
    //   mt.sync();  // ensure the worker finished before querying
    //   mt.knnSearch(q.data(), 1, &idx, &dist);

    return 0;
}
