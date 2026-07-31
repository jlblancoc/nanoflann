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
//  Tests for KDTreeSingleIndexIncrementalAdaptor (single self-balancing tree)
// ===========================================================================

#include <sstream>

#include "test_helpers.h"

TEST(kdtree_incremental, add_knn_vs_bruteforce)
{
    srand(42);
    inc_cloud_t        cloud;
    inc_tree_t         index(3, cloud);
    std::set<uint32_t> live;

    // Insert in several batches (interleaved, like a streaming source).
    for (int batch = 0; batch < 6; ++batch)
    {
        const uint32_t start = static_cast<uint32_t>(cloud.pts.size());
        const size_t   add   = 300 + batch * 50;
        for (size_t i = 0; i < add; ++i)
            cloud.pts.push_back(
                {10.0 * (rand() % 1000) / 1000.0, 10.0 * (rand() % 1000) / 1000.0,
                 10.0 * (rand() % 1000) / 1000.0});
        const uint32_t end = static_cast<uint32_t>(cloud.pts.size()) - 1;
        index.addPoints(start, end);
        for (uint32_t i = start; i <= end; ++i) live.insert(i);
    }
    EXPECT_EQ(index.size(), live.size());

    for (int t = 0; t < 300; ++t)
    {
        const double q[3] = {
            10.0 * (rand() % 1000) / 1000.0, 10.0 * (rand() % 1000) / 1000.0,
            10.0 * (rand() % 1000) / 1000.0};
        uint32_t     ri;
        double       rd;
        const size_t n = index.knnSearch(q, 1, &ri, &rd);
        uint32_t     bi;
        const double bd = bruteforce_nn(cloud, live, q, bi);
        ASSERT_EQ(n, 1u);
        EXPECT_NEAR(rd, bd, 1e-9);
    }
}

TEST(kdtree_incremental, remove_knn_vs_bruteforce)
{
    srand(7);
    inc_cloud_t  cloud;
    inc_tree_t   index(3, cloud);
    const size_t N = 2000;
    for (size_t i = 0; i < N; ++i)
        cloud.pts.push_back(
            {10.0 * (rand() % 1000) / 1000.0, 10.0 * (rand() % 1000) / 1000.0,
             10.0 * (rand() % 1000) / 1000.0});
    index.addPoints(0, static_cast<uint32_t>(N) - 1);

    std::set<uint32_t> live;
    for (uint32_t i = 0; i < N; ++i) live.insert(i);

    // Remove ~40% of points at random.
    std::vector<uint32_t> order(live.begin(), live.end());
    std::mt19937          g(99);
    std::shuffle(order.begin(), order.end(), g);
    for (size_t i = 0; i < order.size() * 4 / 10; ++i)
    {
        index.removePoint(order[i]);
        live.erase(order[i]);
    }
    EXPECT_EQ(index.size(), live.size());

    // Removing twice / removing an out-of-range index must be a no-op.
    index.removePoint(order[0]);
    index.removePoint(1000000);
    EXPECT_EQ(index.size(), live.size());

    for (int t = 0; t < 300; ++t)
    {
        const double q[3] = {
            10.0 * (rand() % 1000) / 1000.0, 10.0 * (rand() % 1000) / 1000.0,
            10.0 * (rand() % 1000) / 1000.0};
        uint32_t     ri;
        double       rd;
        const size_t n = index.knnSearch(q, 1, &ri, &rd);
        uint32_t     bi;
        const double bd = bruteforce_nn(cloud, live, q, bi);
        ASSERT_EQ(n, 1u);
        EXPECT_TRUE(live.count(ri) == 1);
        EXPECT_NEAR(rd, bd, 1e-9);
    }
}

TEST(kdtree_incremental, removeOutsideBox_vs_bruteforce)
{
    srand(123);
    inc_cloud_t  cloud;
    inc_tree_t   index(3, cloud);
    const size_t N = 3000;
    for (size_t i = 0; i < N; ++i)
        cloud.pts.push_back(
            {20.0 * (rand() % 1000) / 1000.0 - 10.0, 20.0 * (rand() % 1000) / 1000.0 - 10.0,
             20.0 * (rand() % 1000) / 1000.0 - 10.0});
    index.addPoints(0, static_cast<uint32_t>(N) - 1);

    const double            lo[3] = {-3, -3, -3}, hi[3] = {3, 3, 3};
    inc_tree_t::BoundingBox keep;
    for (int d = 0; d < 3; ++d)
    {
        keep[d].low  = lo[d];
        keep[d].high = hi[d];
    }
    index.removeOutsideBox(keep);

    std::set<uint32_t> live;
    for (uint32_t i = 0; i < N; ++i)
        if (inc_in_box(cloud, i, lo, hi)) live.insert(i);
    EXPECT_EQ(index.size(), live.size());

    for (int t = 0; t < 300; ++t)
    {
        const double q[3] = {
            6.0 * (rand() % 1000) / 1000.0 - 3.0, 6.0 * (rand() % 1000) / 1000.0 - 3.0,
            6.0 * (rand() % 1000) / 1000.0 - 3.0};
        uint32_t     ri;
        double       rd;
        const size_t n = index.knnSearch(q, 1, &ri, &rd);
        ASSERT_EQ(n, 1u);
        EXPECT_TRUE(live.count(ri) == 1);
        uint32_t     bi;
        const double bd = bruteforce_nn(cloud, live, q, bi);
        EXPECT_NEAR(rd, bd, 1e-9);
    }
}

TEST(kdtree_incremental, removeBox_and_findWithinBox)
{
    srand(321);
    inc_cloud_t  cloud;
    inc_tree_t   index(3, cloud);
    const size_t N = 3000;
    for (size_t i = 0; i < N; ++i)
        cloud.pts.push_back(
            {20.0 * (rand() % 1000) / 1000.0 - 10.0, 20.0 * (rand() % 1000) / 1000.0 - 10.0,
             20.0 * (rand() % 1000) / 1000.0 - 10.0});
    index.addPoints(0, static_cast<uint32_t>(N) - 1);
    std::set<uint32_t> live;
    for (uint32_t i = 0; i < N; ++i) live.insert(i);

    // Delete an inner box.
    const double            blo[3] = {-2, -2, -2}, bhi[3] = {2, 2, 2};
    inc_tree_t::BoundingBox box;
    for (int d = 0; d < 3; ++d)
    {
        box[d].low  = blo[d];
        box[d].high = bhi[d];
    }
    index.removeBox(box);
    for (auto it = live.begin(); it != live.end();)
    {
        if (inc_in_box(cloud, *it, blo, bhi))
            it = live.erase(it);
        else
            ++it;
    }
    EXPECT_EQ(index.size(), live.size());

    // findWithinBox over a different query box, compared against the oracle.
    const double            qlo[3] = {-5, -5, -5}, qhi[3] = {5, 5, 5};
    inc_tree_t::BoundingBox qbox;
    for (int d = 0; d < 3; ++d)
    {
        qbox[d].low  = qlo[d];
        qbox[d].high = qhi[d];
    }
    std::vector<uint32_t>             got;
    nanoflann::BoxResultSet<uint32_t> brs(got);
    (void)index.findWithinBox(brs, qbox);

    std::set<uint32_t> expected;
    for (uint32_t i : live)
        if (inc_in_box(cloud, i, qlo, qhi)) expected.insert(i);
    EXPECT_EQ(got.size(), expected.size());
    for (uint32_t i : got) EXPECT_TRUE(expected.count(i) == 1);
}

TEST(kdtree_incremental, bounded_memory_under_churn)
{
    // Sliding-window LiDAR-like churn: constant-velocity insert + trim. The
    // physical node count (live + tombstones) must stay bounded, not grow with
    // the total number of inserted points.
    inc_cloud_t                            cloud;
    inc_tree_t                             index(3, cloud);
    std::mt19937                           g(2024);
    std::uniform_real_distribution<double> U(0.0, 10.0);
    std::set<uint32_t>                     live;

    double cx      = 0.0;
    size_t maxPhys = 0;
    for (int frame = 0; frame < 150; ++frame)
    {
        cx += 1.0;  // constant velocity
        const uint32_t start = static_cast<uint32_t>(cloud.pts.size());
        for (int i = 0; i < 400; ++i) cloud.pts.push_back({cx + U(g), U(g), U(g)});
        const uint32_t end = static_cast<uint32_t>(cloud.pts.size()) - 1;
        index.addPoints(start, end);
        for (uint32_t i = start; i <= end; ++i) live.insert(i);

        inc_tree_t::BoundingBox keep;
        keep[0].low  = cx - 5;
        keep[0].high = cx + 15;
        keep[1].low  = -1e9;
        keep[1].high = 1e9;
        keep[2].low  = -1e9;
        keep[2].high = 1e9;
        index.removeOutsideBox(keep);
        for (auto it = live.begin(); it != live.end();)
        {
            const double x = cloud.pts[*it].x;
            if (x < cx - 5 || x > cx + 15)
                it = live.erase(it);
            else
                ++it;
        }
        ASSERT_EQ(index.size(), live.size());
        if (frame > 30) maxPhys = std::max(maxPhys, index.physicalSize());
    }
    // Steady-state window holds ~8000 live points; tombstones must stay a small
    // multiple of that, NOT proportional to the ~60000 total inserts.
    EXPECT_LT(index.physicalSize(), 4 * index.size());
    EXPECT_LT(maxPhys, static_cast<size_t>(30000));
}

TEST(kdtree_incremental, radius_and_rknn_vs_bruteforce)
{
    srand(555);
    inc_cloud_t  cloud;
    inc_tree_t   index(3, cloud);
    const size_t N = 1500;
    for (size_t i = 0; i < N; ++i)
        cloud.pts.push_back(
            {10.0 * (rand() % 1000) / 1000.0, 10.0 * (rand() % 1000) / 1000.0,
             10.0 * (rand() % 1000) / 1000.0});
    index.addPoints(0, static_cast<uint32_t>(N) - 1);
    std::set<uint32_t> live;
    for (uint32_t i = 0; i < N; ++i) live.insert(i);

    const double radius = 1.5;  // squared-distance threshold for L2
    for (int t = 0; t < 100; ++t)
    {
        const double q[3] = {
            10.0 * (rand() % 1000) / 1000.0, 10.0 * (rand() % 1000) / 1000.0,
            10.0 * (rand() % 1000) / 1000.0};
        std::vector<nanoflann::ResultItem<uint32_t, double>> matches;
        (void)index.radiusSearch(q, radius, matches);

        size_t expected = 0;
        for (uint32_t i : live)
        {
            const double dx = q[0] - cloud.pts[i].x, dy = q[1] - cloud.pts[i].y,
                         dz = q[2] - cloud.pts[i].z;
            if (dx * dx + dy * dy + dz * dz < radius) expected++;
        }
        EXPECT_EQ(matches.size(), expected);
    }
}

// Reclamation contract: after removeOutsideBox + the rebuild it triggers,
// acquireRemovedPoints() must report exactly the physically-evicted indices
// (all outside the keep box, unique) so the caller can recycle those dataset
// slots and keep its storage bounded.
TEST(kdtree_incremental, removeOutsideBox_acquireRemovedPoints)
{
    srand(99);
    inc_cloud_t cloud;
    inc_tree_t  index(3, cloud);
    index.setCollectRemovedPoints(true);
    const size_t N = 4000;
    for (size_t i = 0; i < N; ++i)
        cloud.pts.push_back(
            {20.0 * (rand() % 1000) / 1000.0 - 10.0, 20.0 * (rand() % 1000) / 1000.0 - 10.0,
             20.0 * (rand() % 1000) / 1000.0 - 10.0});
    index.addPoints(0, static_cast<uint32_t>(N) - 1);

    const double            lo[3] = {-4, -4, -4}, hi[3] = {4, 4, 4};
    inc_tree_t::BoundingBox keep;
    for (int d = 0; d < 3; ++d)
    {
        keep[d].low  = lo[d];
        keep[d].high = hi[d];
    }
    index.removeOutsideBox(keep);

    std::set<uint32_t> expectedEvicted, expectedLive;
    for (uint32_t i = 0; i < N; ++i)
        (inc_in_box(cloud, i, lo, hi) ? expectedLive : expectedEvicted).insert(i);

    const std::vector<uint32_t> reported = index.acquireRemovedPoints();
    const std::set<uint32_t>    reportedSet(reported.begin(), reported.end());
    EXPECT_EQ(reported.size(), reportedSet.size());  // unique
    // Every reported slot is genuinely evicted (outside) and none is still live.
    for (uint32_t i : reportedSet)
    {
        EXPECT_TRUE(expectedEvicted.count(i) == 1);
        EXPECT_TRUE(expectedLive.count(i) == 0);
    }
    // A fully reclaimed run (heavy deletion -> root rebuild) reports them all.
    if (index.physicalSize() == index.size())
    {
        EXPECT_EQ(reportedSet, expectedEvicted);
    }

    // A second call returns nothing new.
    EXPECT_TRUE(index.acquireRemovedPoints().empty());
}

// boundingBox(), reserve(), and buildFromIndices() on a *populated* index
// (rebuild-from-subset; exercises the recycle-existing-nodes branch).
TEST(kdtree_incremental, buildFromIndices_boundingBox_reserve)
{
    inc_cloud_t cloud;
    inc_tree_t  index(3, cloud);
    index.reserve(1000);  // pre-size the index->node map
    const size_t N = 600;
    for (size_t i = 0; i < N; ++i)
        cloud.pts.push_back(
            {static_cast<double>(i % 10), static_cast<double>((i / 10) % 10),
             static_cast<double>(i / 100)});
    index.addPoints(0, static_cast<uint32_t>(N) - 1);

    // boundingBox encloses all inserted points.
    auto bb = index.boundingBox();
    EXPECT_LE(bb[0].low, 0.0);
    EXPECT_GE(bb[0].high, 9.0);
    EXPECT_GE(bb[2].high, 5.0);

    // Rebuild from a subset of indices (this index is non-empty -> takes the
    // recycle-existing-nodes path inside buildFromIndices).
    std::vector<uint32_t> subset;
    for (uint32_t i = 0; i < N; i += 2) subset.push_back(i);  // keep even indices
    index.buildFromIndices(subset);
    EXPECT_EQ(index.size(), subset.size());
    EXPECT_EQ(index.physicalSize(), subset.size());  // fresh balanced tree, no tombstones

    // KNN now only returns kept (even) indices.
    std::set<uint32_t> kept(subset.begin(), subset.end());
    for (uint32_t i = 0; i < N; ++i)
    {
        const double q[3] = {cloud.pts[i].x, cloud.pts[i].y, cloud.pts[i].z};
        uint32_t     ri;
        double       rd;
        ASSERT_EQ(index.knnSearch(q, 1, &ri, &rd), 1u);
        EXPECT_TRUE(kept.count(ri) == 1);
    }
}

// Insert into a region whose subtree was bulk-killed by removeOutsideBox while
// inline rebuilds are disabled, so the lazy whole-subtree tombstone survives and
// the insertion descent must push it down (covers pushDownDelete()).
TEST(kdtree_incremental, pushdown_delete_into_killed_region)
{
    inc_cloud_t cloud;
    inc_tree_t  index(3, cloud);
    index.setInlineRebuild(false);  // keep treeDeleted subtrees around (no reclaim)

    // Two well-separated clusters so they live in separate subtrees.
    for (int i = 0; i < 200; ++i) cloud.pts.push_back({0.1 * i / 200.0, 0.0, 0.0});  // A near x=0
    for (int i = 0; i < 200; ++i)
        cloud.pts.push_back({100.0 + 0.1 * i / 200.0, 0.0, 0.0});  // B near x=100
    index.addPoints(0, 399);
    ASSERT_EQ(index.size(), 400u);

    // Kill cluster B (everything with x>10): its subtree becomes treeDeleted and,
    // with inline rebuild off, is NOT reclaimed.
    inc_tree_t::BoundingBox keep;
    keep[0].low  = -1;
    keep[0].high = 10;
    keep[1].low  = -1;
    keep[1].high = 1;
    keep[2].low  = -1;
    keep[2].high = 1;
    index.removeOutsideBox(keep);
    ASSERT_EQ(index.size(), 200u);  // only cluster A remains live

    // Now insert a fresh point inside the killed region (x~100): the descent
    // reaches the treeDeleted subtree and must push the deletion down.
    const uint32_t newIdx = static_cast<uint32_t>(cloud.pts.size());
    cloud.pts.push_back({100.05, 0.0, 0.0});
    index.addPoint(newIdx);
    EXPECT_EQ(index.size(), 201u);

    // The new point is the nearest neighbor of its own location; the old killed
    // points stay deleted.
    const double q[3] = {100.05, 0.0, 0.0};
    uint32_t     ri;
    double       rd;
    ASSERT_EQ(index.knnSearch(q, 1, &ri, &rd), 1u);
    EXPECT_EQ(ri, newIdx);
    EXPECT_NEAR(rd, 0.0, 1e-9);
}

// ===========================================================================
//  Tests for saveIndex()/loadIndex() (tree-topology persistence)
// ===========================================================================

// Round-trips a tree that has been through inserts, point removals AND a
// box-trim (so it exercises both makeLeaf's depth-based divfeat and
// buildBalanced's data-dependent widest-axis divfeat), and checks that a
// freshly constructed index loaded from the saved stream returns identical
// query results to the original for many random queries.
TEST(kdtree_incremental, save_load_roundtrip_matches_queries)
{
    srand(2026);
    inc_cloud_t  cloud;
    inc_tree_t   index(3, cloud);
    const size_t N = 3000;
    for (size_t i = 0; i < N; ++i)
        cloud.pts.push_back(
            {20.0 * (rand() % 1000) / 1000.0 - 10.0, 20.0 * (rand() % 1000) / 1000.0 - 10.0,
             20.0 * (rand() % 1000) / 1000.0 - 10.0});
    index.addPoints(0, static_cast<uint32_t>(N) - 1);

    // Remove ~15% of points at random (lazy tombstones, no full rebuild).
    std::vector<uint32_t> order;
    for (uint32_t i = 0; i < N; ++i) order.push_back(i);
    std::mt19937 g(11);
    std::shuffle(order.begin(), order.end(), g);
    for (size_t i = 0; i < order.size() * 15 / 100; ++i) index.removePoint(order[i]);

    // Trim a box (removeOutsideBox), which triggers scapegoat rebuilds
    // (buildBalanced, data-dependent divfeat) on some subtrees.
    inc_tree_t::BoundingBox keep;
    for (int d = 0; d < 3; ++d)
    {
        keep[d].low  = -8;
        keep[d].high = 8;
    }
    index.removeOutsideBox(keep);

    ASSERT_GT(index.size(), 0u);

    std::ostringstream oss(std::ios::binary);
    index.saveIndex(oss);

    // A fresh index attached to the very same dataset, per loadIndex()'s
    // documented precondition.
    inc_tree_t         loaded(3, cloud);
    std::istringstream iss(oss.str(), std::ios::binary);
    loaded.loadIndex(iss);

    EXPECT_EQ(loaded.size(), index.size());
    EXPECT_EQ(loaded.physicalSize(), index.physicalSize());

    for (int t = 0; t < 300; ++t)
    {
        const double q[3] = {
            20.0 * (rand() % 1000) / 1000.0 - 10.0, 20.0 * (rand() % 1000) / 1000.0 - 10.0,
            20.0 * (rand() % 1000) / 1000.0 - 10.0};

        uint32_t     ri_orig, ri_loaded;
        double       rd_orig, rd_loaded;
        const size_t n_orig   = index.knnSearch(q, 1, &ri_orig, &rd_orig);
        const size_t n_loaded = loaded.knnSearch(q, 1, &ri_loaded, &rd_loaded);
        ASSERT_EQ(n_orig, n_loaded);
        if (n_orig == 0) continue;
        EXPECT_EQ(ri_orig, ri_loaded);
        EXPECT_NEAR(rd_orig, rd_loaded, 1e-9);

        std::vector<nanoflann::ResultItem<uint32_t, double>> rad_orig, rad_loaded;
        (void)index.radiusSearch(q, 2.0, rad_orig);
        (void)loaded.radiusSearch(q, 2.0, rad_loaded);
        EXPECT_EQ(rad_orig.size(), rad_loaded.size());
    }
}

// A tree that was bulk-built with buildFromIndices() (no incremental inserts
// at all): every internal node's divfeat comes from the widest-spread-axis
// rule, never the depth-based default, so this exercises that path in
// isolation.
TEST(kdtree_incremental, save_load_after_buildFromIndices)
{
    inc_cloud_t  cloud;
    inc_tree_t   index(3, cloud);
    const size_t N = 500;
    for (size_t i = 0; i < N; ++i)
        cloud.pts.push_back(
            {static_cast<double>(i % 10), static_cast<double>((i / 10) % 10),
             static_cast<double>(i / 100)});
    std::vector<uint32_t> idxs;
    for (uint32_t i = 0; i < N; i += 2) idxs.push_back(i);  // even only
    index.buildFromIndices(idxs);

    std::ostringstream oss(std::ios::binary);
    index.saveIndex(oss);

    inc_tree_t         loaded(3, cloud);
    std::istringstream iss(oss.str(), std::ios::binary);
    loaded.loadIndex(iss);

    EXPECT_EQ(loaded.size(), index.size());
    for (uint32_t i = 0; i < N; ++i)
    {
        const double q[3] = {cloud.pts[i].x, cloud.pts[i].y, cloud.pts[i].z};
        uint32_t     ri_orig, ri_loaded;
        double       rd_orig, rd_loaded;
        ASSERT_EQ(index.knnSearch(q, 1, &ri_orig, &rd_orig), 1u);
        ASSERT_EQ(loaded.knnSearch(q, 1, &ri_loaded, &rd_loaded), 1u);
        EXPECT_EQ(ri_orig, ri_loaded);
    }
}

// Edge cases: an empty tree and a single-point tree must round-trip too.
TEST(kdtree_incremental, save_load_empty_and_single_point)
{
    {
        inc_cloud_t        cloud;
        inc_tree_t         index(3, cloud);  // never populated
        std::ostringstream oss(std::ios::binary);
        index.saveIndex(oss);

        inc_tree_t         loaded(3, cloud);
        std::istringstream iss(oss.str(), std::ios::binary);
        loaded.loadIndex(iss);
        EXPECT_EQ(loaded.size(), 0u);
        EXPECT_TRUE(loaded.empty());
    }
    {
        inc_cloud_t cloud;
        cloud.pts.push_back({1.0, 2.0, 3.0});
        inc_tree_t index(3, cloud);
        index.addPoint(0);

        std::ostringstream oss(std::ios::binary);
        index.saveIndex(oss);

        inc_tree_t         loaded(3, cloud);
        std::istringstream iss(oss.str(), std::ios::binary);
        loaded.loadIndex(iss);
        EXPECT_EQ(loaded.size(), 1u);

        const double q[3] = {1.0, 2.0, 3.0};
        uint32_t     ri;
        double       rd;
        ASSERT_EQ(loaded.knnSearch(q, 1, &ri, &rd), 1u);
        EXPECT_EQ(ri, 0u);
        EXPECT_NEAR(rd, 0.0, 1e-12);
    }
}

// loadIndex() must reject a stream that was not written by saveIndex()
// (wrong magic number), instead of silently misinterpreting the bytes.
TEST(kdtree_incremental, load_index_wrong_magic_throws)
{
    inc_cloud_t cloud;
    cloud.pts.push_back({0.0, 0.0, 0.0});
    inc_tree_t index(3, cloud);
    index.addPoint(0);

    std::istringstream garbage(std::string(64, '\xAB'), std::ios::binary);
    EXPECT_THROW(index.loadIndex(garbage), std::runtime_error);
}

#ifndef NANOFLANN_NO_THREADS
// The MT wrapper's saveIndex()/loadIndex() must block on any in-flight
// background rebuild and (de)serialize the active tree transparently.
TEST(kdtree_incremental, async_mt_save_load_roundtrip)
{
    using mt_tree_t = nanoflann::KDTreeSingleIndexIncrementalAdaptorMT<
        nanoflann::L2_Simple_Adaptor<double, inc_cloud_t>, inc_cloud_t, 3, uint32_t>;

    inc_cloud_t cloud;
    cloud.pts.reserve(200000);
    mt_tree_t index(3, cloud, nanoflann::KDTreeIncrementalIndexParams(0.8f, 0.5f), 1.3, 2000);

    std::mt19937                           g(4321);
    std::uniform_real_distribution<double> U(0.0, 10.0);
    for (int frame = 0; frame < 10; ++frame)
    {
        const uint32_t start = static_cast<uint32_t>(cloud.pts.size());
        for (int i = 0; i < 1500; ++i) cloud.pts.push_back({U(g), U(g), U(g)});
        const uint32_t end = static_cast<uint32_t>(cloud.pts.size()) - 1;
        index.addPoints(start, end);  // large enough batches to trigger a background rebuild
    }

    std::ostringstream oss(std::ios::binary);
    index.saveIndex(oss);  // blocks on any pending rebuild internally

    mt_tree_t loaded(3, cloud, nanoflann::KDTreeIncrementalIndexParams(0.8f, 0.5f), 1.3, 2000);
    std::istringstream iss(oss.str(), std::ios::binary);
    loaded.loadIndex(iss);

    EXPECT_EQ(loaded.size(), index.size());

    for (int t = 0; t < 200; ++t)
    {
        const double q[3] = {U(g), U(g), U(g)};
        uint32_t     ri_orig, ri_loaded;
        double       rd_orig, rd_loaded;
        ASSERT_EQ(index.knnSearch(q, 1, &ri_orig, &rd_orig), 1u);
        ASSERT_EQ(loaded.knnSearch(q, 1, &ri_loaded, &rd_loaded), 1u);
        EXPECT_EQ(ri_orig, ri_loaded);
        EXPECT_NEAR(rd_orig, rd_loaded, 1e-9);
    }
}
#endif  // NANOFLANN_NO_THREADS

#ifndef NANOFLANN_NO_THREADS
TEST(kdtree_incremental, async_mt_vs_bruteforce_under_churn)
{
    using mt_tree_t = nanoflann::KDTreeSingleIndexIncrementalAdaptorMT<
        nanoflann::L2_Simple_Adaptor<double, inc_cloud_t>, inc_cloud_t, 3, uint32_t>;

    inc_cloud_t cloud;
    cloud.pts.reserve(400000);  // stable storage required while a rebuild is in flight
    mt_tree_t index(3, cloud, nanoflann::KDTreeIncrementalIndexParams(0.8f, 0.5f), 1.3, 2000);
    std::set<uint32_t>                     live;
    std::mt19937                           g(2024);
    std::uniform_real_distribution<double> U(0.0, 10.0);

    double cx         = 0.0;
    bool   sawRebuild = false;
    for (int frame = 0; frame < 80; ++frame)
    {
        cx += 1.0;
        const uint32_t start = static_cast<uint32_t>(cloud.pts.size());
        for (int i = 0; i < 1500; ++i) cloud.pts.push_back({cx + U(g), U(g), U(g)});
        const uint32_t end = static_cast<uint32_t>(cloud.pts.size()) - 1;
        index.addPoints(start, end);
        for (uint32_t i = start; i <= end; ++i) live.insert(i);

        mt_tree_t::BoundingBox keep;
        keep[0].low  = cx - 5;
        keep[0].high = cx + 15;
        keep[1].low  = -1e9;
        keep[1].high = 1e9;
        keep[2].low  = -1e9;
        keep[2].high = 1e9;
        index.removeOutsideBox(keep);
        for (auto it = live.begin(); it != live.end();)
        {
            const double x = cloud.pts[*it].x;
            if (x < cx - 5 || x > cx + 15)
                it = live.erase(it);
            else
                ++it;
        }
        if (index.isRebuilding()) sawRebuild = true;
        ASSERT_EQ(index.size(), live.size());
    }
    index.sync();  // finish any in-flight background rebuild
    EXPECT_TRUE(sawRebuild);  // background rebuilds actually triggered
    EXPECT_EQ(index.size(), live.size());
    EXPECT_LT(index.physicalSize(), 4 * index.size());  // bounded memory

    for (int t = 0; t < 500; ++t)
    {
        const double q[3] = {cx + U(g), U(g), U(g)};
        uint32_t     ri;
        double       rd;
        const size_t n = index.knnSearch(q, 1, &ri, &rd);
        ASSERT_EQ(n, 1u);
        EXPECT_TRUE(live.count(ri) == 1);
        uint32_t     bi;
        const double bd = bruteforce_nn(cloud, live, q, bi);
        EXPECT_NEAR(rd, bd, 1e-9);
    }
}

// The MT index must also report freed dataset slots (across background rebuilds)
// so a streaming caller can recycle storage and stay bounded.
TEST(kdtree_incremental, async_mt_slot_recycling_bounds_dataset)
{
    using mt_tree_t = nanoflann::KDTreeSingleIndexIncrementalAdaptorMT<
        nanoflann::L2_Simple_Adaptor<double, inc_cloud_t>, inc_cloud_t, 3, uint32_t>;

    inc_cloud_t cloud;
    cloud.pts.reserve(60000);  // bounded thanks to recycling; reserve for MT safety
    mt_tree_t index(3, cloud, nanoflann::KDTreeIncrementalIndexParams(0.8f, 0.5f), 1.3, 2000);
    index.setCollectRemovedPoints(true);

    std::vector<uint32_t>                  freeSlots;
    std::set<uint32_t>                     live;
    std::mt19937                           g(2024);
    std::uniform_real_distribution<double> U(0.0, 10.0);

    double cx            = 0.0;
    size_t totalInserted = 0, maxDataset = 0;
    for (int frame = 0; frame < 120; ++frame)
    {
        cx += 1.0;
        for (int i = 0; i < 800; ++i)
        {
            const inc_cloud_t::Point p{cx + U(g), U(g), U(g)};
            uint32_t                 slot;
            if (!freeSlots.empty())
            {
                slot = freeSlots.back();
                freeSlots.pop_back();
                cloud.pts[slot] = p;
            }
            else
            {
                slot = static_cast<uint32_t>(cloud.pts.size());
                cloud.pts.push_back(p);
            }
            index.addPoint(slot);
            live.insert(slot);
            ++totalInserted;
        }
        mt_tree_t::BoundingBox keep;
        keep[0].low  = cx - 5;
        keep[0].high = cx + 15;
        keep[1].low  = -1e9;
        keep[1].high = 1e9;
        keep[2].low  = -1e9;
        keep[2].high = 1e9;
        index.removeOutsideBox(keep);
        for (auto it = live.begin(); it != live.end();)
        {
            const double x = cloud.pts[*it].x;
            if (x < cx - 5 || x > cx + 15)
                it = live.erase(it);
            else
                ++it;
        }
        for (uint32_t s : index.acquireRemovedPoints()) freeSlots.push_back(s);
        maxDataset = std::max(maxDataset, cloud.pts.size());
        ASSERT_EQ(index.size(), live.size());
    }
    index.sync();
    for (uint32_t s : index.acquireRemovedPoints()) freeSlots.push_back(s);

    // Dataset stayed bounded (far below total inserts) via slot recycling.
    EXPECT_LT(maxDataset, totalInserted / 2);

    // Final KNN correctness on recycled storage.
    for (int t = 0; t < 300; ++t)
    {
        const double q[3] = {cx + U(g), U(g), U(g)};
        uint32_t     ri;
        double       rd;
        const size_t n = index.knnSearch(q, 1, &ri, &rd);
        ASSERT_EQ(n, 1u);
        EXPECT_TRUE(live.count(ri) == 1);
        uint32_t     bi;
        const double bd = bruteforce_nn(cloud, live, q, bi);
        EXPECT_NEAR(rd, bd, 1e-9);
    }
}

// Exercises the MT wrapper's op-log replay for Remove and RemoveBox (ops issued
// while a background rebuild is in flight), the background rebuild callback, and
// the forwarder observers (boundingBox / snapshotLiveIndices / reserve).
TEST(kdtree_incremental, async_mt_replay_remove_box_and_callback)
{
    using mt_tree_t = nanoflann::KDTreeSingleIndexIncrementalAdaptorMT<
        nanoflann::L2_Simple_Adaptor<double, inc_cloud_t>, inc_cloud_t, 3, uint32_t>;

    inc_cloud_t cloud;
    cloud.pts.reserve(600000);  // stable storage for the background reads
    mt_tree_t index(3, cloud, nanoflann::KDTreeIncrementalIndexParams(0.8f, 0.5f), 1.3, 2000);
    index.reserve(600000);

    // The callback runs on the background worker thread for each rebuilt tree.
    std::atomic<int> cbCalls{0};
    index.setRebuildCallback(
        [&cbCalls](mt_tree_t::Inner& fresh)
        {
            // Touch the fresh tree to make sure it is usable inside the callback.
            (void)fresh.size();
            cbCalls.fetch_add(1, std::memory_order_relaxed);
        });

    std::set<uint32_t>                     live;
    std::mt19937                           g(123);
    std::uniform_real_distribution<double> U(-50.0, 50.0);

    auto inBox = [](const inc_cloud_t& c, uint32_t i, const double lo[3], const double hi[3])
    { return inc_in_box(c, i, lo, hi); };

    // Several rounds: each big insert re-triggers a background rebuild, and the
    // removePoint / removeBox issued right afterwards land in the op-log while
    // the (large) build is still running.
    for (int round = 0; round < 4; ++round)
    {
        const uint32_t start = static_cast<uint32_t>(cloud.pts.size());
        for (int i = 0; i < 60000; ++i) cloud.pts.push_back({U(g), U(g), U(g)});
        const uint32_t end = static_cast<uint32_t>(cloud.pts.size()) - 1;
        index.addPoints(start, end);  // triggers a background rebuild
        for (uint32_t i = start; i <= end; ++i) live.insert(i);

        // Remove a handful of individual points (logged Remove while building).
        for (uint32_t k = 0; k < 20; ++k)
        {
            const uint32_t victim = start + k * 7;
            index.removePoint(victim);
            live.erase(victim);
        }
        // Remove a box region (logged RemoveBox while building).
        const double           blo[3] = {-50, -50, -50}, bhi[3] = {-30, -30, -30};
        mt_tree_t::BoundingBox box;
        for (int d = 0; d < 3; ++d)
        {
            box[d].low  = blo[d];
            box[d].high = bhi[d];
        }
        index.removeBox(box);
        for (auto it = live.begin(); it != live.end();)
        {
            if (inBox(cloud, *it, blo, bhi))
                it = live.erase(it);
            else
                ++it;
        }
    }
    index.sync();  // drain the final background rebuild (replays the op-log)

    EXPECT_GT(cbCalls.load(), 0);  // background callback actually ran
    EXPECT_EQ(index.size(), live.size());

    // Forwarder observers:
    EXPECT_EQ(index.physicalSize() >= index.size(), true);
    std::vector<uint32_t> liveIdx;
    index.snapshotLiveIndices(liveIdx);
    EXPECT_EQ(liveIdx.size(), live.size());
    auto bb = index.boundingBox();
    EXPECT_LE(bb[0].low, bb[0].high);

    // Correctness vs brute force on the final live set.
    for (int t = 0; t < 500; ++t)
    {
        const double q[3] = {U(g), U(g), U(g)};
        uint32_t     ri;
        double       rd;
        const size_t n = index.knnSearch(q, 1, &ri, &rd);
        ASSERT_EQ(n, 1u);
        EXPECT_TRUE(live.count(ri) == 1);
        uint32_t     bi;
        const double bd = bruteforce_nn(cloud, live, q, bi);
        EXPECT_NEAR(rd, bd, 1e-9);
    }
}
#endif  // NANOFLANN_NO_THREADS
