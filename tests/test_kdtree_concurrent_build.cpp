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
//  Tests for the concurrent index build (n_thread_build > 1).
//
//  The split decisions taken while building depend only on the point range
//  being divided, so distributing that work over threads must not change the
//  result: the concurrent build has to produce exactly the tree the serial
//  build produces, node for node. That is a much stronger invariant than
//  "the queries still return plausible neighbors", and it is what these tests
//  check, over point counts large enough that tasks really do get spawned.
// ===========================================================================

#include "test_helpers.h"

namespace
{
using num_t   = double;
using cloud_t = PointCloud<num_t>;

using tree_t = KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<num_t, cloud_t>, cloud_t, 3 /* dim */>;
using dyn_tree_t =
    KDTreeSingleIndexDynamicAdaptor<L2_Simple_Adaptor<num_t, cloud_t>, cloud_t, 3 /* dim */>;

// Thread counts to compare against the serial build. Values above the local
// core count are included on purpose: they must still be correct, just not
// faster.
const std::vector<unsigned int> kThreadCounts = {2, 3, 4, 8, 16, 0 /* = all cores */};

/** An order-sensitive digest of everything that defines a built tree. */
struct TreeDigest
{
    uint64_t hash     = 1469598103934665603ULL;  // FNV-1a offset basis
    size_t   nodes    = 0;
    size_t   leaves   = 0;
    size_t   maxDepth = 0;

    void feedBytes(const void* data, const size_t len)
    {
        const auto* p = static_cast<const unsigned char*>(data);
        for (size_t i = 0; i < len; i++)
        {
            hash ^= p[i];
            hash *= 1099511628211ULL;
        }
    }
    void feed(const uint64_t v) { feedBytes(&v, sizeof(v)); }
    void feed(const double v) { feedBytes(&v, sizeof(v)); }
};

void walkTree(const tree_t::Node* n, TreeDigest& d, const size_t depth)
{
    d.nodes++;
    d.maxDepth = std::max(d.maxDepth, depth);

    const bool isLeaf = n->isLeaf();
    d.feed(static_cast<uint64_t>(isLeaf ? 1 : 0));
    if (isLeaf)
    {
        d.leaves++;
        d.feed(static_cast<uint64_t>(n->node_type.lr.left));
        d.feed(static_cast<uint64_t>(n->node_type.lr.right));
        return;
    }
    d.feed(static_cast<uint64_t>(n->node_type.sub.divfeat));
    d.feed(static_cast<double>(n->node_type.sub.divlow));
    d.feed(static_cast<double>(n->node_type.sub.divhigh));
    walkTree(n + 1, d, depth + 1);  // left child: always the next node
    walkTree(n + n->child2, d, depth + 1);
}

TreeDigest digestOf(const tree_t& index)
{
    TreeDigest d;
    // The point-index permutation is part of the tree's identity: two trees of
    // the same shape over different orderings of vAcc_ are different trees.
    for (const auto v : index.vAcc_) d.feed(static_cast<uint64_t>(v));
    for (const auto& iv : index.root_bbox_)
    {
        d.feed(static_cast<double>(iv.low));
        d.feed(static_cast<double>(iv.high));
    }
    if (!index.nodes_.empty()) walkTree(index.nodes_.data(), d, 0);
    return d;
}

void expectSameDigest(const TreeDigest& got, const TreeDigest& ref)
{
    EXPECT_EQ(got.nodes, ref.nodes);
    EXPECT_EQ(got.leaves, ref.leaves);
    EXPECT_EQ(got.maxDepth, ref.maxDepth);
    EXPECT_EQ(got.hash, ref.hash);
}

/** kNN answers (indices *and* distances) for a fixed set of query points. */
using QueryAnswers = std::vector<std::pair<uint32_t, num_t>>;

std::vector<std::array<num_t, 3>> makeQueries(const size_t n, const num_t range)
{
    std::mt19937                          rng(7);
    std::uniform_real_distribution<num_t> d(-range, range);
    std::vector<std::array<num_t, 3>>     qs(n);
    for (auto& q : qs) q = {d(rng), d(rng), d(rng)};
    return qs;
}

template <class INDEX>
QueryAnswers answersOf(
    const INDEX& index, const std::vector<std::array<num_t, 3>>& qs, const size_t K)
{
    QueryAnswers          out;
    std::vector<uint32_t> ri(K);
    std::vector<num_t>    rd(K);
    for (const auto& q : qs)
    {
        KNNResultSet<num_t, uint32_t> rs(K);
        rs.init(ri.data(), rd.data());
        index.findNeighbors(rs, q.data());
        for (size_t i = 0; i < rs.size(); i++) out.emplace_back(ri[i], rd[i]);
        // A sentinel keeps the per-query boundaries in the comparison, so a
        // difference in how many neighbors were found cannot cancel out.
        out.emplace_back(std::numeric_limits<uint32_t>::max(), static_cast<num_t>(rs.size()));
    }
    return out;
}

// --- point distributions -----------------------------------------------------
// Beyond the uniform case, each of these stresses the split logic in a way that
// makes the two children very unbalanced, which is what the concurrent builder
// keys on when deciding what to hand to a task.

cloud_t uniformCloud(const size_t N, const uint32_t seed)
{
    cloud_t                               c;
    std::mt19937                          rng(seed);
    std::uniform_real_distribution<num_t> d(-50, 50);
    c.pts.resize(N);
    for (auto& p : c.pts) p = {d(rng), d(rng), d(rng)};
    return c;
}

cloud_t duplicateCloud(const size_t N)
{
    cloud_t c;
    c.pts.assign(N, {1.0, 2.0, 3.0});
    return c;
}

cloud_t collinearCloud(const size_t N)
{
    cloud_t c;
    c.pts.resize(N);
    for (size_t i = 0; i < N; i++) c.pts[i] = {static_cast<num_t>(i), 0.0, 0.0};
    return c;
}

cloud_t skewedCloud(const size_t N, const uint32_t seed)
{
    // 99% of the points in a tiny cluster, 1% spread far away.
    cloud_t                               c;
    std::mt19937                          rng(seed);
    std::uniform_real_distribution<num_t> tight(-0.01, 0.01);
    std::uniform_real_distribution<num_t> wide(-1000, 1000);
    c.pts.resize(N);
    for (size_t i = 0; i < N; i++)
    {
        if (i % 100 == 0)
            c.pts[i] = {wide(rng), wide(rng), wide(rng)};
        else
            c.pts[i] = {tight(rng), tight(rng), tight(rng)};
    }
    return c;
}

void checkConcurrentMatchesSerial(
    const std::string& what, const cloud_t& cloud, const size_t leafMaxSize)
{
    SCOPED_TRACE(
        what + ", N=" + std::to_string(cloud.pts.size()) +
        ", leaf_max_size=" + std::to_string(leafMaxSize));

    const auto qs = makeQueries(100, 60);

    KDTreeSingleIndexAdaptorParams serialParams;
    serialParams.leaf_max_size  = leafMaxSize;
    serialParams.n_thread_build = 1;
    const tree_t serial(3, cloud, serialParams);
    const auto   refDigest  = digestOf(serial);
    const auto   refAnswers = answersOf(serial, qs, 10);

    for (const unsigned int nThreads : kThreadCounts)
    {
        SCOPED_TRACE("n_thread_build=" + std::to_string(nThreads));

        KDTreeSingleIndexAdaptorParams params;
        params.leaf_max_size  = leafMaxSize;
        params.n_thread_build = nThreads;
        const tree_t concurrent(3, cloud, params);

        expectSameDigest(digestOf(concurrent), refDigest);
        EXPECT_TRUE(answersOf(concurrent, qs, 10) == refAnswers);
    }
}

void checkDynamicConcurrentMatchesSerial(const cloud_t& cloud, const size_t nToRemove)
{
    SCOPED_TRACE(
        "dynamic, N=" + std::to_string(cloud.pts.size()) +
        ", removed=" + std::to_string(nToRemove));

    const auto qs = makeQueries(100, 60);

    // Same removals in every run, so any difference is down to the build.
    const auto churn = [&](dyn_tree_t& index)
    {
        std::mt19937 rng(11);
        for (size_t i = 0; i < nToRemove; i++) index.removePoint(rng() % cloud.pts.size());
    };

    KDTreeSingleIndexAdaptorParams serialParams;
    serialParams.leaf_max_size  = 10;
    serialParams.n_thread_build = 1;
    dyn_tree_t serial(3, cloud, serialParams);
    churn(serial);
    const auto refAnswers = answersOf(serial, qs, 10);

    for (const unsigned int nThreads : kThreadCounts)
    {
        SCOPED_TRACE("n_thread_build=" + std::to_string(nThreads));

        KDTreeSingleIndexAdaptorParams params;
        params.leaf_max_size  = 10;
        params.n_thread_build = nThreads;
        dyn_tree_t concurrent(3, cloud, params);
        churn(concurrent);

        EXPECT_TRUE(answersOf(concurrent, qs, 10) == refAnswers);
    }
}
}  // namespace

TEST(kdtree_concurrent_build, tree_identical_to_serial_uniform)
{
    // Point counts straddle the size below which the builder keeps a subtree on
    // the calling thread instead of spawning a task for it, so both the
    // "no task was spawned" and the "tasks were spawned" paths get exercised.
    for (const size_t N : {1u, 2u, 11u, 100u, 511u, 512u, 513u, 1024u, 5000u, 50000u})
        checkConcurrentMatchesSerial("uniform", uniformCloud(N, 1234u), 10);
}

TEST(kdtree_concurrent_build, tree_identical_to_serial_leaf_sizes)
{
    checkConcurrentMatchesSerial("uniform", uniformCloud(20000, 99u), 1);
    checkConcurrentMatchesSerial("uniform", uniformCloud(20000, 99u), 64);
}

TEST(kdtree_concurrent_build, tree_identical_to_serial_degenerate)
{
    checkConcurrentMatchesSerial("all-duplicate points", duplicateCloud(20000), 10);
    checkConcurrentMatchesSerial("collinear points", collinearCloud(20000), 10);
    checkConcurrentMatchesSerial("heavily skewed", skewedCloud(20000, 5u), 10);
    checkConcurrentMatchesSerial("heavily skewed", skewedCloud(20000, 6u), 1);
}

TEST(kdtree_concurrent_build, dynamic_index_identical_to_serial)
{
    for (const size_t N : {1000u, 20000u})
    {
        const auto cloud = uniformCloud(N, 3u);
        checkDynamicConcurrentMatchesSerial(cloud, 0);
        checkDynamicConcurrentMatchesSerial(cloud, N / 10);
    }
}
