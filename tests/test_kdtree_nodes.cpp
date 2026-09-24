/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2026 Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *************************************************************************/

//  Tests for the contiguous node-array storage of the kd-tree: the tree is one
//  std::vector<Node> in depth-first pre-order, children addressed by relative
//  offsets (see KDTreeBaseClass::Node).

#include <cstddef>
#include <cstring>
#include <sstream>
#include <type_traits>

#include "test_helpers.h"

namespace
{
using num_t   = float;
using cloud_t = PointCloud<num_t>;
using tree_t  = KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<num_t, cloud_t>, cloud_t, 3>;

/** Points concentrated near the origin, so that middleSplit_ produces very
 *  uneven splits (and the concurrent builder takes both of its "spawn the left
 *  child" / "spawn the right child" paths). */
cloud_t skewedCloud(const size_t N, const unsigned seed)
{
    std::mt19937                          rng(seed);
    std::uniform_real_distribution<num_t> u(0, 1);
    cloud_t                               cloud;
    cloud.pts.resize(N);
    for (auto& p : cloud.pts)
    {
        const num_t r = std::pow(u(rng), num_t(4)) * 100;
        const num_t a = u(rng) * num_t(2 * nanoflann::pi_const<num_t>());
        p.x           = r * std::cos(a);
        p.y           = r * std::sin(a);
        p.z           = u(rng) * 4 - 2;
    }
    return cloud;
}

/** Checks the layout invariants of the subtree rooted at nodes_[n]: the left
 *  child is the next node, the right child is `child2` positions ahead, and
 *  leaves cover vAcc_[0, N) in order and without gaps.
 *  @return the position right after the subtree. */
size_t checkSubtree(const tree_t& idx, const size_t n, size_t& nextPoint)
{
    const auto& node = idx.nodes_.at(n);
    if (node.isLeaf())
    {
        EXPECT_EQ(node.node_type.lr.left, nextPoint);
        EXPECT_LT(node.node_type.lr.left, node.node_type.lr.right);
        EXPECT_LE(node.node_type.lr.right - node.node_type.lr.left, idx.leaf_max_size_);
        nextPoint = node.node_type.lr.right;
        return n + 1;
    }
    EXPECT_GE(node.node_type.sub.divfeat, 0);
    EXPECT_LT(node.node_type.sub.divfeat, 3);
    const size_t rightIdx  = n + node.child2;
    const size_t afterLeft = checkSubtree(idx, n + 1, nextPoint);
    EXPECT_EQ(afterLeft, rightIdx);
    return checkSubtree(idx, rightIdx, nextPoint);
}

void checkLayout(const tree_t& idx, const size_t N)
{
    ASSERT_FALSE(idx.nodes_.empty());
    size_t       nextPoint = 0;
    const size_t end       = checkSubtree(idx, 0, nextPoint);
    EXPECT_EQ(end, idx.nodes_.size());
    EXPECT_EQ(nextPoint, N);
}

/** Field-wise comparison (a memcmp would also compare the unused bytes of the
 *  union in leaf nodes). */
void expectSameNodes(const tree_t& a, const tree_t& b)
{
    ASSERT_EQ(a.nodes_.size(), b.nodes_.size());
    ASSERT_EQ(a.vAcc_, b.vAcc_);
    for (size_t i = 0; i < a.nodes_.size(); i++)
    {
        const auto& na = a.nodes_[i];
        const auto& nb = b.nodes_[i];
        ASSERT_EQ(na.child2, nb.child2) << "node " << i;
        if (na.isLeaf())
        {
            EXPECT_EQ(na.node_type.lr.left, nb.node_type.lr.left) << "node " << i;
            EXPECT_EQ(na.node_type.lr.right, nb.node_type.lr.right) << "node " << i;
        }
        else
        {
            EXPECT_EQ(na.node_type.sub.divfeat, nb.node_type.sub.divfeat) << "node " << i;
            EXPECT_EQ(na.node_type.sub.divlow, nb.node_type.sub.divlow) << "node " << i;
            EXPECT_EQ(na.node_type.sub.divhigh, nb.node_type.sub.divhigh) << "node " << i;
        }
    }
}
}  // namespace

TEST(kdtree_nodes, node_is_compact_and_trivially_copyable)
{
    // float coordinates and a 32-bit IndexType fit a node in 16 bytes.
    EXPECT_EQ(sizeof(tree_t::Node), std::max<size_t>(16, NANOFLANN_NODE_ALIGNMENT));
    EXPECT_TRUE(std::is_trivially_copyable<tree_t::Node>::value);
}

TEST(kdtree_nodes, layout_is_preorder_and_covers_all_points)
{
    for (const size_t N : {1u, 2u, 9u, 10u, 11u, 100u, 5000u, 60000u})
    {
        for (const size_t leaf : {1u, 10u, 32u})
        {
            SCOPED_TRACE("N=" + std::to_string(N) + " leaf_max_size=" + std::to_string(leaf));
            const cloud_t cloud = skewedCloud(N, 42);
            const tree_t  idx(3, cloud, KDTreeSingleIndexAdaptorParams(leaf));
            checkLayout(idx, N);
        }
    }
}

TEST(kdtree_nodes, empty_index_has_no_nodes)
{
    cloud_t      empty;
    const tree_t idx(3, empty, KDTreeSingleIndexAdaptorParams(10));
    EXPECT_TRUE(idx.nodes_.empty());
}

TEST(kdtree_nodes, rebuild_reuses_storage)
{
    const cloud_t cloud = skewedCloud(30000, 7);
#ifdef NANOFLANN_NO_THREADS
    for (const unsigned nThreads : {1u})
#else
    for (const unsigned nThreads : {1u, 4u})
#endif
    {
        SCOPED_TRACE("n_thread_build=" + std::to_string(nThreads));
        KDTreeSingleIndexAdaptorParams params(10);
        params.n_thread_build = nThreads;
        tree_t idx(3, cloud, params);

        const auto*  data = idx.nodes_.data();
        const size_t cap  = idx.nodes_.capacity();
        const size_t n    = idx.nodes_.size();
        EXPECT_GE(cap, n);
        for (int i = 0; i < 3; i++)
        {
            idx.buildIndex();  // no reallocation: same buffer, same capacity
            EXPECT_EQ(idx.nodes_.data(), data);
            EXPECT_EQ(idx.nodes_.capacity(), cap);
            EXPECT_EQ(idx.nodes_.size(), n);
        }
        EXPECT_EQ(
            idx.usedMemory(idx), cap * sizeof(tree_t::Node) + cloud.pts.size() * sizeof(uint32_t));
    }
}

TEST(kdtree_nodes, concurrent_build_gives_identical_array)
{
    const cloud_t                  cloud = skewedCloud(200000, 3);
    KDTreeSingleIndexAdaptorParams serialParams(10);
    serialParams.n_thread_build = 1;
    const tree_t serial(3, cloud, serialParams);
    checkLayout(serial, cloud.pts.size());

#ifndef NANOFLANN_NO_THREADS
    for (const unsigned nThreads : {2u, 3u, 4u, 8u, 16u, 0u /* all cores */})
    {
        SCOPED_TRACE("n_thread_build=" + std::to_string(nThreads));
        KDTreeSingleIndexAdaptorParams params(10);
        params.n_thread_build = nThreads;
        const tree_t concurrent(3, cloud, params);
        expectSameNodes(concurrent, serial);
    }
#endif
}

TEST(kdtree_nodes, saveload_roundtrip_preserves_array)
{
    const cloud_t cloud = skewedCloud(20000, 5);
    const tree_t  original(3, cloud, KDTreeSingleIndexAdaptorParams(10));

    std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);
    original.saveIndex(ss);

    tree_t loaded(
        3, cloud,
        KDTreeSingleIndexAdaptorParams(10, KDTreeSingleIndexAdaptorFlags::SkipInitialBuildIndex));
    EXPECT_TRUE(loaded.nodes_.empty());
    loaded.loadIndex(ss);
    EXPECT_FALSE(ss.fail());
    expectSameNodes(loaded, original);

    const num_t q[3] = {1.0f, -2.0f, 0.5f};
    uint32_t    i1, i2;
    num_t       d1, d2;
    EXPECT_EQ(original.knnSearch(q, 1, &i1, &d1), 1u);
    EXPECT_EQ(loaded.knnSearch(q, 1, &i2, &d2), 1u);
    EXPECT_EQ(i1, i2);
    EXPECT_EQ(d1, d2);
}

TEST(kdtree_nodes, free_index_releases_memory)
{
    const cloud_t cloud = skewedCloud(20000, 4);
    tree_t        idx(3, cloud, KDTreeSingleIndexAdaptorParams(10));
    EXPECT_GT(idx.nodes_.capacity(), 0u);
    idx.freeIndex(idx);
    EXPECT_EQ(idx.nodes_.capacity(), 0u);
}

TEST(kdtree_nodes, node_reservation_never_exceeds_max_node_count)
{
    // With one point per leaf, a tree over N points has exactly 2N-1 nodes.
    const cloud_t cloud = skewedCloud(5000, 6);
    const tree_t  idx(3, cloud, KDTreeSingleIndexAdaptorParams(1));
    EXPECT_EQ(idx.nodes_.size(), 2 * cloud.pts.size() - 1);
    EXPECT_LE(idx.nodes_.capacity(), 2 * cloud.pts.size());
}

TEST(kdtree_nodes, loadindex_rejects_corrupt_node_array)
{
    const cloud_t     cloud = skewedCloud(3000, 8);
    const tree_t      original(3, cloud, KDTreeSingleIndexAdaptorParams(10));
    std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);
    original.saveIndex(ss);
    const std::string good = ss.str();

    // The node array is the last block of the stream: point the root's right
    // child past the end of the array.
    std::string  bad  = good;
    const size_t root = bad.size() - original.nodes_.size() * sizeof(tree_t::Node);
    const auto   off  = static_cast<decltype(tree_t::Node::child2)>(original.nodes_.size());
    std::memcpy(&bad[root + offsetof(tree_t::Node, child2)], &off, sizeof(off));

    const auto load = [&](const std::string& bytes)
    {
        std::stringstream in(bytes, std::ios::in | std::ios::binary);
        tree_t            idx(
                       3, cloud,
                       KDTreeSingleIndexAdaptorParams(
                           10, KDTreeSingleIndexAdaptorFlags::SkipInitialBuildIndex));
        idx.loadIndex(in);
    };
    EXPECT_NO_THROW(load(good));
    EXPECT_THROW(load(bad), std::runtime_error);
}

// Copying a built KDTreeSingleIndexDynamicAdaptor_ (its copy constructor is
// defaulted) used to copy the PooledAllocator by value, so both copies then
// freed the same memory blocks. With the node array in a std::vector the copy
// owns its own nodes.
TEST(kdtree_nodes, dynamic_adaptor_copy_owns_its_nodes)
{
    using dyn_t = KDTreeSingleIndexDynamicAdaptor_<L2_Simple_Adaptor<num_t, cloud_t>, cloud_t, 3>;

    const cloud_t    cloud = skewedCloud(5000, 9);
    std::vector<int> treeIndex(cloud.pts.size(), 0);

    std::unique_ptr<dyn_t> copy;
    {
        dyn_t original(3, cloud, treeIndex, KDTreeSingleIndexAdaptorParams(10));
        for (size_t i = 0; i < cloud.pts.size(); i++) original.vAcc_.push_back(uint32_t(i));
        original.buildIndex();
        copy.reset(new dyn_t(original));
        EXPECT_NE(copy->nodes_.data(), original.nodes_.data());
    }  // original is gone, together with anything it owned

    const num_t                              q[3] = {0.3f, 0.1f, 0.0f};
    uint32_t                                 idx;
    num_t                                    dist;
    nanoflann::KNNResultSet<num_t, uint32_t> rs(1);
    rs.init(&idx, &dist);
    EXPECT_TRUE(copy->findNeighbors(rs, q));

    // Brute force check
    num_t best = std::numeric_limits<num_t>::max();
    for (const auto& p : cloud.pts)
    {
        const num_t d =
            (p.x - q[0]) * (p.x - q[0]) + (p.y - q[1]) * (p.y - q[1]) + (p.z - q[2]) * (p.z - q[2]);
        best = std::min(best, d);
    }
    EXPECT_EQ(dist, best);
}
