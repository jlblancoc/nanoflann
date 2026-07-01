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

// Tests for nanoflann::KDTreeEigenMatrixAdaptor. Compiled only when Eigen is
// available (the build system defines NANOFLANN_HAS_EIGEN and adds the Eigen
// include path); otherwise this file is an empty translation unit so the test
// suite still builds without a hard Eigen dependency.

#include <gtest/gtest.h>

#include <nanoflann.hpp>

#if defined(NANOFLANN_HAS_EIGEN)

#include <Eigen/Dense>
#include <algorithm>
#include <cstddef>
#include <map>
#include <random>
#include <vector>

#include "../examples/KDTreeVectorOfVectorsAdaptor.h"

namespace
{
// Build an Eigen matrix + KDTreeEigenMatrixAdaptor for the given layout and
// compile-time DIM, then compare a knnSearch against a brute-force ranking.
template <bool RowMajor, int DIM_TPL>
void eigen_adaptor_knn_vs_bruteforce(const size_t N, const size_t dim, const size_t k)
{
    using num_t    = double;
    using matrix_t = Eigen::Matrix<num_t, Eigen::Dynamic, Eigen::Dynamic>;

    matrix_t mat;
    if (RowMajor)
        mat.resize(static_cast<Eigen::Index>(N), static_cast<Eigen::Index>(dim));
    else
        mat.resize(static_cast<Eigen::Index>(dim), static_cast<Eigen::Index>(N));

    std::mt19937                          rng(1234u);
    std::uniform_real_distribution<num_t> uni(num_t(-10), num_t(10));

    // Keep a plain copy for the brute-force reference:
    std::vector<std::vector<num_t>> pts(N, std::vector<num_t>(dim));
    for (size_t i = 0; i < N; i++)
        for (size_t d = 0; d < dim; d++)
        {
            const num_t v = uni(rng);
            pts[i][d]     = v;
            if (RowMajor)
                mat(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(d)) = v;
            else
                mat(static_cast<Eigen::Index>(d), static_cast<Eigen::Index>(i)) = v;
        }

    using adaptor_t =
        nanoflann::KDTreeEigenMatrixAdaptor<matrix_t, DIM_TPL, nanoflann::metric_L2, RowMajor>;
    using IndexType = typename adaptor_t::IndexType;

    adaptor_t index(
        static_cast<typename adaptor_t::Dimension>(dim), std::cref(mat), 10 /* max leaf */);

    // Query point:
    std::vector<num_t> query(dim);
    for (size_t d = 0; d < dim; d++) query[d] = uni(rng);

    std::vector<IndexType> ret_idx(k);
    std::vector<num_t>     ret_dist(k);
    // Exercise the owned index through the unique_ptr member:
    const size_t nFound = index.index_->knnSearch(query.data(), k, ret_idx.data(), ret_dist.data());
    EXPECT_EQ(static_cast<size_t>(nFound), std::min(k, N));

    // Brute-force reference (squared L2):
    std::multimap<num_t, size_t> bf;
    for (size_t i = 0; i < N; i++)
    {
        num_t d2 = 0;
        for (size_t d = 0; d < dim; d++)
        {
            const num_t diff = query[d] - pts[i][d];
            d2 += diff * diff;
        }
        bf.emplace(d2, i);
    }
    // Index -> distance, to tolerate ties in the ordering:
    std::map<size_t, num_t> idx2dist;
    for (const auto& kv : bf) idx2dist[kv.second] = kv.first;

    auto it = bf.begin();
    for (size_t i = 0; i < nFound; ++i, ++it)
    {
        EXPECT_NEAR(it->first, ret_dist[i], 1e-6);
        EXPECT_NEAR(idx2dist.at(static_cast<size_t>(ret_idx[i])), ret_dist[i], 1e-6);
    }
}
}  // namespace

TEST(eigen_adaptor, rowmajor_dynamic_dim) { eigen_adaptor_knn_vs_bruteforce<true, -1>(500, 4, 5); }
TEST(eigen_adaptor, colmajor_dynamic_dim) { eigen_adaptor_knn_vs_bruteforce<false, -1>(500, 4, 5); }
TEST(eigen_adaptor, rowmajor_fixed_dim3) { eigen_adaptor_knn_vs_bruteforce<true, 3>(500, 3, 5); }
TEST(eigen_adaptor, colmajor_fixed_dim3) { eigen_adaptor_knn_vs_bruteforce<false, 3>(500, 3, 5); }

// The query() convenience wrapper must agree with a direct findNeighbors call.
TEST(eigen_adaptor, query_helper_matches_index)
{
    using num_t    = double;
    using matrix_t = Eigen::Matrix<num_t, Eigen::Dynamic, Eigen::Dynamic>;

    const size_t N = 300, dim = 3, k = 4;
    matrix_t     mat(static_cast<Eigen::Index>(N), static_cast<Eigen::Index>(dim));

    std::mt19937                          rng(7u);
    std::uniform_real_distribution<num_t> uni(num_t(0), num_t(1));
    for (Eigen::Index i = 0; i < mat.rows(); i++)
        for (Eigen::Index d = 0; d < mat.cols(); d++) mat(i, d) = uni(rng);

    using adaptor_t = nanoflann::KDTreeEigenMatrixAdaptor<matrix_t>;
    using IndexType = typename adaptor_t::IndexType;
    adaptor_t index(static_cast<typename adaptor_t::Dimension>(dim), std::cref(mat), 10);

    std::vector<num_t> query = {0.3, 0.6, 0.1};

    std::vector<IndexType> idx_a(k), idx_b(k);
    std::vector<num_t>     dist_a(k), dist_b(k);

    index.query(query.data(), k, idx_a.data(), dist_a.data());

    nanoflann::KNNResultSet<num_t, IndexType> rs(k);
    rs.init(idx_b.data(), dist_b.data());
    index.index_->findNeighbors(rs, query.data());

    for (size_t i = 0; i < k; i++)
    {
        EXPECT_EQ(idx_a[i], idx_b[i]);
        EXPECT_NEAR(dist_a[i], dist_b[i], 1e-9);
    }
}

// Constructor validates the run-time and compile-time dimensionality.
TEST(eigen_adaptor, dimensionality_mismatch_throws)
{
    using num_t    = double;
    using matrix_t = Eigen::Matrix<num_t, Eigen::Dynamic, Eigen::Dynamic>;
    matrix_t mat(10, 3);  // 10 points of dim 3 (row-major)
    mat.setZero();

    using dyn_adaptor_t = nanoflann::KDTreeEigenMatrixAdaptor<matrix_t>;
    // Wrong run-time dimensionality argument:
    EXPECT_THROW({ dyn_adaptor_t bad(4 /* != 3 */, std::cref(mat), 10); }, std::runtime_error);

    using fixed_adaptor_t = nanoflann::KDTreeEigenMatrixAdaptor<matrix_t, 4 /* fixed DIM */>;
    // Matrix has 3 columns but the compile-time DIM is 4:
    EXPECT_THROW({ fixed_adaptor_t bad(3, std::cref(mat), 10); }, std::runtime_error);
}

// Documented-but-previously-untested container variant:
// KDTreeVectorOfVectorsAdaptor over std::vector<Eigen::VectorXd>.
TEST(eigen_adaptor, vector_of_eigen_vectors)
{
    using num_t = double;

    auto mk = [](num_t a, num_t b, num_t c)
    {
        Eigen::VectorXd v(3);
        v << a, b, c;
        return v;
    };
    std::vector<Eigen::VectorXd> pts = {mk(0, 0, 0), mk(1, 0, 0), mk(0, 5, 0), mk(9, 9, 9)};

    using kdtree_t = KDTreeVectorOfVectorsAdaptor<std::vector<Eigen::VectorXd>, num_t, 3>;
    kdtree_t index(3 /*dim*/, pts, 10 /* max leaf */);

    const num_t                    q[3] = {0.9, 0.1, 0.0};
    size_t                         idx  = 0;
    num_t                          d2   = 0;
    nanoflann::KNNResultSet<num_t> rs(1);
    rs.init(&idx, &d2);
    index.index->findNeighbors(rs, q);

    EXPECT_EQ(idx, 1u);  // (1,0,0) is nearest to (0.9,0.1,0)
    EXPECT_NEAR(d2, 0.01 + 0.01, 1e-12);
}

#endif  // NANOFLANN_HAS_EIGEN
