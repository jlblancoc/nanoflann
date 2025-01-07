/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2025 Jose Luis Blanco (joseluisblancoc@gmail.com).
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

#include <Eigen/Dense>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <nanoflann.hpp>

constexpr int SAMPLES_DIM = 15;

template <typename Der>
void generateRandomPointCloud(
    Eigen::MatrixBase<Der>& mat, const size_t N, const size_t dim,
    const typename Der::Scalar max_range = 10)
{
    std::cout << "Generating " << N << " random points...";
    mat.resize(N, dim);
    for (size_t i = 0; i < N; i++)
        for (size_t d = 0; d < dim; d++)
            mat(i, d) =
                max_range * (rand() % 1000) / typename Der::Scalar(1000);
    std::cout << "done\n";
}

template <typename num_t>
void kdtree_demo(const size_t nSamples, const size_t dim)
{
    using matrix_t = Eigen::Matrix<num_t, Eigen::Dynamic, Eigen::Dynamic>;

    matrix_t mat(nSamples, dim);

    const num_t max_range = 20;

    // Generate points:
    generateRandomPointCloud(mat, nSamples, dim, max_range);

    //	cout << mat << endl;

    // Query point:
    std::vector<num_t> query_pt(dim);
    for (size_t d = 0; d < dim; d++)
        query_pt[d] = max_range * (rand() % 1000) / num_t(1000);

        // ------------------------------------------------------------
        // construct a kd-tree index:
        //    Some of the different possibilities (uncomment just one)
        // ------------------------------------------------------------
        // Dimensionality set at run-time (default: L2)
#if 1
    using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<matrix_t>;
#elif 0
    // Dimensionality set at compile-time: Explicit selection of the distance
    // metric: L2
    using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<
        matrix_t, SAMPLES_DIM /*fixed size*/, nanoflann::metric_L2>;
#elif 0
    // Dimensionality set at compile-time: Explicit selection of the distance
    // metric: L2_simple
    using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<
        matrix_t, SAMPLES_DIM /*fixed size*/, nanoflann::metric_L2_Simple>;
#elif 0
    // Dimensionality set at compile-time: Explicit selection of the distance
    // metric: L1
    using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<
        matrix_t, SAMPLES_DIM /*fixed size*/, nanoflann::metric_L1>;
#elif 0
    // Dimensionality set at compile-time: Explicit selection of the distance
    // metric: L2 Row Major matrix layout
    // Eigen::Matrix<num_t, Dynamic, Dynamic> mat(dim, nSamples);
    using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<
        matrix_t, SAMPLES_DIM /*fixed size*/, nanoflann::metric_L2, true>;
#endif

    my_kd_tree_t mat_index(dim, std::cref(mat), 10 /* max leaf */);

    // do a knn search
    const size_t        num_results = 3;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<num_t>  out_dists_sqr(num_results);

    nanoflann::KNNResultSet<num_t> resultSet(num_results);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
    mat_index.index_->findNeighbors(resultSet, &query_pt[0]);

    std::cout << "knnSearch(nn=" << num_results << "): \n";
    for (size_t i = 0; i < resultSet.size(); i++)
        std::cout << "ret_index[" << i << "]=" << ret_indexes[i]
                  << " out_dist_sqr=" << out_dists_sqr[i] << std::endl;
}

int main(int /*argc*/, char** /*argv*/)
{
    // Randomize Seed
    // srand(static_cast<unsigned int>(time(nullptr)));
    kdtree_demo<float>(1000 /* samples */, SAMPLES_DIM /* dim */);

    return 0;
}
