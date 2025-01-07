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

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <nanoflann.hpp>
#include <type_traits>

#include "utils.h"

using num_t = double;

template <typename _DistanceType, typename _IndexType = size_t>
class MyCustomResultSet
{
   public:
    using DistanceType = _DistanceType;
    using IndexType    = _IndexType;

   public:
    const DistanceType radius;

    std::vector<nanoflann::ResultItem<IndexType, DistanceType>>&
        m_indices_dists;

    explicit MyCustomResultSet(
        DistanceType radius_,
        std::vector<nanoflann::ResultItem<IndexType, DistanceType>>&
            indices_dists)
        : radius(radius_), m_indices_dists(indices_dists)
    {
        init();
    }

    void init() { clear(); }
    void clear() { m_indices_dists.clear(); }

    size_t size() const { return m_indices_dists.size(); }
    size_t empty() const { return m_indices_dists.empty(); }

    bool full() const { return true; }

    /**
     * Called during search to add an element matching the criteria.
     * @return true if the search should be continued, false if the results are
     * sufficient
     */
    bool addPoint(DistanceType dist, IndexType index)
    {
        printf(
            "addPoint() called: dist=%f index=%u\n", dist,
            static_cast<unsigned int>(index));

        if (dist < radius) m_indices_dists.emplace_back(index, dist);
        return true;
    }

    DistanceType worstDist() const { return radius; }

    void sort()
    {
        std::sort(
            m_indices_dists.begin(), m_indices_dists.end(),
            nanoflann::IndexDist_Sorter());
    }
};

void kdtree_demo(const size_t N)
{
    PointCloud<num_t> cloud;

    // Generate points:
    generateRandomPointCloud(cloud, N);

    num_t query_pt[3] = {0.5, 0.5, 0.5};

    // construct a kd-tree index:
    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t>>,
        PointCloud<num_t>, 3 /* dim */
        >;

    my_kd_tree_t index(3 /*dim*/, cloud, {10 /* max leaf */});

    {
        // radius search:
        const num_t                                       squaredRadius = 1;
        std::vector<nanoflann::ResultItem<size_t, num_t>> indices_dists;

        MyCustomResultSet<num_t, size_t> resultSet(
            squaredRadius, indices_dists);

        index.findNeighbors(resultSet, query_pt);

        std::cout << "Found: " << indices_dists.size() << " NN points."
                  << std::endl;
    }
}

int main()
{
    // Randomize Seed
    srand(static_cast<unsigned int>(time(nullptr)));
    kdtree_demo(10000);
    return 0;
}
