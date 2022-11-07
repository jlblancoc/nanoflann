/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2022 Jose Luis Blanco (joseluisblancoc@gmail.com).
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

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <nanoflann.hpp>

#include "../../utils.h"  // PointCloud<>

mrpt::opengl::CPointCloud::Ptr pc_to_viz(const PointCloud<double>& pc)
{
    auto gl = mrpt::opengl::CPointCloud::Create();
    for (const auto pt : pc.pts) { gl->insertPoint(pt.x, pt.y, pt.z); }
    gl->setPointSize(2.0f);
    return gl;
}

void kdtree_demo(const size_t N)
{
    mrpt::gui::CDisplayWindow3D win("nanoflann example GUI", 800, 600);

    PointCloud<double> cloud;

    // Generate points:
    generateRandomPointCloud(cloud, N);

    double query_pt[3] = {0.5, 0.5, 0.5};

    // viz:
    {
        mrpt::opengl::COpenGLScene::Ptr   scene;
        mrpt::gui::CDisplayWindow3DLocker lck(win, scene);

        auto glPts = pc_to_viz(cloud);
        scene->insert(glPts);
        scene->insert(mrpt::opengl::stock_objects::CornerXYZSimple());
    }

    // construct a kd-tree index:
    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloud<double>>,
        PointCloud<double>, 3 /* dim */
        >;

    my_kd_tree_t index(3 /*dim*/, cloud, {10 /* max leaf */});

    {
        // Unsorted radius search:
        const double                               radius = 1;
        std::vector<std::pair<size_t, double>>     indices_dists;
        nanoflann::RadiusResultSet<double, size_t> resultSet(
            radius, indices_dists);

        index.findNeighbors(resultSet, query_pt, nanoflann::SearchParams());

        // Get worst (furthest) point, without sorting:
        std::pair<size_t, double> worst_pair = resultSet.worst_item();
        std::cout << "Worst pair: idx=" << worst_pair.first
                  << " dist=" << worst_pair.second << std::endl;
    }

    std::cout << "Press any key or close the gui to exit.\n";
    win.waitForKey();
}

int main()
{
    // Randomize Seed
    srand(static_cast<unsigned int>(time(nullptr)));
    kdtree_demo(10000);
    return 0;
}
