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
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/CTimeLogger.h>

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <nanoflann.hpp>

#include "../../utils.h"  // PointCloud<>

mrpt::opengl::CPointCloud::Ptr pc_to_viz(const PointCloud<double>& pc)
{
    auto gl = mrpt::opengl::CPointCloud::Create();
    for (const auto pt : pc.pts) { gl->insertPoint(pt.x, pt.y, pt.z); }
    gl->setPointSize(3.0f);
    return gl;
}

void kdtree_demo(const size_t N)
{
    mrpt::gui::CDisplayWindow3D win("nanoflann example GUI", 800, 600);

    PointCloud<double> cloud;
    const double       maxRangeXY = 10.0, maxRangeZ = 1.0;

    // Generate points:
    generateRandomPointCloudRanges(cloud, N, maxRangeXY, maxRangeXY, maxRangeZ);

    // viz:
    auto glQueryPt     = mrpt::opengl::CPointCloud::Create();
    auto glFoundPts    = mrpt::opengl::CPointCloud::Create();
    auto glQuerySphere = mrpt::opengl::CSphere::Create();
    {
        mrpt::opengl::COpenGLScene::Ptr   scene;
        mrpt::gui::CDisplayWindow3DLocker lck(win, scene);

        auto glPts = pc_to_viz(cloud);
        scene->insert(glPts);
        scene->insert(mrpt::opengl::stock_objects::CornerXYZSimple());

        glQueryPt->setPointSize(10.0f);
        glQueryPt->setColor_u8(0xff, 0x00, 0x00);
        scene->insert(glQueryPt);

        glFoundPts->setPointSize(7.0f);
        glFoundPts->setColor_u8(0x00, 0x00, 0xff, 0x80);
        scene->insert(glFoundPts);

        glQuerySphere->setColor_u8(0xe0, 0xe0, 0xe0, 0x30);
        scene->insert(glQuerySphere);
    }

    // construct a kd-tree index:
    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloud<double>>,
        PointCloud<double>, 3 /* dim */
        >;

    mrpt::system::CTimeLogger profiler;

    mrpt::system::CTimeLoggerEntry tle(profiler, "build_kd_tree");

    my_kd_tree_t index(3 /*dim*/, cloud, {10 /* max leaf */});

    tle.stop();

    auto& rng = mrpt::random::getRandomGenerator();

    // Declare here to avoid reallocations:
    std::vector<nanoflann::ResultItem<size_t, double>> indicesDists;

    // Loop: different searches until the window is closed:
    while (win.isOpen())
    {
        // Unsorted radius search:
        const double radius   = rng.drawUniform(0.1, maxRangeXY * 0.5);
        const double sqRadius = radius * radius;

        const double queryPt[3] = {
            rng.drawUniform(-0.3, maxRangeXY + 0.3),
            rng.drawUniform(-0.3, maxRangeXY + 0.3),
            rng.drawUniform(-0.3, maxRangeZ + 0.3)};

        mrpt::system::CTimeLoggerEntry tle2(profiler, "query");

        indicesDists.clear();
        nanoflann::RadiusResultSet<double, size_t> resultSet(
            sqRadius, indicesDists);

        index.findNeighbors(resultSet, queryPt);

        tle2.stop();

        std::cout << "\nQuery point: (" << queryPt[0] << "," << queryPt[1]
                  << "," << queryPt[2] << ") => " << resultSet.size()
                  << " results.\n";
        if (!resultSet.empty())
        {
            nanoflann::ResultItem<size_t, double> worstPair =
                resultSet.worst_item();
            std::cout << "Worst pair: idx=" << worstPair.first
                      << " dist=" << std::sqrt(worstPair.second) << std::endl;
        }

        // Color results:
        {
            win.get3DSceneAndLock();

            glQueryPt->clear();
            glQueryPt->insertPoint(queryPt[0], queryPt[1], queryPt[2]);

            glQuerySphere->setLocation(queryPt[0], queryPt[1], queryPt[2]);
            glQuerySphere->setRadius(radius);

            glFoundPts->clear();
            for (const auto& ptIdx : indicesDists)
            {
                const auto& pt = cloud.pts.at(ptIdx.first);
                glFoundPts->insertPoint(pt.x, pt.y, pt.z);
            }

            win.unlockAccess3DScene();
            win.repaint();
        }

        std::cout << "Press any key to pick another random query point. Close "
                     "the GUI window to exit."
                  << std::endl;

        win.waitForKey();
    }
}

int main()
{
    kdtree_demo(1000);
    return 0;
}
