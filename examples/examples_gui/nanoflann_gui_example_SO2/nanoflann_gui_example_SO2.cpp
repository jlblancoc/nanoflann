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

/**
 * @file nanoflann_gui_example_SO2.cpp
 * @author Jose Luis Blanco-Claraco (joseluisblancoc@gmail.com)
 *
 * @brief This file shows how to use nanoflann to search for closest samples in
 * a dataset of SO(2) orientations, parameterized as one angle theta ∈ [-π,π].
 *
 * @date 2023-01-11
 */

#include <mrpt/core/bits_math.h>  // square()
#include <mrpt/math/TPoint3D.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/CTimeLogger.h>

#include <array>
#include <cstdlib>
#include <iostream>
#include <nanoflann.hpp>

// uncomment to show the GUI. Comment it out to profile full speed.
#define SHOW_GUI

#ifdef SHOW_GUI
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>
#endif

constexpr size_t NUM_NEIGHBORS = 10;
constexpr size_t NUM_SAMPLES   = 400;

// See docs at the top of this file for the meaning of variables, etc.
struct Heading
{
    double yaw = 0;

    Heading(double Yaw) : yaw(Yaw) {}
};

#ifdef SHOW_GUI
mrpt::math::TPoint3D bearing_to_point(const Heading& b)
{
    const auto R = mrpt::poses::CPose3D::FromYawPitchRoll(
                       b.yaw, 0.0 /*pitch*/, 0.0 /*roll*/)
                       .getRotationMatrix();
    return {R(0, 0), R(1, 0), R(2, 0)};
}
#endif

class HeadingsDataset
{
   public:
    HeadingsDataset()  = default;
    ~HeadingsDataset() = default;

    std::vector<Heading> samples;

    // Must return the number of data points
    size_t kdtree_get_point_count() const { return samples.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        ASSERT_(dim == 0);
        return samples[idx].yaw;
    }

    // Optional bounding-box computation: return false to default to a standard
    // bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned
    //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
    //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const
    {
        return false;
    }
};

#ifdef SHOW_GUI
mrpt::opengl::CPointCloud::Ptr pc_to_viz(const HeadingsDataset& d)
{
    auto gl = mrpt::opengl::CPointCloud::Create();
    for (const auto sample : d.samples)
    {
        const auto pt = bearing_to_point(sample);
        gl->insertPoint(pt.x, pt.y, pt.z);
    }
    gl->setPointSize(3.0f);
    return gl;
}
#endif

HeadingsDataset generateRandomSamples(const size_t N)
{
    HeadingsDataset d;

    auto& rng = mrpt::random::getRandomGenerator();

    for (size_t i = 0; i < N; i++)
    {
        const double yaw = rng.drawUniform(-M_PI, M_PI);
        d.samples.emplace_back(yaw);
    }
    return d;
}

using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::SO2_Adaptor<double, HeadingsDataset>, HeadingsDataset,
    1 /* dim */
    >;

void kdtree_demo(const size_t N)
{
#ifdef SHOW_GUI
    mrpt::gui::CDisplayWindow3D win("nanoflann example GUI", 800, 600);
#endif

    // Generate points:
    const HeadingsDataset data = generateRandomSamples(N);

#ifdef SHOW_GUI
    // viz:
    auto glQueryPt  = mrpt::opengl::CPointCloud::Create();
    auto glFoundPts = mrpt::opengl::CPointCloud::Create();
    {
        mrpt::opengl::COpenGLScene::Ptr   scene;
        mrpt::gui::CDisplayWindow3DLocker lck(win, scene);

        auto glPts = pc_to_viz(data);
        scene->insert(glPts);
        scene->insert(mrpt::opengl::stock_objects::CornerXYZSimple());
        auto glAxis =
            mrpt::opengl::CAxis::Create(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 0.25);
        glAxis->setTextScale(0.1);
        scene->insert(glAxis);

        glQueryPt->setPointSize(10.0f);
        glQueryPt->setColor_u8(0xff, 0x00, 0x00);
        scene->insert(glQueryPt);

        glFoundPts->setPointSize(7.0f);
        glFoundPts->setColor_u8(0x00, 0x00, 0xff, 0x80);
        scene->insert(glFoundPts);
    }
#endif
    // construct a kd-tree index:

    mrpt::system::CTimeLogger profiler;

    mrpt::system::CTimeLoggerEntry tle(profiler, "build_kd_tree");

    my_kd_tree_t index(2 /*dim*/, data, {20 /* max leaf */});

    tle.stop();

    // Declare here to avoid reallocations:
    std::array<uint32_t, NUM_NEIGHBORS> nn_indices;
    std::array<double, NUM_NEIGHBORS>   nn_distances;

    // Loop: different searches until the window is closed:
#ifdef SHOW_GUI
    for (size_t i = 0; i < N && win.isOpen(); i++)
#else
    for (size_t i = 0; i < N; i++)
#endif
    {
        const double queryPt[1] = {data.kdtree_get_pt(i, 0)};

        mrpt::system::CTimeLoggerEntry tle2(profiler, "query");

        const auto numNN = index.knnSearch(
            queryPt, NUM_NEIGHBORS, nn_indices.data(), nn_distances.data());

        tle2.stop();

        std::cout << "\nQuery point: (" << queryPt[0] << ") => " << numNN
                  << " results.\n";

#ifdef SHOW_GUI
        bool stop = false;
        // Color results:
        {
            win.get3DSceneAndLock();

            glQueryPt->clear();
            {
                const auto pt = bearing_to_point({queryPt[0]});
                glQueryPt->insertPoint(pt.x, pt.y, pt.z);
                stop = pt.x < -0.95;
            }

            glFoundPts->clear();
            for (size_t i = 0; i < numNN; i++)
            {
                const auto pt =
                    bearing_to_point(data.samples.at(nn_indices[i]));
                glFoundPts->insertPoint(pt.x, pt.y, pt.z);
            }

            win.unlockAccess3DScene();
            win.repaint();
        }
        std::cout << "Press any key to pick another random query point. Close "
                     "the GUI window to exit."
                  << std::endl;

        if (stop) win.waitForKey();
#endif
    }
}

int main()
{
    kdtree_demo(NUM_SAMPLES);
    return 0;
}
