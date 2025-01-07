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

/**
 * @file nanoflann_gui_example_bearings.cpp
 * @author Jose Luis Blanco-Claraco (joseluisblancoc@gmail.com)
 *
 * @brief This file shows how to use nanoflann to search for closest samples in
 * a dataset of bearings, parameterized as two angles:
 *  - θ (theta=angle around +Z), and
 *  - φ (phi=angle around the local, already rotated by θ, +Y axis).
 * So, θ=yaw and φ=pitch in the more common robotics notation.
 *
 * This code example shows how to search for "N" closest neighbors for each
 * bearing, for each sample in the dataset.
 *
 * @date 2022-11-15
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

constexpr size_t NUM_NEIGHBORS = 40;
constexpr size_t NUM_SAMPLES   = 4000;

// See docs at the top of this file for the meaning of variables, etc.
struct Bearing
{
    double yaw   = 0;
    double pitch = 0;

    Bearing(double Yaw, double Pitch) : yaw(Yaw), pitch(Pitch) {}
};

#ifdef SHOW_GUI
mrpt::math::TPoint3D bearing_to_point(const Bearing& b)
{
    const auto R =
        mrpt::poses::CPose3D::FromYawPitchRoll(b.yaw, b.pitch, 0.0 /*roll*/)
            .getRotationMatrix();
    return {R(0, 0), R(1, 0), R(2, 0)};
}
#endif

class BearingsDataset
{
   public:
    BearingsDataset()  = default;
    ~BearingsDataset() = default;

    std::vector<Bearing> samples;

    // Must return the number of data points
    size_t kdtree_get_point_count() const { return samples.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return samples[idx].yaw;
        else
            return samples[idx].pitch;
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
mrpt::opengl::CPointCloud::Ptr pc_to_viz(const BearingsDataset& d)
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

BearingsDataset generateRandomSamples(const size_t N)
{
    BearingsDataset d;

    auto& rng = mrpt::random::getRandomGenerator();

    for (size_t i = 0; i < N; i++)
    {
        const double yaw   = rng.drawUniform(-M_PI, M_PI);
        const double pitch = rng.drawUniform(-0.3 * M_PI, 0.3 * M_PI);
        d.samples.emplace_back(yaw, pitch);
    }
    return d;
}

// based on mrpt::math::angDistance(), but without the preliminary wrapToPi()
// step for efficiency:
template <class T>
inline T my_angDistance(T from, T to)
{
    ASSERTDEB_(from >= -M_PI && from <= M_PI);
    ASSERTDEB_(to >= -M_PI && to <= M_PI);

    // wrapToPiInPlace(from);
    // wrapToPiInPlace(to);
    T d = to - from;
    if (d > M_PI)
        d -= 2. * M_PI;
    else if (d < -M_PI)
        d += 2. * M_PI;
    return d;
}

//
// ********************************* IMPORTANT ********************************
//
// This metric adaptor assumes as a precondition than "theta" is already
// normalized to [-pi,pi], not [0,2*pi].
//
// ********************************* IMPORTANT ********************************
//
template <
    class T, class DataSource, typename _DistanceType = T,
    typename IndexType = uint32_t>
struct ThetaPhiMetric_Adaptor
{
    using ElementType  = T;
    using DistanceType = _DistanceType;

    const DataSource& data_source;

    ThetaPhiMetric_Adaptor(const DataSource& _data_source)
        : data_source(_data_source)
    {
    }

    DistanceType evalMetric(
        const T* a, const IndexType b_idx, size_t /*size = 2*/) const
    {
        DistanceType result =
            mrpt::square(
                my_angDistance(a[0], data_source.kdtree_get_pt(b_idx, 0))) +
            mrpt::square(a[1] - data_source.kdtree_get_pt(b_idx, 1));

        return result;
    }

    template <typename U, typename V>
    DistanceType accum_dist(const U a, const V b, const size_t dimIdx) const
    {
        if (dimIdx == 0)
        {  // theta:
            return mrpt::square(my_angDistance(a, b));
        }
        else
        {
            // phi:
            return mrpt::square(a - b);
        }
    }
};

using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
    ThetaPhiMetric_Adaptor<double, BearingsDataset>, BearingsDataset,
    2 /* dim */
    >;

void kdtree_demo(const size_t N)
{
#ifdef SHOW_GUI
    mrpt::gui::CDisplayWindow3D win("nanoflann example GUI", 800, 600);
#endif

    // Generate points:
    const BearingsDataset data = generateRandomSamples(N);

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
        const double queryPt[2] = {
            data.kdtree_get_pt(i, 0), data.kdtree_get_pt(i, 1)};

        mrpt::system::CTimeLoggerEntry tle2(profiler, "query");

        const auto numNN = index.knnSearch(
            queryPt, NUM_NEIGHBORS, nn_indices.data(), nn_distances.data());

        tle2.stop();

        std::cout << "\nQuery point: (" << queryPt[0] << "," << queryPt[1]
                  << ") => " << numNN << " results.\n";

#ifdef SHOW_GUI
        bool stop = false;
        // Color results:
        {
            win.get3DSceneAndLock();

            glQueryPt->clear();
            {
                const auto pt = bearing_to_point({queryPt[0], queryPt[1]});
                glQueryPt->insertPoint(pt.x, pt.y, pt.z);
                stop = pt.x < -0.95;
            }

            glFoundPts->clear();
            for (size_t j = 0; j < numNN; j++)
            {
                const auto pt =
                    bearing_to_point(data.samples.at(nn_indices[j]));
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
    std::cout << my_angDistance(mrpt::DEG2RAD(179.0), mrpt::DEG2RAD(-179.0))
              << std::endl;
    std::cout << my_angDistance(mrpt::DEG2RAD(-179.0), mrpt::DEG2RAD(179.0))
              << std::endl;

    kdtree_demo(NUM_SAMPLES);
    return 0;
}
