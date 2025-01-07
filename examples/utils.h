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
#include <iostream>
#include <vector>

template <typename T>
struct PointCloud
{
    struct Point
    {
        T x, y, z;
    };

    using coord_t = T;  //!< The type of each coordinate

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return pts[idx].x;
        else if (dim == 1)
            return pts[idx].y;
        else
            return pts[idx].z;
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

template <typename T>
void generateRandomPointCloudRanges(
    PointCloud<T>& pc, const size_t N, const T max_range_x, const T max_range_y,
    const T max_range_z)
{
    // Generating Random Point Cloud
    pc.pts.resize(N);
    for (size_t i = 0; i < N; i++)
    {
        pc.pts[i].x = max_range_x * (rand() % 1000) / T(1000);
        pc.pts[i].y = max_range_y * (rand() % 1000) / T(1000);
        pc.pts[i].z = max_range_z * (rand() % 1000) / T(1000);
    }
}

template <typename T>
void generateRandomPointCloud(
    PointCloud<T>& pc, const size_t N, const T max_range = 10)
{
    generateRandomPointCloudRanges(pc, N, max_range, max_range, max_range);
}

// This is an exampleof a custom data set class
template <typename T>
struct PointCloud_Quat
{
    struct Point
    {
        T w, x, y, z;
    };

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return pts[idx].w;
        else if (dim == 1)
            return pts[idx].x;
        else if (dim == 2)
            return pts[idx].y;
        else
            return pts[idx].z;
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

template <typename T>
void generateRandomPointCloud_Quat(PointCloud_Quat<T>& point, const size_t N)
{
    // Generating Random Quaternions
    point.pts.resize(N);
    T theta, X, Y, Z, sinAng, cosAng, mag;
    for (size_t i = 0; i < N; i++)
    {
        theta = static_cast<T>(
            nanoflann::pi_const<double>() * (((double)rand()) / RAND_MAX));
        // Generate random value in [-1, 1]
        X   = static_cast<T>(2 * (((double)rand()) / RAND_MAX) - 1);
        Y   = static_cast<T>(2 * (((double)rand()) / RAND_MAX) - 1);
        Z   = static_cast<T>(2 * (((double)rand()) / RAND_MAX) - 1);
        mag = sqrt(X * X + Y * Y + Z * Z);
        X /= mag;
        Y /= mag;
        Z /= mag;
        cosAng         = cos(theta / 2);
        sinAng         = sin(theta / 2);
        point.pts[i].w = cosAng;
        point.pts[i].x = X * sinAng;
        point.pts[i].y = Y * sinAng;
        point.pts[i].z = Z * sinAng;
    }
}

// This is an exampleof a custom data set class
template <typename T>
struct PointCloud_Orient
{
    struct Point
    {
        T theta;
    };

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, const size_t dim = 0) const
    {
        (void)dim;
        return pts[idx].theta;
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

template <typename T>
void generateRandomPointCloud_Orient(
    PointCloud_Orient<T>& point, const size_t N)
{
    // Generating Random Orientations
    point.pts.resize(N);
    for (size_t i = 0; i < N; i++)
    {
        point.pts[i].theta = static_cast<T>(
            (2 * nanoflann::pi_const<double>() *
             (((double)rand()) / RAND_MAX)) -
            nanoflann::pi_const<double>());
    }
}

inline void dump_mem_usage()
{
    FILE* f = fopen("/proc/self/statm", "rt");
    if (!f) return;
    char   str[300];
    size_t n = fread(str, 1, 200, f);
    str[n]   = 0;
    printf("MEM: %s\n", str);
    fclose(f);
}
