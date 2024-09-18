/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2024 Jose Luis Blanco (joseluisblancoc@gmail.com).
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

        inline T get_component(size_t dim) const
        {
            // Since this is inlined and the "dim" argument is typically an
            // immediate value, the
            //  "if/else's" are actually solved at compile time.
            if (dim == 0)
                return x;
            else if (dim == 1)
                return y;
            else
                return z;
        }

        inline T get_signed_distance(size_t dim, T val) const
        {
            return get_component(dim) - val;
        }

        inline T get_distance_to(const Point& other) const
        {
            const auto dx = x - other.x;
            const auto dy = y - other.y;
            const auto dz = z - other.z;
            return dx * dx + dy * dy + dz * dz;
        }

		// Distance of this point from box.
		inline T get_distance_to(const Point& minPoint, const Point& maxPoint) const
		{
			const auto minmaxx = std::minmax(minPoint.x, maxPoint.x);
			const auto minmaxy = std::minmax(minPoint.y, maxPoint.y);
			const auto minmaxz = std::minmax(minPoint.z, maxPoint.z);
			const auto dx = x < minmaxx.first ? minmaxx.first - x : (minmaxx.second < x ? x - minmaxx.second : T(0));
			const auto dy = y < minmaxy.first ? minmaxy.first - y : (minmaxy.second < y ? y - minmaxy.second : T(0));
			const auto dz = z < minmaxz.first ? minmaxz.first - z : (minmaxz.second < z ? z - minmaxz.second : T(0));
			return dx * dx + dy * dy + dz * dz;
		}
    };

    using PointType = Point;
    using coord_t = T;  //!< The type of each coordinate

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline const PointType& kdtree_get_pt(const size_t idx) const
    {
        return pts[idx];
    }

    // Get limits for list of points
    inline void kdtree_get_limits(
        const uint32_t* ix, size_t count, const size_t dim, T& limit_min, T& limit_max) const
    {
        limit_min = limit_max = kdtree_get_pt(ix[0]).get_component(dim);
        for (size_t k = 1; k < count; ++k)
        {
            const T value = kdtree_get_pt(ix[k]).get_component(dim);
            if (value < limit_min) limit_min = value;
            if (value > limit_max) limit_max = value;
        }
    }

	// Intersection between node's bounding box and line segment (required only if nanoflann::KDTreeSingleIndexAdaptor<>::lineSegSearch is used)
	template<class BBOX>
	bool kdtree_intersects(const PointType& minPoint, const PointType& maxPoint, const BBOX& bbox, T max_dist, nanoflann::Int2Type<3>) const
	{
		const auto minmaxx = std::minmax(minPoint.x, maxPoint.x);
		if (minmaxx.second + max_dist < bbox[0].low || bbox[0].high + max_dist < minmaxx.first)
			return false;

		const auto minmaxy = std::minmax(minPoint.y, maxPoint.y);
		if (minmaxy.second + max_dist < bbox[1].low && bbox[1].high + max_dist < minmaxy.first)
			return false;

		const auto minmaxz = std::minmax(minPoint.z, maxPoint.z);
		if (minmaxz.second + max_dist < bbox[2].low && bbox[2].high + max_dist < minmaxz.first)
			return false;
		
		return true;
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

template <typename T>
void generateGridPointCloud(
    PointCloud<T>& point, const size_t X, const size_t Y, const size_t Z,
    const T cell_size = 1)
{
    const auto offset = point.pts.size();
    point.pts.resize(offset + X * Y * Z);
    for (size_t z = 0; z < Z; ++z)
    {
        for (size_t y = 0; y < Y; ++y)
        {
            for (size_t x = 0; x < X; ++x)
            {
                const size_t ix = offset + x + y * X + z * X * Y;
                point.pts[ix].x = x * cell_size;
                point.pts[ix].y = y * cell_size;
                point.pts[ix].z = z * cell_size;
            }
        }
    }
}

// This is an exampleof a custom data set class
template <typename T>
struct PointCloud_Quat
{
    struct Point
    {
        T w, x, y, z;

        inline T get_component(size_t dim) const
        {
            // Since this is inlined and the "dim" argument is typically an
            // immediate value, the
            //  "if/else's" are actually solved at compile time.
            if (dim == 0)
                return w;
            else if (dim == 1)
                return x;
            else if (dim == 2)
                return y;
            else
                return z;
        }

        inline T get_signed_distance(size_t dim, T val) const
        {
            return get_component(dim) - val;
        }

        inline T get_distance_to(const Point& pt) const
        {
            const auto dw = w - pt.w;
            const auto dx = x - pt.x;
            const auto dy = y - pt.y;
            const auto dz = z - pt.z;
            return dw * dw + dx * dx + dy * dy + dz * dz;
        }
    };

    using PointType = Point;

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline const PointType& kdtree_get_pt(const size_t idx) const
    {
        return pts[idx];
    }

    // Get limits for list of points
    inline void kdtree_get_limits(
        const uint32_t* ix, size_t count, const size_t dim, T& limit_min,
        T& limit_max) const
    {
        limit_min = limit_max = kdtree_get_pt(ix[0]).get_component(dim);
        for (size_t k = 1; k < count; ++k)
        {
            const T value = kdtree_get_pt(ix[k]).get_component(dim);
            if (value < limit_min) limit_min = value;
            if (value > limit_max) limit_max = value;
        }
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

        inline T get_component(size_t) const
        {
            return theta;
        }

        inline T get_signed_distance(size_t, T val) const
        {
            return theta - val;
        }

        inline T get_distance_to(const Point& other) const
        {
            return (theta - other.theta) * (theta - other.theta);
        }
    };

    using PointType = Point;

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline const PointType& kdtree_get_pt(const size_t idx) const
    {
        return pts[idx];
    }

    // Get limits for list of points
    inline void kdtree_get_limits(
        const uint32_t* ix, size_t count, const size_t dim, T& limit_min,
        T& limit_max) const
    {
        limit_min = limit_max = kdtree_get_pt(ix[0]).get_component(dim);
        for (size_t k = 1; k < count; ++k)
        {
            const T value = kdtree_get_pt(ix[k]).get_component(dim);
            if (value < limit_min) limit_min = value;
            if (value > limit_max) limit_max = value;
        }
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
