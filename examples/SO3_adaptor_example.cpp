/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2016 Jose Luis Blanco (joseluisblancoc@gmail.com).
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

#include <nanoflann.hpp>

#include <ctime>
#include <cstdlib>
#include <iostream>

using namespace std;
using namespace nanoflann;

void dump_mem_usage();

// This is an exampleof a custom data set class
template <typename T>
struct PointCloud
{
	struct Point
	{
		T  w,x,y,z;
	};

	std::vector<Point>  pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }\

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline T kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim==0) return pts[idx].w;
		else if (dim==1) return pts[idx].x;
		else if (dim==2) return pts[idx].y;
		else return pts[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

};

template <typename T>
void generateRandomPointCloud(PointCloud<T> &point, const size_t N)
{
	std::cout << "Generating "<< N << " quaternions...";
	point.pts.resize(N);
	T theta, X, Y, Z, sinAng, cosAng, mag;
	for (size_t i=0;i<N;i++)
	{
		theta = 2 * M_PI * (((double)rand()) / RAND_MAX);
		X = (((double)rand()) / RAND_MAX);
		Y = (((double)rand()) / RAND_MAX);
		Z = (((double)rand()) / RAND_MAX);
		mag = sqrt(X*X + Y*Y + Z*Z);
		X /= mag; Y /= mag; Z /= mag;
		cosAng = cos(theta / 2);
		sinAng = sin(theta / 2);
		point.pts[i].w = cosAng;
		point.pts[i].x = X * sinAng;
		point.pts[i].y = Y * sinAng;
		point.pts[i].z = Z * sinAng;
	}

	std::cout << "done\n";
}


template <typename num_t>
void kdtree_demo(const size_t N)
{
	PointCloud<num_t> cloud;

	// Generate points:
	generateRandomPointCloud(cloud, N);

	num_t query_pt[4] = { 0.5, 0.5, 0.5, 0.5};

	// construct a kd-tree index:
	typedef KDTreeSingleIndexAdaptor<
		SO3_Adaptor<num_t, PointCloud<num_t> > ,
		PointCloud<num_t>,
		4 /* dim */
		> my_kd_tree_t;

	dump_mem_usage();

	my_kd_tree_t   index(4 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
	index.buildIndex();

	dump_mem_usage();
	{
		// do a knn search
		const size_t num_results = 1;
		size_t ret_index;
		num_t out_dist_sqr;
		nanoflann::KNNResultSet<num_t> resultSet(num_results);
		resultSet.init(&ret_index, &out_dist_sqr );
		index.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

		std::cout << "knnSearch(nn="<<num_results<<"): \n";
		std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << endl;
	}
	{
		// Unsorted radius search:
		const num_t radius = 1;
		std::vector<std::pair<size_t,num_t> > indices_dists;
		RadiusResultSet<num_t,size_t> resultSet(radius,indices_dists);

		index.findNeighbors(resultSet, query_pt, nanoflann::SearchParams());

		// Get worst (furthest) point, without sorting:
		std::pair<size_t,num_t> worst_pair = resultSet.worst_item();
		cout << "Worst pair: idx=" << worst_pair.first << " dist=" << worst_pair.second << endl;
	}

}

int main()
{
	// Randomize Seed
	srand(time(NULL));
	kdtree_demo<float>(1000000);
	kdtree_demo<double>(1000000);
	return 0;
}

void dump_mem_usage()
{
	FILE* f=fopen("/proc/self/statm","rt");
	if (!f) return;
	char str[300];
	size_t n=fread(str,1,200,f);
	str[n]=0;
	printf("MEM: %s\n",str);
	fclose(f);
}
