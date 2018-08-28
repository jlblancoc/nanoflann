/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011 Jose Luis Blanco (joseluisblancoc@gmail.com).
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

#include "../../include/nanoflann.hpp"
#include <cstdlib>
#include <iostream>
#include <sys/time.h>

using namespace std;
using namespace nanoflann;

//#define VERBOSE_OUTPUT

#ifdef VERBOSE_OUTPUT
#	define VERB_COUT std::cout
#else
#	define VERB_COUT if (0) std::cout
#endif


template <typename T>
struct PointCloud
{
	struct Point
	{
		T  x,y,z;
	};

	std::vector<Point>  pts;

	typedef  PointCloud Derived; //!< In this case the dataset class is myself.

	/// CRTP helper method
	inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
	/// CRTP helper method
	inline       Derived& derived()       { return *static_cast<Derived*>(this); }

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline float kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		if (dim==0) return pts[idx].x;
		else if (dim==1) return pts[idx].y;
		else return pts[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX &bb) const { return false; }

};

double get_time()
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec+tv.tv_usec/1000000.0;
}

template <typename T>
void generateRandomPointCloud(PointCloud<T> &point, const size_t N, const T max_range = 10)
{
	VERB_COUT << "Generating "<< N << " point cloud...";
	const double t0=get_time();

	point.pts.resize(N);
	for (size_t i=0;i<N;i++)
	{
		point.pts[i].x = max_range * (rand() % 1000) / T(1000);
		point.pts[i].y = max_range * (rand() % 1000) / T(1000);
		point.pts[i].z = max_range * (rand() % 1000) / T(1000);
	}

	const double t1=get_time();
	VERB_COUT << "done in "<< (t1-t0)*1e3 << " ms\n";
}

template <typename num_t>
void perf_test(const size_t N)
{
	PointCloud<num_t> cloud;

	// Generate points:
	generateRandomPointCloud(cloud, N);

	num_t query_pt[3] = { 0.5, 0.5, 0.5};


	// construct an randomized kd-tree index using 4 kd-trees
	double t0=get_time();

	typedef KDTreeSingleIndexAdaptor<
		L2_Simple_Adaptor<num_t, PointCloud<num_t> > ,
		PointCloud<num_t>,
		3 /* dim */
		> my_kd_tree_t;

	my_kd_tree_t   index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
	index.buildIndex();
	double t1=get_time();
	const double At_build = t1-t0;
	VERB_COUT << "Build Index<>: " << (t1-t0)*1e3 << " ms\n";

	// do a knn search
	t0=get_time();

	const size_t num_results = 1;
	size_t ret_index;
	num_t out_dist_sqr;
	nanoflann::KNNResultSet<num_t> resultSet(num_results);
	resultSet.init(&ret_index, &out_dist_sqr );
	index.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
	//index.knnSearch(query, indices, dists, num_results, nanoflann::SearchParams(10));

	t1=get_time();
	const double At_search = t1-t0;
	VERB_COUT << "knnSearch(nn="<<num_results<<"): " << (t1-t0)*1e3 << " ms\n";
	VERB_COUT << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << endl;

	// Output for stats generation:
	const double At_mat = 0;

	cout
		<< N << "\t "
		<< At_mat << "\t "
		<< At_build << "\t "
		<< At_search << "\n";

}


int main(int argc, char** argv)
{
	// Number of points
	size_t Ns[]     = {1e3, 5e3, 1e4, 5e4, 1e5, 2e5, 5e5, 7e5, 1e6, 2e6, 5e6};
	// And repetitions for each point cloud size:
	size_t nReps[]  = {1e4, 1e3, 100, 100,  50,  50,  50,  20,  20,  10,  10};
	for (size_t i=0;i<sizeof(Ns)/sizeof(Ns[0]);i++)
	{
		cerr << " ==== N:" << Ns[i] << " ===\n"; cerr.flush();

		for (size_t repets=0;repets<nReps[i];repets++)
			perf_test<float>(Ns[i]);
	}

	return 0;
}

