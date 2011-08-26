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

#include <flann/flann.hpp>
#include <flann/algorithms/kdtree_single_index.h>
#include <cstdlib>
#include <iostream>
#include <sys/time.h>

//#define VERBOSE_OUTPUT

#ifdef VERBOSE_OUTPUT
#	define VERB_COUT std::cout
#else
#	define VERB_COUT if (0) std::cout
#endif


using namespace std;
using namespace flann;

template <typename T>
struct PointCloud
{
	struct Point
	{
		T  x,y,z;
	};

	std::vector<Point>  pts;
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
	VERB_COUT<< "Generating "<< N << " point cloud...";
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

	// Convert to Matrix<>:
	double t0=get_time();
	num_t *cloud_pts = new num_t[N*3];
	{
		num_t *ptr = cloud_pts;
		for (size_t i=0;i<N;i++)
		{
			*ptr++ = cloud.pts[i].x;
			*ptr++ = cloud.pts[i].y;
			*ptr++ = cloud.pts[i].z;
		}
	}
	Matrix<float> dataset(cloud_pts,cloud.pts.size(),3);
	double t1=get_time();
	const double At_mat = t1-t0;
	VERB_COUT<< "Convert to Matrix<>: " << (t1-t0)*1e3 << " ms\n";

	num_t query_pt[3] = { 0.5, 0.5, 0.5};

	const size_t num_results = 1;


	// construct an randomized kd-tree index using 4 kd-trees
	t0=get_time();
	KDTreeSingleIndex<L2<float> > index(dataset, flann::KDTreeSingleIndexParams(10 /* max leaf */) );
	index.buildIndex();
	t1=get_time();
	const double At_build = t1-t0;
	VERB_COUT << "Build Index<>: " << (t1-t0)*1e3 << " ms\n";

	// do a knn search
	t0=get_time();

	int ret_index;
	num_t out_dist_sqr;
	flann::KNNResultSet<num_t> resultSet(num_results);
	resultSet.init(&ret_index, &out_dist_sqr );
	index.findNeighbors(resultSet, &query_pt[0], flann::SearchParams(10));
	//index.knnSearch(query, indices, dists, num_results, mrpt_flann::SearchParams(10));

	t1=get_time();
	const double At_search = t1-t0;
	VERB_COUT << "knnSearch(nn="<<num_results<<"): " << (t1-t0)*1e3 << " ms\n";

	VERB_COUT << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << endl;

	// Output for stats generation:
	cout
		<< N << "\t "
		<< At_mat << "\t "
		<< At_build << "\t "
		<< At_search << "\n";

	dataset.free();
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

