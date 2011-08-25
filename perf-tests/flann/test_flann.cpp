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
#include <cstdlib>
#include <iostream>
#include <sys/time.h>

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
	cout << "Generating "<< N << " point cloud...";cout.flush();
	const double t0=get_time();
	
	point.pts.resize(N);
	for (size_t i=0;i<N;i++)
	{
		point.pts[i].x = max_range * (rand() % 1000) / T(1000);
		point.pts[i].y = max_range * (rand() % 1000) / T(1000);
		point.pts[i].z = max_range * (rand() % 1000) / T(1000);
	}
	
	const double t1=get_time();
	cout << "done in "<< (t1-t0)*1e3 << " ms\n";
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
	cout << "Convert to Matrix<>: " << (t1-t0)*1e3 << " ms\n";

	num_t query_pt[3] = { 0.5, 0.5, 0.5};
	Matrix<num_t> query(query_pt,1,3);
	
	const size_t num_results = 1;

	Matrix<int> indices(new int[query.rows*num_results], query.rows, num_results);
	Matrix<float> dists(new float[query.rows*num_results], query.rows, num_results);

	// construct an randomized kd-tree index using 4 kd-trees
	t0=get_time();
	Index<L2<float> > index(dataset, flann::KDTreeIndexParams(4));
	index.buildIndex();                                                                                               
	t1=get_time();
	cout << "Build Index<>: " << (t1-t0)*1e3 << " ms\n";

	// do a knn search
	t0=get_time();
	index.knnSearch(query, indices, dists, num_results, flann::SearchParams(10));
	t1=get_time();
	cout << "knnSearch(nn="<<num_results<<"): " << (t1-t0)*1e3 << " ms\n";

	dataset.free();
	indices.free();
	dists.free();
}


int main(int argc, char** argv)
{
	perf_test<float>(100000);
	perf_test<float>(1000000);

	return 0;
}

