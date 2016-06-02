/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2011-2016 Jose Luis Blanco (joseluisblancoc@gmail.com).
 *   All rights reserved.
 *
 * THE BSD LICENSE
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

#include <gtest/gtest.h>

#include <nanoflann.hpp>

#include <cstdlib>
#include <iostream>

using namespace std;
using namespace nanoflann;

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}



// This is an exampleof a custom data set class
template <typename T>
struct PointCloud
{
	struct Point
	{
		T  x,y,z;
	};

	std::vector<Point>  pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
	inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t /* size*/) const
	{
		const T d0=p1[0]-pts[idx_p2].x;
		const T d1=p1[1]-pts[idx_p2].y;
		const T d2=p1[2]-pts[idx_p2].z;
		return d0*d0+d1*d1+d2*d2;
	}

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline T kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim==0) return pts[idx].x;
		else if (dim==1) return pts[idx].y;
		else return pts[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX & /* bb*/ ) const { return false; }

};

template <typename T>
void generateRandomPointCloud(PointCloud<T> &point, const size_t N, const T max_range = 10)
{
	point.pts.resize(N);
	for (size_t i=0;i<N;i++)
	{
		point.pts[i].x = max_range * (rand() % 1000) / T(1000);
		point.pts[i].y = max_range * (rand() % 1000) / T(1000);
		point.pts[i].z = max_range * (rand() % 1000) / T(1000);
	}
}

template <typename num_t>
void L2_vs_L2_simple_test(const size_t N, const size_t num_results)
{
	PointCloud<num_t> cloud;

	// Generate points:
	generateRandomPointCloud(cloud, N);

	num_t query_pt[3] = { 0.5, 0.5, 0.5};

	// construct a kd-tree index:
	typedef KDTreeSingleIndexAdaptor<
		L2_Simple_Adaptor<num_t, PointCloud<num_t> > ,
		PointCloud<num_t>,
		3 /* dim */
		> my_kd_tree_simple_t;

	typedef KDTreeSingleIndexAdaptor<
		L2_Adaptor<num_t, PointCloud<num_t> > ,
		PointCloud<num_t>,
		3 /* dim */
		> my_kd_tree_t;

	my_kd_tree_simple_t   index1(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
	index1.buildIndex();

	my_kd_tree_t   index2(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
	index2.buildIndex();

	// do a knn search
	std::vector<size_t>   ret_index(num_results);
	std::vector<num_t> out_dist_sqr(num_results);
	nanoflann::KNNResultSet<num_t> resultSet(num_results);
	resultSet.init(&ret_index[0], &out_dist_sqr[0] );
	index1.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

	std::vector<size_t>   ret_index1 = ret_index;
	std::vector<num_t> out_dist_sqr1 = out_dist_sqr;

	resultSet.init(&ret_index[0], &out_dist_sqr[0] );

	index2.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

	for (size_t i=0;i<num_results;i++)
	{
		EXPECT_EQ(ret_index1[i],ret_index[i]);
		EXPECT_DOUBLE_EQ(out_dist_sqr1[i],out_dist_sqr[i]);
	}
}

TEST(kdtree,L2_vs_L2_simple)
{
	for (int nResults=1;nResults<10;nResults++)
	{
		L2_vs_L2_simple_test<float>(100, nResults);
		L2_vs_L2_simple_test<double>(100, nResults);
	}
}


TEST(kdtree,robust_empty_tree)
{
	// Try to build a tree with 0 data points, to test
	// robustness against this situation:
	PointCloud<double> cloud;

	double query_pt[3] = { 0.5, 0.5, 0.5};

	// construct a kd-tree index:
	typedef KDTreeSingleIndexAdaptor<
		L2_Simple_Adaptor<double, PointCloud<double> > ,
		PointCloud<double>,
		3 /* dim */
		> my_kd_tree_simple_t;

	my_kd_tree_simple_t   index1(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
	index1.buildIndex();


	// Now we will try to search in the tree, and WE EXPECT a result of
	// no neighbors found if the error detection works fine:
	const size_t num_results = 1;
	std::vector<size_t>   ret_index(num_results);
	std::vector<double> out_dist_sqr(num_results);
	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init(&ret_index[0], &out_dist_sqr[0] );
	bool result = index1.findNeighbors(resultSet, &query_pt[0],
		nanoflann::SearchParams(10));
	EXPECT_EQ(result, false);
}

using namespace nanoflann;
#include "../examples/KDTreeVectorOfVectorsAdaptor.h"

template <typename NUM>
void generateRandomPointCloud(std::vector<std::vector<NUM> > &samples, const size_t N,const size_t dim, const NUM max_range)
{
	//std::cout << "Generating "<< N << " random points...";
	samples.resize(N);
	for (size_t i=0;i<N;i++)
	{
		samples[i].resize(dim);
		for (size_t d=0;d<dim;d++)
			samples[i][d] = max_range * (rand() % 1000) / NUM(1000.0);
	}
	//std::cout << "done\n";
}

template <typename NUM>
void L2_vs_bruteforce_test(const size_t nSamples,const int DIM)
{
	std::vector<std::vector<NUM> > samples;

	const NUM max_range = NUM(20.0);

	// Generate points:
	generateRandomPointCloud(samples, nSamples,DIM, max_range);

	// Query point:
	std::vector<NUM> query_pt(DIM);
	for (size_t d=0;d<DIM;d++)
		query_pt[d] = max_range * (rand() % 1000) / (1000.0);

	// construct a kd-tree index:
	// Dimensionality set at run-time (default: L2)
	// ------------------------------------------------------------
	typedef KDTreeVectorOfVectorsAdaptor< std::vector<std::vector<NUM> >, NUM >  my_kd_tree_t;

	my_kd_tree_t   mat_index(DIM /*dim*/, samples, 10 /* max leaf */ );
	mat_index.index->buildIndex();

	// do a knn search
	const size_t num_results = 1;
	std::vector<size_t>   ret_indexes(num_results);
	std::vector<NUM> out_dists_sqr(num_results);

	nanoflann::KNNResultSet<NUM> resultSet(num_results);

	resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
	mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10) );

	// Brute force:
	double min_dist_L2 = std::numeric_limits<double>::max();
	int    min_idx = -1;
	{
		for (size_t i=0;i<nSamples;i++)
		{
			double dist=0.0;
			for (int d=0;d<DIM;d++)
				dist+= (query_pt[d]-samples[i][d])*(query_pt[d]-samples[i][d]);
			if (dist<min_dist_L2)
			{
				min_dist_L2=dist;
				min_idx = i;
			}
		}
		ASSERT_TRUE(min_idx!=-1);
	}

	// Compare:
	EXPECT_EQ(min_idx,ret_indexes[0]);
	EXPECT_NEAR(min_dist_L2,out_dists_sqr[0],1e-3);
}


TEST(kdtree,L2_vs_bruteforce)
{
	srand(time(NULL));
	for (int i=0;i<10;i++)
	{
		L2_vs_bruteforce_test<float>(100, 2);
		L2_vs_bruteforce_test<float>(100, 3);
		L2_vs_bruteforce_test<float>(100, 7);

		L2_vs_bruteforce_test<double>(100, 2);
		L2_vs_bruteforce_test<double>(100, 3);
		L2_vs_bruteforce_test<double>(100, 7);
	}
}


