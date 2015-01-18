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

#include <nanoflann.hpp>

#include <cstdlib>
#include <iostream>

#include <Eigen/Dense>

using namespace nanoflann;

const int SAMPLES_DIM = 15;

// ===== This example shows how to use nanoflann with these types of containers: =======
//typedef std::vector<Eigen::VectorXd> my_vector_of_vectors_t;
typedef std::vector<std::vector<double> > my_vector_of_vectors_t;
// =====================================================================================


/** A simple vector-of-vectors adaptor for nanoflann, without duplicating the storage.
  *  The i'th vector represents a point in the state space.
  *
  *  \tparam DIM If set to >0, it specifies a compile-time fixed dimensionality for the points in the data set, allowing more compiler optimizations.
  *  \tparam num_t The type of the point coordinates (typically, double or float).
  *  \tparam Distance The distance metric to use: nanoflann::metric_L1, nanoflann::metric_L2, nanoflann::metric_L2_Simple, etc.
  *  \tparam IndexType The type for indices in the KD-tree index (typically, size_t of int)
  */
template <class VectorOfVectorsType, typename num_t = double, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = size_t>
struct KDTreeVectorOfVectorsAdaptor
{
	typedef KDTreeVectorOfVectorsAdaptor<VectorOfVectorsType,num_t,DIM,Distance> self_t;
	typedef typename Distance::template traits<num_t,self_t>::distance_t metric_t;
	typedef KDTreeSingleIndexAdaptor< metric_t,self_t,DIM,IndexType>  index_t;

	index_t* index; //! The kd-tree index for the user to call its methods as usual with any other FLANN index.

	/// Constructor: takes a const ref to the vector of vectors object with the data points
	KDTreeVectorOfVectorsAdaptor(const int dimensionality, const VectorOfVectorsType &mat, const int leaf_max_size = 10) : m_data(mat)
	{
		assert(mat.size()!=0 && mat[0].size()!=0);
		const size_t dims = mat[0].size();
		if (DIM>0 && static_cast<int>(dims)!=DIM)
			throw std::runtime_error("Data set dimensionality does not match the 'DIM' template argument");
		index = new index_t( dims, *this /* adaptor */, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size ) );
		index->buildIndex();
	}

	~KDTreeVectorOfVectorsAdaptor() {
		delete index;
	}

	const VectorOfVectorsType &m_data;

	/** Query for the \a num_closest closest points to a given point (entered as query_point[0:dim-1]).
	  *  Note that this is a short-cut method for index->findNeighbors().
	  *  The user can also call index->... methods as desired.
	  * \note nChecks_IGNORED is ignored but kept for compatibility with the original FLANN interface.
	  */
	inline void query(const num_t *query_point, const size_t num_closest, IndexType *out_indices, num_t *out_distances_sq, const int nChecks_IGNORED = 10) const
	{
		nanoflann::KNNResultSet<typename VectorOfVectorsType::Scalar,IndexType> resultSet(num_closest);
		resultSet.init(out_indices, out_distances_sq);
		index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
	}

	/** @name Interface expected by KDTreeSingleIndexAdaptor
	  * @{ */

	const self_t & derived() const {
		return *this;
	}
	self_t & derived()       {
		return *this;
	}

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const {
		return m_data.size();
	}

	// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
	inline num_t kdtree_distance(const num_t *p1, const size_t idx_p2,size_t size) const
	{
		num_t s=0;
		for (size_t i=0; i<size; i++) {
			const num_t d= p1[i]-m_data[idx_p2][i];
			s+=d*d;
		}
		return s;
	}

	// Returns the dim'th component of the idx'th point in the class:
	inline num_t kdtree_get_pt(const size_t idx, int dim) const {
		return m_data[idx][dim];
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX & /*bb*/) const {
		return false;
	}

	/** @} */

}; // end of KDTreeVectorOfVectorsAdaptor


void generateRandomPointCloud(my_vector_of_vectors_t &samples, const size_t N,const size_t dim, const double max_range = 10.0)
{
	std::cout << "Generating "<< N << " random points...";
	samples.resize(N);
	for (size_t i=0;i<N;i++)
	{
		samples[i].resize(dim);
		for (size_t d=0;d<dim;d++)
			samples[i][d] = max_range * (rand() % 1000) / (1000.0);
	}
	std::cout << "done\n";
}

void kdtree_demo(const size_t nSamples,const size_t dim)
{
	my_vector_of_vectors_t  samples;

	const double max_range = 20;

	// Generate points:
	generateRandomPointCloud(samples, nSamples,dim, max_range);

	// Query point:
	std::vector<double> query_pt(dim);
	for (size_t d=0;d<dim;d++)
		query_pt[d] = max_range * (rand() % 1000) / (1000.0);

	// construct a kd-tree index:
	// Dimensionality set at run-time (default: L2)
	// ------------------------------------------------------------
	typedef KDTreeVectorOfVectorsAdaptor< my_vector_of_vectors_t, double >  my_kd_tree_t;

	my_kd_tree_t   mat_index(dim /*dim*/, samples, 10 /* max leaf */ );
	mat_index.index->buildIndex();

	// do a knn search
	const size_t num_results = 3;
	std::vector<size_t>   ret_indexes(num_results);
	std::vector<double> out_dists_sqr(num_results);

	nanoflann::KNNResultSet<double> resultSet(num_results);

	resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
	mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

	std::cout << "knnSearch(nn="<<num_results<<"): \n";
	for (size_t i=0;i<num_results;i++)
		std::cout << "ret_index["<<i<<"]=" << ret_indexes[i] << " out_dist_sqr=" << out_dists_sqr[i] << std::endl;
}

int main()
{
	kdtree_demo(1e3 /* samples */, SAMPLES_DIM /* dim */);
}

