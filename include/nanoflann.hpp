/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2008-2009  Marius Muja (mariusm@cs.ubc.ca). All rights reserved.
 * Copyright 2008-2009  David G. Lowe (lowe@cs.ubc.ca). All rights reserved.
 * Copyright 2011 Jose Luis Blanco (joseluisblancoc@gmail.com).
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

#ifndef  NANOFLANN_HPP_
#define  NANOFLANN_HPP_

#include <vector>
#include <string>
#include <cassert>
#include <map>
#include <algorithm>
#include <stdexcept>
#include <limits>

#include <cassert>
#include <cstring>
#include <cstdio>
#include <cmath>


namespace nanoflann
{
/** @addtogroup nanoflann_grp nanoflann C++ library for ANN
  *  @{ */

  	/** Library version: 0xMmP (M=Major,m=minor,P=path) */
	#define NANOFLANN_VERSION 0x101

	/** @addtogroup result_sets_grp Result set classes
	  *  @{ */
	template <typename DistanceType>
	class KNNResultSet
	{
		int* indices;
		DistanceType* dists;
		int capacity;
		int count;

	public:
		inline KNNResultSet(int capacity_) : capacity(capacity_), count(0)
		{
		}

		inline void init(int* indices_, DistanceType* dists_)
		{
			indices = indices_;
			dists = dists_;
			count = 0;
			dists[capacity-1] = (std::numeric_limits<DistanceType>::max)();
		}

		inline size_t size() const
		{
			return count;
		}

		inline bool full() const
		{
			return count == capacity;
		}


		inline void addPoint(DistanceType dist, int index)
		{
			int i;
			for (i=count; i>0; --i) {
#ifdef NANOFLANN_FIRST_MATCH
				if ( (dists[i-1]>dist) || ((dist==dists[i-1])&&(indices[i-1]>index)) ) {
#else
				if (dists[i-1]>dist) {
#endif
					if (i<capacity) {
						dists[i] = dists[i-1];
						indices[i] = indices[i-1];
					}
				}
				else break;
			}
			if (i<capacity) {
				dists[i] = dist;
				indices[i] = index;
			}
			if (count<capacity) count++;
		}

		inline DistanceType worstDist() const
		{
			return dists[capacity-1];
		}
	};


	/**
	 * A result-set class used when performing a radius based search.
	 */
	template <typename DistanceType>
	class RadiusResultSet
	{
	public:
		const DistanceType radius;

		std::vector<std::pair<int,DistanceType> >& m_indices_dists;

		inline RadiusResultSet(DistanceType radius_, std::vector<std::pair<int,DistanceType> >& indices_dists) : radius(radius_), m_indices_dists(indices_dists)
		{
			init();
		}

		inline ~RadiusResultSet() { }

		inline void init() { m_indices_dists.clear(); }

		inline size_t size() const { return m_indices_dists.size(); }

		inline bool full() const { return true; }

		inline void addPoint(DistanceType dist, int index)
		{
			if (dist<radius)
				m_indices_dists.push_back(std::make_pair<int,DistanceType>(index,dist));
		}

		inline DistanceType worstDist() const { return radius; }
	};

	/** operator "<" for std::sort() */
	template <typename DistanceType>
	struct IndexDist_Sorter
	{
		inline bool operator()(const std::pair<int,DistanceType> &p1, const std::pair<int,DistanceType> &p2) const
		{
			return p1.second < p2.second;
		}
	};

	/** @} */


	/** @addtogroup loadsave_grp Load/save auxiliary functions
	  * @{ */
	template<typename T>
	void save_value(FILE* stream, const T& value, int count = 1)
	{
		fwrite(&value, sizeof(value),count, stream);
	}

	template<typename T>
	void save_value(FILE* stream, const std::vector<T>& value)
	{
		size_t size = value.size();
		fwrite(&size, sizeof(size_t), 1, stream);
		fwrite(&value[0], sizeof(T), size, stream);
	}

	template<typename T>
	void load_value(FILE* stream, T& value, int count = 1)
	{
		int read_cnt = fread(&value, sizeof(value), count, stream);
		if (read_cnt != count) {
			throw std::runtime_error("Cannot read from file");
		}
	}


	template<typename T>
	void load_value(FILE* stream, std::vector<T>& value)
	{
		size_t size;
		int read_cnt = fread(&size, sizeof(size_t), 1, stream);
		if (read_cnt!=1) {
			throw std::runtime_error("Cannot read from file");
		}
		value.resize(size);
		read_cnt = fread(&value[0], sizeof(T), size, stream);
		if (read_cnt!=int(size)) {
			throw std::runtime_error("Cannot read from file");
		}
	}
	/** @} */


	/** @addtogroup metric_grp Metric (distance) classes
	  * @{ */

	template<typename T> inline T abs(T x) { return (x<0) ? -x : x; }
	template<> inline int abs<int>(int x) { return ::abs(x); }
	template<> inline float abs<float>(float x) { return fabsf(x); }
	template<> inline double abs<double>(double x) { return fabs(x); }
	template<> inline long double abs<long double>(long double x) { return fabsl(x); }

	/** Manhattan distance functor (generic version, optimized for high-dimensionality data sets).
	  *  Corresponding distance traits: nanoflann::metric_L1
	 */
	template<class T, class DataSource>
	struct L1_Adaptor
	{
		typedef T ElementType;
		typedef T DistanceType;
		typedef T ResultType;

		const DataSource &data_source;

		L1_Adaptor(const DataSource &_data_source) : data_source(_data_source) { }

		inline T operator()(const T* a, const size_t b_idx, size_t size, ResultType worst_dist = -1) const
		{
			ResultType result = ResultType();
			const T* last = a + size;
			const T* lastgroup = last - 3;
			size_t d = 0;

			/* Process 4 items with each loop for efficiency. */
			while (a < lastgroup) {
				const ResultType diff0 = nanoflann::abs(a[0] - data_source.kdtree_get_pt(b_idx,d++));
				const ResultType diff1 = nanoflann::abs(a[1] - data_source.kdtree_get_pt(b_idx,d++));
				const ResultType diff2 = nanoflann::abs(a[2] - data_source.kdtree_get_pt(b_idx,d++));
				const ResultType diff3 = nanoflann::abs(a[3] - data_source.kdtree_get_pt(b_idx,d++));
				result += diff0 + diff1 + diff2 + diff3;
				a += 4;
				if ((worst_dist>0)&&(result>worst_dist)) {
					return result;
				}
			}
			/* Process last 0-3 components.  Not needed for standard vector lengths. */
			while (a < last) {
				result += nanoflann::abs( *a++ - data_source.kdtree_get_pt(b_idx,d++) );
			}
			return result;
		}

		template <typename U, typename V>
		inline T accum_dist(const U a, const V b, int dim) const
		{
			return (a-b)*(a-b);
		}
	};

	/** Squared Euclidean distance functor (generic version, optimized for high-dimensionality data sets).
	  *  Corresponding distance traits: nanoflann::metric_L2
	 */
	template<class T, class DataSource>
	struct L2_Adaptor
	{
		typedef T ElementType;
		typedef T DistanceType;
		typedef T ResultType;

		const DataSource &data_source;

		L2_Adaptor(const DataSource &_data_source) : data_source(_data_source) { }

		inline T operator()(const T* a, const size_t b_idx, size_t size, ResultType worst_dist = -1) const
		{
			ResultType result = ResultType();
			const T* last = a + size;
			const T* lastgroup = last - 3;
			size_t d = 0;

			/* Process 4 items with each loop for efficiency. */
			while (a < lastgroup) {
				const ResultType diff0 = a[0] - data_source.kdtree_get_pt(b_idx,d++);
				const ResultType diff1 = a[1] - data_source.kdtree_get_pt(b_idx,d++);
				const ResultType diff2 = a[2] - data_source.kdtree_get_pt(b_idx,d++);
				const ResultType diff3 = a[3] - data_source.kdtree_get_pt(b_idx,d++);
				result += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
				a += 4;
				if ((worst_dist>0)&&(result>worst_dist)) {
					return result;
				}
			}
			/* Process last 0-3 components.  Not needed for standard vector lengths. */
			while (a < last) {
				const ResultType diff0 = *a++ - data_source.kdtree_get_pt(b_idx,d++);
				result += diff0 * diff0;
			}
			return result;
		}

		template <typename U, typename V>
		inline T accum_dist(const U a, const V b, int dim) const
		{
			return (a-b)*(a-b);
		}
	};

	/** Squared Euclidean distance functor (suitable for low-dimensionality datasets, like 2D or 3D point clouds)
	  *  Corresponding distance traits: nanoflann::metric_L2_Simple
	 */
	template<class T, class DataSource>
	struct L2_Simple_Adaptor
	{
		typedef T ElementType;
		typedef T DistanceType;
		typedef T ResultType;

		const DataSource &data_source;

		L2_Simple_Adaptor(const DataSource &_data_source) : data_source(_data_source) { }

		inline T operator()(const T* a, const size_t b_idx, size_t size) const {
			return data_source.kdtree_distance(a,b_idx,size);
		}

		template <typename U, typename V>
		inline T accum_dist(const U a, const V b, int dim) const
		{
			return (a-b)*(a-b);
		}
	};

	/** Metaprogramming helper traits class for the L1 (Manhattan) metric */
	struct metric_L1 {
		template<class T, class DataSource>
		struct traits {
			typedef L1_Adaptor<T,DataSource> distance_t;
		};
	};
	/** Metaprogramming helper traits class for the L2 (Euclidean) metric */
	struct metric_L2 {
		template<class T, class DataSource>
		struct traits {
			typedef L2_Adaptor<T,DataSource> distance_t;
		};
	};
	/** Metaprogramming helper traits class for the L2_simple (Euclidean) metric */
	struct metric_L2_Simple {
		template<class T, class DataSource>
		struct traits {
			typedef L2_Simple_Adaptor<T,DataSource> distance_t;
		};
	};

	/** @} */



	/** @addtogroup param_grp Parameter structs
	  * @{ */

	/**  Parameters
	  */
	struct KDTreeSingleIndexAdaptorParams
	{
		KDTreeSingleIndexAdaptorParams(int leaf_max_size_ = 10, int dim_ = -1) :
			leaf_max_size(leaf_max_size_), dim(dim_)
		{}

		int leaf_max_size;
		int dim;
	};

	/** Search options for KDTreeSingleIndexAdaptor::findNeighbors() */
	struct SearchParams
	{
		SearchParams(int checks_ = 32, float eps_ = 0, bool sorted_ = true ) :
			checks(checks_), eps(eps_), sorted(sorted_) {}

		int checks;  //!< how many leafs to visit when searching for neighbours (-1 for unlimited)
		float eps;  //!< search for eps-approximate neighbours (default: 0)
		bool sorted; //!< only for radius search, require neighbours sorted by distance (default: true)
	};
	/** @} */


	/** @addtogroup memalloc_grp Memory allocation
	  * @{ */

	/**
	 * Allocates (using C's malloc) a generic type T.
	 *
	 * Params:
	 *     count = number of instances to allocate.
	 * Returns: pointer (of type T*) to memory buffer
	 */
	template <typename T>
	T* allocate(size_t count = 1)
	{
		T* mem = (T*) ::malloc(sizeof(T)*count);
		return mem;
	}


	/**
	 * Pooled storage allocator
	 *
	 * The following routines allow for the efficient allocation of storage in
	 * small chunks from a specified pool.  Rather than allowing each structure
	 * to be freed individually, an entire pool of storage is freed at once.
	 * This method has two advantages over just using malloc() and free().  First,
	 * it is far more efficient for allocating small objects, as there is
	 * no overhead for remembering all the information needed to free each
	 * object or consolidating fragmented memory.  Second, the decision about
	 * how long to keep an object is made at the time of allocation, and there
	 * is no need to track down all the objects to free them.
	 *
	 */

	const size_t     WORDSIZE=16;
	const  size_t     BLOCKSIZE=8192;

	class PooledAllocator
	{
		/* We maintain memory alignment to word boundaries by requiring that all
		    allocations be in multiples of the machine wordsize.  */
		/* Size of machine word in bytes.  Must be power of 2. */
		/* Minimum number of bytes requested at a time from	the system.  Must be multiple of WORDSIZE. */


		int     remaining;  /* Number of bytes left in current block of storage. */
		void*   base;     /* Pointer to base of current block of storage. */
		void*   loc;      /* Current location in block to next allocate memory. */
		int     blocksize;


	public:
		int     usedMemory;
		int     wastedMemory;

		/**
		    Default constructor. Initializes a new pool.
		 */
		PooledAllocator(int blocksize = BLOCKSIZE)
		{
			this->blocksize = blocksize;
			remaining = 0;
			base = NULL;

			usedMemory = 0;
			wastedMemory = 0;
		}

		/**
		 * Destructor. Frees all the memory allocated in this pool.
		 */
		~PooledAllocator()
		{
			void* prev;

			while (base != NULL) {
				prev = *((void**) base); /* Get pointer to prev block. */
				::free(base);
				base = prev;
			}
		}

		/**
		 * Returns a pointer to a piece of new memory of the given size in bytes
		 * allocated from the pool.
		 */
		void* malloc(int size)
		{
			int blocksize;

			/* Round size up to a multiple of wordsize.  The following expression
			    only works for WORDSIZE that is a power of 2, by masking last bits of
			    incremented size to zero.
			 */
			size = (size + (WORDSIZE - 1)) & ~(WORDSIZE - 1);

			/* Check whether a new block must be allocated.  Note that the first word
			    of a block is reserved for a pointer to the previous block.
			 */
			if (size > remaining) {

				wastedMemory += remaining;

				/* Allocate new storage. */
				blocksize = (size + sizeof(void*) + (WORDSIZE-1) > BLOCKSIZE) ?
							size + sizeof(void*) + (WORDSIZE-1) : BLOCKSIZE;

				// use the standard C malloc to allocate memory
				void* m = ::malloc(blocksize);
				if (!m) {
					fprintf(stderr,"Failed to allocate memory.\n");
					return NULL;
				}

				/* Fill first word of new block with pointer to previous block. */
				((void**) m)[0] = base;
				base = m;

				int shift = 0;
				//int shift = (WORDSIZE - ( (((size_t)m) + sizeof(void*)) & (WORDSIZE-1))) & (WORDSIZE-1);

				remaining = blocksize - sizeof(void*) - shift;
				loc = ((char*)m + sizeof(void*) + shift);
			}
			void* rloc = loc;
			loc = (char*)loc + size;
			remaining -= size;

			usedMemory += size;

			return rloc;
		}

		/**
		 * Allocates (using this pool) a generic type T.
		 *
		 * Params:
		 *     count = number of instances to allocate.
		 * Returns: pointer (of type T*) to memory buffer
		 */
		template <typename T>
		T* allocate(size_t count = 1)
		{
			T* mem = (T*) this->malloc(static_cast<int>( sizeof(T)*count ));
			return mem;
		}

	};
	/** @} */


	/** @addtogroup kdtrees_grp KD-tree classes and adaptors
	  * @{ */

	/** kd-tree index
	 *
	 * Contains the k-d trees and other information for indexing a set of points
	 * for nearest-neighbor matching.
	 *
	 *  The class "DatasetAdaptor" must provide the following interface (can be non-virtual, inlined methods):
	 *
	 *  \code
	 *   // Must return the number of data points
	 *   inline size_t kdtree_get_point_count() const { ... }
	 *
	 *   // Must return the Euclidean (L2) distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
	 *   inline float kdtree_distance(const float *p1, const size_t idx_p2,size_t size) const { ... }
	 *
	 *   // Must return the dim'th component of the idx'th point in the class:
	 *   inline num_t kdtree_get_pt(const size_t idx, int dim) const { ... }
	 *
	 *   // Optional bounding-box computation: return false to default to a standard bbox computation loop.
	 *   //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	 *   //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	 *   template <class BBOX>
	 *   bool kdtree_get_bbox(BBOX &bb) const
	 *   {
	 *      bb[0].low = ...; bb[0].high = ...;  // 0th dimension limits
	 *      bb[1].low = ...; bb[1].high = ...;  // 1st dimension limits
	 *      ...
	 *      return true;
	 *   }
	 *
	 *  \endcode
	 */
	template <typename Distance, class DatasetAdaptor,int DIM = -1>
	class KDTreeSingleIndexAdaptor
	{
		typedef typename Distance::ElementType ElementType;
		typedef typename Distance::ResultType DistanceType;

		/**
		 *  Array of indices to vectors in the dataset.
		 */
		std::vector<int> vind;

		int leaf_max_size_;


		/**
		 * The dataset used by this index
		 */
		const DatasetAdaptor &dataset; //!< The source of our data

		const KDTreeSingleIndexAdaptorParams index_params;

		int size_;
		int dim;


		/*--------------------- Internal Data Structures --------------------------*/
		struct Node
		{
			union {
				struct
				{
					/**
					 * Indices of points in leaf node
					 */
					int left, right;
				} lr;
				struct
				{
					/**
					 * Dimension used for subdivision.
					 */
					int divfeat;
					/**
					 * The values used for subdivision.
					 */
					DistanceType divlow, divhigh;
				} sub;
			};
			/**
			 * The child nodes.
			 */
			Node* child1, * child2;
		};
		typedef Node* NodePtr;


		struct Interval
		{
			ElementType low, high;
		};

		typedef std::vector<Interval> BoundingBox;


		/** This record represents a branch point when finding neighbors in
			the tree.  It contains a record of the minimum distance to the query
			point, as well as the node at which the search resumes.
		 */
		template <typename T, typename DistanceType>
		struct BranchStruct
		{
			T node;           /* Tree node at which search resumes */
			DistanceType mindist;     /* Minimum distance to query for all nodes below. */

			BranchStruct() {}
			BranchStruct(const T& aNode, DistanceType dist) : node(aNode), mindist(dist) {}

			bool operator<(const BranchStruct<T, DistanceType>& rhs) const
			{
				return mindist<rhs.mindist;
			}
		};

		/**
		 * Array of k-d trees used to find neighbours.
		 */
		NodePtr root_node;
		typedef BranchStruct<NodePtr, DistanceType> BranchSt;
		typedef BranchSt* Branch;

		BoundingBox root_bbox;

		/**
		 * Pooled memory allocator.
		 *
		 * Using a pooled memory allocator is more efficient
		 * than allocating memory directly when there is a large
		 * number small of memory allocations.
		 */
		PooledAllocator pool;

	public:

		Distance distance;

		/**
		 * KDTree constructor
		 *
		 * Params:
		 *          inputData = dataset with the input features
		 *          params = parameters passed to the kdtree algorithm
		 */
		KDTreeSingleIndexAdaptor(const int dimensionality, const DatasetAdaptor& inputData, const KDTreeSingleIndexAdaptorParams& params = KDTreeSingleIndexAdaptorParams() ) :
			dataset(inputData), index_params(params), distance(inputData)
		{
			size_ = static_cast<int>(dataset.kdtree_get_point_count());
			dim = dimensionality;
			if (DIM>0) dim=DIM;
			else {
				if (params.dim>0) dim = params.dim;
			}
			leaf_max_size_ = params.leaf_max_size;

			// Create a permutable array of indices to the input vectors.
			vind.resize(size_);
			for (int i = 0; i < size_; i++) {
				vind[i] = i;
			}
		}

		/**
		 * Standard destructor
		 */
		~KDTreeSingleIndexAdaptor()
		{
		}

		/**
		 * Builds the index
		 */
		void buildIndex()
		{
			computeBoundingBox(root_bbox);
			root_node = divideTree(0, size_, root_bbox );   // construct the tree
		}

		/**
		 *  Returns size of index.
		 */
		size_t size() const
		{
			return size_;
		}

		/**
		 * Returns the length of an index feature.
		 */
		size_t veclen() const
		{
			return (DIM>0 ? DIM : dim);
		}

		/**
		 * Computes the inde memory usage
		 * Returns: memory used by the index
		 */
		int usedMemory() const
		{
			return pool.usedMemory+pool.wastedMemory+dataset.kdtree_get_point_count()*sizeof(int);  // pool memory and vind array memory
		}

		/** \name Query methods
		  * @{ */

		/**
		 * Find set of nearest neighbors to vec[0:dim-1]. Their indices are stored inside
		 * the result object.
		 *
		 * Params:
		 *     result = the result object in which the indices of the nearest-neighbors are stored
		 *     vec = the vector for which to search the nearest neighbors
		 *     maxCheck = the maximum number of restarts (in a best-bin-first manner)
		 *
		 * \tparam RESULTSET Should be any ResultSet<DistanceType>
		 * \sa knnSearch, radiusSearch
		 */
		template <typename RESULTSET>
		void findNeighbors(RESULTSET& result, const ElementType* vec, const SearchParams& searchParams) const
		{
			float epsError = 1+searchParams.eps;

			std::vector<DistanceType> dists( (DIM>0 ? DIM : dim) ,0);
			DistanceType distsq = computeInitialDistances(vec, dists);
			int count_leaf = 0;
			searchLevel(result, vec, root_node, distsq, dists, epsError,count_leaf);
		}

		/**
		 * Find the "num_closest" nearest neighbors to the \a query_point[0:dim-1]. Their indices are stored inside
		 * the result object.
		 *  \sa radiusSearch, findNeighbors
		 */
		inline void knnSearch(const ElementType *query_point, int num_closest, int *out_indices, ElementType *out_distances_sq, const int nChecks = 10) const
		{
			nanoflann::KNNResultSet<ElementType> resultSet(num_closest);
			resultSet.init(out_indices, out_distances_sq);
			this->findNeighbors(resultSet, query_point, nanoflann::SearchParams(nChecks));
		}

		/**
		 * Find all the neighbors to \a query_point[0:dim-1] within a maximum radius.
		 *  The output is given as a vector of pairs, of which the first element is a point index and the second the corresponding distance.
		 *  Previous contents of \a IndicesDists are cleared.
		 *
		 *  If searchParams.sorted==true, the output list is sorted by ascending distances.
		 *
		 *  For a better performance, it is advisable to do a .reserve() on the vector if you have any wild guess about the number of expected matches.
		 *
		 *  \sa knnSearch, findNeighbors
		 * \return The number of points within the given radius (i.e. indices.size() or dists.size() )
		 */
		size_t radiusSearch(const ElementType *query_point,const DistanceType radius, std::vector<std::pair<int,DistanceType> >& IndicesDists, const SearchParams& searchParams) const
		{
			RadiusResultSet<DistanceType> resultSet(radius,IndicesDists);
			this->findNeighbors(resultSet, query_point, searchParams);

			if (searchParams.sorted)
				std::sort(IndicesDists.begin(),IndicesDists.end(), IndexDist_Sorter<DistanceType>() );

			return resultSet.size();
		}

		/** @} */

	private:

		/// Helper accessor to the dataset points:
		inline ElementType dataset_get(size_t idx, int component) const {
			return dataset.kdtree_get_pt(idx,component);
		}


		void save_tree(FILE* stream, NodePtr tree)
		{
			save_value(stream, *tree);
			if (tree->child1!=NULL) {
				save_tree(stream, tree->child1);
			}
			if (tree->child2!=NULL) {
				save_tree(stream, tree->child2);
			}
		}


		void load_tree(FILE* stream, NodePtr& tree)
		{
			tree = pool.allocate<Node>();
			load_value(stream, *tree);
			if (tree->child1!=NULL) {
				load_tree(stream, tree->child1);
			}
			if (tree->child2!=NULL) {
				load_tree(stream, tree->child2);
			}
		}


		void computeBoundingBox(BoundingBox& bbox)
		{
			bbox.resize((DIM>0 ? DIM : dim));
			if (dataset.kdtree_get_bbox(bbox))
			{
				// Done! It was implemented in derived class
			}
			else
			{
				for (int i=0; i<(DIM>0 ? DIM : dim); ++i) {
					bbox[i].low =
						bbox[i].high = dataset_get(0,i);
				}
				const size_t N = dataset.kdtree_get_point_count();
				for (size_t k=1; k<N; ++k) {
					for (int i=0; i<(DIM>0 ? DIM : dim); ++i) {
						if (dataset_get(k,i)<bbox[i].low) bbox[i].low = dataset_get(k,i);
						if (dataset_get(k,i)>bbox[i].high) bbox[i].high = dataset_get(k,i);
					}
				}
			}
		}


		/**
		 * Create a tree node that subdivides the list of vecs from vind[first]
		 * to vind[last].  The routine is called recursively on each sublist.
		 * Place a pointer to this new tree node in the location pTree.
		 *
		 * Params: pTree = the new node to create
		 *                  first = index of the first vector
		 *                  last = index of the last vector
		 */
		NodePtr divideTree(int left, int right, BoundingBox& bbox)
		{
			NodePtr node = pool.allocate<Node>(); // allocate memory

			/* If too few exemplars remain, then make this a leaf node. */
			if ( (right-left) <= leaf_max_size_) {
				node->child1 = node->child2 = NULL;    /* Mark as leaf node. */
				node->lr.left = left;
				node->lr.right = right;

				// compute bounding-box of leaf points
				for (int i=0; i<(DIM>0 ? DIM : dim); ++i) {
					bbox[i].low = dataset_get(vind[left],i);
					bbox[i].high = dataset_get(vind[left],i);
				}
				for (int k=left+1; k<right; ++k) {
					for (int i=0; i<(DIM>0 ? DIM : dim); ++i) {
						if (bbox[i].low>dataset_get(vind[k],i)) bbox[i].low=dataset_get(vind[k],i);
						if (bbox[i].high<dataset_get(vind[k],i)) bbox[i].high=dataset_get(vind[k],i);
					}
				}
			}
			else {
				int idx;
				int cutfeat;
				DistanceType cutval;
				middleSplit_(&vind[0]+left, right-left, idx, cutfeat, cutval, bbox);

				node->sub.divfeat = cutfeat;

				BoundingBox left_bbox(bbox);
				left_bbox[cutfeat].high = cutval;
				node->child1 = divideTree(left, left+idx, left_bbox);

				BoundingBox right_bbox(bbox);
				right_bbox[cutfeat].low = cutval;
				node->child2 = divideTree(left+idx, right, right_bbox);

				node->sub.divlow = left_bbox[cutfeat].high;
				node->sub.divhigh = right_bbox[cutfeat].low;

				for (size_t i=0; i<(DIM>0 ? DIM : dim); ++i) {
					bbox[i].low = std::min(left_bbox[i].low, right_bbox[i].low);
					bbox[i].high = std::max(left_bbox[i].high, right_bbox[i].high);
				}
			}

			return node;
		}

		void computeMinMax(int* ind, int count, int element, ElementType& min_elem, ElementType& max_elem)
		{
			min_elem = dataset_get(ind[0],element);
			max_elem = dataset_get(ind[0],element);
			for (int i=1; i<count; ++i) {
				ElementType val = dataset_get(ind[i],element);
				if (val<min_elem) min_elem = val;
				if (val>max_elem) max_elem = val;
			}
		}

		void middleSplit(int* ind, int count, int& index, int& cutfeat, DistanceType& cutval, const BoundingBox& bbox)
		{
			// find the largest span from the approximate bounding box
			ElementType max_span = bbox[0].high-bbox[0].low;
			cutfeat = 0;
			cutval = (bbox[0].high+bbox[0].low)/2;
			for (size_t i=1; i<(DIM>0 ? DIM : dim); ++i) {
				ElementType span = bbox[i].low-bbox[i].low;
				if (span>max_span) {
					max_span = span;
					cutfeat = i;
					cutval = (bbox[i].high+bbox[i].low)/2;
				}
			}

			// compute exact span on the found dimension
			ElementType min_elem, max_elem;
			computeMinMax(ind, count, cutfeat, min_elem, max_elem);
			cutval = (min_elem+max_elem)/2;
			max_span = max_elem - min_elem;

			// check if a dimension of a largest span exists
			size_t k = cutfeat;
			for (size_t i=0; i<(DIM>0 ? DIM : dim); ++i) {
				if (i==k) continue;
				ElementType span = bbox[i].high-bbox[i].low;
				if (span>max_span) {
					computeMinMax(ind, count, i, min_elem, max_elem);
					span = max_elem - min_elem;
					if (span>max_span) {
						max_span = span;
						cutfeat = i;
						cutval = (min_elem+max_elem)/2;
					}
				}
			}
			int lim1, lim2;
			planeSplit(ind, count, cutfeat, cutval, lim1, lim2);

			if (lim1>count/2) index = lim1;
			else if (lim2<count/2) index = lim2;
			else index = count/2;
		}


		void middleSplit_(int* ind, int count, int& index, int& cutfeat, DistanceType& cutval, const BoundingBox& bbox)
		{
			const DistanceType EPS=static_cast<DistanceType>(0.00001);
			ElementType max_span = bbox[0].high-bbox[0].low;
			for (size_t i=1; i<(DIM>0 ? DIM : dim); ++i) {
				ElementType span = bbox[i].high-bbox[i].low;
				if (span>max_span) {
					max_span = span;
				}
			}
			ElementType max_spread = -1;
			cutfeat = 0;
			for (int i=0; i<(DIM>0 ? DIM : dim); ++i) {
				ElementType span = bbox[i].high-bbox[i].low;
				if (span>(1-EPS)*max_span) {
					ElementType min_elem, max_elem;
					computeMinMax(ind, count, cutfeat, min_elem, max_elem);
					ElementType spread = max_elem-min_elem;;
					if (spread>max_spread) {
						cutfeat = i;
						max_spread = spread;
					}
				}
			}
			// split in the middle
			DistanceType split_val = (bbox[cutfeat].low+bbox[cutfeat].high)/2;
			ElementType min_elem, max_elem;
			computeMinMax(ind, count, cutfeat, min_elem, max_elem);

			if (split_val<min_elem) cutval = min_elem;
			else if (split_val>max_elem) cutval = max_elem;
			else cutval = split_val;

			int lim1, lim2;
			planeSplit(ind, count, cutfeat, cutval, lim1, lim2);

			if (lim1>count/2) index = lim1;
			else if (lim2<count/2) index = lim2;
			else index = count/2;
		}


		/**
		 *  Subdivide the list of points by a plane perpendicular on axe corresponding
		 *  to the 'cutfeat' dimension at 'cutval' position.
		 *
		 *  On return:
		 *  dataset[ind[0..lim1-1]][cutfeat]<cutval
		 *  dataset[ind[lim1..lim2-1]][cutfeat]==cutval
		 *  dataset[ind[lim2..count]][cutfeat]>cutval
		 */
		void planeSplit(int* ind, int count, int cutfeat, DistanceType cutval, int& lim1, int& lim2)
		{
			/* Move vector indices for left subtree to front of list. */
			int left = 0;
			int right = count-1;
			for (;; ) {
				while (left<=right && dataset_get(ind[left],cutfeat)<cutval) ++left;
				while (left<=right && dataset_get(ind[right],cutfeat)>=cutval) --right;
				if (left>right) break;
				std::swap(ind[left], ind[right]);
				++left;
				--right;
			}
			/* If either list is empty, it means that all remaining features
			 * are identical. Split in the middle to maintain a balanced tree.
			 */
			lim1 = left;
			right = count-1;
			for (;; ) {
				while (left<=right && dataset_get(ind[left],cutfeat)<=cutval) ++left;
				while (left<=right && dataset_get(ind[right],cutfeat)>cutval) --right;
				if (left>right) break;
				std::swap(ind[left], ind[right]);
				++left;
				--right;
			}
			lim2 = left;
		}

		DistanceType computeInitialDistances(const ElementType* vec, std::vector<DistanceType>& dists) const
		{
			DistanceType distsq = 0.0;

			for (int i = 0; i < (DIM>0 ? DIM : dim); ++i) {
				if (vec[i] < root_bbox[i].low) {
					dists[i] = distance.accum_dist(vec[i], root_bbox[i].low, i);
					distsq += dists[i];
				}
				if (vec[i] > root_bbox[i].high) {
					dists[i] = distance.accum_dist(vec[i], root_bbox[i].high, i);
					distsq += dists[i];
				}
			}

			return distsq;
		}

		/**
		 * Performs an exact search in the tree starting from a node.
		 * \tparam RESULTSET Should be any ResultSet<DistanceType>
		 */
		template <class RESULTSET>
		void searchLevel(RESULTSET& result_set, const ElementType* vec, const NodePtr node, DistanceType mindistsq,
						 std::vector<DistanceType>& dists, const float epsError, int &count_leaf) const
		{
			/* If this is a leaf node, then do check and return. */
			if ((node->child1 == NULL)&&(node->child2 == NULL)) {
				count_leaf += (node->lr.right-node->lr.left);
				DistanceType worst_dist = result_set.worstDist();
				for (int i=node->lr.left; i<node->lr.right; ++i) {
					int index = vind[i];// reorder... : i;
					DistanceType dist = distance(vec, index, (DIM>0 ? DIM : dim));
					if (dist<worst_dist) {
						result_set.addPoint(dist,vind[i]);
					}
				}
				return;
			}

			/* Which child branch should be taken first? */
			int idx = node->sub.divfeat;
			ElementType val = vec[idx];
			DistanceType diff1 = val - node->sub.divlow;
			DistanceType diff2 = val - node->sub.divhigh;

			NodePtr bestChild;
			NodePtr otherChild;
			DistanceType cut_dist;
			if ((diff1+diff2)<0) {
				bestChild = node->child1;
				otherChild = node->child2;
				cut_dist = distance.accum_dist(val, node->sub.divhigh, idx);
			}
			else {
				bestChild = node->child2;
				otherChild = node->child1;
				cut_dist = distance.accum_dist( val, node->sub.divlow, idx);
			}

			/* Call recursively to search next level down. */
			searchLevel(result_set, vec, bestChild, mindistsq, dists, epsError,count_leaf);

			DistanceType dst = dists[idx];
			mindistsq = mindistsq + cut_dist - dst;
			dists[idx] = cut_dist;
			if (mindistsq*epsError<=result_set.worstDist()) {
				searchLevel(result_set, vec, otherChild, mindistsq, dists, epsError,count_leaf);
			}
			dists[idx] = dst;
		}


		void saveIndex(FILE* stream)
		{
			save_value(stream, size_);
			save_value(stream, dim);
			save_value(stream, root_bbox);
			save_value(stream, leaf_max_size_);
			save_value(stream, vind);
			save_tree(stream, root_node);
		}

		void loadIndex(FILE* stream)
		{
			load_value(stream, size_);
			load_value(stream, dim);
			load_value(stream, root_bbox);
			load_value(stream, leaf_max_size_);
			load_value(stream, vind);
			load_tree(stream, root_node);
		}

	};   // class KDTree


	/** A simple KD-tree adaptor for working with data directly stored in an Eigen Matrix, without duplicating the data storage.
	  *  Each row in the matrix represents a point in the state space.
	  *
	  *  Example of usage:
	  * \code
	  * 	Eigen::Matrix<num_t,Dynamic,Dynamic>  mat;
	  * 	// Fill out "mat"...
	  *
	  * 	typedef KDTreeEigenMatrixAdaptor< Eigen::Matrix<num_t,Dynamic,Dynamic> >  my_kd_tree_t;
	  * 	const int max_leaf = 10;
	  * 	my_kd_tree_t   mat_index(dimdim, mat, max_leaf );
	  * 	mat_index.index->buildIndex();
	  * 	mat_index.index->...
	  * \endcode
	  *
	  *  \tparam DIM If set to >0, it specifies a compile-time fixed dimensionality for the points in the data set, allowing more compiler optimizations.
	  *  \tparam Distance The distance metric to use: nanoflann::metric_L1, nanoflann::metric_L2, nanoflann::metric_L2_Simple, etc.
	  */
	template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2>
	struct KDTreeEigenMatrixAdaptor
	{
		typedef KDTreeEigenMatrixAdaptor<MatrixType,DIM,Distance> self_t;
		typedef typename MatrixType::Scalar              num_t;
		typedef typename Distance::template traits<num_t,self_t>::distance_t metric_t;
		typedef KDTreeSingleIndexAdaptor< metric_t,self_t,DIM>  index_t;

		index_t* index; //! The kd-tree index for the user to call its methods as usual with any other FLANN index.

		/// Constructor: takes a const ref to the matrix object with the data points
		KDTreeEigenMatrixAdaptor(const int dimensionality, const MatrixType &mat, const int leaf_max_size = 10) : m_data_matrix(mat)
		{
			const size_t dims = mat.cols();
			if (DIM>0 && static_cast<int>(dims)!=DIM)
				throw std::runtime_error("Data set dimensionality does not match the 'DIM' template argument");
			index = new index_t( dims, *this /* adaptor */, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size, dims ) );
			index->buildIndex();
		}

		~KDTreeEigenMatrixAdaptor() {
			delete index;
		}

		const MatrixType &m_data_matrix;

		/** Query for the \a num_closest closest points to a given point (entered as query_point[0:dim-1]).
		  *  Note that this is a short-cut method for index->findNeighbors().
		  *  The user can also call index->... methods as desired.
		  */
		inline void query(const num_t *query_point, int num_closest, int *out_indices, num_t *out_distances_sq, const int nChecks = 10) const
		{
			nanoflann::KNNResultSet<typename MatrixType::Scalar> resultSet(num_closest);
			resultSet.init(out_indices, out_distances_sq);
			index->findNeighbors(resultSet, query_point, nanoflann::SearchParams(nChecks));
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
			return m_data_matrix.rows();
		}

		// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
		inline num_t kdtree_distance(const num_t *p1, const size_t idx_p2,size_t size) const
		{
			num_t s=0;
			for (size_t i=0; i<size; i++) {
				const num_t d= p1[i]-m_data_matrix.coeff(idx_p2,i);
				s+=d*d;
			}
			return s;
		}

		// Returns the dim'th component of the idx'th point in the class:
		inline num_t kdtree_get_pt(const size_t idx, int dim) const {
			return m_data_matrix.coeff(idx,dim);
		}

		// Optional bounding-box computation: return false to default to a standard bbox computation loop.
		//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
		//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
		template <class BBOX>
		bool kdtree_get_bbox(BBOX &bb) const {
			return false;
		}

		/** @} */

	}; // end of KDTreeEigenMatrixAdaptor
	/** @} */

/** @} */ // end of grouping
} // end of NS


#endif /* NANOFLANN_HPP_ */
