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
#include <fstream>
#include <string>

using namespace std;
using namespace nanoflann;

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
	inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t /*size*/) const
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
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

};

// Scan all points from file
template <typename T>
void scanPointCloud(PointCloud<T> &point, string file)
{
        ifstream read(file.c_str());
        
        string temp;
        getline(read,temp);

        vector<vector<T> > cloud;
        vector<T> tmp;
        
        double x,y,z,d;
        unsigned int N=0;
        while(read>>x>>y>>z>>d){
            tmp.clear();
            tmp.push_back(x); tmp.push_back(y); tmp.push_back(z);
            cloud.push_back(tmp);
            N++;
        }
    random_shuffle(cloud.begin(), cloud.end());
	point.pts.resize(N);
	for (size_t i=0;i<N;i++)
	{
		point.pts[i].x = cloud[i][0];
		point.pts[i].y = cloud[i][1];
		point.pts[i].z = cloud[i][2];
        }
}


template <typename num_t>
void kdtree_demo(string &path)
{
	PointCloud<num_t> PcloudS, PcloudT;
	// Scan points from file:
	scanPointCloud<num_t>(PcloudS, path+"scan1.dat");
	
        scanPointCloud<num_t>(PcloudT, path+"scan2.dat");

        num_t query_pt[3];

        const unsigned int N = PcloudS.pts.size();
	
        // buildTime : time required to build the kd-tree index
        // queryTime : time required to find nearest neighbor for a single point in the kd-tree
        vector<double> buildTime, queryTime;

        int plotCount=10;

        for(int i=1;i<=plotCount;i++)
        {
	    PointCloud<num_t> cloudS, cloudT;

            // size of dataset currently being used
            int currSize=((i*1.0)/plotCount)*N;

            cloudS.pts.resize(currSize);
            cloudT.pts.resize(currSize);
            for(int j=0;j<currSize;j++)
            {
                cloudS.pts[j]=PcloudS.pts[j];
                cloudT.pts[j]=PcloudT.pts[j];
            }

            //	 construct a kd-tree index:
            typedef KDTreeSingleIndexAdaptor<
                L2_Simple_Adaptor<num_t, PointCloud<num_t> > ,
                PointCloud<num_t>,
                3 /* dim */
                    > my_kd_tree_t;

            my_kd_tree_t   index(3 /*dim*/, cloudS, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
            
            clock_t begin = clock();
            
            index.buildIndex();
            
            clock_t end = clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            buildTime.push_back(elapsed_secs);

            {
                clock_t begin = clock();

                for(int j=0;j<currSize;j++)
                {
                    query_pt[0]=cloudT.pts[j].x;
                    query_pt[1]=cloudT.pts[j].y;
                    query_pt[2]=cloudT.pts[j].z;
                    
                    // do a knn search
                    const size_t num_results = 1;
                    size_t ret_index;
                    num_t out_dist_sqr;
                    nanoflann::KNNResultSet<num_t> resultSet(num_results);
                    resultSet.init(&ret_index, &out_dist_sqr );
                    index.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
                }

                clock_t end = clock();
                double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
                queryTime.push_back(elapsed_secs/currSize);
            }
        }

        for(int i=0;i<buildTime.size();i++)
            std::cout<<buildTime[i]<<" ";
        std::cout<<"\n";

        for(int i=0;i<queryTime.size();i++)
            std::cout<<queryTime[i]<<" ";
        std::cout<<"\n";
}

int main()
{
	srand(time(NULL));
        //randomly choose some dataset from dat_avz/001 to dat_avz/010 [fixed right now -- update later]
        string dataset_path="/home/pranjalr34/gsoc/nanoflann/benchmarkTool/realTests/dat_avz/001/";
	kdtree_demo<double>(dataset_path);
	return 0;
}
